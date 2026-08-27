#!/usr/bin/env python3
"""Create deterministic, review-only towel segmentation proposals.

The proposals are deliberately marked AMBIGUOUS and are not training labels
until a human reviews them.  Empty-table frames receive explicit EMPTY
annotations.  This tool never changes the source images.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Mapping

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_dataset import (  # noqa: E402
    build_dataset_manifest,
    load_annotation,
    validate_annotation,
)
from tools.lib.towel_perception import propose_blue_towel_mask  # noqa: E402


CATEGORY_ORDER = (
    "00_empty_table",
    "01_flat",
    "02_light_wrinkle",
    "03_heavy_wrinkle",
    "04_curled_or_overlapped",
    "05_first_fold",
    "06_second_fold",
    "07_robot_occluded",
)
EMPTY_CATEGORY = "00_empty_table"
ROBOT_OCCLUDED_CATEGORY = "07_robot_occluded"
REVIEW_CONFIRMATION = "TOWEL_LABELME_REVIEW_COMPLETE"
VALID_SPLITS = {"train", "validation", "test"}


class SegmentationBootstrapError(ValueError):
    """A source session cannot produce a safe review proposal."""


def deterministic_subset(paths: list[Path], count: int) -> list[Path]:
    if count <= 0:
        raise SegmentationBootstrapError("sample count must be positive")
    if len(paths) < count:
        raise SegmentationBootstrapError(
            f"requested {count} samples from only {len(paths)} images"
        )
    if count == 1:
        return [paths[len(paths) // 2]]
    indices = [round(index * (len(paths) - 1) / (count - 1)) for index in range(count)]
    if len(indices) != len(set(indices)):
        raise SegmentationBootstrapError("deterministic sample indices collided")
    return [paths[index] for index in indices]


def reviewed_source_paths(annotation_root: Path) -> set[str]:
    """Return source paths already covered by reviewed contract annotations."""
    paths: set[str] = set()
    for path in sorted(annotation_root.rglob("*.json")):
        document = load_annotation(path)
        if document.get("record_kind") != "towel_observation_annotation":
            continue
        source = document.get("source")
        if not isinstance(source, Mapping):
            raise SegmentationBootstrapError(
                f"reviewed annotation has no source object: {path}"
            )
        image_path = source.get("image_path")
        if not isinstance(image_path, str) or not image_path:
            raise SegmentationBootstrapError(
                f"reviewed annotation has no source image_path: {path}"
            )
        paths.add(Path(image_path).as_posix())
    return paths


def load_capture_session_index(
    session_root: Path,
) -> tuple[str, dict[str, str]]:
    """Load an optional capture manifest without inventing episode boundaries."""
    metadata_path = session_root / "session.json"
    manifest_path = session_root / "capture_manifest.jsonl"
    if not metadata_path.exists() and not manifest_path.exists():
        return "train", {}
    if not metadata_path.is_file() or not manifest_path.is_file():
        raise SegmentationBootstrapError(
            "capture session requires both session.json and capture_manifest.jsonl"
        )
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SegmentationBootstrapError(
            f"could not read capture session metadata: {exc}"
        ) from exc
    if metadata.get("record_kind") != "towel_capture_session":
        raise SegmentationBootstrapError("invalid capture session record_kind")
    if metadata.get("session_id") != session_root.name:
        raise SegmentationBootstrapError("capture session_id does not match directory")
    split = metadata.get("split")
    if split not in VALID_SPLITS:
        raise SegmentationBootstrapError("capture session has invalid split")
    image_index: dict[str, str] = {}
    capture_ids: set[str] = set()
    try:
        lines = manifest_path.read_text(encoding="utf-8").splitlines()
        records = [json.loads(line) for line in lines if line.strip()]
    except (OSError, json.JSONDecodeError) as exc:
        raise SegmentationBootstrapError(
            f"could not read capture manifest: {exc}"
        ) from exc
    if not records:
        raise SegmentationBootstrapError("capture manifest is empty")
    for record in records:
        if not isinstance(record, Mapping) or record.get(
            "record_kind"
        ) != "towel_capture_episode":
            raise SegmentationBootstrapError("invalid capture episode record")
        if record.get("session_id") != session_root.name or record.get(
            "split"
        ) != split:
            raise SegmentationBootstrapError(
                "capture episode session or split mismatch"
            )
        if split != "train" and record.get(
            "physical_reposition_confirmed"
        ) is not True:
            raise SegmentationBootstrapError(
                "held-out capture lacks physical reposition confirmation"
            )
        image_path = record.get("image_path")
        capture_id = record.get("capture_id")
        if not isinstance(image_path, str) or not image_path:
            raise SegmentationBootstrapError("capture episode image_path is required")
        relative = Path(image_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise SegmentationBootstrapError("capture episode image_path is unsafe")
        if not isinstance(capture_id, str) or not capture_id:
            raise SegmentationBootstrapError("capture episode capture_id is required")
        normalized_path = relative.as_posix()
        if normalized_path in image_index:
            raise SegmentationBootstrapError(
                f"duplicate capture image_path: {normalized_path}"
            )
        if capture_id in capture_ids:
            raise SegmentationBootstrapError(
                f"duplicate capture_id: {capture_id}"
            )
        image_index[normalized_path] = capture_id
        capture_ids.add(capture_id)
    return str(split), image_index


def propose_towel_mask(image_bgr: np.ndarray) -> np.ndarray:
    """Compatibility wrapper around the shared runtime proposal backend."""
    try:
        return propose_blue_towel_mask(image_bgr)
    except ValueError as exc:
        raise SegmentationBootstrapError(str(exc)) from exc


def mask_polygon(mask: np.ndarray) -> list[list[float]]:
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        raise SegmentationBootstrapError("proposal mask has no contour")
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) <= 1.0:
        raise SegmentationBootstrapError("proposal contour area is degenerate")
    epsilon = max(1.0, 0.0025 * cv2.arcLength(contour, True))
    simplified = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
    if len(simplified) < 3:
        raise SegmentationBootstrapError("proposal polygon has fewer than three points")
    return [[float(x), float(y)] for x, y in simplified]


def _touches_border(mask: np.ndarray) -> bool:
    return bool(
        np.any(mask[0])
        or np.any(mask[-1])
        or np.any(mask[:, 0])
        or np.any(mask[:, -1])
    )


def _annotation(
    *,
    image_path: Path,
    session_root: Path,
    category: str,
    image: np.ndarray,
    polygon: list[list[float]],
    split: str = "train",
    capture_id: str | None = None,
) -> dict[str, Any]:
    relative_path = image_path.relative_to(session_root).as_posix()
    digest = sha256(image_path.read_bytes()).hexdigest()
    is_empty = category == EMPTY_CATEGORY
    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "towel_observation_annotation",
        "observation_id": f"bootstrap-{session_root.name}-{image_path.stem}",
        "split": split,
        "state_label": "EMPTY" if is_empty else "AMBIGUOUS",
        "image_width_px": int(image.shape[1]),
        "image_height_px": int(image.shape[0]),
        "segmentation_polygon_px": [] if is_empty else polygon,
        "corners": [],
        "fold_lines_px": [],
        "height_available": False,
        "occluded": category == ROBOT_OCCLUDED_CATEGORY,
        "ambiguous_reason": (
            None if is_empty else "bootstrap_segmentation_requires_human_review"
        ),
        "source": {
            "image_path": relative_path,
            "sha256": digest,
            "capture_id": capture_id,
        },
    }
    validate_annotation(document, dataset_root=session_root)
    return document


def labelme_document(
    *,
    image_name: str,
    image_width: int,
    image_height: int,
    polygon: list[list[float]],
) -> dict[str, Any]:
    """Build a portable LabelMe document without embedding image bytes."""
    shapes = []
    if polygon:
        shapes.append(
            {
                "label": "towel",
                "points": polygon,
                "group_id": None,
                "description": "",
                "shape_type": "polygon",
                "flags": {},
                "mask": None,
            }
        )
    return {
        "version": "5.6.1",
        "flags": {},
        "shapes": shapes,
        "imagePath": image_name,
        "imageData": None,
        "imageHeight": image_height,
        "imageWidth": image_width,
    }


def reviewed_annotation_from_labelme(
    proposal: Mapping[str, Any],
    labelme: Mapping[str, Any],
    *,
    dataset_root: Path,
) -> dict[str, Any]:
    """Convert one explicitly reviewed LabelMe polygon to our contract."""
    width = proposal.get("image_width_px")
    height = proposal.get("image_height_px")
    if labelme.get("imageWidth") != width or labelme.get("imageHeight") != height:
        raise SegmentationBootstrapError(
            "LabelMe image dimensions do not match the proposal"
        )
    expected_name = Path(proposal["source"]["image_path"]).name
    if Path(str(labelme.get("imagePath", ""))).name != expected_name:
        raise SegmentationBootstrapError(
            "LabelMe imagePath does not match the proposal source"
        )
    shapes = labelme.get("shapes")
    if not isinstance(shapes, list):
        raise SegmentationBootstrapError("LabelMe shapes must be a list")
    is_empty = proposal.get("state_label") == "EMPTY"
    if is_empty:
        if shapes:
            raise SegmentationBootstrapError(
                "EMPTY review must not contain a towel polygon"
            )
        polygon: list[list[float]] = []
    else:
        towel_shapes = [
            shape
            for shape in shapes
            if isinstance(shape, Mapping) and shape.get("label") == "towel"
        ]
        if len(towel_shapes) != 1 or len(shapes) != 1:
            raise SegmentationBootstrapError(
                "non-empty review requires exactly one towel shape"
            )
        shape = towel_shapes[0]
        if shape.get("shape_type") != "polygon":
            raise SegmentationBootstrapError(
                "the towel shape must use shape_type=polygon"
            )
        raw_points = shape.get("points")
        if not isinstance(raw_points, list) or len(raw_points) < 3:
            raise SegmentationBootstrapError(
                "the reviewed towel polygon needs at least three points"
            )
        polygon = raw_points
    reviewed = deepcopy(dict(proposal))
    reviewed["segmentation_polygon_px"] = polygon
    if not is_empty:
        reviewed["ambiguous_reason"] = "human_reviewed_segmentation_only"
    validate_annotation(reviewed, dataset_root=dataset_root)
    return reviewed


def finalize_labelme_review(
    *,
    session_root: Path,
    pilot_root: Path,
    output_root: Path,
    confirmation: str | None,
) -> dict[str, Any]:
    """Import an explicitly attested pilot review without authorizing states."""
    if confirmation != REVIEW_CONFIRMATION:
        raise SegmentationBootstrapError(
            f"finalization requires --confirmation {REVIEW_CONFIRMATION}"
        )
    manifest_path = pilot_root / "pilot_manifest.json"
    if not manifest_path.is_file():
        raise SegmentationBootstrapError("pilot_manifest.json was not found")
    if (output_root / "review_manifest.json").exists():
        raise SegmentationBootstrapError(
            "output already contains review_manifest.json"
        )
    try:
        pilot = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SegmentationBootstrapError(
            f"could not read pilot manifest: {exc}"
        ) from exc
    if pilot.get("source_session") != session_root.name:
        raise SegmentationBootstrapError(
            "pilot source_session does not match session_root"
        )
    documents = []
    pending_writes = []
    reviewed_records = []
    rejected_records = []
    for record in pilot.get("records", []):
        proposal_relative = record.get("annotation")
        labelme_relative = record.get("labelme_annotation")
        if proposal_relative is None:
            continue
        if not isinstance(labelme_relative, str):
            raise SegmentationBootstrapError(
                "pilot record is missing its LabelMe review path"
            )
        proposal_path = pilot_root / proposal_relative
        labelme_path = pilot_root / labelme_relative
        proposal = load_annotation(proposal_path)
        try:
            labelme = json.loads(labelme_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SegmentationBootstrapError(
                f"could not read LabelMe review {labelme_path}: {exc}"
            ) from exc
        flags = labelme.get("flags")
        if isinstance(flags, Mapping) and flags.get("review_rejected") is True:
            rejected_records.append(
                {
                    "image": record["image"],
                    "reason": "human_review_rejected",
                }
            )
            continue
        reviewed = reviewed_annotation_from_labelme(
            proposal, labelme, dataset_root=session_root
        )
        relative_output = Path("annotations") / record["category"] / (
            Path(record["image"]).stem + ".json"
        )
        destination = output_root / relative_output
        documents.append(reviewed)
        pending_writes.append((destination, reviewed))
        reviewed_records.append(
            {
                "image": record["image"],
                "annotation": relative_output.as_posix(),
                "source_sha256": reviewed["source"]["sha256"],
            }
        )
    expected = pilot.get("annotation_proposal_count")
    if (
        not isinstance(expected, int)
        or len(documents) + len(rejected_records) != expected
    ):
        raise SegmentationBootstrapError(
            "reviewed plus rejected count does not match the pilot"
        )
    if not documents:
        raise SegmentationBootstrapError("review accepted no annotations")
    dataset_manifest = build_dataset_manifest(
        documents, dataset_root=session_root
    )
    output_root.mkdir(parents=True, exist_ok=True)
    for destination, reviewed in pending_writes:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(reviewed, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    (output_root / "dataset_manifest.json").write_text(
        json.dumps(dataset_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    review_manifest = {
        "schema_version": 1,
        "record_kind": "towel_segmentation_pilot_review",
        "source_session": session_root.name,
        "source_pilot_manifest_sha256": sha256(
            manifest_path.read_bytes()
        ).hexdigest(),
        "review_confirmation": REVIEW_CONFIRMATION,
        "reviewed_count": len(documents) + len(rejected_records),
        "accepted_annotation_count": len(documents),
        "rejected_count": len(rejected_records),
        "segmentation_labels_authorized": True,
        "state_labels_authorized": False,
        "robot_occluded_training_labels_authorized": False,
        "dataset_items_sha256": dataset_manifest["items_sha256"],
        "records": reviewed_records,
        "rejected_records": rejected_records,
    }
    (output_root / "review_manifest.json").write_text(
        json.dumps(review_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return review_manifest


def _review_overlay(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    overlay = image.copy()
    selected = mask > 0
    overlay[selected] = (
        0.45 * overlay[selected] + 0.55 * np.array([0, 255, 0])
    ).astype(np.uint8)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours:
        cv2.drawContours(
            overlay, [max(contours, key=cv2.contourArea)], -1, (0, 0, 255), 3
        )
    return overlay


def _write_review_sheets(
    output_root: Path, records: list[dict[str, Any]]
) -> list[str]:
    sheets: list[str] = []
    for category in CATEGORY_ORDER:
        category_records = [
            record for record in records if record["category"] == category
        ]
        tiles = []
        for record in category_records:
            path = output_root / record["review_overlay"]
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise SegmentationBootstrapError(
                    f"could not read review overlay {path}"
                )
            tile = cv2.resize(image, (384, 288), interpolation=cv2.INTER_AREA)
            label = Path(record["image"]).name
            cv2.rectangle(tile, (0, 0), (384, 30), (0, 0, 0), -1)
            cv2.putText(
                tile,
                label,
                (7, 21),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            tiles.append(tile)
        sheet = np.concatenate(tiles, axis=1)
        relative_path = Path(f"{category}.review_sheet.jpg")
        if not cv2.imwrite(str(output_root / relative_path), sheet):
            raise SegmentationBootstrapError(
                f"could not write review sheet for {category}"
            )
        sheets.append(relative_path.as_posix())
    return sheets


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_root", type=Path)
    parser.add_argument("--output", type=Path, default=Path("tmp/towel_segmentation_pilot"))
    parser.add_argument("--per-category", type=int, default=5)
    parser.add_argument(
        "--all-images",
        action="store_true",
        help="prepare every source frame instead of a fixed pilot subset",
    )
    parser.add_argument(
        "--exclude-reviewed",
        type=Path,
        metavar="ANNOTATION_ROOT",
        help="exclude source paths already present in reviewed annotations",
    )
    parser.add_argument(
        "--finalize-labelme",
        type=Path,
        metavar="PILOT_ROOT",
        help="import the reviewed LabelMe workspace instead of bootstrapping",
    )
    parser.add_argument("--confirmation")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session_root = args.session_root.resolve()
    output_root = args.output.resolve()
    if args.finalize_labelme is not None:
        try:
            review = finalize_labelme_review(
                session_root=session_root,
                pilot_root=args.finalize_labelme.resolve(),
                output_root=output_root,
                confirmation=args.confirmation,
            )
        except (OSError, ValueError, SegmentationBootstrapError) as exc:
            print(f"[FAIL] {exc}")
            return 1
        print(
            "TOWEL_SEGMENTATION_LABELME_REVIEW_PASS "
            f"reviewed={review['reviewed_count']} "
            f"accepted={review['accepted_annotation_count']} "
            f"rejected={review['rejected_count']} "
            "segmentation_labels_authorized=true "
            "state_labels_authorized=false "
            "robot_occluded_training_labels_authorized=false "
            f"output={output_root}"
        )
        return 0
    if args.per_category <= 0:
        print("[FAIL] --per-category must be positive")
        return 1
    if (output_root / "pilot_manifest.json").exists():
        print(
            "[FAIL] output already contains pilot_manifest.json; "
            "choose a new review directory"
        )
        return 1
    records: list[dict[str, Any]] = []
    try:
        session_split, capture_index = load_capture_session_index(session_root)
        excluded = (
            set()
            if args.exclude_reviewed is None
            else reviewed_source_paths(args.exclude_reviewed.resolve())
        )
        for category in CATEGORY_ORDER:
            category_root = session_root / category
            paths = [
                path
                for path in sorted(category_root.glob("*.jpg"))
                if path.relative_to(session_root).as_posix() not in excluded
            ]
            selected = (
                paths
                if args.all_images
                else deterministic_subset(paths, args.per_category)
            )
            for image_path in selected:
                image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                if image is None:
                    raise SegmentationBootstrapError(
                        f"could not decode {image_path}"
                    )
                if category == EMPTY_CATEGORY:
                    mask = np.zeros(image.shape[:2], dtype=np.uint8)
                    polygon: list[list[float]] = []
                else:
                    mask = propose_towel_mask(image)
                    polygon = mask_polygon(mask)
                category_output = output_root / category
                category_output.mkdir(parents=True, exist_ok=True)
                overlay_path = category_output / f"{image_path.stem}.review.jpg"
                annotation_relative: str | None = None
                labelme_annotation_relative: str | None = None
                labelme_image_relative: str | None = None
                if category != ROBOT_OCCLUDED_CATEGORY:
                    relative_image_path = image_path.relative_to(
                        session_root
                    ).as_posix()
                    if capture_index and relative_image_path not in capture_index:
                        raise SegmentationBootstrapError(
                            "source image is missing from capture manifest: "
                            f"{relative_image_path}"
                        )
                    annotation = _annotation(
                        image_path=image_path,
                        session_root=session_root,
                        category=category,
                        image=image,
                        polygon=polygon,
                        split=session_split,
                        capture_id=capture_index.get(relative_image_path),
                    )
                    annotation_path = category_output / f"{image_path.stem}.json"
                    annotation_path.write_text(
                        json.dumps(annotation, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    annotation_relative = annotation_path.relative_to(
                        output_root
                    ).as_posix()
                    labelme_output = output_root / "labelme" / category
                    labelme_output.mkdir(parents=True, exist_ok=True)
                    labelme_image_path = labelme_output / image_path.name
                    shutil.copy2(image_path, labelme_image_path)
                    labelme_path = labelme_output / f"{image_path.stem}.json"
                    labelme_path.write_text(
                        json.dumps(
                            labelme_document(
                                image_name=image_path.name,
                                image_width=int(image.shape[1]),
                                image_height=int(image.shape[0]),
                                polygon=polygon,
                            ),
                            ensure_ascii=False,
                            indent=2,
                        )
                        + "\n",
                        encoding="utf-8",
                    )
                    labelme_annotation_relative = labelme_path.relative_to(
                        output_root
                    ).as_posix()
                    labelme_image_relative = labelme_image_path.relative_to(
                        output_root
                    ).as_posix()
                if not cv2.imwrite(str(overlay_path), _review_overlay(image, mask)):
                    raise SegmentationBootstrapError(
                        f"could not write {overlay_path}"
                    )
                area_ratio = float(np.count_nonzero(mask) / mask.size)
                records.append(
                    {
                        "category": category,
                        "image": image_path.relative_to(session_root).as_posix(),
                        "annotation": annotation_relative,
                        "labelme_annotation": labelme_annotation_relative,
                        "labelme_image": labelme_image_relative,
                        "review_overlay": overlay_path.relative_to(output_root).as_posix(),
                        "area_ratio": area_ratio,
                        "polygon_points": len(polygon),
                        "touches_border": _touches_border(mask),
                        "requires_human_review": category != EMPTY_CATEGORY,
                        "robot_occluded": category == ROBOT_OCCLUDED_CATEGORY,
                        "usage": (
                            "clear_view_rejection_only"
                            if category == ROBOT_OCCLUDED_CATEGORY
                            else "review_only_annotation_proposal"
                        ),
                    }
                )
    except (OSError, ValueError, SegmentationBootstrapError) as exc:
        print(f"[FAIL] {exc}")
        return 1

    try:
        review_sheets = _write_review_sheets(output_root, records)
        manifest = {
            "schema_version": 1,
            "record_kind": "towel_segmentation_bootstrap_pilot",
        "source_session": session_root.name,
        "split": session_split,
            "selection": "all_images" if args.all_images else "pilot_subset",
            "excluded_reviewed_source_count": len(excluded),
            "selected_image_count": len(records),
            "annotation_proposal_count": sum(
                record["annotation"] is not None for record in records
            ),
            "rejection_only_count": sum(
                record["usage"] == "clear_view_rejection_only"
                for record in records
            ),
            "reviewed_count": 0,
            "training_labels_authorized": False,
            "method": "grabcut_relative_blue_largest_component",
            "review_sheets": review_sheets,
            "records": records,
        }
        output_root.mkdir(parents=True, exist_ok=True)
        (output_root / "pilot_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, SegmentationBootstrapError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    print(
        "TOWEL_SEGMENTATION_BOOTSTRAP_PILOT_PASS "
        f"selected={len(records)} "
        f"proposals={sum(record['annotation'] is not None for record in records)} "
        f"rejection_only={sum(record['usage'] == 'clear_view_rejection_only' for record in records)} "
        "reviewed=0 training_labels_authorized=false "
        f"output={output_root}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
