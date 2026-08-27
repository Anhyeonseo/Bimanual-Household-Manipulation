#!/usr/bin/env python3
"""Prepare unreviewed towel frames for human review using a YOLO-seg weight.

Predictions are annotation proposals only.  They remain unauthorized for
training until the existing explicit LabelMe finalization step is completed.
Robot-occluded frames are retained as OOD overlays and never receive training
annotations.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.run.bootstrap_towel_segmentation_pilot import (  # noqa: E402
    CATEGORY_ORDER,
    EMPTY_CATEGORY,
    ROBOT_OCCLUDED_CATEGORY,
    SegmentationBootstrapError,
    _annotation,
    _review_overlay,
    _touches_border,
    _write_review_sheets,
    labelme_document,
    load_capture_session_index,
    mask_polygon,
    propose_towel_mask,
    reviewed_source_paths,
)


DEFAULT_SESSION_ROOT = ROOT / "datasets/towel_yolo_source/20260826_top_01"
DEFAULT_REVIEW_ROOTS = (
    ROOT / "datasets/towel_yolo_annotations/20260827_pilot_reviewed",
    ROOT / "datasets/towel_yolo_annotations/20260827_review_batch2",
)
DEFAULT_MODEL = ROOT / "artifacts/models/towel_yolo26n_seg_r0/best.pt"
DEFAULT_OUTPUT = ROOT / "tmp/towel_yolo_assisted_review_r1"


def collect_unreviewed_paths(
    session_root: Path,
    review_roots: Sequence[Path],
) -> tuple[dict[str, list[Path]], set[str]]:
    """Return category paths not covered by any approved train review root."""

    if not review_roots:
        raise SegmentationBootstrapError("at least one review root is required")
    excluded: set[str] = set()
    for review_root in review_roots:
        if not review_root.is_dir():
            raise SegmentationBootstrapError(
                f"review root does not exist: {review_root}"
            )
        excluded.update(reviewed_source_paths(review_root))
    selected: dict[str, list[Path]] = {}
    for category in CATEGORY_ORDER:
        category_root = session_root / category
        if not category_root.is_dir():
            raise SegmentationBootstrapError(
                f"source category does not exist: {category_root}"
            )
        selected[category] = [
            path
            for path in sorted(category_root.glob("*.jpg"))
            if path.relative_to(session_root).as_posix() not in excluded
        ]
    return selected, excluded


def union_towel_prediction_masks(
    masks: np.ndarray,
    classes: np.ndarray,
    confidences: np.ndarray,
    *,
    width: int,
    height: int,
    mask_threshold: float,
) -> tuple[np.ndarray, int, float]:
    """Union class-zero YOLO masks at original resolution."""

    if len(masks) != len(classes) or len(masks) != len(confidences):
        raise SegmentationBootstrapError("YOLO mask, class, confidence counts differ")
    output = np.zeros((height, width), dtype=np.uint8)
    count = 0
    max_confidence = 0.0
    for prediction, class_id, confidence in zip(
        masks, classes, confidences, strict=True
    ):
        if int(class_id) != 0:
            continue
        resized = cv2.resize(
            np.asarray(prediction, dtype=np.float32),
            (width, height),
            interpolation=cv2.INTER_LINEAR,
        )
        output |= (resized >= mask_threshold).astype(np.uint8) * 255
        count += 1
        max_confidence = max(max_confidence, float(confidence))
    return output, count, max_confidence


def result_towel_mask(
    result: Any,
    *,
    width: int,
    height: int,
    mask_threshold: float,
) -> tuple[np.ndarray, int, float]:
    """Convert one Ultralytics result without leaking its types into tests."""

    if result.masks is None or result.boxes is None:
        return np.zeros((height, width), dtype=np.uint8), 0, 0.0
    return union_towel_prediction_masks(
        result.masks.data.cpu().numpy(),
        result.boxes.cls.cpu().numpy(),
        result.boxes.conf.cpu().numpy(),
        width=width,
        height=height,
        mask_threshold=mask_threshold,
    )


def review_priority(
    *,
    fallback_used: bool,
    prediction_count: int,
    max_confidence: float,
    touches_border: bool,
    area_ratio: float,
) -> str:
    """Prioritize cases most likely to need polygon correction."""

    if (
        fallback_used
        or prediction_count != 1
        or max_confidence < 0.75
        or touches_border
        or area_ratio < 0.01
        or area_ratio > 0.85
    ):
        return "high"
    if max_confidence < 0.90:
        return "medium"
    return "low"


def _review_root_identity(review_root: Path) -> dict[str, Any]:
    manifest_path = review_root / "review_manifest.json"
    if not manifest_path.is_file():
        raise SegmentationBootstrapError(
            f"review manifest does not exist: {manifest_path}"
        )
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (
        document.get("record_kind") != "towel_segmentation_pilot_review"
        or document.get("segmentation_labels_authorized") is not True
        or document.get("robot_occluded_training_labels_authorized") is not False
    ):
        raise SegmentationBootstrapError(
            f"review root is not an approved segmentation source: {review_root}"
        )
    return {
        "review_root": review_root.name,
        "review_manifest_sha256": sha256(manifest_path.read_bytes()).hexdigest(),
        "accepted_annotation_count": document.get("accepted_annotation_count"),
    }


def _predict_results(
    model: Any,
    paths: Sequence[Path],
    *,
    imgsz: int,
    confidence: float,
    device: str,
) -> dict[Path, Any]:
    if not paths:
        return {}
    results = model.predict(
        [str(path) for path in paths],
        imgsz=imgsz,
        conf=confidence,
        device=device,
        retina_masks=True,
        verbose=False,
        stream=False,
    )
    if len(results) != len(paths):
        raise SegmentationBootstrapError(
            "YOLO prediction count does not match source image count"
        )
    return {path: result for path, result in zip(paths, results, strict=True)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", type=Path, default=DEFAULT_SESSION_ROOT)
    parser.add_argument(
        "--review-root",
        action="append",
        type=Path,
        dest="review_roots",
        help="approved train review root; repeat for multiple batches",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--mask-threshold", type=float, default=0.5)
    args = parser.parse_args()

    if not 0.0 < args.confidence < 1.0:
        parser.error("--confidence must be within 0..1")
    if not 0.0 < args.mask_threshold < 1.0:
        parser.error("--mask-threshold must be within 0..1")
    if args.imgsz <= 0:
        parser.error("--imgsz must be positive")

    session_root = args.session_root.resolve()
    review_roots = tuple(
        path.resolve()
        for path in (args.review_roots or list(DEFAULT_REVIEW_ROOTS))
    )
    model_path = args.model.resolve()
    output_root = args.output.resolve()
    if not session_root.is_dir():
        parser.error(f"source session does not exist: {session_root}")
    if not model_path.is_file():
        parser.error(f"model does not exist: {model_path}")
    if output_root.exists() and any(output_root.iterdir()):
        parser.error(f"output directory is not empty: {output_root}")

    try:
        import torch
        import ultralytics
        from ultralytics import YOLO

        selected, excluded = collect_unreviewed_paths(session_root, review_roots)
        session_split, capture_index = load_capture_session_index(session_root)
        if session_split != "train":
            raise SegmentationBootstrapError(
                "assisted expansion is restricted to the train session"
            )
        review_identities = [
            _review_root_identity(review_root) for review_root in review_roots
        ]
        prediction_paths = [
            path
            for category in CATEGORY_ORDER
            if category != EMPTY_CATEGORY
            for path in selected[category]
        ]
        predictions = _predict_results(
            YOLO(str(model_path)),
            prediction_paths,
            imgsz=args.imgsz,
            confidence=args.confidence,
            device=args.device,
        )

        records: list[dict[str, Any]] = []
        fallback_count = 0
        for category in CATEGORY_ORDER:
            for image_path in selected[category]:
                image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                if image is None:
                    raise SegmentationBootstrapError(
                        f"could not decode source image: {image_path}"
                    )
                height, width = image.shape[:2]
                fallback_used = False
                if category == EMPTY_CATEGORY:
                    mask = np.zeros((height, width), dtype=np.uint8)
                    prediction_count = 0
                    max_confidence = 1.0
                    proposal_method = "explicit_empty"
                else:
                    mask, prediction_count, max_confidence = result_towel_mask(
                        predictions[image_path],
                        width=width,
                        height=height,
                        mask_threshold=args.mask_threshold,
                    )
                    proposal_method = "yolo26n_seg"
                    if (
                        prediction_count == 0
                        and category != ROBOT_OCCLUDED_CATEGORY
                    ):
                        mask = propose_towel_mask(image)
                        fallback_used = True
                        fallback_count += 1
                        proposal_method = "opencv_fallback_after_yolo_miss"

                polygon = mask_polygon(mask) if np.any(mask) else []
                category_output = output_root / category
                category_output.mkdir(parents=True, exist_ok=True)
                overlay_path = category_output / f"{image_path.stem}.review.jpg"
                if not cv2.imwrite(str(overlay_path), _review_overlay(image, mask)):
                    raise SegmentationBootstrapError(
                        f"could not write review overlay: {overlay_path}"
                    )

                annotation_relative: str | None = None
                labelme_annotation_relative: str | None = None
                labelme_image_relative: str | None = None
                if category != ROBOT_OCCLUDED_CATEGORY:
                    relative_image = image_path.relative_to(session_root).as_posix()
                    if capture_index and relative_image not in capture_index:
                        raise SegmentationBootstrapError(
                            f"source image is missing from capture manifest: {relative_image}"
                        )
                    annotation = _annotation(
                        image_path=image_path,
                        session_root=session_root,
                        category=category,
                        image=image,
                        polygon=polygon,
                        split=session_split,
                        capture_id=capture_index.get(relative_image),
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
                                image_width=width,
                                image_height=height,
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

                area_ratio = float(np.count_nonzero(mask) / mask.size)
                touches_border = _touches_border(mask)
                is_ood = category == ROBOT_OCCLUDED_CATEGORY
                priority = (
                    "ood"
                    if is_ood
                    else "low"
                    if category == EMPTY_CATEGORY
                    else review_priority(
                        fallback_used=fallback_used,
                        prediction_count=prediction_count,
                        max_confidence=max_confidence,
                        touches_border=touches_border,
                        area_ratio=area_ratio,
                    )
                )
                records.append(
                    {
                        "category": category,
                        "image": image_path.relative_to(session_root).as_posix(),
                        "annotation": annotation_relative,
                        "labelme_annotation": labelme_annotation_relative,
                        "labelme_image": labelme_image_relative,
                        "review_overlay": overlay_path.relative_to(
                            output_root
                        ).as_posix(),
                        "proposal_method": proposal_method,
                        "prediction_count": prediction_count,
                        "max_confidence": round(max_confidence, 9),
                        "area_ratio": round(area_ratio, 9),
                        "polygon_points": len(polygon),
                        "touches_border": touches_border,
                        "fallback_used": fallback_used,
                        "review_priority": priority,
                        "requires_human_review": category != EMPTY_CATEGORY,
                        "robot_occluded": is_ood,
                        "usage": (
                            "clear_view_rejection_only"
                            if is_ood
                            else "review_only_annotation_proposal"
                        ),
                    }
                )

        review_sheets = _write_review_sheets(output_root, records)
        priority_rank = {"high": 0, "medium": 1, "low": 2}
        review_queue = sorted(
            (record for record in records if not record["robot_occluded"]),
            key=lambda record: (
                priority_rank[record["review_priority"]],
                record["max_confidence"],
                record["category"],
                record["image"],
            ),
        )
        (output_root / "review_queue.json").write_text(
            json.dumps(review_queue, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        priority_counts = {
            priority: sum(
                record["review_priority"] == priority for record in review_queue
            )
            for priority in ("high", "medium", "low")
        }
        instructions_path = output_root / "REVIEW_INSTRUCTIONS.md"
        instructions_path.write_text(
            "# YOLO-assisted towel mask review\n\n"
            "These polygons are proposals, not approved training labels.\n\n"
            "1. Review `high`, then `medium`, then `low` entries from "
            "`review_queue.json`.\n"
            "2. Open the matching image/JSON pair below `labelme/<category>/`.\n"
            "3. Keep exactly one `towel` polygon for usable non-empty frames; "
            "empty frames keep zero shapes.\n"
            "4. Set `flags.review_rejected=true` for unusable or human-occluded "
            "frames.\n"
            "5. Only after every proposal is reviewed, run:\n\n"
            "```bash\n"
            "python3 tools/run/bootstrap_towel_segmentation_pilot.py \\\n"
            "  datasets/towel_yolo_source/20260826_top_01 \\\n"
            "  --finalize-labelme tmp/towel_yolo_assisted_review_r1 \\\n"
            "  --output datasets/towel_yolo_annotations/"
            "20260827_yolo_assisted_reviewed \\\n"
            "  --confirmation TOWEL_LABELME_REVIEW_COMPLETE\n"
            "```\n",
            encoding="utf-8",
        )
        manifest = {
            "schema_version": 1,
            "record_kind": "towel_segmentation_bootstrap_pilot",
            "source_session": session_root.name,
            "split": session_split,
            "selection": "all_unreviewed_train_images",
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
            "state_labels_authorized": False,
            "robot_occluded_training_labels_authorized": False,
            "method": "yolo26n_seg_assisted_human_review",
            "model": {
                "name": model_path.name,
                "sha256": sha256(model_path.read_bytes()).hexdigest(),
            },
            "configuration": {
                "device": str(args.device),
                "imgsz": args.imgsz,
                "confidence": args.confidence,
                "mask_threshold": args.mask_threshold,
                "torch_version": torch.__version__,
                "torch_cuda_runtime": torch.version.cuda,
                "ultralytics_version": ultralytics.__version__,
            },
            "review_sources": review_identities,
            "fallback_count": fallback_count,
            "review_priority_counts": priority_counts,
            "review_queue": "review_queue.json",
            "review_instructions": instructions_path.name,
            "review_sheets": review_sheets,
            "records": records,
        }
        output_root.mkdir(parents=True, exist_ok=True)
        (output_root / "pilot_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, SegmentationBootstrapError) as exc:
        print(f"TOWEL_YOLO_ASSISTED_REVIEW_FAIL {exc}")
        return 1

    print(
        "TOWEL_YOLO_ASSISTED_REVIEW_PASS "
        f"selected={manifest['selected_image_count']} "
        f"proposals={manifest['annotation_proposal_count']} "
        f"ood={manifest['rejection_only_count']} "
        f"fallback={manifest['fallback_count']} "
        f"high={priority_counts['high']} "
        f"medium={priority_counts['medium']} "
        f"low={priority_counts['low']} "
        "reviewed=0 training_labels_authorized=false "
        f"output={output_root}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
