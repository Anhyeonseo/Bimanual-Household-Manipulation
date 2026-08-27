"""Export reviewed towel polygons to a deterministic YOLO-seg dataset."""

from __future__ import annotations

from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
import shutil
from typing import Any, Iterable, Mapping, Sequence

import cv2
import numpy as np
import yaml

from tools.lib.towel_dataset import (
    TowelDatasetError,
    build_dataset_manifest,
    load_annotation,
    validate_annotation,
)
from tools.lib.towel_perception import rasterize_annotation_mask


YOLO_CLASS_ID = 0
YOLO_CLASS_NAME = "towel"
YOLO_SPLIT_DIRECTORY = {
    "train": "train",
    "validation": "val",
    "test": "test",
}
MINIMUM_ROUNDTRIP_IOU = 0.999


class TowelYoloExportError(ValueError):
    """Reviewed annotations cannot be exported without weakening the contract."""


def _sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TowelYoloExportError(f"could not read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise TowelYoloExportError(f"JSON root must be an object: {path}")
    return value


def _safe_relative_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise TowelYoloExportError(f"{label} must be a nonempty relative path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path == Path("."):
        raise TowelYoloExportError(f"{label} must be a safe relative path")
    return path


def _resolve_inside(root: Path, relative: Path, label: str) -> Path:
    resolved_root = root.resolve()
    resolved = (resolved_root / relative).resolve()
    if resolved_root not in resolved.parents:
        raise TowelYoloExportError(f"{label} escapes its root")
    return resolved


def _canonical_digest(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256(encoded).hexdigest()


def annotation_to_yolo_line(annotation: Mapping[str, Any]) -> str:
    """Return one normalized YOLO segmentation row, or empty text for EMPTY."""

    validate_annotation(annotation)
    polygon = annotation["segmentation_polygon_px"]
    if not polygon:
        return ""
    width = float(annotation["image_width_px"])
    height = float(annotation["image_height_px"])
    coordinates: list[str] = []
    for point in polygon:
        normalized_x = float(point[0]) / width
        normalized_y = float(point[1]) / height
        if not (
            math.isfinite(normalized_x)
            and math.isfinite(normalized_y)
            and 0.0 <= normalized_x <= 1.0
            and 0.0 <= normalized_y <= 1.0
        ):
            raise TowelYoloExportError("normalized polygon is outside 0..1")
        coordinates.extend((f"{normalized_x:.10f}", f"{normalized_y:.10f}"))
    return " ".join((str(YOLO_CLASS_ID), *coordinates))


def parse_yolo_segmentation_line(
    text: str,
    *,
    width: int,
    height: int,
) -> list[list[float]]:
    """Parse the single-class row emitted by :func:`annotation_to_yolo_line`."""

    stripped = text.strip()
    if not stripped:
        return []
    fields = stripped.split()
    if fields[0] != str(YOLO_CLASS_ID):
        raise TowelYoloExportError("YOLO row has an unexpected class id")
    values = fields[1:]
    if len(values) < 6 or len(values) % 2 != 0:
        raise TowelYoloExportError("YOLO polygon requires at least three XY pairs")
    try:
        normalized = [float(value) for value in values]
    except ValueError as exc:
        raise TowelYoloExportError("YOLO polygon coordinates must be numbers") from exc
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in normalized):
        raise TowelYoloExportError("YOLO polygon coordinates must be within 0..1")
    return [
        [normalized[index] * width, normalized[index + 1] * height]
        for index in range(0, len(normalized), 2)
    ]


def polygon_roundtrip_iou(
    annotation: Mapping[str, Any],
    yolo_line: str,
) -> float:
    """Rasterize an exported row and compare it with its reviewed polygon."""

    expected = rasterize_annotation_mask(annotation) > 0
    height = int(annotation["image_height_px"])
    width = int(annotation["image_width_px"])
    polygon = parse_yolo_segmentation_line(yolo_line, width=width, height=height)
    actual = np.zeros((height, width), dtype=np.uint8)
    if polygon:
        points = np.rint(np.asarray(polygon, dtype=float)).astype(np.int32)
        points[:, 0] = np.clip(points[:, 0], 0, width - 1)
        points[:, 1] = np.clip(points[:, 1], 0, height - 1)
        cv2.fillPoly(actual, [points], 1)
    actual_binary = actual > 0
    union = int(np.count_nonzero(expected | actual_binary))
    if union == 0:
        return 1.0
    intersection = int(np.count_nonzero(expected & actual_binary))
    return intersection / union


def _review_records(
    review_root: Path,
    source_root: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    manifest_path = review_root / "review_manifest.json"
    manifest = _load_json(manifest_path)
    if (
        manifest.get("record_kind") != "towel_segmentation_pilot_review"
        or manifest.get("segmentation_labels_authorized") is not True
        or manifest.get("robot_occluded_training_labels_authorized") is not False
    ):
        raise TowelYoloExportError(
            f"review manifest is not authorized for segmentation training: {manifest_path}"
        )
    records = manifest.get("records")
    if not isinstance(records, list) or not records:
        raise TowelYoloExportError(f"review manifest has no accepted records: {manifest_path}")
    if manifest.get("accepted_annotation_count") != len(records):
        raise TowelYoloExportError(
            f"accepted record count does not match review manifest: {manifest_path}"
        )
    source_session = _safe_relative_path(
        manifest.get("source_session"), "review source_session"
    )
    session_root = _resolve_inside(source_root, source_session, "source session")
    if not session_root.is_dir():
        raise TowelYoloExportError(f"source session does not exist: {session_root}")

    accepted: list[dict[str, Any]] = []
    annotations: list[dict[str, Any]] = []
    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise TowelYoloExportError(f"review record {index} must be an object")
        annotation_relative = _safe_relative_path(
            record.get("annotation"), f"review record {index} annotation"
        )
        image_relative = _safe_relative_path(
            record.get("image"), f"review record {index} image"
        )
        annotation_path = _resolve_inside(
            review_root, annotation_relative, "review annotation"
        )
        image_path = _resolve_inside(session_root, image_relative, "source image")
        if not annotation_path.is_file():
            raise TowelYoloExportError(f"review annotation does not exist: {annotation_path}")
        if not image_path.is_file():
            raise TowelYoloExportError(f"review source image does not exist: {image_path}")
        annotation = load_annotation(annotation_path)
        try:
            normalized = validate_annotation(annotation, dataset_root=session_root)
        except TowelDatasetError as exc:
            raise TowelYoloExportError(str(exc)) from exc
        if normalized["source_path"] != image_relative.as_posix():
            raise TowelYoloExportError(
                f"review image and annotation source differ: {annotation_path}"
            )
        record_digest = record.get("source_sha256")
        if record_digest != normalized["source_sha256"]:
            raise TowelYoloExportError(
                f"review and annotation SHA differ: {annotation_path}"
            )
        annotations.append(annotation)
        accepted.append(
            {
                "annotation": annotation,
                "annotation_path": annotation_path,
                "image_path": image_path,
                "source_session": source_session.as_posix(),
            }
        )

    generated_manifest = build_dataset_manifest(annotations)
    if manifest.get("dataset_items_sha256") != generated_manifest["items_sha256"]:
        raise TowelYoloExportError(
            f"review dataset digest does not match accepted annotations: {manifest_path}"
        )
    return accepted, {
        "review_root": review_root.name,
        "manifest": manifest_path.name,
        "sha256": _sha256_file(manifest_path),
        "source_session": source_session.as_posix(),
        "dataset_items_sha256": generated_manifest["items_sha256"],
        "accepted_annotation_count": len(accepted),
    }


def _ensure_empty_output(output_root: Path) -> None:
    if output_root.exists() and any(output_root.iterdir()):
        raise TowelYoloExportError(
            f"output directory is not empty: {output_root}"
        )
    output_root.mkdir(parents=True, exist_ok=True)


def _dataset_yaml(split_counts: Mapping[str, int]) -> dict[str, Any]:
    value: dict[str, Any] = {
        "train": "images/train",
        "val": "images/val",
    }
    if split_counts.get("test", 0):
        value["test"] = "images/test"
    value["names"] = {YOLO_CLASS_ID: YOLO_CLASS_NAME}
    return value


def export_reviewed_yolo_dataset(
    review_roots: Sequence[Path],
    *,
    source_root: Path,
    output_root: Path,
) -> dict[str, Any]:
    """Create a portable one-class YOLO-seg dataset from approved reviews."""

    if not review_roots:
        raise TowelYoloExportError("at least one review root is required")
    _ensure_empty_output(output_root)
    accepted: list[dict[str, Any]] = []
    review_sources: list[dict[str, Any]] = []
    for review_root in review_roots:
        if not review_root.is_dir():
            raise TowelYoloExportError(f"review root does not exist: {review_root}")
        records, review_source = _review_records(review_root, source_root)
        accepted.extend(records)
        review_sources.append(review_source)

    annotations = [record["annotation"] for record in accepted]
    try:
        combined_manifest = build_dataset_manifest(annotations)
    except TowelDatasetError as exc:
        raise TowelYoloExportError(str(exc)) from exc
    source_digests = [item["source_sha256"] for item in combined_manifest["items"]]
    if len(source_digests) != len(set(source_digests)):
        raise TowelYoloExportError("a source image appears more than once in the export")

    for directory in YOLO_SPLIT_DIRECTORY.values():
        (output_root / "images" / directory).mkdir(parents=True, exist_ok=True)
        (output_root / "labels" / directory).mkdir(parents=True, exist_ok=True)

    items: list[dict[str, Any]] = []
    split_counts: Counter[str] = Counter()
    empty_counts: Counter[str] = Counter()
    for record in sorted(
        accepted,
        key=lambda value: value["annotation"]["observation_id"],
    ):
        annotation = record["annotation"]
        split = str(annotation["split"])
        split_directory = YOLO_SPLIT_DIRECTORY[split]
        source_digest = str(annotation["source"]["sha256"])
        image_path = record["image_path"]
        extension = image_path.suffix.lower()
        if extension not in {".jpg", ".jpeg", ".png"}:
            raise TowelYoloExportError(f"unsupported image extension: {image_path}")
        stem = source_digest
        image_relative = Path("images") / split_directory / f"{stem}{extension}"
        label_relative = Path("labels") / split_directory / f"{stem}.txt"
        output_image = output_root / image_relative
        output_label = output_root / label_relative
        shutil.copyfile(image_path, output_image)
        yolo_line = annotation_to_yolo_line(annotation)
        output_label.write_text(
            f"{yolo_line}\n" if yolo_line else "",
            encoding="utf-8",
        )
        roundtrip_iou = polygon_roundtrip_iou(annotation, yolo_line)
        if roundtrip_iou < MINIMUM_ROUNDTRIP_IOU:
            raise TowelYoloExportError(
                f"YOLO polygon round-trip IoU {roundtrip_iou:.6f} is below "
                f"{MINIMUM_ROUNDTRIP_IOU:.6f}: {record['annotation_path']}"
            )
        split_counts[split] += 1
        is_empty = not bool(annotation["segmentation_polygon_px"])
        if is_empty:
            empty_counts[split] += 1
        items.append(
            {
                "observation_id": annotation["observation_id"],
                "split": split,
                "state_label": annotation["state_label"],
                "source_session": record["source_session"],
                "source_sha256": source_digest,
                "image": image_relative.as_posix(),
                "label": label_relative.as_posix(),
                "label_sha256": _sha256_file(output_label),
                "empty_negative": is_empty,
                "polygon_point_count": len(annotation["segmentation_polygon_px"]),
                "roundtrip_iou": round(roundtrip_iou, 9),
            }
        )

    counts = {
        split: split_counts.get(split, 0)
        for split in ("train", "validation", "test")
    }
    yaml_document = _dataset_yaml(counts)
    dataset_yaml_path = output_root / "dataset.yaml"
    dataset_yaml_path.write_text(
        yaml.safe_dump(yaml_document, sort_keys=False),
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "record_kind": "towel_yolo_segmentation_export",
        "class_names": {str(YOLO_CLASS_ID): YOLO_CLASS_NAME},
        "split_policy": "preserve_reviewed_annotation_split",
        "unreviewed_labels_authorized": False,
        "robot_occluded_training_labels_authorized": False,
        "review_sources": sorted(
            review_sources,
            key=lambda value: (value["source_session"], value["review_root"]),
        ),
        "source_dataset_items_sha256": combined_manifest["items_sha256"],
        "dataset_yaml_sha256": _sha256_file(dataset_yaml_path),
        "item_count": len(items),
        "split_counts": counts,
        "empty_negative_counts": {
            split: empty_counts.get(split, 0)
            for split in ("train", "validation", "test")
        },
        "minimum_roundtrip_iou": min(item["roundtrip_iou"] for item in items),
        "items_sha256": _canonical_digest(items),
        "items": items,
    }
    manifest_path = output_root / "export_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def collect_review_roots(values: Iterable[Path]) -> tuple[Path, ...]:
    """Normalize and reject duplicate review roots without reordering them."""

    roots = tuple(path.resolve() for path in values)
    if len(roots) != len(set(roots)):
        raise TowelYoloExportError("review roots must be unique")
    return roots
