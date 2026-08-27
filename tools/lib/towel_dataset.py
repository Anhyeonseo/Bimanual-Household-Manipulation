"""Validate towel annotations and build deterministic dataset manifests."""

from __future__ import annotations

from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from tools.lib.towel_geometry import TowelGeometryError, polygon_area

VALID_SPLITS = {"train", "validation", "test"}
VALID_STATES = {
    "EMPTY",
    "CRUMPLED",
    "PARTIALLY_OPEN",
    "TWO_CORNERS_VISIBLE",
    "FOUR_CORNERS_VISIBLE",
    "FLAT_BUT_ROTATED",
    "ALIGNED",
    "FOLD_1_COMPLETE",
    "FOLD_2_COMPLETE",
    "AMBIGUOUS",
}
CORNER_LABELS = {"top_left", "top_right", "bottom_right", "bottom_left"}
ROOT_FIELDS = {
    "schema_version", "record_kind", "observation_id", "split", "state_label",
    "image_width_px", "image_height_px", "segmentation_polygon_px", "corners",
    "fold_lines_px", "height_available", "occluded", "ambiguous_reason", "source",
}
CORNER_FIELDS = {"label", "point_px", "visible", "graspable", "confidence"}
SOURCE_FIELDS = {"image_path", "sha256", "capture_id"}


class TowelDatasetError(ValueError):
    """An annotation collection violates the dataset contract."""


def _sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _digest(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise TowelDatasetError(f"{label} must be a string")
    digest = value
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise TowelDatasetError(f"{label} must be lowercase SHA-256")
    return digest


def _point(
    value: Any,
    width: int,
    height: int,
    label: str,
    *,
    allow_image_edge: bool = False,
) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise TowelDatasetError(f"{label} must contain x and y")
    if any(
        isinstance(coordinate, bool) or not isinstance(coordinate, (int, float))
        for coordinate in value
    ):
        raise TowelDatasetError(f"{label} coordinates must be numbers")
    point = float(value[0]), float(value[1])
    if not all(math.isfinite(coordinate) for coordinate in point):
        raise TowelDatasetError(f"{label} coordinates must be finite")
    maximum_x_ok = point[0] <= width if allow_image_edge else point[0] < width
    maximum_y_ok = point[1] <= height if allow_image_edge else point[1] < height
    if not 0.0 <= point[0] or not maximum_x_ok or not 0.0 <= point[1] or not maximum_y_ok:
        raise TowelDatasetError(f"{label} is outside the image")
    return point


def validate_annotation(
    document: Mapping[str, Any],
    *,
    dataset_root: Path | None = None,
) -> dict[str, Any]:
    unknown_fields = set(document) - ROOT_FIELDS
    if unknown_fields:
        raise TowelDatasetError(
            f"annotation contains unknown fields: {sorted(unknown_fields)}"
        )
    if document.get("schema_version") != 1:
        raise TowelDatasetError("annotation schema_version must be 1")
    if document.get("record_kind") != "towel_observation_annotation":
        raise TowelDatasetError(
            "record_kind must be towel_observation_annotation"
        )
    observation_id = document.get("observation_id")
    if not isinstance(observation_id, str) or not observation_id:
        raise TowelDatasetError("observation_id is required")
    split = document.get("split")
    if split not in VALID_SPLITS:
        raise TowelDatasetError(f"invalid split: {split}")
    state = document.get("state_label")
    if state not in VALID_STATES:
        raise TowelDatasetError(f"invalid state_label: {state}")
    width = document.get("image_width_px")
    height = document.get("image_height_px")
    if (
        not isinstance(width, int) or isinstance(width, bool) or width <= 0
        or not isinstance(height, int) or isinstance(height, bool) or height <= 0
    ):
        raise TowelDatasetError("image dimensions must be positive integers")
    polygon = document.get("segmentation_polygon_px")
    if not isinstance(polygon, list):
        raise TowelDatasetError("segmentation_polygon_px must be a list")
    if state == "EMPTY" and polygon:
        raise TowelDatasetError(
            "EMPTY annotations require an empty segmentation polygon"
        )
    if state != "EMPTY" and len(polygon) < 3:
        raise TowelDatasetError(
            "non-empty segmentation_polygon_px requires at least three points"
        )
    normalized_polygon = [
        _point(
            value,
            width,
            height,
            f"segmentation_polygon_px[{index}]",
            allow_image_edge=True,
        )
        for index, value in enumerate(polygon)
    ]
    if len(set(normalized_polygon)) != len(normalized_polygon):
        raise TowelDatasetError("segmentation polygon points must be unique")
    if normalized_polygon:
        try:
            if polygon_area(normalized_polygon) <= 1.0e-12:
                raise TowelDatasetError(
                    "segmentation polygon area must be positive"
                )
        except TowelGeometryError as exc:
            raise TowelDatasetError(
                f"invalid segmentation polygon: {exc}"
            ) from exc
    corners = document.get("corners")
    if not isinstance(corners, list) or len(corners) > 4:
        raise TowelDatasetError("corners must contain at most four entries")
    labels = []
    normalized_corners = []
    for index, corner in enumerate(corners):
        if not isinstance(corner, Mapping):
            raise TowelDatasetError(f"corners[{index}] must be an object")
        required_corner_keys = {
            "label", "point_px", "visible", "graspable", "confidence"
        }
        if not required_corner_keys.issubset(corner):
            raise TowelDatasetError(
                f"corners[{index}] is missing required fields"
            )
        if set(corner) - CORNER_FIELDS:
            raise TowelDatasetError(f"corners[{index}] contains unknown fields")
        label = corner.get("label")
        if label not in CORNER_LABELS:
            raise TowelDatasetError(f"corners[{index}] has invalid label")
        labels.append(label)
        raw_confidence = corner.get("confidence")
        if isinstance(raw_confidence, bool) or not isinstance(
            raw_confidence, (int, float)
        ):
            raise TowelDatasetError(
                f"corners[{index}].confidence must be numeric"
            )
        confidence = float(raw_confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise TowelDatasetError(
                f"corners[{index}].confidence must be within 0..1"
            )
        if not isinstance(corner["visible"], bool) or not isinstance(
            corner["graspable"], bool
        ):
            raise TowelDatasetError(
                f"corners[{index}] visibility flags must be boolean"
            )
        normalized_corners.append({
            "label": label,
            "point_px": _point(
                corner.get("point_px"), width, height,
                f"corners[{index}].point_px",
            ),
            "visible": corner["visible"],
            "graspable": corner["graspable"],
            "confidence": confidence,
        })
    if len(labels) != len(set(labels)):
        raise TowelDatasetError("corner labels must be unique")
    fold_lines = document.get("fold_lines_px", [])
    if not isinstance(fold_lines, list) or len(fold_lines) > 2:
        raise TowelDatasetError("fold_lines_px must contain at most two lines")
    for line_index, line in enumerate(fold_lines):
        if not isinstance(line, list) or len(line) != 2:
            raise TowelDatasetError(
                f"fold_lines_px[{line_index}] must contain two points"
            )
        for point_index, value in enumerate(line):
            _point(
                value, width, height,
                f"fold_lines_px[{line_index}][{point_index}]",
            )
        if tuple(line[0]) == tuple(line[1]):
            raise TowelDatasetError(
                f"fold_lines_px[{line_index}] endpoints must differ"
            )
    if state == "FOLD_1_COMPLETE" and len(fold_lines) != 1:
        raise TowelDatasetError("FOLD_1_COMPLETE requires one fold line")
    if state == "FOLD_2_COMPLETE" and len(fold_lines) != 2:
        raise TowelDatasetError("FOLD_2_COMPLETE requires two fold lines")
    if state == "EMPTY" and (corners or fold_lines):
        raise TowelDatasetError(
            "EMPTY annotations cannot contain corners or fold lines"
        )
    if state == "AMBIGUOUS" and not document.get("ambiguous_reason"):
        raise TowelDatasetError(
            "AMBIGUOUS annotations require ambiguous_reason"
        )
    if document.get("ambiguous_reason") is not None and not isinstance(
        document.get("ambiguous_reason"), str
    ):
        raise TowelDatasetError("ambiguous_reason must be a string or null")
    for flag_name in ("height_available", "occluded"):
        if flag_name in document and not isinstance(document[flag_name], bool):
            raise TowelDatasetError(f"{flag_name} must be boolean")
    source = document.get("source")
    if not isinstance(source, Mapping):
        raise TowelDatasetError("source must be an object")
    if set(source) - SOURCE_FIELDS:
        raise TowelDatasetError("source contains unknown fields")
    raw_image_path = source.get("image_path")
    if not isinstance(raw_image_path, str) or not raw_image_path.strip():
        raise TowelDatasetError("source.image_path is required")
    image_path = Path(raw_image_path)
    if image_path.is_absolute() or ".." in image_path.parts or image_path == Path("."):
        raise TowelDatasetError(
            "source.image_path must be a safe relative path"
        )
    source_digest = _digest(source.get("sha256"), "source.sha256")
    capture_id = source.get("capture_id")
    if capture_id is not None and (
        not isinstance(capture_id, str) or not capture_id.strip()
    ):
        raise TowelDatasetError(
            "source.capture_id must be a nonempty string or null"
        )
    if dataset_root is not None:
        resolved_root = dataset_root.resolve()
        resolved_source = (resolved_root / image_path).resolve()
        if resolved_root not in resolved_source.parents:
            raise TowelDatasetError("source image escapes dataset root")
        if not resolved_source.is_file():
            raise TowelDatasetError(f"source image does not exist: {image_path}")
        if _sha256_file(resolved_source) != source_digest:
            raise TowelDatasetError(
                f"source SHA mismatch: {image_path}"
            )
    return {
        "observation_id": observation_id,
        "split": split,
        "state_label": state,
        "source_sha256": source_digest,
        "source_path": image_path.as_posix(),
        "capture_id": capture_id,
        "polygon_point_count": len(normalized_polygon),
        "corner_count": len(normalized_corners),
    }


def build_dataset_manifest(
    annotations: Iterable[Mapping[str, Any]],
    *,
    dataset_root: Path | None = None,
) -> dict[str, Any]:
    normalized = [
        validate_annotation(document, dataset_root=dataset_root)
        for document in annotations
    ]
    ids = [item["observation_id"] for item in normalized]
    if len(ids) != len(set(ids)):
        raise TowelDatasetError("observation_id values must be unique")
    digest_splits: dict[str, set[str]] = {}
    for item in normalized:
        digest_splits.setdefault(item["source_sha256"], set()).add(item["split"])
    leaked = sorted(
        digest for digest, splits in digest_splits.items() if len(splits) > 1
    )
    if leaked:
        raise TowelDatasetError(
            f"source SHA appears in multiple splits: {leaked[0]}"
        )
    capture_splits: dict[str, set[str]] = {}
    for item in normalized:
        capture_id = item["capture_id"]
        if capture_id is not None:
            capture_splits.setdefault(capture_id, set()).add(item["split"])
    leaked_captures = sorted(
        capture_id
        for capture_id, splits in capture_splits.items()
        if len(splits) > 1
    )
    if leaked_captures:
        raise TowelDatasetError(
            "capture_id appears in multiple splits: "
            f"{leaked_captures[0]}"
        )
    items = sorted(normalized, key=lambda item: item["observation_id"])
    split_counts = Counter(item["split"] for item in items)
    state_counts = Counter(item["state_label"] for item in items)
    canonical_items = json.dumps(
        items, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return {
        "schema_version": 1,
        "record_kind": "towel_dataset_manifest",
        "annotation_count": len(items),
        "split_counts": {
            split: split_counts.get(split, 0)
            for split in ("train", "validation", "test")
        },
        "state_counts": {
            state: state_counts.get(state, 0)
            for state in sorted(VALID_STATES)
        },
        "items_sha256": sha256(canonical_items).hexdigest(),
        "items": items,
    }


def load_annotation(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TowelDatasetError(f"could not read annotation {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise TowelDatasetError(f"annotation root must be an object: {path}")
    return document
