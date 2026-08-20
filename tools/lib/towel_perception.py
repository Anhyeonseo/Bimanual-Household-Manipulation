"""Offline prerecorded-polygon backend for the towel observation contract."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np

from tools.lib.towel_dataset import validate_annotation
from tools.lib.towel_geometry import polygon_area, quadrilateral_metrics
from tools.lib.towel_task_runtime import TowelTaskContractError


def _homography(value: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise TowelTaskContractError(
            "pixel_to_workcell_homography must be a finite 3x3 matrix"
        )
    if abs(float(np.linalg.det(matrix))) <= 1.0e-12:
        raise TowelTaskContractError(
            "pixel_to_workcell_homography must be invertible"
        )
    return matrix


def project_pixel_to_workcell(
    point_px: Sequence[float],
    pixel_to_workcell_homography: Sequence[Sequence[float]],
) -> tuple[float, float]:
    matrix = _homography(pixel_to_workcell_homography)
    homogeneous = matrix @ np.array(
        [float(point_px[0]), float(point_px[1]), 1.0], dtype=float
    )
    if abs(float(homogeneous[2])) <= 1.0e-12:
        raise TowelTaskContractError(
            "homography projects point to infinity"
        )
    point = (
        float(homogeneous[0] / homogeneous[2]),
        float(homogeneous[1] / homogeneous[2]),
    )
    if not all(math.isfinite(value) for value in point):
        raise TowelTaskContractError(
            "homography produced a non-finite workcell point"
        )
    return point


def prerecorded_annotation_observation(
    annotation: Mapping[str, Any],
    *,
    pixel_to_workcell_homography: Sequence[Sequence[float]],
    calibration_sha256: str,
    expected_full_mask_area_px: float,
    topology_confidence: float,
    stale: bool = False,
    outline_iou: float | None = None,
    fold_axis_costs: Mapping[str, float] | None = None,
    fold_direction_costs: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    """Convert reviewed polygon evidence into a runtime observation.

    This is intentionally not an image model. It lets the rest of the
    perception→state→plan pipeline run against prerecorded reviewed evidence.
    """
    if not isinstance(stale, bool):
        raise TowelTaskContractError("stale must be boolean")
    normalized = validate_annotation(annotation)
    if not math.isfinite(expected_full_mask_area_px) or expected_full_mask_area_px <= 0.0:
        raise TowelTaskContractError(
            "expected_full_mask_area_px must be finite and positive"
        )
    if not math.isfinite(topology_confidence) or not 0.0 <= topology_confidence <= 1.0:
        raise TowelTaskContractError(
            "topology_confidence must be within 0..1"
        )
    if len(calibration_sha256) != 64 or any(
        character not in "0123456789abcdef"
        for character in calibration_sha256
    ):
        raise TowelTaskContractError(
            "calibration_sha256 must be lowercase SHA-256"
        )
    matrix = _homography(pixel_to_workcell_homography)
    mask_area = polygon_area(annotation["segmentation_polygon_px"])
    visible_area_ratio = min(1.0, mask_area / expected_full_mask_area_px)
    corners = [
        {
            "point_xy_m": list(
                project_pixel_to_workcell(corner["point_px"], matrix)
            ),
            "confidence": float(corner["confidence"]),
            "visible": bool(corner["visible"]),
            "graspable": bool(corner["graspable"]),
        }
        for corner in annotation["corners"]
    ]
    flatness_score = 0.0
    usable_points = [
        corner["point_xy_m"]
        for corner in corners
        if corner["visible"] and corner["graspable"]
    ]
    if len(usable_points) == 4:
        metrics = quadrilateral_metrics(usable_points)
        flatness_score = max(
            0.0,
            1.0
            - max(
                metrics.edge_relative_spread,
                metrics.diagonal_relative_difference,
            ),
        )
    state = annotation["state_label"]
    fold_count = (
        2 if state == "FOLD_2_COMPLETE"
        else 1 if state == "FOLD_1_COMPLETE"
        else 0
    )
    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": normalized["observation_id"],
        "source_sha256": normalized["source_sha256"],
        "calibration_sha256": calibration_sha256,
        "visible_area_ratio": visible_area_ratio,
        "topology_confidence": topology_confidence,
        "flatness_score": flatness_score,
        "fold_count": fold_count,
        "outline_iou": outline_iou,
        "stale": bool(stale),
        "corners": corners,
        "backend": "prerecorded_reviewed_polygon_v1",
        "height_available": bool(annotation.get("height_available", False)),
    }
    if fold_axis_costs is not None:
        document["fold_axis_costs"] = dict(fold_axis_costs)
    if fold_direction_costs is not None:
        document["fold_direction_costs"] = dict(fold_direction_costs)
    return document
