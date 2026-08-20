#!/usr/bin/env python3
"""Assemble a multi-position Top-camera tabletop calibration session.

Calibration images fit only the common tabletop plane.  Independent validation
images are evaluated after that fit and never influence it.  The existing
validated eye-to-hand transform retains the left-base XY origin and yaw; this
tool applies only the minimum roll/pitch correction required to flatten the
observed tabletop at its physically configured base Z.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from calibrate_top_base_table import (
    base_plane_homography,
    detect_gridboard,
    load_yaml,
    matrix,
    matrix_document,
    transform_pixels,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def portable_path(path: Path) -> str:
    """Prefer a repository-relative evidence path when possible."""
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPOSITORY_ROOT))
    except ValueError:
        return str(resolved)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def minimal_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return the minimum proper rotation mapping one unit vector to another."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    source /= np.linalg.norm(source)
    target /= np.linalg.norm(target)
    cosine = float(np.clip(source @ target, -1.0, 1.0))
    if cosine > 1.0 - 1e-12:
        return np.eye(3, dtype=np.float64)
    if cosine < -1.0 + 1e-12:
        axis = np.cross(source, np.asarray([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(source, np.asarray([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3, dtype=np.float64)
    cross = np.cross(source, target)
    skew = np.asarray(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3) + skew + skew @ skew / (1.0 + cosine)


def fit_common_plane(
    base_points: np.ndarray,
    table_z_m: float,
) -> tuple[np.ndarray, dict]:
    """Fit one plane and return a base-frame roll/pitch correction."""
    points = np.asarray(base_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("base_points must be an Nx3 array")
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    residual_m = (points - centroid) @ normal
    rotation = minimal_rotation(normal, np.asarray([0.0, 0.0, 1.0]))
    preserved_centroid = np.asarray(
        [centroid[0], centroid[1], float(table_z_m)],
        dtype=np.float64,
    )
    correction = np.eye(4, dtype=np.float64)
    correction[:3, :3] = rotation
    correction[:3, 3] = preserved_centroid - rotation @ centroid
    rotation_deg = math.degrees(
        math.acos(
            float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
        )
    )
    return correction, {
        "normal_before_correction": normal.astype(float).tolist(),
        "centroid_before_correction_m": centroid.astype(float).tolist(),
        "roll_pitch_correction_deg": float(rotation_deg),
        "plane_fit_rms_mm": float(
            np.sqrt(np.mean(residual_m * residual_m)) * 1000.0
        ),
        "plane_fit_max_mm": float(np.max(np.abs(residual_m)) * 1000.0),
    }


def orthogonal_metric_errors_mm(
    object_xy: np.ndarray,
    mapped_xy: np.ndarray,
) -> np.ndarray:
    """Compare target geometry after a free in-plane orthogonal alignment.

    OpenCV GridBoard coordinates may face either table normal, so reflection is
    intentionally allowed.  Scale is fixed at one and cannot hide metric error.
    """
    source = np.asarray(object_xy, dtype=np.float64)
    target = np.asarray(mapped_xy, dtype=np.float64)
    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)
    u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
    orthogonal = vt.T @ u.T
    predicted = source_centered @ orthogonal.T + target.mean(axis=0)
    return np.linalg.norm(predicted - target, axis=1) * 1000.0


def horizontal_interval(hull: np.ndarray, y_value: float) -> tuple[float, float]:
    intersections: list[float] = []
    for start, end in zip(hull, np.roll(hull, -1, axis=0), strict=True):
        y0 = float(start[1])
        y1 = float(end[1])
        if abs(y1 - y0) < 1e-12:
            if abs(y_value - y0) < 1e-9:
                intersections.extend([float(start[0]), float(end[0])])
            continue
        if y_value < min(y0, y1) - 1e-9 or y_value > max(y0, y1) + 1e-9:
            continue
        ratio = (y_value - y0) / (y1 - y0)
        intersections.append(float(start[0] + ratio * (end[0] - start[0])))
    if len(intersections) < 2:
        raise ValueError("failed to intersect calibration coverage hull")
    return min(intersections), max(intersections)


def largest_axis_aligned_rectangle(
    points_xy: np.ndarray,
    samples: int = 501,
) -> np.ndarray:
    """Approximate the largest axis-aligned rectangle inside a convex hull."""
    hull = cv2.convexHull(
        np.asarray(points_xy, dtype=np.float32)
    ).reshape(-1, 2).astype(np.float64)
    y_values = np.linspace(float(hull[:, 1].min()), float(hull[:, 1].max()), samples)
    intervals = [horizontal_interval(hull, float(value)) for value in y_values]
    best: tuple[float, float, float, float, float] | None = None
    for lower_index, y_min in enumerate(y_values[:-1]):
        left_min, right_min = intervals[lower_index]
        for upper_index in range(lower_index + 1, len(y_values)):
            y_max = y_values[upper_index]
            left_max, right_max = intervals[upper_index]
            x_min = max(left_min, left_max)
            x_max = min(right_min, right_max)
            area = max(0.0, x_max - x_min) * float(y_max - y_min)
            if best is None or area > best[0]:
                best = (area, x_min, float(y_min), x_max, float(y_max))
    if best is None or best[0] <= 0.0:
        raise ValueError("calibration coverage has no positive-area rectangle")
    return np.asarray(best[1:], dtype=np.float64)


def load_detection(
    path: Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> dict:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to read image: {path}")
    return detect_gridboard(image, camera_matrix, distortion)


def base_points_for_detection(
    detection: dict,
    base_from_camera: np.ndarray,
) -> np.ndarray:
    homogeneous = np.column_stack(
        (
            detection["object_points"],
            np.ones(len(detection["object_points"]), dtype=np.float64),
        )
    )
    return (
        base_from_camera @ detection["camera_from_grid"] @ homogeneous.T
    ).T[:, :3]


def build_session(args: argparse.Namespace) -> tuple[dict, dict]:
    camera_info = load_yaml(args.camera_info)
    eye_to_hand = load_yaml(args.eye_to_hand)
    if not str(eye_to_hand.get("status", "")).startswith(
        "EYE_TO_HAND_VALIDATED_"
    ):
        raise RuntimeError("eye-to-hand input is not independently validated")
    camera_matrix = matrix(camera_info, "camera_matrix", 3, 3)
    distortion = matrix(
        camera_info, "distortion_coefficients", 1, 5
    ).reshape(-1)
    projection = matrix(camera_info, "projection_matrix", 3, 4)[:, :3]
    base_from_camera = matrix(eye_to_hand, "base_to_camera", 4, 4)

    calibration_detections = [
        load_detection(path, camera_matrix, distortion)
        for path in args.calibration_image
    ]
    validation_detections = [
        load_detection(path, camera_matrix, distortion)
        for path in args.validation_image
    ]
    base_points = np.concatenate(
        [
            base_points_for_detection(detection, base_from_camera)
            for detection in calibration_detections
        ]
    )
    correction, plane_fit = fit_common_plane(base_points, args.table_z_m)
    corrected_base_from_camera = correction @ base_from_camera
    pixel_to_base = base_plane_homography(
        projection,
        corrected_base_from_camera,
        args.table_z_m,
        np.zeros(2, dtype=np.float64),
    )

    coverage_points = []
    for detection in calibration_detections:
        rectified = cv2.undistortPoints(
            detection["pixels_raw"].reshape(-1, 1, 2),
            camera_matrix,
            distortion,
            P=projection,
        ).reshape(-1, 2)
        coverage_points.append(transform_pixels(rectified, pixel_to_base))
    coverage_points_array = np.concatenate(coverage_points)
    rectangle = largest_axis_aligned_rectangle(coverage_points_array)
    rectangle[:2] += args.roi_inset_m
    rectangle[2:] -= args.roi_inset_m
    x_min, y_min, x_max, y_max = rectangle
    if x_max <= x_min or y_max <= y_min:
        raise RuntimeError("ROI inset removed the full calibrated region")
    origin = np.asarray([x_min, y_min], dtype=np.float64)
    span = np.asarray([x_max - x_min, y_max - y_min], dtype=np.float64)
    board_from_base = np.asarray(
        [[1.0, 0.0, -x_min], [0.0, 1.0, -y_min], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    pixel_to_board = board_from_base @ pixel_to_base
    pixel_to_board /= pixel_to_board[2, 2]

    validation_results = []
    for path, detection in zip(
        args.validation_image, validation_detections, strict=True
    ):
        rectified = cv2.undistortPoints(
            detection["pixels_raw"].reshape(-1, 1, 2),
            camera_matrix,
            distortion,
            P=projection,
        ).reshape(-1, 2)
        mapped_base = transform_pixels(rectified, pixel_to_base)
        errors_mm = orthogonal_metric_errors_mm(
            detection["object_points"][:, :2], mapped_base
        )
        corrected_points = base_points_for_detection(
            detection, corrected_base_from_camera
        )
        z_errors_mm = np.abs(corrected_points[:, 2] - args.table_z_m) * 1000.0
        validation_results.append(
            {
                "image": portable_path(path),
                "image_sha256": sha256(path),
                "pnp_rms_px": float(detection["pnp_rms_px"]),
                "image_border_px": float(detection["image_border_px"]),
                "center_in_left_base_link_xy_m": mapped_base.mean(axis=0).astype(float).tolist(),
                "metric_error_rms_mm": float(
                    np.sqrt(np.mean(errors_mm * errors_mm))
                ),
                "metric_error_max_mm": float(np.max(errors_mm)),
                "plane_height_error_rms_mm": float(
                    np.sqrt(np.mean(z_errors_mm * z_errors_mm))
                ),
                "plane_height_error_max_mm": float(np.max(z_errors_mm)),
            }
        )

    thresholds = {
        "pnp_rms_px_max": 1.5,
        "image_border_px_min": 10.0,
        "plane_fit_rms_mm_max": 2.0,
        "plane_fit_max_mm_max": 5.0,
        "roll_pitch_correction_deg_max": 3.0,
        "validation_metric_error_max_mm": 5.0,
        "validation_plane_height_error_max_mm": 4.0,
    }
    failures = []
    all_detections = calibration_detections + validation_detections
    if max(item["pnp_rms_px"] for item in all_detections) > thresholds["pnp_rms_px_max"]:
        failures.append("PnP RMS exceeds threshold")
    if min(item["image_border_px"] for item in all_detections) < thresholds["image_border_px_min"]:
        failures.append("image border margin is below threshold")
    if plane_fit["plane_fit_rms_mm"] > thresholds["plane_fit_rms_mm_max"]:
        failures.append("common-plane RMS exceeds threshold")
    if plane_fit["plane_fit_max_mm"] > thresholds["plane_fit_max_mm_max"]:
        failures.append("common-plane maximum exceeds threshold")
    if plane_fit["roll_pitch_correction_deg"] > thresholds["roll_pitch_correction_deg_max"]:
        failures.append("roll/pitch correction exceeds threshold")
    if max(item["metric_error_max_mm"] for item in validation_results) > thresholds["validation_metric_error_max_mm"]:
        failures.append("validation metric error exceeds threshold")
    if max(item["plane_height_error_max_mm"] for item in validation_results) > thresholds["validation_plane_height_error_max_mm"]:
        failures.append("validation plane-height error exceeds threshold")

    validation = {
        "schema_version": 1,
        "status": (
            "PASS_FULL_TABLE_BASE_CALIBRATION_MOTION_STILL_NOT_AUTHORIZED"
            if not failures
            else "REJECTED_FULL_TABLE_BASE_CALIBRATION"
        ),
        "transform_validated": not failures,
        "motion_authorized": False,
        "robot_target_available": False,
        "method": "six_position_common_plane_fit_plus_two_independent_validations",
        "table_z_in_left_base_link_m": float(args.table_z_m),
        "plane_fit": plane_fit,
        "corrected_base_from_camera": matrix_document(corrected_base_from_camera),
        "calibrated_region": {
            "base_xy_min_m": origin.astype(float).tolist(),
            "base_xy_max_m": (origin + span).astype(float).tolist(),
            "span_xy_m": span.astype(float).tolist(),
            "coverage_hull_inset_m": float(args.roi_inset_m),
        },
        "calibration_images": [
            {"path": portable_path(path), "sha256": sha256(path)}
            for path in args.calibration_image
        ],
        "independent_validation": validation_results,
        "acceptance": thresholds,
        "failure_reasons": failures,
        "required_next_gate": "plan-only can pose validation; motion remains disabled",
    }
    runtime = {
        "schema_version": 1,
        "status": (
            "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
            if not failures
            else "REJECTED_TABLE_BASE_CALIBRATION"
        ),
        "motion_authorized": False,
        "robot_target_available": False,
        "camera": {
            "name": "so101_top",
            "image_width": int(camera_info["image_width"]),
            "image_height": int(camera_info["image_height"]),
            "input_domain": "rectified_pixel_using_projection_matrix",
            "camera_info_file": args.camera_info.name,
            "camera_info_sha256": sha256(args.camera_info),
        },
        "capture": {
            "method": validation["method"],
            "validation_file": portable_path(args.validation_output),
            "image_sha256": [sha256(path) for path in args.calibration_image + args.validation_image],
        },
        "board": {
            "origin": "left_base_link aligned region inside multi-position coverage hull",
            "positive_x": "parallel to left_base_link +X",
            "positive_y": "parallel to left_base_link +Y",
            "calibrated_span_m": span.astype(float).tolist(),
            "origin_in_left_base_link_xy_m": origin.astype(float).tolist(),
            "table_z_in_left_base_link_m": float(args.table_z_m),
        },
        "homography": {
            "rectified_pixel_to_board_m": matrix_document(pixel_to_board),
            "board_m_to_rectified_pixel": matrix_document(np.linalg.inv(pixel_to_board)),
        },
        "fit_quality": {
            **plane_fit,
            "pnp_rms_px_max": float(max(item["pnp_rms_px"] for item in all_detections)),
            "validation_metric_error_max_mm": float(max(item["metric_error_max_mm"] for item in validation_results)),
            "validation_plane_height_error_max_mm": float(max(item["plane_height_error_max_mm"] for item in validation_results)),
        },
        "base_registration": {
            "status": "MULTI_POSITION_COMMON_PLANE_INDEPENDENTLY_VALIDATED" if not failures else "REJECTED",
            "transform_validated": not failures,
            "motion_authorized": False,
            "reference": "left_base_link",
            "base_from_board": matrix_document(
                np.asarray(
                    [
                        [1.0, 0.0, 0.0, origin[0]],
                        [0.0, 1.0, 0.0, origin[1]],
                        [0.0, 0.0, 1.0, args.table_z_m],
                        [0.0, 0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                )
            ),
        },
        "required_next_gate": "plan-only can pose validation; robot motion remains disabled",
    }
    return validation, runtime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-image", type=Path, action="append", required=True)
    parser.add_argument("--validation-image", type=Path, action="append", required=True)
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=Path("ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml"),
    )
    parser.add_argument(
        "--eye-to-hand",
        type=Path,
        default=Path("artifacts/top_eye_to_hand/2026-07-30/independent_validation/candidate.yaml"),
    )
    parser.add_argument("--table-z-m", type=float, default=-0.005)
    parser.add_argument("--roi-inset-m", type=float, default=0.010)
    parser.add_argument("--validation-output", type=Path, required=True)
    parser.add_argument("--runtime-output", type=Path, required=True)
    args = parser.parse_args()
    if len(args.calibration_image) < 4:
        parser.error("at least four --calibration-image values are required")
    if len(args.validation_image) < 2:
        parser.error("at least two --validation-image values are required")
    if args.roi_inset_m < 0.0:
        parser.error("--roi-inset-m must be non-negative")
    args.calibration_image = [path.resolve() for path in args.calibration_image]
    args.validation_image = [path.resolve() for path in args.validation_image]
    args.camera_info = args.camera_info.resolve()
    args.eye_to_hand = args.eye_to_hand.resolve()
    args.validation_output = args.validation_output.resolve()
    args.runtime_output = args.runtime_output.resolve()
    return args


def main() -> int:
    args = parse_args()
    validation, runtime = build_session(args)
    args.validation_output.parent.mkdir(parents=True, exist_ok=True)
    args.validation_output.write_text(
        yaml.safe_dump(validation, sort_keys=False), encoding="utf-8"
    )
    runtime["capture"]["validation_sha256"] = sha256(args.validation_output)
    args.runtime_output.parent.mkdir(parents=True, exist_ok=True)
    args.runtime_output.write_text(
        yaml.safe_dump(runtime, sort_keys=False), encoding="utf-8"
    )
    passed = not validation["failure_reasons"]
    print(
        "TOP_BASE_TABLE_SESSION_%s span_m=%.6fx%.6f "
        "plane_rms_mm=%.6f validation_max_mm=%.6f output=%s"
        % (
            "PASS" if passed else "REJECTED",
            runtime["board"]["calibrated_span_m"][0],
            runtime["board"]["calibrated_span_m"][1],
            validation["plane_fit"]["plane_fit_rms_mm"],
            runtime["fit_quality"]["validation_metric_error_max_mm"],
            args.runtime_output,
        )
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
