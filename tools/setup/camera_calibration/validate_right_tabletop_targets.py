#!/usr/bin/env python3
"""Cross-check tabletop GridBoard targets with Top, right FK, and wrist_b.

Each capture observes one board pose simultaneously from the independently
calibrated Top and right wrist cameras while the right arm is held under
resident torque.  The Top chain gives ``T_workcell_target`` directly.  The
right chain gives the same pose through the registered right FK and validated
eye-in-hand transform.  Agreement at three spatially separated, post-solve
board placements validates the right FK for tabletop target coordinates; it
still does not authorize motion.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from solve_top_base_visual_registration import load_yaml, yaml_matrix  # noqa: E402
from solve_top_eye_to_hand import (  # noqa: E402
    ARM_JOINT_NAMES_BY_SIDE,
    average_target_poses,
    detect_target_pose,
    matrix_document,
    parse_target,
    urdf_fk,
)
from solve_wrist_eye_in_hand import (  # noqa: E402
    RIGHT_REGISTRATION_METHOD,
    validated_right_joint_zero_offsets,
)


CAPTURE_STATUS = "RIGHT_TABLETOP_DUAL_VIEW_RESIDENT_HOLD_CAPTURE_PASS"
PASS_STATUS = "RIGHT_TABLETOP_TARGETS_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
REJECTED_STATUS = "RIGHT_TABLETOP_TARGETS_REJECTED"
MINIMUM_CAPTURES = 3
MINIMUM_TRAINING_CAPTURES = 2
MINIMUM_VALIDATION_CAPTURES = 2
MINIMUM_TARGET_XY_SPAN_M = 0.080
MINIMUM_TARGET_XY_TRIANGLE_ALTITUDE_M = 0.050
MINIMUM_RIGHT_JOINT_CONFIGURATION_SPAN_RAD = 0.10
# A 12 mm RMS remains below the unchanged 15 mm per-pose ceiling and is
# meaningful for a 300 mm towel corner followed by wrist-guided refinement.
# Tighter sub-centimetre positioning is not inferred from this RGB-only gate.
MAXIMUM_XY_RMS_M = 0.012
MAXIMUM_XY_ERROR_M = 0.015
MAXIMUM_Z_ERROR_M = 0.020
MAXIMUM_YAW_ERROR_RAD = math.radians(3.0)
MAXIMUM_PNP_RMS_PX = 2.5
# The original eye-in-hand fit's near-edge sample had its largest residual.
# Keep tabletop fusion inside a modest, operationally supported image margin;
# outside it the system must fall back to Top/observe-clear rather than
# pretending that a low reprojection RMS proves metric accuracy.
MINIMUM_IMAGE_BORDER_PX = 25.0
MAXIMUM_WITHIN_CAPTURE_TRANSLATION_SPAN_M = 0.005
MAXIMUM_WITHIN_CAPTURE_ROTATION_SPAN_RAD = math.radians(1.5)
REQUIRED_CONTROLLED_HOLD_OWNER = "resident_right_calibration_operator"
MAXIMUM_GRIPPER_TRANSLATION_CORRECTION_M = 0.025
TRANSLATION_CORRECTION_METHOD = (
    "bounded_gripper_frame_translation_only_least_squares"
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _matrix(document: dict, key: str) -> np.ndarray:
    value = yaml_matrix(document, key, 4, 4)
    return _validated_transform(value, key)


def _matrix_value(document: dict, name: str) -> np.ndarray:
    if document.get("rows") != 4 or document.get("cols") != 4:
        raise RuntimeError(f"{name} is not a 4x4 matrix")
    values = np.asarray(document.get("data", []), dtype=float)
    if values.size != 16 or not np.all(np.isfinite(values)):
        raise RuntimeError(f"{name} contains invalid values")
    return _validated_transform(values.reshape(4, 4), name)


def _validated_transform(value: np.ndarray, name: str) -> np.ndarray:
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-7):
        raise RuntimeError(f"{name} rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-7):
        raise RuntimeError(f"{name} rotation determinant is not +1")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-10):
        raise RuntimeError(f"{name} homogeneous row is invalid")
    return value


def wrap_half_turn(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi / 2.0) % math.pi - math.pi / 2.0


def planar_yaw(transform: np.ndarray) -> float:
    axis = np.asarray(transform[:2, 0], dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm < 1.0e-6:
        raise RuntimeError("target x-axis has no stable tabletop projection")
    return math.atan2(float(axis[1]), float(axis[0]))


def compare_target_poses(
    top_target: np.ndarray, right_target: np.ndarray
) -> dict:
    delta = np.asarray(right_target[:3, 3] - top_target[:3, 3], dtype=float)
    yaw_error = abs(
        wrap_half_turn(planar_yaw(right_target) - planar_yaw(top_target))
    )
    rotation_error = Rotation.from_matrix(
        top_target[:3, :3].T @ right_target[:3, :3]
    ).magnitude()
    return {
        "delta_xyz_m": [float(value) for value in delta],
        "xy_error_m": float(np.linalg.norm(delta[:2])),
        "z_error_m": abs(float(delta[2])),
        "translation_error_m": float(np.linalg.norm(delta)),
        "yaw_error_rad": float(yaw_error),
        "yaw_error_deg": math.degrees(yaw_error),
        "rotation_error_rad": float(rotation_error),
        "rotation_error_deg": math.degrees(rotation_error),
    }


def validate_capture_provenance(document: dict) -> None:
    record_kind = document.get("record_kind")
    if (
        document.get("schema_version") != 1
        or record_kind
        not in (
            "right_tabletop_dual_view_capture",
            "right_tabletop_staged_capture",
        )
        or document.get("status") != CAPTURE_STATUS
        or document.get("motion_authorized") is not False
        or document.get("source_motion_authorized") is not True
        or document.get("robot_target_available") is not False
    ):
        raise RuntimeError("dual-view capture is not fail-closed and validated")
    capture = document.get("capture", {})
    hold = document.get("resident_torque_hold", {})
    positions = np.asarray(capture.get("measured_arm_rad", []), dtype=float)
    if (
        capture.get("arm") != "right"
        or positions.shape != (5,)
        or not np.all(np.isfinite(positions))
        or capture.get("joint_state_source")
        != "resident_terminal_measured_anchor"
        or hold.get("status_service") != "/bimanual_stream_adapter/status"
        or hold.get("torque_hold_active") is not True
        or hold.get("owner") != REQUIRED_CONTROLLED_HOLD_OWNER
        or int(hold.get("arbiter_epoch", 0)) <= 0
        or hold.get("owner") != hold.get("required_owner")
        or int(hold.get("arbiter_epoch", 0))
        != int(hold.get("required_epoch", 0))
        or float(hold.get("terminal_anchor_stamp", 0.0)) <= 0.0
    ):
        raise RuntimeError("dual-view capture lacks resident hold provenance")
    top_files = capture.get("top_image_files", [])
    wrist_files = capture.get("wrist_image_files", [])
    if len(top_files) < 5 or len(top_files) != len(wrist_files):
        raise RuntimeError("dual-view capture has invalid image pairs")
    if record_kind == "right_tabletop_staged_capture":
        staged = document.get("staged_capture", {})
        top_stamps = capture.get("top_source_stamps", [])
        wrist_stamps = capture.get("wrist_source_stamps", [])
        if (
            capture.get("capture_mode") != "staged_top_then_wrist"
            or staged.get("stationary_board_confirmation")
            != "RIGHT_TABLETOP_BOARD_FIXED_BETWEEN_STAGES"
            or staged.get("top_completed_before_wrist") is not True
            or len(top_stamps) < 5
            or len(wrist_stamps) < 5
            or max(float(value) for value in top_stamps)
            >= min(float(value) for value in wrist_stamps)
            or float(staged.get("top_source_stamp_last", math.inf))
            >= float(staged.get("wrist_source_stamp_first", -math.inf))
        ):
            raise RuntimeError("staged capture lacks fixed-board provenance")
    elif float(capture.get("maximum_pair_skew_s", math.inf)) > float(
        capture.get("pair_skew_limit_s", 0.0)
    ):
        raise RuntimeError("dual-view capture exceeds its synchronization gate")


def _candidate_capture_ids(candidate: dict) -> set[str]:
    result: set[str] = set()
    for key in ("training_fit", "validation_fit"):
        for capture in candidate.get(key, {}).get("captures", []):
            result.add(str(capture.get("id", "")))
    return result


def validate_sources(
    top_worktable: dict,
    right_registration: dict,
    wrist_candidate: dict,
    top_camera_info_path: Path,
    wrist_camera_info_path: Path,
) -> None:
    camera = top_worktable.get("camera", {})
    base_registration = top_worktable.get("base_registration", {})
    if (
        top_worktable.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or top_worktable.get("motion_authorized") is not False
        or base_registration.get("transform_validated") is not True
        or base_registration.get("motion_authorized") is not False
        or camera.get("image_width") != 1280
        or camera.get("image_height") != 960
        or camera.get("input_domain")
        != "rectified_pixel_using_projection_matrix"
    ):
        raise RuntimeError("Top worktable calibration is not validated")
    validated_right_joint_zero_offsets(right_registration)
    if (
        wrist_candidate.get("status")
        != "EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or wrist_candidate.get("arm") != "right"
        or wrist_candidate.get("motion_authorized") is not False
        or wrist_candidate.get("method")
        != "planar_gridboard_hand_eye_tsai_registered_right_fk"
    ):
        raise RuntimeError("right wrist eye-in-hand candidate is not validated")
    if camera.get("camera_info_sha256") != sha256_file(top_camera_info_path):
        raise RuntimeError("Top camera-info SHA does not match the worktable")
    if wrist_candidate.get("camera_info_sha256") != sha256_file(
        wrist_camera_info_path
    ):
        raise RuntimeError("wrist_b camera-info SHA does not match its calibration")


def _observed_pose(
    image_files: list[str],
    capture_directory: Path,
    camera_info: dict,
    target_specification,
) -> tuple[np.ndarray, dict]:
    camera_matrix = yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = yaml_matrix(
        camera_info, "distortion_coefficients", 1, 5
    ).reshape(-1)
    poses: list[np.ndarray] = []
    pnp: list[float] = []
    borders: list[float] = []
    detected_ids: tuple[int, ...] | None = None
    expected_size = (
        int(camera_info["image_width"]),
        int(camera_info["image_height"]),
    )
    for value in image_files:
        path = Path(str(value))
        if not path.is_absolute():
            path = capture_directory / path
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"failed to read image: {path}")
        if (image.shape[1], image.shape[0]) != expected_size:
            raise RuntimeError(
                f"image size {(image.shape[1], image.shape[0])} does not "
                f"match CameraInfo {expected_size}: {path}"
            )
        pose, error, border, ids = detect_target_pose(
            path, camera_matrix, distortion, target_specification
        )
        poses.append(pose)
        pnp.append(error)
        borders.append(border)
        if detected_ids is None:
            detected_ids = ids
        elif ids != detected_ids:
            raise RuntimeError("marker IDs changed within one dual-view capture")
    translation_span = 0.0
    rotation_span = 0.0
    for first in range(len(poses)):
        for second in range(first + 1, len(poses)):
            translation_span = max(
                translation_span,
                float(np.linalg.norm(poses[first][:3, 3] - poses[second][:3, 3])),
            )
            rotation_span = max(
                rotation_span,
                float(
                    Rotation.from_matrix(
                        poses[first][:3, :3].T @ poses[second][:3, :3]
                    ).magnitude()
                ),
            )
    return average_target_poses(poses), {
        "pnp_rms_px_max": max(pnp),
        "image_border_px_min": min(borders),
        "within_capture_translation_span_m": translation_span,
        "within_capture_rotation_span_rad": rotation_span,
        "within_capture_rotation_span_deg": math.degrees(rotation_span),
        "detected_marker_ids": list(detected_ids or ()),
    }


def top_target_from_worktable(
    camera_to_target: np.ndarray,
    camera_info: dict,
    worktable: dict,
) -> np.ndarray:
    """Map the observed flat target into workcell via the runtime homography.

    PnP supplies the target origin and in-plane axes in raw image pixels.  The
    pixels are rectified with the active CameraInfo projection matrix and then
    passed through the independently validated table homography.  Depth from
    the monocular PnP is deliberately not consumed.
    """
    camera_matrix = yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = yaml_matrix(
        camera_info, "distortion_coefficients", 1, 5
    ).reshape(-1)
    projection = yaml_matrix(camera_info, "projection_matrix", 3, 4)
    rotation_vector, _ = cv2.Rodrigues(camera_to_target[:3, :3])
    object_points = np.asarray(
        [[0.0, 0.0, 0.0], [0.050, 0.0, 0.0], [0.0, 0.050, 0.0]],
        dtype=np.float64,
    )
    raw_pixels, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        camera_to_target[:3, 3],
        camera_matrix,
        distortion,
    )
    rectified = cv2.undistortPoints(
        raw_pixels,
        camera_matrix,
        distortion,
        P=projection,
    ).reshape(3, 2)
    pixel_to_board = yaml_matrix(
        worktable["homography"], "rectified_pixel_to_board_m", 3, 3
    )
    base_from_board = _matrix_value(
        worktable["base_registration"]["base_from_board"],
        "top_worktable.base_from_board",
    )

    workcell_points: list[np.ndarray] = []
    for pixel in rectified:
        homogeneous = pixel_to_board @ np.asarray(
            [float(pixel[0]), float(pixel[1]), 1.0], dtype=float
        )
        if abs(float(homogeneous[2])) < 1.0e-12:
            raise RuntimeError("Top worktable homography is singular at target")
        board_xy = homogeneous[:2] / homogeneous[2]
        board_point = np.asarray([board_xy[0], board_xy[1], 0.0, 1.0])
        workcell_points.append((base_from_board @ board_point)[:3])

    origin, x_point, y_point = workcell_points
    x_axis = x_point - origin
    y_hint = y_point - origin
    x_axis = x_axis / np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, y_hint)
    z_axis = z_axis / np.linalg.norm(z_axis)
    if z_axis[2] < 0.0:
        z_axis = -z_axis
    y_axis = np.cross(z_axis, x_axis)
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    result[:3, 3] = origin
    return _validated_transform(result, "top_worktable_target")


def evaluate_capture(
    capture_path: Path,
    capture_document: dict,
    top_worktable: dict,
    right_registration: dict,
    wrist_candidate: dict,
    top_camera_info: dict,
    wrist_camera_info: dict,
    urdf_xml: str,
) -> dict:
    validate_capture_provenance(capture_document)
    capture = capture_document["capture"]
    capture_id = str(capture["id"])
    calibration_ids = _candidate_capture_ids(wrist_candidate)
    if capture_id in calibration_ids:
        raise RuntimeError(
            f"tabletop capture {capture_id} was already used by wrist calibration"
        )
    specification = parse_target(capture_document)
    wrist_specification = parse_target(wrist_candidate)
    if specification != wrist_specification:
        raise RuntimeError("dual-view target does not match wrist calibration board")

    directory = capture_path.resolve().parent
    if capture_document.get("record_kind") == "right_tabletop_staged_capture":
        staged = capture_document["staged_capture"]
        top_stage_path = directory / str(staged.get("top_stage_file", ""))
        if (
            not top_stage_path.is_file()
            or sha256_file(top_stage_path) != staged.get("top_stage_sha256")
        ):
            raise RuntimeError("staged Top capture file hash does not match")
    top_camera_to_target, top_quality = _observed_pose(
        list(capture["top_image_files"]),
        directory,
        top_camera_info,
        specification,
    )
    wrist_camera_to_target, wrist_quality = _observed_pose(
        list(capture["wrist_image_files"]),
        directory,
        wrist_camera_info,
        specification,
    )
    for name, quality in (("top", top_quality), ("wrist", wrist_quality)):
        if quality["pnp_rms_px_max"] > MAXIMUM_PNP_RMS_PX:
            raise RuntimeError(f"{name} PnP RMS exceeds {MAXIMUM_PNP_RMS_PX}px")
        if quality["image_border_px_min"] < MINIMUM_IMAGE_BORDER_PX:
            raise RuntimeError(
                f"{name} board margin is below {MINIMUM_IMAGE_BORDER_PX}px"
            )
        if (
            quality["within_capture_translation_span_m"]
            > MAXIMUM_WITHIN_CAPTURE_TRANSLATION_SPAN_M
        ):
            raise RuntimeError(f"{name} target moved during capture")
        if (
            quality["within_capture_rotation_span_rad"]
            > MAXIMUM_WITHIN_CAPTURE_ROTATION_SPAN_RAD
        ):
            raise RuntimeError(f"{name} target rotated during capture")

    workcell_to_target_top = top_target_from_worktable(
        top_camera_to_target,
        top_camera_info,
        top_worktable,
    )

    offsets, registration = validated_right_joint_zero_offsets(
        right_registration
    )
    measured = np.asarray(capture["measured_arm_rad"], dtype=float)
    corrected = measured + offsets
    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    right_base_to_gripper = urdf_fk(
        urdf_xml,
        "right_base_link",
        "right_gripper_frame_link",
        dict(zip(joint_names, corrected, strict=True)),
    )
    workcell_to_right_base = _matrix_value(
        registration["workcell_to_right_base"],
        "workcell_to_right_base",
    )
    gripper_to_wrist = _matrix(wrist_candidate, "gripper_to_camera")
    workcell_to_target_right = (
        workcell_to_right_base
        @ right_base_to_gripper
        @ gripper_to_wrist
        @ wrist_camera_to_target
    )
    residual = compare_target_poses(
        workcell_to_target_top, workcell_to_target_right
    )
    return {
        "id": capture_id,
        "source": {
            "path": str(capture_path.resolve()),
            "sha256": sha256_file(capture_path),
            "resident_owner": capture_document["resident_torque_hold"]["owner"],
            "resident_epoch": int(
                capture_document["resident_torque_hold"]["arbiter_epoch"]
            ),
        },
        "measured_right_arm_rad": [float(value) for value in measured],
        "top_target": matrix_document(workcell_to_target_top),
        "right_fk_wrist_target": matrix_document(workcell_to_target_right),
        "top_target_xyz_m": [
            float(value) for value in workcell_to_target_top[:3, 3]
        ],
        "residual": residual,
        "quality": {"top": top_quality, "wrist": wrist_quality},
    }


def _workcell_to_gripper_rotation(
    sample: dict,
    right_registration: dict,
    urdf_xml: str,
) -> np.ndarray:
    offsets, registration = validated_right_joint_zero_offsets(
        right_registration
    )
    measured = np.asarray(sample["measured_right_arm_rad"], dtype=float)
    corrected = measured + offsets
    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    base_to_gripper = urdf_fk(
        urdf_xml,
        "right_base_link",
        "right_gripper_frame_link",
        dict(zip(joint_names, corrected, strict=True)),
    )
    workcell_to_base = _matrix_value(
        registration["workcell_to_right_base"],
        "workcell_to_right_base",
    )
    return (workcell_to_base @ base_to_gripper)[:3, :3]


def solve_gripper_translation_correction(
    training_samples: list[dict],
    right_registration: dict,
    urdf_xml: str,
) -> np.ndarray:
    """Fit only camera translation in gripper coordinates.

    A translation correction in the gripper frame appears in workcell
    coordinates as ``R_workcell_gripper @ correction``.  Rotation stays
    exactly equal to the independently validated eye-in-hand candidate.
    """
    if len(training_samples) < MINIMUM_TRAINING_CAPTURES:
        raise RuntimeError(
            f"training capture count {len(training_samples)} < "
            f"{MINIMUM_TRAINING_CAPTURES}"
        )
    blocks: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    for sample in training_samples:
        blocks.append(
            _workcell_to_gripper_rotation(
                sample, right_registration, urdf_xml
            )
        )
        targets.append(-np.asarray(sample["residual"]["delta_xyz_m"]))
    correction, _, rank, _ = np.linalg.lstsq(
        np.vstack(blocks), np.hstack(targets), rcond=None
    )
    if rank != 3 or not np.all(np.isfinite(correction)):
        raise RuntimeError("translation correction fit is rank deficient")
    magnitude = float(np.linalg.norm(correction))
    if magnitude > MAXIMUM_GRIPPER_TRANSLATION_CORRECTION_M:
        raise RuntimeError(
            f"translation correction {magnitude:.6f} m exceeds "
            f"{MAXIMUM_GRIPPER_TRANSLATION_CORRECTION_M:.6f} m"
        )
    return correction


def apply_gripper_translation_correction(
    sample: dict,
    correction_gripper_m: np.ndarray,
    right_registration: dict,
    urdf_xml: str,
) -> dict:
    corrected = copy.deepcopy(sample)
    workcell_delta = _workcell_to_gripper_rotation(
        sample, right_registration, urdf_xml
    ) @ np.asarray(correction_gripper_m, dtype=float)
    top_target = _matrix_value(sample["top_target"], "top_target")
    right_target = _matrix_value(
        sample["right_fk_wrist_target"], "right_fk_wrist_target"
    ).copy()
    right_target[:3, 3] += workcell_delta
    corrected["uncorrected_residual"] = copy.deepcopy(sample["residual"])
    corrected["right_fk_wrist_target"] = matrix_document(right_target)
    corrected["residual"] = compare_target_poses(top_target, right_target)
    corrected["translation_correction_workcell_m"] = [
        float(value) for value in workcell_delta
    ]
    return corrected


def partition_metrics(samples: list[dict]) -> dict:
    positions = [
        np.asarray(sample["top_target_xyz_m"][:2], dtype=float)
        for sample in samples
    ]
    joints = [
        np.asarray(sample["measured_right_arm_rad"], dtype=float)
        for sample in samples
    ]
    span = max(
        (
            float(np.linalg.norm(first - second))
            for index, first in enumerate(positions)
            for second in positions[index + 1 :]
        ),
        default=0.0,
    )
    joint_span = max(
        (
            float(np.linalg.norm(first - second))
            for index, first in enumerate(joints)
            for second in joints[index + 1 :]
        ),
        default=0.0,
    )
    xy = [float(sample["residual"]["xy_error_m"]) for sample in samples]
    z = [float(sample["residual"]["z_error_m"]) for sample in samples]
    yaw = [float(sample["residual"]["yaw_error_rad"]) for sample in samples]
    return {
        "capture_count": len(samples),
        "target_xy_span_m": span,
        "right_joint_configuration_span_rad": joint_span,
        "xy_rms_m": math.sqrt(
            sum(value * value for value in xy) / max(1, len(xy))
        ),
        "xy_max_m": max(xy, default=math.inf),
        "z_max_m": max(z, default=math.inf),
        "yaw_max_rad": max(yaw, default=math.inf),
        "yaw_max_deg": math.degrees(max(yaw, default=math.inf)),
    }


def classify_calibrated_partitions(
    training_samples: list[dict], validation_samples: list[dict]
) -> tuple[str, list[str], dict]:
    failures: list[str] = []
    training_ids = {str(sample["id"]) for sample in training_samples}
    validation_ids = {str(sample["id"]) for sample in validation_samples}
    if len(training_samples) < MINIMUM_TRAINING_CAPTURES:
        failures.append("training capture count is below minimum")
    if len(validation_samples) < MINIMUM_VALIDATION_CAPTURES:
        failures.append("validation capture count is below minimum")
    if training_ids & validation_ids:
        failures.append("training and validation capture IDs overlap")
    source_hashes = [
        str(sample["source"]["sha256"])
        for sample in training_samples + validation_samples
    ]
    if len(source_hashes) != len(set(source_hashes)):
        failures.append("training and validation source hashes overlap")

    combined_status, combined_failures, combined = classify(
        training_samples + validation_samples
    )
    if combined_status != PASS_STATUS:
        failures.extend(combined_failures)
    result = {
        "training": partition_metrics(training_samples),
        "validation": partition_metrics(validation_samples),
        "combined": combined,
    }
    for name in ("training", "validation"):
        metrics = result[name]
        if metrics["target_xy_span_m"] < MINIMUM_TARGET_XY_SPAN_M:
            failures.append(f"{name} target XY span is below minimum")
        if (
            metrics["right_joint_configuration_span_rad"]
            < MINIMUM_RIGHT_JOINT_CONFIGURATION_SPAN_RAD
        ):
            failures.append(
                f"{name} right joint configuration span is below minimum"
            )
        if metrics["xy_rms_m"] > MAXIMUM_XY_RMS_M:
            failures.append(f"{name} XY RMS exceeds limit")
        if metrics["xy_max_m"] > MAXIMUM_XY_ERROR_M:
            failures.append(f"{name} XY max exceeds limit")
        if metrics["z_max_m"] > MAXIMUM_Z_ERROR_M:
            failures.append(f"{name} Z max exceeds limit")
        if metrics["yaw_max_rad"] > MAXIMUM_YAW_ERROR_RAD:
            failures.append(f"{name} yaw max exceeds limit")
    failures = list(dict.fromkeys(failures))
    return (PASS_STATUS if not failures else REJECTED_STATUS), failures, result


def corrected_wrist_transform_document(
    wrist_candidate: dict, correction_gripper_m: np.ndarray
) -> dict:
    original_gripper_to_camera = _matrix(
        wrist_candidate, "gripper_to_camera"
    )
    original_mount_to_camera = _matrix(
        wrist_candidate, "mount_center_to_camera"
    )
    gripper_to_mount = (
        original_gripper_to_camera @ np.linalg.inv(original_mount_to_camera)
    )
    corrected_gripper_to_camera = original_gripper_to_camera.copy()
    corrected_gripper_to_camera[:3, 3] += correction_gripper_m
    corrected_mount_to_camera = (
        np.linalg.inv(gripper_to_mount) @ corrected_gripper_to_camera
    )
    rpy = Rotation.from_matrix(
        corrected_mount_to_camera[:3, :3]
    ).as_euler("xyz")
    return {
        "gripper_to_camera": matrix_document(corrected_gripper_to_camera),
        "mount_center_to_camera": matrix_document(corrected_mount_to_camera),
        "xacro_defaults": {
            "right_wrist_camera_xyz": [
                float(value) for value in corrected_mount_to_camera[:3, 3]
            ],
            "right_wrist_camera_optical_rpy": [float(value) for value in rpy],
        },
    }


def classify(samples: list[dict]) -> tuple[str, list[str], dict]:
    failures: list[str] = []
    if len(samples) < MINIMUM_CAPTURES:
        failures.append(
            f"capture count {len(samples)} < {MINIMUM_CAPTURES}"
        )
    ids = [str(sample["id"]) for sample in samples]
    if len(ids) != len(set(ids)):
        failures.append("capture IDs are not unique")
    positions = [np.asarray(sample["top_target_xyz_m"][:2]) for sample in samples]
    span = 0.0
    for first in range(len(positions)):
        for second in range(first + 1, len(positions)):
            span = max(
                span,
                float(np.linalg.norm(positions[first] - positions[second])),
            )
    if span < MINIMUM_TARGET_XY_SPAN_M:
        failures.append(
            f"target XY span {span:.6f} m < {MINIMUM_TARGET_XY_SPAN_M:.6f} m"
        )
    triangle_altitude = 0.0
    for first in range(len(positions)):
        for second in range(first + 1, len(positions)):
            for third in range(second + 1, len(positions)):
                first_to_second = positions[second] - positions[first]
                first_to_third = positions[third] - positions[first]
                second_to_third = positions[third] - positions[second]
                longest_side = max(
                    float(np.linalg.norm(first_to_second)),
                    float(np.linalg.norm(first_to_third)),
                    float(np.linalg.norm(second_to_third)),
                )
                if longest_side <= 0.0:
                    continue
                twice_area = abs(
                    float(
                        first_to_second[0] * first_to_third[1]
                        - first_to_second[1] * first_to_third[0]
                    )
                )
                triangle_altitude = max(
                    triangle_altitude,
                    twice_area / longest_side,
                )
    if triangle_altitude < MINIMUM_TARGET_XY_TRIANGLE_ALTITUDE_M:
        failures.append(
            f"target XY triangle altitude {triangle_altitude:.6f} m < "
            f"{MINIMUM_TARGET_XY_TRIANGLE_ALTITUDE_M:.6f} m"
        )
    joint_positions = [
        np.asarray(sample["measured_right_arm_rad"], dtype=float)
        for sample in samples
    ]
    joint_span = 0.0
    for first in range(len(joint_positions)):
        for second in range(first + 1, len(joint_positions)):
            joint_span = max(
                joint_span,
                float(np.linalg.norm(joint_positions[first] - joint_positions[second])),
            )
    if joint_span < MINIMUM_RIGHT_JOINT_CONFIGURATION_SPAN_RAD:
        failures.append(
            f"right joint configuration span {joint_span:.6f} rad < "
            f"{MINIMUM_RIGHT_JOINT_CONFIGURATION_SPAN_RAD:.6f} rad"
        )
    xy = [float(sample["residual"]["xy_error_m"]) for sample in samples]
    z = [float(sample["residual"]["z_error_m"]) for sample in samples]
    yaw = [float(sample["residual"]["yaw_error_rad"]) for sample in samples]
    xy_rms = math.sqrt(sum(value * value for value in xy) / max(1, len(xy)))
    xy_max = max(xy, default=math.inf)
    z_max = max(z, default=math.inf)
    yaw_max = max(yaw, default=math.inf)
    if xy_rms > MAXIMUM_XY_RMS_M:
        failures.append(f"XY RMS {xy_rms:.6f} m > {MAXIMUM_XY_RMS_M:.6f} m")
    if xy_max > MAXIMUM_XY_ERROR_M:
        failures.append(f"XY max {xy_max:.6f} m > {MAXIMUM_XY_ERROR_M:.6f} m")
    if z_max > MAXIMUM_Z_ERROR_M:
        failures.append(f"Z max {z_max:.6f} m > {MAXIMUM_Z_ERROR_M:.6f} m")
    if yaw_max > MAXIMUM_YAW_ERROR_RAD:
        failures.append(
            f"yaw max {math.degrees(yaw_max):.3f} deg > "
            f"{math.degrees(MAXIMUM_YAW_ERROR_RAD):.3f} deg"
        )
    metrics = {
        "capture_count": len(samples),
        "target_xy_span_m": span,
        "target_xy_triangle_altitude_m": triangle_altitude,
        "right_joint_configuration_span_rad": joint_span,
        "xy_rms_m": xy_rms,
        "xy_max_m": xy_max,
        "z_max_m": z_max,
        "yaw_max_rad": yaw_max,
        "yaw_max_deg": math.degrees(yaw_max),
    }
    return (PASS_STATUS if not failures else REJECTED_STATUS), failures, metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--training-capture", action="append", required=True, type=Path
    )
    parser.add_argument(
        "--validation-capture", action="append", required=True, type=Path
    )
    parser.add_argument("--top-camera-info", required=True, type=Path)
    parser.add_argument("--wrist-camera-info", required=True, type=Path)
    parser.add_argument("--top-worktable", required=True, type=Path)
    parser.add_argument("--right-registration", required=True, type=Path)
    parser.add_argument("--right-wrist-eye-in-hand", required=True, type=Path)
    parser.add_argument("--urdf-xacro", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise RuntimeError(f"refusing to overwrite existing output: {args.output}")
    top_worktable = load_yaml(args.top_worktable)
    right_registration = load_yaml(args.right_registration)
    wrist_candidate = load_yaml(args.right_wrist_eye_in_hand)
    validate_sources(
        top_worktable,
        right_registration,
        wrist_candidate,
        args.top_camera_info,
        args.wrist_camera_info,
    )
    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro)],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    def evaluate(path: Path) -> dict:
        return evaluate_capture(
            path,
            load_yaml(path),
            top_worktable,
            right_registration,
            wrist_candidate,
            load_yaml(args.top_camera_info),
            load_yaml(args.wrist_camera_info),
            urdf_xml,
        )

    uncorrected_training = [evaluate(path) for path in args.training_capture]
    uncorrected_validation = [
        evaluate(path) for path in args.validation_capture
    ]
    correction = solve_gripper_translation_correction(
        uncorrected_training, right_registration, urdf_xml
    )
    training_samples = [
        apply_gripper_translation_correction(
            sample, correction, right_registration, urdf_xml
        )
        for sample in uncorrected_training
    ]
    validation_samples = [
        apply_gripper_translation_correction(
            sample, correction, right_registration, urdf_xml
        )
        for sample in uncorrected_validation
    ]
    status, failures, metrics = classify_calibrated_partitions(
        training_samples, validation_samples
    )
    corrected_wrist = corrected_wrist_transform_document(
        wrist_candidate, correction
    )
    document = {
        "schema_version": 1,
        "record_kind": "right_tabletop_target_validation",
        "status": status,
        "motion_authorized": False,
        "robot_target_available": False,
        "tabletop_object_validation_performed": True,
        "method": TRANSLATION_CORRECTION_METHOD,
        "sources": {
            "top_worktable": {
                "path": str(args.top_worktable.resolve()),
                "sha256": sha256_file(args.top_worktable),
            },
            "right_registration": {
                "path": str(args.right_registration.resolve()),
                "sha256": sha256_file(args.right_registration),
                "method": RIGHT_REGISTRATION_METHOD,
            },
            "right_wrist_eye_in_hand": {
                "path": str(args.right_wrist_eye_in_hand.resolve()),
                "sha256": sha256_file(args.right_wrist_eye_in_hand),
            },
            "top_camera_info_sha256": sha256_file(args.top_camera_info),
            "wrist_camera_info_sha256": sha256_file(args.wrist_camera_info),
            "urdf_xacro": {
                "path": str(args.urdf_xacro.resolve()),
                "sha256": sha256_file(args.urdf_xacro),
            },
        },
        "acceptance_thresholds": {
            "training_capture_count_min": MINIMUM_TRAINING_CAPTURES,
            "validation_capture_count_min": MINIMUM_VALIDATION_CAPTURES,
            "target_xy_span_m_min": MINIMUM_TARGET_XY_SPAN_M,
            "target_xy_triangle_altitude_m_min": (
                MINIMUM_TARGET_XY_TRIANGLE_ALTITUDE_M
            ),
            "right_joint_configuration_span_rad_min": (
                MINIMUM_RIGHT_JOINT_CONFIGURATION_SPAN_RAD
            ),
            "xy_rms_m_max": MAXIMUM_XY_RMS_M,
            "xy_error_m_max": MAXIMUM_XY_ERROR_M,
            "z_error_m_max": MAXIMUM_Z_ERROR_M,
            "yaw_error_deg_max": math.degrees(MAXIMUM_YAW_ERROR_RAD),
            "pnp_rms_px_max": MAXIMUM_PNP_RMS_PX,
            "image_border_px_min": MINIMUM_IMAGE_BORDER_PX,
            "within_capture_translation_span_m_max": (
                MAXIMUM_WITHIN_CAPTURE_TRANSLATION_SPAN_M
            ),
            "within_capture_rotation_span_deg_max": math.degrees(
                MAXIMUM_WITHIN_CAPTURE_ROTATION_SPAN_RAD
            ),
            "gripper_translation_correction_m_max": (
                MAXIMUM_GRIPPER_TRANSLATION_CORRECTION_M
            ),
        },
        "translation_correction": {
            "frame": "right_gripper_frame_link",
            "translation_only": True,
            "rotation_changed": False,
            "fit_partition": "training_only",
            "correction_m": [float(value) for value in correction],
            "magnitude_m": float(np.linalg.norm(correction)),
        },
        "corrected_right_wrist_transform": corrected_wrist,
        "metrics": metrics,
        "training_captures": training_samples,
        "validation_captures": validation_samples,
        "failure_reasons": failures,
        "required_next_gate": (
            "use only in 300 mm rigid-proxy MoveIt plan-only; this artifact "
            "never authorizes motion"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    print(
        f"{status} training={metrics['training']['capture_count']} "
        f"validation={metrics['validation']['capture_count']} "
        f"correction_mm={float(np.linalg.norm(correction)) * 1000.0:.3f} "
        f"validation_xy_rms_mm="
        f"{metrics['validation']['xy_rms_m'] * 1000.0:.3f} "
        f"validation_xy_max_mm="
        f"{metrics['validation']['xy_max_m'] * 1000.0:.3f} "
        f"validation_z_max_mm="
        f"{metrics['validation']['z_max_m'] * 1000.0:.3f} "
        f"validation_yaw_max_deg="
        f"{metrics['validation']['yaw_max_deg']:.3f} "
        f"output={args.output}"
    )
    return 0 if status == PASS_STATUS else 1


if __name__ == "__main__":
    raise SystemExit(main())
