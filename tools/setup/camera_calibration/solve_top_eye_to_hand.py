#!/usr/bin/env python3
"""Solve a fail-closed Top-camera eye-to-hand calibration.

The calibration target is rigidly held by the gripper, but its exact gripper
offset does not have to be measured.  For every capture:

    T_base_gripper * T_gripper_target
        = T_base_camera * T_camera_target

OpenCV robot-world/hand-eye calibration solves both constant transforms.  The
result never authorizes robot motion; independent validation captures are
required before the transform can be considered validated.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from solve_top_base_visual_registration import (  # noqa: E402
    load_yaml,
    urdf_fk,
    yaml_matrix,
)


MIN_TRAINING_CAPTURES = 8
MIN_RESIDENT_TORQUE_HOLD_TRAINING_CAPTURES = 6
MIN_VALIDATION_CAPTURES = 2
MIN_TRANSLATION_SPAN_M = 0.040
MIN_ROTATION_SPAN_RAD = math.radians(15.0)
# These gates reserve calibration error budget for downstream corner detection,
# arm repeatability, and jaw alignment in the 300 mm towel task.  They reject
# centimetre-scale outliers without treating physically unattainable zero error
# as the objective.  Held-out and tabletop metric validation remain mandatory.
TRAIN_RMS_TRANSLATION_M = 0.005
TRAIN_MAX_TRANSLATION_M = 0.008
TRAIN_RMS_ROTATION_RAD = math.radians(1.5)
TRAIN_MAX_ROTATION_RAD = math.radians(3.0)
VALIDATION_MAX_TRANSLATION_M = 0.008
VALIDATION_MAX_ROTATION_RAD = math.radians(3.0)
# The small 45 mm moving target is accepted as an observation up to 2.5 px.
# Task acceptance remains governed by held-out base-coordinate residuals and
# the independent tabletop metric gate, not by minimizing this image-only
# diagnostic.
MAX_PNP_RMS_PX = 2.5
MIN_IMAGE_BORDER_PX = 10.0
ARM_JOINT_NAMES_BY_SIDE = {
    side: tuple(
        f"{side}_{joint}_joint"
        for joint in (
            "base",
            "shoulder",
            "elbow",
            "wrist_flex",
            "wrist_roll",
        )
    )
    for side in ("left", "right")
}

# The first and last revolute-joint offsets are gauge-equivalent to the
# right-base yaw and the unknown gripper-to-target yaw respectively.  Fit only
# the three internal offsets that are identifiable from this target session.
RIGHT_IDENTIFIABLE_ZERO_INDICES = (1, 2, 3)
RIGHT_ZERO_BOUND_RAD = math.radians(20.0)
RIGHT_ZERO_PRIOR_RAD = math.radians(10.0)
RIGHT_MOUNT_TRANSLATION_PRIOR_M = 0.010
RIGHT_MOUNT_ROTATION_PRIOR_RAD = math.radians(5.0)
RIGHT_REGISTRATION_TRANSLATION_SCALE_M = 0.005
RIGHT_REGISTRATION_ROTATION_SCALE_RAD = math.radians(1.5)
RIGHT_MOUNT_PRIOR_XYZ_M = np.asarray(
    [0.0, -0.232064146, 0.0],
    dtype=np.float64,
)
READ_ONLY_CAPTURE_MODE = "stationary_read_only"
RESIDENT_TORQUE_HOLD_CAPTURE_MODE = "stationary_resident_torque_hold"
RESIDENT_STATUS_SERVICE = "/bimanual_stream_adapter/status"


@dataclass(frozen=True)
class TargetSpecification:
    dictionary_name: str
    markers_x: int
    markers_y: int
    marker_length_m: float
    marker_separation_m: float
    first_marker_id: int


@dataclass(frozen=True)
class PoseObservation:
    capture_id: str
    base_to_gripper: np.ndarray
    camera_to_target: np.ndarray
    pnp_rms_px: float
    image_border_px: float
    detected_marker_ids: tuple[int, ...]


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be 4x4")
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def matrix_document(matrix: np.ndarray) -> dict:
    values = np.asarray(matrix, dtype=np.float64)
    return {
        "rows": 4,
        "cols": 4,
        "data": [[float(value) for value in row] for row in values],
    }


def parse_target(document: dict) -> TargetSpecification:
    target = document["target"]
    specification = TargetSpecification(
        dictionary_name=str(target["dictionary"]),
        markers_x=int(target["markers_x"]),
        markers_y=int(target["markers_y"]),
        marker_length_m=float(target["marker_length_m"]),
        marker_separation_m=float(target["marker_separation_m"]),
        first_marker_id=int(target.get("first_marker_id", 0)),
    )
    if specification.markers_x < 2 or specification.markers_y < 2:
        raise ValueError("target must contain at least a 2x2 marker grid")
    if (
        specification.marker_length_m <= 0.0
        or specification.marker_separation_m <= 0.0
    ):
        raise ValueError("target dimensions must be positive")
    if not hasattr(cv2.aruco, specification.dictionary_name):
        raise ValueError(
            f"unknown ArUco dictionary: {specification.dictionary_name}"
        )
    return specification


def session_capture_mode(session: dict) -> str:
    mode = str(session.get("capture_mode", READ_ONLY_CAPTURE_MODE))
    captures = list(session.get("training_captures", [])) + list(
        session.get("validation_captures", [])
    )
    if mode == READ_ONLY_CAPTURE_MODE:
        if bool(session.get("source_motion_authorized", False)):
            raise RuntimeError(
                "read-only session unexpectedly records authorized motion"
            )
        for capture in captures:
            source_mode = capture.get(
                "source_capture_mode",
                READ_ONLY_CAPTURE_MODE,
            )
            if source_mode != READ_ONLY_CAPTURE_MODE or bool(
                capture.get("source_motion_authorized", False)
            ):
                raise RuntimeError(
                    "read-only session contains a non-read-only capture"
                )
        return mode
    if mode != RESIDENT_TORQUE_HOLD_CAPTURE_MODE:
        raise ValueError(f"unsupported capture mode: {mode}")
    if session.get("source_motion_authorized") is not True:
        raise RuntimeError(
            "resident torque-hold session lacks source-motion provenance"
        )
    for capture in captures:
        hold = capture.get("resident_torque_hold")
        if not isinstance(hold, dict):
            raise RuntimeError("resident capture lacks torque-hold evidence")
        owner = str(hold.get("owner", "")).strip()
        epoch = int(hold.get("arbiter_epoch", 0))
        if (
            capture.get("source_capture_mode") != mode
            or capture.get("source_motion_authorized") is not True
            or hold.get("status_service") != RESIDENT_STATUS_SERVICE
            or hold.get("torque_hold_active") is not True
            or not owner
            or str(hold.get("required_owner", "")).strip() != owner
            or epoch <= 0
            or int(hold.get("required_epoch", 0)) != epoch
            or float(hold.get("terminal_anchor_stamp", 0.0)) <= 0.0
        ):
            raise RuntimeError(
                "resident capture has inconsistent torque-hold evidence"
            )
    return mode


def minimum_training_captures_for_mode(capture_mode: str) -> int:
    if capture_mode == READ_ONLY_CAPTURE_MODE:
        return MIN_TRAINING_CAPTURES
    if capture_mode == RESIDENT_TORQUE_HOLD_CAPTURE_MODE:
        return MIN_RESIDENT_TORQUE_HOLD_TRAINING_CAPTURES
    raise ValueError(f"unsupported capture mode: {capture_mode}")


def make_board(
    specification: TargetSpecification,
) -> tuple[object, object, tuple[int, ...]]:
    dictionary_id = int(
        getattr(cv2.aruco, specification.dictionary_name)
    )
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.GridBoard_create(
        specification.markers_x,
        specification.markers_y,
        specification.marker_length_m,
        specification.marker_separation_m,
        dictionary,
        specification.first_marker_id,
    )
    expected_ids = tuple(int(value) for value in board.ids.reshape(-1))
    return dictionary, board, expected_ids


def detect_target_pose(
    image_path: Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    specification: TargetSpecification,
) -> tuple[np.ndarray, float, float, tuple[int, ...]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary, board, expected_ids = make_board(specification)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )
    if ids is None:
        raise RuntimeError(f"no ArUco marker detected: {image_path}")
    detected_ids = tuple(sorted(int(value) for value in ids.reshape(-1)))
    if detected_ids != tuple(sorted(expected_ids)):
        raise RuntimeError(
            f"expected marker IDs {expected_ids}, detected {detected_ids}: "
            f"{image_path}"
        )

    solved, rotation_vector, translation_vector = cv2.aruco.estimatePoseBoard(
        corners,
        ids,
        board,
        camera_matrix,
        distortion,
        None,
        None,
    )
    if int(solved) != len(expected_ids):
        raise RuntimeError(
            f"pose used {int(solved)} of {len(expected_ids)} markers: "
            f"{image_path}"
        )
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_to_target = make_transform(rotation, translation_vector)

    board_points_by_id = {
        int(marker_id): np.asarray(points, dtype=np.float64)
        for marker_id, points in zip(
            board.ids.reshape(-1),
            board.objPoints,
            strict=True,
        )
    }
    squared_errors: list[float] = []
    all_pixels: list[np.ndarray] = []
    for detected_corners, marker_id in zip(
        corners,
        ids.reshape(-1),
        strict=True,
    ):
        pixels = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
        projected, _ = cv2.projectPoints(
            board_points_by_id[int(marker_id)],
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        residual = projected.reshape(4, 2) - pixels
        squared_errors.extend(
            float(value) for value in np.sum(residual * residual, axis=1)
        )
        all_pixels.append(pixels)

    pnp_rms_px = math.sqrt(float(np.mean(squared_errors)))
    pixels = np.concatenate(all_pixels, axis=0)
    height, width = gray.shape
    image_border_px = float(
        min(
            pixels[:, 0].min(),
            pixels[:, 1].min(),
            (width - 1) - pixels[:, 0].max(),
            (height - 1) - pixels[:, 1].max(),
        )
    )
    return camera_to_target, pnp_rms_px, image_border_px, detected_ids


def average_target_poses(
    poses: list[np.ndarray],
) -> np.ndarray:
    if not poses:
        raise ValueError("at least one target pose is required")
    rotations = Rotation.from_matrix(
        np.asarray([pose[:3, :3] for pose in poses])
    )
    rotation = rotations.mean().as_matrix()
    translations = np.asarray([pose[:3, 3] for pose in poses])
    translation = np.median(translations, axis=0)
    return make_transform(rotation, translation)


def capture_observation(
    capture: dict,
    session_dir: Path,
    urdf_xml: str,
    robot_frame: str,
    gripper_frame: str,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    specification: TargetSpecification,
    joint_names: tuple[str, ...] = ARM_JOINT_NAMES_BY_SIDE["left"],
) -> PoseObservation:
    capture_id = str(capture["id"])
    measured = np.asarray(capture["measured_arm_rad"], dtype=np.float64)
    if measured.shape != (len(joint_names),) or not np.all(
        np.isfinite(measured)
    ):
        raise ValueError(f"{capture_id} has invalid measured_arm_rad")
    image_values = capture.get("image_files", [])
    if not image_values:
        raise ValueError(f"{capture_id} has no image_files")

    poses: list[np.ndarray] = []
    pnp_errors: list[float] = []
    borders: list[float] = []
    marker_ids: tuple[int, ...] | None = None
    for image_value in image_values:
        image_path = Path(str(image_value))
        if not image_path.is_absolute():
            image_path = session_dir / image_path
        pose, pnp_error, border, detected = detect_target_pose(
            image_path,
            camera_matrix,
            distortion,
            specification,
        )
        poses.append(pose)
        pnp_errors.append(pnp_error)
        borders.append(border)
        if marker_ids is None:
            marker_ids = detected
        elif detected != marker_ids:
            raise RuntimeError(f"{capture_id} marker IDs changed between frames")

    joint_positions = dict(zip(joint_names, measured, strict=True))
    base_to_gripper = urdf_fk(
        urdf_xml,
        robot_frame,
        gripper_frame,
        joint_positions,
    )
    assert marker_ids is not None
    return PoseObservation(
        capture_id=capture_id,
        base_to_gripper=base_to_gripper,
        camera_to_target=average_target_poses(poses),
        pnp_rms_px=float(max(pnp_errors)),
        image_border_px=float(min(borders)),
        detected_marker_ids=marker_ids,
    )


def solve_eye_to_hand(
    observations: list[PoseObservation],
) -> tuple[np.ndarray, np.ndarray]:
    if len(observations) < 3:
        raise ValueError("at least three observations are required")

    # OpenCV solves A X = Z B using:
    #   A = T_target_camera = inverse(T_camera_target)
    #   B = T_gripper_base = inverse(T_base_gripper)
    # and returns X = T_camera_base and Z = T_target_gripper.
    target_to_camera = [
        invert_transform(observation.camera_to_target)
        for observation in observations
    ]
    gripper_to_base = [
        invert_transform(observation.base_to_gripper)
        for observation in observations
    ]
    camera_to_base_rotation, camera_to_base_translation, (
        target_to_gripper_rotation
    ), target_to_gripper_translation = cv2.calibrateRobotWorldHandEye(
        [transform[:3, :3] for transform in target_to_camera],
        [transform[:3, 3].reshape(3, 1) for transform in target_to_camera],
        [transform[:3, :3] for transform in gripper_to_base],
        [transform[:3, 3].reshape(3, 1) for transform in gripper_to_base],
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )
    camera_to_base = make_transform(
        camera_to_base_rotation,
        camera_to_base_translation,
    )
    target_to_gripper = make_transform(
        target_to_gripper_rotation,
        target_to_gripper_translation,
    )
    return invert_transform(camera_to_base), invert_transform(
        target_to_gripper
    )


def transform_residual(
    observation: PoseObservation,
    base_to_camera: np.ndarray,
    gripper_to_target: np.ndarray,
) -> tuple[float, float]:
    from_robot = observation.base_to_gripper @ gripper_to_target
    from_camera = base_to_camera @ observation.camera_to_target
    error = invert_transform(from_robot) @ from_camera
    translation_m = float(np.linalg.norm(error[:3, 3]))
    rotation_rad = float(Rotation.from_matrix(error[:3, :3]).magnitude())
    return translation_m, rotation_rad


def maximum_pair_translation(observations: list[PoseObservation]) -> float:
    return max(
        float(
            np.linalg.norm(
                first.base_to_gripper[:3, 3]
                - second.base_to_gripper[:3, 3]
            )
        )
        for index, first in enumerate(observations)
        for second in observations[index + 1 :]
    )


def maximum_pair_rotation(observations: list[PoseObservation]) -> float:
    return max(
        float(
            Rotation.from_matrix(
                first.base_to_gripper[:3, :3].T
                @ second.base_to_gripper[:3, :3]
            ).magnitude()
        )
        for index, first in enumerate(observations)
        for second in observations[index + 1 :]
    )


def residual_summary(
    observations: list[PoseObservation],
    base_to_camera: np.ndarray,
    gripper_to_target: np.ndarray,
) -> dict:
    per_capture = []
    translations = []
    rotations = []
    for observation in observations:
        translation, rotation = transform_residual(
            observation,
            base_to_camera,
            gripper_to_target,
        )
        translations.append(translation)
        rotations.append(rotation)
        per_capture.append(
            {
                "id": observation.capture_id,
                "translation_residual_mm": translation * 1000.0,
                "rotation_residual_deg": math.degrees(rotation),
                "pnp_rms_px": observation.pnp_rms_px,
                "image_border_px": observation.image_border_px,
                "detected_marker_ids": list(
                    observation.detected_marker_ids
                ),
            }
        )
    return {
        "count": len(observations),
        "translation_rms_mm": math.sqrt(
            float(np.mean(np.square(translations)))
        )
        * 1000.0,
        "translation_max_mm": max(translations) * 1000.0,
        "rotation_rms_deg": math.degrees(
            math.sqrt(float(np.mean(np.square(rotations))))
        ),
        "rotation_max_deg": math.degrees(max(rotations)),
        "pnp_rms_px_max": max(
            observation.pnp_rms_px for observation in observations
        ),
        "image_border_px_min": min(
            observation.image_border_px for observation in observations
        ),
        "captures": per_capture,
    }


def observations_with_joint_zero_offsets(
    captures: list[dict],
    camera_observations: list[PoseObservation],
    urdf_xml: str,
    robot_frame: str,
    gripper_frame: str,
    joint_names: tuple[str, ...],
    joint_zero_offsets_rad: np.ndarray,
) -> list[PoseObservation]:
    offsets = np.asarray(joint_zero_offsets_rad, dtype=np.float64)
    if offsets.shape != (len(joint_names),) or not np.all(
        np.isfinite(offsets)
    ):
        raise ValueError("joint zero offsets have invalid dimensions")
    if len(captures) != len(camera_observations):
        raise ValueError("captures and camera observations must be paired")

    adjusted = []
    for capture, observation in zip(
        captures,
        camera_observations,
        strict=True,
    ):
        measured = np.asarray(
            capture["measured_arm_rad"],
            dtype=np.float64,
        )
        positions = measured + offsets
        base_to_gripper = urdf_fk(
            urdf_xml,
            robot_frame,
            gripper_frame,
            dict(zip(joint_names, positions, strict=True)),
        )
        adjusted.append(
            PoseObservation(
                capture_id=observation.capture_id,
                base_to_gripper=base_to_gripper,
                camera_to_target=observation.camera_to_target,
                pnp_rms_px=observation.pnp_rms_px,
                image_border_px=observation.image_border_px,
                detected_marker_ids=observation.detected_marker_ids,
            )
        )
    return adjusted


def unpack_right_registration(
    values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = np.asarray(values, dtype=np.float64)
    expected = 12 + len(RIGHT_IDENTIFIABLE_ZERO_INDICES)
    if values.shape != (expected,):
        raise ValueError("right registration vector has invalid dimensions")
    workcell_to_right_base = make_transform(
        Rotation.from_rotvec(values[0:3]).as_matrix(),
        values[3:6],
    )
    gripper_to_target = make_transform(
        Rotation.from_rotvec(values[6:9]).as_matrix(),
        values[9:12],
    )
    offsets = np.zeros(len(ARM_JOINT_NAMES_BY_SIDE["right"]))
    offsets[list(RIGHT_IDENTIFIABLE_ZERO_INDICES)] = values[12:]
    return workcell_to_right_base, gripper_to_target, offsets


def fit_right_joint_zero_registration(
    training_captures: list[dict],
    training_camera_observations: list[PoseObservation],
    urdf_xml: str,
    robot_frame: str,
    gripper_frame: str,
    workcell_to_camera: np.ndarray,
    workcell_to_right_base_prior: np.ndarray,
    minimum_training_captures: int = MIN_TRAINING_CAPTURES,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, object]:
    if len(training_captures) < minimum_training_captures:
        raise ValueError("right registration requires the training gate")
    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    nominal_right_to_camera, nominal_gripper_to_target = solve_eye_to_hand(
        training_camera_observations
    )
    initial_mount = workcell_to_camera @ invert_transform(
        nominal_right_to_camera
    )
    initial = np.concatenate(
        (
            Rotation.from_matrix(initial_mount[:3, :3]).as_rotvec(),
            initial_mount[:3, 3],
            Rotation.from_matrix(
                nominal_gripper_to_target[:3, :3]
            ).as_rotvec(),
            nominal_gripper_to_target[:3, 3],
            np.zeros(len(RIGHT_IDENTIFIABLE_ZERO_INDICES)),
        )
    )

    lower = np.full(initial.shape, -np.inf)
    upper = np.full(initial.shape, np.inf)
    lower[12:] = -RIGHT_ZERO_BOUND_RAD
    upper[12:] = RIGHT_ZERO_BOUND_RAD

    def residual(values: np.ndarray) -> np.ndarray:
        mount, gripper_to_target, offsets = unpack_right_registration(
            values
        )
        adjusted = observations_with_joint_zero_offsets(
            training_captures,
            training_camera_observations,
            urdf_xml,
            robot_frame,
            gripper_frame,
            joint_names,
            offsets,
        )
        values_out: list[float] = []
        for observation in adjusted:
            predicted = mount @ observation.base_to_gripper @ gripper_to_target
            measured = workcell_to_camera @ observation.camera_to_target
            error = invert_transform(predicted) @ measured
            values_out.extend(
                (
                    error[:3, 3]
                    / RIGHT_REGISTRATION_TRANSLATION_SCALE_M
                ).tolist()
            )
            values_out.extend(
                (
                    Rotation.from_matrix(error[:3, :3]).as_rotvec()
                    / RIGHT_REGISTRATION_ROTATION_SCALE_RAD
                ).tolist()
            )

        mount_error = invert_transform(workcell_to_right_base_prior) @ mount
        values_out.extend(
            (
                mount_error[:3, 3]
                / RIGHT_MOUNT_TRANSLATION_PRIOR_M
            ).tolist()
        )
        values_out.extend(
            (
                Rotation.from_matrix(mount_error[:3, :3]).as_rotvec()
                / RIGHT_MOUNT_ROTATION_PRIOR_RAD
            ).tolist()
        )
        values_out.extend(
            (
                offsets[list(RIGHT_IDENTIFIABLE_ZERO_INDICES)]
                / RIGHT_ZERO_PRIOR_RAD
            ).tolist()
        )
        return np.asarray(values_out, dtype=np.float64)

    fit = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=1000,
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    mount, gripper_to_target, offsets = unpack_right_registration(fit.x)
    return mount, gripper_to_target, offsets, fit


def classify(
    training: list[PoseObservation],
    validation: list[PoseObservation],
    training_summary: dict,
    validation_summary: dict | None,
    minimum_training_captures: int = MIN_TRAINING_CAPTURES,
) -> tuple[str, list[str]]:
    failures: list[str] = []
    if len(training) < minimum_training_captures:
        failures.append(
            f"training capture count {len(training)} < "
            f"{minimum_training_captures}"
        )
    if len(training) >= 2:
        if maximum_pair_translation(training) < MIN_TRANSLATION_SPAN_M:
            failures.append("training translation span is too small")
        if maximum_pair_rotation(training) < MIN_ROTATION_SPAN_RAD:
            failures.append("training rotation span is too small")
    if training_summary["translation_rms_mm"] > (
        TRAIN_RMS_TRANSLATION_M * 1000.0
    ):
        failures.append("training translation RMS exceeds threshold")
    if training_summary["translation_max_mm"] > (
        TRAIN_MAX_TRANSLATION_M * 1000.0
    ):
        failures.append("training translation max exceeds threshold")
    if training_summary["rotation_rms_deg"] > math.degrees(
        TRAIN_RMS_ROTATION_RAD
    ):
        failures.append("training rotation RMS exceeds threshold")
    if training_summary["rotation_max_deg"] > math.degrees(
        TRAIN_MAX_ROTATION_RAD
    ):
        failures.append("training rotation max exceeds threshold")
    if training_summary["pnp_rms_px_max"] > MAX_PNP_RMS_PX:
        failures.append("training PnP reprojection error exceeds threshold")
    if training_summary["image_border_px_min"] < MIN_IMAGE_BORDER_PX:
        failures.append("training target is too close to an image border")

    if failures:
        return "REJECTED_EYE_TO_HAND_CALIBRATION", failures
    if len(validation) < MIN_VALIDATION_CAPTURES or validation_summary is None:
        return (
            "PROVISIONAL_EYE_TO_HAND_REQUIRES_INDEPENDENT_VALIDATION",
            [
                f"validation capture count {len(validation)} < "
                f"{MIN_VALIDATION_CAPTURES}"
            ],
        )
    if validation_summary["translation_max_mm"] > (
        VALIDATION_MAX_TRANSLATION_M * 1000.0
    ):
        failures.append("validation translation max exceeds threshold")
    if validation_summary["rotation_max_deg"] > math.degrees(
        VALIDATION_MAX_ROTATION_RAD
    ):
        failures.append("validation rotation max exceeds threshold")
    if validation_summary["pnp_rms_px_max"] > MAX_PNP_RMS_PX:
        failures.append("validation PnP reprojection error exceeds threshold")
    if validation_summary["image_border_px_min"] < MIN_IMAGE_BORDER_PX:
        failures.append("validation target is too close to an image border")
    if failures:
        return "REJECTED_EYE_TO_HAND_VALIDATION", failures
    return "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED", []


def solve_document(
    session: dict,
    session_path: Path,
    camera_info: dict,
    urdf_xml: str,
    workcell_reference: dict | None = None,
    workcell_to_right_base_prior: np.ndarray | None = None,
) -> dict:
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    capture_mode = session_capture_mode(session)
    frames = session["frames"]
    arm = str(session.get("arm", "left"))
    if arm not in ARM_JOINT_NAMES_BY_SIDE:
        raise ValueError(f"unsupported session arm: {arm}")
    if capture_mode == RESIDENT_TORQUE_HOLD_CAPTURE_MODE and (
        arm != "right" or workcell_reference is None
    ):
        raise ValueError(
            "resident torque-hold sessions require the workcell-anchored "
            "right-arm registration path"
        )
    minimum_training_captures = minimum_training_captures_for_mode(
        capture_mode
    )
    expected_frames = (f"{arm}_base_link", f"{arm}_gripper_frame_link")
    if (str(frames["robot"]), str(frames["gripper"])) != expected_frames:
        raise ValueError(
            "session arm/frame mismatch: "
            f"arm={arm} robot={frames['robot']} gripper={frames['gripper']}"
        )
    joint_names = ARM_JOINT_NAMES_BY_SIDE[arm]
    specification = parse_target(session)
    camera_matrix = yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = yaml_matrix(
        camera_info,
        "distortion_coefficients",
        1,
        5,
    ).reshape(-1)

    def observations(key: str) -> list[PoseObservation]:
        return [
            capture_observation(
                capture,
                session_path.resolve().parent,
                urdf_xml,
                str(frames["robot"]),
                str(frames["gripper"]),
                camera_matrix,
                distortion,
                specification,
                joint_names,
            )
            for capture in session.get(key, [])
        ]

    training_captures = list(session.get("training_captures", []))
    validation_captures = list(session.get("validation_captures", []))
    training = observations("training_captures")
    validation = observations("validation_captures")
    registration: dict | None = None
    method = "tcp_gridboard_robot_world_hand_eye"

    if workcell_reference is not None:
        if arm != "right":
            raise ValueError(
                "a workcell reference is only valid for a right-arm session"
            )
        if workcell_reference.get("arm") != "left":
            raise ValueError("workcell reference must be left-arm eye-to-hand")
        if workcell_reference.get("status") != (
            "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        ):
            raise RuntimeError("workcell reference is not independently validated")
        if bool(workcell_reference.get("motion_authorized", False)):
            raise RuntimeError("workcell reference unexpectedly authorizes motion")
        workcell_to_camera = yaml_matrix(
            workcell_reference,
            "base_to_camera",
            4,
            4,
        )
        if workcell_to_right_base_prior is None:
            workcell_to_right_base_prior = make_transform(
                np.eye(3),
                RIGHT_MOUNT_PRIOR_XYZ_M,
            )
        (
            workcell_to_right_base,
            gripper_to_target,
            joint_zero_offsets,
            fit,
        ) = fit_right_joint_zero_registration(
            training_captures,
            training,
            urdf_xml,
            str(frames["robot"]),
            str(frames["gripper"]),
            workcell_to_camera,
            workcell_to_right_base_prior,
            minimum_training_captures,
        )
        training = observations_with_joint_zero_offsets(
            training_captures,
            training,
            urdf_xml,
            str(frames["robot"]),
            str(frames["gripper"]),
            joint_names,
            joint_zero_offsets,
        )
        validation = observations_with_joint_zero_offsets(
            validation_captures,
            validation,
            urdf_xml,
            str(frames["robot"]),
            str(frames["gripper"]),
            joint_names,
            joint_zero_offsets,
        )
        base_to_camera = invert_transform(workcell_to_right_base) @ (
            workcell_to_camera
        )
        method = "workcell_anchored_right_joint_zero_bundle_adjustment"
        registration = {
            "workcell_frame": str(workcell_reference["frames"]["robot"]),
            "workcell_to_right_base": matrix_document(
                workcell_to_right_base
            ),
            "workcell_to_right_base_prior": matrix_document(
                workcell_to_right_base_prior
            ),
            "joint_zero_offsets_rad": {
                name: float(value)
                for name, value in zip(
                    joint_names,
                    joint_zero_offsets,
                    strict=True,
                )
            },
            "identifiable_joint_names": [
                joint_names[index]
                for index in RIGHT_IDENTIFIABLE_ZERO_INDICES
            ],
            "gauge_fixed_joint_names": [
                joint_names[0],
                joint_names[-1],
            ],
            "training_only_fit": True,
            "validation_used_in_fit": False,
            "optimizer": {
                "success": bool(fit.success),
                "message": str(fit.message),
                "nfev": int(fit.nfev),
                "cost": float(fit.cost),
            },
            "priors": {
                "joint_zero_deg": math.degrees(RIGHT_ZERO_PRIOR_RAD),
                "joint_zero_bound_deg": math.degrees(
                    RIGHT_ZERO_BOUND_RAD
                ),
                "mount_translation_mm": (
                    RIGHT_MOUNT_TRANSLATION_PRIOR_M * 1000.0
                ),
                "mount_rotation_deg": math.degrees(
                    RIGHT_MOUNT_ROTATION_PRIOR_RAD
                ),
            },
        }
    else:
        base_to_camera, gripper_to_target = solve_eye_to_hand(training)
    training_summary = residual_summary(
        training,
        base_to_camera,
        gripper_to_target,
    )
    validation_summary = (
        residual_summary(validation, base_to_camera, gripper_to_target)
        if validation
        else None
    )
    status, failures = classify(
        training,
        validation,
        training_summary,
        validation_summary,
        minimum_training_captures,
    )
    result = {
        "schema_version": 1,
        "status": status,
        "motion_authorized": False,
        "robot_target_available": False,
        "arm": arm,
        "capture_mode": capture_mode,
        "source_motion_authorized": bool(
            session.get("source_motion_authorized", False)
        ),
        "method": method,
        "frames": dict(frames),
        "target": {
            "dictionary": specification.dictionary_name,
            "markers_x": specification.markers_x,
            "markers_y": specification.markers_y,
            "marker_length_m": specification.marker_length_m,
            "marker_separation_m": specification.marker_separation_m,
            "first_marker_id": specification.first_marker_id,
        },
        "base_to_camera": matrix_document(base_to_camera),
        "gripper_to_target": matrix_document(gripper_to_target),
        "geometry": {
            "training_translation_span_mm": (
                maximum_pair_translation(training) * 1000.0
                if len(training) >= 2
                else 0.0
            ),
            "training_rotation_span_deg": (
                math.degrees(maximum_pair_rotation(training))
                if len(training) >= 2
                else 0.0
            ),
        },
        "training_fit": training_summary,
        "validation_fit": validation_summary,
        "acceptance_thresholds": {
            "training_capture_count_min": minimum_training_captures,
            "validation_capture_count_min": MIN_VALIDATION_CAPTURES,
            "training_translation_span_mm_min": (
                MIN_TRANSLATION_SPAN_M * 1000.0
            ),
            "training_rotation_span_deg_min": math.degrees(
                MIN_ROTATION_SPAN_RAD
            ),
            "training_translation_rms_mm_max": (
                TRAIN_RMS_TRANSLATION_M * 1000.0
            ),
            "training_translation_max_mm_max": (
                TRAIN_MAX_TRANSLATION_M * 1000.0
            ),
            "training_rotation_rms_deg_max": math.degrees(
                TRAIN_RMS_ROTATION_RAD
            ),
            "training_rotation_max_deg_max": math.degrees(
                TRAIN_MAX_ROTATION_RAD
            ),
            "validation_translation_max_mm_max": (
                VALIDATION_MAX_TRANSLATION_M * 1000.0
            ),
            "validation_rotation_max_deg_max": math.degrees(
                VALIDATION_MAX_ROTATION_RAD
            ),
            "pnp_rms_px_max": MAX_PNP_RMS_PX,
            "image_border_px_min": MIN_IMAGE_BORDER_PX,
        },
        "failure_reasons": failures,
        "required_next_gate": (
            "validate the transform with independent held-out captures and "
            "then verify tabletop x/y/yaw against measured object poses; "
            "never use this file alone as motion authorization"
        ),
    }
    if registration is not None:
        result["right_kinematic_registration"] = registration
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=Path(
            "ros2_ws/src/manipulation_camera_manager/config/"
            "top_camera_info.yaml"
        ),
    )
    parser.add_argument(
        "--urdf-xacro",
        type=Path,
        default=Path(
            "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/top_eye_to_hand_candidate.yaml"),
    )
    parser.add_argument(
        "--workcell-reference",
        type=Path,
        help=(
            "independently validated left-arm eye-to-hand result; enables "
            "training-only constrained right-arm joint-zero registration"
        ),
    )
    parser.add_argument(
        "--right-mount-prior-xyz-m",
        nargs=3,
        type=float,
        default=RIGHT_MOUNT_PRIOR_XYZ_M.tolist(),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--right-mount-prior-rpy-rad",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        metavar=("R", "P", "Y"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = load_yaml(args.session)
    arm = str(session.get("arm", "left"))
    if arm not in ARM_JOINT_NAMES_BY_SIDE:
        raise ValueError(f"unsupported session arm: {arm}")
    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro), f"arm_slot:={arm}"],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    result = solve_document(
        session,
        args.session,
        load_yaml(args.camera_info),
        urdf_xml,
        (
            load_yaml(args.workcell_reference)
            if args.workcell_reference is not None
            else None
        ),
        make_transform(
            Rotation.from_euler(
                "xyz",
                args.right_mount_prior_rpy_rad,
            ).as_matrix(),
            np.asarray(args.right_mount_prior_xyz_m, dtype=np.float64),
        ),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, sort_keys=False),
        encoding="utf-8",
    )
    print(
        f"TOP_EYE_TO_HAND_{result['status']} "
        f"train_rms_mm="
        f"{result['training_fit']['translation_rms_mm']:.3f} "
        f"train_max_mm="
        f"{result['training_fit']['translation_max_mm']:.3f} "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
