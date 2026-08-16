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
MIN_VALIDATION_CAPTURES = 2
MIN_TRANSLATION_SPAN_M = 0.040
MIN_ROTATION_SPAN_RAD = math.radians(15.0)
TRAIN_RMS_TRANSLATION_M = 0.003
TRAIN_MAX_TRANSLATION_M = 0.005
TRAIN_RMS_ROTATION_RAD = math.radians(1.0)
TRAIN_MAX_ROTATION_RAD = math.radians(2.0)
VALIDATION_MAX_TRANSLATION_M = 0.005
VALIDATION_MAX_ROTATION_RAD = math.radians(2.0)
MAX_PNP_RMS_PX = 1.5
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


def classify(
    training: list[PoseObservation],
    validation: list[PoseObservation],
    training_summary: dict,
    validation_summary: dict | None,
) -> tuple[str, list[str]]:
    failures: list[str] = []
    if len(training) < MIN_TRAINING_CAPTURES:
        failures.append(
            f"training capture count {len(training)} < "
            f"{MIN_TRAINING_CAPTURES}"
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
) -> dict:
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    frames = session["frames"]
    arm = str(session.get("arm", "left"))
    if arm not in ARM_JOINT_NAMES_BY_SIDE:
        raise ValueError(f"unsupported session arm: {arm}")
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

    training = observations("training_captures")
    validation = observations("validation_captures")
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
    )
    return {
        "schema_version": 1,
        "status": status,
        "motion_authorized": False,
        "robot_target_available": False,
        "arm": arm,
        "method": "tcp_gridboard_robot_world_hand_eye",
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
            "training_capture_count_min": MIN_TRAINING_CAPTURES,
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
