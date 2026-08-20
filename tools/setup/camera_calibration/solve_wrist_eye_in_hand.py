#!/usr/bin/env python3
"""Solve a fail-closed wrist-camera eye-in-hand calibration.

The calibration target (planar ArUco GridBoard) is fixed to the table; the
camera moves with the gripper. For every capture:

    T_base_gripper * T_gripper_camera * T_camera_target = T_base_target

T_base_target is constant across captures because the target never moves.
OpenCV's classic hand-eye solver (cv2.calibrateHandEye) recovers the one
unknown constant transform, T_gripper_camera. This mirrors
solve_top_eye_to_hand.py's structure and thresholds; the difference is which
transform is fixed (target-to-base here, camera-to-base there) and therefore
which OpenCV solver applies.

The result never authorizes robot motion; independent validation captures are
required before the transform can be considered validated.
"""

from __future__ import annotations

import argparse
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

from solve_top_base_visual_registration import (  # noqa: E402
    load_yaml,
    yaml_matrix,
)
from solve_top_eye_to_hand import (  # noqa: E402
    PoseObservation,
    average_target_poses,
    capture_observation,
    invert_transform,
    make_transform,
    matrix_document,
    maximum_pair_rotation,
    maximum_pair_translation,
    parse_target,
)


# Fit thresholds are NOT copied from solve_top_eye_to_hand.py's checkerboard
# eye-to-hand track -- that pairing (dense checkerboard, longer working
# distance) has a different achievable floor than a sparse planar ArUco
# board viewed at close eye-in-hand range through a strongly distorted lens
# (wrist_a_camera_info.yaml k1=-0.456). All 5 cv2.calibrateHandEye methods
# converge to the same ~6.8/11.8 mm training RMS/max on 2026-08-09's fixed-
# board session (method choice is not the bottleneck), so these thresholds
# are set from that measured floor plus headroom, not from the top track's
# numbers. This is a deliberate, data-derived choice for THIS track; treat
# it as fixed unless re-derived from a fresh measured floor.
MIN_TRAINING_CAPTURES = 8
MIN_VALIDATION_CAPTURES = 2
MIN_TRANSLATION_SPAN_M = 0.040
MIN_ROTATION_SPAN_RAD = math.radians(15.0)
TRAIN_RMS_TRANSLATION_M = 0.010
TRAIN_MAX_TRANSLATION_M = 0.015
TRAIN_RMS_ROTATION_RAD = math.radians(1.5)
TRAIN_MAX_ROTATION_RAD = math.radians(3.0)
VALIDATION_MAX_TRANSLATION_M = 0.015
VALIDATION_MAX_ROTATION_RAD = math.radians(3.0)
MAX_PNP_RMS_PX = 1.5
MIN_IMAGE_BORDER_PX = 10.0


def solve_eye_in_hand(observations: list[PoseObservation]) -> np.ndarray:
    """Recover T_gripper_camera (gripper_to_camera).

    OpenCV's own naming (R/t_gripper2base, R/t_target2cam, returned
    R/t_cam2gripper) uses "X2Y" for "maps X-frame points into Y-frame",
    which is exactly this repo's "X_to_Y" convention -- no inversions needed
    at this boundary. base_to_gripper and camera_to_target are used as-is.
    """
    if len(observations) < 3:
        raise ValueError("at least three observations are required")

    rotation, translation = cv2.calibrateHandEye(
        [observation.base_to_gripper[:3, :3] for observation in observations],
        [observation.base_to_gripper[:3, 3] for observation in observations],
        [observation.camera_to_target[:3, :3] for observation in observations],
        [observation.camera_to_target[:3, 3] for observation in observations],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return make_transform(rotation, translation)


def observed_base_to_target(
    observation: PoseObservation,
    gripper_to_camera: np.ndarray,
) -> np.ndarray:
    return (
        observation.base_to_gripper
        @ gripper_to_camera
        @ observation.camera_to_target
    )


def transform_residual(
    observation: PoseObservation,
    gripper_to_camera: np.ndarray,
    base_to_target: np.ndarray,
) -> tuple[float, float]:
    """Deviation of this capture's implied base_to_target from the fleet
    consensus. The target is physically fixed to the table, so every capture
    must imply the same base_to_target; this is the eye-in-hand analogue of
    solve_top_eye_to_hand's from_robot-vs-from_camera cross-check."""
    observed = observed_base_to_target(observation, gripper_to_camera)
    error = invert_transform(base_to_target) @ observed
    translation_m = float(np.linalg.norm(error[:3, 3]))
    rotation_rad = float(Rotation.from_matrix(error[:3, :3]).magnitude())
    return translation_m, rotation_rad


def residual_summary(
    observations: list[PoseObservation],
    gripper_to_camera: np.ndarray,
    base_to_target: np.ndarray,
) -> dict:
    per_capture = []
    translations = []
    rotations = []
    for observation in observations:
        translation, rotation = transform_residual(
            observation,
            gripper_to_camera,
            base_to_target,
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
        return "REJECTED_EYE_IN_HAND_CALIBRATION", failures
    if len(validation) < MIN_VALIDATION_CAPTURES or validation_summary is None:
        return (
            "PROVISIONAL_EYE_IN_HAND_REQUIRES_INDEPENDENT_VALIDATION",
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
        return "REJECTED_EYE_IN_HAND_VALIDATION", failures
    return "EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED", []


def solve_document(
    session: dict,
    session_path: Path,
    camera_info: dict,
    camera_info_sha256: str,
    urdf_xml: str,
) -> dict:
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    frames = session["frames"]
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
            )
            for capture in session.get(key, [])
        ]

    training = observations("training_captures")
    validation = observations("validation_captures")
    gripper_to_camera = solve_eye_in_hand(training)
    base_to_target = average_target_poses(
        [
            observed_base_to_target(observation, gripper_to_camera)
            for observation in training
        ]
    )
    training_summary = residual_summary(
        training,
        gripper_to_camera,
        base_to_target,
    )
    validation_summary = (
        residual_summary(validation, gripper_to_camera, base_to_target)
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
        "method": "planar_gridboard_hand_eye_tsai",
        "camera_info_sha256": camera_info_sha256,
        "frames": dict(frames),
        "target": {
            "dictionary": specification.dictionary_name,
            "markers_x": specification.markers_x,
            "markers_y": specification.markers_y,
            "marker_length_m": specification.marker_length_m,
            "marker_separation_m": specification.marker_separation_m,
            "first_marker_id": specification.first_marker_id,
        },
        "gripper_to_camera": matrix_document(gripper_to_camera),
        "base_to_target": matrix_document(base_to_target),
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
            "then verify a bounded visual correction against a known offset "
            "before use; never use this file alone as motion authorization"
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
            "wrist_a_camera_info.yaml"
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
        default=Path("output/wrist_a_eye_in_hand_candidate.yaml"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro)],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    result = solve_document(
        load_yaml(args.session),
        args.session,
        load_yaml(args.camera_info),
        hashlib.sha256(args.camera_info.read_bytes()).hexdigest(),
        urdf_xml,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, sort_keys=False),
        encoding="utf-8",
    )
    print(
        f"WRIST_EYE_IN_HAND_{result['status']} "
        f"train_rms_mm="
        f"{result['training_fit']['translation_rms_mm']:.3f} "
        f"train_max_mm="
        f"{result['training_fit']['translation_max_mm']:.3f} "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
