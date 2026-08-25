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
    ARM_JOINT_NAMES_BY_SIDE,
    PoseObservation,
    average_target_poses,
    capture_observation,
    invert_transform,
    make_transform,
    matrix_document,
    maximum_pair_rotation,
    maximum_pair_translation,
    observations_with_joint_zero_offsets,
    parse_target,
    urdf_fk,
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
MIN_RIGHT_RESIDENT_TRAINING_CAPTURES = 6
MIN_VALIDATION_CAPTURES = 2
MIN_TRANSLATION_SPAN_M = 0.040
MIN_ROTATION_SPAN_RAD = math.radians(15.0)
TRAIN_RMS_TRANSLATION_M = 0.010
TRAIN_MAX_TRANSLATION_M = 0.015
TRAIN_RMS_ROTATION_RAD = math.radians(1.5)
TRAIN_MAX_ROTATION_RAD = math.radians(3.0)
VALIDATION_MAX_TRANSLATION_M = 0.015
VALIDATION_MAX_ROTATION_RAD = math.radians(3.0)
# PnP is a gross image/board quality gate, not the final coordinate-accuracy
# gate.  At the 640x480 wrist resolution (fx ~= 560 px), the measured physical
# GridBoard reaches about 2.1 px RMS under ordinary lighting.  A 2.5 px bound
# keeps those usable observations while the independent 15 mm / 3 degree
# held-out transform gates still reject an inaccurate hand-eye result.
MAX_PNP_RMS_PX = 2.5
# Every marker must still be detected in every accepted source frame.  This
# border value is only a gross clipping guard; the measured right-wrist set
# retains stable 1.1 px PnP at a 3.39 px board margin.
MIN_IMAGE_BORDER_PX = 3.0
RIGHT_REGISTRATION_METHOD = (
    "workcell_anchored_right_joint_zero_bundle_adjustment"
)
VALIDATED_EYE_TO_HAND_STATUS = (
    "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
)
RIGHT_CAPTURE_MODE = "stationary_resident_torque_hold"
RESIDENT_STATUS_SERVICE = "/bimanual_stream_adapter/status"


def validate_session_capture_provenance(session: dict, arm: str) -> None:
    """Reject right-wrist sessions that were measured without loaded hold."""
    captures = list(session.get("training_captures", [])) + list(
        session.get("validation_captures", [])
    )
    if arm != "right":
        return
    if session.get("capture_mode") != RIGHT_CAPTURE_MODE:
        raise RuntimeError(
            "right wrist session requires stationary resident torque hold"
        )
    if session.get("source_motion_authorized") is not True:
        raise RuntimeError(
            "right wrist session lacks armed source-capture evidence"
        )
    for capture in captures:
        capture_id = str(capture.get("id", "missing"))
        hold = capture.get("resident_torque_hold")
        if (
            capture.get("source_capture_mode") != RIGHT_CAPTURE_MODE
            or capture.get("source_motion_authorized") is not True
            or not isinstance(hold, dict)
            or hold.get("status_service") != RESIDENT_STATUS_SERVICE
            or hold.get("torque_hold_active") is not True
            or not str(hold.get("owner", "")).strip()
            or int(hold.get("arbiter_epoch", 0)) <= 0
            or hold.get("owner") != hold.get("required_owner")
            or int(hold.get("arbiter_epoch", 0))
            != int(hold.get("required_epoch", 0))
            or float(hold.get("terminal_anchor_stamp", 0.0)) <= 0.0
        ):
            raise RuntimeError(
                f"right wrist capture {capture_id} lacks consistent "
                "resident torque-hold provenance"
            )


def validated_right_joint_zero_offsets(
    candidate: dict,
) -> tuple[np.ndarray, dict]:
    """Load only the independently validated R0-C right-arm registration.

    The right SO-101 has a measured joint-zero mismatch relative to the
    nominal URDF.  Eye-in-hand FK must use the training-only registration;
    accepting an all-capture preview fit here would leak validation data into
    the wrist-camera solve and understate its held-out error.
    """
    if candidate.get("status") != VALIDATED_EYE_TO_HAND_STATUS:
        raise RuntimeError("right registration is not independently validated")
    if candidate.get("arm") != "right":
        raise RuntimeError("right registration candidate must declare arm=right")
    if candidate.get("method") != RIGHT_REGISTRATION_METHOD:
        raise RuntimeError("unsupported right registration method")
    if bool(candidate.get("motion_authorized", False)):
        raise RuntimeError("right registration unexpectedly authorizes motion")

    registration = candidate.get("right_kinematic_registration")
    if not isinstance(registration, dict):
        raise RuntimeError("right registration payload is missing")
    if registration.get("training_only_fit") is not True:
        raise RuntimeError("right registration was not fit on training only")
    if registration.get("validation_used_in_fit") is not False:
        raise RuntimeError("right registration leaked validation into its fit")

    joint_names = ARM_JOINT_NAMES_BY_SIDE["right"]
    values = registration.get("joint_zero_offsets_rad")
    if not isinstance(values, dict) or set(values) != set(joint_names):
        raise RuntimeError("right registration joint-zero set is incomplete")
    offsets = np.asarray([values[name] for name in joint_names], dtype=np.float64)
    if offsets.shape != (len(joint_names),) or not np.all(np.isfinite(offsets)):
        raise RuntimeError("right registration joint-zero values are invalid")
    return offsets, registration


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
    right_registration: dict | None = None,
    right_registration_sha256: str | None = None,
) -> dict:
    if bool(session.get("motion_authorized", False)):
        raise RuntimeError("input session must remain motion_authorized=false")
    frames = session["frames"]
    arm = str(session.get("arm", "left"))
    if arm not in ARM_JOINT_NAMES_BY_SIDE:
        raise RuntimeError(f"unsupported session arm: {arm}")
    validate_session_capture_provenance(session, arm)
    expected_frames = {
        "robot": f"{arm}_base_link",
        "gripper": f"{arm}_gripper_frame_link",
        "camera": f"{arm}_wrist_camera_optical_frame",
        "target": "wrist_planar_aruco_gridboard",
    }
    if dict(frames) != expected_frames:
        raise RuntimeError(
            f"session frames do not match {arm} wrist contract: {frames}"
        )
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
                ARM_JOINT_NAMES_BY_SIDE[arm],
            )
            for capture in session.get(key, [])
        ]

    training_captures = list(session.get("training_captures", []))
    validation_captures = list(session.get("validation_captures", []))
    training = observations("training_captures")
    validation = observations("validation_captures")
    registration_document: dict | None = None
    method = "planar_gridboard_hand_eye_tsai"
    if arm == "right":
        if right_registration is None:
            raise RuntimeError(
                "right wrist solve requires the independently validated "
                "right-arm registration"
            )
        offsets, registration = validated_right_joint_zero_offsets(
            right_registration
        )
        joint_names = ARM_JOINT_NAMES_BY_SIDE[arm]
        training = observations_with_joint_zero_offsets(
            training_captures,
            training,
            urdf_xml,
            str(frames["robot"]),
            str(frames["gripper"]),
            joint_names,
            offsets,
        )
        validation = observations_with_joint_zero_offsets(
            validation_captures,
            validation,
            urdf_xml,
            str(frames["robot"]),
            str(frames["gripper"]),
            joint_names,
            offsets,
        )
        method = "planar_gridboard_hand_eye_tsai_registered_right_fk"
        registration_document = {
            "source_sha256": right_registration_sha256,
            "source_status": str(right_registration["status"]),
            "source_method": str(right_registration["method"]),
            "joint_zero_offsets_rad": {
                name: float(value)
                for name, value in zip(joint_names, offsets, strict=True)
            },
            "training_only_fit": bool(registration["training_only_fit"]),
            "validation_used_in_fit": bool(
                registration["validation_used_in_fit"]
            ),
        }
    elif right_registration is not None:
        raise RuntimeError("right registration cannot be applied to the left arm")

    gripper_to_camera = solve_eye_in_hand(training)
    gripper_link = f"{arm}_gripper_link"
    mount_center_link = f"{arm}_wrist_camera_mount_center_link"
    gripper_link_to_gripper_frame = urdf_fk(
        urdf_xml,
        gripper_link,
        str(frames["gripper"]),
        {},
    )
    gripper_link_to_mount_center = urdf_fk(
        urdf_xml,
        gripper_link,
        mount_center_link,
        {},
    )
    mount_center_to_camera = (
        invert_transform(gripper_link_to_mount_center)
        @ gripper_link_to_gripper_frame
        @ gripper_to_camera
    )
    mount_center_rpy = Rotation.from_matrix(
        mount_center_to_camera[:3, :3]
    ).as_euler("xyz")
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
    minimum_training_captures = (
        MIN_RIGHT_RESIDENT_TRAINING_CAPTURES
        if arm == "right"
        else MIN_TRAINING_CAPTURES
    )
    status, failures = classify(
        training,
        validation,
        training_summary,
        validation_summary,
        minimum_training_captures,
    )
    document = {
        "schema_version": 1,
        "status": status,
        "motion_authorized": False,
        "robot_target_available": False,
        "arm": arm,
        "method": method,
        "camera_info_sha256": camera_info_sha256,
        "session_sha256": hashlib.sha256(session_path.read_bytes()).hexdigest(),
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
        "mount_center_to_camera": matrix_document(mount_center_to_camera),
        "xacro_defaults": {
            "wrist_camera_xyz": [
                float(value) for value in mount_center_to_camera[:3, 3]
            ],
            "wrist_camera_optical_rpy": [
                float(value) for value in mount_center_rpy
            ],
            "parent_frame": mount_center_link,
            "camera_frame": str(frames["camera"]),
        },
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
            "then verify a bounded visual correction against a known offset "
            "before use; never use this file alone as motion authorization"
        ),
    }
    if registration_document is not None:
        document["right_kinematic_registration"] = registration_document
    return document


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--urdf-xacro",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--right-registration",
        type=Path,
        help=(
            "independently validated R0-C right-arm registration; required "
            "for --arm right"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    camera_name = "wrist_a" if args.arm == "left" else "wrist_b"
    if args.camera_info is None:
        args.camera_info = Path(
            "ros2_ws/src/manipulation_camera_manager/config/"
            f"{camera_name}_camera_info.yaml"
        )
    if args.urdf_xacro is None:
        args.urdf_xacro = Path(
            "ros2_ws/src/so101_description/urdf/"
            + (
                "so101_left.urdf.xacro"
                if args.arm == "left"
                else "so101_dual_preview.urdf.xacro"
            )
        )
    if args.output is None:
        args.output = Path(f"output/{camera_name}_eye_in_hand_candidate.yaml")
    if args.arm == "right" and args.right_registration is None:
        parser.error("--right-registration is required for --arm right")
    if args.arm == "left" and args.right_registration is not None:
        parser.error("--right-registration is only valid for --arm right")
    return args


def main() -> int:
    args = parse_args()
    session = load_yaml(args.session)
    if str(session.get("arm", "left")) != args.arm:
        raise RuntimeError(
            f"session arm {session.get('arm', 'left')} != --arm {args.arm}"
        )
    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro)],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    right_registration = (
        load_yaml(args.right_registration)
        if args.right_registration is not None
        else None
    )
    right_registration_sha256 = (
        hashlib.sha256(args.right_registration.read_bytes()).hexdigest()
        if args.right_registration is not None
        else None
    )
    result = solve_document(
        session,
        args.session,
        load_yaml(args.camera_info),
        hashlib.sha256(args.camera_info.read_bytes()).hexdigest(),
        urdf_xml,
        right_registration,
        right_registration_sha256,
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
