#!/usr/bin/env python3
"""Validate right-arm workcell x/y/yaw in a motionless shadow calculation.

The held-out GridBoard captures are camera measurements that were never used
to fit the right-arm registration.  This tool recomputes their workcell poses
from pixels and compares them with corrected right-arm FK.  It deliberately
does not claim a tabletop-object validation: the board was attached to the
gripper above the table, not placed on the table plane.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[3]
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import solve_top_eye_to_hand as eye  # noqa: E402


STATUS = "RIGHT_REGISTRATION_WORKCELL_SHADOW_VALIDATED"
MAX_XY_ERROR_M = 0.008
MAX_YAW_ERROR_RAD = math.radians(3.0)
MIN_VALIDATION_CAPTURES = 2


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrapped_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def yaw(matrix: np.ndarray) -> float:
    return float(Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")[2])


def validate_documents(candidate: dict, session: dict, workcell: dict) -> None:
    if (
        candidate.get("status")
        != "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or candidate.get("arm") != "right"
        or candidate.get("method")
        != "workcell_anchored_right_joint_zero_bundle_adjustment"
        or candidate.get("motion_authorized") is not False
        or candidate.get("robot_target_available") is not False
    ):
        raise RuntimeError("right registration candidate is not fail-closed and validated")
    registration = candidate.get("right_kinematic_registration", {})
    if (
        registration.get("training_only_fit") is not True
        or registration.get("validation_used_in_fit") is not False
    ):
        raise RuntimeError("right registration did not preserve held-out validation")
    if (
        session.get("status") != "CAPTURE_SET_VALIDATED_READY_TO_SOLVE"
        or session.get("motion_authorized") is not False
        or session.get("arm") != "right"
        or len(session.get("validation_captures", ())) < MIN_VALIDATION_CAPTURES
    ):
        raise RuntimeError("right registration session has no independent validation set")
    if (
        workcell.get("status") != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or workcell.get("motion_authorized") is not False
        or workcell.get("base_registration", {}).get("transform_validated") is not True
    ):
        raise RuntimeError("worktable metric calibration is not independently validated")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        type=Path,
        default=ROOT / "artifacts/calibration/top_eye_to_hand_20260825_r0c/right_resident_torque_candidate.yaml",
    )
    parser.add_argument(
        "--session",
        type=Path,
        default=ROOT / "artifacts/calibration/top_eye_to_hand_20260825_r0c/right_resident_torque_session.yaml",
    )
    parser.add_argument(
        "--workcell-reference",
        type=Path,
        default=ROOT / "artifacts/calibration/top_eye_to_hand_20260825_r0b/candidate.yaml",
    )
    parser.add_argument(
        "--worktable",
        type=Path,
        default=ROOT / "ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml",
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=ROOT / "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml",
    )
    parser.add_argument(
        "--urdf-xacro",
        type=Path,
        default=ROOT / "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro",
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def build_document(args: argparse.Namespace) -> dict:
    paths = {
        "candidate": args.candidate.resolve(),
        "session": args.session.resolve(),
        "workcell_reference": args.workcell_reference.resolve(),
        "worktable": args.worktable.resolve(),
        "camera_info": args.camera_info.resolve(),
    }
    documents = {
        name: yaml.safe_load(path.read_text(encoding="utf-8"))
        for name, path in paths.items()
    }
    candidate = documents["candidate"]
    session = documents["session"]
    validate_documents(candidate, session, documents["worktable"])

    urdf_xml = subprocess.run(
        ["xacro", str(args.urdf_xacro), "arm_slot:=right"],
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    camera_info = documents["camera_info"]
    camera_matrix = eye.yaml_matrix(camera_info, "camera_matrix", 3, 3)
    distortion = eye.yaml_matrix(
        camera_info, "distortion_coefficients", 1, 5
    ).reshape(-1)
    specification = eye.parse_target(session)
    joint_names = eye.ARM_JOINT_NAMES_BY_SIDE["right"]
    registration = candidate["right_kinematic_registration"]
    offsets = np.asarray(
        [registration["joint_zero_offsets_rad"][name] for name in joint_names],
        dtype=np.float64,
    )
    workcell_to_right_base = np.asarray(
        registration["workcell_to_right_base"]["data"], dtype=np.float64
    )
    workcell_to_camera = np.asarray(
        documents["workcell_reference"]["base_to_camera"]["data"],
        dtype=np.float64,
    )
    gripper_to_target = np.asarray(
        candidate["gripper_to_target"]["data"], dtype=np.float64
    )

    rows = []
    for capture in session["validation_captures"]:
        camera_observation = eye.capture_observation(
            capture,
            paths["session"].parent,
            urdf_xml,
            "right_base_link",
            "right_gripper_frame_link",
            camera_matrix,
            distortion,
            specification,
            joint_names,
        )
        corrected = eye.observations_with_joint_zero_offsets(
            [capture],
            [camera_observation],
            urdf_xml,
            "right_base_link",
            "right_gripper_frame_link",
            joint_names,
            offsets,
        )[0]
        predicted = (
            workcell_to_right_base
            @ corrected.base_to_gripper
            @ gripper_to_target
        )
        measured = workcell_to_camera @ camera_observation.camera_to_target
        delta = measured[:3, 3] - predicted[:3, 3]
        xy_error = float(np.linalg.norm(delta[:2]))
        yaw_error = wrapped_angle(yaw(measured) - yaw(predicted))
        rows.append(
            {
                "id": str(capture["id"]),
                "predicted_workcell_xyz_m": predicted[:3, 3].tolist(),
                "camera_measured_workcell_xyz_m": measured[:3, 3].tolist(),
                "signed_xyz_error_mm": (delta * 1000.0).tolist(),
                "xy_error_mm": xy_error * 1000.0,
                "yaw_error_deg": math.degrees(yaw_error),
                "pnp_rms_px": camera_observation.pnp_rms_px,
            }
        )

    maximum_xy = max(row["xy_error_mm"] for row in rows)
    maximum_yaw = max(abs(row["yaw_error_deg"]) for row in rows)
    failures = []
    if maximum_xy > MAX_XY_ERROR_M * 1000.0:
        failures.append("held-out workcell XY error exceeds 8 mm")
    if maximum_yaw > math.degrees(MAX_YAW_ERROR_RAD):
        failures.append("held-out workcell yaw error exceeds 3 deg")
    if failures:
        raise RuntimeError("; ".join(failures))

    return {
        "schema_version": 1,
        "record_kind": "right_registration_workcell_shadow_validation",
        "status": STATUS,
        "motion_authorized": False,
        "robot_target_available": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "validation_used_in_fit": False,
        "tabletop_object_validation_performed": False,
        "scope": "held_out_gripper_marker_workcell_xy_yaw_only",
        "sources": {
            name: {"path": str(path), "sha256": sha256_file(path)}
            for name, path in paths.items()
        },
        "limits": {
            "maximum_xy_error_mm": MAX_XY_ERROR_M * 1000.0,
            "maximum_yaw_error_deg": math.degrees(MAX_YAW_ERROR_RAD),
        },
        "metrics": {
            "validation_capture_count": len(rows),
            "maximum_xy_error_mm": maximum_xy,
            "maximum_abs_yaw_error_deg": maximum_yaw,
            "captures": rows,
        },
        "required_next_gate": (
            "plan-only and supervised OBSERVE_CLEAR roundtrip; independent "
            "table-plane object validation remains required before using "
            "right FK for tabletop motion targets"
        ),
    }


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise RuntimeError(f"refusing to overwrite existing artifact: {args.output}")
    document = build_document(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    metrics = document["metrics"]
    print(
        f"{STATUS} captures={metrics['validation_capture_count']} "
        f"xy_max_mm={metrics['maximum_xy_error_mm']:.3f} "
        f"yaw_max_deg={metrics['maximum_abs_yaw_error_deg']:.3f} "
        "motion_commands=0 tabletop_object_validation=false "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
