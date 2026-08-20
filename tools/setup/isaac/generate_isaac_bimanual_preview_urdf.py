#!/usr/bin/env python3
"""Expand the simulation-only bimanual xacro for Isaac Sim URDF import."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
XACRO = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
DEFAULT_OUTPUT = ROOT / "artifacts/bimanual/preview/so101_dual_preview.urdf"
SIMULATION_ONLY = True
MOTION_AUTHORIZED = False
APPROVED_OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
APPROVED_OPERATIONAL_LIMITS_SHA256 = (
    "436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263"
)


def _xacro_executable() -> str:
    discovered = shutil.which("xacro")
    if discovered is not None:
        return discovered
    jazzy = Path("/opt/ros/jazzy/bin/xacro")
    if jazzy.is_file():
        return str(jazzy)
    raise FileNotFoundError(
        "xacro was not found; source /opt/ros/jazzy/setup.bash first"
    )


def _xyz(values: list[float]) -> str:
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("mount coordinates must contain three finite values")
    return " ".join(f"{value:.9g}" for value in values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a non-motion bimanual URDF for Isaac Sim import."
    )
    parser.add_argument(
        "--right-mount-xyz-m",
        nargs=3,
        type=float,
        default=(0.0, -0.2320641457, 0.0),
        metavar=("X", "Y", "Z"),
        help=(
            "CAD-fit right-base origin in the left-base workcell frame; "
            "both plates overlap the center camera mount by 10 mm"
        ),
    )
    parser.add_argument(
        "--right-mount-rpy-rad",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("R", "P", "Y"),
    )
    parser.add_argument(
        "--right-wrist-camera-mount",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="use the physical wrist-camera replacement STL on the right arm",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def generate(args: argparse.Namespace) -> tuple[Path, Path, str]:
    approved_actual = hashlib.sha256(APPROVED_OPERATIONAL_LIMITS.read_bytes()).hexdigest()
    if approved_actual != APPROVED_OPERATIONAL_LIMITS_SHA256:
        raise RuntimeError(
            "approved operational-limit SHA mismatch: "
            f"expected={APPROVED_OPERATIONAL_LIMITS_SHA256} actual={approved_actual}"
        )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    right_xyz = _xyz(list(args.right_mount_xyz_m))
    right_rpy = _xyz(list(args.right_mount_rpy_rad))
    command = [
        _xacro_executable(),
        str(XACRO),
        f"right_mount_xyz:={right_xyz}",
        f"right_mount_rpy:={right_rpy}",
        "right_use_wrist_camera_mount:="
        + str(args.right_wrist_camera_mount).lower(),
    ]
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )
    output.write_text(completed.stdout, encoding="utf-8")
    digest = hashlib.sha256(output.read_bytes()).hexdigest()

    manifest = output.with_suffix(".manifest.json")
    document = {
        "schema_version": 1,
        "simulation_only": SIMULATION_ONLY,
        "motion_authorized": MOTION_AUTHORIZED,
        "source_xacro": str(XACRO.relative_to(ROOT)),
        "urdf": str(output),
        "urdf_sha256": digest,
        "left_mount": {
            "xyz_m": [0.0, 0.0, 0.0],
            "rpy_rad": [0.0, 0.0, 0.0],
            "status": "REFERENCE",
        },
        "right_mount": {
            "xyz_m": list(args.right_mount_xyz_m),
            "rpy_rad": list(args.right_mount_rpy_rad),
            "status": "PROVISIONAL_UNCALIBRATED",
        },
        "joint_limits": {
            "status": "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE",
            "approved_path": str(APPROVED_OPERATIONAL_LIMITS.relative_to(ROOT)),
            "approved_sha256": APPROVED_OPERATIONAL_LIMITS_SHA256,
            "arm_joint_count": 10,
            "grippers_excluded": True,
            "runtime_change_authorized": False,
        },
        "joint_zero_status": {
            "left": "VALIDATED_EXISTING_Q0_CONTRACT",
            "right": "AWAITING_EXTERNAL_MULTI_POSE_VALIDATION",
        },
        "wrist_camera_mount_geometry": {
            "left": True,
            "right": bool(args.right_wrist_camera_mount),
            "same_wrist_joint_origin": True,
            "left_optical_frame": "VALIDATED",
            "right_optical_frame": "ABSENT_AWAITING_CAMERA_CALIBRATION",
        },
        "isaac_import": {
            "input_file": str(output),
            "ros_package_list": [
                {
                    "package_name": "so101_description",
                    "package_path": str(
                        ROOT / "ros2_ws/src/so101_description"
                    ),
                }
            ],
        },
    }
    manifest.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return output, manifest, digest


def main() -> int:
    output, manifest, digest = generate(parse_args())
    print(f"BIMANUAL_PREVIEW_URDF={output}")
    print(f"SHA256={digest}")
    print(f"MANIFEST={manifest}")
    print("ISAAC_ROS_PACKAGE_NAME=so101_description")
    print(
        "ISAAC_ROS_PACKAGE_PATH="
        f"{ROOT / 'ros2_ws/src/so101_description'}"
    )
    print("SIMULATION_ONLY=true MOTION_AUTHORIZED=false")
    print("RIGHT_MOUNT_STATUS=PROVISIONAL_UNCALIBRATED")
    print("JOINT_LIMITS_STATUS=OPERATOR_VERIFIED_FULL_TASK_ENVELOPE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
