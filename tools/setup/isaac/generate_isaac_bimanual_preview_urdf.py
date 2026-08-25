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
from xml.etree import ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


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
RIGHT_ARM_JOINTS = (
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
)
REGISTERED_METHOD = "workcell_anchored_right_joint_zero_bundle_adjustment"


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


def _format_vector(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_registration_candidate(path: Path) -> tuple[dict, np.ndarray, dict[str, float]]:
    candidate = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(candidate, dict):
        raise ValueError("registration candidate root must be a mapping")
    if (
        candidate.get("status")
        != "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or candidate.get("arm") != "right"
        or candidate.get("method") != REGISTERED_METHOD
        or bool(candidate.get("motion_authorized", False))
    ):
        raise RuntimeError(
            "right registration candidate is not independently validated"
        )
    registration = candidate.get("right_kinematic_registration")
    if not isinstance(registration, dict):
        raise ValueError("right kinematic registration is missing")
    if (
        registration.get("training_only_fit") is not True
        or registration.get("validation_used_in_fit") is not False
    ):
        raise RuntimeError("registration did not preserve held-out validation")
    mount_document = registration.get("workcell_to_right_base")
    if not isinstance(mount_document, dict):
        raise ValueError("workcell-to-right-base transform is missing")
    mount = np.asarray(mount_document.get("data"), dtype=np.float64)
    if mount.shape != (4, 4) or not np.all(np.isfinite(mount)):
        raise ValueError("workcell-to-right-base transform is invalid")
    offsets_document = registration.get("joint_zero_offsets_rad")
    if not isinstance(offsets_document, dict):
        raise ValueError("right joint-zero offsets are missing")
    offsets = {
        name: float(offsets_document[name]) for name in RIGHT_ARM_JOINTS
    }
    if not all(math.isfinite(value) for value in offsets.values()):
        raise ValueError("right joint-zero offsets must be finite")
    return candidate, mount, offsets


def _apply_right_joint_zero_offsets(
    urdf_xml: str,
    offsets: dict[str, float],
) -> str:
    root = ET.fromstring(urdf_xml)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    for name in RIGHT_ARM_JOINTS:
        if name not in joints:
            raise ValueError(f"preview URDF is missing {name}")
        joint = joints[name]
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
        axis_element = joint.find("axis")
        axis = np.fromstring(
            "1 0 0"
            if axis_element is None
            else axis_element.attrib.get("xyz", "1 0 0"),
            sep=" ",
        )
        if xyz.shape != (3,) or rpy.shape != (3,) or axis.shape != (3,):
            raise ValueError(f"{name} has an invalid origin or axis")
        norm = float(np.linalg.norm(axis))
        if norm <= 0.0:
            raise ValueError(f"{name} has a zero joint axis")
        origin_rotation = Rotation.from_euler("xyz", rpy).as_matrix()
        zero_rotation = Rotation.from_rotvec(
            axis / norm * offsets[name]
        ).as_matrix()
        corrected_rpy = Rotation.from_matrix(
            origin_rotation @ zero_rotation
        ).as_euler("xyz")
        origin.attrib["xyz"] = _format_vector(xyz)
        origin.attrib["rpy"] = _format_vector(corrected_rpy)
    root.insert(
        0,
        ET.Comment(
            " RIGHT REGISTRATION SHADOW/PREVIEW; permitted for plan-only "
            "collision checks, never firmware or hardware motion authority "
        ),
    )
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


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
        "--right-registration-candidate",
        type=Path,
        help=(
            "independently validated constrained right registration; "
            "overrides the preview mount and absorbs joint-zero offsets into "
            "right joint origins for simulation-only q0 inspection"
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
    registration_candidate = None
    registration_path = None
    joint_zero_offsets = None
    right_mount_xyz = list(args.right_mount_xyz_m)
    right_mount_rpy = list(args.right_mount_rpy_rad)
    if args.right_registration_candidate is not None:
        registration_path = args.right_registration_candidate.resolve()
        (
            registration_candidate,
            registered_mount,
            joint_zero_offsets,
        ) = _load_registration_candidate(registration_path)
        right_mount_xyz = registered_mount[:3, 3].tolist()
        right_mount_rpy = Rotation.from_matrix(
            registered_mount[:3, :3]
        ).as_euler("xyz").tolist()
    right_xyz = _xyz(right_mount_xyz)
    right_rpy = _xyz(right_mount_rpy)
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
    urdf_xml = completed.stdout
    if joint_zero_offsets is not None:
        urdf_xml = _apply_right_joint_zero_offsets(
            urdf_xml,
            joint_zero_offsets,
        )
    output.write_text(urdf_xml, encoding="utf-8")
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
            "xyz_m": right_mount_xyz,
            "rpy_rad": right_mount_rpy,
            "status": (
                "MULTI_POSE_REGISTERED_SHADOW_ONLY"
                if registration_candidate is not None
                else "PROVISIONAL_UNCALIBRATED"
            ),
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
            "right": (
                "MULTI_POSE_VALIDATED_SHADOW_ONLY"
                if registration_candidate is not None
                else "AWAITING_EXTERNAL_MULTI_POSE_VALIDATION"
            ),
        },
        "wrist_camera_mount_geometry": {
            "left": True,
            "right": bool(args.right_wrist_camera_mount),
            "same_wrist_joint_origin": True,
            "left_optical_frame": "VALIDATED",
            "right_optical_frame": (
                "VALIDATED_TORQUE_HOLD_EYE_IN_HAND"
                if args.right_wrist_camera_mount
                else "DISABLED_WITH_CAMERA_MOUNT"
            ),
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
    if registration_candidate is not None:
        assert registration_path is not None
        assert joint_zero_offsets is not None
        document["right_registration_candidate"] = {
            "path": str(registration_path),
            "sha256": _sha256(registration_path),
            "status": str(registration_candidate["status"]),
            "method": str(registration_candidate["method"]),
            "runtime_promotion_authorized": False,
            "joint_zero_offsets_rad": joint_zero_offsets,
            "validation_translation_max_mm": float(
                registration_candidate["validation_fit"][
                    "translation_max_mm"
                ]
            ),
            "validation_rotation_max_deg": float(
                registration_candidate["validation_fit"][
                    "rotation_max_deg"
                ]
            ),
        }
    manifest.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return output, manifest, digest


def main() -> int:
    args = parse_args()
    output, manifest, digest = generate(args)
    print(f"BIMANUAL_PREVIEW_URDF={output}")
    print(f"SHA256={digest}")
    print(f"MANIFEST={manifest}")
    print("ISAAC_ROS_PACKAGE_NAME=so101_description")
    print(
        "ISAAC_ROS_PACKAGE_PATH="
        f"{ROOT / 'ros2_ws/src/so101_description'}"
    )
    print("SIMULATION_ONLY=true MOTION_AUTHORIZED=false")
    print(
        "RIGHT_MOUNT_STATUS="
        + (
            "MULTI_POSE_REGISTERED_SHADOW_ONLY"
            if args.right_registration_candidate is not None
            else "PROVISIONAL_UNCALIBRATED"
        )
    )
    print("JOINT_LIMITS_STATUS=OPERATOR_VERIFIED_FULL_TASK_ENVELOPE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
