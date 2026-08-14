#!/usr/bin/env python3
"""Verify immutable J1-L limits across URDF, MoveIt, and Isaac input."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import yaml


APPROVED_SHA256 = (
    "ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff"
)
STATUS = "J1_L_MODEL_STACK_PARITY_PLAN_ONLY_PASS"
ARM_JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
ACTIVE_LEGACY_LIMITS = {
    "base": (-1.91986, 1.91986),
    "shoulder": (-0.174533673205103, 3.3161263267949),
    "elbow": (-0.730068911403119, 2.64993108859688),
    "wrist_flex": (-0.525371313497813, 2.790748686502188),
    "wrist_roll": (-4.3146463267949, 1.2704136732051),
}


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bound_json(path: Path, expected_sha256: str, label: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise ValueError(
            f"{label} SHA mismatch expected={expected_sha256.lower()} actual={actual}"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def xacro_executable() -> str:
    executable = shutil.which("xacro")
    if executable is not None:
        return executable
    jazzy = Path("/opt/ros/jazzy/bin/xacro")
    if jazzy.is_file():
        return str(jazzy)
    raise FileNotFoundError("xacro executable was not found")


def expand_xacro(path: Path) -> str:
    return subprocess.run(
        [xacro_executable(), str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def parse_urdf_arm_limits(xml: str) -> dict[str, tuple[float, float]]:
    root = ET.fromstring(xml)
    joints = {item.attrib["name"]: item for item in root.findall("joint")}
    result: dict[str, tuple[float, float]] = {}
    for arm in ("left", "right"):
        for joint in ARM_JOINTS:
            name = f"{arm}_{joint}_joint"
            element = joints.get(name)
            if element is None:
                raise ValueError(f"URDF is missing {name}")
            limit = element.find("limit")
            if limit is None:
                raise ValueError(f"URDF joint has no limit: {name}")
            result[name] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )
    return result


def approved_limits(document: dict[str, Any]) -> dict[str, tuple[float, float]]:
    if document.get("status") != "J1_L_ARM_5_APPROVED_FOR_PARITY_ONLY":
        raise ValueError("approved J1-L status changed")
    if document.get("operator_approved") is not True:
        raise ValueError("J1-L limits are not operator approved")
    if document.get("motion_authorized") is not False:
        raise ValueError("approved limits must remain motion_authorized=false")
    if document.get("runtime_change_authorized") is not False:
        raise ValueError("approved limits must remain runtime-disabled")
    result: dict[str, tuple[float, float]] = {}
    for arm in ("left", "right"):
        for joint in ARM_JOINTS:
            item = document["arms"][arm][joint]
            result[f"{arm}_{joint}_joint"] = (
                int(item["minimum_urad"]) / 1_000_000.0,
                int(item["maximum_urad"]) / 1_000_000.0,
            )
        if document["arms"][arm]["gripper"]["status"] != (
            "BLOCKED_SEMANTIC_GRIPPER_MAPPING_REQUIRED"
        ):
            raise ValueError(f"{arm} gripper unexpectedly became approved")
    return result


def moveit_candidate_limits(path: Path) -> dict[str, tuple[float, float]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    joints = document.get("joint_limits", {})
    expected_names = {
        f"{arm}_{joint}_joint"
        for arm in ("left", "right")
        for joint in ARM_JOINTS
    }
    if set(joints) != expected_names:
        raise ValueError("MoveIt candidate must contain exactly ten arm joints")
    result: dict[str, tuple[float, float]] = {}
    for name, item in joints.items():
        if item.get("has_position_limits") is not True:
            raise ValueError(f"MoveIt position limit disabled: {name}")
        result[name] = (float(item["min_position"]), float(item["max_position"]))
    return result


def require_exact(
    expected: dict[str, tuple[float, float]],
    actual: dict[str, tuple[float, float]],
    label: str,
) -> None:
    if actual.keys() != expected.keys():
        raise ValueError(f"{label} joint identity differs")
    for name, interval in expected.items():
        if actual[name] != interval:
            raise ValueError(
                f"{label} limit drift for {name}: expected={interval} actual={actual[name]}"
            )


def verify_active_single_arm_defaults(root: Path) -> None:
    xml = expand_xacro(
        root / "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"
    )
    # The active tree is intentionally single-arm; inspect its five limits directly.
    tree = ET.fromstring(xml)
    joints = {item.attrib["name"]: item for item in tree.findall("joint")}
    for joint, expected in ACTIVE_LEGACY_LIMITS.items():
        limit = joints[f"left_{joint}_joint"].find("limit")
        assert limit is not None
        actual = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
        if actual != expected:
            raise ValueError(
                f"active single-arm URDF default changed for {joint}: {actual}"
            )


def candidate_not_loaded_by_moveit(root: Path, candidate: Path) -> None:
    launch_dir = root / "ros2_ws/src/so101_moveit_config/launch"
    for path in launch_dir.glob("*.py"):
        if candidate.name in path.read_text(encoding="utf-8"):
            raise ValueError(f"candidate is active in MoveIt launch: {path}")


def validate_evidence(firmware_host: dict[str, Any], hardware: dict[str, Any]) -> None:
    if firmware_host.get("status") != "J1_L_FIRMWARE_HOST_PARITY_PLAN_ONLY_PASS":
        raise ValueError("firmware/host parity evidence did not pass")
    if firmware_host.get("motion_authorized") is not False:
        raise ValueError("firmware/host evidence authorized motion")
    if hardware.get("overall_verdict") != "J1L_ARM_LIMITS_SHADOW_NO_OUTPUT_PASS":
        raise ValueError("J1-L hardware evidence did not pass")
    if hardware.get("motion_authorized") is not False:
        raise ValueError("J1-L hardware evidence authorized motion")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--approved",
        type=Path,
        default=root / "config/bimanual_j1_operational_limits.approved.json",
    )
    parser.add_argument("--approved-sha256", required=True)
    parser.add_argument(
        "--firmware-host-evidence",
        type=Path,
        default=root / "artifacts/joint_ranges/2026-08-13/j1_firmware_host_parity_plan_only.json",
    )
    parser.add_argument("--firmware-host-evidence-sha256", required=True)
    parser.add_argument(
        "--hardware-evidence",
        type=Path,
        default=root / "artifacts/protocol_v2/2026-08-13/j1l_arm_limits_shadow_run01.json",
    )
    parser.add_argument("--hardware-evidence-sha256", required=True)
    parser.add_argument(
        "--moveit-candidate",
        type=Path,
        default=root / "ros2_ws/src/so101_moveit_config/config/bimanual_j1_joint_limits.candidate.yaml",
    )
    parser.add_argument(
        "--isaac-urdf",
        type=Path,
        default=root / "artifacts/bimanual/j1l_model/so101_dual_j1l_preview.urdf",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.set_defaults(root=root)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")
    approved = load_bound_json(args.approved, args.approved_sha256, "approved")
    expected = approved_limits(approved)
    firmware_host = load_bound_json(
        args.firmware_host_evidence,
        args.firmware_host_evidence_sha256,
        "firmware/host evidence",
    )
    hardware = load_bound_json(
        args.hardware_evidence, args.hardware_evidence_sha256, "hardware evidence"
    )
    validate_evidence(firmware_host, hardware)

    preview_xacro = args.root / (
        "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
    )
    require_exact(expected, parse_urdf_arm_limits(expand_xacro(preview_xacro)), "URDF")
    require_exact(expected, moveit_candidate_limits(args.moveit_candidate), "MoveIt")
    verify_active_single_arm_defaults(args.root)
    candidate_not_loaded_by_moveit(args.root, args.moveit_candidate)

    generator = args.root / "tools/generate_isaac_bimanual_preview_urdf.py"
    subprocess.run(
        ["python3", str(generator), "--output", str(args.isaac_urdf)],
        check=True,
        capture_output=True,
        text=True,
    )
    require_exact(
        expected,
        parse_urdf_arm_limits(args.isaac_urdf.read_text(encoding="utf-8")),
        "Isaac import URDF",
    )
    isaac_manifest = args.isaac_urdf.with_suffix(".manifest.json")
    manifest = json.loads(isaac_manifest.read_text(encoding="utf-8"))
    if manifest.get("simulation_only") is not True or manifest.get("motion_authorized") is not False:
        raise ValueError("Isaac manifest is not fail-closed")

    report = {
        "status": STATUS,
        "motion_authorized": False,
        "runtime_change_authorized": False,
        "execution_api_used": False,
        "active_single_arm_urdf_defaults_unchanged": True,
        "active_moveit_launch_unchanged": True,
        "arm_joint_count": 10,
        "grippers_excluded": True,
        "parity": {
            "firmware_unwrapped_urad": True,
            "host_unwrapped_rad": True,
            "hardware_shadow_no_output": True,
            "urdf_candidate": True,
            "moveit_candidate": True,
            "isaac_import_urdf": True,
        },
        "limits_rad": {name: list(interval) for name, interval in expected.items()},
        "inputs": {
            "approved": {"path": str(args.approved), "sha256": args.approved_sha256.lower()},
            "firmware_host_evidence": {"path": str(args.firmware_host_evidence), "sha256": args.firmware_host_evidence_sha256.lower()},
            "hardware_evidence": {"path": str(args.hardware_evidence), "sha256": args.hardware_evidence_sha256.lower()},
            "moveit_candidate": {"path": str(args.moveit_candidate), "sha256": file_sha256(args.moveit_candidate)},
            "isaac_urdf": {"path": str(args.isaac_urdf), "sha256": file_sha256(args.isaac_urdf)},
            "isaac_manifest": {"path": str(isaac_manifest), "sha256": file_sha256(isaac_manifest)},
        },
        "remaining_blockers": [
            "gripper semantic mapping",
            "physical q0 and inter-base model alignment",
            "right-arm representative route coverage",
            "J2 bounded active validation",
        ],
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"STATUS={STATUS} arm_joints=10 grippers=blocked "
        f"motion_authorized=false output={output} sha256={file_sha256(output)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
