from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/check_bimanual_j1_model_limit_parity_plan_only.py"
GENERATOR_PATH = ROOT / "tools/generate_isaac_bimanual_preview_urdf.py"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TOOL = load_module("j1_model_parity", TOOL_PATH)
GENERATOR = load_module("j1_isaac_generator", GENERATOR_PATH)
APPROVED = ROOT / "config/bimanual_operational_limits.json"
MOVEIT = ROOT / (
    "ros2_ws/src/so101_moveit_config/config/joint_limits_dual.yaml"
)
HISTORICAL_J1_MOVEIT = ROOT / (
    "ros2_ws/src/so101_moveit_config/config/"
    "bimanual_j1_joint_limits.candidate.yaml"
)
PREVIEW = ROOT / (
    "ros2_ws/src/so101_description/urdf/so101_dual_preview.urdf.xacro"
)
FIRMWARE_HOST = ROOT / (
    "artifacts/joint_ranges/2026-08-13/"
    "j1_firmware_host_parity_plan_only.json"
)
HARDWARE = ROOT / (
    "artifacts/protocol_v2/2026-08-13/"
    "j1l_arm_limits_shadow_run01.json"
)


def expected_limits() -> dict[str, tuple[float, float]]:
    approved = json.loads(APPROVED.read_text(encoding="utf-8"))
    result = {}
    for side in ("left", "right"):
        for short_name in (
            "base", "shoulder", "elbow", "wrist_flex", "wrist_roll"
        ):
            item = approved["arms"][side][short_name]
            result[f"{side}_{short_name}_joint"] = (
                item["minimum_urad"] / 1e6,
                item["maximum_urad"] / 1e6,
            )
    return result


def test_active_dual_urdf_and_moveit_match_full_operational_approval() -> None:
    expected = expected_limits()
    urdf = TOOL.parse_urdf_arm_limits(TOOL.expand_xacro(PREVIEW))
    moveit = TOOL.moveit_candidate_limits(MOVEIT)
    TOOL.require_exact(expected, urdf, "URDF")
    TOOL.require_exact(expected, moveit, "MoveIt")
    assert not any(name.endswith("gripper_joint") for name in moveit)


def test_active_single_arm_defaults_and_launch_remain_unchanged() -> None:
    TOOL.verify_active_single_arm_defaults(ROOT)
    TOOL.candidate_not_loaded_by_moveit(ROOT, HISTORICAL_J1_MOVEIT)


def test_firmware_and_hardware_evidence_are_bound_to_passes() -> None:
    firmware_host = TOOL.load_bound_json(
        FIRMWARE_HOST, TOOL.file_sha256(FIRMWARE_HOST), "firmware/host"
    )
    hardware = TOOL.load_bound_json(
        HARDWARE, TOOL.file_sha256(HARDWARE), "hardware"
    )
    TOOL.validate_evidence(firmware_host, hardware)


def test_isaac_generator_binds_full_limits_and_stays_simulation_only(
    tmp_path: Path,
) -> None:
    output = tmp_path / "dual.urdf"
    args = argparse.Namespace(
        right_mount_xyz_m=(0.0, -0.2320641457, 0.0),
        right_mount_rpy_rad=(0.0, 0.0, 0.0),
        right_wrist_camera_mount=True,
        output=output,
    )
    urdf, manifest_path, _ = GENERATOR.generate(args)
    TOOL.require_exact(
        expected_limits(),
        TOOL.parse_urdf_arm_limits(urdf.read_text(encoding="utf-8")),
        "Isaac",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["simulation_only"] is True
    assert manifest["motion_authorized"] is False
    assert manifest["joint_limits"] == {
        "status": "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE",
        "approved_path": "config/bimanual_operational_limits.json",
        "approved_sha256": "436a5cfdc80aeaacfc4fd55812ec7ce102c7ecfe7443071484a942cad0946263",
        "arm_joint_count": 10,
        "grippers_excluded": True,
        "runtime_change_authorized": False,
    }


def test_plan_only_tool_contains_no_runtime_or_motion_api() -> None:
    source = TOOL_PATH.read_text(encoding="utf-8")
    assert "--plan-only is required" in source
    for forbidden in (
        "rclpy",
        "serial",
        "send_goal_async",
        "arm_and_enable",
        "openocd",
        "Servo_",
        "RightServoBus_",
    ):
        assert forbidden not in source
