"""Contract tests for the simulation-only Isaac bimanual snapshot tool."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/isaac_apply_bimanual_snapshot.py"
SPEC = importlib.util.spec_from_file_location("isaac_bimanual_snapshot", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)
SOURCE = TOOL_PATH.read_text(encoding="utf-8")


def test_exact_r4_joint_contract_is_accepted_without_conversion() -> None:
    positions = tuple(index * 0.01 for index in range(12))
    result = TOOL.validate_snapshot(TOOL.EXPECTED_JOINT_NAMES, positions)
    assert tuple(result) == TOOL.EXPECTED_JOINT_NAMES
    assert tuple(result.values()) == positions


@pytest.mark.parametrize(
    ("names", "positions"),
    (
        (("wrong_joint",) + TOOL.EXPECTED_JOINT_NAMES[1:], (0.0,) * 12),
        (TOOL.EXPECTED_JOINT_NAMES, (0.0,) * 11),
        (TOOL.EXPECTED_JOINT_NAMES, (0.0,) * 11 + (math.nan,)),
    ),
)
def test_invalid_or_partial_r4_snapshot_is_rejected(names, positions) -> None:
    with pytest.raises(ValueError):
        TOOL.validate_snapshot(names, positions)


def test_all_twelve_imported_joint_paths_must_be_unique() -> None:
    paths = [f"/robot/Physics/{name}" for name in TOOL.EXPECTED_JOINT_NAMES]
    result = TOOL.map_joint_paths(paths)
    assert result["left_base_joint"] == "/robot/Physics/left_base_joint"
    assert result["right_gripper_joint"] == "/robot/Physics/right_gripper_joint"

    with pytest.raises(RuntimeError):
        TOOL.map_joint_paths(paths[:-1])
    with pytest.raises(RuntimeError):
        TOOL.map_joint_paths(paths + ["/duplicate/left_base_joint"])


def test_each_fixed_base_branch_can_be_selected_as_fk_root() -> None:
    links = [
        "/robot/Geometry/workcell/left_mount_arm_base_link",
        "/robot/Geometry/workcell/left_mount_arm_base_link/left_base_link",
        "/robot/Geometry/workcell/right_mount_arm_base_link",
        "/robot/Geometry/workcell/right_mount_arm_base_link/right_base_link",
    ]
    assert str(TOOL.branch_first_link_paths(links, "left")[0]).endswith(
        "/left_mount_arm_base_link"
    )
    assert str(TOOL.branch_first_link_paths(links, "right")[0]).endswith(
        "/right_mount_arm_base_link"
    )
    with pytest.raises(ValueError):
        TOOL.branch_first_link_paths(links, "middle")


def test_tool_is_one_shot_simulation_only_and_has_no_motion_api() -> None:
    assert TOOL.SIMULATION_ONLY is True
    assert TOOL.MOTION_AUTHORIZED is False
    assert TOOL.HARDWARE_SYNCHRONOUS is False
    assert TOOL.BIMANUAL_TOPIC == "/bimanual_joint_states"
    assert TOOL.IMPORTED_LIMIT_TOLERANCE_DEG == 1.0
    assert "timeline.is_playing()" in SOURCE
    assert "stage.GetSessionLayer()" in SOURCE
    assert "apply_count=1" in SOURCE
    for forbidden in (
        "create_publisher",
        "create_client",
        "ActionClient",
        "send_goal_async",
        "arm_and_enable",
        "clear_fault",
        "right_arm_jog_once",
        "right_arm_torque_enable_once",
    ):
        assert forbidden not in SOURCE
