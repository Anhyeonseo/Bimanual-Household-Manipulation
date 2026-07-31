import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "assemble_pick_place_plan_only.py"
SPEC = importlib.util.spec_from_file_location(
    "assemble_pick_place_plan_only",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_phase(
    tmp_path: Path,
    name: str,
    target_name: str,
    start: tuple[float, ...],
    target: tuple[float, ...],
) -> Path:
    path = tmp_path / f"{name}.json"
    delta = max(abs(a - b) for a, b in zip(start, target, strict=True))
    path.write_text(
        json.dumps(
            {
                "status": (
                    f"{target_name.upper()}_SEGMENT_PLAN_ONLY_PASS"
                ),
                "execution_api_used": False,
                "motion_authorized": False,
                "robot_target_available": False,
                "target_name": target_name,
                "joint_names": list(MODULE.ARM_JOINTS),
                "max_joint_step_rad": MODULE.MAX_JOINT_STEP_RAD,
                "segments": [
                    {
                        "index": 1,
                        "success": True,
                        "expected_start_positions_rad": list(start),
                        "target_positions_rad": list(target),
                        "maximum_joint_delta_rad": delta,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    return path


def phase_chain(tmp_path: Path):
    q0 = (0.0,) * 5
    pick_pre = (0.01,) * 5
    pick = (0.02,) * 5
    lift = (0.03,) * 5
    place_pre = (0.04,) * 5
    place = (0.05,) * 5
    specs = [
        (
            "q0_to_pick_pregrasp",
            write_phase(tmp_path, "a", "pregrasp", q0, pick_pre),
            "pregrasp",
            False,
        ),
        (
            "pick_pregrasp_to_grasp",
            write_phase(tmp_path, "b", "grasp", pick_pre, pick),
            "grasp",
            False,
        ),
        (
            "pick_grasp_to_lift20",
            write_phase(tmp_path, "c", "grasp", pick, lift),
            "grasp",
            False,
        ),
        (
            "lift_to_place_pregrasp",
            write_phase(tmp_path, "d", "pregrasp", lift, place_pre),
            "pregrasp",
            False,
        ),
        (
            "place_pregrasp_to_place",
            write_phase(tmp_path, "e", "grasp", place_pre, place),
            "grasp",
            False,
        ),
        (
            "place_to_retreat",
            write_phase(tmp_path, "f", "pregrasp", place, place_pre),
            "pregrasp",
            False,
        ),
        (
            "place_pregrasp_to_q0",
            write_phase(tmp_path, "g", "pregrasp", q0, place_pre),
            "pregrasp",
            True,
        ),
    ]
    return specs


def test_assemble_is_non_executable_contiguous_and_returns_q0(tmp_path):
    result = MODULE.assemble(
        phase_chain(tmp_path),
        ROOT / "config" / "single_arm_calibration.json",
        (0.37, -0.07, 0.0063, -0.03),
        0.13,
        0.06,
    )
    assert result["status"] == "FULL_PICK_PLACE_PLAN_ONLY_PASS"
    assert result["execution_api_used"] is False
    assert result["motion_authorized"] is False
    assert result["robot_target_available"] is False
    assert result["automatic_execution_permitted"] is False
    assert result["calibration_hash"] == "0x8AD27897"
    assert result["arm_segment_count"] == 7
    assert result["command_step_count"] == 9
    assert [step["phase"] for step in result["steps"] if step["kind"] == "gripper"] == [
        "pick_close",
        "place_release",
    ]
    arm = [step for step in result["steps"] if step["kind"] == "arm"]
    assert arm[0]["start_positions_rad"] == [0.0] * 5
    assert arm[-1]["target_positions_rad"] == [0.0] * 5
    assert all(step["manual_gate_required"] for step in arm)


def test_reverse_phase_swaps_and_reverses_segments(tmp_path):
    path = tmp_path / "reverse.json"
    path.write_text(
        json.dumps(
            {
                "status": "PREGRASP_SEGMENT_PLAN_ONLY_PASS",
                "execution_api_used": False,
                "motion_authorized": False,
                "robot_target_available": False,
                "target_name": "pregrasp",
                "joint_names": list(MODULE.ARM_JOINTS),
                "max_joint_step_rad": 0.18,
                "segments": [
                    {
                        "index": 1,
                        "success": True,
                        "expected_start_positions_rad": [0.0] * 5,
                        "target_positions_rad": [0.1] * 5,
                        "maximum_joint_delta_rad": 0.1,
                    },
                    {
                        "index": 2,
                        "success": True,
                        "expected_start_positions_rad": [0.1] * 5,
                        "target_positions_rad": [0.2] * 5,
                        "maximum_joint_delta_rad": 0.1,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    steps = MODULE.load_phase(path, "pregrasp", reverse=True)
    assert steps[0]["start_positions_rad"] == [0.2] * 5
    assert steps[0]["target_positions_rad"] == [0.1] * 5
    assert steps[1]["target_positions_rad"] == [0.0] * 5


def test_executable_unbounded_or_discontinuous_source_is_rejected(tmp_path):
    path = write_phase(
        tmp_path,
        "bad",
        "pregrasp",
        (0.0,) * 5,
        (0.1,) * 5,
    )
    document = json.loads(path.read_text(encoding="utf-8"))
    document["motion_authorized"] = True
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="motion_authorized=false"):
        MODULE.load_phase(path, "pregrasp")

    path = write_phase(
        tmp_path,
        "oversized",
        "pregrasp",
        (0.0,) * 5,
        (0.1,) * 5,
    )
    document = json.loads(path.read_text(encoding="utf-8"))
    document["max_joint_step_rad"] = 0.19
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="exceeds Stage 7"):
        MODULE.load_phase(path, "pregrasp")


def test_chain_discontinuity_is_rejected(tmp_path):
    specs = phase_chain(tmp_path)
    name, path, target_name, reverse = specs[1]
    document = json.loads(path.read_text(encoding="utf-8"))
    document["segments"][0]["expected_start_positions_rad"] = [0.015] * 5
    document["segments"][0]["maximum_joint_delta_rad"] = 0.005
    path.write_text(json.dumps(document), encoding="utf-8")
    specs[1] = (name, path, target_name, reverse)
    with pytest.raises(ValueError, match="does not continue"):
        MODULE.assemble(
            specs,
            ROOT / "config" / "single_arm_calibration.json",
            (0.37, -0.07, 0.0063, -0.03),
            0.13,
            0.06,
        )


def test_place_target_must_be_inside_workspace_and_board(tmp_path):
    specs = phase_chain(tmp_path)
    with pytest.raises(ValueError, match="approved workspace"):
        MODULE.assemble(
            specs,
            ROOT / "config" / "single_arm_calibration.json",
            (0.50, -0.07, 0.0063, -0.03),
            0.13,
            0.06,
        )

    specs = phase_chain(tmp_path)
    with pytest.raises(ValueError, match="table board"):
        MODULE.assemble(
            specs,
            ROOT / "config" / "single_arm_calibration.json",
            (0.30, -0.07, 0.0063, -0.03),
            0.13,
            0.06,
        )

    specs = phase_chain(tmp_path)
    with pytest.raises(ValueError, match="TCP z"):
        MODULE.assemble(
            specs,
            ROOT / "config" / "single_arm_calibration.json",
            (0.37, -0.07, 0.06, -0.03),
            0.13,
            0.06,
        )


def test_tool_has_no_execution_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    for forbidden in (
        "ActionClient",
        "ExecuteTrajectory",
        "send_goal_async",
        "FollowJointTrajectory",
    ):
        assert forbidden not in source
