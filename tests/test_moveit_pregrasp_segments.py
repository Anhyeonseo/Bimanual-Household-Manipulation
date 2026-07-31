import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "ros_moveit_plan_pregrasp_segments.py"
SPEC = importlib.util.spec_from_file_location(
    "ros_moveit_plan_pregrasp_segments",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def source_plan(tmp_path: Path) -> Path:
    path = tmp_path / "plan.json"
    path.write_text(
        json.dumps(
            {
                "status": "PLAN_ONLY_PASS",
                "execution_api_used": False,
                "plans": [
                    {
                        "name": "pregrasp",
                        "success": True,
                        "joint_names": list(reversed(MODULE.ARM_JOINTS)),
                        "final_joint_positions_rad": list(
                            reversed((0.30, 1.77, 0.67, 1.28, 0.14))
                        ),
                    },
                    {
                        "name": "grasp",
                        "success": True,
                        "joint_names": list(MODULE.ARM_JOINTS),
                        "final_joint_positions_rad": [
                            0.34,
                            2.09,
                            0.75,
                            1.29,
                            0.11,
                        ],
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    return path


def test_source_plan_is_reordered_to_project_joint_contract(tmp_path):
    assert MODULE.load_pregrasp_target(source_plan(tmp_path)) == (
        0.30,
        1.77,
        0.67,
        1.28,
        0.14,
    )
    assert MODULE.load_target(source_plan(tmp_path), "grasp") == (
        0.34,
        2.09,
        0.75,
        1.29,
        0.11,
    )


def test_unknown_target_name_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="target_name"):
        MODULE.load_target(source_plan(tmp_path), "lift")


def test_rejected_or_executable_source_plan_is_rejected(tmp_path):
    path = source_plan(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["status"] = "PLAN_ONLY_FAIL"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="not PLAN_ONLY_PASS"):
        MODULE.load_pregrasp_target(path)

    document["status"] = "PLAN_ONLY_PASS"
    document["execution_api_used"] = True
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="execution_api_used=false"):
        MODULE.load_pregrasp_target(path)


def test_segments_are_bounded_contiguous_and_reach_target():
    start = (0.0, 0.0, 0.0, 0.0, 0.0)
    target = (0.30, 1.77, 0.67, 1.28, 0.14)
    segments = MODULE.interpolate_segments(start, target, 0.30)
    assert len(segments) == 6
    assert segments[0][0] == start
    assert segments[-1][1] == target
    for index, (segment_start, segment_target) in enumerate(segments):
        assert max(
            abs(goal - current)
            for current, goal in zip(segment_start, segment_target, strict=True)
        ) <= 0.30
        if index:
            assert segments[index - 1][1] == segment_start


def test_maximum_step_cannot_exceed_stage7_gate():
    with pytest.raises(ValueError, match="within"):
        MODULE.interpolate_segments((0.0,) * 5, (0.1,) * 5, 0.31)


def test_request_uses_explicit_start_and_joint_constraints():
    start = (0.0, 0.1, 0.2, 0.3, 0.4)
    target = (0.1, 0.2, 0.3, 0.4, 0.5)
    request = MODULE.build_request(start, target).motion_plan_request
    assert request.start_state.is_diff is False
    assert tuple(request.start_state.joint_state.name) == MODULE.ARM_JOINTS
    assert tuple(request.start_state.joint_state.position) == start
    assert request.group_name == "left_arm"
    assert request.max_velocity_scaling_factor == 0.15
    assert request.max_acceleration_scaling_factor == 0.15
    constraints = request.goal_constraints[0].joint_constraints
    assert tuple(item.joint_name for item in constraints) == MODULE.ARM_JOINTS
    assert tuple(item.position for item in constraints) == target
    grasp_request = MODULE.build_request(start, target, "grasp")
    assert (
        grasp_request.motion_plan_request.goal_constraints[0].name
        == "bounded_grasp_segment"
    )


def test_tool_has_no_execution_action_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "ExecuteTrajectory" not in source
    assert "ActionClient" not in source
    assert MODULE.DEFAULT_MAX_JOINT_STEP_RAD == 0.30


def test_cli_separates_interpolation_step_from_execution_ceiling(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(MODULE_PATH),
            "--source-plan",
            "source.json",
            "--calibration",
            "calibration.json",
            "--start",
            "0,0,0,0,0",
            "--max-joint-step",
            "0.08",
            "--execution-step-limit",
            "0.15",
            "--output",
            "output.json",
            "--plan-only",
        ],
    )
    args = MODULE.parse_args()
    assert args.max_joint_step == pytest.approx(0.08)
    assert args.execution_step_limit == pytest.approx(0.15)


def test_execution_ceiling_defaults_to_interpolation_step(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(MODULE_PATH),
            "--source-plan",
            "source.json",
            "--calibration",
            "calibration.json",
            "--start",
            "0,0,0,0,0",
            "--max-joint-step",
            "0.08",
            "--output",
            "output.json",
            "--plan-only",
        ],
    )
    args = MODULE.parse_args()
    assert args.execution_step_limit == pytest.approx(0.08)


def test_execution_ceiling_cannot_be_below_interpolation_step(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(MODULE_PATH),
            "--source-plan", "source.json",
            "--calibration", "calibration.json",
            "--start", "0,0,0,0,0",
            "--max-joint-step", "0.10",
            "--execution-step-limit", "0.09",
            "--output", "output.json",
            "--plan-only",
        ],
    )
    with pytest.raises(SystemExit):
        MODULE.parse_args()
