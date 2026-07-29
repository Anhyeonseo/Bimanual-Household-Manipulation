import importlib.util
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "ros_moveit_plan_grasp.py"
SPEC = importlib.util.spec_from_file_location("ros_moveit_plan_grasp", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_plan_only_tool_has_no_execution_action_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "ExecuteTrajectory" not in source
    assert "ActionClient" not in source
    assert MODULE.PLAN_SERVICE == "/plan_kinematic_path"


def test_top_down_quaternion_is_normalized_and_points_z_down():
    qx, qy, qz, qw = MODULE.top_down_quaternion(math.radians(30.0))
    assert math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) == 1.0
    assert qz == 0.0
    assert qw == 0.0


def test_request_is_bounded_and_uses_current_state_diff():
    request = MODULE.build_request(
        0.37,
        -0.13,
        0.10,
        0.0,
        0.006,
        math.radians(25.0),
    ).motion_plan_request
    assert request.group_name == "left_arm"
    assert request.start_state.is_diff is True
    assert request.allowed_planning_time == 5.0
    assert request.max_velocity_scaling_factor == 0.20
    assert request.max_acceleration_scaling_factor == 0.20
    assert len(request.goal_constraints) == 1
    assert request.goal_constraints[0].orientation_constraints == []
    assert (
        request.goal_constraints[0].position_constraints[0].link_name
        == "left_gripper_frame_link"
    )
