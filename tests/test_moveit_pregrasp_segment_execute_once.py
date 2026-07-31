import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import JointState


TOOLS_ROOT = Path("tools")
sys.path.insert(0, str(TOOLS_ROOT))
SPEC = importlib.util.spec_from_file_location(
    "ros_moveit_execute_pregrasp_segment_once",
    TOOLS_ROOT / "ros_moveit_execute_pregrasp_segment_once.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def valid_plan() -> dict:
    return {
        "status": "PREGRASP_SEGMENT_PLAN_ONLY_PASS",
        "target_name": "pregrasp",
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "joint_names": list(MODULE.ARM_JOINTS),
        "max_joint_step_rad": 0.30,
        "interpolation_joint_step_rad": 0.10,
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


def write_plan(tmp_path: Path, document: dict) -> tuple[Path, str]:
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return path, digest


def test_load_segment_requires_exact_digest_and_fail_closed_source(
    tmp_path: Path,
) -> None:
    path, digest = write_plan(tmp_path, valid_plan())
    segment = MODULE.load_segment(path, 1, digest)
    assert segment.index == 1
    assert segment.target_name == "pregrasp"
    assert segment.target == (0.1,) * 5

    with pytest.raises(ValueError, match="sha256 mismatch"):
        MODULE.load_segment(path, 1, "0" * 64)

    document = valid_plan()
    document["motion_authorized"] = True
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="motion_authorized=false"):
        MODULE.load_segment(path, 1, digest)


def test_grasp_plan_contract_is_supported(tmp_path: Path) -> None:
    document = valid_plan()
    document["status"] = "GRASP_SEGMENT_PLAN_ONLY_PASS"
    document["target_name"] = "grasp"
    path, digest = write_plan(tmp_path, document)
    segment = MODULE.load_segment(path, 2, digest)
    assert segment.target_name == "grasp"
    assert segment.target == (0.2,) * 5

    document["target_name"] = "pregrasp"
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="inconsistent"):
        MODULE.load_segment(path, 2, digest)


def test_any_unsuccessful_segment_rejects_whole_source(tmp_path: Path) -> None:
    document = valid_plan()
    document["segments"][1]["success"] = False
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="unsuccessful"):
        MODULE.load_segment(path, 1, digest)


def test_inconsistent_or_oversized_step_is_rejected(tmp_path: Path) -> None:
    document = valid_plan()
    document["segments"][0]["maximum_joint_delta_rad"] = 0.2
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="inconsistent"):
        MODULE.load_segment(path, 1, digest)

    document = valid_plan()
    document["segments"][0]["target_positions_rad"] = [0.31] * 5
    document["segments"][0]["maximum_joint_delta_rad"] = 0.31
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="interpolation"):
        MODULE.load_segment(path, 1, digest)

    document = valid_plan()
    document["interpolation_joint_step_rad"] = 0.11
    document["max_joint_step_rad"] = 0.10
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="step contract"):
        MODULE.load_segment(path, 1, digest)


def test_plan_interpolation_and_fresh_execution_limits_are_separate(
    tmp_path: Path,
) -> None:
    document = valid_plan()
    document["interpolation_joint_step_rad"] = 0.10
    document["max_joint_step_rad"] = 0.15
    path, digest = write_plan(tmp_path, document)
    segment = MODULE.load_segment(path, 1, digest)
    assert segment.max_joint_step_rad == pytest.approx(0.15)

    document["segments"][0]["target_positions_rad"] = [0.101] * 5
    document["segments"][0]["maximum_joint_delta_rad"] = 0.101
    path, digest = write_plan(tmp_path, document)
    with pytest.raises(ValueError, match="interpolation"):
        MODULE.load_segment(path, 1, digest)


def test_joint_state_is_reordered_and_must_be_complete() -> None:
    message = JointState()
    message.name = [
        "left_wrist_roll_joint",
        "left_elbow_joint",
        "left_gripper_joint",
        "left_base_joint",
        "left_wrist_flex_joint",
        "left_shoulder_joint",
    ]
    message.position = [0.5, 0.3, 0.6, 0.1, 0.4, 0.2]
    assert MODULE.positions_from_joint_state(message) == (
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
    )

    message.name.remove("left_shoulder_joint")
    message.position.pop()
    with pytest.raises(ValueError, match="missing"):
        MODULE.positions_from_joint_state(message)


def test_current_start_gate_is_strict_and_reports_largest_joint() -> None:
    expected = (0.1,) * 5
    current = (0.11, 0.12, 0.09, 0.10, 0.13)
    assert MODULE.validate_current_start(current, expected) == pytest.approx(0.03)

    shoulder_within_dedicated_margin = (0.1, 0.154, 0.1, 0.1, 0.1)
    assert MODULE.validate_current_start(
        shoulder_within_dedicated_margin,
        expected,
    ) == pytest.approx(0.054)

    current = (0.1, 0.1, 0.1, 0.151, 0.1)
    with pytest.raises(ValueError, match="left_wrist_flex_joint"):
        MODULE.validate_current_start(current, expected)

    with pytest.raises(ValueError, match="left_shoulder_joint"):
        MODULE.validate_current_start(
            (0.1, 0.156, 0.1, 0.1, 0.1),
            expected,
        )


def test_actual_current_to_target_step_cannot_exceed_030_rad() -> None:
    current = (0.0,) * 5
    target = (0.29, 0.10, 0.0, 0.0, 0.0)
    assert MODULE.validate_actual_step(current, target) == pytest.approx(0.29)

    target = (0.31, 0.10, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="actual current-to-target"):
        MODULE.validate_actual_step(current, target)


def test_goal_is_one_point_and_source_contains_no_retry_loop() -> None:
    preset = MODULE.Preset(
        MODULE.ARM_CONTROLLER,
        MODULE.ARM_JOINTS,
        (0.1,) * 5,
        MODULE.EXECUTION_DURATION_S,
    )
    goal = MODULE.build_goal(preset)
    assert goal.controller_names == [MODULE.ARM_CONTROLLER]
    assert len(goal.trajectory.joint_trajectory.points) == 1

    source = Path(
        "tools/ros_moveit_execute_pregrasp_segment_once.py"
    ).read_text(encoding="utf-8")
    assert "send_goal_async" in source
    assert source.count("send_goal_async") == 1
    assert "for attempt" not in source
    assert source.count("require_healthy_diagnostics(node)") == 3


def test_soft_abort_post_settle_requires_control_failed_healthy_and_close() -> None:
    target = (0.1,) * 5
    current = (0.149, 0.154, 0.149, 0.149, 0.149)
    assert MODULE.validate_post_settle_soft_abort(
        MODULE.GoalStatus.STATUS_ABORTED,
        MODULE.MoveItErrorCodes.CONTROL_FAILED,
        current,
        target,
        True,
    ) == pytest.approx(0.054)

    with pytest.raises(ValueError, match="diagnostics"):
        MODULE.validate_post_settle_soft_abort(
            MODULE.GoalStatus.STATUS_ABORTED,
            MODULE.MoveItErrorCodes.CONTROL_FAILED,
            current,
            target,
            False,
        )
    with pytest.raises(ValueError, match="left_base_joint"):
        MODULE.validate_post_settle_soft_abort(
            MODULE.GoalStatus.STATUS_ABORTED,
            MODULE.MoveItErrorCodes.CONTROL_FAILED,
            (0.151,) * 5,
            target,
            True,
        )
    with pytest.raises(ValueError, match="left_shoulder_joint"):
        MODULE.validate_post_settle_soft_abort(
            MODULE.GoalStatus.STATUS_ABORTED,
            MODULE.MoveItErrorCodes.CONTROL_FAILED,
            (0.149, 0.156, 0.149, 0.149, 0.149),
            target,
            True,
        )


def valid_diagnostics(shoulder_temperature_c: int = 49) -> dict:
    joints = []
    for servo_id, name in MODULE.EXPECTED_SERVO_IDENTITIES:
        joints.append(
            {
                "name": name,
                "servo_id": servo_id,
                "torque_enabled": True,
                "temperature_c": (
                    shoulder_temperature_c
                    if name == "left_shoulder_joint"
                    else 40
                ),
            }
        )
    return {
        "protocol_version": MODULE.EXPECTED_PROTOCOL_VERSION,
        "joint_count": 6,
        "calibration_hash": MODULE.EXPECTED_CALIBRATION_HASH,
        "joints": joints,
    }


def test_diagnostics_gate_requires_all_axes_and_shoulder_below_50c() -> None:
    assert MODULE.parse_diagnostics_gate(
        json.dumps(valid_diagnostics(49))
    ) == 49

    with pytest.raises(ValueError, match="temperature gate"):
        MODULE.parse_diagnostics_gate(json.dumps(valid_diagnostics(50)))

    document = valid_diagnostics()
    document["joints"][4]["torque_enabled"] = False
    with pytest.raises(ValueError, match="torque disabled"):
        MODULE.parse_diagnostics_gate(json.dumps(document))


def test_diagnostics_gate_rejects_wrong_calibration_or_axis_identity() -> None:
    document = valid_diagnostics()
    document["calibration_hash"] = "0x00000000"
    with pytest.raises(ValueError, match="calibration identity"):
        MODULE.parse_diagnostics_gate(json.dumps(document))

    document = valid_diagnostics()
    document["joints"][0]["servo_id"] = 6
    with pytest.raises(ValueError, match="identity mismatch"):
        MODULE.parse_diagnostics_gate(json.dumps(document))
