from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

CORE_PATH = ROOT / "tools" / "top_pick_place_application.py"
RUNNER_PATH = ROOT / "tools" / "run_top_pick_place_application_once.py"
spec = importlib.util.spec_from_file_location("top_pick_place_application", CORE_PATH)
APP = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = APP
spec.loader.exec_module(APP)
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")


def sample(x=0.403679, y=-0.144727, z=0.0063, yaw=-0.12, confidence=0.9):
    return APP.BaseTargetSample(x, y, z, yaw, confidence)


def test_exact_hardware_proven_inputs_are_pinned() -> None:
    manifest, center = APP.load_application_inputs(
        ROOT / "artifacts/h2/2026-08-12/offset011_nominal_routes/manifest.json",
        ROOT / "artifacts/h2/2026-08-12/offset011_nominal_routes/report.json",
        ROOT / "artifacts/h2/2026-08-12/offset011_h1_rate200_run01/result.json",
        ROOT / "artifacts/h2/2026-08-12/pick_pose_plan_only_offset011.json",
    )
    assert manifest["command_step_count"] == 39
    assert center == pytest.approx(
        (0.403678997178, -0.144727082535, 0.0063, -0.120319231991)
    )


def test_stable_camera_lock_matches_the_proven_pick() -> None:
    samples = [sample(x=0.403679 + offset) for offset in (-0.001, 0, 0.001, 0, 0)]
    locked = APP.lock_target(samples)
    error, yaw_error = APP.validate_locked_target(
        locked, (0.403679, -0.144727, 0.0063, -0.12)
    )
    assert error < 1e-9
    assert yaw_error < 1e-9
    assert locked.maximum_position_spread_m == pytest.approx(0.001)


def test_camera_lock_rejects_unstable_or_wrong_targets() -> None:
    with pytest.raises(APP.TopPickPlaceContractError, match="unstable"):
        APP.lock_target([sample(x=0.403679 + i * 0.002) for i in range(5)])
    locked = APP.lock_target([sample(x=0.42) for _ in range(5)])
    with pytest.raises(APP.TopPickPlaceContractError, match="does not match"):
        APP.validate_locked_target(
            locked, (0.403679, -0.144727, 0.0063, -0.12)
        )


def test_camera_lock_rejects_wrong_undirected_yaw() -> None:
    locked = APP.lock_target([sample(yaw=0.4) for _ in range(5)])
    with pytest.raises(APP.TopPickPlaceContractError, match="yaw does not match"):
        APP.validate_locked_target(
            locked, (0.403679, -0.144727, 0.0063, -0.12)
        )


def test_camera_pixel_routes_left_or_right_with_center_deadband() -> None:
    assert APP.select_arm_for_pixel(100.0, 640, 40.0) == "left"
    assert APP.select_arm_for_pixel(540.0, 640, 40.0) == "right"
    with pytest.raises(APP.TopPickPlaceContractError, match="routing deadband"):
        APP.select_arm_for_pixel(320.0, 640, 40.0)
    with pytest.raises(APP.TopPickPlaceContractError, match="routing deadband"):
        APP.select_arm_for_pixel(300.0, 640, 40.0)


def test_camera_arm_selection_must_remain_stable_during_lock() -> None:
    assert APP.require_consistent_arm_selection(["right"] * 7) == "right"
    with pytest.raises(APP.TopPickPlaceContractError, match="changed"):
        APP.require_consistent_arm_selection(["left", "right"])


def test_conservative_workspace_is_expressed_in_the_selected_arm_frame() -> None:
    assert APP.workspace_coordinates_for_arm(0.40, -0.10, "left") == pytest.approx(
        (0.40, -0.10)
    )
    assert APP.workspace_coordinates_for_arm(
        0.40, -0.10, "right", 0.0063
    ) == pytest.approx((0.40, 0.132064146), abs=1e-9)


def test_left_q0_is_required_but_right_arm_may_hold_any_valid_pose() -> None:
    APP.validate_q0_anchor((0.0,) * 6 + (1.0, 1.1, 1.2, 0.1, 0.2, 0.0))
    with pytest.raises(APP.TopPickPlaceContractError, match="not at q0"):
        APP.validate_q0_anchor((0.0, 0.05, 0.0, 0.0, 0.0, 0.0) + (0.0,) * 6)


def test_left_arm_step_freezes_gripper_and_the_entire_right_arm() -> None:
    current = tuple(i / 10 for i in range(12))
    right = current[6:]
    target = APP.step_target(
        current,
        {"kind": "arm", "target_positions_rad": [0.1, 0.2, 0.3, 0.4, 0.5]},
        right,
    )
    assert target[:5] == (0.1, 0.2, 0.3, 0.4, 0.5)
    assert target[5] == current[5]
    assert target[6:] == right


def test_right_arm_step_freezes_gripper_and_the_entire_left_arm() -> None:
    current = tuple(i / 10 for i in range(12))
    left = current[:6]
    target = APP.step_target(
        current,
        {"kind": "arm", "target_positions_rad": [0.1, 0.2, 0.3, 0.4, 0.5]},
        left,
        arm="right",
    )
    assert target[:6] == left
    assert target[6:11] == (0.1, 0.2, 0.3, 0.4, 0.5)
    assert target[11] == current[11]


def test_bimanual_q0_target_preserves_grippers_and_zeros_both_arms() -> None:
    start = tuple(float(index + 1) / 10.0 for index in range(12))
    target = APP.bimanual_q0_target(start)
    assert target[:5] == (0.0,) * 5
    assert target[5] == start[5]
    assert target[6:11] == (0.0,) * 5
    assert target[11] == start[11]


def test_bimanual_q0_validation_checks_both_arm_chains_only() -> None:
    positions = (0.01,) * 5 + (1.2,) + (0.02,) * 5 + (1.3,)
    assert APP.validate_bimanual_q0(positions) == pytest.approx(0.02)
    assert APP.validate_bimanual_q0(
        (0.0,) * 5 + (1.2,) + (0.046,) + (0.0,) * 4 + (1.3,)
    ) == pytest.approx(0.046)
    with pytest.raises(APP.TopPickPlaceContractError, match="both arms"):
        APP.validate_bimanual_q0(
            (0.0,) * 5 + (1.2,) + (0.047,) + (0.0,) * 4 + (1.3,)
        )


def test_selected_arm_q0_gate_checks_the_selected_side_only() -> None:
    APP.validate_selected_q0_anchor((1.0,) * 6 + (0.0,) * 6, "right")
    with pytest.raises(APP.TopPickPlaceContractError, match="right arm is not at q0"):
        APP.validate_selected_q0_anchor((0.0,) * 6 + (0.0, 0.05, 0.0, 0.0, 0.0, 0.0), "right")


def test_finite_points_are_12_axis_5ms_grid_and_end_exactly_at_target() -> None:
    start = (0.0,) * 12
    target = tuple(0.18 if index == 1 else 0.0 for index in range(12))
    points = APP.interpolate_finite_points(start, target)
    assert len(points) == 8
    assert points[0][0] == 50
    assert points[-1] == (400, target)
    for (left_ms, left), (right_ms, right) in zip(points, points[1:]):
        assert right_ms - left_ms == 50
        assert max(abs(b - a) for a, b in zip(left, right)) <= 0.0225 + 1e-12


def test_reviewed_checkpoint_is_split_below_the_proven_rate() -> None:
    start = (0.0,) * 12
    target = tuple(0.18 if index == 1 else 0.0 for index in range(12))
    subtargets = APP.split_finite_targets(start, target)
    assert len(subtargets) == 2
    previous = start
    for current in subtargets:
        assert max(abs(b - a) for a, b in zip(previous, current)) <= 0.12 + 1e-12
        previous = current
    assert subtargets[-1] == target


def test_finite_rate_stays_within_the_physically_proven_200_raw_s() -> None:
    assert APP.MAXIMUM_FINITE_DELTA_RAD == pytest.approx(0.12)
    commanded_rate_rad_s = (
        APP.MAXIMUM_FINITE_DELTA_RAD
        / APP.FINITE_POINT_OFFSETS_MS[-1]
        * 1000.0
    )
    proven_rate_rad_s = 200.0 * APP.RAW_STEP_RAD
    assert commanded_rate_rad_s <= proven_rate_rad_s


def test_gripper_contact_uses_the_existing_raw_threshold() -> None:
    measured = 0.130388 - 21 * APP.RAW_STEP_RAD
    assert APP.residual_raw(0.130388, measured) == 21
    assert APP.residual_raw(0.130388, measured) >= APP.CONTACT_THRESHOLD_RAW


def test_runtime_has_one_confirmation_no_retry_and_fail_closed_stop() -> None:
    assert 'CONFIRMATION = "RUN_TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE"' in RUNNER_SOURCE
    assert 'RIGHT_PLACE_CONFIRMATION = "RUN_RIGHT_PLACE_HEIGHT_CHECK_ONCE"' in RUNNER_SOURCE
    assert '"right_place_height_check_mode"' in RUNNER_SOURCE
    assert '"right_place_height_check_motion_completed"' in RUNNER_SOURCE
    assert '"right_place_height_operator_observation_required"' in RUNNER_SOURCE
    assert '"automatic_retry_count": 0' in RUNNER_SOURCE
    assert "if motion_started and not successful_hold" in RUNNER_SOURCE
    assert "CONTINUOUS_COMMAND_RATE_RAD_S = 200.0 * RAW_STEP_RAD" in RUNNER_SOURCE
    assert "continuous_finite_request(commanded, [q0_target])" in RUNNER_SOURCE
    assert "terminal_anchor_settle_evidence" in RUNNER_SOURCE
    assert "resident_ready_fresh_feedback_fallback" in RUNNER_SOURCE
    assert "firmware_consecutive_joint_pairs" in RUNNER_SOURCE
    assert (
        "finite leg completed without terminal anchor or fresh feedback"
        in RUNNER_SOURCE
    )
    assert "actions = continuous_actions(plan)" in RUNNER_SOURCE
    assert "proven_three_continuous_arm_legs_with_gripper_stops" in RUNNER_SOURCE
    assert "TOP_PICK_PLACE_CONTINUOUS_ACTION_PASS" in RUNNER_SOURCE
    assert "commanded = measured" in RUNNER_SOURCE
    assert "TOP_PICK_PLACE_FAILURE_HOLD_PRESERVED" in RUNNER_SOURCE
    assert "TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE_FAIL_HOLDING" in RUNNER_SOURCE
    assert "if not hold_preserved" in RUNNER_SOURCE
    assert "TOP_PICK_PLACE_BIMANUAL_Q0_CONTINUOUS_HOLD_PASS" in RUNNER_SOURCE
    assert '"torque_hold_active"' in RUNNER_SOURCE
    assert '"coordinated_stop_sent"' in RUNNER_SOURCE
    assert '"failure_hold_preserved"' in RUNNER_SOURCE
    assert "stop_request()" in RUNNER_SOURCE
    assert "serial.Serial" not in RUNNER_SOURCE
    assert "current_positions = startup_anchor" in RUNNER_SOURCE
    assert "resident_immediate_pre_motion_anchor" in RUNNER_SOURCE
    assert "resident_armed_status_terminal_anchor" in RUNNER_SOURCE
    assert "same_owner_armed_ready_reuses_status_terminal_anchor" in RUNNER_SOURCE
    assert "if initial_owner is None:" in RUNNER_SOURCE
    assert 'REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"' in RUNNER_SOURCE
    assert "TOP_PICK_PLACE_FRESH_ANCHOR_PASS" in RUNNER_SOURCE
    assert "q0_settle_source" in RUNNER_SOURCE
    assert "open_exclusive_serial" not in RUNNER_SOURCE


def test_validate_only_returns_before_ros_and_resident_access() -> None:
    start = RUNNER_SOURCE.index("    if args.validate_only:", RUNNER_SOURCE.index("def main()"))
    ros_init = RUNNER_SOURCE.index("    rclpy.init()", start)
    validate_only = RUNNER_SOURCE[start:ros_init]
    assert '"resident_services_called": 0' in validate_only
    assert '"motion_commands": 0' in validate_only
    assert "return 0" in validate_only
    assert "create_client" not in validate_only
    assert "refresh_anchor" not in validate_only


def test_runtime_prefers_terminal_anchor_and_bounds_fresh_feedback_fallback() -> None:
    ready = RUNNER_SOURCE[
        RUNNER_SOURCE.index("def wait_until_ready("):
        RUNNER_SOURCE.index("\ndef stop_request()")
    ]
    assert "terminal_anchors: list[JointState]" in ready
    assert "if terminal_anchors:" in ready
    assert "feedback_messages.clear()" in ready
    assert "resident_status_terminal_anchor" in ready
    assert "status_prepared_positions(" in ready
    assert "resident_ready_fresh_feedback_fallback" in ready
    assert "feedback_positions(topic_feedback, label=\"terminal\")" in ready
    assert "MAXIMUM_FALLBACK_FEEDBACK_AGE_MS = 150" in RUNNER_SOURCE
    assert "int(feedback.present_mask) != 0x0FFF" in RUNNER_SOURCE
    assert "maximum_age_ms > MAXIMUM_FALLBACK_FEEDBACK_AGE_MS" in RUNNER_SOURCE
    assert RUNNER_SOURCE.count("anchors.clear()") >= 2

def test_runtime_requires_q0_locked_wrist() -> None:
    assert "REQUIRED_ENDPOINTS" in RUNNER_SOURCE
    assert 'plan.get("schema_version") != 12' in RUNNER_SOURCE
    assert (
        'endpoint.get("wrist_roll_yaw_correction_applied") is not False'
        in RUNNER_SOURCE
    )
    assert "MAXIMUM_LOCKED_WRIST_ROLL_RAD" in RUNNER_SOURCE
    assert "for name in ENDPOINT_SEQUENCE" in RUNNER_SOURCE
    assert '"hold_bimanual_q0"' in RUNNER_SOURCE
    assert "attempts to move the locked wrist" in RUNNER_SOURCE


def test_runtime_requires_the_reviewed_third_three_mm_lower_grasp() -> None:
    assert "EXPECTED_BASELINE_PICK_GRASP_OFFSET_M = 0.011" in RUNNER_SOURCE
    assert "EXPECTED_PREVIOUS_PICK_GRASP_OFFSET_M = 0.002" in RUNNER_SOURCE
    assert "EXPECTED_PICK_GRASP_OFFSET_M = -0.001" in RUNNER_SOURCE
    assert (
        "EXPECTED_PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M = 0.012"
        in RUNNER_SOURCE
    )
    assert "dynamic camera plan grasp-height contract is invalid" in RUNNER_SOURCE



def test_each_arm_plan_requires_operator_screen_lateral_correction() -> None:
    planner_source = (
        ROOT / "tools/plan_top_camera_pick_place_once.py"
    ).read_text(encoding="utf-8")
    assert "LEFT_SCREEN_X_CORRECTION_M = 0.01372" in planner_source
    assert "RIGHT_SCREEN_X_CORRECTION_M = -0.02947" in planner_source
    assert "screen_positive_x_unit_workcell" in planner_source
    assert '"schema_version": 12' in planner_source
    assert "EXPECTED_LEFT_SCREEN_X_CORRECTION_M = 0.01372" in RUNNER_SOURCE
    assert "EXPECTED_RIGHT_SCREEN_X_CORRECTION_M = -0.02947" in RUNNER_SOURCE
    assert "dynamic camera plan {side} lateral contract is invalid" in RUNNER_SOURCE

def test_open_grasp_height_check_never_closes_and_returns_via_pregrasp() -> None:
    assert "OPEN_GRASP_HEIGHT_CHECK_CONFIRMATION" in RUNNER_SOURCE
    assert "--open-grasp-height-check" in RUNNER_SOURCE
    assert "TOP_OPEN_GRASP_HEIGHT_CHECK_HOLD" in RUNNER_SOURCE
    assert "height_check_close_commands" in RUNNER_SOURCE
    assert "height_check_return_via_pregrasp" in RUNNER_SOURCE
    assert "pick_pregrasp" in RUNNER_SOURCE
    assert "[pregrasp_target, q0_return_target]" in RUNNER_SOURCE
    assert "TOP_CAMERA_OPEN_GRASP_HEIGHT_CHECK_PASS_HOLDING" in RUNNER_SOURCE


def test_runtime_requires_the_deeper_reviewed_gripper_target() -> None:
    assert "EXPECTED_GRIPPER_OPEN_TARGET_RAW = 2048" in RUNNER_SOURCE
    assert "EXPECTED_GRIPPER_CLOSE_TARGET_RAW = 1948" in RUNNER_SOURCE
    assert 'plan["steps"][0] is not pick_open_steps[0]' in RUNNER_SOURCE
    assert 'action["label"] in ("pick_open", "place_release")' in RUNNER_SOURCE
    assert 'plan.get("gripper_contract", {})' in RUNNER_SOURCE
    assert "dynamic camera plan gripper contract is invalid" in RUNNER_SOURCE


def test_resident_right_q0_tool_holds_other_axes_and_fails_closed() -> None:
    source = (
        ROOT / "tools" / "execute_resident_right_arm_q0_once.py"
    ).read_text(encoding="utf-8")
    assert "RIGHT_ARM_INDICES = (6, 7, 8, 9, 10)" in source
    assert "MAXIMUM_SUBLEG_DELTA_RAD = 0.075" in source
    assert "target[index] = Q0_RAD" in source
    assert "for index in RIGHT_ARM_INDICES" in source
    assert "maximum_final_residual > Q0_TOLERANCE_RAD" in source
    assert "if motion_request_sent and not stop_accepted" in source
    assert "BimanualStreamCommand.Request.STOP" in source
    assert "0x00024806" in source
