"""J2 execution is restricted to one right-arm axis and fail-safe services."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/execute_right_arm_j2_axis_roundtrip_once.py"
SOURCE = TOOL.read_text(encoding="utf-8")
BRIDGE = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bridge_node.py"
).read_text(encoding="utf-8")


def test_exact_operator_and_power_isolation_confirmations_are_required() -> None:
    assert 'CONFIRMATION = "J2_RIGHT_ARM_AXIS_ROUNDTRIP_ONCE"' in SOURCE
    assert "--left-arm-12v-off-confirmed" in SOURCE
    assert "--operator-support-confirmed" in SOURCE
    assert "--targets-sha256" in SOURCE
    assert "--j0-envelope-sha256" in SOURCE
    assert "--candidate-sha256" in SOURCE
    assert "EXPECTED_APPROVED_SHA256" in SOURCE
    assert "EXPECTED_J0_SHA256" in SOURCE


def test_only_bounded_single_servo_primitives_are_used() -> None:
    for imported_service_constant in (
        "CONFIGURE_SERVICE",
        "CONFIGURATION_SERVICE",
        "TORQUE_SERVICE",
        "JOG_SERVICE",
    ):
        assert imported_service_constant in SOURCE
    for local_service in (
        "/right_arm_disable",
        "/right_arm_stop",
    ):
        assert local_service in SOURCE
    assert "MAX_CYCLES_PER_LEG = 100" in SOURCE
    assert '"maximum_jog_step_raw": 20' in SOURCE
    assert '"multi_joint_commands_forbidden": True' in SOURCE
    assert '"general_trajectory_authorized": False' in SOURCE
    for forbidden in ("ActionClient", "FollowJointTrajectory", "trajectory_msgs"):
        assert forbidden not in SOURCE


def test_health_tracking_and_non_selected_joint_checks_are_present() -> None:
    for required in (
        "voltage_raw",
        "temperature_c",
        "load_raw",
        "current_raw",
        "protection_current_raw",
        "MAX_COMMAND_TRACKING_ERROR_RAW",
        "MAX_OTHER_JOINT_DRIFT_RAW",
        "passive_baseline_interval_raw",
        "passive_other_intervals",
        "J2_SELECTED_Q0_PASSIVE_BASELINE_PASS",
        "STALL_CYCLE_LIMIT",
        "TARGET_CONSECUTIVE_SAMPLES",
        "MAX_PRECOMMAND_DRIFT_RAW",
        "MAX_PRECOMMAND_DRIFT_RAW = SETTLE_TOLERANCE_RAW",
        "SETTLE_S = 0.35",
        "precommand_drift_raw",
    ):
        assert required in SOURCE


def test_targets_must_fit_both_j1l_and_validated_r21_primitive_limits() -> None:
    assert "J2 target is not strictly inside J1-L" in SOURCE
    assert "currently validated" in SOURCE
    assert "J2_PREFLIGHT_VERIFIED_DISABLE_PASS torque_mask=0x00" in SOURCE
    assert "primitive_minimum_raw" in SOURCE
    assert "primitive_maximum_raw" in SOURCE


def test_success_disables_without_latching_and_failures_latch_stop() -> None:
    assert "disable_response = call_trigger(disable_client)" in SOURCE
    assert "post_disable = read_all(0)" in SOURCE
    assert "def call_pre_motion_disable" in SOURCE
    assert "if motion_started or not preflight_verified" in SOURCE
    assert "elif not call_pre_motion_disable(reason)" in SOURCE
    assert "call_stop(reason)" in SOURCE
    assert "motion_started = False" in SOURCE
    disable_handler = BRIDGE[
        BRIDGE.index("def _on_right_arm_disable"):
        BRIDGE.index("def _on_right_arm_stop")
    ]
    stop_handler = BRIDGE[
        BRIDGE.index("def _on_right_arm_stop"):
        BRIDGE.index("def _on_clear_fault")
    ]
    assert "disable_right_arm_verified" in disable_handler
    assert "safe_stop" not in disable_handler
    assert "safe_stop" in stop_handler


def test_gravity_sagged_elbow_is_passive_only_and_inside_reviewed_j0d() -> None:
    j0 = json.loads(
        (ROOT / "config/bimanual_j0_desired_envelope.reviewed.json").read_text()
    )
    targets = json.loads(
        (ROOT / "artifacts/joint_ranges/2026-08-13/j2_axis_targets_plan_only.json").read_text()
    )
    passive = j0["arms"]["right"]["joints"]["elbow"]
    commanded = targets["arms"]["right"]["joints"]["elbow"]
    assert passive["observed_minimum"] <= 2520 <= passive["observed_maximum"]
    assert 2520 > commanded["approved_maximum_unwrapped_raw"]
