import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.action_validation import GoalValidationError  # noqa: E402
from single_arm_bridge.buffered_transport_driver import (  # noqa: E402
    BufferedExchangeResponse,
)
from single_arm_bridge.protocol import MotionResult  # noqa: E402


SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_joint_delta_once",
    ROOT / "tools" / "execute_buffered_joint_delta_once.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

CALIBRATION = load_calibration(
    PACKAGE_ROOT / "config" / "single_arm_calibration.json"
)


def test_commissioning_tool_requires_bounded_lateness_firmware() -> None:
    assert MODULE.EXPECTED_FIRMWARE == 0x00022100


def test_plan_is_exactly_sixteen_samples_and_changes_only_selected_joint() -> None:
    plan, target_raw = MODULE.build_execution_plan(
        CALIBRATION,
        (2048,) * 6,
        joint_name="left_wrist_roll_joint",
        delta_rad=0.03,
        current_tick_ms=1000,
    )

    assert len(plan.samples) == 16
    assert plan.samples[0].trajectory_elapsed_ms == 0
    assert plan.samples[-1].trajectory_elapsed_ms == 300
    assert plan.samples[0].apply_tick_ms == 1220
    assert plan.anchor_tick_ms == 1200
    assert plan.samples[0].positions_urad == (0,) * 6
    assert plan.samples[-1].positions_urad[:4] == (0,) * 4
    assert plan.samples[-1].positions_urad[4] == 30000
    assert plan.samples[-1].positions_urad[5] == 0
    assert target_raw[:4] == (2048,) * 4
    assert target_raw[4] == 2068
    assert target_raw[5] == 2048


@pytest.mark.parametrize("delta", (0.0, 0.030001, -0.030001, float("nan")))
def test_delta_outside_commissioning_envelope_is_rejected(delta: float) -> None:
    with pytest.raises(ValueError, match="at most 0.03 rad"):
        MODULE.build_execution_plan(
            CALIBRATION,
            (2048,) * 6,
            joint_name="left_wrist_roll_joint",
            delta_rad=delta,
            current_tick_ms=1000,
        )


def test_gripper_is_not_an_arm_delta_target() -> None:
    with pytest.raises(ValueError, match="five arm joints"):
        MODULE.build_execution_plan(
            CALIBRATION,
            (2048,) * 6,
            joint_name="left_gripper_joint",
            delta_rad=0.01,
            current_tick_ms=1000,
        )


def test_sub_observable_delta_is_rejected_before_motion_authority() -> None:
    with pytest.raises(ValueError, match="not physically observable"):
        MODULE.build_execution_plan(
            CALIBRATION,
            (2048,) * 6,
            joint_name="left_wrist_roll_joint",
            delta_rad=0.015,
            current_tick_ms=1000,
        )


def test_observable_command_raw_boundary_is_fail_closed() -> None:
    with pytest.raises(ValueError, match="planned_delta_raw=15 minimum=16"):
        MODULE.require_observable_command(
            (2048,) * 6,
            (2048, 2048, 2048, 2048, 2063, 2048),
            joint_index=4,
        )

    assert MODULE.require_observable_command(
        (2048,) * 6,
        (2048, 2048, 2048, 2048, 2064, 2048),
        joint_index=4,
    ) == 16


def test_preflight_rejects_unsafe_preserved_axis_before_motion_authority() -> None:
    with pytest.raises(
        GoalValidationError,
        match="left_wrist_flex_joint position .* outside safe range",
    ):
        MODULE.preflight_execution_request(
            CALIBRATION,
            (2048, 2048, 2048, 2264, 2048, 2048),
            joint_name="left_wrist_roll_joint",
            delta_rad=0.015,
        )


def test_settle_requires_torque_on_and_reports_maximum_error() -> None:
    snapshot = SimpleNamespace(
        joints=tuple(
            SimpleNamespace(
                servo_id=index + 1,
                torque_enabled=True,
                position_raw=2048 + index,
            )
            for index in range(6)
        )
    )
    assert MODULE.maximum_settle_error_raw(snapshot, (2048,) * 6) == 5

    snapshot.joints[2].torque_enabled = False
    with pytest.raises(RuntimeError, match="torque disabled"):
        MODULE.maximum_settle_error_raw(snapshot, (2048,) * 6)


def _motion_snapshot(
    selected_position_raw: int,
    *,
    other_axis_error_raw: int = 0,
):
    positions = [2048 + other_axis_error_raw, 2048, 2048, 2048, 2048, 2048]
    positions[4] = selected_position_raw
    return SimpleNamespace(
        joints=tuple(
            SimpleNamespace(
                servo_id=index + 1,
                torque_enabled=True,
                position_raw=position,
            )
            for index, position in enumerate(positions)
        )
    )


def _observable_metrics(selected_position_raw: int, *, other_axis_error_raw=0):
    return MODULE.observable_motion_metrics(
        _motion_snapshot(
            selected_position_raw,
            other_axis_error_raw=other_axis_error_raw,
        ),
        (2048,) * 6,
        (2048, 2048, 2048, 2048, 2068, 2048),
        joint_index=4,
    )


def test_observable_motion_normal_tracking_passes() -> None:
    metrics = _observable_metrics(2063)

    assert metrics.planned_delta_raw == 20
    assert metrics.observed_delta_raw == 15
    assert metrics.target_error_raw == 5
    assert metrics.passed


def test_observable_motion_no_movement_is_rejected() -> None:
    metrics = _observable_metrics(2048)

    assert metrics.observed_delta_raw == 0
    assert not metrics.direction_ok
    assert not metrics.progress_ok
    assert not metrics.passed


def test_observable_motion_wrong_direction_is_rejected() -> None:
    metrics = _observable_metrics(2040)

    assert metrics.observed_delta_raw == -8
    assert not metrics.direction_ok
    assert not metrics.progress_ok
    assert not metrics.passed


def test_observable_motion_target_error_boundary_is_inclusive() -> None:
    passing = _observable_metrics(2060)
    failing = _observable_metrics(2059)

    assert passing.target_error_raw == 8
    assert passing.observed_delta_raw == 12
    assert passing.passed
    assert failing.target_error_raw == 9
    assert not failing.target_ok
    assert not failing.passed


def test_observable_motion_rejects_other_axis_outside_legacy_tolerance() -> None:
    metrics = _observable_metrics(2063, other_axis_error_raw=31)

    assert metrics.maximum_other_axis_error_raw == 31
    assert not metrics.other_axes_ok
    assert not metrics.passed


def test_disabled_gate_rejects_any_enabled_axis() -> None:
    disabled = SimpleNamespace(
        joints=tuple(
            SimpleNamespace(servo_id=index + 1, torque_enabled=False)
            for index in range(6)
        )
    )
    MODULE.require_disabled(disabled)
    disabled.joints[4].torque_enabled = True
    with pytest.raises(RuntimeError, match=r"servo IDs \[5\]"):
        MODULE.require_disabled(disabled)


class _DisableTransport:
    def __init__(self, snapshot) -> None:
        self.snapshot = snapshot
        self.disable_calls = 0
        self.diagnostic_calls = 0

    def disable(self) -> None:
        self.disable_calls += 1

    def get_diagnostics(self):
        self.diagnostic_calls += 1
        return self.snapshot


def test_failed_attempt_disables_without_heartbeat_diagnostics() -> None:
    transport = _DisableTransport(None)

    MODULE.disable_after_attempt(transport, verify_readback=False)

    assert transport.disable_calls == 1
    assert transport.diagnostic_calls == 0


def test_successful_attempt_disables_and_requires_six_axis_readback() -> None:
    snapshot = SimpleNamespace(
        joints=tuple(
            SimpleNamespace(servo_id=index + 1, torque_enabled=False)
            for index in range(6)
        )
    )
    transport = _DisableTransport(snapshot)

    MODULE.disable_after_attempt(transport, verify_readback=True)

    assert transport.disable_calls == 1
    assert transport.diagnostic_calls == 1


def test_terminal_diagnostic_includes_hold_cause_and_accounting() -> None:
    response = BufferedExchangeResponse(
        frame_sequence=41,
        result=MotionResult(
            status_code=6,
            sample_count=0,
            safety_state=2,
            detail=4,
            request_sequence=41,
            apply_tick_ms=1234,
            calibration_hash=0x2D90167E,
            executor_state=2,
            terminal_reason=4,
            safe_stop_required=True,
            queue_result=0,
            queued_samples=0,
            peak_queued_samples=16,
            accepted_samples=16,
            applied_samples=7,
        ),
    )

    diagnostic = MODULE.format_buffered_terminal(response)

    assert diagnostic.startswith("BUFFERED_TERMINAL_RECEIVED ")
    assert "frame_sequence=41" in diagnostic
    assert "executor_state=2" in diagnostic
    assert "terminal_reason=4" in diagnostic
    assert "safe_stop_required=True" in diagnostic
    assert "peak_queued=16" in diagnostic
    assert "accepted=16" in diagnostic
    assert "applied=7" in diagnostic
