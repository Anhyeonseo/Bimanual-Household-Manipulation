from dataclasses import replace
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.action_validation import (  # noqa: E402
    GoalValidationError,
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (  # noqa: E402
    INITIAL_FIRST_SAMPLE_LEAD_MS,
    MAXIMUM_APPLY_LATENESS_MS,
    SAMPLE_PERIOD_MS,
    STARTUP_PRIME_MINIMUM_ELAPSED_MS,
    BufferedActionAdapterError,
    BufferedAdapterState,
    BufferedBatchScheduler,
    BufferedExecutorState,
    BufferedTerminalReason,
    prepare_buffered_execution_plan,
    reanchor_buffered_execution_plan,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.protocol import (  # noqa: E402
    BufferedSetpointFlags,
    BufferedSetpointSample,
    Frame,
    MessageType,
    MotionResult,
    encode_buffered_setpoint_payload,
    encode_frame,
)


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


def validated_path(duration_ms: int = 800, target: float = 0.08):
    calibration = load_calibration(CALIBRATION_PATH)
    names = tuple(calibration.ros_joint_names[:5])
    positions = {name: calibration.ros_radian_limits[name] for name in names}
    velocity = {name: 0.5 for name in names}
    acceleration = {name: 1.0 for name in names}
    points = (
        TrajectoryPointData((0.0,) * 5, 0),
        TrajectoryPointData((target,) * 5, duration_ms * 1_000_000),
    )
    trajectory = validate_buffered_trajectory(
        names,
        points,
        names,
        positions,
        (0.0,) * 5,
        velocity,
        acceleration,
        start_tolerance_rad=0.01,
    )
    return calibration, trajectory


def scheduler(duration_ms: int = 800, *, current_tick_ms: int = 1_000):
    calibration, trajectory = validated_path(duration_ms)
    plan = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=0.06,
        current_tick_ms=current_tick_ms,
    )
    return plan, BufferedBatchScheduler(plan)


def test_reanchor_preserves_positions_and_uses_fresh_tick() -> None:
    calibration, trajectory = validated_path(duration_ms=800)
    provisional = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=0.06,
        current_tick_ms=10,
    )

    rebased = reanchor_buffered_execution_plan(
        provisional,
        current_tick_ms=50_000,
    )

    assert rebased.anchor_tick_ms == 50_200
    assert rebased.samples[0].apply_tick_ms == (
        50_000 + INITIAL_FIRST_SAMPLE_LEAD_MS
    )
    assert rebased.samples[-1].apply_tick_ms == 51_020
    assert tuple(sample.positions_urad for sample in rebased.samples) == tuple(
        sample.positions_urad for sample in provisional.samples
    )
    assert tuple(
        sample.trajectory_elapsed_ms for sample in rebased.samples
    ) == tuple(
        sample.trajectory_elapsed_ms for sample in provisional.samples
    )


def test_reanchor_handles_uint32_wraparound() -> None:
    plan, _ = scheduler(duration_ms=300)
    rebased = reanchor_buffered_execution_plan(
        plan,
        current_tick_ms=0xFFFFFFC0,
    )

    assert rebased.anchor_tick_ms == 136
    assert rebased.samples[0].apply_tick_ms == 156
    assert rebased.samples[-1].apply_tick_ms == 456


def ack_pending(
    value: BufferedBatchScheduler,
    batch,
    *,
    applied: int,
) -> None:
    assert value.snapshot().pending_batch
    accepted = batch.accepted_samples_after_ack
    value.acknowledge_batch(
        status_code=0,
        accepted_samples=accepted,
        applied_samples=applied,
        queued_samples=accepted - applied,
    )


def prime(value: BufferedBatchScheduler, *, current_tick_ms: int = 1_000):
    first = value.next_batch(current_tick_ms=current_tick_ms)
    assert first is not None
    ack_pending(value, first, applied=0)
    second = value.next_batch(
        current_tick_ms=current_tick_ms + STARTUP_PRIME_MINIMUM_ELAPSED_MS
    )
    assert second is not None
    ack_pending(value, second, applied=0)
    return first, second


def extended_result(
    *,
    status: int,
    sample_count: int,
    apply_tick_ms: int,
    executor_state: int,
    terminal_reason: int,
    safe_stop_required: bool,
    queued: int,
    accepted: int,
    applied: int,
    calibration_hash: int = 0xB317C672,
    detail: int = 0,
) -> MotionResult:
    return MotionResult(
        status_code=status,
        sample_count=sample_count,
        safety_state=3,
        detail=detail,
        request_sequence=123,
        apply_tick_ms=apply_tick_ms,
        calibration_hash=calibration_hash,
        executor_state=executor_state,
        terminal_reason=terminal_reason,
        safe_stop_required=safe_stop_required,
        queue_result=0,
        queued_samples=queued,
        peak_queued_samples=16,
        accepted_samples=accepted,
        applied_samples=applied,
    )


def test_resamples_at_20ms_with_220ms_initial_lead_and_preserves_gripper() -> None:
    plan, _ = scheduler()

    assert plan.anchor_tick_ms == 1_200
    assert len(plan.samples) == 41
    assert plan.samples[0].trajectory_elapsed_ms == 0
    assert plan.samples[0].apply_tick_ms == 1_220
    assert plan.samples[0].positions_urad[:5] == (0,) * 5
    assert plan.samples[0].positions_urad[5] == 60_000
    assert plan.samples[1].trajectory_elapsed_ms == 20
    assert plan.samples[1].apply_tick_ms == 1_240
    assert plan.samples[1].positions_urad[:5] == (2_000,) * 5
    assert plan.samples[-1].trajectory_elapsed_ms == 800
    assert plan.samples[-1].positions_urad[:5] == (80_000,) * 5


def test_plan_rejects_duration_too_short_for_prime() -> None:
    with pytest.raises(GoalValidationError):
        validated_path(280, target=0.01)


def test_plan_rounds_non_20ms_duration_up_to_sample_boundary() -> None:
    calibration, trajectory = validated_path(330, target=0.01)
    plan = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=0.06,
        current_tick_ms=1_000,
    )

    assert trajectory.duration_ms == 340
    assert len(plan.samples) == 18


def test_initial_prime_is_9_plus_7_and_starts_only_at_depth_16() -> None:
    _, value = scheduler()
    first, second = prime(value)

    assert first.sample_count == 9
    assert first.first_sample_index == 1
    assert first.samples[0].tick_offset_ms == 0
    assert first.samples[-1].tick_offset_ms == 160
    assert first.flags == (
        BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.BEGIN
    )
    assert second.sample_count == 7
    assert second.first_sample_index == 10
    assert second.flags == (
        BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.START
    )
    assert value.state is BufferedAdapterState.RUNNING
    assert value.snapshot().queued_samples == 16


def test_watermark_refills_from_10_to_16() -> None:
    _, value = scheduler()
    prime(value)
    value.record_applied(6)

    refill = value.next_batch(current_tick_ms=1_261)
    assert refill is not None
    assert refill.sample_count == 6
    assert refill.first_sample_index == 17
    ack_pending(value, refill, applied=6)
    assert value.snapshot().queued_samples == 16
    assert value.next_batch(current_tick_ms=1_280) is None


def test_80ms_outage_refill_uses_9_plus_2_without_gap() -> None:
    _, value = scheduler()
    prime(value)
    value.record_applied(11)

    first = value.next_batch(current_tick_ms=1_378)
    assert first is not None and first.sample_count == 9
    ack_pending(value, first, applied=11)
    assert value.snapshot().queued_samples == 14

    second = value.next_batch(current_tick_ms=1_396)
    assert second is not None and second.sample_count == 2
    ack_pending(value, second, applied=11)
    assert value.snapshot().queued_samples == 16


def test_pending_batch_is_not_returned_or_retransmitted() -> None:
    _, value = scheduler()
    value.next_batch(current_tick_ms=1_000)

    with pytest.raises(BufferedActionAdapterError, match="cannot be retransmitted"):
        value.next_batch(current_tick_ms=1_001)
    assert value.snapshot().pending_batch is True


def test_rejected_ack_aborts_without_retry() -> None:
    _, value = scheduler()
    value.next_batch(current_tick_ms=1_000)

    with pytest.raises(BufferedActionAdapterError, match="rejected or mismatched"):
        value.acknowledge_batch(
            status_code=1,
            accepted_samples=0,
            applied_samples=0,
            queued_samples=0,
        )
    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.ABORTED
    assert snapshot.safe_stop_required is True
    with pytest.raises(BufferedActionAdapterError, match="terminal"):
        value.next_batch(current_tick_ms=1_020)


def test_late_refill_aborts_before_a_frame_can_be_sent() -> None:
    _, value = scheduler()
    prime(value)
    value.record_applied(6)

    with pytest.raises(BufferedActionAdapterError, match="below 60 ms"):
        value.next_batch(current_tick_ms=1_485)
    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.ABORTED
    assert snapshot.reason == "batch_lead_outside_reviewed_window"
    assert snapshot.pending_batch is False


def test_early_second_prime_waits_until_maximum_horizon() -> None:
    _, value = scheduler()
    first = value.next_batch(current_tick_ms=1_000)
    assert first is not None
    value.acknowledge_batch(
        status_code=0,
        accepted_samples=9,
        applied_samples=0,
        queued_samples=9,
    )

    assert value.next_batch(current_tick_ms=1_080) is None
    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.PRIMING
    assert snapshot.pending_batch is False
    assert value.next_batch(current_tick_ms=1_120) is not None


def test_underflow_and_cancel_are_terminal_fail_closed() -> None:
    _, underflow = scheduler()
    prime(underflow)
    with pytest.raises(BufferedActionAdapterError, match="underflow"):
        underflow.record_applied(16)
    assert underflow.snapshot().reason == "queue_underflow"

    _, canceled = scheduler()
    prime(canceled)
    canceled.cancel()
    assert canceled.state is BufferedAdapterState.CANCELED
    assert canceled.snapshot().safe_stop_required is True
    with pytest.raises(BufferedActionAdapterError, match="terminal"):
        canceled.next_batch(current_tick_ms=1_100)


def test_final_prime_batch_marks_start_end_and_completion() -> None:
    _, value = scheduler(duration_ms=300)
    first, second = prime(value)

    assert BufferedSetpointFlags.END not in first.flags
    assert second.flags == (
        BufferedSetpointFlags.CANDIDATE
        | BufferedSetpointFlags.START
        | BufferedSetpointFlags.END
    )
    assert value.state is BufferedAdapterState.INPUT_COMPLETE
    value.record_applied(16)
    assert value.state is BufferedAdapterState.SUCCEEDED


def test_uint32_tick_wrap_preserves_lead_and_offsets() -> None:
    current = 0xFFFFFFB0
    plan, value = scheduler(current_tick_ms=current)

    assert plan.anchor_tick_ms == 120
    assert plan.samples[0].apply_tick_ms == 140
    first = value.next_batch(current_tick_ms=current)
    assert first is not None
    assert first.first_apply_tick_ms == 140
    assert first.samples[-1].tick_offset_ms == 160


def test_220ms_lead_covers_physical_9_plus_7_uart_wire_budget() -> None:
    def encoded_command_size(sample_count: int, flags: int) -> int:
        samples = tuple(
            BufferedSetpointSample(index * 20, (0,) * 6)
            for index in range(sample_count)
        )
        payload = encode_buffered_setpoint_payload(1_000, samples)
        return len(
            encode_frame(
                Frame(
                    MessageType.SETPOINT_BATCH,
                    flags,
                    1,
                    0,
                    payload,
                )
            )
        )

    first_bytes = encoded_command_size(
        9,
        int(BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.BEGIN),
    )
    second_bytes = encoded_command_size(
        7,
        int(
            BufferedSetpointFlags.CANDIDATE
            | BufferedSetpointFlags.START
            | BufferedSetpointFlags.END
        ),
    )
    ack_bytes = len(
        encode_frame(Frame(MessageType.SETPOINT_STATUS, 0, 1, 0, bytes(32)))
    )
    heartbeat_bytes = len(
        encode_frame(Frame(MessageType.HEARTBEAT, 0, 2, 0, b""))
    ) + len(
        encode_frame(Frame(MessageType.STATE_FEEDBACK, 0, 2, 0, bytes(20)))
    )
    wire_ms = (
        first_bytes + ack_bytes + heartbeat_bytes + second_bytes
    ) * 10_000.0 / 115_200.0
    anchor_lead_ms = INITIAL_FIRST_SAMPLE_LEAD_MS - SAMPLE_PERIOD_MS

    assert wire_ms == pytest.approx(87.673611, abs=0.000001)
    assert anchor_lead_ms - wire_ms >= 52.0
    assert INITIAL_FIRST_SAMPLE_LEAD_MS - (
        first_bytes * 10_000.0 / 115_200.0
    ) >= 60.0


def test_extended_motion_result_ack_maps_priming_then_running() -> None:
    _, value = scheduler()
    first = value.next_batch(current_tick_ms=1_000)
    assert first is not None
    value.acknowledge_motion_result(
        extended_result(
            status=0,
            sample_count=9,
            apply_tick_ms=first.first_apply_tick_ms,
            executor_state=BufferedExecutorState.PRIMING.value,
            terminal_reason=BufferedTerminalReason.NONE.value,
            safe_stop_required=False,
            queued=9,
            accepted=9,
            applied=0,
        )
    )
    second = value.next_batch(current_tick_ms=1_120)
    assert second is not None
    value.acknowledge_motion_result(
        extended_result(
            status=0,
            sample_count=7,
            apply_tick_ms=second.first_apply_tick_ms,
            executor_state=BufferedExecutorState.RUNNING.value,
            terminal_reason=BufferedTerminalReason.NONE.value,
            safe_stop_required=False,
            queued=16,
            accepted=16,
            applied=0,
        )
    )
    assert value.state is BufferedAdapterState.RUNNING


@pytest.mark.parametrize(
    ("change", "expected_reason"),
    [
        ({"status": 1}, "extended_ack_rejected_or_mismatched"),
        ({"sample_count": 8}, "extended_ack_rejected_or_mismatched"),
        ({"calibration_hash": 0xDEADBEEF}, "extended_ack_rejected_or_mismatched"),
        ({"executor_state": BufferedExecutorState.RUNNING.value},
         "extended_ack_rejected_or_mismatched"),
        ({"safe_stop_required": True}, "extended_ack_rejected_or_mismatched"),
    ],
)
def test_extended_ack_mismatch_aborts_without_retransmission(
    change,
    expected_reason,
) -> None:
    _, value = scheduler()
    batch = value.next_batch(current_tick_ms=1_000)
    assert batch is not None
    fields = {
        "status": 0,
        "sample_count": 9,
        "apply_tick_ms": batch.first_apply_tick_ms,
        "executor_state": BufferedExecutorState.PRIMING.value,
        "terminal_reason": BufferedTerminalReason.NONE.value,
        "safe_stop_required": False,
        "queued": 9,
        "accepted": 9,
        "applied": 0,
    }
    fields.update(change)
    with pytest.raises(BufferedActionAdapterError, match="extended buffered ACK"):
        value.acknowledge_motion_result(extended_result(**fields))
    assert value.snapshot().reason == expected_reason
    assert value.snapshot().pending_batch is False


def test_transport_timeout_discards_pending_batch_and_aborts_once() -> None:
    _, value = scheduler()
    value.next_batch(current_tick_ms=1_000)
    value.transport_failure("setpoint_status_timeout")

    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.ABORTED
    assert snapshot.pending_batch is False
    assert snapshot.reason == "setpoint_status_timeout"
    with pytest.raises(BufferedActionAdapterError, match="terminal"):
        value.next_batch(current_tick_ms=1_020)


def test_extended_success_terminal_requires_all_samples_applied() -> None:
    plan, value = scheduler(duration_ms=300)
    prime(value)
    value.observe_terminal_motion_result(
        extended_result(
            status=6,
            sample_count=0,
            apply_tick_ms=plan.samples[-1].apply_tick_ms,
            executor_state=BufferedExecutorState.SUCCEEDED.value,
            terminal_reason=BufferedTerminalReason.NONE.value,
            safe_stop_required=False,
            queued=0,
            accepted=16,
            applied=16,
            detail=MAXIMUM_APPLY_LATENESS_MS,
        )
    )
    assert value.state is BufferedAdapterState.SUCCEEDED


def test_extended_success_terminal_rejects_lateness_above_contract() -> None:
    plan, value = scheduler(duration_ms=300)
    prime(value)
    result = extended_result(
        status=6,
        sample_count=0,
        apply_tick_ms=plan.samples[-1].apply_tick_ms,
        executor_state=BufferedExecutorState.SUCCEEDED.value,
        terminal_reason=BufferedTerminalReason.NONE.value,
        safe_stop_required=False,
        queued=0,
        accepted=16,
        applied=16,
        detail=MAXIMUM_APPLY_LATENESS_MS,
    )
    result = replace(result, detail=MAXIMUM_APPLY_LATENESS_MS + 1)

    with pytest.raises(
        BufferedActionAdapterError,
        match="state and reason",
    ):
        value.observe_terminal_motion_result(result)
    assert value.state is BufferedAdapterState.ABORTED


def test_extended_underflow_terminal_clears_queue_and_requires_safe_stop() -> None:
    plan, value = scheduler()
    prime(value)
    value.record_applied(10)
    value.observe_terminal_motion_result(
        extended_result(
            status=6,
            sample_count=0,
            apply_tick_ms=plan.samples[10].apply_tick_ms,
            executor_state=BufferedExecutorState.HOLD.value,
            terminal_reason=BufferedTerminalReason.QUEUE_UNDERFLOW.value,
            safe_stop_required=True,
            queued=0,
            accepted=16,
            applied=10,
        )
    )
    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.HOLD
    assert snapshot.safe_stop_required is True
    assert snapshot.reason == "queue_underflow"


def test_terminal_state_reason_or_safe_stop_mismatch_aborts() -> None:
    plan, value = scheduler()
    prime(value)
    with pytest.raises(BufferedActionAdapterError, match="state and reason"):
        value.observe_terminal_motion_result(
            extended_result(
                status=6,
                sample_count=0,
                apply_tick_ms=plan.samples[5].apply_tick_ms,
                executor_state=BufferedExecutorState.HOLD.value,
                terminal_reason=BufferedTerminalReason.QUEUE_UNDERFLOW.value,
                safe_stop_required=False,
                queued=0,
                accepted=16,
                applied=5,
            )
        )
    assert value.state is BufferedAdapterState.ABORTED
