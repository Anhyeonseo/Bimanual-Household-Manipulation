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
    BufferedActionAdapterError,
    BufferedAdapterState,
    BufferedBatchScheduler,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.protocol import BufferedSetpointFlags  # noqa: E402


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
    second = value.next_batch(current_tick_ms=current_tick_ms + 18)
    assert second is not None
    ack_pending(value, second, applied=0)
    return first, second


def test_resamples_at_20ms_with_100ms_initial_lead_and_preserves_gripper() -> None:
    plan, _ = scheduler()

    assert plan.anchor_tick_ms == 1_080
    assert len(plan.samples) == 40
    assert plan.samples[0].trajectory_elapsed_ms == 20
    assert plan.samples[0].apply_tick_ms == 1_100
    assert plan.samples[0].positions_urad[:5] == (2_000,) * 5
    assert plan.samples[0].positions_urad[5] == 60_000
    assert plan.samples[-1].trajectory_elapsed_ms == 800
    assert plan.samples[-1].positions_urad[:5] == (80_000,) * 5


@pytest.mark.parametrize("duration_ms", [319, 330])
def test_plan_rejects_short_or_non_20ms_duration(duration_ms: int) -> None:
    calibration, trajectory = validated_path(duration_ms, target=0.01)
    with pytest.raises(GoalValidationError):
        prepare_buffered_execution_plan(
            trajectory,
            calibration,
            preserved_gripper_rad=0.06,
            current_tick_ms=1_000,
        )


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

    refill = value.next_batch(current_tick_ms=1_201)
    assert refill is not None
    assert refill.sample_count == 6
    assert refill.first_sample_index == 17
    ack_pending(value, refill, applied=6)
    assert value.snapshot().queued_samples == 16
    assert value.next_batch(current_tick_ms=1_220) is None


def test_80ms_outage_refill_uses_9_plus_2_without_gap() -> None:
    _, value = scheduler()
    prime(value)
    value.record_applied(11)

    first = value.next_batch(current_tick_ms=1_318)
    assert first is not None and first.sample_count == 9
    ack_pending(value, first, applied=11)
    assert value.snapshot().queued_samples == 14

    second = value.next_batch(current_tick_ms=1_336)
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
        value.next_batch(current_tick_ms=1_370)
    snapshot = value.snapshot()
    assert snapshot.state is BufferedAdapterState.ABORTED
    assert snapshot.reason == "batch_lead_outside_reviewed_window"
    assert snapshot.pending_batch is False


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
    _, value = scheduler(duration_ms=320)
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

    assert plan.anchor_tick_ms == 0
    assert plan.samples[0].apply_tick_ms == 20
    first = value.next_batch(current_tick_ms=current)
    assert first is not None
    assert first.first_apply_tick_ms == 20
    assert first.samples[-1].tick_offset_ms == 160
