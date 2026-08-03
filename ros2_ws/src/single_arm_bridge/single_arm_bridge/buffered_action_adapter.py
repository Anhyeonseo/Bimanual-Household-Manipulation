"""Pure host scheduler for the buffered FollowJointTrajectory candidate.

This module has no ROS or serial side effects.  It produces deterministic
batches and accounts for admission/terminal acknowledgements; ROS Action
ownership is connected through a separately testable runtime execution core.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .action_validation import (
    GoalValidationError,
    ValidatedBufferedTrajectory,
    interpolate_buffered_trajectory,
)
from .calibration import ArmCalibration
from .protocol import BufferedSetpointFlags, BufferedSetpointSample, MotionResult


UINT32_MAX = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x7FFFFFFF
SAMPLE_PERIOD_MS = 20
INITIAL_FIRST_SAMPLE_LEAD_MS = 140
MINIMUM_LEAD_MS = 60
MAXIMUM_LEAD_MS = 400
STARTUP_PRIME_SAMPLES = 16
LOW_WATERMARK_SAMPLES = 10
REFILL_TARGET_SAMPLES = 16
MAXIMUM_BATCH_SAMPLES = 9
MAXIMUM_APPLY_LATENESS_MS = 5


class BufferedActionAdapterError(RuntimeError):
    """Raised when the host scheduler must stop fail-closed."""


class BufferedAdapterState(Enum):
    PRIMING = "priming"
    RUNNING = "running"
    INPUT_COMPLETE = "input_complete"
    SUCCEEDED = "succeeded"
    HOLD = "hold"
    CANCELED = "canceled"
    ABORTED = "aborted"


class BufferedExecutorState(Enum):
    PRIMING = 0
    RUNNING = 1
    HOLD = 2
    SUCCEEDED = 3
    CANCELED = 4
    ABORTED = 5


class BufferedTerminalReason(Enum):
    NONE = 0
    PLANNED_HOLD = 1
    OPERATOR_CANCEL = 2
    QUEUE_UNDERFLOW = 3
    MISSED_APPLY_TICK = 4
    CONNECTION_LOSS = 5
    TRACKING_ERROR = 6


@dataclass(frozen=True, slots=True)
class ScheduledBufferedSample:
    sample_index: int
    trajectory_elapsed_ms: int
    apply_tick_ms: int
    positions_urad: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class BufferedExecutionPlan:
    anchor_tick_ms: int
    sample_period_ms: int
    samples: tuple[ScheduledBufferedSample, ...]
    final_arm_positions_rad: tuple[float, ...]
    preserved_gripper_rad: float
    calibration_hash: int


@dataclass(frozen=True, slots=True)
class BufferedCommandBatch:
    first_apply_tick_ms: int
    samples: tuple[BufferedSetpointSample, ...]
    flags: BufferedSetpointFlags
    first_sample_index: int
    accepted_samples_after_ack: int

    @property
    def sample_count(self) -> int:
        return len(self.samples)


@dataclass(frozen=True, slots=True)
class BufferedAdapterSnapshot:
    state: BufferedAdapterState
    total_samples: int
    accepted_samples: int
    applied_samples: int
    queued_samples: int
    pending_batch: bool
    input_complete: bool
    safe_stop_required: bool
    reason: str | None


def _require_uint32(value: int, field: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= UINT32_MAX
    ):
        raise BufferedActionAdapterError(f"{field} must be uint32 milliseconds")
    return value


def _tick_delta(candidate: int, reference: int) -> int:
    return (candidate - reference) & UINT32_MAX


def _checked_lead(candidate: int, reference: int) -> int:
    lead = _tick_delta(candidate, reference)
    if lead == 0 or lead > UINT32_HALF_RANGE:
        raise BufferedActionAdapterError("buffered sample tick is stale")
    if lead < MINIMUM_LEAD_MS:
        raise BufferedActionAdapterError("buffered sample lead is below 60 ms")
    if lead > MAXIMUM_LEAD_MS:
        raise BufferedActionAdapterError("buffered sample lead exceeds 400 ms")
    return lead


def prepare_buffered_execution_plan(
    trajectory: ValidatedBufferedTrajectory,
    calibration: ArmCalibration,
    *,
    preserved_gripper_rad: float,
    current_tick_ms: int,
) -> BufferedExecutionPlan:
    """Resample a validated five-axis path at 20 ms without hardware access."""

    _require_uint32(current_tick_ms, "current tick")
    if len(trajectory.start_positions) != 5:
        raise GoalValidationError("buffered arm trajectory must contain five joints")
    if trajectory.duration_ms % SAMPLE_PERIOD_MS:
        raise GoalValidationError("buffered duration must align to 20 ms")
    # Include the already-validated fresh start as the first wire sample.  The
    # firmware can use this t=0 position as its interpolation anchor without a
    # blocking six-servo read sweep immediately before execution.
    sample_count = trajectory.duration_ms // SAMPLE_PERIOD_MS + 1
    if sample_count < STARTUP_PRIME_SAMPLES:
        raise GoalValidationError(
            "buffered duration must provide at least 16 samples (300 ms)"
        )

    anchor_tick = (
        current_tick_ms + INITIAL_FIRST_SAMPLE_LEAD_MS - SAMPLE_PERIOD_MS
    ) & UINT32_MAX
    samples: list[ScheduledBufferedSample] = []
    for sample_index in range(1, sample_count + 1):
        elapsed_ms = (sample_index - 1) * SAMPLE_PERIOD_MS
        arm_positions = interpolate_buffered_trajectory(
            trajectory,
            elapsed_ms * 1_000_000,
        )
        positions_urad = tuple(
            calibration.radians_to_urad(
                [*arm_positions, preserved_gripper_rad]
            )
        )
        samples.append(
            ScheduledBufferedSample(
                sample_index=sample_index,
                trajectory_elapsed_ms=elapsed_ms,
                apply_tick_ms=(
                    anchor_tick + sample_index * SAMPLE_PERIOD_MS
                ) & UINT32_MAX,
                positions_urad=positions_urad,
            )
        )

    return BufferedExecutionPlan(
        anchor_tick_ms=anchor_tick,
        sample_period_ms=SAMPLE_PERIOD_MS,
        samples=tuple(samples),
        final_arm_positions_rad=trajectory.ordered_points[-1],
        preserved_gripper_rad=float(preserved_gripper_rad),
        calibration_hash=calibration.calibration_hash,
    )


class BufferedBatchScheduler:
    """Prime/refill state machine with no retransmission or hardware access."""

    def __init__(self, plan: BufferedExecutionPlan) -> None:
        if plan.sample_period_ms != SAMPLE_PERIOD_MS:
            raise ValueError("execution plan sample period must be 20 ms")
        if len(plan.samples) < STARTUP_PRIME_SAMPLES:
            raise ValueError("execution plan cannot satisfy startup prime depth")
        self._plan = plan
        self._state = BufferedAdapterState.PRIMING
        self._accepted = 0
        self._applied = 0
        self._pending: BufferedCommandBatch | None = None
        self._refill_active = False
        self._safe_stop_required = False
        self._reason: str | None = None

    @property
    def state(self) -> BufferedAdapterState:
        return self._state

    def snapshot(self) -> BufferedAdapterSnapshot:
        return BufferedAdapterSnapshot(
            state=self._state,
            total_samples=len(self._plan.samples),
            accepted_samples=self._accepted,
            applied_samples=self._applied,
            queued_samples=self._accepted - self._applied,
            pending_batch=self._pending is not None,
            input_complete=self._accepted == len(self._plan.samples),
            safe_stop_required=self._safe_stop_required,
            reason=self._reason,
        )

    def next_batch(self, *, current_tick_ms: int) -> BufferedCommandBatch | None:
        """Return the next frame once; a pending frame is never returned again."""

        _require_uint32(current_tick_ms, "current tick")
        self._require_schedulable(allow_input_complete=True)
        if self._state is BufferedAdapterState.INPUT_COMPLETE:
            return None
        if self._pending is not None:
            raise BufferedActionAdapterError(
                "pending buffered batch cannot be retransmitted"
            )

        total = len(self._plan.samples)
        queued = self._accepted - self._applied
        if not 0 <= queued <= REFILL_TARGET_SAMPLES:
            self._abort("queue_accounting_invalid")
            raise BufferedActionAdapterError("buffered queue accounting is invalid")
        if self._accepted == total:
            self._state = BufferedAdapterState.INPUT_COMPLETE
            return None

        if self._state is BufferedAdapterState.PRIMING:
            desired = STARTUP_PRIME_SAMPLES
        else:
            if not self._refill_active and queued > LOW_WATERMARK_SAMPLES:
                return None
            self._refill_active = True
            desired = REFILL_TARGET_SAMPLES

        needed = desired - queued
        if needed <= 0:
            self._refill_active = False
            return None
        count = min(MAXIMUM_BATCH_SAMPLES, needed, total - self._accepted)
        selected = self._plan.samples[self._accepted : self._accepted + count]
        try:
            leads = tuple(
                _tick_delta(sample.apply_tick_ms, current_tick_ms)
                for sample in selected
            )
            if any(lead == 0 or lead > UINT32_HALF_RANGE for lead in leads):
                raise BufferedActionAdapterError("buffered sample tick is stale")
            if any(lead < MINIMUM_LEAD_MS for lead in leads):
                raise BufferedActionAdapterError(
                    "buffered sample lead is below 60 ms"
                )
            # A valid future batch may be outside the maximum horizon only
            # because the host is polling early.  Wait without creating a
            # pending frame; stale/late batches above remain fail-closed.
            if any(lead > MAXIMUM_LEAD_MS for lead in leads):
                return None
        except BufferedActionAdapterError:
            self._abort("batch_lead_outside_reviewed_window")
            raise

        first_tick = selected[0].apply_tick_ms
        wire_samples = tuple(
            BufferedSetpointSample(
                tick_offset_ms=_tick_delta(sample.apply_tick_ms, first_tick),
                positions_urad=sample.positions_urad,
            )
            for sample in selected
        )
        accepted_after = self._accepted + count
        flags = BufferedSetpointFlags.CANDIDATE
        if self._accepted == 0:
            flags |= BufferedSetpointFlags.BEGIN
        if (
            self._state is BufferedAdapterState.PRIMING
            and accepted_after >= STARTUP_PRIME_SAMPLES
        ):
            flags |= BufferedSetpointFlags.START
        if accepted_after == total:
            flags |= BufferedSetpointFlags.END

        self._pending = BufferedCommandBatch(
            first_apply_tick_ms=first_tick,
            samples=wire_samples,
            flags=flags,
            first_sample_index=selected[0].sample_index,
            accepted_samples_after_ack=accepted_after,
        )
        return self._pending

    def acknowledge_batch(
        self,
        *,
        status_code: int,
        accepted_samples: int,
        applied_samples: int,
        queued_samples: int,
    ) -> None:
        """Accept one matching ACK or enter terminal fail-closed state."""

        self._require_schedulable()
        pending = self._pending
        if pending is None:
            self._abort("unexpected_ack")
            raise BufferedActionAdapterError("buffered ACK has no pending batch")
        expected_accepted = pending.accepted_samples_after_ack
        valid = (
            status_code == 0
            and accepted_samples == expected_accepted
            and self._applied <= applied_samples <= accepted_samples
            and queued_samples == accepted_samples - applied_samples
            and 0 <= queued_samples <= REFILL_TARGET_SAMPLES
        )
        if not valid:
            self._pending = None
            self._abort("batch_ack_rejected_or_mismatched")
            raise BufferedActionAdapterError("buffered ACK rejected or mismatched")

        self._accepted = accepted_samples
        self._applied = applied_samples
        self._pending = None
        if self._state is BufferedAdapterState.PRIMING:
            if self._accepted >= STARTUP_PRIME_SAMPLES:
                self._state = BufferedAdapterState.RUNNING
        elif self._refill_active and queued_samples >= REFILL_TARGET_SAMPLES:
            self._refill_active = False

        if self._accepted == len(self._plan.samples):
            # Admission/progress accounting is not a terminal.  Success is
            # entered only by observe_terminal_motion_result().
            self._state = BufferedAdapterState.INPUT_COMPLETE

    def acknowledge_motion_result(self, result: MotionResult) -> None:
        """Map one extended admission response without accepting legacy status."""

        self._require_schedulable()
        pending = self._pending
        if pending is None:
            self._abort("unexpected_ack")
            raise BufferedActionAdapterError("buffered ACK has no pending batch")
        expected_state = (
            BufferedExecutorState.RUNNING.value
            if (
                BufferedSetpointFlags.START in pending.flags
                or self._state is BufferedAdapterState.RUNNING
            )
            else BufferedExecutorState.PRIMING.value
        )
        extended = (
            result.executor_state is not None
            and result.terminal_reason is not None
            and result.safe_stop_required is not None
            and result.queue_result is not None
            and result.queued_samples is not None
            and result.accepted_samples is not None
            and result.applied_samples is not None
        )
        valid_envelope = (
            extended
            and result.status_code == 0
            and result.sample_count == pending.sample_count
            and result.apply_tick_ms == pending.first_apply_tick_ms
            and result.calibration_hash == self._plan.calibration_hash
            and result.executor_state == expected_state
            and result.terminal_reason == BufferedTerminalReason.NONE.value
            and result.safe_stop_required is False
            and result.queue_result == 0
        )
        if not valid_envelope:
            actual = {
                "status_code": result.status_code,
                "sample_count": result.sample_count,
                "apply_tick_ms": result.apply_tick_ms,
                "calibration_hash": f"0x{result.calibration_hash:08X}",
                "executor_state": result.executor_state,
                "terminal_reason": result.terminal_reason,
                "safe_stop_required": result.safe_stop_required,
                "queue_result": result.queue_result,
                "queued_samples": result.queued_samples,
                "accepted_samples": result.accepted_samples,
                "applied_samples": result.applied_samples,
            }
            self._pending = None
            self._abort("extended_ack_rejected_or_mismatched")
            raise BufferedActionAdapterError(
                "extended buffered ACK rejected or mismatched: "
                f"expected_state={expected_state} "
                f"expected_sample_count={pending.sample_count} "
                f"expected_apply_tick={pending.first_apply_tick_ms} "
                f"expected_calibration=0x{self._plan.calibration_hash:08X} "
                f"actual={actual}"
            )
        self.acknowledge_batch(
            status_code=result.status_code,
            accepted_samples=result.accepted_samples,
            applied_samples=result.applied_samples,
            queued_samples=result.queued_samples,
        )

    def observe_terminal_motion_result(self, result: MotionResult) -> None:
        """Map a future asynchronous terminal result into one host terminal."""

        self._require_schedulable(allow_input_complete=True)
        if self._pending is not None:
            self._pending = None
            self._abort("terminal_while_batch_pending")
            raise BufferedActionAdapterError(
                "terminal result arrived while a batch ACK was pending"
            )
        extended = all(
            value is not None
            for value in (
                result.executor_state,
                result.terminal_reason,
                result.safe_stop_required,
                result.queue_result,
                result.queued_samples,
                result.accepted_samples,
                result.applied_samples,
            )
        )
        accounting_valid = (
            extended
            and result.status_code == 6
            and result.calibration_hash == self._plan.calibration_hash
            and result.accepted_samples == self._accepted
            and result.applied_samples is not None
            and self._applied <= result.applied_samples <= self._accepted
            and result.queued_samples == 0
        )
        if not accounting_valid:
            self._abort("terminal_status_rejected_or_mismatched")
            raise BufferedActionAdapterError(
                "extended buffered terminal status is invalid"
            )

        self._applied = result.applied_samples
        state = result.executor_state
        reason = result.terminal_reason
        if state == BufferedExecutorState.SUCCEEDED.value:
            valid = (
                reason == BufferedTerminalReason.NONE.value
                and result.safe_stop_required is False
                and 0 <= result.detail <= MAXIMUM_APPLY_LATENESS_MS
                and self._accepted == len(self._plan.samples)
                and self._applied == self._accepted
                and result.queued_samples == 0
            )
            target = BufferedAdapterState.SUCCEEDED
        elif state == BufferedExecutorState.HOLD.value:
            valid = reason in (
                BufferedTerminalReason.PLANNED_HOLD.value,
                BufferedTerminalReason.QUEUE_UNDERFLOW.value,
                BufferedTerminalReason.MISSED_APPLY_TICK.value,
            )
            valid = valid and result.safe_stop_required is (
                reason != BufferedTerminalReason.PLANNED_HOLD.value
            )
            target = BufferedAdapterState.HOLD
        elif state == BufferedExecutorState.CANCELED.value:
            valid = (
                reason == BufferedTerminalReason.OPERATOR_CANCEL.value
                and result.safe_stop_required is True
            )
            target = BufferedAdapterState.CANCELED
        elif state == BufferedExecutorState.ABORTED.value:
            valid = reason in (
                BufferedTerminalReason.CONNECTION_LOSS.value,
                BufferedTerminalReason.TRACKING_ERROR.value,
            ) and result.safe_stop_required is True
            target = BufferedAdapterState.ABORTED
        else:
            valid = False
            target = BufferedAdapterState.ABORTED
        if not valid:
            self._abort("terminal_state_reason_mismatch")
            raise BufferedActionAdapterError(
                "buffered terminal state and reason do not match"
            )
        self._state = target
        self._safe_stop_required = bool(result.safe_stop_required)
        self._reason = BufferedTerminalReason(reason).name.lower()

    def transport_failure(self, reason: str = "transport_failure") -> None:
        """Abort a timeout/disconnect once; no pending frame is retained."""

        self._require_schedulable(allow_input_complete=True)
        if not isinstance(reason, str) or not reason:
            raise ValueError("transport failure reason must be non-empty")
        self._pending = None
        self._abort(reason)

    def record_applied(self, applied_samples: int) -> None:
        """Record monotonic firmware progress and detect an actual underflow."""

        self._require_schedulable(allow_input_complete=True)
        if (
            isinstance(applied_samples, bool)
            or not isinstance(applied_samples, int)
            or not self._applied <= applied_samples <= self._accepted
        ):
            self._abort("applied_count_invalid")
            raise BufferedActionAdapterError("applied sample count is invalid")
        self._applied = applied_samples
        if self._applied == len(self._plan.samples):
            self._state = BufferedAdapterState.SUCCEEDED
        elif self._applied == self._accepted:
            self._abort("queue_underflow")
            raise BufferedActionAdapterError("buffered queue underflow")

    def record_clock_progress(self, applied_samples: int) -> None:
        """Record conservative clock progress while still requiring terminal ACK."""

        self._require_schedulable(allow_input_complete=True)
        if (
            isinstance(applied_samples, bool)
            or not isinstance(applied_samples, int)
            or not self._applied <= applied_samples <= self._accepted
        ):
            self._abort("clock_progress_invalid")
            raise BufferedActionAdapterError("clock progress count is invalid")
        self._applied = applied_samples
        if self._applied == self._accepted < len(self._plan.samples):
            self._abort("queue_underflow")
            raise BufferedActionAdapterError("buffered queue underflow")
        if self._accepted == len(self._plan.samples):
            self._state = BufferedAdapterState.INPUT_COMPLETE

    def cancel(self) -> None:
        self._require_schedulable(allow_input_complete=True)
        self._pending = None
        self._state = BufferedAdapterState.CANCELED
        self._safe_stop_required = True
        self._reason = "operator_cancel"

    def _require_schedulable(self, *, allow_input_complete: bool = False) -> None:
        allowed = {BufferedAdapterState.PRIMING, BufferedAdapterState.RUNNING}
        if allow_input_complete:
            allowed.add(BufferedAdapterState.INPUT_COMPLETE)
        if self._state not in allowed:
            raise BufferedActionAdapterError("buffered scheduler is terminal")

    def _abort(self, reason: str) -> None:
        self._state = BufferedAdapterState.ABORTED
        self._safe_stop_required = True
        self._reason = reason
