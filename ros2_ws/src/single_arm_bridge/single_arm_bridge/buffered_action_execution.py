"""Runtime execution core for continuous buffered arm trajectories."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .action_execution import ExecutionError, ExecutionOutcome, TerminalState
from .action_validation import ValidatedBufferedTrajectory
from .buffered_action_adapter import (
    INITIAL_FIRST_SAMPLE_LEAD_MS,
    MAXIMUM_APPLY_LATENESS_MS,
    STARTUP_PRIME_SAMPLES,
    UINT32_HALF_RANGE,
    UINT32_MAX,
    BufferedAdapterState,
    BufferedBatchScheduler,
    BufferedExecutionPlan,
    prepare_buffered_execution_plan,
    reanchor_buffered_execution_plan,
)
from .buffered_transport_driver import BufferedExchangeResponse, BufferedTransportDriver
from .calibration import ArmCalibration
from .hardware_identity import validate_hardware_identity
from .protocol import Hello


POST_SETTLE_TOLERANCE_RAW = 30
POST_SETTLE_CONSECUTIVE_SNAPSHOTS = 2
POST_SETTLE_TIMEOUT_S = 2.5
POST_SETTLE_POLL_INTERVAL_S = 0.1
STARTUP_FIRST_SAMPLE_LEAD_GATE_MS = 80
STARTUP_MAXIMUM_HEARTBEAT_GATES = 3


def format_apply_lateness_profile(result: Any) -> str:
    """
    Render the firmware's apply-lateness distribution for the terminal string.

    A single maximum cannot separate a rare spike from systemic drift.
    Motion-11 reached the top of its 0..5 ms allowance with only that number,
    which is why 0x00022800 reports per-bucket counts and the applied-sample
    index where the maximum was last raised. From 0x00022900 the block
    rides on terminal frames only: every status frame is transmitted by a
    blocking call on the loop that steps the executor, so carrying it on
    refill acknowledgements charged its own length to the lateness it was
    measuring.

    Firmware older than 0x00022800 omits the block, so this degrades to a
    marker rather than failing.
    """
    histogram = getattr(result, "apply_lateness_histogram", None)
    if not histogram:
        return "lateness_profile=unavailable"
    buckets = ",".join(str(int(count)) for count in histogram)
    worst = getattr(result, "maximum_apply_lateness_sample_index", None)
    worst_text = "none" if not worst else str(int(worst))
    return f"lateness_buckets={buckets} lateness_worst_sample={worst_text}"


def _tick_has_reached(current_tick_ms: int, apply_tick_ms: int, margin_ms: int) -> bool:
    elapsed = (current_tick_ms - apply_tick_ms) & UINT32_MAX
    return margin_ms <= elapsed <= UINT32_HALF_RANGE


def conservative_applied_count(
    plan: BufferedExecutionPlan,
    *,
    accepted_samples: int,
    current_tick_ms: int,
) -> int:
    """Count only accepted samples whose apply deadline is safely in the past."""

    count = 0
    for sample in plan.samples[:accepted_samples]:
        if not _tick_has_reached(
            current_tick_ms,
            sample.apply_tick_ms,
            MAXIMUM_APPLY_LATENESS_MS,
        ):
            break
        count += 1
    return count


class BufferedActionExecutionCore:
    """Own one buffered goal and keep all transport failures fail-closed."""

    def __init__(
        self,
        transport: Any,
        hello: Hello,
        calibration: ArmCalibration,
        *,
        post_settle_timeout_s: float = POST_SETTLE_TIMEOUT_S,
        post_settle_poll_interval_s: float = POST_SETTLE_POLL_INTERVAL_S,
    ) -> None:
        validate_hardware_identity(hello, calibration.calibration_hash)
        if post_settle_timeout_s <= 0.0 or post_settle_poll_interval_s < 0.0:
            raise ValueError("post-settle timing values are invalid")
        self._transport = transport
        self._calibration = calibration
        self._blocked = hello.stop_latched
        self._post_settle_timeout_s = float(post_settle_timeout_s)
        self._post_settle_poll_interval_s = float(
            post_settle_poll_interval_s
        )
        self._plan: BufferedExecutionPlan | None = None
        self._scheduler: BufferedBatchScheduler | None = None
        self._driver: BufferedTransportDriver | None = None
        self._startup_diagnostics: str | None = None
        self._lock = threading.RLock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._scheduler is not None

    @property
    def blocked(self) -> bool:
        with self._lock:
            return self._blocked

    def start_goal(
        self,
        trajectory: ValidatedBufferedTrajectory,
        *,
        preserved_gripper_rad: float,
    ) -> BufferedExecutionPlan:
        with self._lock:
            if self._blocked:
                raise ExecutionError("buffered execution is blocked pending recovery")
            if self._scheduler is not None:
                raise ExecutionError("another buffered motion goal is active")
            self._startup_diagnostics = None
            stage = "precompute"
            precompute_started = time.monotonic()
            try:
                plan = prepare_buffered_execution_plan(
                    trajectory,
                    self._calibration,
                    preserved_gripper_rad=preserved_gripper_rad,
                    # Provisional only: no physical apply tick is retained
                    # across potentially expensive position resampling.
                    current_tick_ms=0,
                )
                precompute_ms = round(
                    (time.monotonic() - precompute_started) * 1000.0,
                    3,
                )
                stage = "fresh_reanchor_heartbeat"
                heartbeat = self._transport.heartbeat()
                plan = reanchor_buffered_execution_plan(
                    plan,
                    current_tick_ms=heartbeat.last_heartbeat_ms,
                )
                scheduler = BufferedBatchScheduler(plan)
                driver = BufferedTransportDriver(scheduler, self._transport)
                self._plan = plan
                self._scheduler = scheduler
                self._driver = driver
                stage = "prime_frame_1"
                first = driver.service_once(
                    current_tick_ms=heartbeat.last_heartbeat_ms
                )
                if first is None:
                    raise ExecutionError("startup prime frame 1 was not produced")

                second = None
                prime_heartbeat = None
                first_sample_lead_ms = None
                prime_heartbeat_gates = 0
                for gate_index in range(1, STARTUP_MAXIMUM_HEARTBEAT_GATES + 1):
                    stage = f"prime_frame_2_heartbeat_{gate_index}"
                    prime_heartbeat = self._transport.heartbeat()
                    prime_heartbeat_gates += 1
                    first_sample_lead_ms = (
                        plan.samples[0].apply_tick_ms
                        - prime_heartbeat.last_heartbeat_ms
                    ) & UINT32_MAX
                    if not (
                        STARTUP_FIRST_SAMPLE_LEAD_GATE_MS
                        <= first_sample_lead_ms
                        <= INITIAL_FIRST_SAMPLE_LEAD_MS
                    ):
                        raise ExecutionError(
                            "startup first-sample lead gate failed: "
                            f"lead_ms={first_sample_lead_ms} "
                            f"required={STARTUP_FIRST_SAMPLE_LEAD_GATE_MS}.."
                            f"{INITIAL_FIRST_SAMPLE_LEAD_MS} "
                            f"heartbeat_gates={prime_heartbeat_gates}"
                        )
                    stage = f"prime_frame_2_attempt_{gate_index}"
                    second = driver.service_once(
                        current_tick_ms=prime_heartbeat.last_heartbeat_ms
                    )
                    if second is not None:
                        break
                if second is None:
                    raise ExecutionError(
                        "startup prime frame 2 did not enter the reviewed "
                        f"horizon after {prime_heartbeat_gates} heartbeat gates"
                    )
                assert prime_heartbeat is not None
                assert first_sample_lead_ms is not None
                snapshot = scheduler.snapshot()
                if (
                    snapshot.accepted_samples != STARTUP_PRIME_SAMPLES
                    or snapshot.applied_samples != 0
                    or snapshot.queued_samples != STARTUP_PRIME_SAMPLES
                    or snapshot.pending_batch
                ):
                    raise ExecutionError(
                        "startup prime accounting gate failed: "
                        f"accepted={snapshot.accepted_samples} "
                        f"applied={snapshot.applied_samples} "
                        f"queued={snapshot.queued_samples} "
                        f"pending={int(snapshot.pending_batch)}"
                    )
                self._startup_diagnostics = (
                    f"precompute_ms={precompute_ms:.3f} "
                    f"fresh_tick={heartbeat.last_heartbeat_ms} "
                    f"prime_tick={prime_heartbeat.last_heartbeat_ms} "
                    f"first_sample_lead_ms={first_sample_lead_ms} "
                    f"prime_heartbeat_gates={prime_heartbeat_gates} "
                    f"prime_frames={driver.commands_sent} "
                    f"accepted={snapshot.accepted_samples} "
                    f"applied={snapshot.applied_samples} "
                    f"queued={snapshot.queued_samples}"
                )
                return plan
            except Exception as error:
                diagnostics = self._startup_diagnostics or (
                    "precompute_ms="
                    f"{(time.monotonic() - precompute_started) * 1000.0:.3f}"
                )
                self._clear_active()
                self._fail_closed()
                raise ExecutionError(
                    "buffered start failed: "
                    f"stage={stage} {diagnostics} error={error}"
                ) from error

    def poll(self) -> ExecutionOutcome | None:
        with self._lock:
            scheduler, driver, plan = self._require_active()
            try:
                terminal = self._take_terminal(driver)
                if terminal is not None:
                    return self._finish_terminal(terminal)
                heartbeat = self._transport.heartbeat()
                snapshot = scheduler.snapshot()
                if snapshot.state in (
                    BufferedAdapterState.RUNNING,
                    BufferedAdapterState.INPUT_COMPLETE,
                ):
                    applied = conservative_applied_count(
                        plan,
                        accepted_samples=snapshot.accepted_samples,
                        current_tick_ms=heartbeat.last_heartbeat_ms,
                    )
                    if applied > snapshot.applied_samples:
                        scheduler.record_clock_progress(applied)
                driver.service_once(current_tick_ms=heartbeat.last_heartbeat_ms)
                terminal = self._take_terminal(driver)
                if terminal is not None:
                    return self._finish_terminal(terminal)
                return None
            except Exception as error:
                return self._abort_active(
                    f"buffered runtime failed: {error}"
                )

    def cancel_active_goal(self) -> ExecutionOutcome:
        with self._lock:
            scheduler, _, _ = self._require_active()
            scheduler.cancel()
            self._clear_active()
            self._blocked = True
            try:
                self._transport.safe_stop()
                return ExecutionOutcome(
                    TerminalState.CANCELED,
                    0,
                    8,
                    None,
                    "buffered goal canceled and SAFE_STOP latched",
                )
            except Exception as error:
                return ExecutionOutcome(
                    TerminalState.ABORTED,
                    0,
                    None,
                    None,
                    f"buffered cancel SAFE_STOP failed: {error}",
                )

    def handle_connection_loss(self, reason: str) -> ExecutionOutcome | None:
        with self._lock:
            if self._scheduler is None:
                self._blocked = True
                return None
            return self._abort_active(f"connection lost: {reason}")

    def replace_transport_after_explicit_recovery(
        self,
        transport: Any,
        hello: Hello,
    ) -> None:
        with self._lock:
            if self._scheduler is not None:
                raise ExecutionError("cannot recover while a buffered goal is active")
            validate_hardware_identity(hello, self._calibration.calibration_hash)
            if hello.stop_latched:
                self._blocked = True
                raise ExecutionError("cannot recover while stop is latched")
            self._transport = transport
            self._blocked = False

    def _take_terminal(
        self,
        driver: BufferedTransportDriver,
    ) -> BufferedExchangeResponse | None:
        responses = self._transport.drain_buffered_motion_results()
        if not responses:
            return None
        if len(responses) > 1:
            raise ExecutionError("multiple buffered terminals received")
        driver.observe_terminal(responses[0])
        return responses[0]

    def _finish_terminal(
        self,
        response: BufferedExchangeResponse,
    ) -> ExecutionOutcome:
        scheduler, _, plan = self._require_active()
        snapshot = scheduler.snapshot()
        result = response.result
        if snapshot.state is not BufferedAdapterState.SUCCEEDED:
            reason = snapshot.reason or snapshot.state.value
            return self._abort_active(
                "buffered terminal did not succeed: "
                f"state={snapshot.state.value} reason={reason}",
                request_sequence=result.request_sequence,
                status_code=result.status_code,
                detail=result.detail,
            )
        try:
            settle_error = self._verify_post_settle(plan)
        except Exception as error:
            return self._abort_active(
                f"buffered post-settle diagnostics failed: {error}",
                request_sequence=result.request_sequence,
                status_code=result.status_code,
                detail=result.detail,
            )
        startup = self._startup_diagnostics or "startup=unavailable"
        lateness = format_apply_lateness_profile(result)
        self._clear_active()
        return ExecutionOutcome(
            TerminalState.SUCCEEDED,
            result.request_sequence,
            result.status_code,
            result.detail,
            "buffered trajectory completed; "
            f"maximum_apply_lateness_ms={result.detail} "
            f"post_settle_max_error_raw={settle_error}; {startup}; {lateness}",
        )

    def _verify_post_settle(self, plan: BufferedExecutionPlan) -> int:
        final = (*plan.final_arm_positions_rad, plan.preserved_gripper_rad)
        targets = tuple(
            round(
                joint.zero_raw
                + joint.direction * position * 4096.0 / (2.0 * math.pi)
            )
            for joint, position in zip(
                self._calibration.joints,
                final,
                strict=True,
            )
        )
        started_at = time.monotonic()
        deadline = time.monotonic() + self._post_settle_timeout_s
        consecutive = 0
        maximum = 0
        last_error = 0
        observations = 0
        heartbeat_gates = 0
        error_trace: list[tuple[int, ...]] = []
        per_axis_minimum: tuple[int, ...] | None = None
        final_errors: tuple[int, ...] | None = None
        best_maximum_error: int | None = None

        def elapsed_ms() -> int:
            return max(0, round((time.monotonic() - started_at) * 1000.0))

        def format_errors(values: tuple[int, ...] | None) -> str:
            if values is None:
                return "none"
            return "[" + ",".join(str(value) for value in values) + "]"

        def diagnostic_context() -> str:
            trace = "|".join(
                f"{index}:{format_errors(values)}"
                for index, values in enumerate(error_trace, start=1)
            )
            return (
                f"observations={observations} consecutive={consecutive} "
                f"heartbeat_gates={heartbeat_gates} "
                f"elapsed_ms={elapsed_ms()} "
                f"error_trace_raw={trace or 'none'} "
                "per_axis_minimum_error_raw="
                f"{format_errors(per_axis_minimum)} "
                f"final_errors_raw={format_errors(final_errors)} "
                "best_maximum_error_raw="
                f"{best_maximum_error if best_maximum_error is not None else 'none'}"
            )

        def heartbeat_gate(stage: str) -> None:
            nonlocal heartbeat_gates
            try:
                self._transport.heartbeat()
            except Exception as error:
                raise ExecutionError(
                    "post-settle heartbeat gate failed "
                    f"stage={stage} {diagnostic_context()}: {error}"
                ) from error
            heartbeat_gates += 1

        # A successful firmware terminal ends setpoint application, but does
        # not relax the 500 ms MCU heartbeat watchdog.  Re-establish the host
        # lease before any potentially long servo-bus position sweep.
        heartbeat_gate("terminal_received")
        while time.monotonic() < deadline:
            stage = f"position_snapshot_{observations + 1}"
            heartbeat_gate(stage)
            try:
                state = self._transport.get_state(include_positions=True)
            except Exception as error:
                raise ExecutionError(
                    "post-settle position feedback failed "
                    f"stage={stage} {diagnostic_context()}: {error}"
                ) from error
            if state.raw_positions is None:
                raise ExecutionError(
                    "position-only post-settle feedback is missing "
                    f"stage={stage} {diagnostic_context()}"
                )
            errors = tuple(
                abs(position - target)
                for position, target in zip(
                    state.raw_positions,
                    targets,
                    strict=True,
                )
            )
            observations += 1
            error_trace.append(errors)
            final_errors = errors
            if per_axis_minimum is None:
                per_axis_minimum = errors
            else:
                per_axis_minimum = tuple(
                    min(previous, current)
                    for previous, current in zip(
                        per_axis_minimum,
                        errors,
                        strict=True,
                    )
                )
            last_error = max(errors)
            best_maximum_error = (
                last_error
                if best_maximum_error is None
                else min(best_maximum_error, last_error)
            )
            if last_error <= POST_SETTLE_TOLERANCE_RAW:
                consecutive += 1
                maximum = max(maximum, last_error)
                if consecutive >= POST_SETTLE_CONSECUTIVE_SNAPSHOTS:
                    break
            else:
                consecutive = 0
                maximum = 0
            time.sleep(self._post_settle_poll_interval_s)
        else:
            raise ExecutionError(
                f"last maximum error {last_error} did not provide "
                f"{POST_SETTLE_CONSECUTIVE_SNAPSHOTS} consecutive "
                f"position-only snapshots within "
                f"{POST_SETTLE_TOLERANCE_RAW} raw; "
                f"{diagnostic_context()}"
            )

        heartbeat_gate("full_diagnostics")
        try:
            diagnostics = self._transport.get_diagnostics()
        except Exception as error:
            raise ExecutionError(
                "post-settle full diagnostics failed "
                f"stage=full_diagnostics {diagnostic_context()}: {error}"
            ) from error
        if any(not sample.torque_enabled for sample in diagnostics.joints):
            raise ExecutionError(
                "torque disabled before final full diagnostics verification; "
                f"{diagnostic_context()}"
            )
        diagnostic_errors = tuple(
            abs(sample.position_raw - target)
            for sample, target in zip(
                diagnostics.joints,
                targets,
                strict=True,
            )
        )
        diagnostic_error = max(diagnostic_errors)
        if diagnostic_error > POST_SETTLE_TOLERANCE_RAW:
            raise ExecutionError(
                f"final full diagnostics maximum error {diagnostic_error} "
                f"exceeds {POST_SETTLE_TOLERANCE_RAW} raw; "
                "diagnostic_errors_raw="
                f"{format_errors(diagnostic_errors)} "
                f"{diagnostic_context()}"
            )
        return max(maximum, diagnostic_error)

    def _abort_active(
        self,
        reason: str,
        *,
        request_sequence: int = 0,
        status_code: int | None = None,
        detail: int | None = None,
    ) -> ExecutionOutcome:
        startup = self._startup_diagnostics or "startup=unavailable"
        self._clear_active()
        self._fail_closed()
        return ExecutionOutcome(
            TerminalState.ABORTED,
            request_sequence,
            status_code,
            detail,
            f"{reason}; {startup}",
        )

    def _fail_closed(self) -> None:
        self._blocked = True
        try:
            self._transport.safe_stop()
        except Exception:
            pass

    def _require_active(self):
        if self._scheduler is None or self._driver is None or self._plan is None:
            raise ExecutionError("there is no active buffered goal")
        return self._scheduler, self._driver, self._plan

    def _clear_active(self) -> None:
        self._scheduler = None
        self._driver = None
        self._plan = None
        self._startup_diagnostics = None
