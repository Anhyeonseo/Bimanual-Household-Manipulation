"""Runtime execution core for continuous buffered arm trajectories."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .action_execution import ExecutionError, ExecutionOutcome, TerminalState
from .action_validation import ValidatedBufferedTrajectory
from .buffered_action_adapter import (
    MAXIMUM_APPLY_LATENESS_MS,
    UINT32_HALF_RANGE,
    UINT32_MAX,
    BufferedAdapterState,
    BufferedBatchScheduler,
    BufferedExecutionPlan,
    prepare_buffered_execution_plan,
)
from .buffered_transport_driver import BufferedExchangeResponse, BufferedTransportDriver
from .calibration import ArmCalibration
from .hardware_identity import validate_hardware_identity
from .protocol import Hello


POST_SETTLE_TOLERANCE_RAW = 30
POST_SETTLE_CONSECUTIVE_SNAPSHOTS = 2
POST_SETTLE_TIMEOUT_S = 1.5
POST_SETTLE_POLL_INTERVAL_S = 0.1


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
            try:
                heartbeat = self._transport.heartbeat()
                plan = prepare_buffered_execution_plan(
                    trajectory,
                    self._calibration,
                    preserved_gripper_rad=preserved_gripper_rad,
                    current_tick_ms=heartbeat.last_heartbeat_ms,
                )
                scheduler = BufferedBatchScheduler(plan)
                driver = BufferedTransportDriver(scheduler, self._transport)
                self._plan = plan
                self._scheduler = scheduler
                self._driver = driver
                driver.service_once(current_tick_ms=heartbeat.last_heartbeat_ms)
                return plan
            except Exception as error:
                self._clear_active()
                self._fail_closed()
                raise ExecutionError(f"buffered start failed: {error}") from error

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
                return self._abort_active(f"buffered runtime failed: {error}")

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
        self._clear_active()
        return ExecutionOutcome(
            TerminalState.SUCCEEDED,
            result.request_sequence,
            result.status_code,
            result.detail,
            "buffered trajectory completed; "
            f"maximum_apply_lateness_ms={result.detail} "
            f"post_settle_max_error_raw={settle_error}",
        )

    def _verify_post_settle(self, plan: BufferedExecutionPlan) -> int:
        final = (*plan.final_arm_positions_rad, plan.preserved_gripper_rad)
        targets = tuple(
            round(
                joint.zero_raw
                + joint.direction * position * 4096.0 / (2.0 * math.pi)
            )
            for joint, position in zip(self._calibration.joints, final, strict=True)
        )
        deadline = time.monotonic() + self._post_settle_timeout_s
        consecutive = 0
        maximum = 0
        last_error = 0
        observations = 0
        while time.monotonic() < deadline:
            state = self._transport.get_state(include_positions=True)
            if state.raw_positions is None:
                raise ExecutionError("position-only post-settle feedback is missing")
            errors = tuple(
                abs(position - target)
                for position, target in zip(
                    state.raw_positions,
                    targets,
                    strict=True,
                )
            )
            observations += 1
            last_error = max(errors)
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
                f"observations={observations} consecutive={consecutive}"
            )

        diagnostics = self._transport.get_diagnostics()
        if any(not sample.torque_enabled for sample in diagnostics.joints):
            raise ExecutionError(
                "torque disabled before final full diagnostics verification"
            )
        diagnostic_error = max(
            abs(sample.position_raw - target)
            for sample, target in zip(
                diagnostics.joints,
                targets,
                strict=True,
            )
        )
        if diagnostic_error > POST_SETTLE_TOLERANCE_RAW:
            raise ExecutionError(
                f"final full diagnostics maximum error {diagnostic_error} "
                f"exceeds {POST_SETTLE_TOLERANCE_RAW} raw"
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
        self._clear_active()
        self._fail_closed()
        return ExecutionOutcome(
            TerminalState.ABORTED,
            request_sequence,
            status_code,
            detail,
            reason,
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
