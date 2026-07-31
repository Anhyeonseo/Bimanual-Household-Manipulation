"""Transport-independent execution state for one STM32 motion goal."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .calibration import ArmCalibration
from .hardware_identity import validate_hardware_identity
from .protocol import Hello, MotionResult


# Physical Stage 7 evidence observed a transient terminal maximum of 26 raw
# while fresh feedback settled within 18 raw of the target. Keep this completion
# tolerance separate from the stricter feedback recovery envelope.
MAX_FINAL_ERROR_RAW = 30


class ExecutionError(RuntimeError):
    """Raised when a goal cannot safely enter execution."""


class TerminalState(Enum):
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class ExecutionOutcome:
    state: TerminalState
    request_sequence: int
    status_code: int | None
    final_error_raw: int | None
    reason: str


def format_execution_outcome(outcome: ExecutionOutcome) -> str:
    """Format every terminal field for persistent operator diagnostics."""

    status = "none" if outcome.status_code is None else str(outcome.status_code)
    detail = (
        "none" if outcome.final_error_raw is None else str(outcome.final_error_raw)
    )
    return (
        "ARM_EXECUTION_TERMINAL "
        f"state={outcome.state.value} sequence={outcome.request_sequence} "
        f"status={status} detail={detail} reason={outcome.reason}"
    )


class MotionExecutionCore:
    """Own one active goal and never resume it across a reconnect."""

    def __init__(
        self,
        transport: Any,
        hello: Hello,
        calibration: ArmCalibration,
        maximum_final_error_raw: int = MAX_FINAL_ERROR_RAW,
    ) -> None:
        validate_hardware_identity(hello, calibration.calibration_hash)
        if maximum_final_error_raw < 0:
            raise ValueError("maximum_final_error_raw must be non-negative")
        self._transport = transport
        self._calibration = calibration
        self._maximum_final_error_raw = maximum_final_error_raw
        self._active_sequence: int | None = None
        self._blocked = hello.stop_latched
        self._stale_result_count = 0
        self._lock = threading.RLock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active_sequence is not None

    @property
    def blocked(self) -> bool:
        with self._lock:
            return self._blocked

    @property
    def stale_result_count(self) -> int:
        with self._lock:
            return self._stale_result_count

    def start_goal(
        self,
        ordered_positions_rad: list[float] | tuple[float, ...],
        duration_ms: int,
    ) -> int:
        with self._lock:
            if self._blocked:
                raise ExecutionError("execution is blocked pending explicit recovery")
            if self._active_sequence is not None:
                raise ExecutionError("another motion goal is active")

            if not 300 <= duration_ms <= 2000:
                raise ExecutionError("duration must be within 300..2000 ms")
            positions_urad = self._calibration.radians_to_urad(
                list(ordered_positions_rad)
            )
            try:
                accepted = self._transport.send_setpoint(
                    positions_urad,
                    duration_ms,
                )
            except Exception as error:
                self._blocked = True
                self._request_safe_stop_best_effort()
                raise ExecutionError(f"setpoint transport failed: {error}") from error

            if accepted.status_code != 0:
                self._blocked = True
                self._request_safe_stop_best_effort()
                raise ExecutionError(
                    f"setpoint was not accepted: status={accepted.status_code}"
                )
            if accepted.sample_count != 1:
                self._blocked = True
                self._request_safe_stop_best_effort()
                raise ExecutionError("accepted setpoint has invalid sample count")
            if accepted.calibration_hash != self._calibration.calibration_hash:
                self._blocked = True
                self._request_safe_stop_best_effort()
                raise ExecutionError("accepted setpoint calibration hash mismatch")

            self._active_sequence = accepted.request_sequence
            return accepted.request_sequence

    def poll(self) -> ExecutionOutcome | None:
        with self._lock:
            try:
                results = self._transport.drain_motion_results()
            except Exception as error:
                return self.handle_connection_loss(str(error))
            outcome = None
            for result in results:
                if result.request_sequence != self._active_sequence:
                    self._stale_result_count += 1
                    continue
                if outcome is not None:
                    self._stale_result_count += 1
                    continue
                outcome = self._terminal_from_result(result)
            return outcome

    def _terminal_from_result(self, result: MotionResult) -> ExecutionOutcome | None:
        if result.status_code == 0:
            return None

        sequence = result.request_sequence
        self._active_sequence = None
        if result.status_code == 6:
            if result.detail <= self._maximum_final_error_raw:
                return ExecutionOutcome(
                    TerminalState.SUCCEEDED,
                    sequence,
                    result.status_code,
                    result.detail,
                    "motion completed within final error tolerance",
                )
            return ExecutionOutcome(
                TerminalState.ABORTED,
                sequence,
                result.status_code,
                result.detail,
                f"final error {result.detail} exceeds "
                f"{self._maximum_final_error_raw} raw; "
                "soft abort without safety latch",
            )

        self._blocked = True
        reason = (
            f"motion failed with status={result.status_code} "
            f"detail={result.detail}"
        )
        self._request_safe_stop_best_effort()
        return ExecutionOutcome(
            TerminalState.ABORTED,
            sequence,
            result.status_code,
            result.detail,
            reason,
        )

    def cancel_active_goal(self) -> ExecutionOutcome:
        with self._lock:
            if self._active_sequence is None:
                raise ExecutionError("there is no active goal to cancel")
            sequence = self._active_sequence
            self._active_sequence = None
            self._blocked = True
            try:
                self._transport.safe_stop()
            except Exception as error:
                return ExecutionOutcome(
                    TerminalState.ABORTED,
                    sequence,
                    None,
                    None,
                    f"SAFE_STOP acknowledgement failed: {error}",
                )
            return ExecutionOutcome(
                TerminalState.CANCELED,
                sequence,
                8,
                None,
                "goal canceled and SAFE_STOP latched",
            )

    def handle_connection_loss(self, reason: str) -> ExecutionOutcome | None:
        with self._lock:
            sequence = self._active_sequence
            self._active_sequence = None
            self._blocked = True
            if sequence is None:
                return None
            self._request_safe_stop_best_effort()
            return ExecutionOutcome(
                TerminalState.ABORTED,
                sequence,
                None,
                None,
                f"connection lost: {reason}",
            )

    def replace_transport_after_explicit_recovery(
        self,
        transport: Any,
        hello: Hello,
    ) -> None:
        with self._lock:
            if self._active_sequence is not None:
                raise ExecutionError("cannot recover while a goal is active")
            validate_hardware_identity(
                hello,
                self._calibration.calibration_hash,
            )
            if hello.stop_latched:
                self._blocked = True
                raise ExecutionError("cannot recover while stop is latched")
            self._transport = transport
            self._blocked = False

    def _request_safe_stop_best_effort(self) -> None:
        try:
            self._transport.safe_stop()
        except Exception:
            pass
