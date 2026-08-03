"""Mockable exchange driver for the buffered host scheduler.

The driver never opens serial and is not connected to the ROS Action server.
It defines the one-shot frame/response ordering used by the physical transport
method while keeping runtime ownership and motion authorization separate.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .buffered_action_adapter import (
    BufferedActionAdapterError,
    BufferedBatchScheduler,
    BufferedCommandBatch,
)
from .protocol import MotionResult, encode_buffered_setpoint_payload


DEFAULT_EXCHANGE_TIMEOUT_S = 0.1


class BufferedTransportDriverError(RuntimeError):
    """Raised after the driver has already transitioned fail-closed."""


@dataclass(frozen=True, slots=True)
class BufferedOutboundCommand:
    flags: int
    payload: bytes
    first_apply_tick_ms: int
    sample_count: int
    first_sample_index: int
    accepted_samples_after_ack: int


@dataclass(frozen=True, slots=True)
class BufferedExchangeResponse:
    frame_sequence: int
    result: MotionResult


class BufferedExchangePort(Protocol):
    def exchange_buffered_command(
        self,
        command: BufferedOutboundCommand,
        *,
        timeout_s: float,
    ) -> BufferedExchangeResponse:
        """Exchange exactly one command for exactly one matching response."""


def encode_buffered_command_batch(
    batch: BufferedCommandBatch,
) -> BufferedOutboundCommand:
    return BufferedOutboundCommand(
        flags=int(batch.flags),
        payload=encode_buffered_setpoint_payload(
            batch.first_apply_tick_ms,
            batch.samples,
        ),
        first_apply_tick_ms=batch.first_apply_tick_ms,
        sample_count=batch.sample_count,
        first_sample_index=batch.first_sample_index,
        accepted_samples_after_ack=batch.accepted_samples_after_ack,
    )


class BufferedTransportDriver:
    """One-shot scheduler-to-port adapter with no automatic retransmission."""

    def __init__(
        self,
        scheduler: BufferedBatchScheduler,
        port: BufferedExchangePort,
        *,
        exchange_timeout_s: float = DEFAULT_EXCHANGE_TIMEOUT_S,
    ) -> None:
        if (
            isinstance(exchange_timeout_s, bool)
            or not isinstance(exchange_timeout_s, (int, float))
            or not 0.0 < float(exchange_timeout_s) <= 1.0
        ):
            raise ValueError("exchange timeout must be within 0..1 seconds")
        self._scheduler = scheduler
        self._port = port
        self._timeout_s = float(exchange_timeout_s)
        self._sent_keys: set[tuple[int, int]] = set()
        self._commands_sent = 0

    @property
    def commands_sent(self) -> int:
        return self._commands_sent

    def service_once(
        self,
        *,
        current_tick_ms: int,
    ) -> BufferedOutboundCommand | None:
        try:
            batch = self._scheduler.next_batch(current_tick_ms=current_tick_ms)
        except BufferedActionAdapterError as exc:
            raise BufferedTransportDriverError(
                "buffered scheduler refused transport service"
            ) from exc
        if batch is None:
            return None
        key = (batch.first_sample_index, batch.accepted_samples_after_ack)
        if key in self._sent_keys:
            self._scheduler.transport_failure("duplicate_batch_identity")
            raise BufferedTransportDriverError(
                "buffered batch identity was already transmitted"
            )
        command = encode_buffered_command_batch(batch)
        self._sent_keys.add(key)
        self._commands_sent += 1
        try:
            response = self._port.exchange_buffered_command(
                command,
                timeout_s=self._timeout_s,
            )
        except Exception as exc:
            reason = (
                "setpoint_status_timeout"
                if isinstance(exc, TimeoutError)
                else "buffered_transport_exchange_error"
            )
            self._scheduler.transport_failure(reason)
            raise BufferedTransportDriverError(reason) from exc

        if (
            isinstance(response.frame_sequence, bool)
            or not isinstance(response.frame_sequence, int)
            or not 0 <= response.frame_sequence <= 0xFFFFFFFF
            or response.result.request_sequence != response.frame_sequence
        ):
            self._scheduler.transport_failure("response_sequence_mismatch")
            raise BufferedTransportDriverError("response sequence mismatch")
        try:
            self._scheduler.acknowledge_motion_result(response.result)
        except BufferedActionAdapterError as exc:
            raise BufferedTransportDriverError(
                "buffered response failed host admission"
            ) from exc
        return command

    def observe_terminal(self, response: BufferedExchangeResponse) -> None:
        if response.result.request_sequence != response.frame_sequence:
            self._scheduler.transport_failure("terminal_sequence_mismatch")
            raise BufferedTransportDriverError("terminal sequence mismatch")
        try:
            self._scheduler.observe_terminal_motion_result(response.result)
        except BufferedActionAdapterError as exc:
            raise BufferedTransportDriverError(
                "buffered terminal failed host admission"
            ) from exc
