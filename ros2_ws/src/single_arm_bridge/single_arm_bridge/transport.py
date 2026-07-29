"""Synchronous serial session used by the ROS node and unit tests."""

from __future__ import annotations

import struct
import threading
import time
from collections import deque
from functools import wraps
from typing import Any

from .protocol import (
    ARM_RESPONSE,
    Frame,
    Hello,
    MessageType,
    MotionResult,
    ProtocolError,
    State,
    decode_frame,
    encode_frame,
    parse_hello,
    parse_setpoint_status,
    parse_state,
)


POSITION_STATE_RESPONSE_TIMEOUT_S = 0.5
DISABLE_RESPONSE_TIMEOUT_S = 0.5


class TransportError(RuntimeError):
    pass


class StateResponseDeferred(TransportError):
    """A valid terminal result superseded this feedback cycle."""


def _synchronized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._io_lock:
            return method(self, *args, **kwargs)

    return wrapped


class ActuatorTransport:
    def __init__(self, port: Any, response_timeout_s: float = 0.4) -> None:
        self._port = port
        self._timeout_s = response_timeout_s
        self._sequence = 1
        self.hello_info: Hello | None = None
        self._motion_results: deque[Any] = deque(maxlen=16)
        self._io_lock = threading.RLock()

    def _next_sequence(self) -> int:
        result = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        return result

    @staticmethod
    def _now_ms() -> int:
        return int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    def _send(
        self,
        message_type: MessageType,
        payload: bytes = b"",
        flags: int = 0,
    ) -> int:
        sequence = self._next_sequence()
        self._port.write(
            encode_frame(
                Frame(message_type, flags, sequence, self._now_ms(), payload)
            )
        )
        self._port.flush()
        return sequence

    def _receive_matching(
        self,
        sequence: int,
        message_type: MessageType,
        timeout_s: float | None = None,
        defer_state_after_motion_result: bool = False,
    ) -> Frame:
        response_timeout = self._timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + response_timeout
        while time.monotonic() < deadline:
            packet = self._port.read_until(b"\x00")
            if not packet.endswith(b"\x00"):
                continue
            try:
                frame = decode_frame(packet)
            except ProtocolError:
                continue
            if frame.sequence == sequence and frame.message_type is message_type:
                return frame
            if frame.message_type is MessageType.SETPOINT_STATUS:
                result = parse_setpoint_status(frame.payload)
                if result.status_code != 0:
                    self._motion_results.append(result)
                    if defer_state_after_motion_result:
                        # A valid terminal result proves that the link is alive,
                        # but the MCU may omit a GET_STATE response while final
                        # servo verification is finishing. Defer feedback to the
                        # next regular timer cycle instead of treating this as a
                        # transport failure or immediately retrying in the same
                        # busy window.
                        raise StateResponseDeferred(
                            "terminal motion result superseded state response"
                        )
        raise TransportError(f"timeout waiting for {message_type.name}")

    def _collect_available_motion_results(self) -> None:
        while int(getattr(self._port, "in_waiting", 0)) > 0:
            packet = self._port.read_until(b"\x00")
            if not packet.endswith(b"\x00"):
                break
            try:
                frame = decode_frame(packet)
            except ProtocolError:
                continue
            if frame.message_type is not MessageType.SETPOINT_STATUS:
                continue
            result = parse_setpoint_status(frame.payload)
            if result.status_code != 0:
                self._motion_results.append(result)

    @_synchronized
    def drain_motion_results(self) -> list[Any]:
        # Motion completion is unsolicited. Collect only bytes already waiting
        # in the UART buffer; never issue GET_STATE or resend a motion command.
        self._collect_available_motion_results()
        results = list(self._motion_results)
        self._motion_results.clear()
        return results

    @_synchronized
    def enter_binary_mode(self) -> Hello:
        self._port.reset_input_buffer()
        self._port.write(b"P")
        self._port.flush()

        acknowledgement = ""
        acknowledgement_deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < acknowledgement_deadline:
            line = self._port.readline().decode("ascii", errors="replace").strip()
            if line == "BINARY_PROTOCOL_READY_RESET_TO_EXIT":
                acknowledgement = line
                break
            if not line:
                break

        if not acknowledgement:
            # A previous host may already have switched the MCU to binary mode.
            # Terminate any partial ASCII byte buffered by the COBS parser, then
            # prove the mode with HELLO instead of requiring a physical reset.
            self._port.write(b"\x00")
            self._port.flush()
            self._port.reset_input_buffer()

        sequence = self._send(MessageType.HELLO_REQUEST)
        try:
            hello = parse_hello(
                self._receive_matching(sequence, MessageType.HELLO_RESPONSE).payload
            )
        except Exception as error:
            raise TransportError(
                "binary mode entry/reconnect failed; press RESET and retry"
            ) from error
        if hello.protocol_version != 1 or hello.joint_count != 6:
            raise TransportError("protocol version or joint count mismatch")
        if (hello.capabilities & 0x00000008) == 0:
            raise TransportError("firmware does not provide position feedback")
        self.hello_info = hello
        return hello

    @_synchronized
    def heartbeat(self) -> None:
        self._send(MessageType.HEARTBEAT)

    @_synchronized
    def get_state(self, include_positions: bool = True) -> State:
        payload = b"\x01" if include_positions else b""
        sequence = self._send(MessageType.GET_STATE, payload)
        state = parse_state(
            self._receive_matching(
                sequence,
                MessageType.STATE_FEEDBACK,
                timeout_s=(
                    POSITION_STATE_RESPONSE_TIMEOUT_S
                    if include_positions
                    else None
                ),
                defer_state_after_motion_result=True,
            ).payload
        )
        if state.status_code != 0:
            raise TransportError(f"GET_STATE status={state.status_code}")
        if include_positions and state.raw_positions is None:
            raise TransportError("position feedback missing")
        return state

    @_synchronized
    def arm_and_enable(self, calibration_hash: int) -> None:
        sequence = self._send(
            MessageType.ARM_REQUEST,
            struct.pack("<I", calibration_hash),
        )
        result, state, returned_hash = ARM_RESPONSE.unpack(
            self._receive_matching(
                sequence,
                MessageType.ARM_RESPONSE,
                timeout_s=1.5,
            ).payload
        )
        if result != 0 or state != 2 or returned_hash != calibration_hash:
            raise TransportError("ARM_REQUEST rejected")
        self.heartbeat()
        sequence = self._send(MessageType.ENABLE)
        enabled = parse_state(
            self._receive_matching(sequence, MessageType.STATE_FEEDBACK).payload
        )
        if enabled.status_code != 0 or enabled.stop_latched:
            raise TransportError("ENABLE rejected")

    @_synchronized
    def send_setpoint(
        self,
        positions_urad: list[int],
        duration_ms: int,
    ) -> MotionResult:
        if len(positions_urad) != 6 or not 300 <= duration_ms <= 2000:
            raise ValueError("six positions and duration 300..2000ms are required")
        self.heartbeat()
        state = self.get_state(include_positions=False)
        apply_tick = (state.last_heartbeat_ms + duration_ms) & 0xFFFFFFFF
        payload = struct.pack(
            "<IBBH" + "I" + "i" * 12,
            apply_tick,
            1,
            1,
            0,
            0,
            *positions_urad,
            *([0] * 6),
        )
        sequence = self._send(MessageType.SETPOINT_BATCH, payload)
        result = parse_setpoint_status(
            self._receive_matching(sequence, MessageType.SETPOINT_STATUS).payload
        )
        if result.status_code != 0:
            raise TransportError(
                f"SETPOINT_BATCH rejected: status={result.status_code}"
            )
        return result

    @_synchronized
    def safe_stop(self) -> None:
        sequence = self._send(MessageType.SAFE_STOP)
        state = parse_state(
            self._receive_matching(
                sequence,
                MessageType.STATE_FEEDBACK,
                timeout_s=0.5,
            ).payload
        )
        if not state.stop_latched:
            raise TransportError("SAFE_STOP was not latched")

    @_synchronized
    def disable(self) -> None:
        sequence = self._send(MessageType.DISABLE)
        state = parse_state(
            self._receive_matching(
                sequence,
                MessageType.STATE_FEEDBACK,
                timeout_s=DISABLE_RESPONSE_TIMEOUT_S,
            ).payload
        )
        if state.status_code != 0:
            raise TransportError(f"DISABLE rejected: status={state.status_code}")

    @_synchronized
    def clear_fault(self) -> State:
        sequence = self._send(MessageType.CLEAR_FAULT)
        state = parse_state(
            self._receive_matching(
                sequence,
                MessageType.STATE_FEEDBACK,
                timeout_s=0.5,
            ).payload
        )
        if state.status_code != 0 or state.stop_latched:
            raise TransportError(
                f"CLEAR_FAULT rejected: status={state.status_code} "
                f"latched={int(state.stop_latched)}"
            )
        return state
