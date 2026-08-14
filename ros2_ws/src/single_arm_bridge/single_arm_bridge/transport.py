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
    MAX_PAYLOAD,
    BufferedSetpointFlags,
    BufferedSetpointSample,
    Frame,
    Hello,
    MessageType,
    MotionResult,
    ProtocolError,
    RightArmDiscovery,
    RightArmConfiguration,
    RightArmConfigureOnce,
    RightArmDisable,
    RightArmJogOnce,
    RightArmTorqueEnableOnce,
    ServoDiagnostics,
    State,
    decode_frame,
    encode_buffered_setpoint_payload,
    encode_frame,
    parse_hello,
    parse_right_arm_disable,
    parse_right_arm_discovery,
    parse_right_arm_configuration,
    parse_right_arm_configure_once,
    parse_right_arm_jog_once,
    parse_right_arm_torque_enable_once,
    parse_servo_diagnostic,
    parse_setpoint_status,
    parse_state,
)


POSITION_STATE_RESPONSE_TIMEOUT_S = 0.5
# DISABLE performs six torque-off writes and six physical readbacks before
# acknowledging. With the bounded 0x218 READ parser and WRITE-reply settling,
# its firmware timeout envelope is about 1.529 seconds, so this operation needs
# a dedicated bound instead of the regular state timeout.
DISABLE_RESPONSE_TIMEOUT_S = 2.5
DIAGNOSTIC_RESPONSE_TIMEOUT_S = 0.5
DIAGNOSTIC_CAPABILITY = 0x00000010
RIGHT_ARM_READ_ONLY_DISCOVERY_CAPABILITY = 0x00020000
RIGHT_ARM_JOG_ONCE_CAPABILITY = 0x00040000
RIGHT_ARM_TORQUE_ENABLE_ONCE_CAPABILITY = 0x00080000
RIGHT_ARM_CONFIGURATION_SNAPSHOT_CAPABILITY = 0x00100000
RIGHT_ARM_CONFIGURE_ONCE_CAPABILITY = 0x00200000
RIGHT_ARM_VERIFIED_DISABLE_CAPABILITY = 0x00400000
RIGHT_ARM_DISABLE_RESPONSE_TIMEOUT_S = 0.8
# The MCU latches if it does not *process* a heartbeat within
# HOST_BINARY_HEARTBEAT_TIMEOUT_MS (500 ms); the same starvation that delays a
# response also delays feeding that watchdog. A host budget below the MCU's own
# limit made the host give up first and reported a transport fault on a link
# that was merely busy. Keep the budget under the MCU limit with room for the
# next send, so the MCU decides when the link is dead.
HEARTBEAT_RESPONSE_TIMEOUT_S = 0.40
MCU_HEARTBEAT_WATCHDOG_TIMEOUT_S = 0.50
# A frame is at most a 16-byte header plus MAX_PAYLOAD plus CRC, and COBS
# adds about one byte per 254. Two frames of slack is ample.
RX_RESIDUAL_LIMIT_BYTES = 2 * (MAX_PAYLOAD + 64)
BUFFERED_VALIDATION_ROUTE_CAPABILITY = 0x00000400
BUFFERED_EXECUTION_ROUTE_CAPABILITY = 0x00000800


class TransportError(RuntimeError):
    pass


class StopLatchedError(TransportError):
    """The MCU reports a physical safety stop that requires explicit recovery.

    Carries the parsed state, which is not the same thing as ignoring the stop.
    The latch gates motion, not observation: the response decoded cleanly and
    a status of zero means the position read itself succeeded. Discarding it
    forced recovery tooling to choose between honouring the gate and reading
    the sensor, and an operator deciding whether a latched arm can be moved
    back inside its limits needs exactly those positions. A 2026-08-06 recovery
    stalled on that: the arm had sagged past the elbow maximum with torque off,
    and the only reading that could have said so was thrown away.
    """

    def __init__(self, message: str, state: object | None = None) -> None:
        super().__init__(message)
        self.state = state


class ResponseTimeoutError(TransportError):
    """
    A response did not arrive in time, with evidence of what the wait saw.

    A bare timeout cannot separate two very different faults: the host burning
    its budget consuming other traffic, or the MCU answering late on a quiet
    link. The 2026-08-06 buffered startup abort needed that distinction and the
    old message could not provide it, so the counts are carried here.
    """

    def __init__(
        self,
        message_type: Any,
        sequence: int,
        timeout_s: float,
        elapsed_s: float,
        empty_reads: int,
        undecodable_frames: int,
        observed_frames: tuple[str, ...],
        partial_read_lengths: tuple[int, ...] = (),
        rejected_packets: tuple[bytes, ...] = (),
    ) -> None:
        self.message_type = message_type
        self.sequence = sequence
        self.timeout_s = timeout_s
        self.elapsed_s = elapsed_s
        self.empty_reads = empty_reads
        self.undecodable_frames = undecodable_frames
        self.observed_frames = observed_frames
        self.partial_read_lengths = partial_read_lengths
        self.rejected_packets = rejected_packets
        observed = ",".join(observed_frames) if observed_frames else "none"
        super().__init__(
            f"timeout waiting for {message_type.name} "
            f"seq={sequence} elapsed_ms={elapsed_s * 1000.0:.1f} "
            f"budget_ms={timeout_s * 1000.0:.1f} "
            f"empty_reads={empty_reads} "
            f"undecodable={undecodable_frames} "
            f"observed={observed} "
            f"partial_lengths={','.join(str(v) for v in partial_read_lengths) or 'none'} "
            f"rejected={','.join(p.hex() for p in rejected_packets) or 'none'}"
        )


SERVO_BUS_FAILURE_NAMES = {
    0: "none",
    1: "tx",
    2: "rx_timeout",
    3: "uart",
    4: "header",
    5: "id",
    6: "length",
    7: "servo_status",
    8: "checksum",
    9: "recovery",
    10: "rx_overflow",
    11: "dma",
}


class ServoDiagnosticReadError(TransportError):
    """A long-form servo diagnostic read failed with UART/DMA evidence."""

    def __init__(self, joint_index: int, sample: Any) -> None:
        self.joint_index = joint_index
        self.sample = sample
        health = sample.bus_health
        if health is None:
            detail = "bus_health=legacy_unavailable"
        else:
            reason = SERVO_BUS_FAILURE_NAMES.get(
                health.failure_reason,
                f"unknown_{health.failure_reason}",
            )
            detail = (
                f"reason={reason} hal={health.hal_status} "
                f"servo_status=0x{health.servo_status:02X} "
                f"uart_error=0x{health.uart_error_code:08X} "
                f"uart_isr=0x{health.uart_isr:08X} "
                f"dma_error=0x{health.dma_error_code:08X} "
                f"received={health.received_bytes} "
                f"producer={health.producer_index} "
                f"transactions={health.transaction_count} "
                f"failures={health.failure_count} "
                f"recoveries={health.recovery_count} "
                f"timeouts={health.timeout_count} "
                f"overflows={health.overflow_count} "
                f"pe/ne/fe/ore/rto/dma="
                f"{health.pe_count}/{health.ne_count}/{health.fe_count}/"
                f"{health.ore_count}/{health.rto_count}/{health.dma_error_count}"
                f" lazy_arms={health.lazy_arm_count} "
                f"receiver_resyncs={health.receiver_resync_count} "
                f"receiver_armed={int(health.receiver_armed)} "
                f"snapshot={health.failure_snapshot.hex()}"
            )
        super().__init__(
            "diagnostic read failed: "
            f"joint_index={joint_index} status={sample.status_code} "
            f"read_status=0x{sample.read_status:02X} {detail}"
        )


class PositionReadError(TransportError):
    """A complete background position sweep exhausted its per-servo retries."""

    def __init__(
        self,
        servo_id: int,
        streak: int,
        limit: int,
        stop_latched: bool,
        reason: int = 0,
        hal_status: int = 0,
        servo_status: int = 0,
        recovery_count: int = 0,
        discarded_bytes: int = 0,
        uart_error_code: int = 0,
        uart_isr: int = 0,
        snapshot: bytes = b"",
        receiver_armed: bool = False,
    ) -> None:
        self.servo_id = servo_id
        self.streak = streak
        self.limit = limit
        self.stop_latched = stop_latched
        self.reason = reason
        self.hal_status = hal_status
        self.servo_status = servo_status
        self.recovery_count = recovery_count
        self.discarded_bytes = discarded_bytes
        self.uart_error_code = uart_error_code
        self.uart_isr = uart_isr
        self.snapshot = bytes(snapshot)
        self.receiver_armed = receiver_armed
        super().__init__(
            "GET_STATE position read failed: "
            f"servo_id={servo_id} "
            f"streak={streak}/{limit} "
            f"latched={int(stop_latched)} "
            f"cause={SERVO_BUS_FAILURE_NAMES.get(reason, f'unknown_{reason}')} "
            f"hal={hal_status} servo_status=0x{servo_status:02X} "
            f"recoveries={recovery_count} discarded={discarded_bytes} "
            f"uart_error=0x{uart_error_code:08X} "
            f"uart_isr=0x{uart_isr:08X} "
            f"receiver_armed={int(receiver_armed)} "
            f"snapshot={bytes(snapshot).hex()}"
        )


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
        # read_until returns whatever it has when its timeout expires, so a
        # frame that straddles that boundary comes back without its delimiter.
        # Every caller used to discard those bytes, which destroyed the frame
        # outright: the tail then decoded as garbage and the waiter timed out
        # even though the data had arrived. Two 2026-08-06 buffered startups
        # lost a 42-byte STATE_FEEDBACK exactly this way, split 24+18 and
        # 19+22, while the MCU reported every transmit clean. Carry the
        # fragment to the next read instead.
        self._rx_residual = bytearray()
        self._sequence = 1
        self.hello_info: Hello | None = None
        self._motion_results: deque[Any] = deque(maxlen=16)
        self._buffered_motion_results: deque[Any] = deque(maxlen=4)
        self._io_lock = threading.RLock()

    def _record_unsolicited_motion_result(self, frame: Frame) -> MotionResult:
        result = parse_setpoint_status(frame.payload)
        if result.status_code != 0:
            self._motion_results.append(result)
            if result.executor_state is not None:
                from .buffered_transport_driver import BufferedExchangeResponse

                self._buffered_motion_results.append(
                    BufferedExchangeResponse(frame.sequence, result)
                )
        return result

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

    def _read_framed_packet(self) -> bytes | None:
        """
        Return one delimiter-terminated packet, or None if none is complete yet.

        Fragments are retained across calls so a frame split by the read
        timeout is reassembled instead of lost.
        """
        if b"\x00" in self._rx_residual:
            index = self._rx_residual.index(b"\x00")
            packet = bytes(self._rx_residual[: index + 1])
            del self._rx_residual[: index + 1]
            return packet

        chunk = self._port.read_until(b"\x00")
        if not chunk:
            return None
        self._rx_residual.extend(chunk)
        if b"\x00" not in self._rx_residual:
            # Still incomplete. Bound the buffer so a stream that never
            # delivers a delimiter cannot grow without limit.
            if len(self._rx_residual) > RX_RESIDUAL_LIMIT_BYTES:
                del self._rx_residual[:-RX_RESIDUAL_LIMIT_BYTES]
            return None
        index = self._rx_residual.index(b"\x00")
        packet = bytes(self._rx_residual[: index + 1])
        del self._rx_residual[: index + 1]
        return packet

    def _receive_matching(
        self,
        sequence: int,
        message_type: MessageType,
        timeout_s: float | None = None,
        defer_state_after_motion_result: bool = False,
    ) -> Frame:
        response_timeout = self._timeout_s if timeout_s is None else timeout_s
        started = time.monotonic()
        deadline = started + response_timeout
        # A timeout alone cannot distinguish a host that burned its budget
        # consuming other traffic from an MCU that simply answered late. The
        # 2026-08-06 buffered startup abort needed exactly that distinction,
        # so record what the wait actually observed.
        empty_reads = 0
        undecodable = 0
        observed: list[str] = []
        # A count alone cannot say whether a rejected packet was a frame split
        # by the port read timeout, an intact frame with a bad checksum, or
        # noise. Keep a bounded sample of the actual bytes.
        partial_lengths: list[int] = []
        rejected_packets: list[bytes] = []
        while time.monotonic() < deadline:
            packet = self._read_framed_packet()
            if packet is None:
                empty_reads += 1
                if self._rx_residual and len(partial_lengths) < 4:
                    partial_lengths.append(len(self._rx_residual))
                continue
            try:
                frame = decode_frame(packet)
            except ProtocolError:
                undecodable += 1
                if len(rejected_packets) < 2:
                    rejected_packets.append(packet[:48])
                continue
            if frame.sequence == sequence and frame.message_type is message_type:
                return frame
            if len(observed) < 8:
                observed.append(f"{frame.message_type.name}#{frame.sequence}")
            if frame.message_type is MessageType.SETPOINT_STATUS:
                result = self._record_unsolicited_motion_result(frame)
                if result.status_code != 0:
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
        raise ResponseTimeoutError(
            message_type=message_type,
            sequence=sequence,
            timeout_s=response_timeout,
            elapsed_s=time.monotonic() - started,
            empty_reads=empty_reads,
            undecodable_frames=undecodable,
            observed_frames=tuple(observed),
            partial_read_lengths=tuple(partial_lengths),
            rejected_packets=tuple(rejected_packets),
        )

    def _collect_available_motion_results(self) -> None:
        while (
            int(getattr(self._port, "in_waiting", 0)) > 0
            or b"\x00" in self._rx_residual
        ):
            packet = self._read_framed_packet()
            if packet is None:
                # Incomplete frame retained for the next read rather than
                # dropped; dropping it is what destroyed responses before.
                break
            try:
                frame = decode_frame(packet)
            except ProtocolError:
                continue
            if frame.message_type is not MessageType.SETPOINT_STATUS:
                continue
            self._record_unsolicited_motion_result(frame)

    @_synchronized
    def drain_motion_results(self) -> list[Any]:
        # Motion completion is unsolicited. Collect only bytes already waiting
        # in the UART buffer; never issue GET_STATE or resend a motion command.
        self._collect_available_motion_results()
        results = list(self._motion_results)
        self._motion_results.clear()
        return results

    @_synchronized
    def drain_buffered_motion_results(self) -> list[Any]:
        """Return queued extended terminals with their outer frame identity."""

        self._collect_available_motion_results()
        results = list(self._buffered_motion_results)
        self._buffered_motion_results.clear()
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
    def heartbeat(self) -> State:
        sequence = self._send(MessageType.HEARTBEAT)
        state = parse_state(
            self._receive_matching(
                sequence,
                MessageType.STATE_FEEDBACK,
                timeout_s=HEARTBEAT_RESPONSE_TIMEOUT_S,
            ).payload
        )
        if state.stop_latched:
            raise StopLatchedError(
                "HEARTBEAT rejected: "
                f"status={state.status_code} latched=1",
                state=state,
            )
        if state.status_code != 0:
            raise TransportError(
                f"HEARTBEAT rejected: status={state.status_code} latched=0"
            )
        return state

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
        if (
            state.status_code == 2
            and state.position_read_failed_servo_id is not None
        ):
            raise PositionReadError(
                servo_id=state.position_read_failed_servo_id,
                streak=state.position_read_failure_streak,
                limit=state.position_read_failure_limit,
                stop_latched=state.stop_latched,
                reason=state.position_read_failure_reason,
                hal_status=state.position_read_hal_status,
                servo_status=state.position_read_servo_status,
                recovery_count=state.position_read_recovery_count,
                discarded_bytes=state.position_read_discarded_bytes,
                uart_error_code=state.position_read_uart_error_code,
                uart_isr=state.position_read_uart_isr,
                snapshot=state.position_read_snapshot,
                receiver_armed=state.position_read_receiver_armed,
            )
        if state.stop_latched:
            raise StopLatchedError(
                f"GET_STATE rejected: status={state.status_code} latched=1",
                state=state,
            )
        if state.status_code != 0:
            raise TransportError(f"GET_STATE status={state.status_code}")
        if include_positions and state.raw_positions is None:
            raise TransportError("position feedback missing")
        return state

    @_synchronized
    def get_diagnostics(self) -> ServoDiagnostics:
        hello = self.hello_info
        if hello is None:
            raise TransportError("diagnostics require a completed HELLO")
        if (hello.capabilities & DIAGNOSTIC_CAPABILITY) == 0:
            raise TransportError("firmware does not provide servo diagnostics")

        samples = []
        for joint_index in range(hello.joint_count):
            # One bounded servo transaction per request prevents a six-joint
            # snapshot from starving the MCU heartbeat watchdog.
            self.heartbeat()
            sequence = self._send(
                MessageType.GET_STATE,
                bytes((2, joint_index)),
            )
            sample = parse_servo_diagnostic(
                self._receive_matching(
                    sequence,
                    MessageType.DIAGNOSTICS,
                    timeout_s=DIAGNOSTIC_RESPONSE_TIMEOUT_S,
                ).payload
            )
            if sample.status_code != 0 or sample.read_status != 0:
                raise ServoDiagnosticReadError(joint_index, sample)
            if (
                sample.joint_index != joint_index
                or sample.joint_count != hello.joint_count
                or sample.protocol_version != hello.protocol_version
                or sample.calibration_hash != hello.calibration_hash
            ):
                raise TransportError(
                    f"diagnostic identity mismatch at joint_index={joint_index}"
                )
            samples.append(sample)

        return ServoDiagnostics(
            protocol_version=hello.protocol_version,
            joint_count=hello.joint_count,
            calibration_hash=hello.calibration_hash,
            joints=tuple(samples),
        )

    @_synchronized
    def discover_right_arm_read_only(self) -> RightArmDiscovery:
        hello = self.hello_info
        if hello is None:
            raise TransportError("right-arm discovery requires a completed HELLO")
        if (
            hello.capabilities & RIGHT_ARM_READ_ONLY_DISCOVERY_CAPABILITY
        ) == 0:
            raise TransportError(
                "firmware does not provide right-arm read-only discovery"
            )
        sequence = self._send(MessageType.RIGHT_ARM_DISCOVERY_REQUEST)
        snapshot = parse_right_arm_discovery(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_DISCOVERY_RESPONSE,
                timeout_s=0.5,
            ).payload
        )
        if snapshot.status_code == 1:
            raise TransportError(
                "right-arm discovery rejected while left-arm motion is active"
            )
        return snapshot

    @_synchronized
    def disable_right_arm_verified(self) -> RightArmDisable:
        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "right-arm verified disable requires a completed HELLO"
            )
        if (hello.capabilities & RIGHT_ARM_VERIFIED_DISABLE_CAPABILITY) == 0:
            raise TransportError(
                "firmware does not provide right-arm verified disable"
            )
        sequence = self._send(MessageType.RIGHT_ARM_DISABLE_REQUEST)
        snapshot = parse_right_arm_disable(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_DISABLE_RESPONSE,
                timeout_s=RIGHT_ARM_DISABLE_RESPONSE_TIMEOUT_S,
            ).payload
        )
        if snapshot.status_code != 0:
            raise TransportError(
                "right-arm verified disable failed: "
                f"status={snapshot.status_code} "
                f"torque_mask=0x{snapshot.torque_enabled_mask:02X} "
                f"failures={snapshot.failure_count}"
            )
        return snapshot

    @_synchronized
    def configure_right_arm_once(
        self, servo_id: int
    ) -> RightArmConfigureOnce:
        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "right-arm configure-once requires a completed HELLO"
            )
        if (hello.capabilities & RIGHT_ARM_CONFIGURE_ONCE_CAPABILITY) == 0:
            raise TransportError(
                "firmware does not provide right-arm configure-once"
            )
        if not 1 <= servo_id <= 6:
            raise TransportError("right-arm configure-once servo ID is invalid")
        sequence = self._send(
            MessageType.RIGHT_ARM_CONFIGURE_ONCE_REQUEST,
            bytes((servo_id,)),
        )
        return parse_right_arm_configure_once(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_CONFIGURE_ONCE_RESPONSE,
                timeout_s=0.4,
            ).payload
        )

    @_synchronized
    def read_right_arm_configuration(
        self, servo_id: int
    ) -> RightArmConfiguration:
        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "right-arm configuration read requires a completed HELLO"
            )
        if (hello.capabilities & RIGHT_ARM_CONFIGURATION_SNAPSHOT_CAPABILITY) == 0:
            raise TransportError(
                "firmware does not provide right-arm configuration snapshots"
            )
        if not 1 <= servo_id <= 6:
            raise TransportError("right-arm configuration servo ID is invalid")
        sequence = self._send(
            MessageType.RIGHT_ARM_CONFIGURATION_REQUEST,
            bytes((servo_id,)),
        )
        snapshot = parse_right_arm_configuration(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_CONFIGURATION_RESPONSE,
                timeout_s=0.4,
            ).payload
        )
        if snapshot.status_code == 1:
            raise TransportError(
                "right-arm configuration read rejected while motion is active"
            )
        return snapshot

    @_synchronized
    def jog_right_arm_once(self, servo_id: int, delta_raw: int) -> RightArmJogOnce:
        hello = self.hello_info
        if hello is None:
            raise TransportError("right-arm jog requires a completed HELLO")
        if (hello.capabilities & RIGHT_ARM_JOG_ONCE_CAPABILITY) == 0:
            raise TransportError("firmware does not provide right-arm jog-once")
        if not 1 <= servo_id <= 6 or not -128 <= delta_raw <= 127:
            raise TransportError("right-arm jog request is outside wire bounds")
        sequence = self._send(
            MessageType.RIGHT_ARM_JOG_ONCE_REQUEST,
            struct.pack("<Bb", servo_id, delta_raw),
        )
        return parse_right_arm_jog_once(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_JOG_ONCE_RESPONSE,
                timeout_s=0.5,
            ).payload
        )

    @_synchronized
    def enable_right_arm_torque_once(
        self,
        servo_id: int,
    ) -> RightArmTorqueEnableOnce:
        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "right-arm torque enable requires a completed HELLO"
            )
        if (hello.capabilities & RIGHT_ARM_TORQUE_ENABLE_ONCE_CAPABILITY) == 0:
            raise TransportError(
                "firmware does not provide right-arm torque-enable-once"
            )
        if not 1 <= servo_id <= 6:
            raise TransportError("right-arm torque-enable servo ID is invalid")
        sequence = self._send(
            MessageType.RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST,
            bytes((servo_id,)),
        )
        return parse_right_arm_torque_enable_once(
            self._receive_matching(
                sequence,
                MessageType.RIGHT_ARM_TORQUE_ENABLE_ONCE_RESPONSE,
                timeout_s=0.5,
            ).payload
        )

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
            # 거부 사유는 응답에 실려 온다. 버리면 어느 조건에서 막혔는지
            # 알 수 없어 펌웨어 소스를 뒤져야 한다. 2026-08-06 A5 복구에서
            # 실제로 그렇게 막혔다.
            #   result: 0=OK 1=BAD_STATE 2=ESTOP 3=HEALTH_FAILED
            #           4=CONFIG_MISMATCH  (actuator_safety_result_t)
            #   state : 2 를 기대한다 (ARMED)
            raise TransportError(
                f"ARM_REQUEST rejected result={result} state={state} "
                f"hash=0x{returned_hash:08X} "
                f"expected_hash=0x{calibration_hash:08X}"
            )
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
    def validate_buffered_candidate(
        self,
        first_apply_tick_ms: int,
        samples: tuple[BufferedSetpointSample, ...],
    ) -> MotionResult:
        """Validate a multi-sample batch without queueing or servo output."""
        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "buffered validation requires a completed HELLO"
            )
        if (
            hello.capabilities
            & BUFFERED_VALIDATION_ROUTE_CAPABILITY
        ) == 0:
            raise TransportError(
                "firmware does not provide buffered validation route"
            )

        payload = encode_buffered_setpoint_payload(
            first_apply_tick_ms,
            samples,
        )
        flags = int(
            BufferedSetpointFlags.VALIDATION_ONLY
            | BufferedSetpointFlags.CANDIDATE
            | BufferedSetpointFlags.BEGIN
            | BufferedSetpointFlags.START
            | BufferedSetpointFlags.END
        )
        sequence = self._send(
            MessageType.SETPOINT_BATCH,
            payload,
            flags,
        )
        result = parse_setpoint_status(
            self._receive_matching(
                sequence,
                MessageType.SETPOINT_STATUS,
            ).payload
        )
        if result.status_code != 5:
            raise TransportError(
                "buffered validation rejected: "
                f"status={result.status_code} detail={result.detail}"
            )
        if (
            result.executor_state is None
            or result.queued_samples != 0
            or result.accepted_samples != 0
            or result.applied_samples != 0
        ):
            raise TransportError(
                "buffered validation response does not prove no-motion state"
            )
        return result

    @_synchronized
    def exchange_buffered_command(
        self,
        command: Any,
        *,
        timeout_s: float,
    ) -> Any:
        """Exchange one physical buffered frame exactly once.

        Admission rejection remains a parsed response for the scheduler to
        classify.  Transport failures raise without any automatic resend.
        """
        from .buffered_transport_driver import BufferedExchangeResponse

        hello = self.hello_info
        if hello is None:
            raise TransportError(
                "buffered execution requires a completed HELLO"
            )
        if (
            hello.capabilities
            & BUFFERED_EXECUTION_ROUTE_CAPABILITY
        ) == 0:
            raise TransportError(
                "firmware does not provide buffered execution route"
            )
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not 0.0 < float(timeout_s) <= 1.0
        ):
            raise ValueError("buffered exchange timeout must be within 0..1s")

        sequence = self._send(
            MessageType.SETPOINT_BATCH,
            command.payload,
            int(command.flags),
        )
        frame = self._receive_matching(
            sequence,
            MessageType.SETPOINT_STATUS,
            timeout_s=float(timeout_s),
        )
        return BufferedExchangeResponse(
            frame_sequence=frame.sequence,
            result=parse_setpoint_status(frame.payload),
        )

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
