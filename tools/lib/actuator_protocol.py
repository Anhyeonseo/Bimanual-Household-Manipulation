"""Pi-side codec for the actuator protocol v1.

This module intentionally depends only on the Python standard library.  The
serial transport is kept separate so framing and integrity checks can be unit
tested without hardware or pyserial.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct


MAGIC = 0xA55A
VERSION = 1
MAX_PAYLOAD = 512
HEADER = struct.Struct("<HBBHHII")
CRC = struct.Struct("<I")
STATE_FEEDBACK_BASE = struct.Struct("<BBBBIIII")
STATE_FEEDBACK_POSITIONS = struct.Struct("<6H")
STATE_FEEDBACK_POSITION_READ_FAILURE_LEGACY = struct.Struct("<BBBB")
STATE_FEEDBACK_POSITION_READ_FAILURE = struct.Struct("<BBBBBBHH2xII")
STATE_FEEDBACK_POSITION_READ_FAILURE_V2 = struct.Struct("<BBBBBBHH2xIIBB16s")


class MessageType(IntEnum):
    HELLO_REQUEST = 1
    HELLO_RESPONSE = 2
    HEARTBEAT = 3
    TIME_SYNC_REQUEST = 4
    TIME_SYNC_RESPONSE = 5
    ARM_REQUEST = 16
    ARM_RESPONSE = 17
    ENABLE = 18
    HOLD = 19
    SAFE_STOP = 20
    DISABLE = 21
    CLEAR_FAULT = 22
    SETPOINT_BATCH = 32
    SETPOINT_STATUS = 33
    RIGHT_ARM_JOG_ONCE_REQUEST = 34
    RIGHT_ARM_JOG_ONCE_RESPONSE = 35
    RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST = 36
    RIGHT_ARM_TORQUE_ENABLE_ONCE_RESPONSE = 37
    RIGHT_ARM_CONFIGURE_ONCE_REQUEST = 38
    RIGHT_ARM_CONFIGURE_ONCE_RESPONSE = 39
    GET_STATE = 48
    STATE_FEEDBACK = 49
    FAULT_REPORT = 50
    DIAGNOSTICS = 51
    RIGHT_ARM_DISCOVERY_REQUEST = 52
    RIGHT_ARM_DISCOVERY_RESPONSE = 53
    RIGHT_ARM_CONFIGURATION_REQUEST = 54
    RIGHT_ARM_CONFIGURATION_RESPONSE = 55
    RIGHT_ARM_DISABLE_REQUEST = 56
    RIGHT_ARM_DISABLE_RESPONSE = 57


KNOWN_MESSAGE_TYPES = {int(message) for message in MessageType}


class ProtocolError(ValueError):
    """Raised when a frame violates the wire protocol contract."""


@dataclass(frozen=True, slots=True)
class Frame:
    message_type: MessageType
    flags: int = 0
    sequence: int = 0
    sender_time_ms: int = 0
    payload: bytes = b""


@dataclass(frozen=True, slots=True)
class StateFeedback:
    stop_latched: bool
    status_code: int
    joint_count: int
    protocol_version: int
    heartbeat_count: int
    rejected_frame_count: int
    calibration_hash: int
    last_heartbeat_ms: int
    raw_positions: tuple[int, ...] | None = None
    position_read_failed_servo_id: int | None = None
    position_read_failure_streak: int = 0
    position_read_failure_limit: int = 0
    position_read_failure_reason: int = 0
    position_read_hal_status: int = 0
    position_read_servo_status: int = 0
    position_read_recovery_count: int = 0
    position_read_discarded_bytes: int = 0
    position_read_uart_error_code: int = 0
    position_read_uart_isr: int = 0
    position_read_snapshot: bytes = b""
    position_read_receiver_armed: bool = False


def parse_state_feedback(payload: bytes) -> StateFeedback:
    """Parse state, position feedback, or position-read failure diagnostics."""

    base_size = STATE_FEEDBACK_BASE.size
    position_size = STATE_FEEDBACK_POSITIONS.size
    legacy_failure_size = STATE_FEEDBACK_POSITION_READ_FAILURE_LEGACY.size
    failure_size = STATE_FEEDBACK_POSITION_READ_FAILURE.size
    failure_v2_size = STATE_FEEDBACK_POSITION_READ_FAILURE_V2.size
    valid_lengths = (
        base_size,
        base_size + legacy_failure_size,
        base_size + failure_size,
        base_size + failure_v2_size,
        base_size + position_size,
    )
    if len(payload) not in valid_lengths:
        raise ProtocolError(
            "STATE_FEEDBACK payload must be "
            f"{base_size}, {base_size + legacy_failure_size}, "
            f"{base_size + failure_size}, {base_size + failure_v2_size}, or "
            f"{base_size + position_size} bytes"
        )

    (
        stop_latched,
        status_code,
        joint_count,
        protocol_version,
        heartbeat_count,
        rejected_frame_count,
        calibration_hash,
        last_heartbeat_ms,
    ) = STATE_FEEDBACK_BASE.unpack_from(payload)

    raw_positions = None
    failed_servo_id = None
    failure_streak = 0
    failure_limit = 0
    failure_reason = 0
    hal_status = 0
    servo_status = 0
    recovery_count = 0
    discarded_bytes = 0
    uart_error_code = 0
    uart_isr = 0
    failure_snapshot = b""
    receiver_armed = False
    if len(payload) == base_size + position_size:
        raw_positions = STATE_FEEDBACK_POSITIONS.unpack_from(payload, base_size)
    elif len(payload) == base_size + legacy_failure_size:
        (
            failed_servo_id,
            failure_streak,
            failure_limit,
            _,
        ) = STATE_FEEDBACK_POSITION_READ_FAILURE_LEGACY.unpack_from(
            payload, base_size
        )
    elif len(payload) == base_size + failure_size:
        (
            failed_servo_id,
            failure_streak,
            failure_limit,
            failure_reason,
            hal_status,
            servo_status,
            recovery_count,
            discarded_bytes,
            uart_error_code,
            uart_isr,
        ) = STATE_FEEDBACK_POSITION_READ_FAILURE.unpack_from(payload, base_size)
    elif len(payload) == base_size + failure_v2_size:
        (
            failed_servo_id,
            failure_streak,
            failure_limit,
            failure_reason,
            hal_status,
            servo_status,
            recovery_count,
            discarded_bytes,
            uart_error_code,
            uart_isr,
            snapshot_length,
            receiver_armed_raw,
            snapshot_raw,
        ) = STATE_FEEDBACK_POSITION_READ_FAILURE_V2.unpack_from(payload, base_size)
        failure_snapshot = snapshot_raw[:snapshot_length]
        receiver_armed = receiver_armed_raw != 0

    return StateFeedback(
        stop_latched=stop_latched != 0,
        status_code=status_code,
        joint_count=joint_count,
        protocol_version=protocol_version,
        heartbeat_count=heartbeat_count,
        rejected_frame_count=rejected_frame_count,
        calibration_hash=calibration_hash,
        last_heartbeat_ms=last_heartbeat_ms,
        raw_positions=raw_positions,
        position_read_failed_servo_id=failed_servo_id,
        position_read_failure_streak=failure_streak,
        position_read_failure_limit=failure_limit,
        position_read_failure_reason=failure_reason,
        position_read_hal_status=hal_status,
        position_read_servo_status=servo_status,
        position_read_recovery_count=recovery_count,
        position_read_discarded_bytes=discarded_bytes,
        position_read_uart_error_code=uart_error_code,
        position_read_uart_isr=uart_isr,
        position_read_snapshot=failure_snapshot,
        position_read_receiver_armed=receiver_armed,
    )


def crc32c(data: bytes) -> int:
    """Return reflected Castagnoli CRC-32C, matching the STM32 C core."""

    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0x82F63B78 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


def cobs_encode(data: bytes) -> bytes:
    output = bytearray(b"\x00")
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(encoded: bytes) -> bytes:
    if not encoded:
        raise ProtocolError("empty COBS frame")

    output = bytearray()
    index = 0
    while index < len(encoded):
        code = encoded[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1

        block_end = index + code - 1
        if block_end > len(encoded):
            raise ProtocolError("truncated COBS block")
        output.extend(encoded[index:block_end])
        index = block_end

        if code != 0xFF and index < len(encoded):
            output.append(0)

    return bytes(output)


def encode_frame(frame: Frame) -> bytes:
    message_type = int(frame.message_type)
    if message_type not in KNOWN_MESSAGE_TYPES:
        raise ProtocolError(f"unknown message type: {message_type}")
    if not 0 <= frame.flags <= 0xFFFF:
        raise ProtocolError("flags do not fit uint16")
    if not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ProtocolError("sequence does not fit uint32")
    if not 0 <= frame.sender_time_ms <= 0xFFFFFFFF:
        raise ProtocolError("sender_time_ms does not fit uint32")
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 512 bytes")

    header = HEADER.pack(
        MAGIC,
        VERSION,
        message_type,
        frame.flags,
        len(frame.payload),
        frame.sequence,
        frame.sender_time_ms,
    )
    decoded = header + frame.payload
    decoded += CRC.pack(crc32c(decoded))
    return cobs_encode(decoded) + b"\x00"


def decode_frame(encoded_with_optional_delimiter: bytes) -> Frame:
    encoded = encoded_with_optional_delimiter
    if encoded.endswith(b"\x00"):
        encoded = encoded[:-1]
    decoded = cobs_decode(encoded)

    minimum_size = HEADER.size + CRC.size
    if len(decoded) < minimum_size:
        raise ProtocolError("frame is shorter than header plus CRC")

    magic, version, raw_type, flags, payload_length, sequence, sender_time_ms = (
        HEADER.unpack_from(decoded)
    )
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError("bad protocol version")
    if raw_type not in KNOWN_MESSAGE_TYPES:
        raise ProtocolError("unknown message type")
    if payload_length > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 512 bytes")

    expected_size = HEADER.size + payload_length + CRC.size
    if len(decoded) != expected_size:
        raise ProtocolError("payload length does not match frame length")

    expected_crc = CRC.unpack_from(decoded, HEADER.size + payload_length)[0]
    actual_crc = crc32c(decoded[: HEADER.size + payload_length])
    if actual_crc != expected_crc:
        raise ProtocolError("bad CRC-32C")

    payload = decoded[HEADER.size : HEADER.size + payload_length]
    return Frame(
        message_type=MessageType(raw_type),
        flags=flags,
        sequence=sequence,
        sender_time_ms=sender_time_ms,
        payload=payload,
    )


class StreamDecoder:
    """Collect a byte stream and yield validated frames at 0x00 delimiters."""

    def __init__(self, maximum_encoded_size: int = 535) -> None:
        self._buffer = bytearray()
        self._dropping = False
        self._maximum_encoded_size = maximum_encoded_size

    def push(self, byte: int) -> Frame | None:
        if not 0 <= byte <= 0xFF:
            raise ValueError("byte must fit uint8")

        if byte != 0:
            if self._dropping:
                return None
            if len(self._buffer) >= self._maximum_encoded_size:
                self._buffer.clear()
                self._dropping = True
                return None
            self._buffer.append(byte)
            return None

        if self._dropping:
            self._dropping = False
            self._buffer.clear()
            return None
        if not self._buffer:
            return None

        encoded = bytes(self._buffer)
        self._buffer.clear()
        return decode_frame(encoded)
