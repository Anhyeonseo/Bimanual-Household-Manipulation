"""Binary actuator protocol codec. This module has no ROS dependency."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct


MAGIC = 0xA55A
VERSION = 1
MAX_PAYLOAD = 512
HEADER = struct.Struct("<HBBHHII")
CRC = struct.Struct("<I")
HELLO_PAYLOAD = struct.Struct("<BBBBIIII")
STATE_BASE = struct.Struct("<BBBBIIII")
STATE_POSITIONS = struct.Struct("<6H")
STATE_POSITION_READ_FAILURE_LEGACY = struct.Struct("<BBBB")
STATE_POSITION_READ_FAILURE = struct.Struct("<BBBBBBHH2xII")
ARM_RESPONSE = struct.Struct("<BB2xI")
SETPOINT_STATUS = struct.Struct("<BBBBIII")
SETPOINT_STATUS_EXTENDED = struct.Struct("<BBBBHHII")
BUFFERED_SETPOINT_HEADER = struct.Struct("<IBBH")
BUFFERED_SETPOINT_SAMPLE = struct.Struct("<I12i")
BUFFERED_SETPOINT_MAX_SAMPLES = 9
DIAGNOSTICS_BASE = struct.Struct("<BBBBII")
DIAGNOSTICS_JOINT = struct.Struct("<8B7H2B2H2BH4B")


class MessageType(IntEnum):
    HELLO_REQUEST = 1
    HELLO_RESPONSE = 2
    HEARTBEAT = 3
    ARM_REQUEST = 16
    ARM_RESPONSE = 17
    ENABLE = 18
    HOLD = 19
    SAFE_STOP = 20
    DISABLE = 21
    CLEAR_FAULT = 22
    SETPOINT_BATCH = 32
    SETPOINT_STATUS = 33
    GET_STATE = 48
    STATE_FEEDBACK = 49
    DIAGNOSTICS = 51


class BufferedSetpointFlags(IntFlag):
    """Dormant Motion-3 frame flags; not advertised by firmware 0x00021800."""

    VALIDATION_ONLY = 0x0001
    CANDIDATE = 0x0002
    BEGIN = 0x0004
    START = 0x0008
    END = 0x0010


BUFFERED_SETPOINT_FLAG_MASK = int(
    BufferedSetpointFlags.VALIDATION_ONLY
    | BufferedSetpointFlags.CANDIDATE
    | BufferedSetpointFlags.BEGIN
    | BufferedSetpointFlags.START
    | BufferedSetpointFlags.END
)


KNOWN_TYPES = {int(value) for value in MessageType}


class ProtocolError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class Frame:
    message_type: MessageType
    flags: int = 0
    sequence: int = 0
    sender_time_ms: int = 0
    payload: bytes = b""


@dataclass(frozen=True, slots=True)
class Hello:
    protocol_version: int
    joint_count: int
    stop_latched: bool
    firmware_version: int
    calibration_hash: int
    capabilities: int
    rejected_frame_count: int


@dataclass(frozen=True, slots=True)
class State:
    stop_latched: bool
    status_code: int
    joint_count: int
    protocol_version: int
    heartbeat_count: int
    rejected_frame_count: int
    calibration_hash: int
    last_heartbeat_ms: int
    raw_positions: tuple[int, ...] | None
    position_read_failed_servo_id: int | None
    position_read_failure_streak: int
    position_read_failure_limit: int
    position_read_failure_reason: int
    position_read_hal_status: int
    position_read_servo_status: int
    position_read_recovery_count: int
    position_read_discarded_bytes: int
    position_read_uart_error_code: int
    position_read_uart_isr: int


@dataclass(frozen=True, slots=True)
class MotionResult:
    status_code: int
    sample_count: int
    safety_state: int
    detail: int
    request_sequence: int
    apply_tick_ms: int
    calibration_hash: int
    executor_state: int | None = None
    terminal_reason: int | None = None
    safe_stop_required: bool | None = None
    queue_result: int | None = None
    queued_samples: int | None = None
    peak_queued_samples: int | None = None
    accepted_samples: int | None = None
    applied_samples: int | None = None


@dataclass(frozen=True, slots=True)
class BufferedSetpointSample:
    tick_offset_ms: int
    positions_urad: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class ServoDiagnostic:
    status_code: int
    joint_index: int
    joint_count: int
    protocol_version: int
    calibration_hash: int
    sample_time_ms: int
    servo_id: int
    read_status: int
    torque_enabled: bool
    p_gain: int
    d_gain: int
    i_gain: int
    voltage_raw: int
    temperature_c: int
    position_raw: int
    speed_raw: int
    load_raw: int
    current_raw: int
    torque_limit_raw: int
    goal_position_raw: int
    model_number: int
    firmware_major_version: int
    firmware_minor_version: int
    maximum_torque_limit_raw: int
    minimum_startup_force_raw: int
    cw_dead_zone_raw: int
    ccw_dead_zone_raw: int
    protection_current_raw: int
    operating_mode: int
    protective_torque_raw: int
    protection_time_raw: int
    overload_torque_raw: int


@dataclass(frozen=True, slots=True)
class ServoDiagnostics:
    protocol_version: int
    joint_count: int
    calibration_hash: int
    joints: tuple[ServoDiagnostic, ...]


def crc32c(data: bytes) -> int:
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
            raise ProtocolError("zero inside COBS frame")
        index += 1
        block_end = index + code - 1
        if block_end > len(encoded):
            raise ProtocolError("truncated COBS frame")
        output.extend(encoded[index:block_end])
        index = block_end
        if code != 0xFF and index < len(encoded):
            output.append(0)
    return bytes(output)


def encode_frame(frame: Frame) -> bytes:
    if int(frame.message_type) not in KNOWN_TYPES:
        raise ProtocolError("unknown message type")
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload too large")
    header = HEADER.pack(
        MAGIC,
        VERSION,
        int(frame.message_type),
        frame.flags,
        len(frame.payload),
        frame.sequence,
        frame.sender_time_ms,
    )
    decoded = header + frame.payload
    return cobs_encode(decoded + CRC.pack(crc32c(decoded))) + b"\x00"


def decode_frame(packet: bytes) -> Frame:
    encoded = packet[:-1] if packet.endswith(b"\x00") else packet
    decoded = cobs_decode(encoded)
    if len(decoded) < HEADER.size + CRC.size:
        raise ProtocolError("short frame")
    magic, version, raw_type, flags, length, sequence, sender_ms = (
        HEADER.unpack_from(decoded)
    )
    if magic != MAGIC or version != VERSION or raw_type not in KNOWN_TYPES:
        raise ProtocolError("invalid header")
    expected_length = HEADER.size + length + CRC.size
    if len(decoded) != expected_length:
        raise ProtocolError("invalid payload length")
    expected_crc = CRC.unpack_from(decoded, HEADER.size + length)[0]
    if crc32c(decoded[: HEADER.size + length]) != expected_crc:
        raise ProtocolError("bad CRC-32C")
    return Frame(
        message_type=MessageType(raw_type),
        flags=flags,
        sequence=sequence,
        sender_time_ms=sender_ms,
        payload=decoded[HEADER.size : HEADER.size + length],
    )


def parse_hello(payload: bytes) -> Hello:
    if len(payload) != HELLO_PAYLOAD.size:
        raise ProtocolError("invalid HELLO_RESPONSE length")
    version, count, stop, _, firmware, cal_hash, capabilities, rejected = (
        HELLO_PAYLOAD.unpack(payload)
    )
    return Hello(version, count, stop != 0, firmware, cal_hash, capabilities, rejected)


def parse_state(payload: bytes) -> State:
    valid_lengths = (
        STATE_BASE.size,
        STATE_BASE.size + STATE_POSITION_READ_FAILURE_LEGACY.size,
        STATE_BASE.size + STATE_POSITION_READ_FAILURE.size,
        STATE_BASE.size + STATE_POSITIONS.size,
    )
    if len(payload) not in valid_lengths:
        raise ProtocolError("invalid STATE_FEEDBACK length")
    values = STATE_BASE.unpack_from(payload)
    positions = None
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
    if len(payload) == STATE_BASE.size + STATE_POSITIONS.size:
        positions = STATE_POSITIONS.unpack_from(payload, STATE_BASE.size)
    elif len(payload) == STATE_BASE.size + STATE_POSITION_READ_FAILURE_LEGACY.size:
        (
            failed_servo_id,
            failure_streak,
            failure_limit,
            _,
        ) = STATE_POSITION_READ_FAILURE_LEGACY.unpack_from(
            payload, STATE_BASE.size
        )
    elif len(payload) == STATE_BASE.size + STATE_POSITION_READ_FAILURE.size:
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
        ) = STATE_POSITION_READ_FAILURE.unpack_from(payload, STATE_BASE.size)
    return State(
        stop_latched=values[0] != 0,
        status_code=values[1],
        joint_count=values[2],
        protocol_version=values[3],
        heartbeat_count=values[4],
        rejected_frame_count=values[5],
        calibration_hash=values[6],
        last_heartbeat_ms=values[7],
        raw_positions=positions,
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
    )


def encode_buffered_setpoint_payload(
    first_apply_tick_ms: int,
    samples: tuple[BufferedSetpointSample, ...],
) -> bytes:
    """Encode the dormant candidate batch payload without authorizing motion."""

    if (
        isinstance(first_apply_tick_ms, bool)
        or not isinstance(first_apply_tick_ms, int)
        or not 0 <= first_apply_tick_ms <= 0xFFFFFFFF
    ):
        raise ProtocolError("first apply tick is outside uint32")
    if not 1 <= len(samples) <= BUFFERED_SETPOINT_MAX_SAMPLES:
        raise ProtocolError("buffered batch requires 1..9 samples")
    payload = bytearray(
        BUFFERED_SETPOINT_HEADER.pack(first_apply_tick_ms, len(samples), 1, 0)
    )
    previous_offset: int | None = None
    for sample in samples:
        if (
            isinstance(sample.tick_offset_ms, bool)
            or not isinstance(sample.tick_offset_ms, int)
            or not 0 <= sample.tick_offset_ms <= 0xFFFFFFFF
        ):
            raise ProtocolError("sample tick offset is outside uint32")
        if previous_offset is not None and sample.tick_offset_ms <= previous_offset:
            raise ProtocolError("sample tick offsets must be strictly increasing")
        if previous_offset is not None and (
            sample.tick_offset_ms - previous_offset > 0x7FFFFFFF
        ):
            raise ProtocolError("sample tick delta exceeds uint32 half-range")
        if len(sample.positions_urad) != 6:
            raise ProtocolError("buffered sample requires six positions")
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not -(2**31) <= value < 2**31
            for value in sample.positions_urad
        ):
            raise ProtocolError("sample position is outside int32")
        payload.extend(BUFFERED_SETPOINT_SAMPLE.pack(
            sample.tick_offset_ms, *sample.positions_urad, *([0] * 6)))
        previous_offset = sample.tick_offset_ms
    return bytes(payload)


def validate_buffered_setpoint_flags(flags: int) -> BufferedSetpointFlags:
    if flags & ~BUFFERED_SETPOINT_FLAG_MASK:
        raise ProtocolError("unknown buffered setpoint flag")
    decoded = BufferedSetpointFlags(flags)
    if BufferedSetpointFlags.CANDIDATE not in decoded:
        raise ProtocolError("candidate flag is required")
    return decoded


def parse_setpoint_status(payload: bytes) -> MotionResult:
    extended_size = SETPOINT_STATUS.size + SETPOINT_STATUS_EXTENDED.size
    if len(payload) not in (SETPOINT_STATUS.size, extended_size):
        raise ProtocolError("invalid SETPOINT_STATUS length")
    base = SETPOINT_STATUS.unpack_from(payload)
    if len(payload) == SETPOINT_STATUS.size:
        return MotionResult(*base)
    extended = SETPOINT_STATUS_EXTENDED.unpack_from(payload, SETPOINT_STATUS.size)
    return MotionResult(
        *base,
        executor_state=extended[0], terminal_reason=extended[1],
        safe_stop_required=extended[2] != 0, queue_result=extended[3],
        queued_samples=extended[4], peak_queued_samples=extended[5],
        accepted_samples=extended[6], applied_samples=extended[7],
    )


def parse_servo_diagnostic(payload: bytes) -> ServoDiagnostic:
    expected_length = DIAGNOSTICS_BASE.size + DIAGNOSTICS_JOINT.size
    if len(payload) != expected_length:
        raise ProtocolError("invalid DIAGNOSTICS length")
    base = DIAGNOSTICS_BASE.unpack_from(payload)
    joint = DIAGNOSTICS_JOINT.unpack_from(payload, DIAGNOSTICS_BASE.size)
    return ServoDiagnostic(
        status_code=base[0],
        joint_index=base[1],
        joint_count=base[2],
        protocol_version=base[3],
        calibration_hash=base[4],
        sample_time_ms=base[5],
        servo_id=joint[0],
        read_status=joint[1],
        torque_enabled=joint[2] != 0,
        p_gain=joint[3],
        d_gain=joint[4],
        i_gain=joint[5],
        voltage_raw=joint[6],
        temperature_c=joint[7],
        position_raw=joint[8],
        speed_raw=joint[9],
        load_raw=joint[10],
        current_raw=joint[11],
        torque_limit_raw=joint[12],
        goal_position_raw=joint[13],
        model_number=joint[14],
        firmware_major_version=joint[15],
        firmware_minor_version=joint[16],
        maximum_torque_limit_raw=joint[17],
        minimum_startup_force_raw=joint[18],
        cw_dead_zone_raw=joint[19],
        ccw_dead_zone_raw=joint[20],
        protection_current_raw=joint[21],
        operating_mode=joint[22],
        protective_torque_raw=joint[23],
        protection_time_raw=joint[24],
        overload_torque_raw=joint[25],
    )
