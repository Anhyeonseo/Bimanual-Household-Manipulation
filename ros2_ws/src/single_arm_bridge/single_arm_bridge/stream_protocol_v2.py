"""Staged protocol-v2 stream contract with no ROS or hardware dependency.

The active robot remains on protocol v1 until firmware and host transport are
migrated together.  This module freezes the source-agnostic v2 wire layout:
one absolute 12-joint stream, one policy, and append/splice operations.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

from .protocol import (
    CRC,
    HEADER,
    MAGIC,
    MAX_PAYLOAD,
    cobs_decode,
    cobs_encode,
    crc32c,
)


PROTOCOL_VERSION = 2
ARM_COUNT = 2
JOINTS_PER_ARM = 6
JOINT_COUNT = 12
QUEUE_CAPACITY = 16
MAX_SAMPLES = 9
ARM_MASK_LEFT = 0x01
ARM_MASK_RIGHT = 0x02
ARM_MASK_BOTH = 0x03
MINIMUM_SPLICE_LEAD_MS = 20

STREAM_OPEN = struct.Struct("<HBBIIIII12i12i")
BATCH_HEADER = struct.Struct("<IIIIBBH")
BATCH_SAMPLE = struct.Struct("<I12i")
HELLO_V2 = struct.Struct("<BBBBIIIII")
STATE_V2 = struct.Struct("<BBBBIIIII")
STREAM_STATUS_V2 = struct.Struct("<BBBB8I")
EXECUTOR_DIAGNOSTICS_V2 = struct.Struct("<BBBB14I")
SHADOW_SNAPSHOT_V2 = struct.Struct("<BBBB12H12i")
UNWRAP_SHADOW_PREPARE_V2 = struct.Struct("<HH12i")
UNWRAPPED_SHADOW_SNAPSHOT_V2 = struct.Struct("<BBBB12H12i12i")
DISPATCH_DIAGNOSTICS_V2 = struct.Struct("<BBBB10I")
TRACKING_DIAGNOSTICS_V2 = struct.Struct("<BBBB18I")
FEEDBACK_SNAPSHOT_V2 = struct.Struct("<BBH4I12i12I")
MAX_BATCH_PAYLOAD_SIZE = BATCH_HEADER.size + MAX_SAMPLES * BATCH_SAMPLE.size


class StreamMessageTypeV2(IntEnum):
    HELLO_REQUEST = 1
    HELLO_RESPONSE = 2
    HEARTBEAT = 3
    ARM_REQUEST = 16
    ARM_RESPONSE = 17
    ENABLE = 18
    SAFE_STOP = 20
    DISABLE = 21
    CLEAR_FAULT = 22
    SETPOINT_BATCH = 32
    SETPOINT_STATUS = 33
    STREAM_OPEN = 40
    STREAM_STATUS = 41
    SPLICE = 42
    GET_EXECUTOR_DIAGNOSTICS = 43
    EXECUTOR_DIAGNOSTICS = 44
    PREPARE_SHADOW = 45
    SHADOW_SNAPSHOT = 46
    GET_DISPATCH_DIAGNOSTICS = 47
    GET_STATE = 48
    STATE_FEEDBACK = 49
    DISPATCH_DIAGNOSTICS = 58
    GET_TRACKING_DIAGNOSTICS = 59
    TRACKING_DIAGNOSTICS = 60
    GET_FEEDBACK_SNAPSHOT = 61
    FEEDBACK_SNAPSHOT = 62


class BatchKindV2(IntEnum):
    APPEND = 0
    SPLICE = 1


class StreamStatusCodeV2(IntEnum):
    OK = 0
    CONTRACT_REJECTED = 1
    NOT_OPEN = 2
    QUEUE_OVERFLOW = 3
    SPLICE_POSITION_UNAVAILABLE = 4
    VALIDATION_ONLY = 5


class StreamContractResultV2(IntEnum):
    OK = 0
    NULL_ARGUMENT = 1
    INVALID_LENGTH = 2
    INVALID_ARM_MASK = 3
    INVALID_RESERVED = 4
    INVALID_MINIMUM_START_SAMPLES = 5
    MINIMUM_LEAD_TOO_SMALL = 6
    MAXIMUM_LEAD_TOO_LARGE = 7
    LEAD_WINDOW_INVERTED = 8
    COMMAND_TIMEOUT_TOO_LARGE = 9
    OPEN_TIMEOUT_TOO_LARGE = 10
    APPLY_LATENESS_TOO_LARGE = 11
    TRACKING_ERROR_TOO_LARGE = 12
    MAXIMUM_STEP_TOO_LARGE = 13
    STALE_HORIZON = 14
    INVALID_SAMPLE_COUNT = 15
    NON_MONOTONIC_TICK = 16
    APPEND_EPOCH_MISMATCH = 17
    HORIZON_REGRESSION = 18
    HORIZON_BEFORE_LAST_SAMPLE = 19
    SPLICE_FIELD_MISMATCH = 20
    SPLICE_TOO_LATE = 21
    SPLICE_AFTER_LAST_SAMPLE = 22
    SPLICE_FIRST_TICK_MISMATCH = 23
    SPLICE_DISCONTINUITY = 24
    FIRST_SAMPLE_TOO_EARLY = 25
    LAST_SAMPLE_TOO_LATE = 26
    SAMPLE_DISCONTINUITY = 27


class StreamExecutorStateV2(IntEnum):
    CLOSED = 0
    PRIMING = 1
    RUNNING = 2
    HOLD = 3
    SUCCEEDED = 4
    ABORTED = 5


class StreamTerminalReasonV2(IntEnum):
    NONE = 0
    PLANNED_HORIZON = 1
    QUEUE_UNDERFLOW = 2
    COMMAND_TIMEOUT = 3
    MISSED_APPLY_TICK = 4
    TRACKING_ERROR = 5
    JOINT_LIMIT = 6
    INVALID_TIMELINE = 7


class StreamContractError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class FrameV2:
    message_type: StreamMessageTypeV2
    flags: int = 0
    sequence: int = 0
    sender_time_ms: int = 0
    payload: bytes = b""


@dataclass(frozen=True, slots=True)
class HelloV2:
    protocol_version: int
    joint_count: int
    stop_latched: bool
    firmware_version: int
    left_calibration_hash: int
    right_calibration_hash: int
    capabilities: int
    rejected_frame_count: int


@dataclass(frozen=True, slots=True)
class StateV2:
    stop_latched: bool
    status_code: int
    joint_count: int
    protocol_version: int
    heartbeat_count: int
    rejected_frame_count: int
    left_calibration_hash: int
    right_calibration_hash: int
    last_heartbeat_ms: int


@dataclass(frozen=True, slots=True)
class StreamStatusV2:
    status_code: int
    contract_result: int
    safety_state: int
    arm_mask: int
    request_sequence: int
    sender_time_ms_echo: int
    arbiter_epoch: int
    horizon_end_tick: int
    validated_tail_tick: int
    execution_queue_samples: int
    accepted_samples: int
    applied_samples: int


@dataclass(frozen=True, slots=True)
class StreamExecutorDiagnosticsV2:
    state: StreamExecutorStateV2
    terminal_reason: StreamTerminalReasonV2
    safe_stop_required: bool
    tracking_error_joint: int
    request_sequence: int
    sender_time_ms_echo: int
    arbiter_epoch: int
    horizon_end_tick: int
    validated_tail_tick: int
    queued_samples: int
    peak_queued_samples: int
    accepted_samples: int
    applied_samples: int
    control_outputs: int
    splice_count: int
    maximum_apply_lateness_ms: int
    last_command_tick: int
    terminal_tick: int


@dataclass(frozen=True, slots=True)
class StreamShadowSnapshotV2:
    status_code: int
    joint_count: int
    left_present_mask: int
    right_present_mask: int
    positions_raw: tuple[int, ...]
    anchor_positions_urad: tuple[int, ...]
    unwrapped_positions_raw: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class BimanualDispatchDiagnosticsV2:
    status_code: int
    active: bool
    faulted: bool
    ready: bool
    request_sequence: int
    sender_time_ms_echo: int
    launch_count: int
    completed_count: int
    failure_count: int
    maximum_start_skew_us: int
    maximum_launch_lateness_us: int
    last_control_tick_ms: int
    last_left_start_us: int
    last_right_start_us: int


@dataclass(frozen=True, slots=True)
class BimanualTrackingDiagnosticsV2:
    status_code: int
    active: bool
    pending: bool
    next_joint: int
    request_sequence: int
    sender_time_ms_echo: int
    requested_pairs: int
    completed_pairs: int
    failed_pairs: int
    maximum_reply_latency_ms: int
    maximum_tracking_error_urad: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class BimanualFeedbackSnapshotV2:
    status_code: int
    joint_count: int
    present_mask: int
    request_sequence: int
    sender_time_ms_echo: int
    firmware_tick_ms: int
    completed_pairs: int
    positions_urad: tuple[int, ...]
    sample_age_ms: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class StreamHardCapsV2:
    minimum_lead_ms: int
    maximum_lead_ms: int
    maximum_command_timeout_ms: int
    maximum_open_command_timeout_ms: int
    maximum_apply_lateness_ms: int
    tracking_error_limit_urad: tuple[int, ...]
    maximum_step_urad_per_tick: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class StreamPolicyV2:
    minimum_start_samples: int
    minimum_lead_ms: int
    horizon_end_tick: int
    maximum_lead_ms: int
    command_timeout_ms: int
    maximum_apply_lateness_ms: int
    tracking_error_limit_urad: tuple[int, ...]
    maximum_step_urad_per_tick: tuple[int, ...]
    arm_mask: int = ARM_MASK_BOTH


@dataclass(frozen=True, slots=True)
class StreamSampleV2:
    apply_tick: int
    positions_urad: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class StreamBatchV2:
    horizon_end_tick: int
    arbiter_epoch: int
    samples: tuple[StreamSampleV2, ...]
    arm_mask: int = ARM_MASK_BOTH
    splice_at_tick: int = 0


def _require_uint(name: str, value: int, bits: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise StreamContractError(f"{name} must be an integer")
    if not 0 <= value < (1 << bits):
        raise StreamContractError(f"{name} must fit uint{bits}")


def _require_int32_vector(name: str, values: tuple[int, ...]) -> None:
    if len(values) != JOINT_COUNT:
        raise StreamContractError(f"{name} must contain {JOINT_COUNT} joints")
    for value in values:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not -(1 << 31) <= value < (1 << 31)
        ):
            raise StreamContractError(f"{name} values must fit int32")


def _tick_after(candidate: int, reference: int) -> bool:
    difference = (candidate - reference) & 0xFFFFFFFF
    return 0 < difference < 0x80000000


def _validate_arm_mask(arm_mask: int) -> None:
    _require_uint("arm_mask", arm_mask, 8)
    if arm_mask == 0 or arm_mask & ~ARM_MASK_BOTH:
        raise StreamContractError("arm_mask must select left, right, or both")


def validate_stream_policy_v2(
    policy: StreamPolicyV2,
    hard_caps: StreamHardCapsV2,
    *,
    current_tick: int,
) -> None:
    _require_uint("current_tick", current_tick, 32)
    _validate_arm_mask(policy.arm_mask)
    if not 1 <= policy.minimum_start_samples <= QUEUE_CAPACITY:
        raise StreamContractError("minimum_start_samples is outside 1..16")
    if policy.minimum_lead_ms < hard_caps.minimum_lead_ms:
        raise StreamContractError("minimum_lead_ms loosens the hard cap")
    if policy.maximum_lead_ms > hard_caps.maximum_lead_ms:
        raise StreamContractError("maximum_lead_ms loosens the hard cap")
    if policy.minimum_lead_ms > policy.maximum_lead_ms:
        raise StreamContractError("lead window is inverted")
    if not 1 <= policy.command_timeout_ms <= hard_caps.maximum_command_timeout_ms:
        raise StreamContractError("command_timeout_ms loosens the hard cap")
    if (
        policy.horizon_end_tick == 0
        and policy.command_timeout_ms > hard_caps.maximum_open_command_timeout_ms
    ):
        raise StreamContractError("open stream command timeout loosens the hard cap")
    if policy.maximum_apply_lateness_ms > hard_caps.maximum_apply_lateness_ms:
        raise StreamContractError("maximum_apply_lateness_ms loosens the hard cap")
    if policy.horizon_end_tick and not _tick_after(
        policy.horizon_end_tick, current_tick
    ):
        raise StreamContractError("horizon_end_tick is stale")

    _require_int32_vector(
        "tracking_error_limit_urad", policy.tracking_error_limit_urad
    )
    _require_int32_vector(
        "maximum_step_urad_per_tick", policy.maximum_step_urad_per_tick
    )
    _require_int32_vector(
        "hard tracking_error_limit_urad", hard_caps.tracking_error_limit_urad
    )
    _require_int32_vector(
        "hard maximum_step_urad_per_tick",
        hard_caps.maximum_step_urad_per_tick,
    )
    for requested, hard in zip(
        policy.tracking_error_limit_urad,
        hard_caps.tracking_error_limit_urad,
        strict=True,
    ):
        if requested <= 0 or hard <= 0 or requested > hard:
            raise StreamContractError("tracking error limit loosens the hard cap")
    for requested, hard in zip(
        policy.maximum_step_urad_per_tick,
        hard_caps.maximum_step_urad_per_tick,
        strict=True,
    ):
        if requested <= 0 or hard <= 0 or requested > hard:
            raise StreamContractError("maximum step loosens the hard cap")


def encode_stream_open_v2(policy: StreamPolicyV2) -> bytes:
    _validate_arm_mask(policy.arm_mask)
    _require_uint("minimum_start_samples", policy.minimum_start_samples, 16)
    for name, value in (
        ("minimum_lead_ms", policy.minimum_lead_ms),
        ("horizon_end_tick", policy.horizon_end_tick),
        ("maximum_lead_ms", policy.maximum_lead_ms),
        ("command_timeout_ms", policy.command_timeout_ms),
        ("maximum_apply_lateness_ms", policy.maximum_apply_lateness_ms),
    ):
        _require_uint(name, value, 32)
    _require_int32_vector(
        "tracking_error_limit_urad", policy.tracking_error_limit_urad
    )
    _require_int32_vector(
        "maximum_step_urad_per_tick", policy.maximum_step_urad_per_tick
    )
    return STREAM_OPEN.pack(
        policy.minimum_start_samples,
        policy.arm_mask,
        0,
        policy.minimum_lead_ms,
        policy.horizon_end_tick,
        policy.maximum_lead_ms,
        policy.command_timeout_ms,
        policy.maximum_apply_lateness_ms,
        *policy.tracking_error_limit_urad,
        *policy.maximum_step_urad_per_tick,
    )


def encode_unwrap_shadow_prepare_v2(
    reference_unwrapped_raw: tuple[int, ...],
    maximum_reference_delta_raw: int,
) -> bytes:
    _require_int32_vector(
        "reference_unwrapped_raw", reference_unwrapped_raw
    )
    if not 1 <= maximum_reference_delta_raw < 2048:
        raise StreamContractError(
            "maximum reference delta must be within 1..2047"
        )
    return UNWRAP_SHADOW_PREPARE_V2.pack(
        maximum_reference_delta_raw,
        0,
        *reference_unwrapped_raw,
    )


def encode_stream_batch_v2(batch: StreamBatchV2, kind: BatchKindV2) -> bytes:
    _validate_arm_mask(batch.arm_mask)
    _require_uint("horizon_end_tick", batch.horizon_end_tick, 32)
    _require_uint("arbiter_epoch", batch.arbiter_epoch, 32)
    _require_uint("splice_at_tick", batch.splice_at_tick, 32)
    if not 1 <= len(batch.samples) <= MAX_SAMPLES:
        raise StreamContractError("sample count is outside 1..9")
    if kind is BatchKindV2.APPEND and batch.splice_at_tick != 0:
        raise StreamContractError("append must not carry splice_at_tick")
    if kind is BatchKindV2.SPLICE and batch.splice_at_tick == 0:
        raise StreamContractError("splice requires splice_at_tick")

    first_apply_tick = batch.samples[0].apply_tick
    _require_uint("first_apply_tick", first_apply_tick, 32)
    encoded_samples: list[bytes] = []
    previous_tick: int | None = None
    for sample in batch.samples:
        _require_uint("sample.apply_tick", sample.apply_tick, 32)
        _require_int32_vector("sample.positions_urad", sample.positions_urad)
        if previous_tick is not None and not _tick_after(
            sample.apply_tick, previous_tick
        ):
            raise StreamContractError("sample ticks must increase")
        tick_offset = (sample.apply_tick - first_apply_tick) & 0xFFFFFFFF
        if tick_offset >= 0x80000000:
            raise StreamContractError("sample tick offset is ambiguous")
        encoded_samples.append(
            BATCH_SAMPLE.pack(tick_offset, *sample.positions_urad)
        )
        previous_tick = sample.apply_tick

    return b"".join(
        (
            BATCH_HEADER.pack(
                first_apply_tick,
                batch.horizon_end_tick,
                batch.arbiter_epoch,
                batch.splice_at_tick,
                len(batch.samples),
                batch.arm_mask,
                0,
            ),
            *encoded_samples,
        )
    )


def encode_frame_v2(frame: FrameV2) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise StreamContractError("payload too large")
    for name, value, bits in (
        ("flags", frame.flags, 16),
        ("sequence", frame.sequence, 32),
        ("sender_time_ms", frame.sender_time_ms, 32),
    ):
        _require_uint(name, value, bits)
    header = HEADER.pack(
        MAGIC,
        PROTOCOL_VERSION,
        int(frame.message_type),
        frame.flags,
        len(frame.payload),
        frame.sequence,
        frame.sender_time_ms,
    )
    decoded = header + frame.payload
    return cobs_encode(decoded + CRC.pack(crc32c(decoded))) + b"\x00"


def decode_frame_v2(packet: bytes) -> FrameV2:
    encoded = packet[:-1] if packet.endswith(b"\x00") else packet
    try:
        decoded = cobs_decode(encoded)
    except Exception as error:
        raise StreamContractError("invalid COBS frame") from error
    if len(decoded) < HEADER.size + CRC.size:
        raise StreamContractError("short frame")
    magic, version, raw_type, flags, length, sequence, sender_ms = (
        HEADER.unpack_from(decoded)
    )
    try:
        message_type = StreamMessageTypeV2(raw_type)
    except ValueError as error:
        raise StreamContractError("unknown protocol v2 message") from error
    if magic != MAGIC or version != PROTOCOL_VERSION:
        raise StreamContractError("invalid protocol v2 header")
    expected_length = HEADER.size + length + CRC.size
    if length > MAX_PAYLOAD or len(decoded) != expected_length:
        raise StreamContractError("invalid protocol v2 payload length")
    expected_crc = CRC.unpack_from(decoded, HEADER.size + length)[0]
    if crc32c(decoded[: HEADER.size + length]) != expected_crc:
        raise StreamContractError("bad protocol v2 CRC-32C")
    return FrameV2(
        message_type=message_type,
        flags=flags,
        sequence=sequence,
        sender_time_ms=sender_ms,
        payload=decoded[HEADER.size : HEADER.size + length],
    )


def parse_hello_v2(payload: bytes) -> HelloV2:
    if len(payload) != HELLO_V2.size:
        raise StreamContractError("invalid protocol v2 HELLO length")
    values = HELLO_V2.unpack(payload)
    if values[0] != PROTOCOL_VERSION or values[1] != JOINT_COUNT or values[3] != 0:
        raise StreamContractError("invalid protocol v2 HELLO identity")
    return HelloV2(
        protocol_version=values[0],
        joint_count=values[1],
        stop_latched=values[2] != 0,
        firmware_version=values[4],
        left_calibration_hash=values[5],
        right_calibration_hash=values[6],
        capabilities=values[7],
        rejected_frame_count=values[8],
    )


def parse_state_v2(payload: bytes) -> StateV2:
    if len(payload) != STATE_V2.size:
        raise StreamContractError("invalid protocol v2 STATE length")
    values = STATE_V2.unpack(payload)
    if values[2] != JOINT_COUNT or values[3] != PROTOCOL_VERSION:
        raise StreamContractError("invalid protocol v2 STATE identity")
    return StateV2(
        stop_latched=values[0] != 0,
        status_code=values[1],
        joint_count=values[2],
        protocol_version=values[3],
        heartbeat_count=values[4],
        rejected_frame_count=values[5],
        left_calibration_hash=values[6],
        right_calibration_hash=values[7],
        last_heartbeat_ms=values[8],
    )


def parse_stream_status_v2(payload: bytes) -> StreamStatusV2:
    if len(payload) != STREAM_STATUS_V2.size:
        raise StreamContractError("invalid protocol v2 STREAM_STATUS length")
    values = STREAM_STATUS_V2.unpack(payload)
    return StreamStatusV2(
        status_code=values[0],
        contract_result=values[1],
        safety_state=values[2],
        arm_mask=values[3],
        request_sequence=values[4],
        sender_time_ms_echo=values[5],
        arbiter_epoch=values[6],
        horizon_end_tick=values[7],
        validated_tail_tick=values[8],
        execution_queue_samples=values[9],
        accepted_samples=values[10],
        applied_samples=values[11],
    )


def parse_executor_diagnostics_v2(payload: bytes) -> StreamExecutorDiagnosticsV2:
    if len(payload) != EXECUTOR_DIAGNOSTICS_V2.size:
        raise StreamContractError("invalid protocol v2 executor diagnostics length")
    values = EXECUTOR_DIAGNOSTICS_V2.unpack(payload)
    try:
        state = StreamExecutorStateV2(values[0])
        terminal_reason = StreamTerminalReasonV2(values[1])
    except ValueError as error:
        raise StreamContractError("invalid protocol v2 executor state") from error
    return StreamExecutorDiagnosticsV2(
        state=state,
        terminal_reason=terminal_reason,
        safe_stop_required=values[2] != 0,
        tracking_error_joint=values[3],
        request_sequence=values[4],
        sender_time_ms_echo=values[5],
        arbiter_epoch=values[6],
        horizon_end_tick=values[7],
        validated_tail_tick=values[8],
        queued_samples=values[9],
        peak_queued_samples=values[10],
        accepted_samples=values[11],
        applied_samples=values[12],
        control_outputs=values[13],
        splice_count=values[14],
        maximum_apply_lateness_ms=values[15],
        last_command_tick=values[16],
        terminal_tick=values[17],
    )


def parse_shadow_snapshot_v2(payload: bytes) -> StreamShadowSnapshotV2:
    if len(payload) == SHADOW_SNAPSHOT_V2.size:
        values = SHADOW_SNAPSHOT_V2.unpack(payload)
        positions_raw = tuple(values[4:16])
        unwrapped_positions_raw: tuple[int, ...] = ()
        anchor_positions_urad = tuple(values[16:28])
    elif len(payload) == UNWRAPPED_SHADOW_SNAPSHOT_V2.size:
        values = UNWRAPPED_SHADOW_SNAPSHOT_V2.unpack(payload)
        positions_raw = tuple(values[4:16])
        unwrapped_positions_raw = tuple(values[16:28])
        anchor_positions_urad = tuple(values[28:40])
    else:
        raise StreamContractError("invalid protocol v2 shadow snapshot length")
    if values[1] != JOINT_COUNT:
        raise StreamContractError("invalid protocol v2 shadow joint count")
    return StreamShadowSnapshotV2(
        status_code=values[0],
        joint_count=values[1],
        left_present_mask=values[2],
        right_present_mask=values[3],
        positions_raw=positions_raw,
        anchor_positions_urad=anchor_positions_urad,
        unwrapped_positions_raw=unwrapped_positions_raw,
    )


def parse_dispatch_diagnostics_v2(
    payload: bytes,
) -> BimanualDispatchDiagnosticsV2:
    if len(payload) != DISPATCH_DIAGNOSTICS_V2.size:
        raise StreamContractError("invalid bimanual dispatch diagnostics length")
    values = DISPATCH_DIAGNOSTICS_V2.unpack(payload)
    return BimanualDispatchDiagnosticsV2(
        status_code=values[0],
        active=values[1] != 0,
        faulted=values[2] != 0,
        ready=values[3] != 0,
        request_sequence=values[4],
        sender_time_ms_echo=values[5],
        launch_count=values[6],
        completed_count=values[7],
        failure_count=values[8],
        maximum_start_skew_us=values[9],
        maximum_launch_lateness_us=values[10],
        last_control_tick_ms=values[11],
        last_left_start_us=values[12],
        last_right_start_us=values[13],
    )


def parse_tracking_diagnostics_v2(
    payload: bytes,
) -> BimanualTrackingDiagnosticsV2:
    if len(payload) != TRACKING_DIAGNOSTICS_V2.size:
        raise StreamContractError("invalid bimanual tracking diagnostics length")
    values = TRACKING_DIAGNOSTICS_V2.unpack(payload)
    return BimanualTrackingDiagnosticsV2(
        status_code=values[0],
        active=values[1] != 0,
        pending=values[2] != 0,
        next_joint=values[3],
        request_sequence=values[4],
        sender_time_ms_echo=values[5],
        requested_pairs=values[6],
        completed_pairs=values[7],
        failed_pairs=values[8],
        maximum_reply_latency_ms=values[9],
        maximum_tracking_error_urad=tuple(values[10:22]),
    )



def parse_feedback_snapshot_v2(payload: bytes) -> BimanualFeedbackSnapshotV2:
    if len(payload) != FEEDBACK_SNAPSHOT_V2.size:
        raise StreamContractError("invalid bimanual feedback snapshot length")
    values = FEEDBACK_SNAPSHOT_V2.unpack(payload)
    if values[1] != JOINT_COUNT:
        raise StreamContractError("invalid bimanual feedback joint count")
    return BimanualFeedbackSnapshotV2(
        status_code=values[0],
        joint_count=values[1],
        present_mask=values[2],
        request_sequence=values[3],
        sender_time_ms_echo=values[4],
        firmware_tick_ms=values[5],
        completed_pairs=values[6],
        positions_urad=tuple(values[7:19]),
        sample_age_ms=tuple(values[19:31]),
    )
