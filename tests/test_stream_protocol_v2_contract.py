from dataclasses import fields, replace
import struct

import pytest

from single_arm_bridge.stream_protocol_v2 import (
    ARM_MASK_BOTH,
    ARM_MASK_LEFT,
    ARM_MASK_RIGHT,
    BATCH_HEADER,
    BATCH_SAMPLE,
    DISPATCH_DIAGNOSTICS_V2,
    EXECUTOR_DIAGNOSTICS_V2,
    HELLO_V2,
    MAX_BATCH_PAYLOAD_SIZE,
    MAX_SAMPLES,
    SHADOW_SNAPSHOT_V2,
    UNWRAPPED_SHADOW_SNAPSHOT_V2,
    UNWRAP_SHADOW_PREPARE_V2,
    STREAM_OPEN,
    STREAM_STATUS_V2,
    TRACKING_DIAGNOSTICS_V2,
    BatchKindV2,
    FrameV2,
    StreamBatchV2,
    StreamContractError,
    StreamExecutorStateV2,
    StreamHardCapsV2,
    StreamMessageTypeV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamTerminalReasonV2,
    decode_frame_v2,
    encode_frame_v2,
    encode_stream_batch_v2,
    encode_stream_open_v2,
    encode_unwrap_shadow_prepare_v2,
    parse_hello_v2,
    parse_dispatch_diagnostics_v2,
    parse_executor_diagnostics_v2,
    parse_shadow_snapshot_v2,
    parse_stream_status_v2,
    parse_tracking_diagnostics_v2,
    validate_stream_policy_v2,
)


def hard_caps() -> StreamHardCapsV2:
    return StreamHardCapsV2(
        minimum_lead_ms=20,
        maximum_lead_ms=400,
        maximum_command_timeout_ms=500,
        maximum_open_command_timeout_ms=100,
        maximum_apply_lateness_ms=5,
        tracking_error_limit_urad=(100_000,) * 12,
        maximum_step_urad_per_tick=(10_000,) * 12,
    )


def finite_policy() -> StreamPolicyV2:
    return StreamPolicyV2(
        minimum_start_samples=2,
        minimum_lead_ms=20,
        horizon_end_tick=300,
        maximum_lead_ms=400,
        command_timeout_ms=500,
        maximum_apply_lateness_ms=5,
        tracking_error_limit_urad=(90_000,) * 12,
        maximum_step_urad_per_tick=(9_000,) * 12,
    )


def samples(count: int = 2) -> tuple[StreamSampleV2, ...]:
    return tuple(
        StreamSampleV2(140 + index * 20, (100 + index,) * 12)
        for index in range(count)
    )


def test_v2_contract_has_no_source_mode_or_residual_slot() -> None:
    policy_names = {field.name for field in fields(StreamPolicyV2)}
    batch_names = {field.name for field in fields(StreamBatchV2)}
    all_names = policy_names | batch_names
    assert not any(
        name in {"track", "source", "mode", "residual"}
        or name.startswith(("track_", "source_", "mode_", "residual_"))
        for name in all_names
    )
    assert {"horizon_end_tick", "arbiter_epoch", "splice_at_tick"} <= all_names


def test_stream_message_ids_fit_the_existing_motion_range() -> None:
    assert StreamMessageTypeV2.SETPOINT_BATCH == 32
    assert StreamMessageTypeV2.SETPOINT_STATUS == 33
    assert StreamMessageTypeV2.STREAM_OPEN == 40
    assert StreamMessageTypeV2.STREAM_STATUS == 41
    assert StreamMessageTypeV2.SPLICE == 42
    assert StreamMessageTypeV2.GET_EXECUTOR_DIAGNOSTICS == 43
    assert StreamMessageTypeV2.EXECUTOR_DIAGNOSTICS == 44
    assert StreamMessageTypeV2.PREPARE_SHADOW == 45
    assert StreamMessageTypeV2.SHADOW_SNAPSHOT == 46
    assert StreamMessageTypeV2.GET_DISPATCH_DIAGNOSTICS == 47
    assert StreamMessageTypeV2.DISPATCH_DIAGNOSTICS == 58


def test_shadow_snapshot_wire_layout_is_exact() -> None:
    payload = SHADOW_SNAPSHOT_V2.pack(
        0,
        12,
        0x3F,
        0x3F,
        *range(2048, 2060),
        *range(-6, 6),
    )
    snapshot = parse_shadow_snapshot_v2(payload)
    assert len(payload) == 76
    assert snapshot.status_code == 0
    assert snapshot.positions_raw == tuple(range(2048, 2060))
    assert snapshot.anchor_positions_urad == tuple(range(-6, 6))
    assert snapshot.unwrapped_positions_raw == ()


def test_unwrapped_shadow_wire_layout_is_exact() -> None:
    prepare = encode_unwrap_shadow_prepare_v2(
        tuple(range(2048, 2060)), 512
    )
    assert len(prepare) == UNWRAP_SHADOW_PREPARE_V2.size == 52
    assert UNWRAP_SHADOW_PREPARE_V2.unpack(prepare) == (
        512,
        0,
        *range(2048, 2060),
    )
    payload = UNWRAPPED_SHADOW_SNAPSHOT_V2.pack(
        0,
        12,
        0x3F,
        0x3F,
        *range(10, 22),
        *range(4090, 4102),
        *range(-6, 6),
    )
    snapshot = parse_shadow_snapshot_v2(payload)
    assert len(payload) == 124
    assert snapshot.positions_raw == tuple(range(10, 22))
    assert snapshot.unwrapped_positions_raw == tuple(range(4090, 4102))
    assert snapshot.anchor_positions_urad == tuple(range(-6, 6))


@pytest.mark.parametrize("window", [0, 2048, 4096])
def test_unwrapped_shadow_rejects_ambiguous_reference_window(
    window: int,
) -> None:
    with pytest.raises(StreamContractError, match="1..2047"):
        encode_unwrap_shadow_prepare_v2((2048,) * 12, window)


@pytest.mark.parametrize("arm_mask", [ARM_MASK_LEFT, ARM_MASK_RIGHT, ARM_MASK_BOTH])
def test_stream_open_wire_layout_is_exact(arm_mask: int) -> None:
    policy = replace(finite_policy(), arm_mask=arm_mask)
    payload = encode_stream_open_v2(policy)
    assert len(payload) == STREAM_OPEN.size == 120
    unpacked = STREAM_OPEN.unpack(payload)
    assert unpacked[:8] == (2, arm_mask, 0, 20, 300, 400, 500, 5)
    assert unpacked[8:20] == (90_000,) * 12
    assert unpacked[20:32] == (9_000,) * 12


def test_nine_sample_batch_fits_the_existing_512_byte_payload() -> None:
    batch = StreamBatchV2(
        horizon_end_tick=400,
        arbiter_epoch=7,
        samples=samples(MAX_SAMPLES),
    )
    payload = encode_stream_batch_v2(batch, BatchKindV2.APPEND)
    assert len(payload) == MAX_BATCH_PAYLOAD_SIZE == 488
    assert len(payload) <= 512
    assert BATCH_HEADER.unpack_from(payload) == (
        140, 400, 7, 0, 9, ARM_MASK_BOTH, 0
    )
    assert BATCH_SAMPLE.unpack_from(payload, BATCH_HEADER.size) == (
        0, *((100,) * 12)
    )


def test_splice_uses_the_same_absolute_twelve_joint_sample() -> None:
    batch = StreamBatchV2(
        horizon_end_tick=400,
        arbiter_epoch=8,
        splice_at_tick=140,
        samples=samples(),
    )
    payload = encode_stream_batch_v2(batch, BatchKindV2.SPLICE)
    assert BATCH_HEADER.unpack_from(payload) == (
        140, 400, 8, 140, 2, ARM_MASK_BOTH, 0
    )
    assert struct.unpack_from("<12i", payload, BATCH_HEADER.size + 4) == (100,) * 12


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"minimum_lead_ms": 19}, "minimum_lead_ms"),
        ({"maximum_lead_ms": 401}, "maximum_lead_ms"),
        ({"command_timeout_ms": 501}, "command_timeout_ms"),
        ({"maximum_apply_lateness_ms": 6}, "maximum_apply_lateness_ms"),
        (
            {"tracking_error_limit_urad": (100_001,) + (90_000,) * 11},
            "tracking error",
        ),
        (
            {"maximum_step_urad_per_tick": (10_001,) + (9_000,) * 11},
            "maximum step",
        ),
    ],
)
def test_every_loosened_safety_field_is_rejected(
    change: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(StreamContractError, match=message):
        validate_stream_policy_v2(
            replace(finite_policy(), **change),
            hard_caps(),
            current_tick=100,
        )


def test_open_horizon_uses_100ms_timeout_cap() -> None:
    policy = replace(finite_policy(), horizon_end_tick=0, command_timeout_ms=100)
    validate_stream_policy_v2(policy, hard_caps(), current_tick=100)
    with pytest.raises(StreamContractError, match="open stream"):
        validate_stream_policy_v2(
            replace(policy, command_timeout_ms=101),
            hard_caps(),
            current_tick=100,
        )


def test_append_and_splice_fields_are_mutually_exclusive() -> None:
    with pytest.raises(StreamContractError, match="append"):
        encode_stream_batch_v2(
            StreamBatchV2(
                horizon_end_tick=400,
                arbiter_epoch=7,
                splice_at_tick=140,
                samples=samples(),
            ),
            BatchKindV2.APPEND,
        )
    with pytest.raises(StreamContractError, match="splice requires"):
        encode_stream_batch_v2(
            StreamBatchV2(
                horizon_end_tick=400,
                arbiter_epoch=8,
                samples=samples(),
            ),
            BatchKindV2.SPLICE,
        )


def test_samples_require_all_twelve_absolute_joint_targets() -> None:
    with pytest.raises(StreamContractError, match="12 joints"):
        encode_stream_batch_v2(
            StreamBatchV2(
                horizon_end_tick=400,
                arbiter_epoch=7,
                samples=(StreamSampleV2(140, (0,) * 11),),
            ),
            BatchKindV2.APPEND,
        )


def test_protocol_v2_frame_round_trip_and_identity_payload() -> None:
    hello_payload = HELLO_V2.pack(
        2,
        12,
        0,
        0,
        0x00023D00,
        0x2D90167E,
        0x2D90167E,
        0x01802000,
        0,
    )
    encoded = encode_frame_v2(
        FrameV2(
            StreamMessageTypeV2.HELLO_RESPONSE,
            sequence=7,
            sender_time_ms=123,
            payload=hello_payload,
        )
    )
    decoded = decode_frame_v2(encoded)
    assert decoded.sequence == 7
    hello = parse_hello_v2(decoded.payload)
    assert hello.protocol_version == 2
    assert hello.joint_count == 12
    assert hello.left_calibration_hash == hello.right_calibration_hash
    assert hello.capabilities == 0x01802000


def test_stream_status_freezes_no_motion_counters_and_echoes() -> None:
    payload = STREAM_STATUS_V2.pack(
        5,
        0,
        1,
        ARM_MASK_BOTH,
        42,
        1234,
        8,
        2000,
        1290,
        0,
        0,
        0,
    )
    status = parse_stream_status_v2(payload)
    assert status.request_sequence == 42
    assert status.sender_time_ms_echo == 1234
    assert status.arbiter_epoch == 8
    assert status.execution_queue_samples == 0
    assert status.accepted_samples == 0
    assert status.applied_samples == 0


def test_executor_diagnostics_wire_layout_and_enums_are_exact() -> None:
    payload = EXECUTOR_DIAGNOSTICS_V2.pack(
        StreamExecutorStateV2.SUCCEEDED,
        StreamTerminalReasonV2.PLANNED_HORIZON,
        0,
        11,
        42,
        1234,
        8,
        2000,
        0,
        0,
        9,
        5,
        3,
        25,
        1,
        0,
        1900,
        2000,
    )
    diagnostics = parse_executor_diagnostics_v2(payload)
    assert len(payload) == 60
    assert diagnostics.state is StreamExecutorStateV2.SUCCEEDED
    assert diagnostics.terminal_reason is StreamTerminalReasonV2.PLANNED_HORIZON
    assert diagnostics.request_sequence == 42
    assert diagnostics.accepted_samples == 5
    assert diagnostics.applied_samples == 3
    assert diagnostics.splice_count == 1


def test_bimanual_dispatch_diagnostics_wire_layout_is_exact() -> None:
    payload = DISPATCH_DIAGNOSTICS_V2.pack(
        0, 0, 0, 1, 42, 1234, 9, 9, 0, 3, 17, 900, 1000, 1003
    )
    diagnostics = parse_dispatch_diagnostics_v2(payload)
    assert len(payload) == 44
    assert diagnostics.ready
    assert not diagnostics.active
    assert not diagnostics.faulted
    assert diagnostics.request_sequence == 42
    assert diagnostics.sender_time_ms_echo == 1234
    assert diagnostics.launch_count == diagnostics.completed_count == 9
    assert diagnostics.failure_count == 0
    assert diagnostics.maximum_start_skew_us == 3
    assert diagnostics.maximum_launch_lateness_us == 17


def test_bimanual_tracking_diagnostics_wire_layout_is_exact() -> None:
    errors = tuple(index * 1000 for index in range(12))
    payload = TRACKING_DIAGNOSTICS_V2.pack(
        0, 1, 0, 4, 42, 1234, 36, 36, 0, 3, *errors
    )
    diagnostics = parse_tracking_diagnostics_v2(payload)
    assert len(payload) == 76
    assert diagnostics.active
    assert not diagnostics.pending
    assert diagnostics.next_joint == 4
    assert diagnostics.request_sequence == 42
    assert diagnostics.sender_time_ms_echo == 1234
    assert diagnostics.requested_pairs == diagnostics.completed_pairs == 36
    assert diagnostics.failed_pairs == 0
    assert diagnostics.maximum_reply_latency_ms == 3
    assert diagnostics.maximum_tracking_error_urad == errors
