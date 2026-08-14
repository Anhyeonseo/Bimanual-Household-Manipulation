import struct

import pytest

from single_arm_bridge.protocol import (
    BufferedSetpointFlags,
    BufferedSetpointSample,
    ProtocolError,
    encode_buffered_setpoint_payload,
    parse_setpoint_status,
    SETPOINT_STATUS_F0_METRICS,
    SETPOINT_STATUS_F3_CONTROL_TICK_METRICS,
    SETPOINT_STATUS_H2_TELEMETRY,
    validate_buffered_setpoint_flags,
)


def test_candidate_batch_payload_matches_g474_wire_layout() -> None:
    payload = encode_buffered_setpoint_payload(
        1_000,
        (
            BufferedSetpointSample(10, (1, 2, 3, 4, 5, 6)),
            BufferedSetpointSample(20, (-1, -2, -3, -4, -5, -6)),
        ),
    )
    assert len(payload) == 8 + (2 * 52)
    assert struct.unpack_from("<IBBH", payload) == (1_000, 2, 1, 0)
    assert struct.unpack_from("<I12i", payload, 8) == (
        10, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0,
    )


@pytest.mark.parametrize(
    "samples",
    [
        (),
        tuple(BufferedSetpointSample(i + 1, (0,) * 6) for i in range(10)),
        (BufferedSetpointSample(10, (0,) * 6), BufferedSetpointSample(10, (0,) * 6)),
        (BufferedSetpointSample(0, (0,) * 6), BufferedSetpointSample(0x80000000, (0,) * 6)),
        (BufferedSetpointSample(10, (0,) * 5),),
        (BufferedSetpointSample(10, (False, 0, 0, 0, 0, 0)),),
    ],
)
def test_candidate_batch_payload_rejects_invalid_batches(samples) -> None:
    with pytest.raises(ProtocolError):
        encode_buffered_setpoint_payload(1_000, samples)


def test_candidate_flags_require_candidate_and_reject_unknown_bits() -> None:
    flags = validate_buffered_setpoint_flags(int(
        BufferedSetpointFlags.CANDIDATE | BufferedSetpointFlags.BEGIN
        | BufferedSetpointFlags.START | BufferedSetpointFlags.END
    ))
    assert BufferedSetpointFlags.START in flags
    with pytest.raises(ProtocolError):
        validate_buffered_setpoint_flags(int(BufferedSetpointFlags.BEGIN))
    with pytest.raises(ProtocolError):
        validate_buffered_setpoint_flags(0x8002)


def test_legacy_setpoint_status_remains_supported() -> None:
    result = parse_setpoint_status(
        struct.pack("<BBBBIII", 0, 1, 3, 0, 42, 1_500, 0x2D90167E)
    )
    assert result.request_sequence == 42
    assert result.executor_state is None
    assert result.applied_samples is None


def test_extended_setpoint_status_exposes_buffer_diagnostics() -> None:
    payload = struct.pack(
        "<BBBBIII" "BBBBHHII", 6, 2, 3, 9, 42, 1_500, 0x2D90167E,
        4, 3, 1, 7, 5, 8, 12, 10,
    )
    result = parse_setpoint_status(payload)
    assert result.executor_state == 4
    assert result.terminal_reason == 3
    assert result.safe_stop_required is True
    assert result.queue_result == 7
    assert result.queued_samples == 5
    assert result.peak_queued_samples == 8
    assert result.accepted_samples == 12
    assert result.applied_samples == 10


def test_f0_terminal_status_exposes_microsecond_timing_metrics() -> None:
    payload = struct.pack(
        "<BBBBIII" "BBBBHHII" "7I" "4I",
        6, 2, 3, 9, 42, 1_500, 0x2D90167E,
        4, 3, 1, 7, 5, 8, 12, 10,
        2, 1, 0, 0, 0, 0, 3,
        5_200, 4_980, 310, 4_720,
    )
    assert len(payload) == 60 + SETPOINT_STATUS_F0_METRICS.size
    result = parse_setpoint_status(payload)
    assert result.apply_lateness_histogram == (2, 1, 0, 0, 0, 0)
    assert result.maximum_apply_lateness_sample_index == 3
    assert result.f0_loop_period_max_us == 5_200
    assert result.f0_loop_work_max_us == 4_980
    assert result.f0_servo_sync_write_max_us == 310
    assert result.f0_host_tx_max_us == 4_720


def test_h2_terminal_status_exposes_position_only_in_motion_telemetry() -> None:
    payload = struct.pack(
        "<BBBBIII" "BBBBHHII" "7I" "4I" "6H4I",
        6, 2, 3, 9, 42, 1_500, 0x2D90167E,
        4, 3, 1, 7, 5, 8, 12, 10,
        2, 1, 0, 0, 0, 0, 3,
        5_200, 4_980, 310, 4_720,
        1, 2, 3, 4, 5, 6, 30, 29, 1, 2,
    )
    assert len(payload) == 76 + SETPOINT_STATUS_H2_TELEMETRY.size
    result = parse_setpoint_status(payload)
    assert result.h2_tracking_error_max_raw == (1, 2, 3, 4, 5, 6)
    assert result.h2_telemetry_requested_samples == 30
    assert result.h2_telemetry_completed_samples == 29
    assert result.h2_telemetry_failed_samples == 1
    assert result.h2_telemetry_maximum_reply_latency_ms == 2


def test_f3_terminal_status_exposes_observation_only_control_tick_metrics() -> None:
    payload = struct.pack(
        "<BBBBIII" "BBBBHHII" "7I" "4I" "6H4I" "4I",
        6, 2, 3, 9, 42, 1_500, 0x2D90167E,
        4, 3, 1, 7, 5, 8, 12, 10,
        2, 1, 0, 0, 0, 0, 3,
        5_200, 4_980, 310, 4_720,
        1, 2, 3, 4, 5, 6, 30, 29, 1, 2,
        5_007, 7, 3, 2_400,
    )
    assert len(payload) == (
        104 + SETPOINT_STATUS_F3_CONTROL_TICK_METRICS.size
    )
    result = parse_setpoint_status(payload)
    assert result.f3_control_tick_period_max_us == 5_007
    assert result.f3_control_tick_jitter_max_us == 7
    assert result.f3_control_tick_work_max_us == 3
    assert result.f3_control_tick_count == 2_400


@pytest.mark.parametrize("length", [0, 15, 17, 31, 33, 77, 103, 105])
def test_setpoint_status_rejects_unknown_lengths(length: int) -> None:
    with pytest.raises(ProtocolError):
        parse_setpoint_status(bytes(length))
