import struct

import pytest

from single_arm_bridge.protocol import (
    BufferedSetpointFlags,
    BufferedSetpointSample,
    ProtocolError,
    encode_buffered_setpoint_payload,
    parse_setpoint_status,
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


@pytest.mark.parametrize("length", [0, 15, 17, 31, 33])
def test_setpoint_status_rejects_unknown_lengths(length: int) -> None:
    with pytest.raises(ProtocolError):
        parse_setpoint_status(bytes(length))
