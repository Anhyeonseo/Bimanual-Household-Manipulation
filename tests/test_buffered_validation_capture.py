import pytest

from single_arm_bridge.buffered_validation_capture import (
    build_capture_document,
    elapsed_ms,
    estimate_mcu_tick_ms,
    nonnegative_lateness_ms,
)


def test_mcu_tick_projection_handles_uint32_wrap() -> None:
    assert estimate_mcu_tick_ms(0xFFFFFFFE, 1_000_000, 4_000_000) == 1
    with pytest.raises(ValueError, match="monotonic"):
        estimate_mcu_tick_ms(0, 4, 3)


def test_raw_clock_metrics_are_nonnegative() -> None:
    assert elapsed_ms(1_000_000, 3_500_000) == 2.5
    assert nonnegative_lateness_ms(5_000_000, 4_000_000) == 0.0
    assert nonnegative_lateness_ms(5_000_000, 6_500_000) == 1.5


def test_capture_document_preserves_no_motion_provenance() -> None:
    document = build_capture_document(
        firmware_version=0x00021900,
        calibration_hash=0x8AD27897,
        capabilities=0x000007FF,
        requested_samples=1_000,
        interval_ms=20,
        lead_ms=100,
        sample_spacing_ms=20,
        serial_round_trip_ms=[2.0] * 1_000,
        host_command_jitter_ms=[0.1] * 1_000,
        delivery_lateness_ms=[0.0] * 1_000,
        host_outage_ms=[20.0, 40.0, 80.0],
        transport_error_count=0,
    )
    assert document["source"]["provenance"] == "pi_vcp_hardware"
    assert document["source"]["firmware_buffered_capability"] is True
    assert document["source"]["validation_only"] is True
    assert document["source"]["motion_authorized"] is False
    assert document["capture_parameters"]["captured_samples"] == 1_000


def test_capture_document_rejects_short_or_mismatched_series() -> None:
    kwargs = dict(
        firmware_version=0x00021900,
        calibration_hash=0x8AD27897,
        capabilities=0x000007FF,
        requested_samples=1_000,
        interval_ms=20,
        lead_ms=100,
        sample_spacing_ms=20,
        serial_round_trip_ms=[2.0],
        host_command_jitter_ms=[0.1],
        delivery_lateness_ms=[0.0],
        host_outage_ms=[20.0],
        transport_error_count=0,
    )
    with pytest.raises(ValueError, match="series lengths"):
        build_capture_document(
            **{**kwargs, "delivery_lateness_ms": []},
        )
    with pytest.raises(ValueError, match="at least 1000"):
        build_capture_document(
            **{**kwargs, "requested_samples": 999},
        )
