import pytest

from single_arm_bridge.buffered_timing import (
    BufferedTimingError,
    analyze_buffered_timing_capture,
)


def capture(*, count: int = 10, provenance: str = "synthetic_fault_injection"):
    return {
        "schema_version": 1,
        "capture_kind": "single_arm_buffered_timing_capture",
        "source": {
            "provenance": provenance,
            "firmware_buffered_capability": False,
            "clock_source": "simulated_monotonic",
            "transport_error_count": 0,
        },
        "measurements": {
            "serial_round_trip_ms": [4.0 + (i % 3) for i in range(count)],
            "host_command_jitter_ms": [1.0 + (i % 2) for i in range(count)],
            "delivery_lateness_ms": [2.0 + (i % 4) for i in range(count)],
        },
        "fault_injection": {"host_outage_ms": [20.0, 40.0, 80.0]},
    }


def test_synthetic_capture_never_authorizes_operational_values() -> None:
    analysis = analyze_buffered_timing_capture(capture())
    assert analysis["status"] == "HOST_ONLY_NOT_DEPLOYABLE"
    assert analysis["motion_authorized"] is False
    assert analysis["measurement_input_authorized"] is False
    assert analysis["operational_values_authorized"] is False
    assert all(value is None for value in analysis["deployment_values"].values())
    assert analysis["observed_p95_delivery_floor_ms"] == 13.0


def test_hardware_label_alone_cannot_bypass_all_gates() -> None:
    value = capture(count=1_000, provenance="pi_vcp_hardware")
    value["source"]["clock_source"] = "monotonic_raw"
    analysis = analyze_buffered_timing_capture(value)
    assert analysis["measurement_input_authorized"] is False
    assert analysis["operational_values_authorized"] is False
    assert "buffered_firmware_capability_not_advertised" in analysis["authorization_failures"]


def test_complete_future_measurement_marks_input_but_does_not_derive_values() -> None:
    value = capture(count=1_000, provenance="pi_vcp_hardware")
    value["source"].update(
        firmware_buffered_capability=True, clock_source="monotonic_raw",
    )
    analysis = analyze_buffered_timing_capture(value)
    assert analysis["status"] == "MEASURED_DEPLOYMENT_INPUT"
    assert analysis["measurement_input_authorized"] is True
    assert analysis["operational_values_authorized"] is False
    assert all(item is None for item in analysis["deployment_values"].values())


@pytest.mark.parametrize(
    "mutation",
    [
        lambda value: value.update(schema_version=2),
        lambda value: value["measurements"].update(serial_round_trip_ms=[]),
        lambda value: value["measurements"].update(host_command_jitter_ms=[-1]),
        lambda value: value["measurements"].update(delivery_lateness_ms=[True]),
    ],
)
def test_invalid_capture_fails_closed(mutation) -> None:
    value = capture()
    mutation(value)
    with pytest.raises(BufferedTimingError):
        analyze_buffered_timing_capture(value)
