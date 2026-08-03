"""Pure helpers for no-motion buffered validation timing capture."""

from __future__ import annotations

import math
from typing import Any, Sequence


UINT32_MASK = 0xFFFFFFFF
MINIMUM_CAPTURE_SAMPLES = 1_000


def estimate_mcu_tick_ms(
    sync_mcu_tick_ms: int,
    sync_host_raw_ns: int,
    host_raw_ns: int,
) -> int:
    """Project a synchronized MCU tick using CLOCK_MONOTONIC_RAW elapsed time."""

    if not 0 <= sync_mcu_tick_ms <= UINT32_MASK:
        raise ValueError("sync MCU tick must be uint32")
    if sync_host_raw_ns < 0 or host_raw_ns < sync_host_raw_ns:
        raise ValueError("host raw clock must be monotonic")
    elapsed_ms = (host_raw_ns - sync_host_raw_ns) // 1_000_000
    return (sync_mcu_tick_ms + elapsed_ms) & UINT32_MASK


def elapsed_ms(start_raw_ns: int, end_raw_ns: int) -> float:
    if start_raw_ns < 0 or end_raw_ns < start_raw_ns:
        raise ValueError("raw clock interval is invalid")
    return (end_raw_ns - start_raw_ns) / 1_000_000.0


def nonnegative_lateness_ms(
    expected_raw_ns: int,
    actual_raw_ns: int,
) -> float:
    if expected_raw_ns < 0 or actual_raw_ns < 0:
        raise ValueError("raw clock values must be nonnegative")
    return max(0.0, (actual_raw_ns - expected_raw_ns) / 1_000_000.0)


def _validated_series(values: Sequence[float], name: str) -> list[float]:
    result: list[float] = []
    for raw in values:
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must contain finite nonnegative samples")
        result.append(value)
    return result


def build_capture_document(
    *,
    firmware_version: int,
    calibration_hash: int,
    capabilities: int,
    requested_samples: int,
    interval_ms: int,
    lead_ms: int,
    sample_spacing_ms: int,
    serial_round_trip_ms: Sequence[float],
    host_command_jitter_ms: Sequence[float],
    delivery_lateness_ms: Sequence[float],
    host_outage_ms: Sequence[float],
    transport_error_count: int,
) -> dict[str, Any]:
    """Build the analyzer input without authorizing operational timing values."""

    if requested_samples < MINIMUM_CAPTURE_SAMPLES:
        raise ValueError("hardware capture requires at least 1000 samples")
    if interval_ms <= 0 or lead_ms <= 0 or sample_spacing_ms <= 0:
        raise ValueError("capture timing values must be positive")
    if transport_error_count < 0:
        raise ValueError("transport error count must be nonnegative")
    round_trip = _validated_series(
        serial_round_trip_ms, "serial_round_trip_ms"
    )
    jitter = _validated_series(
        host_command_jitter_ms, "host_command_jitter_ms"
    )
    lateness = _validated_series(
        delivery_lateness_ms, "delivery_lateness_ms"
    )
    outages = _validated_series(host_outage_ms, "host_outage_ms")
    if not (len(round_trip) == len(jitter) == len(lateness)):
        raise ValueError("measurement series lengths must match")
    return {
        "schema_version": 1,
        "capture_kind": "single_arm_buffered_timing_capture",
        "source": {
            "provenance": "pi_vcp_hardware",
            "firmware_buffered_capability": bool(capabilities & 0x00000400),
            "clock_source": "monotonic_raw",
            "transport_error_count": transport_error_count,
            "firmware_version": f"0x{firmware_version:08X}",
            "calibration_hash": f"0x{calibration_hash:08X}",
            "capabilities": f"0x{capabilities:08X}",
            "validation_only": True,
            "motion_authorized": False,
        },
        "capture_parameters": {
            "requested_samples": requested_samples,
            "captured_samples": len(round_trip),
            "interval_ms": interval_ms,
            "lead_ms": lead_ms,
            "sample_spacing_ms": sample_spacing_ms,
        },
        "measurements": {
            "serial_round_trip_ms": round_trip,
            "host_command_jitter_ms": jitter,
            "delivery_lateness_ms": lateness,
        },
        "fault_injection": {"host_outage_ms": outages},
    }
