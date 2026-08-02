"""Host-only buffered timing analysis with fail-closed deployment semantics."""

from __future__ import annotations

import math
from typing import Any, Iterable

SCHEMA_VERSION = 1
CAPTURE_KIND = "single_arm_buffered_timing_capture"
REQUIRED_SERIES = (
    "serial_round_trip_ms", "host_command_jitter_ms", "delivery_lateness_ms",
)
DEPLOYMENT_FIELDS = (
    "minimum_lead_ms", "maximum_lead_ms", "startup_prime_depth_samples",
    "low_watermark_samples", "refill_target_samples",
)
MINIMUM_HARDWARE_SAMPLES = 1_000


class BufferedTimingError(ValueError):
    pass


def _finite_nonnegative(values: Iterable[Any], name: str) -> list[float]:
    result: list[float] = []
    for raw in values:
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise BufferedTimingError(f"{name} contains a non-numeric sample")
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            raise BufferedTimingError(f"{name} samples must be finite and nonnegative")
        result.append(value)
    if not result:
        raise BufferedTimingError(f"{name} must not be empty")
    return result


def _nearest_rank(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    rank = max(1, math.ceil((percentile / 100.0) * len(ordered)))
    return ordered[rank - 1]


def summarize_samples(values: Iterable[Any], name: str) -> dict[str, float | int]:
    samples = _finite_nonnegative(values, name)
    return {
        "count": len(samples), "minimum_ms": min(samples),
        "p50_ms": _nearest_rank(samples, 50.0),
        "p95_ms": _nearest_rank(samples, 95.0),
        "p99_ms": _nearest_rank(samples, 99.0), "maximum_ms": max(samples),
    }


def analyze_buffered_timing_capture(capture: dict[str, Any]) -> dict[str, Any]:
    if capture.get("schema_version") != SCHEMA_VERSION:
        raise BufferedTimingError("unsupported buffered timing schema")
    if capture.get("capture_kind") != CAPTURE_KIND:
        raise BufferedTimingError("unexpected buffered timing capture kind")
    measurements = capture.get("measurements")
    if not isinstance(measurements, dict):
        raise BufferedTimingError("measurements must be an object")
    summaries = {
        name: summarize_samples(measurements.get(name, ()), name)
        for name in REQUIRED_SERIES
    }
    fault_outages = summarize_samples(
        capture.get("fault_injection", {}).get("host_outage_ms", ()),
        "host_outage_ms",
    )
    source = capture.get("source", {})
    gates = (
        source.get("provenance") == "pi_vcp_hardware",
        source.get("firmware_buffered_capability") is True,
        source.get("clock_source") == "monotonic_raw",
        source.get("transport_error_count") == 0,
        all(summary["count"] >= MINIMUM_HARDWARE_SAMPLES
            for summary in summaries.values()),
    )
    authorized = all(gates)
    reasons: list[str] = []
    labels = (
        "pi_vcp_hardware_provenance_missing",
        "buffered_firmware_capability_not_advertised",
        "monotonic_raw_clock_not_confirmed",
        "transport_error_free_capture_not_confirmed",
        "minimum_1000_samples_per_series_not_met",
    )
    for passed, label in zip(gates, labels, strict=True):
        if not passed:
            reasons.append(label)
    observed_floor = sum(
        summaries[name]["p95_ms"] for name in REQUIRED_SERIES
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_kind": "single_arm_buffered_timing_analysis",
        "status": "MEASURED_DEPLOYMENT_INPUT" if authorized
        else "HOST_ONLY_NOT_DEPLOYABLE",
        "motion_authorized": False,
        "measurement_input_authorized": authorized,
        "operational_values_authorized": False,
        "authorization_failures": reasons,
        "summaries": summaries,
        "fault_injection": {"host_outage_ms": fault_outages},
        "observed_p95_delivery_floor_ms": observed_floor,
        "deployment_values": {field: None for field in DEPLOYMENT_FIELDS},
        "note": (
            "Observed statistics are evidence only. Deployment values require a "
            "separate reviewed derivation even when authorization gates pass."
        ),
    }
