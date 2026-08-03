"""Host-only buffered timing analysis with fail-closed deployment semantics."""

from __future__ import annotations

import math
from typing import Any, Iterable, Sequence

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
POLICY_KIND = "single_arm_buffered_timing_policy"
POLICY_STATUS = "REVIEWED_DEPLOYMENT_INPUT"


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


def derive_buffered_timing_policy(
    captures: Sequence[dict[str, Any]],
    *,
    rejected_first_lead_ms: int,
    rejected_status_code: int,
    rejected_detail: int,
    sample_period_ms: int = 20,
    maximum_batch_samples: int = 9,
    queue_capacity_samples: int = 16,
) -> dict[str, Any]:
    """Derive reviewed no-motion deployment input from Pi-VCP evidence."""
    integer_inputs = (
        rejected_first_lead_ms,
        rejected_status_code,
        rejected_detail,
        sample_period_ms,
        maximum_batch_samples,
        queue_capacity_samples,
    )
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in integer_inputs
    ):
        raise BufferedTimingError("policy inputs must be nonnegative integers")
    if sample_period_ms == 0 or maximum_batch_samples == 0:
        raise BufferedTimingError("period and batch size must be positive")
    if maximum_batch_samples > queue_capacity_samples:
        raise BufferedTimingError("batch size exceeds queue capacity")
    if len(captures) < 4:
        raise BufferedTimingError("at least four successful captures are required")

    analyses: list[dict[str, Any]] = []
    successful_first_leads: list[int] = []
    observed_horizons: list[int] = []
    outage_maxima: list[float] = []
    firmware_versions: set[str] = set()
    calibration_hashes: set[str] = set()
    capabilities: set[str] = set()

    for capture in captures:
        analysis = analyze_buffered_timing_capture(capture)
        if not analysis["measurement_input_authorized"]:
            raise BufferedTimingError("all policy captures must pass hardware gates")
        parameters = capture.get("capture_parameters", {})
        lead_ms = parameters.get("lead_ms")
        spacing_ms = parameters.get("sample_spacing_ms")
        interval_ms = parameters.get("interval_ms")
        if (
            isinstance(lead_ms, bool)
            or not isinstance(lead_ms, int)
            or isinstance(spacing_ms, bool)
            or not isinstance(spacing_ms, int)
            or interval_ms != sample_period_ms
            or spacing_ms != sample_period_ms
        ):
            raise BufferedTimingError(
                "captures must use the reviewed sample period and spacing"
            )
        source = capture.get("source", {})
        if source.get("validation_only") is not True:
            raise BufferedTimingError("capture is not validation-only")
        if source.get("motion_authorized") is not False:
            raise BufferedTimingError("capture unexpectedly authorizes motion")
        successful_first_leads.append(lead_ms)
        observed_horizons.append(lead_ms + spacing_ms)
        outage_maxima.append(
            float(analysis["fault_injection"]["host_outage_ms"]["maximum_ms"])
        )
        firmware_versions.add(str(source.get("firmware_version")))
        calibration_hashes.add(str(source.get("calibration_hash")))
        capabilities.add(str(source.get("capabilities")))
        analyses.append(analysis)

    if len(firmware_versions) != 1 or len(calibration_hashes) != 1:
        raise BufferedTimingError("capture hardware identity is inconsistent")
    if len(capabilities) != 1:
        raise BufferedTimingError("capture capability set is inconsistent")

    ordered_leads = sorted(successful_first_leads)
    if len(set(ordered_leads)) != len(ordered_leads):
        raise BufferedTimingError("successful lead captures must be unique")
    minimum_lead_ms = ordered_leads[0]
    maximum_lead_ms = max(observed_horizons)
    if rejected_first_lead_ms + sample_period_ms != minimum_lead_ms:
        raise BufferedTimingError(
            "rejected boundary must be exactly one sample below minimum lead"
        )
    if rejected_status_code != 1 or rejected_detail != 9:
        raise BufferedTimingError(
            "lower boundary must be a fail-closed queue rejection"
        )
    if not {60, 80, 100}.issubset(set(ordered_leads)):
        raise BufferedTimingError("60/80/100 ms successful lead evidence is required")
    if maximum_lead_ms != 400:
        raise BufferedTimingError("reviewed maximum horizon must be 400 ms")

    worst_rtt_p95 = max(
        float(item["summaries"]["serial_round_trip_ms"]["p95_ms"])
        for item in analyses
    )
    worst_rtt_p99 = max(
        float(item["summaries"]["serial_round_trip_ms"]["p99_ms"])
        for item in analyses
    )
    worst_jitter_p95 = max(
        float(item["summaries"]["host_command_jitter_ms"]["p95_ms"])
        for item in analyses
    )
    worst_jitter_p99 = max(
        float(item["summaries"]["host_command_jitter_ms"]["p99_ms"])
        for item in analyses
    )
    worst_lateness_p95 = max(
        float(item["summaries"]["delivery_lateness_ms"]["p95_ms"])
        for item in analyses
    )
    observed_max_outage_ms = max(outage_maxima)
    recovery_budget_ms = (
        observed_max_outage_ms + worst_rtt_p99 + sample_period_ms +
        minimum_lead_ms
    )
    recovery_consumption_samples = math.ceil(
        recovery_budget_ms / sample_period_ms
    )
    low_watermark_samples = recovery_consumption_samples + 1
    startup_prime_depth_samples = queue_capacity_samples
    refill_target_samples = min(
        queue_capacity_samples,
        low_watermark_samples + maximum_batch_samples,
    )

    if low_watermark_samples >= startup_prime_depth_samples:
        raise BufferedTimingError("startup prime does not exceed low watermark")
    if refill_target_samples != queue_capacity_samples:
        raise BufferedTimingError("reviewed refill must restore full queue depth")
    full_queue_horizon_ms = (
        minimum_lead_ms +
        (queue_capacity_samples - 1) * sample_period_ms
    )
    if full_queue_horizon_ms > maximum_lead_ms:
        raise BufferedTimingError("maximum lead cannot hold a full queue horizon")

    return {
        "schema_version": SCHEMA_VERSION,
        "policy_kind": POLICY_KIND,
        "status": POLICY_STATUS,
        "measurement_input_authorized": True,
        "operational_values_authorized": True,
        "motion_authorized": False,
        "deployment_values": {
            "sample_period_ms": sample_period_ms,
            "minimum_lead_ms": minimum_lead_ms,
            "maximum_lead_ms": maximum_lead_ms,
            "startup_prime_depth_samples": startup_prime_depth_samples,
            "low_watermark_samples": low_watermark_samples,
            "refill_target_samples": refill_target_samples,
        },
        "measurement_envelope": {
            "successful_first_leads_ms": ordered_leads,
            "serial_round_trip_p95_ms": worst_rtt_p95,
            "serial_round_trip_p99_ms": worst_rtt_p99,
            "host_command_jitter_p95_ms": worst_jitter_p95,
            "host_command_jitter_p99_ms": worst_jitter_p99,
            "delivery_lateness_p95_ms": worst_lateness_p95,
            "observed_max_host_outage_ms": observed_max_outage_ms,
        },
        "rejected_boundary": {
            "first_lead_ms": rejected_first_lead_ms,
            "status_code": rejected_status_code,
            "detail": rejected_detail,
            "meaning": "queue_rejected_fail_closed",
            "automatic_retry": False,
        },
        "derivation": {
            "recovery_budget_ms": recovery_budget_ms,
            "recovery_consumption_samples": recovery_consumption_samples,
            "full_queue_horizon_ms": full_queue_horizon_ms,
            "queue_capacity_samples": queue_capacity_samples,
            "maximum_batch_samples": maximum_batch_samples,
            "watermark_guard_samples": 1,
        },
        "note": (
            "These reviewed values are deployment input only. Buffered motion "
            "remains disabled until a separate firmware and physical-motion gate."
        ),
    }
