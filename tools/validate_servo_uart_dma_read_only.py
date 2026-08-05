#!/usr/bin/env python3
"""Fail-closed READ_ONLY soak for the transaction-scoped servo RX DMA lifecycle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.hardware_identity import validate_hardware_identity
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


EXPECTED_CALIBRATION_HASH = 0x8AD27897
ERROR_COUNTERS = (
    "failure_count",
    "recovery_count",
    "receiver_resync_count",
    "discarded_bytes",
    "timeout_count",
    "overflow_count",
    "pe_count",
    "ne_count",
    "fe_count",
    "ore_count",
    "rto_count",
    "dma_error_count",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--duration-s", type=float, default=300.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--heartbeat-hz", type=float, default=10.0)
    parser.add_argument("--position-hz", type=float, default=5.0)
    parser.add_argument("--diagnostics-period-s", type=float, default=60.0)
    return parser.parse_args()


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = min(len(ordered) - 1, int(quantile * len(ordered)))
    return ordered[index]


def health_counters(sample) -> dict[str, int]:
    health = sample.bus_health
    if health is None or health.schema_version != 2:
        raise RuntimeError("power-domain lifecycle health schema v2 unavailable")
    if health.dma_started or health.receiver_armed:
        raise RuntimeError(
            "transaction-scoped RX DMA remained armed after diagnostics"
        )
    if health.failure_snapshot:
        raise RuntimeError(
            "successful diagnostics retained a failure RX snapshot: "
            f"{health.failure_snapshot.hex()}"
        )
    return {name: int(getattr(health, name)) for name in ERROR_COUNTERS}


def validate_initial_health(counters: dict[str, int]) -> None:
    fatal_names = (
        "failure_count",
        "discarded_bytes",
        "timeout_count",
        "overflow_count",
        "pe_count",
        "ne_count",
        "ore_count",
        "rto_count",
        "dma_error_count",
    )
    fatal = {
        name: counters[name]
        for name in fatal_names
        if counters[name] != 0
    }
    recovery_count = counters["recovery_count"]
    framing_count = counters["fe_count"]
    receiver_resync_count = counters["receiver_resync_count"]
    bounded_power_edge_cleanup = (
        recovery_count in (0, 1)
        and framing_count == recovery_count
        and receiver_resync_count == recovery_count
    )
    if fatal or not bounded_power_edge_cleanup:
        raise RuntimeError(
            "cold-start servo UART health was not bounded: "
            f"fatal={fatal} counters={counters}"
        )


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0:
        raise SystemExit("--duration-s must be positive")
    if args.heartbeat_hz <= 0 or args.position_hz <= 0:
        raise SystemExit("heartbeat and position rates must be positive")
    if args.diagnostics_period_s <= 0:
        raise SystemExit("--diagnostics-period-s must be positive")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, 115200, timeout_s=0.5)
    started = time.monotonic()
    deadline = started + args.duration_s
    next_heartbeat = started
    next_position = started
    next_diagnostics = started
    heartbeat_latencies: list[float] = []
    position_latencies: list[float] = []
    diagnostics_latencies: list[float] = []
    diagnostics_snapshots: list[dict] = []
    latest_positions: tuple[int, ...] | None = None
    baseline_counters: dict[str, int] | None = None
    failure: str | None = None
    calls = {"heartbeat": 0, "position": 0, "diagnostics": 0}

    try:
        transport = ActuatorTransport(port, response_timeout_s=2.5)
        hello = transport.enter_binary_mode()
        validate_hardware_identity(hello, EXPECTED_CALIBRATION_HASH)
        if hello.stop_latched:
            raise RuntimeError("HELLO reports STOP latch")

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                call_started = time.monotonic()
                state = transport.heartbeat()
                heartbeat_latencies.append(
                    (time.monotonic() - call_started) * 1000.0
                )
                calls["heartbeat"] += 1
                if state.stop_latched or state.status_code != 0:
                    raise RuntimeError("heartbeat/latch gate failed")
                next_heartbeat += 1.0 / args.heartbeat_hz
                continue

            if now >= next_position:
                call_started = time.monotonic()
                state = transport.get_state(include_positions=True)
                position_latencies.append(
                    (time.monotonic() - call_started) * 1000.0
                )
                calls["position"] += 1
                if state.stop_latched or state.raw_positions is None:
                    raise RuntimeError("position/latch gate failed")
                latest_positions = tuple(state.raw_positions)
                next_position += 1.0 / args.position_hz
                continue

            if now >= next_diagnostics:
                call_started = time.monotonic()
                diagnostics = transport.get_diagnostics()
                diagnostics_latencies.append(
                    (time.monotonic() - call_started) * 1000.0
                )
                calls["diagnostics"] += 1
                counters = health_counters(diagnostics.joints[-1])
                if baseline_counters is None:
                    validate_initial_health(counters)
                    baseline_counters = counters
                deltas = {
                    name: counters[name] - baseline_counters[name]
                    for name in ERROR_COUNTERS
                }
                if any(value != 0 for value in deltas.values()):
                    raise RuntimeError(f"servo UART health delta changed: {deltas}")
                print(
                    "SOAK_CHECKPOINT "
                    f"elapsed_s={time.monotonic() - started:.3f} "
                    f"heartbeat_calls={calls['heartbeat']} "
                    f"position_calls={calls['position']} "
                    f"diagnostics_calls={calls['diagnostics']} "
                    f"recovery_count={counters['recovery_count']} "
                    f"fe_count={counters['fe_count']}",
                    flush=True,
                )
                diagnostics_snapshots.append({
                    "elapsed_s": time.monotonic() - started,
                    "positions_raw": [joint.position_raw for joint in diagnostics.joints],
                    "goals_raw": [joint.goal_position_raw for joint in diagnostics.joints],
                    "torque_enabled": [joint.torque_enabled for joint in diagnostics.joints],
                    "voltage_v": [joint.voltage_raw / 10.0 for joint in diagnostics.joints],
                    "temperature_c": [joint.temperature_c for joint in diagnostics.joints],
                    "error_counters": counters,
                    "lazy_arm_count": (
                        diagnostics.joints[-1].bus_health.lazy_arm_count
                    ),
                    "receiver_resync_count": (
                        diagnostics.joints[-1].bus_health.receiver_resync_count
                    ),
                    "receiver_armed": (
                        diagnostics.joints[-1].bus_health.receiver_armed
                    ),
                })
                next_diagnostics += args.diagnostics_period_s
                continue

            time.sleep(min(0.002, deadline - now))
    except Exception as error:  # fail closed and preserve the first cause
        failure = f"{type(error).__name__}: {error}"
    finally:
        port.close()

    def latency_summary(values: list[float]) -> dict[str, float | int]:
        return {
            "count": len(values),
            "average_ms": statistics.fmean(values) if values else 0.0,
            "p95_ms": percentile(values, 0.95),
            "maximum_ms": max(values, default=0.0),
        }

    document = {
        "schema_version": 1,
        "kind": "servo_uart_circular_dma_read_only_soak",
        "passed": failure is None,
        "failure": failure,
        "device": device,
        "duration_requested_s": args.duration_s,
        "duration_observed_s": time.monotonic() - started,
        "automatic_host_retry_count": 0,
        "motion_command_count": 0,
        "calls": calls,
        "latency": {
            "heartbeat": latency_summary(heartbeat_latencies),
            "position": latency_summary(position_latencies),
            "diagnostics": latency_summary(diagnostics_latencies),
        },
        "latest_positions_raw": latest_positions,
        "baseline_error_counters": baseline_counters,
        "diagnostics_snapshots": diagnostics_snapshots,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(document, indent=2, sort_keys=True) + "\n").encode()
    args.output.write_bytes(encoded)
    print(f"OUTPUT={args.output}")
    print(f"SHA256={hashlib.sha256(encoded).hexdigest()}")
    print(f"PASSED={int(document['passed'])}")
    print("AUTOMATIC_HOST_RETRY_COUNT=0")
    print("MOTION_COMMAND_COUNT=0")
    return 0 if document["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
