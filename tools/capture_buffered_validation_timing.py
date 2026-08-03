#!/usr/bin/env python3
"""Capture Pi-VCP timing using validation-only buffered frames; never move."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import serial

from single_arm_bridge.buffered_validation_capture import (
    MINIMUM_CAPTURE_SAMPLES,
    build_capture_document,
    elapsed_ms,
    estimate_mcu_tick_ms,
    nonnegative_lateness_ms,
)
from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.hardware_identity import validate_hardware_identity
from single_arm_bridge.protocol import BufferedSetpointSample
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


EXPECTED_CALIBRATION_HASH = 0x8AD27897
SAFE_DISABLED_STATE = 1
ZERO_POSITIONS_URAD = (0, 0, 0, 0, 0, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture validation-only buffered timing. This tool never sends "
            "ARM, ENABLE, CLEAR_FAULT, SAFE_STOP, or an executable setpoint."
        )
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=1_000)
    parser.add_argument("--interval-ms", type=int, default=20)
    parser.add_argument("--lead-ms", type=int, default=100)
    parser.add_argument("--sample-spacing-ms", type=int, default=20)
    parser.add_argument("--sync-every", type=int, default=100)
    return parser.parse_args()


def raw_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def wait_until_raw(deadline_ns: int) -> None:
    while True:
        remaining_ns = deadline_ns - raw_ns()
        if remaining_ns <= 0:
            return
        time.sleep(min(remaining_ns / 1_000_000_000.0, 0.005))


def outage_plan(count: int) -> dict[int, int]:
    return {
        count // 4: 20,
        count // 2: 40,
        (3 * count) // 4: 80,
    }


def main() -> int:
    args = parse_args()
    if args.count < MINIMUM_CAPTURE_SAMPLES:
        raise SystemExit("--count must be at least 1000")
    if args.interval_ms <= 0 or args.sync_every <= 0:
        raise SystemExit("interval and sync cadence must be positive")
    if not 20 <= args.lead_ms <= 2_000:
        raise SystemExit("--lead-ms must remain inside firmware 20..2000 ms")
    if args.sample_spacing_ms <= 0:
        raise SystemExit("--sample-spacing-ms must be positive")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, 115200, timeout_s=0.5)
    round_trip: list[float] = []
    jitter: list[float] = []
    lateness: list[float] = []
    outages: list[float] = []
    transport_errors = 0
    hello = None
    try:
        transport = ActuatorTransport(port, response_timeout_s=2.5)
        hello = transport.enter_binary_mode()
        validate_hardware_identity(hello, EXPECTED_CALIBRATION_HASH)
        if hello.stop_latched:
            raise RuntimeError("HELLO reports a latched stop")

        samples = (
            BufferedSetpointSample(0, ZERO_POSITIONS_URAD),
            BufferedSetpointSample(
                args.sample_spacing_ms,
                ZERO_POSITIONS_URAD,
            ),
        )
        planned_dispatch_ns = raw_ns()
        sync_host_ns = planned_dispatch_ns
        sync_mcu_tick_ms = 0
        planned_outages = outage_plan(args.count)

        for index in range(args.count):
            if index % args.sync_every == 0:
                sync_start_ns = raw_ns()
                heartbeat = transport.heartbeat()
                sync_end_ns = raw_ns()
                if heartbeat.stop_latched or heartbeat.status_code != 0:
                    raise RuntimeError("heartbeat is not healthy")
                sync_host_ns = (sync_start_ns + sync_end_ns) // 2
                sync_mcu_tick_ms = heartbeat.last_heartbeat_ms
                planned_dispatch_ns = sync_end_ns + args.interval_ms * 1_000_000

            wait_until_raw(planned_dispatch_ns)
            request_start_ns = raw_ns()
            jitter.append(
                nonnegative_lateness_ms(
                    planned_dispatch_ns,
                    request_start_ns,
                )
            )
            estimated_mcu_now = estimate_mcu_tick_ms(
                sync_mcu_tick_ms,
                sync_host_ns,
                request_start_ns,
            )
            first_apply_tick_ms = (
                estimated_mcu_now + args.lead_ms
            ) & 0xFFFFFFFF
            try:
                result = transport.validate_buffered_candidate(
                    first_apply_tick_ms,
                    samples,
                )
            except Exception:
                transport_errors += 1
                raise
            response_end_ns = raw_ns()
            if result.safety_state != SAFE_DISABLED_STATE:
                raise RuntimeError(
                    "validation response is not SAFE_DISABLED: "
                    f"state={result.safety_state}"
                )
            round_trip.append(elapsed_ms(request_start_ns, response_end_ns))
            apply_deadline_ns = (
                request_start_ns + args.lead_ms * 1_000_000
            )
            lateness.append(
                nonnegative_lateness_ms(apply_deadline_ns, response_end_ns)
            )
            planned_dispatch_ns += args.interval_ms * 1_000_000

            outage_ms = planned_outages.get(index + 1)
            if outage_ms is not None:
                outage_start_ns = raw_ns()
                time.sleep(outage_ms / 1_000.0)
                outage_end_ns = raw_ns()
                outages.append(elapsed_ms(outage_start_ns, outage_end_ns))
                planned_dispatch_ns = (
                    outage_end_ns + args.interval_ms * 1_000_000
                )

        document = build_capture_document(
            firmware_version=hello.firmware_version,
            calibration_hash=hello.calibration_hash,
            capabilities=hello.capabilities,
            requested_samples=args.count,
            interval_ms=args.interval_ms,
            lead_ms=args.lead_ms,
            sample_spacing_ms=args.sample_spacing_ms,
            serial_round_trip_ms=round_trip,
            host_command_jitter_ms=jitter,
            delivery_lateness_ms=lateness,
            host_outage_ms=outages,
            transport_error_count=transport_errors,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"DEVICE={device}")
        print(f"FIRMWARE=0x{hello.firmware_version:08X}")
        print(f"CAPABILITIES=0x{hello.capabilities:08X}")
        print(f"CAPTURED_SAMPLES={len(round_trip)}")
        print(f"TRANSPORT_ERRORS={transport_errors}")
        print(f"OUTPUT={args.output}")
        print("BUFFERED_VALIDATION_TIMING_CAPTURE_PASS_NO_MOTION")
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
