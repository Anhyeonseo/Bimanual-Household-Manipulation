#!/usr/bin/env python3
"""Stress the 921600-baud F2.5 host-link candidate without motor power.

This tool uses only protocol-v1 validation-only buffered frames.  Firmware
proves that every response remained outside the executor: queued, accepted,
and applied sample counts must all stay zero.  Both servo 12 V domains must be
physically off before the exact confirmation is supplied.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.protocol import (
    BUFFERED_SETPOINT_MAX_SAMPLES,
    SETPOINT_STATUS,
    SETPOINT_STATUS_EXTENDED,
    SETPOINT_STATUS_F0_METRICS,
    SETPOINT_STATUS_F3_CONTROL_TICK_METRICS,
    SETPOINT_STATUS_H2_TELEMETRY,
    SETPOINT_STATUS_LATENESS,
    BufferedSetpointFlags,
    BufferedSetpointSample,
    Frame,
    MessageType,
    encode_buffered_setpoint_payload,
    encode_frame,
)
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


CONFIRMATION = "F25_HOST_LINK_STRESS_NO_MOTION_SERVO_12V_OFF"
EXPECTED_FIRMWARE_VERSION = 0x00023C01
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_PROTOCOL_VERSION = 1
EXPECTED_JOINT_COUNT = 6
HOST_BAUD = 921_600
F25_CAPABILITY = 0x00800000
POSITION_FEEDBACK_CAPABILITY = 0x00000008
VALIDATION_CAPABILITY = 0x00000400
ASYNC_HOST_TX_CAPABILITY = 0x00002000
EXPECTED_CAPABILITIES = (
    F25_CAPABILITY
    | POSITION_FEEDBACK_CAPABILITY
    | VALIDATION_CAPABILITY
    | ASYNC_HOST_TX_CAPABILITY
)
SAFE_DISABLED_STATE = 1

DEFAULT_DURATION_S = 30.0 * 60.0
DEFAULT_INTERVAL_MS = 20
DEFAULT_LEAD_MS = 200
DEFAULT_SAMPLE_SPACING_MS = 20
TRACK_A_REFILL_INTERVAL_MS = 180
TRACK_B_CONTROL_INTERVAL_MS = 50
WIRE_TRAFFIC_MULTIPLIER_GATE = 2.0
ZERO_POSITIONS_URAD = (0,) * 6


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS)
    parser.add_argument("--lead-ms", type=int, default=DEFAULT_LEAD_MS)
    parser.add_argument(
        "--sample-spacing-ms",
        type=int,
        default=DEFAULT_SAMPLE_SPACING_MS,
    )
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/f25/host_link_921600_no_motion.json",
    )
    return parser.parse_args()


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("percentile requires at least one value")
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def representative_wire_bytes(sample_count: int) -> tuple[int, int]:
    """Return encoded request and maximum current status response sizes."""

    if not 1 <= sample_count <= BUFFERED_SETPOINT_MAX_SAMPLES:
        raise ValueError("sample_count must be in 1..9")
    samples = tuple(
        BufferedSetpointSample(index * 20, ZERO_POSITIONS_URAD)
        for index in range(sample_count)
    )
    payload = encode_buffered_setpoint_payload(0xFFFFFFFF, samples)
    flags = int(
        BufferedSetpointFlags.VALIDATION_ONLY
        | BufferedSetpointFlags.CANDIDATE
        | BufferedSetpointFlags.BEGIN
        | BufferedSetpointFlags.START
        | BufferedSetpointFlags.END
    )
    request = encode_frame(
        Frame(
            MessageType.SETPOINT_BATCH,
            flags=flags,
            sequence=0xFFFFFFFF,
            sender_time_ms=0xFFFFFFFF,
            payload=payload,
        )
    )
    status_payload_size = sum(
        item.size
        for item in (
            SETPOINT_STATUS,
            SETPOINT_STATUS_EXTENDED,
            SETPOINT_STATUS_LATENESS,
            SETPOINT_STATUS_F0_METRICS,
            SETPOINT_STATUS_H2_TELEMETRY,
            SETPOINT_STATUS_F3_CONTROL_TICK_METRICS,
        )
    )
    # 0xFF prevents the representative response from looking artificially
    # short due to long zero runs in the COBS payload.
    response = encode_frame(
        Frame(
            MessageType.SETPOINT_STATUS,
            sequence=0xFFFFFFFF,
            sender_time_ms=0xFFFFFFFF,
            payload=bytes([0xFF]) * status_payload_size,
        )
    )
    return len(request), len(response)


def planned_worst_case_wire_bps() -> float:
    """Derive the larger Track A/B wire load; never sum two owners."""

    response_bytes = representative_wire_bytes(9)[1]
    track_a_request = representative_wire_bytes(9)[0]
    track_b_request = representative_wire_bytes(3)[0]
    track_a = (track_a_request + response_bytes) * (
        1000.0 / TRACK_A_REFILL_INTERVAL_MS
    )
    track_b = (track_b_request + response_bytes) * (
        1000.0 / TRACK_B_CONTROL_INTERVAL_MS
    )
    return max(track_a, track_b)


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; both servo 12 V domains must be off"
        )
    if args.duration_s < 10.0:
        raise SystemExit("--duration-s must be at least 10 seconds")
    if args.interval_ms <= 0 or args.sample_spacing_ms <= 0:
        raise SystemExit("interval and sample spacing must be positive")
    if not 20 <= args.lead_ms <= 2_000:
        raise SystemExit("--lead-ms must remain inside firmware 20..2000 ms")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    round_trip_ms: list[float] = []
    schedule_lateness_ms: list[float] = []
    transport_error_count = 0
    validation_failure_count = 0
    request_bytes, response_bytes = representative_wire_bytes(9)
    expected_minimum_bps = (
        planned_worst_case_wire_bps() * WIRE_TRAFFIC_MULTIPLIER_GATE
    )
    started = 0.0
    next_dispatch = 0.0
    last_progress = 0.0
    completed = 0
    hello = None
    start_state = None
    end_state = None

    try:
        transport = ActuatorTransport(port, response_timeout_s=0.4)
        hello = transport.enter_binary_mode()
        if hello.firmware_version != EXPECTED_FIRMWARE_VERSION:
            raise RuntimeError(
                "wrong firmware for F2.5 gate: "
                f"expected=0x{EXPECTED_FIRMWARE_VERSION:08X} "
                f"actual=0x{hello.firmware_version:08X}"
            )
        if hello.protocol_version != EXPECTED_PROTOCOL_VERSION:
            raise RuntimeError("F2.5 candidate must retain protocol v1")
        if hello.joint_count != EXPECTED_JOINT_COUNT:
            raise RuntimeError("F2.5 candidate joint identity changed")
        if hello.calibration_hash != EXPECTED_CALIBRATION_HASH:
            raise RuntimeError("F2.5 candidate calibration hash changed")
        if hello.capabilities != EXPECTED_CAPABILITIES:
            raise RuntimeError(
                "F2.5 candidate capabilities are not validation-only: "
                f"expected=0x{EXPECTED_CAPABILITIES:08X} "
                f"actual=0x{hello.capabilities:08X}"
            )
        if hello.stop_latched:
            raise RuntimeError("F2.5 candidate starts with stop latched")

        start_state = transport.get_state(include_positions=False)
        if start_state.stop_latched or start_state.status_code != 0:
            raise RuntimeError("F2.5 candidate state is not healthy before stress")
        start_rejected = start_state.rejected_frame_count
        samples = tuple(
            BufferedSetpointSample(
                index * args.sample_spacing_ms,
                ZERO_POSITIONS_URAD,
            )
            for index in range(BUFFERED_SETPOINT_MAX_SAMPLES)
        )

        started = time.monotonic()
        next_dispatch = started
        last_progress = started
        print(
            "F25_HOST_LINK_STRESS_START "
            f"device={device} baud={HOST_BAUD} duration_s={args.duration_s:.1f} "
            f"request_samples=9 expected_minimum_wire_Bps={expected_minimum_bps:.1f} "
            "validation_only=true motion_authorized=false servo_12v=off"
        )
        while time.monotonic() - started < args.duration_s:
            now = time.monotonic()
            if now < next_dispatch:
                time.sleep(min(next_dispatch - now, 0.002))
                continue
            schedule_lateness_ms.append(max(0.0, (now - next_dispatch) * 1000.0))
            heartbeat = transport.heartbeat()
            if heartbeat.stop_latched or heartbeat.status_code != 0:
                raise RuntimeError("heartbeat became unhealthy during stress")
            first_apply_tick = (heartbeat.last_heartbeat_ms + args.lead_ms) & 0xFFFFFFFF
            request_started = time.monotonic()
            try:
                result = transport.validate_buffered_candidate(
                    first_apply_tick,
                    samples,
                )
            except Exception:
                transport_error_count += 1
                raise
            round_trip_ms.append((time.monotonic() - request_started) * 1000.0)
            if (
                result.safety_state != SAFE_DISABLED_STATE
                or result.queued_samples != 0
                or result.accepted_samples != 0
                or result.applied_samples != 0
            ):
                validation_failure_count += 1
                raise RuntimeError("validation-only response lost no-motion proof")
            completed += 1
            next_dispatch += args.interval_ms / 1000.0
            now = time.monotonic()
            if now - last_progress >= 10.0:
                print(
                    "F25_HOST_LINK_STRESS_PROGRESS "
                    f"elapsed_s={now - started:.1f} frames={completed} "
                    f"rtt_p99_ms={percentile(round_trip_ms, 0.99):.3f}"
                )
                last_progress = now

        end_state = transport.get_state(include_positions=False)
        if end_state.stop_latched or end_state.status_code != 0:
            raise RuntimeError("F2.5 candidate state is not healthy after stress")
        elapsed_s = time.monotonic() - started
        actual_wire_bps = completed * (request_bytes + response_bytes) / elapsed_s
        rejected_delta = (
            end_state.rejected_frame_count - start_rejected
        ) & 0xFFFFFFFF
        overall_pass = (
            transport_error_count == 0
            and validation_failure_count == 0
            and rejected_delta == 0
            and actual_wire_bps >= expected_minimum_bps
        )
        document = {
            "schema_version": 1,
            "record_kind": "f25_host_link_921600_no_motion_stress",
            "overall_verdict": (
                "F25_HOST_LINK_STRESS_PASS_NO_MOTION"
                if overall_pass
                else "F25_HOST_LINK_STRESS_FAIL"
            ),
            "validation_only": True,
            "motion_authorized": False,
            "servo_12v_confirmed_off": True,
            "protocol_version": hello.protocol_version,
            "firmware_version": f"0x{hello.firmware_version:08X}",
            "calibration_hash": f"0x{hello.calibration_hash:08X}",
            "capabilities": f"0x{hello.capabilities:08X}",
            "device": device,
            "baud": HOST_BAUD,
            "duration_requested_s": args.duration_s,
            "duration_actual_s": elapsed_s,
            "frames_completed": completed,
            "request_samples_per_frame": BUFFERED_SETPOINT_MAX_SAMPLES,
            "representative_request_wire_bytes": request_bytes,
            "representative_response_wire_bytes": response_bytes,
            "planned_worst_case_wire_Bps": planned_worst_case_wire_bps(),
            "required_multiplier": WIRE_TRAFFIC_MULTIPLIER_GATE,
            "required_minimum_wire_Bps": expected_minimum_bps,
            "actual_wire_Bps": actual_wire_bps,
            "transport_error_count": transport_error_count,
            "validation_failure_count": validation_failure_count,
            "rejected_frame_delta": rejected_delta,
            "round_trip_ms": {
                "median": statistics.median(round_trip_ms),
                "p99": percentile(round_trip_ms, 0.99),
                "maximum": max(round_trip_ms),
            },
            "schedule_lateness_ms": {
                "median": statistics.median(schedule_lateness_ms),
                "p99": percentile(schedule_lateness_ms, 0.99),
                "maximum": max(schedule_lateness_ms),
            },
            "no_motion_proof": {
                "required_safety_state": SAFE_DISABLED_STATE,
                "queued_samples": 0,
                "accepted_samples": 0,
                "applied_samples": 0,
            },
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(args.output)
        print(
            f"F25_HOST_LINK_STRESS_{'PASS_NO_MOTION' if overall_pass else 'FAIL'} "
            f"frames={completed} actual_wire_Bps={actual_wire_bps:.1f} "
            f"required_wire_Bps={expected_minimum_bps:.1f} "
            f"transport_errors={transport_error_count} "
            f"rejected_delta={rejected_delta} output={args.output} sha256={digest}"
        )
        return 0 if overall_pass else 2
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
