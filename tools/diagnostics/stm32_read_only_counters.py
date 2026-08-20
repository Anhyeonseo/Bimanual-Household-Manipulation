#!/usr/bin/env python3
"""Measure STM32 heartbeat and rejected-frame counters without motion calls."""

from __future__ import annotations

import argparse
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read HELLO/STATE counters while sending HEARTBEAT only; "
            "never ARM, ENABLE, CLEAR_FAULT, or SETPOINT"
        )
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--duration", type=float, default=60.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(
        serial,
        device,
        115200,
        timeout_s=0.2,
    )
    try:
        transport = ActuatorTransport(port, response_timeout_s=0.2)
        hello = transport.enter_binary_mode()
        start = transport.get_state(include_positions=False)
        print(
            "HELLO "
            f"protocol={hello.protocol_version} "
            f"joints={hello.joint_count} "
            f"firmware=0x{hello.firmware_version:08X} "
            f"calibration=0x{hello.calibration_hash:08X} "
            f"capabilities=0x{hello.capabilities:08X} "
            f"stop_latched={int(hello.stop_latched)}"
        )
        print(
            "START "
            f"status={start.status_code} "
            f"stop_latched={int(start.stop_latched)} "
            f"heartbeat={start.heartbeat_count} "
            f"rejected={start.rejected_frame_count}"
        )

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            transport.heartbeat()
            time.sleep(0.1)

        end = transport.get_state(include_positions=False)
        heartbeat_delta = (
            end.heartbeat_count - start.heartbeat_count
        ) & 0xFFFFFFFF
        rejected_delta = (
            end.rejected_frame_count - start.rejected_frame_count
        ) & 0xFFFFFFFF
        print(
            "END "
            f"status={end.status_code} "
            f"stop_latched={int(end.stop_latched)} "
            f"heartbeat={end.heartbeat_count} "
            f"rejected={end.rejected_frame_count}"
        )
        print(
            "DELTA "
            f"heartbeat={heartbeat_delta} "
            f"rejected={rejected_delta} "
            f"heartbeat_increased={heartbeat_delta > 0} "
            f"rejected_delta_zero={rejected_delta == 0}"
        )
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
