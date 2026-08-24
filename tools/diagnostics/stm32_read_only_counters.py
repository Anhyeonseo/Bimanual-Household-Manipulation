#!/usr/bin/env python3
"""Inspect STM32 state or F8 ASCII servo telemetry without motion calls."""

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
    parser.add_argument(
        "--baud",
        type=int,
        choices=(115200, 921600),
        default=115200,
        help="host UART baud; deployed protocol-v2 F8 firmware uses 921600",
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument(
        "--f8-ascii-servo-scan",
        action="store_true",
        help=(
            "send the firmware's read-only ASCII S command and print all six "
            "left-servo telemetry results; reset to ASCII mode first"
        ),
    )
    return parser.parse_args()


def run_f8_ascii_servo_scan(port: object, timeout_s: float = 5.0) -> bool:
    port.reset_input_buffer()
    port.write(b"S")
    port.flush()
    deadline = time.monotonic() + timeout_s
    axis_lines: list[str] = []
    complete = False
    while time.monotonic() < deadline:
        encoded = port.readline()
        if not encoded:
            continue
        line = encoded.decode("ascii", errors="replace").strip()
        print(line)
        if line.startswith("AXIS ID="):
            axis_lines.append(line)
        if line == "ALL_AXIS_STATUS_END":
            complete = True
            break
    passed = (
        complete
        and len(axis_lines) == 6
        and all("READ_FAIL" not in line for line in axis_lines)
    )
    print(
        "F8_ASCII_SERVO_SCAN_RESULT "
        f"passed={int(passed)} axes={len(axis_lines)} "
        f"complete={int(complete)} motion_commands=0 servo_write_commands=0"
    )
    return passed


def main() -> int:
    args = parse_args()
    if not args.f8_ascii_servo_scan and args.duration <= 0.0:
        raise SystemExit("--duration must be positive")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(
        serial,
        device,
        args.baud,
        timeout_s=0.2,
    )
    try:
        if args.f8_ascii_servo_scan:
            return 0 if run_f8_ascii_servo_scan(port) else 2
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
