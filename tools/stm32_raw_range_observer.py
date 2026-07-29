#!/usr/bin/env python3
"""Observe manually positioned servo raw ranges with torque disabled.

This diagnostic never arms, enables, clears a fault, or sends a setpoint. It
explicitly requests DISABLE before sampling. The user must physically support
the arm because disabling servo torque can let gravity move the mechanism.
The output is evidence only and never modifies a calibration file.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import serial
import yaml

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


CONFIRMATION = "I_AM_SUPPORTING_THE_ARM"
JOINT_NAMES = (
    "BASE",
    "SHOULDER",
    "ELBOW",
    "WRIST_FLEX",
    "WRIST_ROLL",
    "GRIPPER",
)


@dataclass
class RawRangeAccumulator:
    minimum: list[int] | None = None
    maximum: list[int] | None = None
    samples: int = 0

    def update(self, positions: tuple[int, ...]) -> None:
        if len(positions) != len(JOINT_NAMES):
            raise ValueError("expected six raw joint positions")
        values = [int(value) for value in positions]
        if any(value < 0 or value > 4095 for value in values):
            raise ValueError("raw position must be within 0..4095")
        if self.minimum is None:
            self.minimum = values.copy()
            self.maximum = values.copy()
        else:
            assert self.maximum is not None
            self.minimum = [
                min(old, new) for old, new in zip(self.minimum, values, strict=True)
            ]
            self.maximum = [
                max(old, new) for old, new in zip(self.maximum, values, strict=True)
            ]
        self.samples += 1

    def document(self, duration_s: float, firmware: int, calibration_hash: int) -> dict:
        if self.minimum is None or self.maximum is None or self.samples == 0:
            raise RuntimeError("no raw samples were collected")
        return {
            "schema_version": 1,
            "status": "OBSERVED_TORQUE_OFF_RANGE_REQUIRES_ENGINEERING_REVIEW",
            "motion_authorized": False,
            "apply_to_calibration": False,
            "automatic_limit_expansion": False,
            "capture": {
                "duration_s": float(duration_s),
                "sample_count": self.samples,
                "firmware_version": f"0x{firmware:08X}",
                "calibration_hash": f"0x{calibration_hash:08X}",
                "torque_state": "DISABLED",
                "movement_source": "USER_MANUAL_WITH_ARM_PHYSICALLY_SUPPORTED",
            },
            "observed_joints": [
                {
                    "name": name,
                    "observed_minimum_raw": minimum,
                    "observed_maximum_raw": maximum,
                }
                for name, minimum, maximum in zip(
                    JOINT_NAMES,
                    self.minimum,
                    self.maximum,
                    strict=True,
                )
            ],
            "required_next_gate": (
                "review cable clearance and collision-free mechanical margins; "
                "derive conservative limits; update host and firmware together; "
                "reflash and verify calibration hash before any motion"
            ),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Torque-off raw range observer. The arm can fall when DISABLE is sent."
        )
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--sample-hz", type=float, default=5.0)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--confirm-supported-torque-off",
        required=True,
        choices=(CONFIRMATION,),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if not 1.0 <= args.sample_hz <= 20.0:
        raise SystemExit("--sample-hz must be within 1..20")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, 115200, timeout_s=0.2)
    accumulator = RawRangeAccumulator()
    try:
        transport = ActuatorTransport(port, response_timeout_s=0.5)
        hello = transport.enter_binary_mode()
        transport.disable()
        start = transport.get_state(include_positions=True)
        if start.raw_positions is None:
            raise RuntimeError("position feedback missing after DISABLE")
        accumulator.update(start.raw_positions)
        print(
            "RAW_RANGE_OBSERVER_READY "
            "torque=DISABLED motion_commands=NONE "
            f"positions={','.join(str(value) for value in start.raw_positions)}"
        )

        interval = 1.0 / args.sample_hz
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            transport.heartbeat()
            state = transport.get_state(include_positions=True)
            if state.raw_positions is None:
                raise RuntimeError("position feedback missing")
            accumulator.update(state.raw_positions)
            time.sleep(interval)

        result = accumulator.document(
            args.duration,
            hello.firmware_version,
            hello.calibration_hash,
        )
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        print(
            "RAW_RANGE_OBSERVER_PASS "
            f"samples={accumulator.samples} output={output} "
            "motion_authorized=0 apply_to_calibration=0"
        )
    finally:
        try:
            if "transport" in locals():
                try:
                    transport.disable()
                except Exception as error:
                    print(
                        "RAW_RANGE_OBSERVER_FINAL_DISABLE_UNCONFIRMED "
                        f"reason={error}; cut 12V and support the arm",
                    )
        finally:
            port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
