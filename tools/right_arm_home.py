#!/usr/bin/env python3
"""Read and set the current right-arm STS3215 pose as raw-2048 physical q0.

This utility is for a Waveshare Bus Servo Adapter (A) connected directly to a
PC.  It uses the STS3215/Feetech protocol-0 one-key centering command: with
Torque Enable already zero, a write of value 128 to register 40 changes the
servo's internal offset so its *current physical pose* becomes raw 2048.  It
never sends Goal Position and therefore does not command a 2048 movement.

Centering is intentionally opt-in and requires the exact confirmation phrase.
It writes each servo separately, leaves torque disabled, and requires an
operator power-cycle plus read-only verification before it writes a q0 record.
Do not run it while any other controller is attached to the same servo bus.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import time
from typing import Final, Protocol

try:
    import serial
except ImportError as error:  # pragma: no cover - depends on the operator PC
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from error


SERVO_IDS: Final = (1, 2, 3, 4, 5, 6)
TARGET_RAW: Final = 2048
PRESENT_POSITION_ADDRESS: Final = 56
CALIBRATED_OFFSET_ADDRESS: Final = 31
TORQUE_ENABLE_ADDRESS: Final = 40
ONE_KEY_CENTER_COMMAND: Final = 128
DEFAULT_BAUD: Final = 1_000_000
CENTER_CONFIRMATION: Final = "RIGHT_ARM_POSE_IS_Q0_CENTER_WITHOUT_MOTION"


class ServoProtocolError(RuntimeError):
    """The adapter returned no valid, successful response for a request."""


class SerialLike(Protocol):
    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...

    def read(self, size: int = 1) -> bytes: ...

    def reset_input_buffer(self) -> None: ...


def checksum(frame_without_checksum: bytes) -> int:
    """Return the Feetech protocol-0 inverted-byte checksum."""

    if len(frame_without_checksum) < 4 or frame_without_checksum[:2] != b"\xff\xff":
        raise ValueError("a protocol-0 frame must begin with FF FF")
    return (~sum(frame_without_checksum[2:])) & 0xFF


def instruction_packet(servo_id: int, instruction: int, parameters: bytes = b"") -> bytes:
    """Build one unicast Feetech protocol-0 instruction packet."""

    if not 0 <= servo_id <= 0xFE:
        raise ValueError("servo ID must be within 0..254")
    if not 0 <= instruction <= 0xFF:
        raise ValueError("instruction must be a byte")
    body = bytes((servo_id, len(parameters) + 2, instruction)) + parameters
    frame = b"\xff\xff" + body
    return frame + bytes((checksum(frame),))


def parse_status_frame(raw: bytes, expected_id: int, expected_data_length: int) -> bytes:
    """Extract one valid status frame from possibly stale leading serial bytes."""

    for index in range(max(0, len(raw) - 3)):
        if raw[index : index + 2] != b"\xff\xff" or index + 4 > len(raw):
            continue
        servo_id = raw[index + 2]
        length = raw[index + 3]
        frame_end = index + 4 + length
        if length < 2 or frame_end > len(raw):
            continue
        frame = raw[index:frame_end]
        if checksum(frame[:-1]) != frame[-1] or servo_id != expected_id:
            continue
        if length != expected_data_length + 2:
            continue
        status = frame[4]
        if status != 0:
            raise ServoProtocolError(
                f"servo ID {expected_id} returned status error 0x{status:02X}"
            )
        return frame[5:-1]
    raise ServoProtocolError(
        f"no valid response from servo ID {expected_id} "
        f"(expected {expected_data_length} data byte(s))"
    )


@dataclass
class Sts3215Bus:
    port: SerialLike
    response_timeout_s: float = 0.15

    def read_data(self, servo_id: int, address: int, width: int) -> bytes:
        if not 1 <= servo_id <= 0xFD:
            raise ValueError("read requires a unicast servo ID")
        request = instruction_packet(servo_id, 0x02, bytes((address, width)))
        self.port.reset_input_buffer()
        self.port.write(request)
        self.port.flush()
        deadline = time.monotonic() + self.response_timeout_s
        received = bytearray()
        while time.monotonic() < deadline:
            chunk = self.port.read(64)
            if chunk:
                received.extend(chunk)
                try:
                    return parse_status_frame(bytes(received), servo_id, width)
                except ServoProtocolError:
                    # A fragmented reply or stale leading bytes can become valid later.
                    pass
        return parse_status_frame(bytes(received), servo_id, width)

    def read_u16(self, servo_id: int, address: int) -> int:
        return int.from_bytes(self.read_data(servo_id, address, 2), byteorder="little")

    def read_u8(self, servo_id: int, address: int) -> int:
        return self.read_data(servo_id, address, 1)[0]

    def write_u8(self, servo_id: int, address: int, value: int) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError("u8 value is out of range")
        packet = instruction_packet(servo_id, 0x03, bytes((address, value)))
        self.port.reset_input_buffer()
        written = self.port.write(packet)
        self.port.flush()
        if written != len(packet):
            raise ServoProtocolError("serial adapter accepted only a partial Write")


def read_positions(bus: Sts3215Bus) -> dict[int, int]:
    return {
        servo_id: bus.read_u16(servo_id, PRESENT_POSITION_ADDRESS)
        for servo_id in SERVO_IDS
    }


def read_torque_states(bus: Sts3215Bus) -> dict[int, int]:
    return {
        servo_id: bus.read_u8(servo_id, TORQUE_ENABLE_ADDRESS)
        for servo_id in SERVO_IDS
    }


def read_signed_offset(bus: Sts3215Bus, servo_id: int) -> int:
    raw = bus.read_u16(servo_id, CALIBRATED_OFFSET_ADDRESS)
    return raw - 0x10000 if raw >= 0x8000 else raw


def format_positions(positions: dict[int, int]) -> str:
    return " ".join(f"ID{servo_id}={positions[servo_id]}" for servo_id in SERVO_IDS)


def center_current_pose(bus: Sts3215Bus) -> tuple[dict[int, int], dict[int, int]]:
    """Apply the documented torque-off one-key-center sequence to all six IDs."""

    before = read_positions(bus)
    torque_states = read_torque_states(bus)
    enabled = [servo_id for servo_id, state in torque_states.items() if state != 0]
    if enabled:
        raise ServoProtocolError(
            "one-key centering requires torque disabled on every servo; "
            "torque is still enabled for ID(s) " + ", ".join(map(str, enabled))
        )
    offsets_before: dict[int, int] = {}
    for servo_id in SERVO_IDS:
        offsets_before[servo_id] = read_signed_offset(bus, servo_id)
        # Reasserting zero is part of the firmware's centering sequence and is
        # safe only because it was checked above for every servo.
        bus.write_u8(servo_id, TORQUE_ENABLE_ADDRESS, 0)
        time.sleep(0.05)
        bus.write_u8(servo_id, TORQUE_ENABLE_ADDRESS, ONE_KEY_CENTER_COMMAND)
        time.sleep(0.5)
        print(
            "RIGHT_ARM_CENTER_COMMAND_SENT "
            f"ID={servo_id} position_before={before[servo_id]} "
            f"offset_before={offsets_before[servo_id]}"
        )
    return before, offsets_before


def q0_record(
    verified_positions: dict[int, int],
    tolerance_raw: int,
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "record_kind": "physical_right_arm_q0",
        "arm_slot": "right",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "definition": (
            "Physical pose established by STS3215 one-key centering; all six "
            "servos were verified at raw 2048 after a right-arm power cycle. "
            "The five arm joints map to ROS/MoveIt q=0."
        ),
        "servo_ids": list(SERVO_IDS),
        "target_raw": TARGET_RAW,
        "settle_tolerance_raw": tolerance_raw,
        "raw_after_power_cycle": [verified_positions[servo_id] for servo_id in SERVO_IDS],
        "arm_joint_positions_rad": [0.0] * 5,
        "gripper_raw": TARGET_RAW,
        "next_required_calibration": (
            "Confirm ID-to-joint mapping, positive directions, conservative raw limits, "
            "and right-arm URDF visual registration before enabling planned motion."
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        required=True,
        help="Waveshare adapter serial device, e.g. COM5 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help="adapter/servo bus baud rate (default: 1000000)",
    )
    parser.add_argument("--response-timeout", type=float, default=0.15)
    parser.add_argument(
        "--center-current-to-2048", action="store_true",
        help="one-key center current torque-off pose; does not send Goal Position",
    )
    parser.add_argument(
        "--verify-q0", action="store_true",
        help="verify all current raw readings are 2048 after the required power cycle",
    )
    parser.add_argument(
        "--confirm", choices=(CENTER_CONFIRMATION,),
        help=f"required with --center-current-to-2048: {CENTER_CONFIRMATION}",
    )
    parser.add_argument("--tolerance-raw", type=int, default=10)
    parser.add_argument(
        "--save-q0", type=Path,
        help="write q0 only after a successful --verify-q0 check",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.baud <= 0 or args.response_timeout <= 0.0:
        raise SystemExit("--baud and --response-timeout must be positive")
    if not 0 <= args.tolerance_raw <= 50:
        raise SystemExit("--tolerance-raw must be within 0..50")
    if args.center_current_to_2048 and args.verify_q0:
        raise SystemExit("run centering, power-cycle, then run verification separately")
    if args.center_current_to_2048 and args.confirm != CENTER_CONFIRMATION:
        raise SystemExit(f"centering requires --confirm {CENTER_CONFIRMATION}")
    if args.save_q0 and not args.verify_q0:
        raise SystemExit("--save-q0 is allowed only together with --verify-q0")

    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.2) as port:
        bus = Sts3215Bus(port, args.response_timeout)
        positions = read_positions(bus)
        torque_states = read_torque_states(bus)
        print(
            "RIGHT_ARM_RAW_READ_PASS "
            f"positions={format_positions(positions)} "
            + " ".join(
                f"ID{servo_id}_torque={torque_states[servo_id]}"
                for servo_id in SERVO_IDS
            )
        )
        if args.center_current_to_2048:
            before, offsets_before = center_current_pose(bus)
            print(
                "RIGHT_ARM_CENTER_COMMANDS_SENT_POWER_CYCLE_REQUIRED "
                f"positions_before={format_positions(before)} "
                + " ".join(
                    f"ID{servo_id}_offset_before={offsets_before[servo_id]}"
                    for servo_id in SERVO_IDS
                )
            )
            return 0
        if not args.verify_q0:
            return 0
        maximum_error = max(abs(value - TARGET_RAW) for value in positions.values())
        if maximum_error > args.tolerance_raw:
            raise SystemExit(
                "q0 verification failed; expected every raw position to be "
                f"{TARGET_RAW} ± {args.tolerance_raw}, got {format_positions(positions)}"
            )
        print(
            "RIGHT_ARM_Q0_VERIFIED "
            f"positions={format_positions(positions)} max_error_raw={maximum_error}"
        )

    if args.save_q0:
        output = args.save_q0.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(q0_record(positions, args.tolerance_raw), indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"RIGHT_ARM_Q0_SAVED output={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
