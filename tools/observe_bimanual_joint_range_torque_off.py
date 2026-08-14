#!/usr/bin/env python3
"""Observe one manually moved bimanual joint through the R4 read-only topic.

This tool subscribes only.  It never calls a motion, torque, stop, or fault-clear
API and never modifies calibration.  The R4 bridge must already be running in
BIMANUAL_READ_ONLY mode, which verifies Torque Enable=0 on both servo buses.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any


TOPIC = "/bimanual_joint_states"
CONFIRMATION = "I_AM_SUPPORTING_BOTH_ARMS_TORQUE_OFF"
JOINT_BASENAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
RAW_UNITS_PER_TURN = 4096
TURN_RAD = 2.0 * math.pi


def signed_circular_delta(current: int, previous: int) -> int:
    if not 0 <= current < RAW_UNITS_PER_TURN:
        raise ValueError("current raw position must be within 0..4095")
    if not 0 <= previous < RAW_UNITS_PER_TURN:
        raise ValueError("previous raw position must be within 0..4095")
    return (
        (current - previous + RAW_UNITS_PER_TURN // 2)
        % RAW_UNITS_PER_TURN
    ) - RAW_UNITS_PER_TURN // 2


@dataclass(frozen=True)
class JointCalibration:
    name: str
    zero_raw: int
    direction: int


def load_arm_calibration(
    path: Path, expected_slot: str
) -> tuple[JointCalibration, ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("arm_slot") != expected_slot:
        raise ValueError(f"{path}: expected arm_slot={expected_slot}")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError(f"{path}: expected six joints")
    if [joint.get("id") for joint in joints] != list(range(1, 7)):
        raise ValueError(f"{path}: servo IDs must be ordered 1..6")
    output: list[JointCalibration] = []
    for expected_name, joint in zip(JOINT_BASENAMES, joints, strict=True):
        if str(joint.get("name", "")).lower() != expected_name:
            raise ValueError(f"{path}: unexpected joint order")
        direction = int(joint["positive_raw_direction"])
        if direction not in (-1, 1):
            raise ValueError(f"{path}: invalid direction")
        output.append(
            JointCalibration(
                name=f"{expected_slot}_{expected_name}_joint",
                zero_raw=int(joint["zero_raw"]),
                direction=direction,
            )
        )
    return tuple(output)


def radians_to_raw(position_rad: float, calibration: JointCalibration) -> int:
    if not math.isfinite(position_rad):
        raise ValueError("joint position must be finite")
    raw_float = calibration.zero_raw + calibration.direction * (
        position_rad * RAW_UNITS_PER_TURN / TURN_RAD
    )
    raw = int(round(raw_float))
    if not 0 <= raw <= 4095:
        raise ValueError(f"reconstructed raw position outside 0..4095: {raw}")
    if abs(raw_float - raw) > 0.02:
        raise ValueError(
            "joint-state position is not aligned with an integer raw sample: "
            f"raw_float={raw_float:.6f}"
        )
    return raw


@dataclass
class RangeCapture:
    selected_index: int
    minimum: list[int] | None = None
    maximum: list[int] | None = None
    samples: int = 0
    selected_direction_reversals: int = 0
    selected_wrap_crossings: int = 0
    selected_maximum_step_raw: int = 0
    selected_unwrapped_minimum_raw: int | None = None
    selected_unwrapped_maximum_raw: int | None = None
    _last_selected_raw: int | None = None
    _selected_unwrapped_raw: int | None = None
    _last_direction: int = 0

    def update(self, positions_raw: tuple[int, ...]) -> None:
        if len(positions_raw) != 12:
            raise ValueError("expected 12 raw joint positions")
        if any(raw < 0 or raw > 4095 for raw in positions_raw):
            raise ValueError("raw position must be within 0..4095")
        values = list(positions_raw)
        if self.minimum is None:
            self.minimum = values.copy()
            self.maximum = values.copy()
        else:
            assert self.maximum is not None
            self.minimum = [
                min(old, new)
                for old, new in zip(self.minimum, values, strict=True)
            ]
            self.maximum = [
                max(old, new)
                for old, new in zip(self.maximum, values, strict=True)
            ]

        selected = values[self.selected_index]
        if self._last_selected_raw is None:
            self._selected_unwrapped_raw = selected
            self.selected_unwrapped_minimum_raw = selected
            self.selected_unwrapped_maximum_raw = selected
        else:
            delta = signed_circular_delta(selected, self._last_selected_raw)
            self.selected_maximum_step_raw = max(
                self.selected_maximum_step_raw, abs(delta)
            )
            if abs(selected - self._last_selected_raw) > 2048:
                self.selected_wrap_crossings += 1
            assert self._selected_unwrapped_raw is not None
            self._selected_unwrapped_raw += delta
            assert self.selected_unwrapped_minimum_raw is not None
            assert self.selected_unwrapped_maximum_raw is not None
            self.selected_unwrapped_minimum_raw = min(
                self.selected_unwrapped_minimum_raw,
                self._selected_unwrapped_raw,
            )
            self.selected_unwrapped_maximum_raw = max(
                self.selected_unwrapped_maximum_raw,
                self._selected_unwrapped_raw,
            )
            direction = 1 if delta >= 2 else -1 if delta <= -2 else 0
            if (
                direction != 0
                and self._last_direction != 0
                and direction != self._last_direction
            ):
                self.selected_direction_reversals += 1
            if direction != 0:
                self._last_direction = direction
        self._last_selected_raw = selected
        self.samples += 1

    def summary(self) -> tuple[list[int], list[int], list[int]]:
        if self.minimum is None or self.maximum is None or self.samples == 0:
            raise RuntimeError("no joint samples captured")
        spans = [
            maximum - minimum
            for minimum, maximum in zip(
                self.minimum, self.maximum, strict=True
            )
        ]
        return self.minimum, self.maximum, spans


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", required=True, choices=("left", "right"))
    parser.add_argument("--joint", required=True, choices=JOINT_BASENAMES)
    parser.add_argument("--context", required=True)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--minimum-samples", type=int, default=60)
    parser.add_argument(
        "--confirmation", required=True, choices=(CONFIRMATION,)
    )
    parser.add_argument(
        "--left-calibration",
        type=Path,
        default=root
        / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--right-calibration",
        type=Path,
        default=root / "config/right_arm_calibration.candidate.json",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 10.0 <= args.duration_s <= 600.0:
        raise SystemExit("--duration-s must be within 10..600")
    if args.minimum_samples < 10:
        raise SystemExit("--minimum-samples must be at least 10")
    if not args.context.strip():
        raise SystemExit("--context must not be empty")

    left = load_arm_calibration(args.left_calibration, "left")
    right = load_arm_calibration(args.right_calibration, "right")
    calibrations = left + right
    expected_names = tuple(item.name for item in calibrations)
    selected_index = (0 if args.arm == "left" else 6) + JOINT_BASENAMES.index(
        args.joint
    )
    selected_name = expected_names[selected_index]

    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("bimanual_joint_range_torque_off_observer")
    capture = RangeCapture(selected_index=selected_index)
    records: list[dict[str, Any]] = []
    validation_error: Exception | None = None

    def on_joint_state(message: JointState) -> None:
        nonlocal validation_error
        if validation_error is not None:
            return
        try:
            if tuple(message.name) != expected_names:
                raise ValueError("unexpected bimanual joint identity/order")
            if len(message.position) != 12:
                raise ValueError("expected 12 joint positions")
            positions_raw = tuple(
                radians_to_raw(float(position), calibration)
                for position, calibration in zip(
                    message.position, calibrations, strict=True
                )
            )
            capture.update(positions_raw)
            records.append(
                {
                    "sample": capture.samples,
                    "stamp_sec": int(message.header.stamp.sec),
                    "stamp_nanosec": int(message.header.stamp.nanosec),
                    "positions_raw": positions_raw,
                }
            )
            if capture.samples == 1 or capture.samples % 10 == 0:
                print(
                    "J0_RANGE_SAMPLE "
                    f"joint={selected_name} samples={capture.samples} "
                    f"raw={positions_raw[selected_index]} "
                    f"reversals={capture.selected_direction_reversals}"
                )
        except Exception as error:
            validation_error = error

    subscription = node.create_subscription(
        JointState, TOPIC, on_joint_state, 10
    )
    print(
        "J0_RANGE_OBSERVER_READY "
        f"joint={selected_name} context={args.context} "
        "torque_expected=OFF motion_commands=NONE; "
        "support both arms and move only the selected joint slowly"
    )
    started = time.monotonic()
    try:
        while (
            rclpy.ok()
            and validation_error is None
            and time.monotonic() - started < args.duration_s
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        if validation_error is not None:
            raise RuntimeError(f"J0 topic validation failed: {validation_error}")
        if capture.samples < args.minimum_samples:
            raise RuntimeError(
                f"only {capture.samples}/{args.minimum_samples} samples arrived"
            )

        minimum, maximum, spans = capture.summary()
        selected_arm_start = 0 if args.arm == "left" else 6
        selected_arm_indices = range(selected_arm_start, selected_arm_start + 6)
        opposite_arm_indices = range(
            6 if args.arm == "left" else 0,
            12 if args.arm == "left" else 6,
        )
        same_arm_other_spans = [
            spans[index]
            for index in selected_arm_indices
            if index != selected_index
        ]
        opposite_arm_spans = [spans[index] for index in opposite_arm_indices]
        result = {
            "schema_version": 1,
            "record_kind": "bimanual_joint_range_torque_off_observation",
            "status": "J0_CAPTURE_COMPLETE_REQUIRES_ENGINEERING_REVIEW",
            "motion_authorized": False,
            "apply_to_calibration": False,
            "automatic_limit_expansion": False,
            "torque_off_verified_by_r4_bridge_startup": True,
            "movement_source": "USER_MANUAL_WITH_BOTH_ARMS_SUPPORTED",
            "selected": {
                "arm": args.arm,
                "joint": args.joint,
                "joint_name": selected_name,
                "index": selected_index,
                "context": args.context,
                "observed_minimum_raw": minimum[selected_index],
                "observed_maximum_raw": maximum[selected_index],
                "span_raw": spans[selected_index],
                "direction_reversals": capture.selected_direction_reversals,
                "wrap_crossings": capture.selected_wrap_crossings,
                "maximum_step_raw": capture.selected_maximum_step_raw,
                "unwrapped_minimum_raw": (
                    capture.selected_unwrapped_minimum_raw
                ),
                "unwrapped_maximum_raw": (
                    capture.selected_unwrapped_maximum_raw
                ),
                "unwrapped_span_raw": (
                    capture.selected_unwrapped_maximum_raw
                    - capture.selected_unwrapped_minimum_raw
                ),
            },
            "capture": {
                "topic": TOPIC,
                "duration_s": args.duration_s,
                "sample_count": capture.samples,
                "minimum_samples": args.minimum_samples,
                "maximum_same_arm_other_joint_span_raw": max(
                    same_arm_other_spans
                ),
                "maximum_opposite_arm_joint_span_raw": max(
                    opposite_arm_spans
                ),
            },
            "joint_names": expected_names,
            "observed_minimum_raw": minimum,
            "observed_maximum_raw": maximum,
            "observed_span_raw": spans,
            "calibrations": {
                "left": {
                    "path": str(args.left_calibration),
                    "sha256": file_sha256(args.left_calibration),
                },
                "right": {
                    "path": str(args.right_calibration),
                    "sha256": file_sha256(args.right_calibration),
                },
            },
            "records": records,
            "required_review": (
                "identify the limiting mechanism/cable/collision, repeat both "
                "directions at least three times, and derive an inward margin; "
                "do not copy extrema into calibration"
            ),
        }
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(output)
        print(
            "J0_RANGE_CAPTURE_COMPLETE_REQUIRES_REVIEW "
            f"joint={selected_name} "
            f"raw={minimum[selected_index]}..{maximum[selected_index]} "
            f"span={spans[selected_index]} "
            f"reversals={capture.selected_direction_reversals} "
            f"wraps={capture.selected_wrap_crossings} "
            f"unwrapped={capture.selected_unwrapped_minimum_raw}.."
            f"{capture.selected_unwrapped_maximum_raw} "
            f"max_step={capture.selected_maximum_step_raw} "
            f"same_arm_other_span_max={max(same_arm_other_spans)} "
            f"opposite_arm_span_max={max(opposite_arm_spans)} "
            f"output={output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
