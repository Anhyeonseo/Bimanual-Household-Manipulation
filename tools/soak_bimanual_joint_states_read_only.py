#!/usr/bin/env python3
"""Record the R4 12-axis ROS observation without calling any motion API."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any


BIMANUAL_TOPIC = "/bimanual_joint_states"
LEFT_TOPIC = "/joint_states"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument(
        "--left-calibration",
        type=Path,
        default=root / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--right-calibration",
        type=Path,
        default=root / "config/right_arm_calibration.candidate.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/bimanual/r4_read_only_soak.json",
    )
    return parser.parse_args()


def calibration_joint_names(path: Path, expected_slot: str) -> tuple[str, ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("arm_slot") != expected_slot:
        raise ValueError(f"{path}: expected arm_slot={expected_slot}")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError(f"{path}: expected six joints")
    if [item.get("id") for item in joints] != list(range(1, 7)):
        raise ValueError(f"{path}: servo IDs must be ordered 1..6")
    return tuple(
        f"{expected_slot}_{item['name'].lower()}_joint" for item in joints
    )


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    args = parse_args()
    if args.samples < 3:
        raise SystemExit("--samples must be at least 3")
    if args.timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be positive")

    left_names = calibration_joint_names(args.left_calibration, "left")
    right_names = calibration_joint_names(args.right_calibration, "right")
    expected_bimanual_names = left_names + right_names
    if len(set(expected_bimanual_names)) != 12:
        raise RuntimeError("calibrations do not produce 12 unique joint names")

    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("r4_bimanual_joint_states_read_only_soak")
    records: list[dict[str, Any]] = []
    left_samples = 0
    validation_error: Exception | None = None

    def on_bimanual(message: JointState) -> None:
        nonlocal validation_error
        if validation_error is not None or len(records) >= args.samples:
            return
        try:
            names = tuple(message.name)
            positions = tuple(float(value) for value in message.position)
            if names != expected_bimanual_names:
                raise ValueError(f"unexpected bimanual joint names: {names}")
            if len(positions) != 12 or not all(map(math.isfinite, positions)):
                raise ValueError("bimanual positions must be 12 finite radians")
            records.append(
                {
                    "sample": len(records) + 1,
                    "stamp_sec": int(message.header.stamp.sec),
                    "stamp_nanosec": int(message.header.stamp.nanosec),
                    "positions_rad": positions,
                }
            )
            if len(records) == 1 or len(records) % 10 == 0:
                print(
                    "R4_BIMANUAL_SAMPLE "
                    f"count={len(records)}/{args.samples} joints=12"
                )
        except Exception as error:
            validation_error = error

    def on_left(message: JointState) -> None:
        nonlocal left_samples, validation_error
        if validation_error is not None:
            return
        try:
            names = tuple(message.name)
            positions = tuple(float(value) for value in message.position)
            if names != left_names:
                raise ValueError(f"legacy /joint_states changed identity: {names}")
            if len(positions) != 6 or not all(map(math.isfinite, positions)):
                raise ValueError("legacy /joint_states must contain six finite values")
            left_samples += 1
        except Exception as error:
            validation_error = error

    subscriptions = (
        node.create_subscription(JointState, BIMANUAL_TOPIC, on_bimanual, 10),
        node.create_subscription(JointState, LEFT_TOPIC, on_left, 10),
    )
    started = time.monotonic()
    try:
        while (
            rclpy.ok()
            and validation_error is None
            and len(records) < args.samples
            and time.monotonic() - started < args.timeout_s
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        if validation_error is not None:
            raise RuntimeError(f"R4 topic validation failed: {validation_error}")
        if len(records) != args.samples:
            raise TimeoutError(
                f"received {len(records)}/{args.samples} bimanual samples "
                f"within {args.timeout_s:.1f}s"
            )
        if left_samples == 0:
            raise RuntimeError("legacy /joint_states produced no sample")

        per_joint = tuple(
            tuple(record["positions_rad"][index] for record in records)
            for index in range(12)
        )
        result = {
            "schema_version": 1,
            "record_kind": "r4_bimanual_joint_states_read_only_soak",
            "overall_verdict": "R4_BIMANUAL_READ_ONLY_SOAK_PASS",
            "motion_authorized": False,
            "hardware_synchronous": False,
            "sampling_note": "left and right UART buses are read sequentially",
            "torque_off_verified_by_bridge_startup": True,
            "topics": {
                "bimanual": BIMANUAL_TOPIC,
                "legacy_left": LEFT_TOPIC,
            },
            "joint_names": expected_bimanual_names,
            "samples_requested": args.samples,
            "samples_received": len(records),
            "legacy_left_samples_received": left_samples,
            "position_min_rad": [min(values) for values in per_joint],
            "position_max_rad": [max(values) for values in per_joint],
            "position_span_rad": [max(values) - min(values) for values in per_joint],
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
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(args.output)
        print(
            "R4_BIMANUAL_READ_ONLY_SOAK_PASS "
            f"samples={len(records)} left_samples={left_samples} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
