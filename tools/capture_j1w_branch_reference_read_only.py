#!/usr/bin/env python3
"""Capture a stable R4 raw snapshot for explicit J1-W branch binding."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time


CONFIRMATION = "CAPTURE_J1W_BRANCH_REFERENCE_TORQUE_OFF"
TOPIC = "/bimanual_joint_states"
RAW_UNITS_PER_TURN = 4096
SHOULDER_SAFE_BRANCH_MINIMUM_RAW = 1024
SHOULDER_SAFE_BRANCH_MAXIMUM_RAW = 3072
MAXIMUM_STATIONARY_SPAN_RAW = 4


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=15.0)
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


def load_joints(path: Path, arm_slot: str) -> tuple[dict, ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("arm_slot") != arm_slot:
        raise ValueError(f"{path}: arm_slot must be {arm_slot}")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError(f"{path}: expected six joints")
    return tuple(joints)


def joint_names(
    left: tuple[dict, ...], right: tuple[dict, ...]
) -> tuple[str, ...]:
    return tuple(
        f"{arm}_{joint['name'].lower()}_joint"
        for arm, joints in (("left", left), ("right", right))
        for joint in joints
    )


def positions_to_raw(
    positions: tuple[float, ...], joints: tuple[dict, ...]
) -> tuple[int, ...]:
    if len(positions) != 12 or len(joints) != 12:
        raise ValueError("expected 12 positions and calibrations")
    result: list[int] = []
    for position, joint in zip(positions, joints, strict=True):
        if not math.isfinite(position):
            raise ValueError("non-finite joint feedback")
        raw = round(
            joint["zero_raw"]
            + joint["positive_raw_direction"]
            * position
            * RAW_UNITS_PER_TURN
            / (2.0 * math.pi)
        )
        if not 0 <= raw < RAW_UNITS_PER_TURN:
            raise ValueError(f"feedback reconstructs outside raw range: {raw}")
        result.append(raw)
    return tuple(result)


def validate_stable_reference(records: list[tuple[int, ...]]) -> tuple[int, ...]:
    if not records:
        raise ValueError("no feedback samples")
    minima = tuple(min(record[index] for record in records) for index in range(12))
    maxima = tuple(max(record[index] for record in records) for index in range(12))
    spans = tuple(high - low for low, high in zip(minima, maxima, strict=True))
    if max(spans) > MAXIMUM_STATIONARY_SPAN_RAW:
        raise ValueError(f"arm moved during branch capture: spans={spans}")
    reference = records[-1]
    for index in (1, 7):
        if not (
            SHOULDER_SAFE_BRANCH_MINIMUM_RAW
            <= reference[index]
            <= SHOULDER_SAFE_BRANCH_MAXIMUM_RAW
        ):
            raise ValueError(
                "shoulder raw is too close to an ambiguous wrap branch: "
                f"index={index} raw={reference[index]}"
            )
    return reference


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("confirmation mismatch")
    if not 3 <= args.samples <= 100:
        raise SystemExit("--samples must be within 3..100")
    if not 3.0 <= args.timeout_s <= 60.0:
        raise SystemExit("--timeout-s must be within 3..60")

    left = load_joints(args.left_calibration, "left")
    right = load_joints(args.right_calibration, "right")
    joints = left + right
    expected_names = joint_names(left, right)

    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("j1w_branch_reference_read_only_capture")
    records: list[tuple[int, ...]] = []
    error: Exception | None = None

    def callback(message: JointState) -> None:
        nonlocal error
        if error is not None or len(records) >= args.samples:
            return
        try:
            if tuple(message.name) != expected_names:
                raise ValueError("unexpected bimanual joint identity/order")
            records.append(positions_to_raw(tuple(message.position), joints))
            print(
                f"J1W_REFERENCE_SAMPLE count={len(records)}/{args.samples} "
                f"raw={list(records[-1])}",
                flush=True,
            )
        except Exception as caught:
            error = caught

    subscription = node.create_subscription(JointState, TOPIC, callback, 10)
    del subscription
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and len(records) < args.samples and error is None:
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        if error is not None:
            raise RuntimeError(f"branch capture failed: {error}")
        if len(records) != args.samples:
            raise RuntimeError(
                f"only {len(records)}/{args.samples} samples arrived"
            )
        reference = validate_stable_reference(records)
        document = {
            "schema_version": 1,
            "record_kind": "j1w_explicit_branch_reference",
            "status": "J1W_BRANCH_REFERENCE_CAPTURE_PASS",
            "motion_authorized": False,
            "torque_expected": "OFF_VERIFIED_BY_R4_STARTUP",
            "source_topic": TOPIC,
            "joint_names": expected_names,
            "reference_unwrapped_raw": reference,
            "reference_semantics": (
                "shoulders are inside the non-ambiguous commissioning branch; "
                "all other axes use their stationary raw value"
            ),
            "sample_count": len(records),
            "maximum_stationary_span_raw": MAXIMUM_STATIONARY_SPAN_RAW,
            "records_raw": records,
            "calibrations": {
                "left_sha256": file_sha256(args.left_calibration),
                "right_sha256": file_sha256(args.right_calibration),
            },
        }
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(output)
        print(
            "J1W_BRANCH_REFERENCE_CAPTURE_PASS "
            f"reference={list(reference)} output={output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
