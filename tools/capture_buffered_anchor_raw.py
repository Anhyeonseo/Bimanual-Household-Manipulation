#!/usr/bin/env python3
"""
torque 유지 상태에서 /joint_states 를 읽어 buffered 계획용 anchor raw 를 만든다.

Motion-11 계획은 `--anchor-raw` 로 6축 raw 를 받는데, bridge 가 serial 을
독점하는 동안에는 servo 를 직접 읽을 수 없다. 이 도구는 bridge 가 publish 하는
`/joint_states` 를 구독해 계획 생성기와 **같은** 변환
(`plan_buffered_q0_roundtrip.radians_to_raw`)으로 raw 를 만든다.

anchor 는 반드시 torque 가 걸린 상태에서 읽어야 한다. torque OFF 로 읽으면
읽는 시점과 실행 시점 사이에 팔이 흘러내려 fresh-start 게이트가 깨진다.

읽기 전용이다. publisher, Action client, service client, serial 을 만들지 않는다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import time

from single_arm_bridge.calibration import load_calibration

from plan_buffered_q0_roundtrip import radians_to_raw


JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_CALIBRATION = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "config"
    / "single_arm_calibration.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--samples",
        type=int,
        default=15,
        help="집계할 /joint_states 표본 수",
    )
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--maximum-spread-raw",
        type=int,
        default=4,
        help="표본 간 raw 변동 허용치. 넘으면 torque 미유지로 보고 거부한다",
    )
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 3:
        raise SystemExit("--samples must be at least 3")

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    calibration = load_calibration(args.calibration)
    # /joint_states 는 ROS 이름(left_base_joint ...)을 쓰고 calibration 은
    # BASE/SHOULDER/... 를 쓴다. 순서는 두 목록이 동일하다.
    joint_names = tuple(calibration.ros_joint_names)

    rclpy.init()
    node = Node("capture_buffered_anchor_raw")
    messages: list[JointState] = []
    node.create_subscription(
        JointState, JOINT_STATES_TOPIC, messages.append, 10
    )

    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and len(messages) < args.samples:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise SystemExit(
                    f"/joint_states timed out: {len(messages)}/{args.samples} "
                    "samples. bridge 가 실행 중인지 확인한다."
                )
            rclpy.spin_once(node, timeout_sec=min(0.5, remaining))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    per_sample_raw: list[tuple[int, ...]] = []
    for message in messages[: args.samples]:
        index = {name: i for i, name in enumerate(message.name)}
        missing = [name for name in joint_names if name not in index]
        if missing:
            raise SystemExit(f"/joint_states missing joints: {missing}")
        positions = tuple(
            float(message.position[index[name]]) for name in joint_names
        )
        per_sample_raw.append(radians_to_raw(calibration, positions))

    columns = list(zip(*per_sample_raw))
    spreads = [max(column) - min(column) for column in columns]
    anchor = tuple(round(statistics.median(column)) for column in columns)

    print(f"SAMPLES={len(per_sample_raw)}")
    for name, value, spread in zip(joint_names, anchor, spreads):
        print(f"  {name:<24} raw={value:<6} spread={spread}")
    print(f"MAXIMUM_SPREAD_RAW={max(spreads)}")

    if max(spreads) > args.maximum_spread_raw:
        print("ANCHOR_GATE=FAIL")
        raise SystemExit(
            f"raw spread {max(spreads)} exceeds {args.maximum_spread_raw}. "
            "torque 가 유지되지 않아 자세가 흐르고 있다."
        )

    print("ANCHOR_GATE=PASS")
    print("ANCHOR_RAW=" + " ".join(str(value) for value in anchor))
    print(
        "PLAN_ARGUMENT=--anchor-raw "
        + " ".join(str(value) for value in anchor)
    )

    if args.output is not None:
        document = {
            "schema_version": 1,
            "kind": "buffered_anchor_raw_capture",
            "status": "STATIONARY_READ_ONLY_CAPTURE_PASS",
            "joint_names": list(joint_names),
            "anchor_raw": list(anchor),
            "per_axis_spread_raw": spreads,
            "maximum_spread_raw": max(spreads),
            "sample_count": len(per_sample_raw),
            "motion_authorized": False,
            "robot_target_available": False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"OUTPUT={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
