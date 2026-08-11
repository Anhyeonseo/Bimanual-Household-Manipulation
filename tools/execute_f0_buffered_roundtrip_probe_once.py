#!/usr/bin/env python3
"""Execute one small, reversible buffered F0 timing probe through ROS Action.

The probe returns to the captured pose and uses the normal bridge Action path,
so the bridge keeps sending heartbeats after this client exits. It never sends
DISABLE or retries a goal.  The optional H2.0 gate additionally requires a
complete, failure-free position-only in-motion telemetry terminal snapshot.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import time
from typing import Any

from single_arm_bridge.calibration import load_calibration


ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
CONFIRMATION = "EXECUTE_F0_BUFFERED_ROUNDTRIP_PROBE_ONCE"
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 10.0
STATE_TIMEOUT_S = 5.0
DURATION_MS = 1_200
HALF_DURATION_MS = DURATION_MS // 2
MAXIMUM_ABSOLUTE_DELTA_RAD = 0.03
MINIMUM_ABSOLUTE_DELTA_RAD = 0.02
PREDICTED_SAMPLES = (DURATION_MS // 20) + 1
ACTION_STATUS_SUCCEEDED = 4
FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL = 0


@dataclass(frozen=True, slots=True)
class PlanWaypoint:
    time_from_start_ms: int
    positions_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class ProbePlan:
    arm_joint_names: tuple[str, ...]
    waypoints: tuple[PlanWaypoint, ...]
    sample_count: int


def wait_joint_state(
    node: Any, expected_names: tuple[str, ...], timeout_s: float
) -> tuple[float, ...]:
    import rclpy
    from sensor_msgs.msg import JointState

    messages: list[JointState] = []
    subscription = node.create_subscription(
        JointState, "/joint_states", messages.append, 10
    )
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and not messages:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("joint state timed out")
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        names = messages[-1].name
        positions = messages[-1].position
        if len(names) != len(positions) or len(names) != len(set(names)):
            raise ValueError("joint state names and positions are invalid")
        by_name = dict(zip(names, positions, strict=True))
        missing = [name for name in expected_names if name not in by_name]
        if missing:
            raise ValueError(f"joint state is missing {','.join(missing)}")
        result = tuple(float(by_name[name]) for name in expected_names)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("joint state contains a non-finite position")
        return result
    finally:
        node.destroy_subscription(subscription)


def build_goal(plan: ProbePlan) -> Any:
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(plan.arm_joint_names)
    for waypoint in plan.waypoints:
        point = JointTrajectoryPoint()
        point.positions = list(waypoint.positions_rad)
        point.time_from_start.sec = waypoint.time_from_start_ms // 1000
        point.time_from_start.nanosec = (
            waypoint.time_from_start_ms % 1000
        ) * 1_000_000
        goal.trajectory.points.append(point)
    return goal


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("ROS future timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    result = future.result()
    if result is None:
        raise RuntimeError("ROS future completed without a result")
    return result


def send_goal_once(node: Any, action_client: Any, goal: Any) -> tuple[int, Any]:
    goal_handle = wait_future(
        node, action_client.send_goal_async(goal), ACTION_SERVER_TIMEOUT_S
    )
    if not goal_handle.accepted:
        raise RuntimeError("buffered Action goal was rejected")
    response = wait_future(
        node, goal_handle.get_result_async(), ACTION_RESULT_TIMEOUT_S
    )
    return response.status, response.result


def build_probe_plan(
    calibration_path: Path,
    current_arm_rad: tuple[float, ...],
    joint_name: str,
    delta_rad: float,
) -> ProbePlan:
    calibration = load_calibration(calibration_path)
    arm_names = tuple(calibration.ros_joint_names[:5])
    if len(current_arm_rad) != len(arm_names):
        raise ValueError("current arm pose must contain five joints")
    if joint_name not in arm_names:
        raise ValueError("joint must be one of the five arm joints")
    if (
        not math.isfinite(delta_rad)
        or not MINIMUM_ABSOLUTE_DELTA_RAD <= abs(delta_rad) <= MAXIMUM_ABSOLUTE_DELTA_RAD
    ):
        raise ValueError(
            "absolute delta must be within "
            f"{MINIMUM_ABSOLUTE_DELTA_RAD:.3f}..{MAXIMUM_ABSOLUTE_DELTA_RAD:.3f} rad"
        )
    target = list(current_arm_rad)
    index = arm_names.index(joint_name)
    target[index] += delta_rad
    lower, upper = calibration.ros_radian_limits[joint_name]
    if not lower <= target[index] <= upper:
        raise ValueError(
            f"{joint_name} target {target[index]:.6f} is outside {lower:.6f}..{upper:.6f}"
        )
    target_tuple = tuple(target)
    return ProbePlan(
        arm_joint_names=arm_names,
        waypoints=(
            PlanWaypoint(0, current_arm_rad),
            PlanWaypoint(HALF_DURATION_MS, target_tuple),
            PlanWaypoint(DURATION_MS, current_arm_rad),
        ),
        sample_count=PREDICTED_SAMPLES,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--calibration",
        type=Path,
        default=(
            root / "ros2_ws" / "src" / "single_arm_bridge" / "config"
            / "single_arm_calibration.json"
        ),
    )
    parser.add_argument("--joint", required=True)
    parser.add_argument("--delta-rad", type=float, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--require-h2-telemetry",
        action="store_true",
        help="fail unless the terminal includes completed H2.0 telemetry with no failures",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error("exact F0 round-trip probe confirmation is required")

    from control_msgs.action import FollowJointTrajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    calibration = load_calibration(args.calibration)
    all_names = tuple(calibration.ros_joint_names)
    rclpy.init()
    node = Node("f0_buffered_roundtrip_probe_once")
    action_client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
    try:
        current_all = wait_joint_state(node, all_names, STATE_TIMEOUT_S)
        current_arm = tuple(current_all[:5])
        plan = build_probe_plan(
            args.calibration, current_arm, args.joint, args.delta_rad
        )
        print(
            "F0_PROBE_PRECHECK_PASS "
            f"joint={args.joint} delta_rad={args.delta_rad:.6f} "
            f"duration_ms={DURATION_MS} predicted_samples={plan.sample_count}"
        )
        if not action_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_S):
            raise RuntimeError("FollowJointTrajectory Action is unavailable")
        print("ACTION_SEND_COUNT=1")
        status, result = send_goal_once(node, action_client, build_goal(plan))
        diagnostic = str(result.error_string)
        if (
            status != ACTION_STATUS_SUCCEEDED
            or result.error_code != FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL
        ):
            raise RuntimeError(
                f"Action terminal failed: status={status} error_code={result.error_code} "
                f"diagnostic={diagnostic}"
            )
        if "f0_loop_period_max_us=" not in diagnostic:
            raise RuntimeError("terminal did not include F0 metrics")
        if args.require_h2_telemetry:
            required_h2_fragments = (
                "h2_tracking_error_max_raw=",
                "h2_telemetry_requested=",
                "h2_telemetry_completed=",
                "h2_telemetry_failed=0",
                "h2_telemetry_reply_latency_max_ms=",
            )
            missing_h2 = [
                fragment
                for fragment in required_h2_fragments
                if fragment not in diagnostic
            ]
            if missing_h2:
                raise RuntimeError(
                    "terminal did not pass H2 telemetry gate: "
                    + ",".join(missing_h2)
                )
            h2_counts = {
                name: int(match.group(1))
                for name, match in (
                    (
                        "requested",
                        re.search(r"h2_telemetry_requested=(\d+)", diagnostic),
                    ),
                    (
                        "completed",
                        re.search(r"h2_telemetry_completed=(\d+)", diagnostic),
                    ),
                    (
                        "failed",
                        re.search(r"h2_telemetry_failed=(\d+)", diagnostic),
                    ),
                )
                if match is not None
            }
            if (
                h2_counts.get("requested", 0) == 0
                or h2_counts.get("completed") != h2_counts.get("requested")
                or h2_counts.get("failed") != 0
            ):
                raise RuntimeError(
                    "terminal did not complete H2 telemetry: "
                    f"{h2_counts}"
                )
        returned = wait_joint_state(node, all_names, STATE_TIMEOUT_S)
        return_error = max(
            abs(actual - expected)
            for actual, expected in zip(returned[:5], current_arm, strict=True)
        )
        print(f"F0_PROBE_RETURN_ERROR_RAD={return_error:.6f}")
        print(f"F0_TERMINAL_DIAGNOSTICS={diagnostic}")
        print("F0_BUFFERED_ROUNDTRIP_PROBE_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
