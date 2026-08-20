#!/usr/bin/env python3
"""Send one fixed safe gripper goal and cancel it exactly once."""

from __future__ import annotations

import argparse
import time
from typing import Any

from action_msgs.msg import GoalStatus

from control_msgs.action import ParallelGripperCommand

import rclpy
from rclpy.action import ActionClient


ACTION_NAME = "/left_gripper_controller/gripper_cmd"
JOINT_NAME = "left_gripper_joint"
TARGET_RAD = 0.13
CANCEL_DELAY_S = 0.30


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("ROS Action response timeout")
    result = future.result()
    if result is None:
        raise RuntimeError("ROS Action future returned no result")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Execute the approved one-shot 0.13 rad gripper cancel test. "
            "This tool never retries and never clears the resulting stop latch."
        )
    )
    parser.add_argument(
        "--execute-once",
        action="store_true",
        help="required acknowledgement that exactly one physical goal will be sent",
    )
    args = parser.parse_args()
    if not args.execute_once:
        parser.error("--execute-once is required; no goal was sent")
    return args


def main() -> int:
    parse_args()
    rclpy.init()
    node = rclpy.create_node("so101_gripper_cancel_once")
    client = ActionClient(node, ParallelGripperCommand, ACTION_NAME)

    try:
        if not client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError(f"Action server unavailable: {ACTION_NAME}")

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [JOINT_NAME]
        goal.command.position = [TARGET_RAD]

        goal_handle = wait_future(
            node,
            client.send_goal_async(goal),
            timeout_s=5.0,
        )
        if not goal_handle.accepted:
            raise RuntimeError("CANCEL_TEST_GOAL_REJECTED")

        print(
            "CANCEL_TEST_GOAL_ACCEPTED "
            f"target_rad={TARGET_RAD:.2f} cancel_delay_ms={CANCEL_DELAY_S * 1000:.0f}"
        )

        cancel_at = time.monotonic() + CANCEL_DELAY_S
        while rclpy.ok() and time.monotonic() < cancel_at:
            rclpy.spin_once(node, timeout_sec=0.02)

        cancel_response = wait_future(
            node,
            goal_handle.cancel_goal_async(),
            timeout_s=3.0,
        )
        if len(cancel_response.goals_canceling) != 1:
            raise RuntimeError("CANCEL_REQUEST_REJECTED")
        print("CANCEL_REQUEST_ACCEPTED")

        wrapped_result = wait_future(
            node,
            goal_handle.get_result_async(),
            timeout_s=3.0,
        )
        result = wrapped_result.result
        position = (
            result.state.position[0]
            if result.state.position
            else float("nan")
        )
        print(
            "CANCEL_RESULT "
            f"status={wrapped_result.status} "
            f"reached_goal={int(result.reached_goal)} "
            f"stalled={int(result.stalled)} "
            f"position_rad={position:.9f}"
        )
        if wrapped_result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError("CANCEL_RESULT_NOT_CANCELED")
        if result.reached_goal:
            raise RuntimeError("CANCELED_GOAL_REPORTED_REACHED")
        print("CANCEL_TEST_PASS SAFE_STOP_EXPECTED_LATCHED")
        return 0
    except Exception as error:
        print(f"CANCEL_TEST_FAIL {error}")
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
