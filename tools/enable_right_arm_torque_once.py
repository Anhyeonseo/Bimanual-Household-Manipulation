#!/usr/bin/env python3
"""Enable torque for exactly one right-arm servo at its present position.

This R1.1 migration primitive reads the selected servo's position, writes it
back as Goal_Position while torque remains off, enables torque once, then
requires a torque readback. It does not configure PID, speed, limits, or any
other servo. The bridge must be in power-off-confirmed right-arm isolation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any


SERVICE = "/right_arm_torque_enable_once"
CONFIRMATION = "RIGHT_ARM_TORQUE_ENABLE_ONCE"
SERVICE_TIMEOUT_S = 5.0


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm torque-enable service timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm torque-enable service returned no response")
    return response


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servo-id", required=True, type=int)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts" / "right_arm" / "torque_enable_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")

    import rclpy
    from so101_interfaces.srv import RightArmTorqueEnableOnce

    rclpy.init()
    node = rclpy.create_node("right_arm_torque_enable_once_client")
    try:
        client = node.create_client(RightArmTorqueEnableOnce, SERVICE)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {SERVICE}")
        request = RightArmTorqueEnableOnce.Request()
        request.servo_id = args.servo_id
        request.confirmation = CONFIRMATION
        response = wait_future(
            node, client.call_async(request), SERVICE_TIMEOUT_S
        )
        result = {
            "kind": "right_arm_torque_enable_once_at_present_position",
            "commanded_operations": [
                "READ Torque_Enable",
                "READ Present_Position",
                "WRITE Goal_Position=Present_Position",
                "WRITE Torque_Enable=1 for one servo",
                "READ Torque_Enable",
                "READ Present_Position",
            ],
            "servo_id": args.servo_id,
            "accepted": bool(response.accepted),
            "status_code": int(response.status_code),
            "torque_enabled": int(response.torque_enabled),
            "present_position_raw": int(response.present_position_raw),
            "held_goal_position_raw": int(response.held_goal_position_raw),
            "observed_position_raw": int(response.observed_position_raw),
            "diagnostic": response.diagnostic,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            "RIGHT_ARM_TORQUE_ENABLE_ONCE "
            f"accepted={result['accepted']} status={result['status_code']} "
            f"servo_id={result['servo_id']} torque={result['torque_enabled']} "
            f"present={result['present_position_raw']} "
            f"held_goal={result['held_goal_position_raw']} "
            f"observed={result['observed_position_raw']} "
            f"output={args.output}"
        )
        return 0 if result["accepted"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
