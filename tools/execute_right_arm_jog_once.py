#!/usr/bin/env python3
"""Send exactly one bounded R1 right-arm jog through the bridge service.

The firmware rejects every request except one ID in 1..6 and a delta in
[-20,-8] or [8,20] raw. It reads torque state but never enables torque,
changes PID/speed/limits, or writes another servo. A successful command does
not establish a calibration or authorize a second command.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any


SERVICE = "/right_arm_jog_once"
CONFIRMATION = "RIGHT_ARM_JOG_ONCE"
SERVICE_TIMEOUT_S = 5.0


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm jog service timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm jog service returned no response")
    return response


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servo-id", required=True, type=int)
    parser.add_argument("--delta-raw", required=True, type=int)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts" / "right_arm" / "jog_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")

    import rclpy
    from so101_interfaces.srv import RightArmJogOnce

    rclpy.init()
    node = rclpy.create_node("right_arm_jog_once_client")
    try:
        client = node.create_client(RightArmJogOnce, SERVICE)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {SERVICE}")
        request = RightArmJogOnce.Request()
        request.servo_id = args.servo_id
        request.delta_raw = args.delta_raw
        request.confirmation = CONFIRMATION
        response = wait_future(
            node, client.call_async(request), SERVICE_TIMEOUT_S
        )
        result = {
            "kind": "right_arm_bounded_jog_once",
            "commanded_operations": ["WRITE Goal_Position for one servo"],
            "automatic_torque_enable": False,
            "servo_id": args.servo_id,
            "delta_raw": args.delta_raw,
            "accepted": bool(response.accepted),
            "status_code": int(response.status_code),
            "torque_enabled": int(response.torque_enabled),
            "start_position_raw": int(response.start_position_raw),
            "target_position_raw": int(response.target_position_raw),
            "observed_position_raw": int(response.observed_position_raw),
            "diagnostic": response.diagnostic,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            "RIGHT_ARM_JOG_ONCE "
            f"accepted={result['accepted']} status={result['status_code']} "
            f"servo_id={result['servo_id']} delta_raw={result['delta_raw']} "
            f"start={result['start_position_raw']} "
            f"target={result['target_position_raw']} "
            f"observed={result['observed_position_raw']} "
            f"output={args.output}"
        )
        return 0 if result["accepted"] and result["status_code"] == 0 else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
