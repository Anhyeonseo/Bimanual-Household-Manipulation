#!/usr/bin/env python3
"""Apply the reviewed left-arm operational settings to right IDs 1..6 torque-off."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from typing import Any

SERVICE = "/right_arm_configure_once"
CONFIRMATION = "RIGHT_ARM_CONFIGURE_ONCE"
SERVICE_TIMEOUT_S = 5.0
EXPECTED_GOAL_SPEED_RAW = 800
EXPECTED_TORQUE_LIMITS = (400, 900, 800, 400, 250, 150)


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm configure-once service timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm configure-once returned no response")
    return response


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--candidate",
        type=Path,
        default=root / "config" / "right_arm_calibration.candidate.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/right_arm/right_arm_r2_1_configure_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")

    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    joints = {int(joint["id"]): joint for joint in candidate["joints"]}
    if set(joints) != set(range(1, 7)):
        raise RuntimeError("candidate must contain exactly servo IDs 1..6")

    import rclpy
    from so101_interfaces.srv import RightArmConfigureOnce

    print("R2_1_CLIENT_INIT", flush=True)
    rclpy.init()
    node = rclpy.create_node("right_arm_configure_once_client")
    results: list[dict[str, Any]] = []
    try:
        client = node.create_client(RightArmConfigureOnce, SERVICE)
        print(f"R2_1_WAITING_FOR_SERVICE service={SERVICE}", flush=True)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {SERVICE}")
        print("R2_1_SERVICE_READY", flush=True)
        for servo_id in range(1, 7):
            request = RightArmConfigureOnce.Request()
            request.servo_id = servo_id
            request.confirmation = CONFIRMATION
            print(f"R2_1_REQUEST_SEND servo_id={servo_id}", flush=True)
            response = wait_future(
                node, client.call_async(request), SERVICE_TIMEOUT_S
            )
            joint = joints[servo_id]
            checks = {
                "accepted": bool(response.accepted),
                "status": int(response.status_code) == 0,
                "torque_disabled": int(response.torque_enabled) == 0,
                "p_gain": int(response.p_gain) == int(joint["p_gain"]),
                "d_gain": int(response.d_gain) == int(joint["d_gain"]),
                "i_gain": int(response.i_gain) == 0,
                "operating_mode": int(response.operating_mode) == 0,
                "goal_speed": (
                    int(response.goal_speed_raw) == EXPECTED_GOAL_SPEED_RAW
                ),
                "torque_limit": (
                    int(response.torque_limit_raw)
                    == EXPECTED_TORQUE_LIMITS[servo_id - 1]
                ),
            }
            record = {
                "servo_id": servo_id,
                "joint_name": joint["name"],
                "accepted": bool(response.accepted),
                "status_code": int(response.status_code),
                "torque_enabled": int(response.torque_enabled),
                "pid": [int(response.p_gain), int(response.d_gain), int(response.i_gain)],
                "operating_mode": int(response.operating_mode),
                "present_position_raw": int(response.present_position_raw),
                "goal_position_raw": int(response.goal_position_raw),
                "goal_speed_raw": int(response.goal_speed_raw),
                "torque_limit_raw": int(response.torque_limit_raw),
                "diagnostic": response.diagnostic,
                "checks": checks,
                "verdict": "PASS" if all(checks.values()) else "FAIL",
            }
            results.append(record)
            print(
                f"R2_1_CONFIGURE servo_id={servo_id} verdict={record['verdict']} "
                f"torque={record['torque_enabled']} "
                f"present={record['present_position_raw']} goal={record['goal_position_raw']} "
                f"pid={'/'.join(map(str, record['pid']))} speed={record['goal_speed_raw']} "
                f"torque_limit={record['torque_limit_raw']}", flush=True
            )
            if record["verdict"] != "PASS":
                break
            rclpy.spin_once(node, timeout_sec=0.12)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    passed = len(results) == 6 and all(
        result["verdict"] == "PASS" for result in results
    )
    document = {
        "schema_version": 1,
        "record_kind": "right_arm_r2_1_configure_from_left_once",
        "torque_enabled_by_tool": False,
        "automatic_motion": False,
        "persistent_eeprom_write": False,
        "results": results,
        "overall_verdict": (
            "R2_1_CONFIGURE_PASS" if passed else "R2_1_CONFIGURE_FAIL"
        ),
        "motion_authorized": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2) + "\n", encoding="utf-8"
    )
    print(
        f"{document['overall_verdict']} output={args.output} "
        f"sha256={sha256(args.output)}",
        flush=True,
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
