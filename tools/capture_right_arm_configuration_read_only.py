#!/usr/bin/env python3
"""Capture and gate one read-only R2 configuration snapshot for all right servos."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from typing import Any


SERVICE = "/get_right_arm_configuration"
SERVICE_TIMEOUT_S = 5.0
EXPECTED_MODEL_NUMBER = 777
EXPECTED_I_GAIN = 0
EXPECTED_OPERATING_MODE = 0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm configuration service timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm configuration service returned no response")
    return response


def response_record(response: Any) -> dict[str, Any]:
    fields = (
        "status_code",
        "read_status",
        "successful_block_mask",
        "sample_time_ms",
        "torque_enabled",
        "p_gain",
        "d_gain",
        "i_gain",
        "voltage_raw",
        "temperature_c",
        "position_raw",
        "speed_raw",
        "load_raw",
        "current_raw",
        "runtime_torque_limit_raw",
        "goal_position_raw",
        "model_number",
        "firmware_major_version",
        "firmware_minor_version",
        "maximum_torque_limit_raw",
        "minimum_startup_force_raw",
        "cw_dead_zone_raw",
        "ccw_dead_zone_raw",
        "protection_current_raw",
        "operating_mode",
        "protective_torque_raw",
        "protection_time_raw",
        "overload_torque_raw",
    )
    record = {field: int(getattr(response, field)) for field in fields}
    record["service_success"] = bool(response.success)
    record["diagnostic"] = response.diagnostic
    return record


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        type=Path,
        default=root / "config" / "right_arm_calibration.candidate.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root / "artifacts" / "right_arm" / "right_arm_configuration.json"
        ),
    )
    args = parser.parse_args()

    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    expected = {int(joint["id"]): joint for joint in candidate["joints"]}
    if set(expected) != set(range(1, 7)):
        raise RuntimeError("candidate must contain exactly servo IDs 1..6")

    import rclpy
    from so101_interfaces.srv import RightArmConfiguration

    rclpy.init()
    node = rclpy.create_node("right_arm_configuration_read_only_client")
    snapshots: list[dict[str, Any]] = []
    try:
        client = node.create_client(RightArmConfiguration, SERVICE)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {SERVICE}")
        for servo_id in range(1, 7):
            request = RightArmConfiguration.Request()
            request.servo_id = servo_id
            response = wait_future(
                node, client.call_async(request), SERVICE_TIMEOUT_S
            )
            record = response_record(response)
            joint = expected[servo_id]
            checks = {
                "service_and_all_reads": (
                    record["service_success"]
                    and record["status_code"] == 0
                    and record["read_status"] == 0
                    and record["successful_block_mask"] == 0x1F
                ),
                "torque_disabled": record["torque_enabled"] == 0,
                "model_number": record["model_number"] == EXPECTED_MODEL_NUMBER,
                "p_gain": record["p_gain"] == int(joint["p_gain"]),
                "d_gain": record["d_gain"] == int(joint["d_gain"]),
                "i_gain": record["i_gain"] == EXPECTED_I_GAIN,
                "operating_mode": (
                    record["operating_mode"] == EXPECTED_OPERATING_MODE
                ),
            }
            record.update(
                {
                    "servo_id": servo_id,
                    "joint_name": joint["name"],
                    "checks": checks,
                    "candidate_match": all(checks.values()),
                }
            )
            snapshots.append(record)
            failed = ",".join(
                name for name, passed in checks.items() if not passed
            ) or "none"
            print(
                "R2_SERVO_CONFIGURATION "
                f"servo_id={servo_id} mask=0x{record['successful_block_mask']:02X} "
                f"torque={record['torque_enabled']} model={record['model_number']} "
                f"pid={record['p_gain']}/{record['d_gain']}/{record['i_gain']} "
                f"mode={record['operating_mode']} failed_checks={failed}"
            )
            # Give the bridge heartbeat timer an opportunity between servo IDs.
            rclpy.spin_once(node, timeout_sec=0.12)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    overall = all(snapshot["candidate_match"] for snapshot in snapshots)
    result = {
        "schema_version": 1,
        "record_kind": "right_arm_r2_configuration_read_only",
        "read_only": True,
        "commanded_operations": [
            "READ 0:5",
            "READ 16:14",
            "READ 33:4",
            "READ 40:10",
            "READ 56:15",
        ],
        "write_operations": [],
        "candidate": {
            "path": str(args.candidate),
            "sha256": sha256(args.candidate),
        },
        "snapshots": snapshots,
        "overall_verdict": (
            "R2_CONFIGURATION_READ_ONLY_PASS"
            if overall
            else "R2_CONFIGURATION_READ_ONLY_MISMATCH"
        ),
        "motion_authorized": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{result['overall_verdict']} samples={len(snapshots)} "
        f"writes=0 output={args.output} sha256={sha256(args.output)}"
    )
    return 0 if overall else 2


if __name__ == "__main__":
    raise SystemExit(main())

