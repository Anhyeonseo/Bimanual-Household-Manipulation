#!/usr/bin/env python3
"""Validate the exact J2-B bridge/firmware contract with no motion command."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


CONFIRMATION = "J2_B_VALIDATE_NO_MOTION"
IDENTITY_SERVICE = "/right_arm_j2_base_limits_identity"
DISABLE_SERVICE = "/right_arm_disable"
CONFIGURATION_SERVICE = "/get_right_arm_configuration"
SERVICE_TIMEOUT_S = 5.0
EXPECTED_FIRMWARE = "0x00024200"
EXPECTED_CAPABILITY = 0x10000000
EXPECTED_MANIFEST_SHA256 = (
    "dfbfaf6c7138fab30afebc1f3e69c7d53edb01060bd349f65c6f048f150dff34"
)
EXPECTED_STATUS = (
    "J2_B_BASE_LIMIT_CANDIDATE_AWAITING_NO_MOTION_AND_ACTIVE_VALIDATION"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_result(path: Path, document: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return sha256(path)


def wait_future(node: Any, future: Any) -> Any:
    import rclpy

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=SERVICE_TIMEOUT_S,
    )
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}")
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--left-arm-12v-off-confirmed", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")
    if not args.left_arm_12v_off_confirmed:
        raise SystemExit("--left-arm-12v-off-confirmed is required")

    import rclpy
    from so101_interfaces.srv import RightArmConfiguration
    from std_srvs.srv import Trigger

    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "right_arm_j2b_no_motion_validation",
        "overall_verdict": "IN_PROGRESS",
        "motion_authorized": False,
        "motion_commands_sent": 0,
        "torque_enable_requests_sent": 0,
        "identity": None,
        "verified_disable": None,
        "configuration": [],
    }

    rclpy.init()
    node = rclpy.create_node("right_arm_j2b_no_motion_validator")
    identity_client = node.create_client(Trigger, IDENTITY_SERVICE)
    disable_client = node.create_client(Trigger, DISABLE_SERVICE)
    configuration_client = node.create_client(
        RightArmConfiguration,
        CONFIGURATION_SERVICE,
    )
    clients = (
        (IDENTITY_SERVICE, identity_client),
        (DISABLE_SERVICE, disable_client),
        (CONFIGURATION_SERVICE, configuration_client),
    )

    try:
        for service_name, client in clients:
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
                raise RuntimeError(f"service unavailable: {service_name}")

        identity_response = wait_future(
            node,
            identity_client.call_async(Trigger.Request()),
        )
        if not identity_response.success:
            raise RuntimeError(f"identity rejected: {identity_response.message}")
        identity = json.loads(identity_response.message)
        if (
            identity.get("firmware_version") != EXPECTED_FIRMWARE
            or (
                int(identity.get("capabilities", "0"), 16)
                & EXPECTED_CAPABILITY
            )
            == 0
            or identity.get("manifest_sha256") != EXPECTED_MANIFEST_SHA256
            or identity.get("status") != EXPECTED_STATUS
        ):
            raise RuntimeError("J2-B identity contract mismatch")
        document["identity"] = identity

        disable_response = wait_future(
            node,
            disable_client.call_async(Trigger.Request()),
        )
        document["verified_disable"] = {
            "success": bool(disable_response.success),
            "diagnostic": disable_response.message,
        }
        if not disable_response.success:
            raise RuntimeError(
                f"verified disable failed: {disable_response.message}"
            )

        for servo_id in range(1, 7):
            request = RightArmConfiguration.Request()
            request.servo_id = servo_id
            response = wait_future(
                node,
                configuration_client.call_async(request),
            )
            sample = {
                "servo_id": servo_id,
                "success": bool(response.success),
                "status_code": int(response.status_code),
                "read_status": int(response.read_status),
                "successful_block_mask": int(response.successful_block_mask),
                "torque_enabled": int(response.torque_enabled),
                "position_raw": int(response.position_raw),
                "voltage_raw": int(response.voltage_raw),
                "temperature_c": int(response.temperature_c),
            }
            document["configuration"].append(sample)
            if (
                not sample["success"]
                or sample["status_code"] != 0
                or sample["read_status"] != 0
                or sample["successful_block_mask"] != 0x1F
                or sample["torque_enabled"] != 0
                or not 90 <= sample["voltage_raw"] <= 140
                or sample["temperature_c"] > 70
            ):
                raise RuntimeError(
                    f"configuration/health check failed for servo {servo_id}"
                )

        document["overall_verdict"] = "PASS"
        digest = write_result(args.output, document)
        print(
            "J2_B_NO_MOTION_PASS firmware=0x00024200 "
            "torque_mask=0x00 motion_commands=0 "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 0
    except Exception as error:
        document["overall_verdict"] = "FAIL"
        document["failure"] = repr(error)
        try:
            response = wait_future(
                node,
                disable_client.call_async(Trigger.Request()),
            )
            document["failure_disable"] = {
                "success": bool(response.success),
                "diagnostic": response.message,
            }
        except Exception as disable_error:
            document["failure_disable"] = {
                "success": False,
                "diagnostic": repr(disable_error),
            }
        digest = write_result(args.output, document)
        print(
            f"J2_B_NO_MOTION_FAIL error={error} "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
