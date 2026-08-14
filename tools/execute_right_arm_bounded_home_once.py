#!/usr/bin/env python3
"""Move the isolated right arm to its verified raw-2048 q0 in bounded steps.

This R3 gate composes only previously validated primitives.  It reapplies the
reviewed volatile configuration while torque is off, enables each servo at its
fresh present position, then advances all unfinished joints toward q0 using
one-servo jogs no larger than 20 raw.  Every cycle performs independent
configuration/position/torque readback.  Any failure requests a latched
right-bus SAFE_STOP before the tool exits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from typing import Any


CONFIRMATION = "RIGHT_ARM_BOUNDED_HOME_ONCE"
CONFIGURE_CONFIRMATION = "RIGHT_ARM_CONFIGURE_ONCE"
TORQUE_CONFIRMATION = "RIGHT_ARM_TORQUE_ENABLE_ONCE"
JOG_CONFIRMATION = "RIGHT_ARM_JOG_ONCE"
CONFIGURE_SERVICE = "/right_arm_configure_once"
CONFIGURATION_SERVICE = "/get_right_arm_configuration"
TORQUE_SERVICE = "/right_arm_torque_enable_once"
JOG_SERVICE = "/right_arm_jog_once"
DISABLE_SERVICE = "/right_arm_disable"
STOP_SERVICE = "/right_arm_stop"
SERVICE_TIMEOUT_S = 5.0
MINIMUM_STEP_RAW = 8
MAXIMUM_STEP_RAW = 20
HOME_TOLERANCE_RAW = 10
SETTLE_S = 0.20
MAX_CYCLES = 120
STALL_CYCLE_LIMIT = 5
EXPECTED_GOAL_SPEED_RAW = 800
EXPECTED_TORQUE_LIMITS = (400, 900, 800, 400, 250, 150)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm R3 service call timed out")
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm R3 service returned no response")
    return response


def bounded_step_raw(
    position_raw: int,
    target_raw: int,
    tolerance_raw: int = HOME_TOLERANCE_RAW,
) -> int:
    """Return one legal jog that moves strictly closer to target."""
    remaining = target_raw - position_raw
    magnitude = abs(remaining)
    if magnitude <= tolerance_raw:
        return 0
    if magnitude <= MAXIMUM_STEP_RAW:
        step = magnitude
    else:
        step = MAXIMUM_STEP_RAW
        tail = magnitude - step
        if 0 < tail < MINIMUM_STEP_RAW:
            step = magnitude - MINIMUM_STEP_RAW
    if not MINIMUM_STEP_RAW <= step <= MAXIMUM_STEP_RAW:
        raise ValueError(
            f"cannot form legal bounded step for remaining={remaining}"
        )
    return step if remaining > 0 else -step


def configuration_checks(
    response: Any,
    joint: dict[str, Any],
    torque_enabled: int,
) -> dict[str, bool]:
    servo_id = int(joint["id"])
    return {
        "success": bool(response.success),
        "status": int(response.status_code) == 0,
        "read_status": int(response.read_status) == 0,
        "all_blocks": int(response.successful_block_mask) == 0x1F,
        "torque": int(response.torque_enabled) == torque_enabled,
        "model": int(response.model_number) == 777,
        "p_gain": int(response.p_gain) == int(joint["p_gain"]),
        "d_gain": int(response.d_gain) == int(joint["d_gain"]),
        "i_gain": int(response.i_gain) == 0,
        "mode": int(response.operating_mode) == 0,
        "torque_limit": (
            int(response.runtime_torque_limit_raw)
            == EXPECTED_TORQUE_LIMITS[servo_id - 1]
        ),
    }


def write_result(path: Path, document: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return sha256(path)


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
        "--q0",
        type=Path,
        default=root / "config" / "right_arm_q0.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/right_arm/right_arm_r3_bounded_home.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")

    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    q0 = json.loads(args.q0.read_text(encoding="utf-8"))
    joints = {int(item["id"]): item for item in candidate["joints"]}
    if set(joints) != set(range(1, 7)):
        raise RuntimeError("candidate must contain exactly servo IDs 1..6")
    if q0.get("servo_ids") != list(range(1, 7)):
        raise RuntimeError("q0 record must contain ordered servo IDs 1..6")
    target_raw = int(q0["target_raw"])
    if q0.get("raw_after_power_cycle") != [target_raw] * 6:
        raise RuntimeError("q0 record does not prove six equal raw targets")
    if int(q0.get("settle_tolerance_raw", -1)) != HOME_TOLERANCE_RAW:
        raise RuntimeError(
            "q0 settle tolerance does not match the reviewed R3 gate"
        )
    for joint in joints.values():
        if not int(joint["minimum_raw"]) <= target_raw <= int(joint["maximum_raw"]):
            raise RuntimeError(
                f"q0 target is outside candidate range for ID {joint['id']}"
            )

    import rclpy
    from so101_interfaces.srv import (
        RightArmConfigureOnce,
        RightArmConfiguration,
        RightArmJogOnce,
        RightArmTorqueEnableOnce,
    )
    from std_srvs.srv import Trigger

    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "right_arm_r3_bounded_home_once",
        "target_raw": target_raw,
        "tolerance_raw": HOME_TOLERANCE_RAW,
        "maximum_command_step_raw": MAXIMUM_STEP_RAW,
        "candidate": str(args.candidate),
        "q0": str(args.q0),
        "configuration": [],
        "preflight": [],
        "torque_enable": [],
        "cycles": [],
        "stop_requested": False,
        "general_trajectory_authorized": False,
    }

    print(
        "R3_BOUNDED_HOME_PRECHECK_PASS "
        f"target_raw={target_raw} tolerance_raw={HOME_TOLERANCE_RAW} "
        f"max_step_raw={MAXIMUM_STEP_RAW}",
        flush=True,
    )
    rclpy.init()
    node = rclpy.create_node("right_arm_r3_bounded_home_client")
    configure_client = node.create_client(
        RightArmConfigureOnce, CONFIGURE_SERVICE
    )
    configuration_client = node.create_client(
        RightArmConfiguration, CONFIGURATION_SERVICE
    )
    torque_client = node.create_client(
        RightArmTorqueEnableOnce, TORQUE_SERVICE
    )
    jog_client = node.create_client(RightArmJogOnce, JOG_SERVICE)
    disable_client = node.create_client(Trigger, DISABLE_SERVICE)
    stop_client = node.create_client(Trigger, STOP_SERVICE)

    clients = (
        (CONFIGURE_SERVICE, configure_client),
        (CONFIGURATION_SERVICE, configuration_client),
        (TORQUE_SERVICE, torque_client),
        (JOG_SERVICE, jog_client),
        (DISABLE_SERVICE, disable_client),
        (STOP_SERVICE, stop_client),
    )

    def call_stop(reason: str) -> None:
        document["stop_requested"] = True
        document["stop_reason"] = reason
        try:
            response = wait_future(
                node,
                stop_client.call_async(Trigger.Request()),
                SERVICE_TIMEOUT_S,
            )
            document["stop_succeeded"] = bool(response.success)
            document["stop_diagnostic"] = response.message
            print(
                "R3_SAFE_STOP "
                f"success={bool(response.success)} reason={reason}",
                flush=True,
            )
        except Exception as stop_error:
            document["stop_succeeded"] = False
            document["stop_diagnostic"] = repr(stop_error)
            print(
                f"R3_SAFE_STOP_FAIL reason={reason} error={stop_error}",
                flush=True,
            )

    try:
        for service_name, client in clients:
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
                raise RuntimeError(f"service unavailable: {service_name}")
        print("R3_SERVICES_READY", flush=True)

        # Prove the firmware supports the non-latching, physically verified
        # right-bus disable route before any torque can be enabled.
        pre_disable = wait_future(
            node,
            disable_client.call_async(Trigger.Request()),
            SERVICE_TIMEOUT_S,
        )
        document["preflight_verified_disable"] = {
            "success": bool(pre_disable.success),
            "diagnostic": pre_disable.message,
        }
        if not bool(pre_disable.success):
            raise RuntimeError(
                f"preflight verified disable failed: {pre_disable.message}"
            )
        print("R3_PREFLIGHT_VERIFIED_DISABLE_PASS torque_mask=0x00", flush=True)

        # Reapply volatile left-equivalent settings.  This is required after
        # every MCU/servo power cycle and remains torque-off throughout.
        for servo_id in range(1, 7):
            request = RightArmConfigureOnce.Request()
            request.servo_id = servo_id
            request.confirmation = CONFIGURE_CONFIRMATION
            response = wait_future(
                node,
                configure_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            joint = joints[servo_id]
            checks = {
                "accepted": bool(response.accepted),
                "status": int(response.status_code) == 0,
                "torque_disabled": int(response.torque_enabled) == 0,
                "p_gain": int(response.p_gain) == int(joint["p_gain"]),
                "d_gain": int(response.d_gain) == int(joint["d_gain"]),
                "i_gain": int(response.i_gain) == 0,
                "mode": int(response.operating_mode) == 0,
                "speed": int(response.goal_speed_raw) == EXPECTED_GOAL_SPEED_RAW,
                "torque_limit": (
                    int(response.torque_limit_raw)
                    == EXPECTED_TORQUE_LIMITS[servo_id - 1]
                ),
            }
            record = {
                "servo_id": servo_id,
                "present_position_raw": int(response.present_position_raw),
                "goal_position_raw": int(response.goal_position_raw),
                "checks": checks,
            }
            document["configuration"].append(record)
            print(
                f"R3_CONFIGURE servo_id={servo_id} "
                f"verdict={'PASS' if all(checks.values()) else 'FAIL'} "
                f"torque={int(response.torque_enabled)}",
                flush=True,
            )
            if not all(checks.values()):
                raise RuntimeError(f"configuration failed for servo {servo_id}")

        # Independent readback before any torque enable.
        for servo_id in range(1, 7):
            request = RightArmConfiguration.Request()
            request.servo_id = servo_id
            response = wait_future(
                node,
                configuration_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            checks = configuration_checks(response, joints[servo_id], 0)
            record = {
                "servo_id": servo_id,
                "position_raw": int(response.position_raw),
                "checks": checks,
            }
            document["preflight"].append(record)
            print(
                f"R3_PREFLIGHT servo_id={servo_id} "
                f"position={int(response.position_raw)} "
                f"verdict={'PASS' if all(checks.values()) else 'FAIL'}",
                flush=True,
            )
            if not all(checks.values()):
                raise RuntimeError(f"preflight failed for servo {servo_id}")

        # Hold every joint at its fresh present position before the first jog.
        for servo_id in range(1, 7):
            request = RightArmTorqueEnableOnce.Request()
            request.servo_id = servo_id
            request.confirmation = TORQUE_CONFIRMATION
            response = wait_future(
                node,
                torque_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            accepted = (
                bool(response.accepted)
                and int(response.status_code) == 0
                and int(response.torque_enabled) == 1
                and int(response.present_position_raw)
                == int(response.held_goal_position_raw)
            )
            document["torque_enable"].append(
                {
                    "servo_id": servo_id,
                    "accepted": accepted,
                    "present_position_raw": int(response.present_position_raw),
                    "held_goal_position_raw": int(
                        response.held_goal_position_raw
                    ),
                }
            )
            print(
                f"R3_HOLD servo_id={servo_id} accepted={accepted} "
                f"position={int(response.present_position_raw)}",
                flush=True,
            )
            if not accepted:
                raise RuntimeError(f"torque hold failed for servo {servo_id}")

        previous_residuals: list[int] | None = None
        stall_counts = [0] * 6
        final_positions: list[int] = []
        for cycle in range(1, MAX_CYCLES + 1):
            positions: list[int] = []
            snapshots: list[dict[str, Any]] = []
            for servo_id in range(1, 7):
                request = RightArmConfiguration.Request()
                request.servo_id = servo_id
                response = wait_future(
                    node,
                    configuration_client.call_async(request),
                    SERVICE_TIMEOUT_S,
                )
                checks = configuration_checks(response, joints[servo_id], 1)
                if not all(checks.values()):
                    raise RuntimeError(
                        f"in-motion readback failed for servo {servo_id}"
                    )
                position = int(response.position_raw)
                positions.append(position)
                snapshots.append(
                    {
                        "servo_id": servo_id,
                        "position_raw": position,
                        "speed_raw": int(response.speed_raw),
                        "load_raw": int(response.load_raw),
                        "current_raw": int(response.current_raw),
                    }
                )

            residuals = [abs(target_raw - value) for value in positions]
            print(
                f"R3_CYCLE cycle={cycle} positions={positions} "
                f"residuals={residuals}",
                flush=True,
            )
            cycle_record: dict[str, Any] = {
                "cycle": cycle,
                "snapshots": snapshots,
                "residuals_raw": residuals,
                "commands": [],
            }
            document["cycles"].append(cycle_record)
            if max(residuals) <= HOME_TOLERANCE_RAW:
                final_positions = positions
                break

            if previous_residuals is not None:
                for index, residual in enumerate(residuals):
                    if residual <= HOME_TOLERANCE_RAW:
                        stall_counts[index] = 0
                    elif residual < previous_residuals[index]:
                        stall_counts[index] = 0
                    else:
                        stall_counts[index] += 1
                    if stall_counts[index] >= STALL_CYCLE_LIMIT:
                        raise RuntimeError(
                            f"servo {index + 1} did not approach q0 for "
                            f"{STALL_CYCLE_LIMIT} cycles"
                        )

            for servo_id, position in enumerate(positions, start=1):
                delta = bounded_step_raw(position, target_raw)
                if delta == 0:
                    continue
                request = RightArmJogOnce.Request()
                request.servo_id = servo_id
                request.delta_raw = delta
                request.confirmation = JOG_CONFIRMATION
                response = wait_future(
                    node,
                    jog_client.call_async(request),
                    SERVICE_TIMEOUT_S,
                )
                start = int(response.start_position_raw)
                commanded_target = int(response.target_position_raw)
                accepted = (
                    bool(response.accepted)
                    and int(response.status_code) in (0, 8)
                    and int(response.torque_enabled) == 1
                    and MINIMUM_STEP_RAW <= abs(delta) <= MAXIMUM_STEP_RAW
                    and abs(commanded_target - target_raw)
                    < abs(start - target_raw)
                )
                cycle_record["commands"].append(
                    {
                        "servo_id": servo_id,
                        "delta_raw": delta,
                        "start_position_raw": start,
                        "target_position_raw": commanded_target,
                        "accepted": accepted,
                    }
                )
                if not accepted:
                    raise RuntimeError(
                        f"bounded jog failed for servo {servo_id}"
                    )
            previous_residuals = residuals
            time.sleep(SETTLE_S)
        else:
            raise RuntimeError(f"q0 not reached within {MAX_CYCLES} cycles")

        disable_response = wait_future(
            node,
            disable_client.call_async(Trigger.Request()),
            SERVICE_TIMEOUT_S,
        )
        if not bool(disable_response.success):
            raise RuntimeError(
                f"verified right-arm disable failed: {disable_response.message}"
            )
        document["verified_disable_succeeded"] = True
        document["verified_disable_diagnostic"] = disable_response.message
        post_disable = []
        for servo_id in range(1, 7):
            request = RightArmConfiguration.Request()
            request.servo_id = servo_id
            response = wait_future(
                node,
                configuration_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            checks = configuration_checks(response, joints[servo_id], 0)
            post_disable.append(
                {
                    "servo_id": servo_id,
                    "position_raw": int(response.position_raw),
                    "checks": checks,
                }
            )
            if not all(checks.values()):
                raise RuntimeError(
                    f"post-disable readback failed for servo {servo_id}"
                )
        document["post_disable"] = post_disable
        print("R3_VERIFIED_DISABLE_PASS torque_mask=0x00", flush=True)

        document["initial_positions_raw"] = [
            int(item["position_raw"]) for item in document["preflight"]
        ]
        document["final_positions_raw"] = final_positions
        document["final_residuals_raw"] = [
            abs(target_raw - value) for value in final_positions
        ]
        document["torque_enabled_at_completion"] = False
        document["overall_verdict"] = "R3_BOUNDED_HOME_PASS"
        digest = write_result(args.output, document)
        print(
            "R3_BOUNDED_HOME_PASS "
            f"final_positions={final_positions} "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 0
    except BaseException as error:
        call_stop(repr(error))
        document["overall_verdict"] = "R3_BOUNDED_HOME_FAIL"
        document["failure"] = repr(error)
        digest = write_result(args.output, document)
        print(
            f"R3_BOUNDED_HOME_FAIL error={error} "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 130 if isinstance(error, KeyboardInterrupt) else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
