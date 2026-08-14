#!/usr/bin/env python3
"""Run one SHA-bound right-arm J2 axis roundtrip through isolated primitives."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from bimanual_j2_targets import (
    ARM_JOINTS,
    SETTLE_TOLERANCE_RAW,
    bounded_step_raw,
    file_sha256,
    load_bound_json,
    select_target,
)
from execute_right_arm_bounded_home_once import (
    CONFIGURATION_SERVICE,
    CONFIGURE_CONFIRMATION,
    CONFIGURE_SERVICE,
    EXPECTED_GOAL_SPEED_RAW,
    EXPECTED_TORQUE_LIMITS,
    JOG_CONFIRMATION,
    JOG_SERVICE,
    SERVICE_TIMEOUT_S,
    TORQUE_CONFIRMATION,
    TORQUE_SERVICE,
    configuration_checks,
    wait_future,
)


CONFIRMATION = "J2_RIGHT_ARM_AXIS_ROUNDTRIP_ONCE"
DISABLE_SERVICE = "/right_arm_disable"
STOP_SERVICE = "/right_arm_stop"
J2B_IDENTITY_SERVICE = "/right_arm_j2_base_limits_identity"
EXPECTED_J2B_LIMITS_SHA256 = (
    "dfbfaf6c7138fab30afebc1f3e69c7d53edb01060bd349f65c6f048f150dff34"
)
EXPECTED_J2B_STATUS = (
    "J2_B_BASE_LIMIT_CANDIDATE_AWAITING_NO_MOTION_AND_ACTIVE_VALIDATION"
)
EXPECTED_APPROVED_SHA256 = (
    "ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff"
)
EXPECTED_J0_SHA256 = (
    "c1d6d41c402de15c0ac03ceca7c9eeb2d2ffe166dd794599e3fdc8b2db87a48e"
)
SETTLE_S = 0.35
TARGET_CONSECUTIVE_SAMPLES = 3
STALL_CYCLE_LIMIT = 5
MAX_CYCLES_PER_LEG = 100
MAX_COMMAND_TRACKING_ERROR_RAW = 40
MAX_OTHER_JOINT_DRIFT_RAW = 10
MAX_PRECOMMAND_DRIFT_RAW = SETTLE_TOLERANCE_RAW
MINIMUM_VOLTAGE_RAW = 90
MAXIMUM_VOLTAGE_RAW = 140
MAXIMUM_TEMPERATURE_C = 70


def write_result(path: Path, document: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return file_sha256(path)


def health_checks(
    response: Any,
    joint: dict[str, Any],
    expected_torque: int,
) -> dict[str, bool]:
    checks = configuration_checks(response, joint, expected_torque)
    checks.update(
        {
            "voltage": (
                MINIMUM_VOLTAGE_RAW
                <= int(response.voltage_raw)
                <= MAXIMUM_VOLTAGE_RAW
            ),
            "temperature": int(response.temperature_c) <= MAXIMUM_TEMPERATURE_C,
        }
    )
    return checks


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=ARM_JOINTS, required=True)
    parser.add_argument("--direction", choices=("lower", "upper"), required=True)
    parser.add_argument("--fraction-percent", choices=(25, 50, 75), type=int, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--left-arm-12v-off-confirmed", action="store_true")
    parser.add_argument("--operator-support-confirmed", action="store_true")
    parser.add_argument("--target-hold-s", type=float, default=3.0)
    parser.add_argument(
        "--targets",
        type=Path,
        default=root / "artifacts/joint_ranges/2026-08-13/j2_axis_targets_plan_only.json",
    )
    parser.add_argument("--targets-sha256", required=True)
    parser.add_argument(
        "--j0-envelope",
        type=Path,
        default=root / "config/bimanual_j0_desired_envelope.reviewed.json",
    )
    parser.add_argument("--j0-envelope-sha256", required=True)
    parser.add_argument(
        "--candidate",
        type=Path,
        default=root / "config/right_arm_calibration.candidate.json",
    )
    parser.add_argument("--candidate-sha256", required=True)
    parser.add_argument(
        "--command-limits",
        type=Path,
        default=root / "config/right_arm_j2b_command_limits.candidate.json",
    )
    parser.add_argument("--command-limits-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.confirmation != CONFIRMATION:
        raise SystemExit(f"--confirmation must be {CONFIRMATION}")
    if not args.left_arm_12v_off_confirmed:
        raise SystemExit("--left-arm-12v-off-confirmed is required")
    if not args.operator_support_confirmed:
        raise SystemExit("--operator-support-confirmed is required")
    if not 1.0 <= args.target_hold_s <= 10.0:
        raise SystemExit("--target-hold-s must be within 1..10 seconds")

    targets = load_bound_json(args.targets, args.targets_sha256, "J2 targets")
    j0_envelope = load_bound_json(
        args.j0_envelope,
        args.j0_envelope_sha256,
        "reviewed J0-D envelope",
    )
    if args.j0_envelope_sha256.lower() != EXPECTED_J0_SHA256:
        raise RuntimeError("unexpected reviewed J0-D envelope SHA")
    if (
        j0_envelope.get("status") != "J0_D_REVIEWED_PASS_J0_M_NOT_MEASURED"
        or j0_envelope.get("motion_authorized") is not False
    ):
        raise RuntimeError("reviewed J0-D envelope contract changed")
    if targets.get("inputs", {}).get("approved", {}).get("sha256") != (
        EXPECTED_APPROVED_SHA256
    ):
        raise RuntimeError("J2 targets are not bound to the approved J1-L SHA")
    selected = select_target(
        targets,
        "right",
        args.joint,
        args.direction,
        args.fraction_percent,
    )
    candidate = load_bound_json(
        args.candidate,
        args.candidate_sha256,
        "right-arm calibration candidate",
    )
    if candidate.get("arm_slot") != "right":
        raise RuntimeError("candidate arm_slot must be right")
    joints = {int(item["id"]): item for item in candidate["joints"]}
    if set(joints) != set(range(1, 7)):
        raise RuntimeError("candidate must contain exactly servo IDs 1..6")
    command_limits = load_bound_json(
        args.command_limits,
        args.command_limits_sha256,
        "J2-B command-limit manifest",
    )
    if args.command_limits_sha256.lower() != EXPECTED_J2B_LIMITS_SHA256:
        raise RuntimeError("unexpected J2-B command-limit manifest SHA")
    if (
        command_limits.get("record_kind")
        != "right_arm_j2b_command_limits_candidate"
        or command_limits.get("status") != EXPECTED_J2B_STATUS
        or command_limits.get("firmware_version") != "0x00024200"
        or command_limits.get("capability") != "0x10000000"
        or command_limits.get("motion_authorized") is not False
        or command_limits.get("general_trajectory_authorized") is not False
        or int(command_limits.get("maximum_jog_step_raw", 0)) != 20
    ):
        raise RuntimeError("J2-B command-limit manifest contract changed")
    command_joints = {
        int(item["id"]): item for item in command_limits["joints"]
    }
    if set(command_joints) != set(range(1, 7)):
        raise RuntimeError("J2-B limits must contain exactly servo IDs 1..6")
    passive_joints = j0_envelope["arms"]["right"]["joints"]
    if set(passive_joints) != {
        "base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"
    }:
        raise RuntimeError("reviewed J0-D right-arm joint set changed")
    servo_id = int(selected["servo_id"])
    if joints[servo_id]["name"].lower() != args.joint:
        raise RuntimeError("selected J2 joint does not match calibration ID")
    target_raw = int(selected["target_unwrapped_raw"])
    q0_raw = int(selected["q0_unwrapped_raw"])
    lower_raw = int(selected["approved_minimum_unwrapped_raw"])
    upper_raw = int(selected["approved_maximum_unwrapped_raw"])
    if not lower_raw < target_raw < upper_raw:
        raise RuntimeError("J2 target is not strictly inside J1-L")
    if command_joints[servo_id]["name"].lower() != args.joint:
        raise RuntimeError("selected J2 joint does not match command-limit ID")
    primitive_minimum_raw = int(command_joints[servo_id]["minimum_raw"])
    primitive_maximum_raw = int(command_joints[servo_id]["maximum_raw"])
    if not primitive_minimum_raw <= q0_raw <= primitive_maximum_raw:
        raise RuntimeError("q0 is outside the validated bounded-jog primitive limits")
    if not primitive_minimum_raw <= target_raw <= primitive_maximum_raw:
        raise RuntimeError(
            "selected J2 target is outside the currently validated "
            f"bounded-jog primitive limits {primitive_minimum_raw}.."
            f"{primitive_maximum_raw}"
        )
    if args.fraction_percent in (0, 100):
        raise RuntimeError("J2 endpoints are forbidden")

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
        "record_kind": "right_arm_j2_axis_roundtrip_once",
        "overall_verdict": "IN_PROGRESS",
        "motion_authorized": True,
        "authorization_scope": "one reviewed right-arm axis roundtrip only",
        "general_trajectory_authorized": False,
        "multi_joint_commands_forbidden": True,
        "endpoint_commands_forbidden": True,
        "automatic_retry_count": 0,
        "selected": selected,
        "maximum_jog_step_raw": 20,
        "settle_tolerance_raw": SETTLE_TOLERANCE_RAW,
        "target_hold_s": args.target_hold_s,
        "inputs": {
            "targets": {
                "path": str(args.targets),
                "sha256": args.targets_sha256.lower(),
            },
            "j0_envelope": {
                "path": str(args.j0_envelope),
                "sha256": args.j0_envelope_sha256.lower(),
            },
            "candidate": {
                "path": str(args.candidate),
                "sha256": args.candidate_sha256.lower(),
            },
            "command_limits": {
                "path": str(args.command_limits),
                "sha256": args.command_limits_sha256.lower(),
            },
        },
        "j2b_identity": None,
        "configuration": [],
        "preflight": [],
        "torque_enable": None,
        "legs": [],
        "verified_disable": None,
        "preflight_verified_disable": None,
        "pre_motion_failure_disable": None,
        "nonselected_baseline_policy": "torque_off_inside_reviewed_J0D_and_drift_le_10_raw",
        "safe_stop": None,
    }

    print(
        "J2_RIGHT_AXIS_PRECHECK_PASS "
        f"joint={args.joint} servo_id={servo_id} "
        f"direction={args.direction} fraction={args.fraction_percent}% "
        f"q0={q0_raw} target={target_raw} approved={lower_raw}..{upper_raw} "
        f"primitive={primitive_minimum_raw}..{primitive_maximum_raw} "
        "max_step=20 left_12v=off",
        flush=True,
    )

    rclpy.init()
    node = rclpy.create_node("right_arm_j2_axis_roundtrip_client")
    configure_client = node.create_client(
        RightArmConfigureOnce,
        CONFIGURE_SERVICE,
    )
    configuration_client = node.create_client(
        RightArmConfiguration,
        CONFIGURATION_SERVICE,
    )
    torque_client = node.create_client(
        RightArmTorqueEnableOnce,
        TORQUE_SERVICE,
    )
    jog_client = node.create_client(RightArmJogOnce, JOG_SERVICE)
    disable_client = node.create_client(Trigger, DISABLE_SERVICE)
    stop_client = node.create_client(Trigger, STOP_SERVICE)
    j2b_identity_client = node.create_client(Trigger, J2B_IDENTITY_SERVICE)

    clients = (
        (CONFIGURE_SERVICE, configure_client),
        (CONFIGURATION_SERVICE, configuration_client),
        (TORQUE_SERVICE, torque_client),
        (JOG_SERVICE, jog_client),
        (DISABLE_SERVICE, disable_client),
        (STOP_SERVICE, stop_client),
        (J2B_IDENTITY_SERVICE, j2b_identity_client),
    )

    def call_trigger(client: Any) -> Any:
        return wait_future(
            node,
            client.call_async(Trigger.Request()),
            SERVICE_TIMEOUT_S,
        )

    def call_stop(reason: str) -> None:
        try:
            response = call_trigger(stop_client)
            document["safe_stop"] = {
                "requested": True,
                "success": bool(response.success),
                "reason": reason,
                "diagnostic": response.message,
            }
            print(
                f"J2_LATCHED_SAFE_STOP success={bool(response.success)} "
                f"reason={reason}",
                flush=True,
            )
        except Exception as error:
            document["safe_stop"] = {
                "requested": True,
                "success": False,
                "reason": reason,
                "diagnostic": repr(error),
            }
            print(
                f"J2_LATCHED_SAFE_STOP_FAIL reason={reason} error={error}",
                flush=True,
            )

    def call_pre_motion_disable(reason: str) -> bool:
        try:
            response = call_trigger(disable_client)
            success = bool(response.success)
            document["pre_motion_failure_disable"] = {
                "requested": True,
                "success": success,
                "reason": reason,
                "diagnostic": response.message,
            }
            print(
                f"J2_PRE_MOTION_VERIFIED_DISABLE success={success} "
                f"reason={reason}",
                flush=True,
            )
            return success
        except Exception as disable_error:
            document["pre_motion_failure_disable"] = {
                "requested": True,
                "success": False,
                "reason": reason,
                "diagnostic": repr(disable_error),
            }
            print(
                f"J2_PRE_MOTION_VERIFIED_DISABLE_FAIL reason={reason} "
                f"error={disable_error}",
                flush=True,
            )
            return False

    def read_all(expected_selected_torque: int) -> list[dict[str, Any]]:
        snapshots: list[dict[str, Any]] = []
        for current_id in range(1, 7):
            request = RightArmConfiguration.Request()
            request.servo_id = current_id
            response = wait_future(
                node,
                configuration_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            expected_torque = (
                expected_selected_torque if current_id == servo_id else 0
            )
            checks = health_checks(
                response,
                joints[current_id],
                expected_torque,
            )
            snapshot = {
                "servo_id": current_id,
                "position_raw": int(response.position_raw),
                "goal_position_raw": int(response.goal_position_raw),
                "torque_enabled": int(response.torque_enabled),
                "voltage_raw": int(response.voltage_raw),
                "temperature_c": int(response.temperature_c),
                "speed_raw": int(response.speed_raw),
                "load_raw": int(response.load_raw),
                "current_raw": int(response.current_raw),
                "protection_current_raw": int(
                    response.protection_current_raw
                ),
                "checks": checks,
            }
            snapshots.append(snapshot)
            if not all(checks.values()):
                raise RuntimeError(
                    f"configuration/health check failed for servo {current_id}"
                )
        return snapshots
    preflight_verified = False
    motion_started = False

    baseline_other_positions: dict[int, int] = {}
    passive_other_intervals: dict[int, tuple[int, int]] = {}
    maximum_tracking_error = 0

    def run_leg(goal_raw: int, label: str) -> None:
        nonlocal maximum_tracking_error
        leg: dict[str, Any] = {
            "label": label,
            "goal_raw": goal_raw,
            "cycles": [],
        }
        document["legs"].append(leg)
        previous_residual: int | None = None
        stall_count = 0
        consecutive = 0
        pending_command_target: int | None = None

        for cycle in range(1, MAX_CYCLES_PER_LEG + 1):
            snapshots = read_all(1)
            selected_snapshot = snapshots[servo_id - 1]
            position = int(selected_snapshot["position_raw"])
            residual = abs(goal_raw - position)

            for snapshot in snapshots:
                current_id = int(snapshot["servo_id"])
                if current_id == servo_id:
                    continue
                passive_minimum, passive_maximum = passive_other_intervals[
                    current_id
                ]
                passive_position = int(snapshot["position_raw"])
                if not passive_minimum <= passive_position <= passive_maximum:
                    raise RuntimeError(
                        f"non-selected servo {current_id} left reviewed J0-D: "
                        f"position={passive_position} interval="
                        f"{passive_minimum}..{passive_maximum}"
                    )
                drift = abs(
                    passive_position
                    - baseline_other_positions[current_id]
                )
                snapshot["drift_from_preflight_raw"] = drift
                if drift > MAX_OTHER_JOINT_DRIFT_RAW:
                    raise RuntimeError(
                        f"non-selected servo {current_id} drifted {drift} raw"
                    )

            tracking_error = None
            if pending_command_target is not None:
                tracking_error = abs(position - pending_command_target)
                maximum_tracking_error = max(
                    maximum_tracking_error,
                    tracking_error,
                )
                if tracking_error > MAX_COMMAND_TRACKING_ERROR_RAW:
                    raise RuntimeError(
                        f"selected servo tracking error {tracking_error} raw"
                    )
                pending_command_target = None

            cycle_record: dict[str, Any] = {
                "cycle": cycle,
                "selected_position_raw": position,
                "residual_raw": residual,
                "tracking_error_raw": tracking_error,
                "snapshots": snapshots,
                "command": None,
            }
            leg["cycles"].append(cycle_record)
            print(
                f"J2_CYCLE leg={label} cycle={cycle} "
                f"position={position} residual={residual}",
                flush=True,
            )

            if residual <= SETTLE_TOLERANCE_RAW:
                consecutive += 1
                if consecutive >= TARGET_CONSECUTIVE_SAMPLES:
                    leg["final_position_raw"] = position
                    leg["final_residual_raw"] = residual
                    leg["settled_samples"] = consecutive
                    return
                time.sleep(SETTLE_S)
                continue
            consecutive = 0

            if previous_residual is not None:
                if residual < previous_residual:
                    stall_count = 0
                else:
                    stall_count += 1
                if stall_count >= STALL_CYCLE_LIMIT:
                    raise RuntimeError(
                        f"selected servo stalled for {STALL_CYCLE_LIMIT} cycles"
                    )

            delta = bounded_step_raw(position, goal_raw)
            commanded_target = position + delta
            if not lower_raw < commanded_target < upper_raw:
                raise RuntimeError("bounded jog would leave approved J1-L")
            if not primitive_minimum_raw <= commanded_target <= primitive_maximum_raw:
                raise RuntimeError("bounded jog would leave R2.1 primitive limits")
            if abs(commanded_target - goal_raw) >= residual:
                raise RuntimeError("bounded jog does not approach selected goal")

            request = RightArmJogOnce.Request()
            request.servo_id = servo_id
            request.delta_raw = delta
            request.confirmation = JOG_CONFIRMATION
            response = wait_future(
                node,
                jog_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            response_start = int(response.start_position_raw)
            response_target = int(response.target_position_raw)
            precommand_drift = abs(response_start - position)
            # The service intentionally takes a fresh position after the
            # independent health snapshot. Allow only normal settling within
            # the same 10-raw terminal tolerance, then validate from that fresh start.
            accepted = (
                bool(response.accepted)
                and int(response.status_code) in (0, 8)
                and int(response.torque_enabled) == 1
                and precommand_drift <= MAX_PRECOMMAND_DRIFT_RAW
                and response_target == response_start + delta
                and lower_raw < response_target < upper_raw
                and primitive_minimum_raw
                <= response_target
                <= primitive_maximum_raw
                and abs(response_target - goal_raw)
                < abs(response_start - goal_raw)
            )
            cycle_record["command"] = {
                "delta_raw": delta,
                "sampled_position_raw": position,
                "start_position_raw": response_start,
                "precommand_drift_raw": precommand_drift,
                "target_position_raw": response_target,
                "observed_position_raw": int(response.observed_position_raw),
                "status_code": int(response.status_code),
                "accepted": accepted,
            }
            if not accepted:
                raise RuntimeError("bounded jog service rejected or drifted")
            pending_command_target = response_target
            previous_residual = residual
            time.sleep(SETTLE_S)
        raise RuntimeError(
            f"{label} did not settle within {MAX_CYCLES_PER_LEG} cycles"
        )

    try:
        for service_name, client in clients:
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
                raise RuntimeError(f"service unavailable: {service_name}")
        print("J2_SERVICES_READY", flush=True)
        identity_response = call_trigger(j2b_identity_client)
        if not identity_response.success:
            raise RuntimeError(
                f"J2-B identity rejected: {identity_response.message}"
            )
        identity = json.loads(identity_response.message)
        if (
            identity.get("firmware_version") != "0x00024200"
            or (int(identity.get("capabilities", "0"), 16) & 0x10000000) == 0
            or identity.get("manifest_sha256")
            != EXPECTED_J2B_LIMITS_SHA256
            or identity.get("status") != EXPECTED_J2B_STATUS
            or identity.get("joints") != command_limits["joints"]
        ):
            raise RuntimeError("J2-B bridge identity contract mismatch")
        document["j2b_identity"] = identity
        print(
            "J2_B_IDENTITY_PASS firmware=0x00024200 "
            f"manifest_sha256={EXPECTED_J2B_LIMITS_SHA256}",
            flush=True,
        )
        pre_disable = call_trigger(disable_client)
        document["preflight_verified_disable"] = {
            "success": bool(pre_disable.success),
            "diagnostic": pre_disable.message,
        }
        if not bool(pre_disable.success):
            raise RuntimeError(
                f"preflight verified disable failed: {pre_disable.message}"
            )
        preflight_verified = True
        print("J2_PREFLIGHT_VERIFIED_DISABLE_PASS torque_mask=0x00", flush=True)

        for current_id in range(1, 7):
            request = RightArmConfigureOnce.Request()
            request.servo_id = current_id
            request.confirmation = CONFIGURE_CONFIRMATION
            response = wait_future(
                node,
                configure_client.call_async(request),
                SERVICE_TIMEOUT_S,
            )
            joint = joints[current_id]
            checks = {
                "accepted": bool(response.accepted),
                "status": int(response.status_code) == 0,
                "torque_disabled": int(response.torque_enabled) == 0,
                "p_gain": int(response.p_gain) == int(joint["p_gain"]),
                "d_gain": int(response.d_gain) == int(joint["d_gain"]),
                "i_gain": int(response.i_gain) == 0,
                "mode": int(response.operating_mode) == 0,
                "speed": int(response.goal_speed_raw)
                == EXPECTED_GOAL_SPEED_RAW,
                "torque_limit": int(response.torque_limit_raw)
                == EXPECTED_TORQUE_LIMITS[current_id - 1],
            }
            document["configuration"].append(
                {
                    "servo_id": current_id,
                    "present_position_raw": int(
                        response.present_position_raw
                    ),
                    "checks": checks,
                }
            )
            if not all(checks.values()):
                raise RuntimeError(
                    f"torque-off configuration failed for servo {current_id}"
                )

        preflight = read_all(0)
        document["preflight"] = preflight
        for snapshot in preflight:
            current_id = int(snapshot["servo_id"])
            position = int(snapshot["position_raw"])
            if current_id == servo_id:
                if abs(position - q0_raw) > SETTLE_TOLERANCE_RAW:
                    raise RuntimeError(
                        f"selected servo {current_id} is not at q0: "
                        f"position={position}"
                    )
                continue

            joint_name = str(joints[current_id]["name"]).lower()
            passive = passive_joints[joint_name]
            if current_id <= 5:
                baseline_minimum = int(passive["observed_minimum"])
                baseline_maximum = int(passive["observed_maximum"])
            else:
                baseline_minimum = int(passive["manual_sweep_minimum"])
                baseline_maximum = int(passive["manual_sweep_maximum"])
            if not baseline_minimum <= position <= baseline_maximum:
                raise RuntimeError(
                    f"non-selected {joint_name} baseline is outside reviewed "
                    f"J0-D: position={position} interval="
                    f"{baseline_minimum}..{baseline_maximum}"
                )
            snapshot["passive_baseline_interval_raw"] = [
                baseline_minimum,
                baseline_maximum,
            ]
            baseline_other_positions[current_id] = position
            passive_other_intervals[current_id] = (
                baseline_minimum,
                baseline_maximum,
            )
        print(
            f"J2_SELECTED_Q0_PASSIVE_BASELINE_PASS positions="
            f"{[item['position_raw'] for item in preflight]}",
            flush=True,
        )

        motion_started = True
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
            and int(response.status_code) in (0, 11)
            and int(response.torque_enabled) == 1
            and int(response.present_position_raw)
            == int(response.held_goal_position_raw)
        )
        document["torque_enable"] = {
            "accepted": accepted,
            "status_code": int(response.status_code),
            "present_position_raw": int(response.present_position_raw),
            "held_goal_position_raw": int(
                response.held_goal_position_raw
            ),
        }
        if not accepted:
            raise RuntimeError("selected-servo torque hold failed")
        read_all(1)
        print(
            f"J2_SELECTED_TORQUE_ENABLE_PASS servo_id={servo_id} "
            f"position={int(response.present_position_raw)}",
            flush=True,
        )

        run_leg(target_raw, "q0_to_target")
        print(
            f"J2_TARGET_REACHED joint={args.joint} target={target_raw} "
            f"hold_s={args.target_hold_s:.1f}; press Ctrl-C for latched stop",
            flush=True,
        )
        time.sleep(args.target_hold_s)
        target_hold = read_all(1)
        document["target_hold_snapshot"] = target_hold
        run_leg(q0_raw, "target_to_q0")

        disable_response = call_trigger(disable_client)
        if not bool(disable_response.success):
            raise RuntimeError(
                f"verified disable failed: {disable_response.message}"
            )
        motion_started = False
        document["verified_disable"] = {
            "success": True,
            "diagnostic": disable_response.message,
        }
        post_disable = read_all(0)
        document["post_disable"] = post_disable
        final_position = int(post_disable[servo_id - 1]["position_raw"])
        if abs(final_position - q0_raw) > SETTLE_TOLERANCE_RAW:
            raise RuntimeError(
                f"selected servo did not return to q0: {final_position}"
            )

        document["maximum_tracking_error_raw"] = maximum_tracking_error
        document["final_position_raw"] = final_position
        document["overall_verdict"] = "J2_RIGHT_ARM_AXIS_ROUNDTRIP_PASS"
        document["motion_authorized"] = False
        digest = write_result(args.output, document)
        print(
            "J2_RIGHT_ARM_AXIS_ROUNDTRIP_PASS "
            f"joint={args.joint} direction={args.direction} "
            f"fraction={args.fraction_percent}% final={final_position} "
            f"tracking_error_max_raw={maximum_tracking_error} "
            "torque_mask=0x00 "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 0
    except BaseException as error:
        reason = repr(error)
        if motion_started or not preflight_verified:
            call_stop(reason)
        elif not call_pre_motion_disable(reason):
            call_stop(reason)
        document["overall_verdict"] = "J2_RIGHT_ARM_AXIS_ROUNDTRIP_FAIL"
        document["motion_authorized"] = False
        document["failure"] = reason
        document["maximum_tracking_error_raw"] = maximum_tracking_error
        digest = write_result(args.output, document)
        print(
            f"J2_RIGHT_ARM_AXIS_ROUNDTRIP_FAIL error={error} "
            f"output={args.output} sha256={digest}",
            flush=True,
        )
        return 130 if isinstance(error, KeyboardInterrupt) else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
