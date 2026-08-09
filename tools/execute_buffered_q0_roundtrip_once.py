#!/usr/bin/env python3
"""Send one SHA-pinned Motion-10 q0 round-trip Action without retry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from single_arm_bridge.action_validation import (
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (
    prepare_buffered_execution_plan,
)
from single_arm_bridge.calibration import load_calibration
from single_arm_bridge.buffered_trajectory import (
    load_buffered_trajectory_contract,
)

from execute_buffered_action_plan_once import (
    ACTION_NAME,
    ACTION_SERVER_TIMEOUT_S,
    ACTION_STATUS_SUCCEEDED,
    CommissioningPlan,
    FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
    PlanWaypoint,
    START_TOLERANCES_RAD,
    _finite_vector,
    build_goal,
    build_goal_spec,
    send_goal_once,
    sha256_file,
    validate_action_terminal,
    validate_fresh_start,
    wait_joint_state,
)


PLAN_STATUS = "BUFFERED_Q0_ROUNDTRIP_PLAN_ONLY_PASS"
PLAN_PHASE = "motion10_q0_roundtrip"
EXPECTED_FIRMWARE = "0x00022100"
DEPLOYED_0X221_CONTRACT_SHA256 = (
    "2370d6443b082d82afd22dd7e2f16d917c10cbf722f20accae7b9b1a23291f6b"
)
EXPECTED_CAPABILITIES = "0x00000FFF"
EXPECTED_CALIBRATION = "0x2D90167E"
CONFIRMATION = "EXECUTE_MOTION10_Q0_ROUNDTRIP_ONCE"
RESULT_TIMEOUT_S = 15.0
SAMPLE_PERIOD_MS = 20
ROUND_TRIP_DURATION_MS = 4_200
HALF_TRIP_DURATION_MS = ROUND_TRIP_DURATION_MS // 2
WAYPOINT_TIMES_MS = tuple(
    range(0, ROUND_TRIP_DURATION_MS + SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS)
)
EXPECTED_BATCHES = (9, 7) + (6,) * 32 + (3,)


def minimum_jerk_unit_progress(unit_time: float) -> float:
    if not 0.0 <= unit_time <= 1.0:
        raise ValueError("minimum-jerk unit time must be within 0..1")
    return unit_time**3 * (
        10.0 + unit_time * (-15.0 + 6.0 * unit_time)
    )


def expected_q0_progress(elapsed_ms: int) -> float:
    if elapsed_ms <= HALF_TRIP_DURATION_MS:
        return minimum_jerk_unit_progress(
            elapsed_ms / HALF_TRIP_DURATION_MS
        )
    return 1.0 - minimum_jerk_unit_progress(
        (elapsed_ms - HALF_TRIP_DURATION_MS) / HALF_TRIP_DURATION_MS
    )


Q0_PROGRESS = tuple(
    expected_q0_progress(elapsed_ms)
    for elapsed_ms in WAYPOINT_TIMES_MS
)


def load_q0_roundtrip_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
) -> CommissioningPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != PLAN_STATUS:
        raise ValueError("plan status is not the reviewed Motion-10 status")
    if document.get("phase") != PLAN_PHASE:
        raise ValueError("plan phase is not Motion-10 q0 round-trip")
    for field in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
        "buffered_frame_encoded",
    ):
        if document.get(field) is not False:
            raise ValueError(f"plan must keep {field}=false")
    if document.get("firmware_version") != EXPECTED_FIRMWARE:
        raise ValueError("plan firmware identity mismatch")
    if document.get("capabilities") != EXPECTED_CAPABILITIES:
        raise ValueError("plan capability identity mismatch")
    if document.get("calibration_hash") != EXPECTED_CALIBRATION:
        raise ValueError("plan calibration identity mismatch")
    if document.get("calibration_sha256") != sha256_file(calibration_path):
        raise ValueError("plan calibration file sha256 mismatch")
    contract = load_buffered_trajectory_contract(contract_path)
    physical = contract["physical_execution_candidate"]
    if (
        physical["firmware_version"] != EXPECTED_FIRMWARE
        or physical["deployed"] is not True
    ):
        raise ValueError("deployed Motion-10 contract is no longer valid")
    if document.get("contract_sha256") != DEPLOYED_0X221_CONTRACT_SHA256:
        raise ValueError("plan buffered contract sha256 mismatch")

    calibration = load_calibration(calibration_path)
    all_names = tuple(calibration.ros_joint_names)
    if tuple(document.get("joint_names", ())) != all_names:
        raise ValueError("plan joint order does not match calibration")
    arm_names = all_names[:5]

    anchor = document.get("anchor")
    if not isinstance(anchor, dict):
        raise ValueError("plan anchor is invalid")
    anchor_raw = anchor.get("raw")
    if (
        not isinstance(anchor_raw, list)
        or len(anchor_raw) != 6
        or any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in anchor_raw
        )
    ):
        raise ValueError("plan anchor raw must contain six integers")
    anchor_positions = _finite_vector(
        anchor.get("positions_rad"),
        6,
        "plan anchor positions",
    )
    recomputed_anchor = tuple(
        calibration.raw_feedback_to_radians(tuple(anchor_raw))
    )
    if any(
        not math.isclose(recorded, recomputed, abs_tol=1.0e-12)
        for recorded, recomputed in zip(
            anchor_positions,
            recomputed_anchor,
            strict=True,
        )
    ):
        raise ValueError("plan anchor radians are inconsistent with raw")

    q0 = document.get("q0")
    if not isinstance(q0, dict):
        raise ValueError("plan q0 evidence is missing")
    if q0 != {
        "arm_positions_rad": [0.0] * 5,
        "raw_with_preserved_gripper": [2048] * 5 + [anchor_raw[5]],
        "maximum_arm_error_raw": 0,
        "gripper_preserved": True,
    }:
        raise ValueError("plan q0 evidence does not match reviewed target")

    waypoint_documents = document.get("waypoints")
    if (
        not isinstance(waypoint_documents, list)
        or len(waypoint_documents) != len(WAYPOINT_TIMES_MS)
    ):
        raise ValueError("plan requires the reviewed 211 dense waypoints")
    if document.get("analytic_profile") != {
        "kind": "symmetric_quintic_minimum_jerk",
        "polynomial": "10t^3-15t^4+6t^5",
        "half_trip_duration_ms": HALF_TRIP_DURATION_MS,
        "waypoint_period_ms": SAMPLE_PERIOD_MS,
        "waypoint_count": len(WAYPOINT_TIMES_MS),
        "zero_velocity_boundaries_ms": [
            0,
            HALF_TRIP_DURATION_MS,
            ROUND_TRIP_DURATION_MS,
        ],
        "zero_acceleration_boundaries_ms": [
            0,
            HALF_TRIP_DURATION_MS,
            ROUND_TRIP_DURATION_MS,
        ],
    }:
        raise ValueError("plan analytic minimum-jerk profile is invalid")
    waypoints: list[PlanWaypoint] = []
    for item, time_ms, progress in zip(
        waypoint_documents,
        WAYPOINT_TIMES_MS,
        Q0_PROGRESS,
        strict=True,
    ):
        if not isinstance(item, dict):
            raise ValueError("plan contains an invalid waypoint")
        if item.get("time_from_start_ms") != time_ms:
            raise ValueError("plan waypoint timing does not match reviewed route")
        if not math.isclose(
            float(item.get("q0_progress", math.nan)),
            progress,
            abs_tol=1.0e-12,
        ):
            raise ValueError("plan q0 progress does not match reviewed route")
        positions = _finite_vector(
            item.get("positions_rad"),
            5,
            "waypoint positions",
        )
        expected_positions = tuple(
            start * (1.0 - progress)
            for start in anchor_positions[:5]
        )
        if any(
            not math.isclose(actual, expected, abs_tol=1.0e-12)
            for actual, expected in zip(
                positions,
                expected_positions,
                strict=True,
            )
        ):
            raise ValueError("plan waypoint positions do not match q0 route")
        waypoints.append(PlanWaypoint(time_ms, positions))

    midpoint_index = HALF_TRIP_DURATION_MS // SAMPLE_PERIOD_MS
    if waypoints[midpoint_index].positions_rad != (0.0,) * 5:
        raise ValueError("plan midpoint is not exact arm q0")
    if waypoints[-1].positions_rad != anchor_positions[:5]:
        raise ValueError("plan does not return to its anchor")

    points = tuple(
        TrajectoryPointData(
            positions=waypoint.positions_rad,
            time_from_start_ns=waypoint.time_from_start_ms * 1_000_000,
        )
        for waypoint in waypoints
    )
    limits = {
        name: calibration.ros_radian_limits[name]
        for name in arm_names
    }
    trajectory = validate_buffered_trajectory(
        arm_names,
        points,
        arm_names,
        limits,
        anchor_positions[:5],
        {name: 0.5 for name in arm_names},
        {name: 1.0 for name in arm_names},
        start_tolerance_rad=dict(
            zip(arm_names, START_TOLERANCES_RAD, strict=True)
        ),
    )
    recomputed = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=anchor_positions[5],
        current_tick_ms=100_000,
    )
    resampling = document.get("resampling")
    if not isinstance(resampling, dict):
        raise ValueError("plan resampling evidence is missing")
    if resampling.get("period_ms") != 20:
        raise ValueError("plan sample period must be 20 ms")
    if resampling.get("duration_ms") != ROUND_TRIP_DURATION_MS:
        raise ValueError("plan duration must be 4200 ms")
    if resampling.get("sample_count") != len(WAYPOINT_TIMES_MS):
        raise ValueError("plan sample count must be 211")
    recorded_samples = resampling.get("samples")
    if (
        not isinstance(recorded_samples, list)
        or len(recorded_samples) != len(recomputed.samples)
    ):
        raise ValueError("plan resampled samples are incomplete")
    for recorded, sample in zip(
        recorded_samples,
        recomputed.samples,
        strict=True,
    ):
        if recorded != {
            "index": sample.sample_index,
            "elapsed_ms": sample.trajectory_elapsed_ms,
            "apply_offset_ms": sample.apply_tick_ms - 100_000,
            "positions_urad": list(sample.positions_urad),
        }:
            raise ValueError("plan resampled sample evidence mismatch")

    queue = document.get("queue_contract")
    if not isinstance(queue, dict):
        raise ValueError("plan queue evidence is missing")
    if tuple(queue.get("admission_batch_sizes", ())) != EXPECTED_BATCHES:
        raise ValueError("plan queue admission shape mismatch")
    terminal = queue.get("simulation_terminal")
    if terminal != {
        "accepted_samples": 211,
        "applied_samples": 198,
        "queued_samples": 13,
        "safe_stop_required": False,
        "state": "input_complete",
        "success_without_firmware_terminal": False,
    }:
        raise ValueError("plan terminal simulation evidence is invalid")
    round_trip = document.get("round_trip")
    if (
        not isinstance(round_trip, dict)
        or round_trip.get("q0_time_from_start_ms") != HALF_TRIP_DURATION_MS
        or float(round_trip.get("maximum_return_error_rad", math.inf)) != 0.0
    ):
        raise ValueError("plan round-trip evidence is invalid")

    return CommissioningPlan(
        path=path,
        sha256=digest,
        arm_joint_names=arm_names,
        anchor_positions_rad=anchor_positions[:5],
        waypoints=tuple(waypoints),
        duration_ms=trajectory.duration_ms,
        sample_count=len(recomputed.samples),
    )


def main() -> int:
    from control_msgs.action import FollowJointTrajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws" / "src" / "single_arm_bridge"
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=package / "config" / "single_arm_calibration.json",
    )
    parser.add_argument(
        "--contract",
        type=Path,
        default=package / "config" / "buffered_trajectory_contract.json",
    )
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact Motion-10 one-shot confirmation is required")

    plan = load_q0_roundtrip_plan(
        arguments.plan,
        arguments.expected_sha256,
        arguments.calibration,
        arguments.contract,
    )
    print(f"PLAN_SHA256={plan.sha256}")
    print(f"PLAN_DURATION_MS={plan.duration_ms}")
    print(f"PLAN_SAMPLE_COUNT={plan.sample_count}")
    print("PLAN_GATE=PASS")

    rclpy.init()
    node = Node("motion10_buffered_q0_roundtrip_once")
    action_client = ActionClient(
        node,
        FollowJointTrajectory,
        ACTION_NAME,
    )
    try:
        current = wait_joint_state(node, plan.arm_joint_names)
        start_error = validate_fresh_start(
            current,
            plan.anchor_positions_rad,
        )
        print(f"FRESH_START_MAX_ERROR_RAD={start_error:.6f}")
        print("FRESH_START_GATE=PASS")
        if not action_client.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_S
        ):
            raise RuntimeError("FollowJointTrajectory Action is unavailable")

        print("ACTION_SEND_COUNT=1")
        status, result = send_goal_once(
            node,
            action_client,
            build_goal(plan),
            result_timeout_s=RESULT_TIMEOUT_S,
        )
        evidence = validate_action_terminal(status, result)
        final_positions = wait_joint_state(node, plan.arm_joint_names)
        return_error = validate_fresh_start(
            final_positions,
            plan.anchor_positions_rad,
        )
        print(
            "ACTION_TERMINAL_PASS "
            f"status={evidence.action_status} "
            f"error_code={evidence.error_code} "
            f"maximum_apply_lateness_ms="
            f"{evidence.maximum_apply_lateness_ms} "
            f"post_settle_max_error_raw="
            f"{evidence.post_settle_max_error_raw}"
        )
        print(f"ROUND_TRIP_MAX_ERROR_RAD={return_error:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")
        print("MOTION10_BUFFERED_Q0_ROUNDTRIP_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
