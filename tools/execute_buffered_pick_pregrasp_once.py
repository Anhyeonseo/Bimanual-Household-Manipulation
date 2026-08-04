#!/usr/bin/env python3
"""Send one SHA-pinned Motion-11 Pick pregrasp Action without retry."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path

from execute_buffered_q0_roundtrip_once import (
    PlanWaypoint,
    build_goal,
    send_goal_once,
    sha256_file,
    validate_action_terminal,
    validate_fresh_start,
    wait_joint_state,
)
from plan_buffered_pick_pregrasp import (
    PHASE,
    STATUS,
    TOTAL_DURATION_MS,
    build_plan,
)


ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
CONFIRMATION = "EXECUTE_MOTION11_PICK_PREGRASP_ONCE"
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 60.0
START_TOLERANCES_RAD = (0.050, 0.055, 0.050, 0.050, 0.050)
TARGET_TOLERANCES_RAD = START_TOLERANCES_RAD


@dataclass(frozen=True, slots=True)
class PickPregraspPlan:
    path: Path
    sha256: str
    arm_joint_names: tuple[str, ...]
    anchor_positions_rad: tuple[float, ...]
    target_positions_rad: tuple[float, ...]
    waypoints: tuple[PlanWaypoint, ...]
    duration_ms: int
    sample_count: int


def _finite_vector(value, count: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must contain exactly {count} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def load_pick_pregrasp_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
    source_route_path: Path,
) -> PickPregraspPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != STATUS or document.get("phase") != PHASE:
        raise ValueError("plan status or phase is not Motion-11")

    anchor = document.get("anchor")
    if not isinstance(anchor, dict):
        raise ValueError("plan anchor is invalid")
    anchor_raw = anchor.get("raw")
    if (
        not isinstance(anchor_raw, list)
        or len(anchor_raw) != 6
        or any(isinstance(value, bool) or not isinstance(value, int) for value in anchor_raw)
    ):
        raise ValueError("plan anchor raw must contain six integers")

    recomputed = build_plan(
        calibration_path,
        contract_path,
        source_route_path,
        tuple(anchor_raw),
    )
    if document != recomputed:
        raise ValueError("plan evidence is not exactly reproducible")

    names = tuple(document["joint_names"][:5])
    anchor_positions = _finite_vector(
        document["anchor"]["positions_rad"],
        6,
        "anchor positions",
    )
    target_positions = _finite_vector(
        document["target"]["positions_rad"],
        6,
        "target positions",
    )
    waypoints = tuple(
        PlanWaypoint(
            time_from_start_ms=int(item["time_from_start_ms"]),
            positions_rad=_finite_vector(
                item["positions_rad"],
                5,
                "waypoint positions",
            ),
        )
        for item in document["waypoints"]
    )
    duration_ms = int(document["resampling"]["duration_ms"])
    sample_count = int(document["resampling"]["sample_count"])
    if duration_ms != TOTAL_DURATION_MS or sample_count != len(waypoints):
        raise ValueError("plan duration or sample count is invalid")
    if waypoints[0].positions_rad != anchor_positions[:5]:
        raise ValueError("plan first waypoint does not match anchor")
    if waypoints[-1].positions_rad != target_positions[:5]:
        raise ValueError("plan final waypoint does not match pregrasp")

    return PickPregraspPlan(
        path=path,
        sha256=digest,
        arm_joint_names=names,
        anchor_positions_rad=anchor_positions[:5],
        target_positions_rad=target_positions[:5],
        waypoints=waypoints,
        duration_ms=duration_ms,
        sample_count=sample_count,
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
    parser.add_argument("--source-route", required=True, type=Path)
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
        parser.error("exact Motion-11 one-shot confirmation is required")

    plan = load_pick_pregrasp_plan(
        arguments.plan,
        arguments.expected_sha256,
        arguments.calibration,
        arguments.contract,
        arguments.source_route,
    )
    print(f"PLAN_SHA256={plan.sha256}")
    print(f"PLAN_DURATION_MS={plan.duration_ms}")
    print(f"PLAN_SAMPLE_COUNT={plan.sample_count}")
    print("PLAN_GATE=PASS")

    rclpy.init()
    node = Node("motion11_buffered_pick_pregrasp_once")
    action_client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
    try:
        current = wait_joint_state(node, plan.arm_joint_names)
        start_error = validate_fresh_start(
            current,
            plan.anchor_positions_rad,
            START_TOLERANCES_RAD,
        )
        print(f"FRESH_START_MAX_ERROR_RAD={start_error:.6f}")
        print("FRESH_START_GATE=PASS")
        if not action_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_S):
            raise RuntimeError("FollowJointTrajectory Action is unavailable")

        print("ACTION_SEND_COUNT=1")
        status, result = send_goal_once(
            node,
            action_client,
            build_goal(plan),
            result_timeout_s=ACTION_RESULT_TIMEOUT_S,
        )
        evidence = validate_action_terminal(status, result)
        final_positions = wait_joint_state(node, plan.arm_joint_names)
        target_error = validate_fresh_start(
            final_positions,
            plan.target_positions_rad,
            TARGET_TOLERANCES_RAD,
        )
        print(
            "ACTION_TERMINAL_PASS "
            f"status={evidence.action_status} "
            f"error_code={evidence.error_code} "
            f"maximum_apply_lateness_ms={evidence.maximum_apply_lateness_ms} "
            f"post_settle_max_error_raw={evidence.post_settle_max_error_raw}"
        )
        print(f"PREGRASP_MAX_ERROR_RAD={target_error:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")
        print("MOTION11_BUFFERED_PICK_PREGRASP_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
