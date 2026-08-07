#!/usr/bin/env python3
"""
SHA 로 고정된 Motion-13 Pick/Place leg 를 자동 재시도 없이 1회 전송한다.

leg 는 A/B/C 이며 경계는 gripper 동작 지점이다. 접촉은 이 sender 가 아니라
load/current 감시가 있는 별도 gripper 명령에서 일어난다
(`tools/plan_buffered_pick_place_leg.py` 의 서두 참조).

계획 artifact 전체를 같은 생성기로 다시 계산해 정확히 일치할 때만 허용한다.
SHA 만으로는 다른 anchor 로 만든 유효한 계획을 구분하지 못한다.

승인 문구는 leg 마다 다르다. 순서를 건너뛰거나 잘못된 leg 를 보내는 것을
사람의 주의력이 아니라 문자열 대조로 막는다.
"""

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
from plan_buffered_pick_place_leg import (
    LEG_DEFINITIONS,
    PHASE,
    STATUS,
    build_plan,
)


ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
CONFIRMATIONS = {
    "A": "EXECUTE_MOTION13_LEG_A_Q0_TO_PICK_GRASP_ONCE",
    "B": "EXECUTE_MOTION13_LEG_B_GRASP_TO_PLACE_ONCE",
    "C": "EXECUTE_MOTION13_LEG_C_PLACE_TO_Q0_ONCE",
}
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 90.0
START_TOLERANCES_RAD = (0.050, 0.055, 0.050, 0.050, 0.050)
TARGET_TOLERANCES_RAD = START_TOLERANCES_RAD


@dataclass(frozen=True, slots=True)
class PickPlaceLegPlan:
    path: Path
    sha256: str
    leg: str
    arm_joint_names: tuple[str, ...]
    anchor_positions_rad: tuple[float, ...]
    target_positions_rad: tuple[float, ...]
    target_name: str
    waypoints: tuple[PlanWaypoint, ...]
    duration_ms: int
    sample_count: int
    gripper_action_after: str | None
    anchor_deviation_raw: tuple[int, ...]


def load_pick_place_leg_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
    manifest_path: Path,
    require_deployed: bool = True,
) -> PickPlaceLegPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != STATUS:
        raise ValueError("plan status is not the reviewed pick/place leg status")
    if document.get("phase") != PHASE:
        raise ValueError("plan phase is not the reviewed pick/place leg phase")
    leg = document.get("leg")
    if leg not in LEG_DEFINITIONS:
        raise ValueError("plan leg is not one of the reviewed legs")
    for field in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
        "buffered_frame_encoded",
    ):
        if document.get(field) is not False:
            raise ValueError(f"plan must keep {field}=false")

    gate = document.get("firmware_deployment_gate")
    if not isinstance(gate, dict):
        raise ValueError("plan firmware deployment gate is invalid")
    if require_deployed and gate.get("deployed") is not True:
        raise ValueError("firmware candidate is not deployed")
    if gate.get("motion_authorized") is not False:
        raise ValueError("firmware gate must keep motion_authorized=false")

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
    if anchor.get("expected_start_pose") != LEG_DEFINITIONS[leg]["start_pose"]:
        raise ValueError("plan start pose does not match the leg definition")

    # 계획 전체를 같은 생성기로 다시 만들어 정확히 일치하는지 본다.
    rebuilt = build_plan(
        calibration_path,
        contract_path,
        manifest_path,
        leg,
        tuple(anchor_raw),
    )
    if rebuilt != document:
        raise ValueError("plan is not exactly reproducible from its inputs")

    arm_names = tuple(document["joint_names"][:5])
    anchor_positions = tuple(float(v) for v in anchor["positions_rad"])
    target_positions = tuple(
        float(v) for v in document["target"]["positions_rad"]
    )
    if not all(math.isfinite(v) for v in anchor_positions + target_positions):
        raise ValueError("plan endpoints must be finite")

    waypoints = tuple(
        PlanWaypoint(
            time_from_start_ms=index * document["resampling"]["period_ms"],
            positions_rad=tuple(
                value / 1_000_000.0 for value in sample["positions_urad"][:5]
            ),
        )
        for index, sample in enumerate(document["resampling"]["samples"])
    )
    if len(waypoints) != document["resampling"]["sample_count"]:
        raise ValueError("plan sample count does not match its waypoints")

    return PickPlaceLegPlan(
        path=path,
        sha256=digest,
        leg=leg,
        arm_joint_names=arm_names,
        anchor_positions_rad=anchor_positions[:5],
        target_positions_rad=target_positions[:5],
        target_name=document["target"]["name"],
        waypoints=waypoints,
        duration_ms=document["analytic_profile"]["duration_ms"],
        sample_count=document["resampling"]["sample_count"],
        gripper_action_after=document["gripper_action_after_leg"],
        anchor_deviation_raw=tuple(int(v) for v in anchor["deviation_raw"]),
    )


def main() -> int:
    from control_msgs.action import FollowJointTrajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws" / "src" / "single_arm_bridge"
    parser = argparse.ArgumentParser(description=__doc__)
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
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root
        / "artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015"
        / "full_pick_place_plan_only_manifest.json",
    )
    arguments = parser.parse_args()

    document = json.loads(arguments.plan.read_text(encoding="utf-8"))
    declared_leg = document.get("leg")
    if declared_leg not in CONFIRMATIONS:
        parser.error("plan does not declare one of the reviewed legs")
    if arguments.confirmation != CONFIRMATIONS[declared_leg]:
        parser.error(
            f"leg {declared_leg} requires its own confirmation: "
            f"{CONFIRMATIONS[declared_leg]}"
        )

    plan = load_pick_place_leg_plan(
        arguments.plan,
        arguments.expected_sha256,
        arguments.calibration,
        arguments.contract,
        arguments.manifest,
    )
    print(f"PLAN_SHA256={plan.sha256}")
    print(f"LEG={plan.leg}")
    print(f"PLAN_TARGET={plan.target_name}")
    print(f"PLAN_DURATION_MS={plan.duration_ms}")
    print(f"PLAN_SAMPLE_COUNT={plan.sample_count}")
    print(f"ANCHOR_DEVIATION_RAW={list(plan.anchor_deviation_raw)}")
    print("PLAN_GATE=PASS")

    rclpy.init()
    node = Node(f"motion13_pick_place_leg_{plan.leg.lower()}_once")
    action_client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
    try:
        current = wait_joint_state(node, plan.arm_joint_names)
        start_error = validate_fresh_start(
            current, plan.anchor_positions_rad, START_TOLERANCES_RAD
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
            result_timeout_s=ACTION_RESULT_TIMEOUT_S,
        )
        evidence = validate_action_terminal(status, result)
        final_positions = wait_joint_state(node, plan.arm_joint_names)
        target_error = validate_fresh_start(
            final_positions, plan.target_positions_rad, TARGET_TOLERANCES_RAD
        )
        print(
            "ACTION_TERMINAL_PASS "
            f"status={evidence.action_status} "
            f"error_code={evidence.error_code} "
            f"maximum_apply_lateness_ms={evidence.maximum_apply_lateness_ms} "
            f"post_settle_max_error_raw={evidence.post_settle_max_error_raw}"
        )
        if evidence.terminal_diagnostics:
            print(f"TERMINAL_DIAGNOSTICS={evidence.terminal_diagnostics}")
        print(f"TARGET_MAX_ERROR_RAD={target_error:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")
        if plan.gripper_action_after is not None:
            print(f"NEXT_GRIPPER_ACTION={plan.gripper_action_after}")
        print(f"MOTION13_PICK_PLACE_LEG_{plan.leg}_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
