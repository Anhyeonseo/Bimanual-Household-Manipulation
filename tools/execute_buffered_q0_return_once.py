#!/usr/bin/env python3
"""
SHA 로 고정된 q0 복귀 buffered Action 을 자동 재시도 없이 1회 전송한다.

Motion-11 은 팔을 Pick pregrasp 에 남긴다. 이 sender 는
`plan_buffered_q0_return.py` 가 만든 단일 leg 복귀 계획을 실행해 팔을 q0 로
내린다. 계획 artifact 전체를 같은 생성기로 다시 계산해 정확히 일치할 때만
허용하며, 계획 SHA 만 믿지 않는다.

0x00022C00 부터 firmware 는 apply lateness 분포를 terminal 프레임에만
싣는다. 0x00022800 은 refill 응답에도 실어 blocking 전송을 4.688 ms 에서
7.118 ms 로 늘렸고, 그것이 5 ms 예산을 넘겨 첫 sample 에서 죽었다.
이 실행은 그 분포의 첫 실측이기도 하다.
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
from plan_buffered_q0_return import PHASE, STATUS, build_plan


ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
CONFIRMATION = "EXECUTE_MOTION12_Q0_RETURN_ONCE"
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 90.0
START_TOLERANCES_RAD = (0.050, 0.055, 0.050, 0.050, 0.050)
TARGET_TOLERANCES_RAD = START_TOLERANCES_RAD


@dataclass(frozen=True, slots=True)
class Q0ReturnPlan:
    path: Path
    sha256: str
    arm_joint_names: tuple[str, ...]
    anchor_positions_rad: tuple[float, ...]
    target_positions_rad: tuple[float, ...]
    waypoints: tuple[PlanWaypoint, ...]
    duration_ms: int
    sample_count: int


def load_q0_return_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
    require_deployed: bool = True,
) -> Q0ReturnPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != STATUS:
        raise ValueError("plan status is not the reviewed q0 return status")
    if document.get("phase") != PHASE:
        raise ValueError("plan phase is not the reviewed q0 return phase")
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

    # 계획 전체를 같은 생성기로 다시 만들어 정확히 일치하는지 본다.
    # SHA 만으로는 다른 입력으로 만든 유효한 계획을 구분하지 못한다.
    # tracking_rate_raw_s 도 계획이 실제로 쓴 값을 그대로 넣어야 한다 --
    # duration_ms 는 고정하면서 rate 만 기본값(50)으로 되돌리면, 빠른
    # rate 로 고른 짧은 duration 을 느린 rate 모델로 재검증하게 되어
    # 모델 peak 오차가 부풀려져 거짓으로 재현 불가 판정이 난다
    # (2026-08-07 밤 300raw/s q0 복귀에서 실제로 겪음).
    rebuilt = build_plan(
        calibration_path,
        contract_path,
        tuple(anchor_raw),
        document["analytic_profile"]["duration_ms"],
        document["physical_tracking_model"]["conservative_rate_raw_s"],
    )
    # 자동 선택 표시는 재계산 시 명시 지정이 되므로 비교에서 제외한다.
    rebuilt["analytic_profile"]["duration_selected_automatically"] = document[
        "analytic_profile"
    ]["duration_selected_automatically"]
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

    return Q0ReturnPlan(
        path=path,
        sha256=digest,
        arm_joint_names=arm_names,
        anchor_positions_rad=anchor_positions[:5],
        target_positions_rad=target_positions[:5],
        waypoints=waypoints,
        duration_ms=document["analytic_profile"]["duration_ms"],
        sample_count=document["resampling"]["sample_count"],
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
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact q0 return one-shot confirmation is required")

    plan = load_q0_return_plan(
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
    node = Node("motion12_buffered_q0_return_once")
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
        print(f"Q0_MAX_ERROR_RAD={target_error:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")
        print("MOTION12_BUFFERED_Q0_RETURN_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
