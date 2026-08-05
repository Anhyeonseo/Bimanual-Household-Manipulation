#!/usr/bin/env python3
"""
갓 계획된 segment leg 를 자동 재시도 없이 1회 전송한다.

Motion-13 의 leg 실행기와 같은 규율이되 경로 출처가 다르다. 그쪽은 SHA 로
고정된 manifest 를, 이쪽은 **이번 세션에 방금 계획된** segment 파일을 쓴다.

그래서 검증이 하나 더 필요하다. manifest 는 소스에 박힌 상수와 대조하면
되지만, 갓 만든 파일은 그럴 대상이 없다. 대신 세 가지를 요구한다.

  1. 계획 artifact 전체를 같은 생성기로 다시 계산해 정확히 일치할 것
  2. 그 재계산이 참조하는 segment 파일이 **여전히 같은 digest** 일 것
  3. segment 파일 자체가 이번 세션에 collision-checked 로 통과했을 것

2번이 핵심이다. 계획과 실행 사이에 segment 파일이 바뀌면 SHA 가 어긋나
거부된다. 계획 당시 검사된 경로가 아닌 것을 실행할 수 없다.
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
from plan_buffered_segment_leg import PHASE, STATUS, build_plan


ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
CONFIRMATION = "EXECUTE_MOTION14_FRESH_SEGMENT_LEG_ONCE"
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 90.0
START_TOLERANCES_RAD = (0.050, 0.055, 0.050, 0.050, 0.050)
TARGET_TOLERANCES_RAD = START_TOLERANCES_RAD


@dataclass(frozen=True, slots=True)
class SegmentLegPlan:
    path: Path
    sha256: str
    arm_joint_names: tuple[str, ...]
    anchor_positions_rad: tuple[float, ...]
    target_positions_rad: tuple[float, ...]
    waypoints: tuple[PlanWaypoint, ...]
    duration_ms: int
    sample_count: int
    segment_route: dict
    anchor_deviation_raw: tuple[int, ...]


def load_segment_leg_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
    require_deployed: bool = True,
) -> SegmentLegPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != STATUS:
        raise ValueError("plan status is not the reviewed segment leg status")
    if document.get("phase") != PHASE:
        raise ValueError("plan phase is not the reviewed segment leg phase")
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

    route = document.get("segment_route")
    if not isinstance(route, dict):
        raise ValueError("plan does not record its segment route")
    segments_path = Path(route["path"])
    if not segments_path.is_file():
        raise ValueError(f"segment route file is gone: {segments_path}")
    # 계획 당시와 같은 파일인지. 바뀌었다면 검사된 경로가 아니다.
    current = sha256_file(segments_path)
    if current.lower() != str(route["sha256"]).lower():
        raise ValueError(
            "segment route changed since the plan was made: "
            f"planned={route['sha256']} now={current}"
        )

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

    rebuilt = build_plan(
        calibration_path,
        contract_path,
        segments_path,
        route["sha256"],
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

    return SegmentLegPlan(
        path=path,
        sha256=digest,
        arm_joint_names=arm_names,
        anchor_positions_rad=anchor_positions[:5],
        target_positions_rad=target_positions[:5],
        waypoints=waypoints,
        duration_ms=document["analytic_profile"]["duration_ms"],
        sample_count=document["resampling"]["sample_count"],
        segment_route=route,
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
    arguments = parser.parse_args()
    if arguments.confirmation != CONFIRMATION:
        parser.error("exact fresh segment leg confirmation is required")

    plan = load_segment_leg_plan(
        arguments.plan,
        arguments.expected_sha256,
        arguments.calibration,
        arguments.contract,
    )
    print(f"PLAN_SHA256={plan.sha256}")
    print(f"SEGMENT_ROUTE={plan.segment_route['path']}")
    print(f"SEGMENT_ROUTE_SHA256={plan.segment_route['sha256']}")
    print(f"SEGMENT_STATUS={plan.segment_route['status']}")
    print(f"SEGMENT_TARGET_NAME={plan.segment_route['target_name']}")
    print(f"SEGMENT_COUNT={plan.segment_route['segment_count']}")
    print(f"PLAN_DURATION_MS={plan.duration_ms}")
    print(f"PLAN_SAMPLE_COUNT={plan.sample_count}")
    print(f"ANCHOR_DEVIATION_RAW={list(plan.anchor_deviation_raw)}")
    print("PLAN_GATE=PASS")

    rclpy.init()
    node = Node("motion14_fresh_segment_leg_once")
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
        print("MOTION14_FRESH_SEGMENT_LEG_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
