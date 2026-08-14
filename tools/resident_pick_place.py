#!/usr/bin/env python3
"""H1: `move_to()`/`return_to_q0()`의 상주 세션 버전.

`run_grasp_repeatability_pilot.move_to()`가 leg 하나마다 subprocess 3개
(`ros_moveit_plan_pregrasp_segments.py`, `plan_buffered_segment_leg.py`,
`execute_buffered_segment_leg_once.py`)를 띄우는 대신, 이 모듈은 그 세
스크립트가 이미 분리해 둔 순수 함수를 `ResidentArmSession` 하나로 직접
호출한다. **로직을 다시 구현하지 않는다** — 원본 스크립트의 `main()`이
하던 검증·계획·실행·출력 조립을 그대로 재사용하고, `rclpy.init`/노드
생성/서비스·action 탐색만 세션 단위로 끌어올린다.

원본 CLI 도구는 건드리지 않는다. 증거·회귀 경로는 그대로 subprocess 로
계속 동작한다. 이 모듈은 별도 진입점(`run_pick_place_once_resident.py`)에서만
쓰인다.

**telemetry 호환성이 핵심 제약이다.** `buffered_leg_telemetry.parse_leg_telemetry`는
텍스트에서 `KEY=VALUE`를 정규식으로 뽑는다. 아래 각 `*_resident` 함수는
원본 스크립트의 `main()`이 찍던 것과 **글자 그대로 같은 줄**을 만든다 —
telemetry 파서를 전혀 바꾸지 않고도 계속 동작하게 하려는 것이다.
"""

from __future__ import annotations

from hashlib import sha256
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

import execute_buffered_q0_return_once as q0_exec  # noqa: E402
import execute_buffered_segment_leg_once as seg_exec  # noqa: E402
import plan_buffered_q0_return as q0_plan  # noqa: E402
import plan_buffered_segment_leg as seg_plan  # noqa: E402
import ros_moveit_plan_pregrasp_segments as moveit_plan  # noqa: E402
from execute_buffered_action_plan_once import (  # noqa: E402
    build_goal,
    send_goal_once,
    validate_action_terminal,
    validate_fresh_start,
)

from resident_arm_session import ResidentArmSession  # noqa: E402


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


class ResidentLegError(RuntimeError):
    """leg 하나(계획 또는 실행)가 실패했다. 원본 subprocess 예외와 같은 역할."""


def plan_segments_resident(
    session: ResidentArmSession,
    *,
    calibration_path: Path,
    start: tuple[float, ...],
    target_name: str,
    output: Path,
    source_plan: Path | None = None,
    target_joints: tuple[float, ...] | None = None,
    max_joint_step_rad: float = moveit_plan.DEFAULT_MAX_JOINT_STEP_RAD,
    execution_step_limit: float | None = None,
) -> dict:
    """`ros_moveit_plan_pregrasp_segments.main()`과 동일한 검증·계획·출력.

    반환/기록되는 JSON은 그 스크립트가 쓰는 것과 스키마가 바이트 단위로
    같다 — `plan_buffered_segment_leg.build_plan()`이 그 파일을 다시 읽어
    SHA 로 대조하므로 형식이 어긋나면 다음 단계에서 바로 거부된다.
    """
    if (source_plan is None) == (target_joints is None):
        raise ValueError("give exactly one of source_plan or target_joints")
    effective_target_name = target_name
    if target_joints is not None:
        effective_target_name = moveit_plan.EXPLICIT_TARGET_NAME
    if execution_step_limit is None:
        execution_step_limit = max_joint_step_rad

    target = (
        target_joints
        if target_joints is not None
        else moveit_plan.load_target(source_plan, target_name)
    )
    limits = moveit_plan.arm_limits(calibration_path)
    moveit_plan.validate_positions("start", start, limits)
    moveit_plan.validate_positions(effective_target_name, target, limits)
    candidates = moveit_plan.interpolate_segments(start, target, max_joint_step_rad)

    segments = [
        moveit_plan.plan_segment(
            session.moveit_client,
            session.node,
            index,
            segment_start,
            segment_target,
            effective_target_name,
        )
        for index, (segment_start, segment_target) in enumerate(candidates, start=1)
    ]
    passed = all(segment["success"] for segment in segments)
    status_prefix = effective_target_name.upper()
    result = {
        "schema_version": 1,
        "status": (
            f"{status_prefix}_SEGMENT_PLAN_ONLY_PASS"
            if passed
            else f"{status_prefix}_SEGMENT_PLAN_ONLY_FAIL"
        ),
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "service": moveit_plan.PLAN_SERVICE,
        "group": moveit_plan.GROUP_NAME,
        "target_name": effective_target_name,
        "joint_names": list(moveit_plan.ARM_JOINTS),
        "source_plan": None if source_plan is None else str(source_plan),
        "explicit_target_positions_rad": (
            None if target_joints is None else list(target_joints)
        ),
        "joint_goal_tolerance_rad": moveit_plan.JOINT_GOAL_TOLERANCE_RAD,
        "calibration": str(calibration_path),
        "interpolation_joint_step_rad": max_joint_step_rad,
        "max_joint_step_rad": execution_step_limit,
        "recommended_execution_duration_s": moveit_plan.DEFAULT_DURATION_S,
        "segments": segments,
    }
    if not passed:
        raise ResidentLegError(
            f"{status_prefix}_SEGMENT_PLAN_FAIL segments={segments}"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def plan_leg_resident(
    *,
    calibration_path: Path,
    contract_path: Path,
    segments_path: Path,
    segments_sha256: str,
    anchor_raw: tuple[int, ...],
    output: Path,
    tracking_rate_raw_s: float | None = None,
) -> tuple[dict, str]:
    """`plan_buffered_segment_leg.main()`과 동일한 계산·출력.

    반환값은 `(document, text)`다. `text`는 원본이 stdout 에 찍던 줄과
    글자 그대로 같아서 `parse_leg_telemetry`가 그대로 먹는다.
    """
    kwargs = {}
    if tracking_rate_raw_s is not None:
        kwargs["tracking_rate_raw_s"] = tracking_rate_raw_s
    document = seg_plan.build_plan(
        calibration_path,
        contract_path,
        segments_path,
        segments_sha256,
        anchor_raw,
        **kwargs,
    )
    encoded = json.dumps(document, indent=2, sort_keys=True) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(encoded, encoding="utf-8")

    route = document["segment_route"]
    tracking = document["physical_tracking_model"]["legs"][
        "anchor_to_segment_target"
    ]
    lines = [
        f"MOTION14_SEGMENT_LEG_PLAN={output}",
        f"STATUS={document['status']}",
        f"SEGMENT_ROUTE={route['path']}",
        f"SEGMENT_STATUS={route['status']}  target={route['target_name']}",
        f"SEGMENT_COUNT={route['segment_count']}",
        f"ANCHOR_DEVIATION_RAW={document['anchor']['deviation_raw']}",
        f"TARGET_RAW={document['target']['raw']}",
        f"DURATION_MS={document['analytic_profile']['duration_ms']}",
        f"SAMPLES={document['resampling']['sample_count']}",
        "TRACKING_RATE_RAW_S="
        f"{document['physical_tracking_model']['conservative_rate_raw_s']:g}",
        f"MODELED_PEAK_ERROR_RAW={tracking['maximum_peak_error_raw']:.3f}",
        "MODELED_TERMINAL_ERROR_RAW="
        f"{tracking['maximum_terminal_error_raw']:.3f}",
        f"EXECUTION_API_USED={int(document['execution_api_used'])}",
        f"MOTION_AUTHORIZED={int(document['motion_authorized'])}",
        f"SHA256={sha256(encoded.encode('utf-8')).hexdigest()}",
    ]
    return document, "\n".join(lines) + "\n"


def execute_leg_resident(
    session: ResidentArmSession,
    *,
    plan_path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
) -> str:
    """`execute_buffered_segment_leg_once.main()`과 동일한 검증·실행·출력.

    rclpy 라이프사이클(노드/action client 생성·파괴)만 세션이 대신한다.
    나머지 — 재현성 검사, fresh-start 게이트, 1회 전송, terminal 검증 —
    는 원본이 쓰는 함수를 그대로 호출한다.
    """
    plan = seg_exec.load_segment_leg_plan(
        plan_path, expected_sha256, calibration_path, contract_path
    )
    lines = [
        f"PLAN_SHA256={plan.sha256}",
        f"SEGMENT_ROUTE={plan.segment_route['path']}",
        f"SEGMENT_ROUTE_SHA256={plan.segment_route['sha256']}",
        f"SEGMENT_STATUS={plan.segment_route['status']}",
        f"SEGMENT_TARGET_NAME={plan.segment_route['target_name']}",
        f"SEGMENT_COUNT={plan.segment_route['segment_count']}",
        f"PLAN_DURATION_MS={plan.duration_ms}",
        f"PLAN_SAMPLE_COUNT={plan.sample_count}",
        f"PLAN_TRACKING_RATE_RAW_S={plan.tracking_rate_raw_s:g}",
        f"ANCHOR_DEVIATION_RAW={list(plan.anchor_deviation_raw)}",
        "PLAN_GATE=PASS",
    ]

    current = session.wait_joint_state(plan.arm_joint_names)
    start_error = validate_fresh_start(
        current, plan.anchor_positions_rad, seg_exec.START_TOLERANCES_RAD
    )
    lines.append(f"FRESH_START_MAX_ERROR_RAD={start_error:.6f}")
    lines.append("FRESH_START_GATE=PASS")

    lines.append("ACTION_SEND_COUNT=1")
    status, result = send_goal_once(
        session.node,
        session.action_client,
        build_goal(plan),
        result_timeout_s=seg_exec.ACTION_RESULT_TIMEOUT_S,
    )
    evidence = validate_action_terminal(status, result)
    final_positions = session.wait_joint_state(plan.arm_joint_names)
    target_error = validate_fresh_start(
        final_positions, plan.target_positions_rad, seg_exec.TARGET_TOLERANCES_RAD
    )
    lines.append(
        "ACTION_TERMINAL_PASS "
        f"status={evidence.action_status} "
        f"error_code={evidence.error_code} "
        f"maximum_apply_lateness_ms={evidence.maximum_apply_lateness_ms} "
        f"post_settle_max_error_raw={evidence.post_settle_max_error_raw}"
    )
    if evidence.terminal_diagnostics:
        lines.append(f"TERMINAL_DIAGNOSTICS={evidence.terminal_diagnostics}")
    lines.append(f"TARGET_MAX_ERROR_RAD={target_error:.6f}")
    lines.append("AUTOMATIC_RETRY_COUNT=0")
    lines.append("MOTION14_FRESH_SEGMENT_LEG_ONCE_PASS")
    return "\n".join(lines) + "\n"


def plan_q0_return_resident(
    *,
    calibration_path: Path,
    contract_path: Path,
    anchor_raw: tuple[int, ...],
    output: Path,
    tracking_rate_raw_s: float | None = None,
) -> tuple[dict, str]:
    """`plan_buffered_q0_return.main()`과 동일한 계산·출력."""
    kwargs: dict[str, object] = {}
    if tracking_rate_raw_s is not None:
        kwargs["tracking_rate_raw_s"] = tracking_rate_raw_s
    document = q0_plan.build_plan(calibration_path, contract_path, anchor_raw, **kwargs)
    encoded = (
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(encoded, encoding="utf-8")

    profile = document["analytic_profile"]
    tracking = document["physical_tracking_model"]["legs"]["anchor_to_q0"]
    lines = [
        f"MOTION12_Q0_RETURN_PLAN={output}",
        f"STATUS={document['status']}",
        f"DURATION_MS={profile['duration_ms']}",
        f"DURATION_AUTOSELECTED={int(profile['duration_selected_automatically'])}",
        f"SAMPLES={document['resampling']['sample_count']}",
        "MAXIMUM_SAMPLE_STEP_RAD="
        f"{document['resampling']['maximum_sample_step_rad']:.9f}",
        f"MODELED_PEAK_ERROR_RAW={tracking['maximum_peak_error_raw']:.3f}",
        f"MODELED_TERMINAL_ERROR_RAW={tracking['maximum_terminal_error_raw']:.3f}",
        f"EXECUTION_API_USED={int(document['execution_api_used'])}",
        f"MOTION_AUTHORIZED={int(document['motion_authorized'])}",
        f"SHA256={sha256(encoded.encode('utf-8')).hexdigest()}",
    ]
    return document, "\n".join(lines) + "\n"


def execute_q0_return_resident(
    session: ResidentArmSession,
    *,
    plan_path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
) -> str:
    """`execute_buffered_q0_return_once.main()`과 동일한 검증·실행·출력."""
    plan = q0_exec.load_q0_return_plan(
        plan_path, expected_sha256, calibration_path, contract_path
    )
    lines = [
        f"PLAN_SHA256={plan.sha256}",
        f"PLAN_DURATION_MS={plan.duration_ms}",
        f"PLAN_SAMPLE_COUNT={plan.sample_count}",
        "PLAN_GATE=PASS",
    ]

    current = session.wait_joint_state(plan.arm_joint_names)
    start_error = validate_fresh_start(
        current, plan.anchor_positions_rad, q0_exec.START_TOLERANCES_RAD
    )
    lines.append(f"FRESH_START_MAX_ERROR_RAD={start_error:.6f}")
    lines.append("FRESH_START_GATE=PASS")

    lines.append("ACTION_SEND_COUNT=1")
    status, result = send_goal_once(
        session.node,
        session.action_client,
        build_goal(plan),
        result_timeout_s=q0_exec.ACTION_RESULT_TIMEOUT_S,
    )
    evidence = validate_action_terminal(status, result)
    final_positions = session.wait_joint_state(plan.arm_joint_names)
    target_error = validate_fresh_start(
        final_positions, plan.target_positions_rad, q0_exec.TARGET_TOLERANCES_RAD
    )
    lines.append(
        "ACTION_TERMINAL_PASS "
        f"status={evidence.action_status} "
        f"error_code={evidence.error_code} "
        f"maximum_apply_lateness_ms={evidence.maximum_apply_lateness_ms} "
        f"post_settle_max_error_raw={evidence.post_settle_max_error_raw}"
    )
    if evidence.terminal_diagnostics:
        lines.append(f"TERMINAL_DIAGNOSTICS={evidence.terminal_diagnostics}")
    lines.append(f"Q0_MAX_ERROR_RAD={target_error:.6f}")
    lines.append("AUTOMATIC_RETRY_COUNT=0")
    lines.append("MOTION12_BUFFERED_Q0_RETURN_ONCE_PASS")
    return "\n".join(lines) + "\n"


def _record(log, *, tag: str, run_index: int, text: str, ok: bool) -> None:
    if log is not None:
        log.record(tag=tag, run_index=run_index, text=text, ok=ok)


def move_to_resident(
    session: ResidentArmSession,
    workdir: Path,
    tag: str,
    endpoint: Path,
    target_name: str,
    calibration_path: Path,
    arm_names: tuple[str, ...],
    calibration,
    log=None,
    run_index: int = 0,
    tracking_rate_raw_s: float | None = None,
) -> None:
    """`run_grasp_repeatability_pilot.move_to()`의 상주 세션 버전.

    subprocess 3개(ros2 노드 생성 + MoveIt/action 탐색 반복)를 없앤 것
    말고는 그 함수와 동일한 순서·검증·계약을 따른다.
    """
    gripper_name = arm_names[0].replace("base_joint", "gripper_joint")
    # 원본 `read_joint_state(names, timeout_s=20.0)`와 같은 여유. 이 호출은
    # anchor 계산용이지 fresh-start 게이트가 아니므로 execute 단계
    # (`STATE_TIMEOUT_S=5.0`)보다 넉넉하게 둔다.
    observed = session.wait_joint_state((*arm_names, gripper_name), timeout_s=20.0)
    start = observed[:5]
    anchor = tuple(
        round(joint.zero_raw + joint.direction * value * 4096.0 / (2.0 * math.pi))
        for joint, value in zip(calibration.joints, observed, strict=True)
    )

    segments = workdir / f"{tag}_segments.json"
    plan_segments_resident(
        session,
        calibration_path=calibration_path,
        start=start,
        target_name=target_name,
        output=segments,
        source_plan=endpoint,
    )

    # 여기서부터는 넘겨받은 `calibration_path`가 아니라 bridge package 의
    # 자체 사본을 쓴다. 원본 `move_to()`가 `plan_buffered_segment_leg.py`와
    # `execute_buffered_segment_leg_once.py`를 `--calibration` 없이 호출해
    # 그 두 스크립트의 CLI 기본값(PACKAGE/config)이 적용되기 때문이다 —
    # MoveIt 세그먼트 계획(관절 한계 검사)과 buffered leg 실행(raw 변환,
    # 실제 하드웨어 계약)이 서로 다른 calibration 사본을 참조하는 것이
    # 기존 동작이다. 여기서 같은 값으로 합치면 조용히 다른 동작이 된다.
    bridge_calibration_path = PACKAGE / "config" / "single_arm_calibration.json"
    contract_path = PACKAGE / "config" / "buffered_trajectory_contract.json"

    leg = workdir / f"{tag}_leg.json"
    _, planned = plan_leg_resident(
        calibration_path=bridge_calibration_path,
        contract_path=contract_path,
        segments_path=segments,
        segments_sha256=sha256_file(segments),
        anchor_raw=anchor,
        output=leg,
        tracking_rate_raw_s=tracking_rate_raw_s,
    )

    try:
        executed = execute_leg_resident(
            session,
            plan_path=leg,
            expected_sha256=sha256_file(leg),
            calibration_path=bridge_calibration_path,
            contract_path=contract_path,
        )
    except Exception as error:
        text = planned + str(error)
        (workdir / f"{tag}_execute.txt").write_text(text, encoding="utf-8")
        _record(log, tag=tag, run_index=run_index, text=text, ok=False)
        raise
    text = planned + executed
    (workdir / f"{tag}_execute.txt").write_text(text, encoding="utf-8")
    _record(log, tag=tag, run_index=run_index, text=text, ok=True)


def return_to_q0_resident(
    session: ResidentArmSession,
    workdir: Path,
    arm_names: tuple[str, ...],
    calibration,
    log=None,
    run_index: int = 0,
    tracking_rate_raw_s: float | None = None,
) -> None:
    """`run_grasp_repeatability_pilot.return_to_q0()`의 상주 세션 버전.

    원본이 `plan_buffered_q0_return.py`/`execute_buffered_q0_return_once.py`를
    `--calibration`/`--contract` 없이 호출하므로 그 두 스크립트의 CLI
    기본값(PACKAGE/config)이 적용된다 — `move_to_resident`와 같은 이유로
    여기서도 bridge package 사본을 쓴다.
    """
    bridge_calibration_path = PACKAGE / "config" / "single_arm_calibration.json"
    contract_path = PACKAGE / "config" / "buffered_trajectory_contract.json"

    gripper_name = arm_names[0].replace("base_joint", "gripper_joint")
    # 원본 `read_joint_state(names, timeout_s=20.0)`와 같은 여유. 이 호출은
    # anchor 계산용이지 fresh-start 게이트가 아니므로 execute 단계
    # (`STATE_TIMEOUT_S=5.0`)보다 넉넉하게 둔다.
    observed = session.wait_joint_state((*arm_names, gripper_name), timeout_s=20.0)
    anchor = tuple(
        round(joint.zero_raw + joint.direction * value * 4096.0 / (2.0 * math.pi))
        for joint, value in zip(calibration.joints, observed, strict=True)
    )

    plan = workdir / "q0_return.json"
    _, planned = plan_q0_return_resident(
        calibration_path=bridge_calibration_path,
        contract_path=contract_path,
        anchor_raw=anchor,
        output=plan,
        tracking_rate_raw_s=tracking_rate_raw_s,
    )

    try:
        executed = execute_q0_return_resident(
            session,
            plan_path=plan,
            expected_sha256=sha256_file(plan),
            calibration_path=bridge_calibration_path,
            contract_path=contract_path,
        )
    except Exception as error:
        text = planned + str(error)
        (workdir / "q0_return_execute.txt").write_text(text, encoding="utf-8")
        _record(log, tag="q0_return", run_index=run_index, text=text, ok=False)
        raise
    text = planned + executed
    (workdir / "q0_return_execute.txt").write_text(text, encoding="utf-8")
    _record(log, tag="q0_return", run_index=run_index, text=text, ok=True)
