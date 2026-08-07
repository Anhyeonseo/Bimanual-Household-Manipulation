#!/usr/bin/env python3
"""Motion-13 — 연속 Pick/Place 를 buffered leg 3개로 계획한다. plan-only.

전체 경로는 SHA 로 고정된 collision-checked manifest 의 7개 phase 이며,
각 phase 는 관절공간 정확한 직선이고 이음매 불일치가 0.0 이다. 따라서
전체 동작은 key pose 8개를 지나는 조각별 직선이고, gripper 동작 2개가
그 사이에 낀다.

**왜 단일 Action 이 아닌가.**

`Servo_MotionSafetyBegin`/`Poll` 은 비버퍼드 경로(`Host_ServiceBinaryMotion`)
에만 있다. buffered 실행에는 load/current 감시가 없다 — 0x00022700 에서
servo read 가 host UART 처리를 굶겨 제거했기 때문이다. 단일 Action 으로
gripper 를 stream 안에서 닫으면 물체를 문 뒤 약 60초 동안 gripper 가 명령
위치에 도달하지 못한 채 stall 하고, 그 구간 전체가 무감시가 된다.

leg 경계를 gripper 동작 지점에 두면 접촉은 load/current 감시가 있는 기존
gripper 명령 경로에서 일어난다. 사이에 q0 복귀가 없으므로 팔 입장에서는
여전히 연속이다.

각 leg 는 자기 시작 pose 를 manifest 에서 가져와 실제 anchor 와 대조하고,
벗어나면 계획을 거부한다. collision-checked 경로 위에 있음을 계획 시점에
확인하는 것이 목적이다.

이 도구는 계획만 만든다. 실행 API 를 쓰지 않고 프레임도 만들지 않는다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path

from single_arm_bridge.action_validation import (
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (
    SAMPLE_PERIOD_MS,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.buffered_trajectory import (
    load_buffered_trajectory_contract,
)
from single_arm_bridge.calibration import load_calibration

from plan_buffered_q0_roundtrip import (
    minimum_jerk_unit_progress,
    radians_to_raw,
    sha256_file,
    simulate_admission_batches,
    simulate_firmware_output_raw,
)
from plan_buffered_pick_pregrasp import (
    CONSERVATIVE_TRACKING_RATE_RAW_S,
    MAXIMUM_MODELED_PEAK_ERROR_RAW,
    MAXIMUM_MODELED_TERMINAL_ERROR_RAW,
    TRACKING_SIMULATION_PERIOD_MS,
)


STATUS = "BUFFERED_PICK_PLACE_LEG_PLAN_ONLY_PASS"
PHASE = "motion13_continuous_pick_place"
PLAN_TICK_MS = 100_000

MANIFEST_SHA256 = (
    "7c0d44a96dbd4ff214bf9858f1adff183f5fdc9079256ab4534c58d3a73e6d5c"
)
MANIFEST_STATUS = "FULL_PICK_PLACE_PLAN_ONLY_PASS"
EXPECTED_PHASE_NAMES = (
    "q0_to_pick_pregrasp",
    "pick_pregrasp_to_grasp",
    "pick_grasp_to_lift20",
    "lift_to_place_pregrasp",
    "place_pregrasp_to_place",
    "place_to_retreat",
    "place_pregrasp_to_q0",
)

MINIMUM_DURATION_MS = 4_000
MAXIMUM_DURATION_MS = 180_000
DURATION_SEARCH_STEP_MS = 1_000
# Motion-12 와 같은 규율: 통과하는 최소 시간이 아니라 허용 peak 오차의
# 이 비율 이하가 되는 시간을 고른다. 최소 시간은 정의상 상한에 붙는다.
TRACKING_MARGIN_FRACTION = 0.70

# **이 저장소에서 팔이 느린 이유는 여기 하나다.**
#
# `select_duration_ms` 는 "서보가 초당 `CONSERVATIVE_TRACKING_RATE_RAW_S` raw
# 밖에 못 따라온다"고 가정한 모의추종으로 leg 시간을 정한다. 기본값 `50`
# 은 초당 4.4° 다. MoveIt 의 velocity scaling(0.15/0.20)은 이 경로에 아무
# 영향이 없다 — `plan_buffered_segment_leg` 가 MoveIt 타이밍을 버리고 여기서
# 다시 시간을 정하기 때문이다.
#
# 그런데 `50` 의 근거가 약하다. 2026-08-04 Motion-11 이 관측한 `60 raw/s` 는
# **post-terminal** 추종률 — 궤적이 끝난 뒤 뒤처진 팔이 따라잡던 속도지
# 서보의 능력치가 아니다. STS3215 에는 여유가 있다.
#
# **이 가정을 올리면 모델 게이트는 같이 느슨해진다.** peak/terminal 오차를
# 같은 rate 로 계산하기 때문에 self-consistent 하게 통과한다. 그래서 rate 를
# 올릴 때 실제로 지켜주는 것은 두 가지뿐이다:
#
#   1. `validate_buffered_trajectory` 의 관절 속도 상한 `0.5 rad/s`
#      (= 326 raw/s). 이건 rate 가정과 무관한 독립 게이트다.
#   2. 하드웨어에서 실측되는 post-settle `30 raw` 게이트.
#
# 상한을 `326` 아래에 여유를 남기고 정한다. 2026-08-07 실기로 200 raw/s
# 까지(post-settle 20~26raw, 하드 게이트 30까지 여유 4~10raw) 검증했고,
# 그 근거로 300 으로 올렸다 — 여유는 26 raw/s 로 줄어든다.
MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S = 300.0

# leg 시작 시 실제 anchor 가 계획된 시작 pose 에서 벗어날 수 있는 한계.
# buffered terminal 의 post-settle 게이트가 30 raw 이므로 그 위에 여유를
# 조금만 둔다. 넘으면 팔이 collision-checked 경로 위에 있지 않다는 뜻이다.
ANCHOR_DEVIATION_LIMIT_RAW = 40

# 각 leg 는 (시작 key pose, 지나갈 key pose 들) 이다. 이름은 manifest 의
# phase 경계에서 그대로 나온다.
LEG_DEFINITIONS = {
    "A": {
        "description": "q0 에서 물체를 잡을 자세까지",
        "start_pose": "q0",
        "waypoints": ("pick_pregrasp", "pick_grasp"),
        "gripper_action_after": "pick_close",
    },
    "B": {
        "description": "물체를 든 채 놓을 자세까지",
        "start_pose": "pick_grasp",
        "waypoints": ("lift20", "place_pregrasp", "place"),
        "gripper_action_after": "place_release",
    },
    "C": {
        "description": "물체를 놓은 뒤 q0 로 복귀",
        "start_pose": "place",
        "waypoints": ("retreat", "q0"),
        "gripper_action_after": None,
    },
}


def load_key_poses(manifest_path: Path) -> dict[str, tuple[float, ...]]:
    """SHA 로 고정된 manifest 에서 key pose 8개를 유도한다.

    pose 를 손으로 적지 않는다. collision-checked 경로에서만 나온다.
    """
    if sha256_file(manifest_path) != MANIFEST_SHA256:
        raise ValueError("pick/place manifest sha256 mismatch")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != MANIFEST_STATUS:
        raise ValueError("manifest status is not approved")
    for flag in ("motion_authorized", "execution_api_used",
                 "automatic_execution_permitted", "robot_target_available"):
        if manifest.get(flag) is not False:
            raise ValueError(f"manifest must keep {flag}=false")

    summaries = manifest.get("phase_summaries")
    if not isinstance(summaries, list) or len(summaries) != 7:
        raise ValueError("manifest must contain the seven reviewed phases")
    if tuple(entry["name"] for entry in summaries) != EXPECTED_PHASE_NAMES:
        raise ValueError("manifest phase order is not the reviewed order")

    repository_root = Path(__file__).resolve().parents[1]
    boundaries: list[tuple[float, ...]] = []
    for entry in summaries:
        source = repository_root / entry["source"]
        if sha256_file(source) != entry["source_sha256"]:
            raise ValueError(f"phase source sha256 mismatch: {entry['name']}")
        document = json.loads(source.read_text(encoding="utf-8"))
        if document.get("motion_authorized") is not False:
            raise ValueError(f"{entry['name']} must keep motion_authorized=false")
        segments = document["segments"]
        if len(segments) != entry["arm_segment_count"]:
            raise ValueError(f"{entry['name']} segment count disagrees")
        if any(
            segment.get("success") is not True
            or segment.get("moveit_error_code") != 1
            for segment in segments
        ):
            raise ValueError(f"{entry['name']} contains a failed segment")

        start = tuple(
            float(v) for v in segments[0]["expected_start_positions_rad"]
        )
        end = tuple(float(v) for v in segments[-1]["target_positions_rad"])
        # 각 phase 가 관절공간 직선인지 확인한다. 직선이 아니면 중간 경유점을
        # 버리고 minimum-jerk 로 잇는 이 계획기의 전제가 깨진다.
        for index, segment in enumerate(segments, start=1):
            fraction = index / len(segments)
            for a, b, value in zip(
                start, end,
                (float(v) for v in segment["target_positions_rad"]),
                strict=True,
            ):
                if abs(value - (a + fraction * (b - a))) > 1.0e-9:
                    raise ValueError(
                        f"{entry['name']} is not a straight joint-space path"
                    )
        if entry["reversed"]:
            start, end = end, start
        boundaries.append((start, end))

    for index, ((_, previous_end), (next_start, _)) in enumerate(
        zip(boundaries[:-1], boundaries[1:], strict=True)
    ):
        if any(
            abs(a - b) > 1.0e-12
            for a, b in zip(previous_end, next_start, strict=True)
        ):
            raise ValueError(
                f"phase chain breaks between {EXPECTED_PHASE_NAMES[index]} "
                f"and {EXPECTED_PHASE_NAMES[index + 1]}"
            )

    names = (
        "q0", "pick_pregrasp", "pick_grasp", "lift20",
        "place_pregrasp", "place", "retreat", "q0_final",
    )
    poses = [boundaries[0][0]] + [end for _, end in boundaries]
    key_poses = dict(zip(names, poses, strict=True))
    if any(abs(value) > 1.0e-12 for value in key_poses["q0"]):
        raise ValueError("manifest does not start at q0")
    if any(abs(value) > 1.0e-12 for value in key_poses["q0_final"]):
        raise ValueError("manifest does not finish at q0")
    key_poses["q0"] = (0.0,) * 5
    key_poses.pop("q0_final")
    return key_poses


def simulate_stage_tracking(
    actual: tuple[float, ...],
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    duration_ms: int,
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> dict[str, object]:
    """한 stage 의 추종을 모의하되 팔의 실제 위치를 이어받는다.

    `simulate_rate_limited_tracking` 은 팔이 시작점에 정확히 있다고 가정한다.
    단일 leg 에서는 맞지만 경유점이 여러 개인 경로에서는 틀린다. 팔은 앞
    stage 끝에서 이미 뒤처져 있고, 그 상태로 다음 stage 를 시작한다. 오차를
    이어받지 않으면 매 경유점마다 팔이 목표에 정확히 도달했다고 가정하게 되어
    누적 지연을 놓친다.

    명령 궤적은 계획이 정한 대로 start_raw -> target_raw 이고, 팔만 뒤처진다.
    """
    if len(start_raw) != 6 or len(target_raw) != 6 or len(actual) != 6:
        raise ValueError("tracking simulation requires six-axis state")
    if not (
        0.0 < tracking_rate_raw_s <= MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S
    ):
        raise ValueError(
            "tracking rate must be in (0, "
            f"{MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S:g}] raw/s"
        )
    maximum_step = (
        tracking_rate_raw_s * TRACKING_SIMULATION_PERIOD_MS / 1000.0
    )
    position = [float(value) for value in actual]
    peak_errors = [0.0] * 6
    entry_errors = [
        abs(float(start) - value)
        for start, value in zip(start_raw, position, strict=True)
    ]
    step_count = duration_ms // TRACKING_SIMULATION_PERIOD_MS
    for step in range(1, step_count + 1):
        progress = minimum_jerk_unit_progress(step / step_count)
        for index, (start, target) in enumerate(
            zip(start_raw, target_raw, strict=True)
        ):
            command = start + progress * (target - start)
            error = command - position[index]
            position[index] += max(-maximum_step, min(maximum_step, error))
            peak_errors[index] = max(
                peak_errors[index], abs(command - position[index])
            )
    terminal_errors = [
        abs(target - value)
        for target, value in zip(target_raw, position, strict=True)
    ]
    return {
        "duration_ms": duration_ms,
        "entry_error_raw": entry_errors,
        "maximum_entry_error_raw": max(entry_errors),
        "peak_error_raw": peak_errors,
        "terminal_error_raw": terminal_errors,
        "maximum_peak_error_raw": max(peak_errors),
        "maximum_terminal_error_raw": max(terminal_errors),
        "actual_after_raw": tuple(position),
    }


def select_duration_ms(
    actual: tuple[float, ...],
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> int:
    """추종 게이트를 여유 있게 통과하는 최소 시간을 찾는다.

    팔이 이미 뒤처진 상태에서 시작하므로, 그 상태를 넣고 탐색한다.

    `tracking_rate_raw_s` 가 **이 저장소의 속도 손잡이다.** 올리면 같은
    경로가 짧은 시간에 배치된다. 기본값은 바꾸지 않는다 — 2026-08-07
    speed-ramp 로 특정 leg(a45_top_shadow pregrasp)에 대해 `300`까지
    명시적으로 지정해 검증했지만(post-settle 16raw, 하드 게이트까지
    14raw 여유), leg 마다 관절 이동량이 달라 기본값으로 그대로 올리면
    다른 leg(q0_return 등)에서 관절 속도 하드 게이트(0.5 rad/s)를 넘겨
    계획 자체가 거부될 수 있다(실측으로 확인됨). 빠르게 갈 legs 는
    `--tracking-rate-raw-s 300` 을 그때그때 명시할 것.
    """
    peak_budget = MAXIMUM_MODELED_PEAK_ERROR_RAW * TRACKING_MARGIN_FRACTION
    for duration_ms in range(
        MINIMUM_DURATION_MS,
        MAXIMUM_DURATION_MS + DURATION_SEARCH_STEP_MS,
        DURATION_SEARCH_STEP_MS,
    ):
        tracking = simulate_stage_tracking(
            actual, start_raw, target_raw, duration_ms, tracking_rate_raw_s
        )
        if (
            tracking["maximum_peak_error_raw"] <= peak_budget
            and tracking["maximum_terminal_error_raw"]
            <= MAXIMUM_MODELED_TERMINAL_ERROR_RAW
        ):
            return duration_ms
    raise ValueError(
        "no duration within the search range satisfies the tracking contract"
    )


def piecewise_positions(
    elapsed_ms: int,
    stages: tuple[tuple[tuple[float, ...], tuple[float, ...], int], ...],
) -> tuple[float, ...]:
    """구간별 minimum-jerk. 각 경유점에서 속도가 0 이 된다.

    경유점은 collision-checked 경로가 검증한 자세이므로, 매끄럽게 통과하는
    대신 그 위에서 정지하는 쪽을 택한다.
    """
    remaining = elapsed_ms
    for start, end, duration_ms in stages:
        if remaining <= duration_ms:
            progress = minimum_jerk_unit_progress(remaining / duration_ms)
            return tuple(
                a + progress * (b - a)
                for a, b in zip(start, end, strict=True)
            )
        remaining -= duration_ms
    return stages[-1][1]


def build_plan(
    calibration_path: Path,
    contract_path: Path,
    manifest_path: Path,
    leg: str,
    anchor_raw: tuple[int, ...],
) -> dict[str, object]:
    if leg not in LEG_DEFINITIONS:
        raise ValueError(f"unknown leg: {leg}")
    definition = LEG_DEFINITIONS[leg]

    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized=false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("buffered physical execution must be commissioned")
    uart_candidate = contract["servo_uart_receive_candidate"]
    if uart_candidate["motion_authorized"] is not False:
        raise ValueError("servo UART candidate must keep motion_authorized=false")
    # 배포 여부는 계획 시점에 강제하지 않는다. 계획은 게이트를 기록하고
    # 실행기가 거부한다(`load_pick_place_leg_plan` 의 require_deployed).
    # 그래야 펌웨어 후보를 검증하는 동안에도 계획을 만들 수 있다.
    # 기존 계획기들(plan_buffered_q0_return 등)과 같은 분담이다.
    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")

    key_poses = load_key_poses(manifest_path)
    arm_names = tuple(calibration.ros_joint_names[:5])
    manifest_names = tuple(
        json.loads(manifest_path.read_text(encoding="utf-8"))["joint_names"]
    )
    if manifest_names != arm_names:
        raise ValueError("manifest joint order does not match calibration")

    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_anchor = anchor_rad[:5]
    preserved_gripper_rad = anchor_rad[5]

    expected_start = key_poses[definition["start_pose"]]
    expected_start_raw = radians_to_raw(
        calibration, expected_start + (preserved_gripper_rad,)
    )
    anchor_deviation_raw = [
        abs(actual - expected)
        for actual, expected in zip(
            anchor_raw[:5], expected_start_raw[:5], strict=True
        )
    ]
    if max(anchor_deviation_raw) > ANCHOR_DEVIATION_LIMIT_RAW:
        raise ValueError(
            f"leg {leg} anchor is off the collision-checked route: "
            f"deviation_raw={anchor_deviation_raw} "
            f"limit={ANCHOR_DEVIATION_LIMIT_RAW} "
            f"expected_start={definition['start_pose']}"
        )

    stages: list[tuple[tuple[float, ...], tuple[float, ...], int]] = []
    tracking_legs: dict[str, object] = {}
    current_rad = arm_anchor
    current_raw = tuple(anchor_raw)
    # 팔의 모의 실제 위치. anchor 에서는 명령과 일치하지만 이후 stage 에서는
    # 뒤처진 채로 이어진다.
    actual_raw = tuple(float(value) for value in anchor_raw)
    for name in definition["waypoints"]:
        target_rad = key_poses[name]
        target_raw = radians_to_raw(
            calibration, target_rad + (preserved_gripper_rad,)
        )
        duration_ms = select_duration_ms(actual_raw, current_raw, target_raw)
        if duration_ms % SAMPLE_PERIOD_MS != 0:
            raise ValueError("duration must be a whole number of 20 ms samples")
        tracking = simulate_stage_tracking(
            actual_raw, current_raw, target_raw, duration_ms
        )
        if tracking["maximum_peak_error_raw"] > MAXIMUM_MODELED_PEAK_ERROR_RAW:
            raise ValueError(f"modeled peak tracking error exceeds contract: {name}")
        if (
            tracking["maximum_terminal_error_raw"]
            > MAXIMUM_MODELED_TERMINAL_ERROR_RAW
        ):
            raise ValueError(
                f"modeled terminal tracking error exceeds contract: {name}"
            )
        actual_raw = tracking.pop("actual_after_raw")
        tracking_legs[name] = tracking
        stages.append((current_rad, target_rad, duration_ms))
        current_rad = target_rad
        current_raw = target_raw

    total_duration_ms = sum(duration for _, _, duration in stages)
    elapsed_values = range(
        0, total_duration_ms + SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS
    )
    points = tuple(
        TrajectoryPointData(
            positions=piecewise_positions(elapsed_ms, tuple(stages)),
            time_from_start_ns=elapsed_ms * 1_000_000,
        )
        for elapsed_ms in elapsed_values
    )

    position_limits = {
        name: calibration.ros_radian_limits[name] for name in arm_names
    }
    start_tolerances = {
        name: (0.055 if name.endswith("shoulder_joint") else 0.050)
        for name in arm_names
    }
    trajectory = validate_buffered_trajectory(
        arm_names,
        points,
        arm_names,
        position_limits,
        arm_anchor,
        {name: 0.5 for name in arm_names},
        {name: 1.0 for name in arm_names},
        start_tolerance_rad=start_tolerances,
    )
    plan = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=preserved_gripper_rad,
        current_tick_ms=PLAN_TICK_MS,
    )
    batch_sizes, admission = simulate_admission_batches(plan)

    samples_rad = tuple(
        tuple(value / 1_000_000.0 for value in sample.positions_urad)
        for sample in plan.samples
    )
    period_s = SAMPLE_PERIOD_MS / 1000.0
    velocities = tuple(
        tuple(
            (current - previous) / period_s
            for previous, current in zip(a[:5], b[:5], strict=True)
        )
        for a, b in zip(samples_rad[:-1], samples_rad[1:], strict=True)
    )
    zero = (0.0,) * 5
    accelerations = tuple(
        tuple(
            (current - previous) / period_s
            for previous, current in zip(a, b, strict=True)
        )
        for a, b in zip((zero, *velocities[:-1]), velocities, strict=True)
    )
    maximum_sample_step = max(
        abs(current - previous)
        for a, b in zip(samples_rad[:-1], samples_rad[1:], strict=True)
        for previous, current in zip(a[:5], b[:5], strict=True)
    )
    firmware_raw_trace = simulate_firmware_output_raw(calibration, plan.samples)
    raw_steps = tuple(
        tuple(current - previous for previous, current in zip(a, b, strict=True))
        for a, b in zip(
            firmware_raw_trace[:-1], firmware_raw_trace[1:], strict=True
        )
    )
    if firmware_raw_trace[0] != tuple(anchor_raw):
        raise ValueError("firmware output does not start at the anchor")
    if firmware_raw_trace[-1] != tuple(current_raw):
        raise ValueError("firmware output does not finish at the leg target")
    # gripper 는 이 leg 안에서 움직이지 않는다. 접촉은 감시가 있는 별도
    # 명령에서만 일어난다.
    if any(row[5] != anchor_raw[5] for row in firmware_raw_trace):
        raise ValueError("gripper must stay preserved across a buffered leg")

    return {
        "schema_version": 1,
        "status": STATUS,
        "phase": PHASE,
        "leg": leg,
        "leg_description": definition["description"],
        "firmware_version": uart_candidate["firmware_version"],
        "capabilities": "0x00000FFF",
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": sha256_file(calibration_path),
        "contract_sha256": sha256_file(contract_path),
        "contract_status": contract["status"],
        "manifest_sha256": MANIFEST_SHA256,
        "firmware_deployment_gate": {
            "candidate_status": uart_candidate["status"],
            "deployed": uart_candidate["deployed"],
            "motion_authorized": uart_candidate["motion_authorized"],
        },
        "joint_names": list(calibration.ros_joint_names),
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
            "expected_start_pose": definition["start_pose"],
            "expected_start_raw": list(expected_start_raw),
            "deviation_raw": anchor_deviation_raw,
            "deviation_limit_raw": ANCHOR_DEVIATION_LIMIT_RAW,
        },
        "target": {
            "name": definition["waypoints"][-1],
            "raw": list(current_raw),
            "positions_rad": list(current_rad) + [preserved_gripper_rad],
            "gripper_preserved": True,
        },
        "gripper_action_after_leg": definition["gripper_action_after"],
        "analytic_profile": {
            "kind": "piecewise_quintic_minimum_jerk_through_key_poses",
            "polynomial": "10t^3-15t^4+6t^5",
            "zero_velocity_at_waypoints": True,
            "stages": [
                {
                    "target_pose": name,
                    "duration_ms": duration,
                    "duration_selected_automatically": True,
                }
                for name, (_, _, duration) in zip(
                    definition["waypoints"], stages, strict=True
                )
            ],
            "duration_ms": total_duration_ms,
            "waypoint_period_ms": SAMPLE_PERIOD_MS,
            "waypoint_count": len(points),
        },
        "resampling": {
            "period_ms": SAMPLE_PERIOD_MS,
            "sample_count": len(plan.samples),
            "duration_ms": total_duration_ms,
            "maximum_sample_step_rad": maximum_sample_step,
            "samples": [
                {
                    "index": sample.sample_index,
                    "elapsed_ms": sample.trajectory_elapsed_ms,
                    "apply_offset_ms": sample.apply_tick_ms - PLAN_TICK_MS,
                    "positions_urad": list(sample.positions_urad),
                }
                for sample in plan.samples
            ],
        },
        "dynamic_limits": {
            "velocity_rad_s": {name: 0.5 for name in arm_names},
            "acceleration_rad_s2": {name: 1.0 for name in arm_names},
            "finite_difference": {
                "maximum_velocity_rad_s": max(
                    abs(value) for row in velocities for value in row
                ),
                "maximum_acceleration_rad_s2": max(
                    abs(value) for row in accelerations for value in row
                ),
            },
        },
        "physical_tracking_model": {
            "kind": "per_axis_rate_limited_minimum_jerk_follower",
            "conservative_rate_raw_s": CONSERVATIVE_TRACKING_RATE_RAW_S,
            "maximum_allowed_peak_error_raw": MAXIMUM_MODELED_PEAK_ERROR_RAW,
            "maximum_allowed_terminal_error_raw": (
                MAXIMUM_MODELED_TERMINAL_ERROR_RAW
            ),
            "legs": tracking_legs,
        },
        "queue_contract": {
            "admission_batch_sizes": list(batch_sizes),
            "maximum_batch_samples": max(batch_sizes),
            "simulation_terminal": admission,
        },
        "firmware_output_simulation": {
            "executor_step_period_ms": 1,
            "servo_sync_write_period_ms": 5,
            "output_count": len(firmware_raw_trace),
            "maximum_arm_step_raw": max(
                abs(value) for row in raw_steps for value in row[:5]
            ),
            "start_raw": list(firmware_raw_trace[0]),
            "final_raw": list(firmware_raw_trace[-1]),
        },
        "execution_api_used": False,
        "buffered_frame_encoded": False,
        "robot_target_available": False,
        "motion_authorized": False,
    }


def parse_args() -> argparse.Namespace:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--leg", choices=sorted(LEG_DEFINITIONS), required=True)
    parser.add_argument(
        "--anchor-raw", type=int, nargs=6, required=True, metavar="RAW"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=repository_root
        / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--contract",
        type=Path,
        default=repository_root
        / "ros2_ws/src/single_arm_bridge/config/buffered_trajectory_contract.json",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=repository_root
        / "artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015"
        / "full_pick_place_plan_only_manifest.json",
    )
    arguments = parser.parse_args()
    if not arguments.plan_only:
        parser.error("--plan-only is required; this tool never executes")
    return arguments


def main() -> int:
    arguments = parse_args()
    document = build_plan(
        arguments.calibration,
        arguments.contract,
        arguments.manifest,
        arguments.leg,
        tuple(arguments.anchor_raw),
    )
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    profile = document["analytic_profile"]
    print(f"MOTION13_PICK_PLACE_LEG_PLAN={arguments.output}")
    print(f"LEG={document['leg']}  {document['leg_description']}")
    print(f"STATUS={document['status']}")
    print(f"START_POSE={document['anchor']['expected_start_pose']}")
    print(f"ANCHOR_DEVIATION_RAW={document['anchor']['deviation_raw']}")
    for stage in profile["stages"]:
        print(f"  stage -> {stage['target_pose']:16s} {stage['duration_ms']:6d} ms")
    print(f"DURATION_MS={profile['duration_ms']}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(
        "MAXIMUM_SAMPLE_STEP_RAD="
        f"{document['resampling']['maximum_sample_step_rad']:.9f}"
    )
    tracking = document["physical_tracking_model"]["legs"]
    print(
        "MODELED_PEAK_ERROR_RAW="
        f"{max(leg['maximum_peak_error_raw'] for leg in tracking.values()):.3f}"
    )
    print(
        "MODELED_TERMINAL_ERROR_RAW="
        f"{max(leg['maximum_terminal_error_raw'] for leg in tracking.values()):.3f}"
    )
    print(f"GRIPPER_ACTION_AFTER={document['gripper_action_after_leg']}")
    print(f"EXECUTION_API_USED={int(document['execution_api_used'])}")
    print(f"MOTION_AUTHORIZED={int(document['motion_authorized'])}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
