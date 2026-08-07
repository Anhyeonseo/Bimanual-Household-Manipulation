#!/usr/bin/env python3
"""
임의 자세에서 q0 로 복귀하는 단일 leg buffered 계획을 만든다.

Motion-11 은 팔을 Pick pregrasp 에 torque 유지 상태로 남긴다. 거기서 q0 로
내려오려면 shoulder 가 raw 3194 에서 2048 까지 약 1146 raw 를 움직여야 하는데,
`plan_buffered_q0_roundtrip.py` 는 소형 왕복용 고정 시간이라 이 변위에서
가속도 상한에 걸린다.

이 도구는 그 복귀 경로를 계획한다. 시간을 고정하지 않고, 보수적 추종 계약
(`50 raw/s`)에서 모델 peak 오차가 허용치의 70% 이하가 되는 시간을 탐색한다.
통과하는 최소 시간을 고르면 정의상 오차가 상한에 붙으므로 그렇게 하지 않는다.
Motion-11 의 실패 원인이 계획이 서보 추종 능력보다 빨랐던 것이므로, 복귀
경로도 같은 모델로 검증한다.

실행 API, Action goal, buffered frame, 로봇 이동을 사용하지 않는다.
`motion_authorized=false` 를 유지한다.
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
    Q0_RAW,
    simulate_rate_limited_tracking,
)


STATUS = "BUFFERED_Q0_RETURN_PLAN_ONLY_PASS"
PHASE = "motion12_q0_return"
PLAN_TICK_MS = 100_000
MINIMUM_DURATION_MS = 4_000
MAXIMUM_DURATION_MS = 180_000
DURATION_SEARCH_STEP_MS = 1_000
# 최소 시간을 고르면 추종 오차가 상한에 붙는다. 복귀 동작에 시간을 아낄
# 이유가 없으므로 모델 peak 오차가 허용치의 이 비율 이하가 되도록 고른다.
TRACKING_MARGIN_FRACTION = 0.70


def select_duration_ms(
    anchor_raw: tuple[int, ...],
    q0_raw: tuple[int, ...],
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> int:
    """
    추종 게이트를 통과하는 최소 시간을 찾는다.

    시간을 손으로 고르면 Motion-11 1차 시도처럼 서보가 못 따라오는 계획을
    만들게 된다. 보수적 rate 모델을 직접 돌려 결정하되, 통과하는 최소 시간이
    아니라 허용 peak 오차의 TRACKING_MARGIN_FRACTION 이하가 되는 시간을 고른다.
    최소 시간은 정의상 오차가 상한에 붙는다.
    """
    peak_budget = MAXIMUM_MODELED_PEAK_ERROR_RAW * TRACKING_MARGIN_FRACTION
    for duration_ms in range(
        MINIMUM_DURATION_MS,
        MAXIMUM_DURATION_MS + DURATION_SEARCH_STEP_MS,
        DURATION_SEARCH_STEP_MS,
    ):
        tracking = simulate_rate_limited_tracking(
            anchor_raw, q0_raw, duration_ms, tracking_rate_raw_s
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


def build_plan(
    calibration_path: Path,
    contract_path: Path,
    anchor_raw: tuple[int, ...],
    duration_ms: int | None = None,
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> dict[str, object]:
    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized=false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("buffered physical execution must be commissioned")
    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")

    arm_names = tuple(calibration.ros_joint_names[:5])
    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_anchor = anchor_rad[:5]
    q0_with_gripper = (0.0,) * 5 + (anchor_rad[5],)
    q0_raw = radians_to_raw(calibration, q0_with_gripper)
    if q0_raw[:5] != Q0_RAW:
        raise ValueError("calibration does not map arm q0 to raw 2048")

    selected_duration_ms = (
        select_duration_ms(anchor_raw, q0_raw, tracking_rate_raw_s)
        if duration_ms is None
        else duration_ms
    )
    if selected_duration_ms % SAMPLE_PERIOD_MS != 0:
        raise ValueError("duration must be a whole number of 20 ms samples")

    elapsed_values = range(
        0, selected_duration_ms + SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS
    )
    points = tuple(
        TrajectoryPointData(
            positions=tuple(
                value
                * (
                    1.0
                    - minimum_jerk_unit_progress(
                        elapsed_ms / selected_duration_ms
                    )
                )
                for value in arm_anchor
            ),
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
        preserved_gripper_rad=anchor_rad[5],
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
    tracking = simulate_rate_limited_tracking(
        anchor_raw, q0_raw, selected_duration_ms, tracking_rate_raw_s
    )
    if tracking["maximum_peak_error_raw"] > MAXIMUM_MODELED_PEAK_ERROR_RAW:
        raise ValueError("modeled peak tracking error exceeds the contract")
    if (
        tracking["maximum_terminal_error_raw"]
        > MAXIMUM_MODELED_TERMINAL_ERROR_RAW
    ):
        raise ValueError("modeled terminal tracking error exceeds the contract")
    if firmware_raw_trace[0] != tuple(anchor_raw):
        raise ValueError("firmware output does not start at the anchor")
    if firmware_raw_trace[-1] != tuple(q0_raw):
        raise ValueError("firmware output does not finish at q0")

    return {
        "schema_version": 1,
        "status": STATUS,
        "phase": PHASE,
        "firmware_version": contract["servo_uart_receive_candidate"][
            "firmware_version"
        ],
        "capabilities": "0x00000FFF",
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": sha256_file(calibration_path),
        "contract_sha256": sha256_file(contract_path),
        "contract_status": contract["status"],
        "firmware_deployment_gate": {
            "candidate_status": contract["servo_uart_receive_candidate"][
                "status"
            ],
            "deployed": contract["servo_uart_receive_candidate"]["deployed"],
            "motion_authorized": contract["servo_uart_receive_candidate"][
                "motion_authorized"
            ],
        },
        "joint_names": list(calibration.ros_joint_names),
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
        },
        "target": {
            "name": "q0",
            "raw": list(q0_raw),
            "positions_rad": list(q0_with_gripper),
            "gripper_preserved": True,
        },
        "analytic_profile": {
            "kind": "single_leg_quintic_minimum_jerk",
            "polynomial": "10t^3-15t^4+6t^5",
            "duration_ms": selected_duration_ms,
            "duration_selected_automatically": duration_ms is None,
            "waypoint_period_ms": SAMPLE_PERIOD_MS,
            "waypoint_count": len(points),
        },
        "resampling": {
            "period_ms": SAMPLE_PERIOD_MS,
            "sample_count": len(plan.samples),
            "duration_ms": selected_duration_ms,
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
            "conservative_rate_raw_s": tracking_rate_raw_s,
            "maximum_allowed_peak_error_raw": MAXIMUM_MODELED_PEAK_ERROR_RAW,
            "maximum_allowed_terminal_error_raw": (
                MAXIMUM_MODELED_TERMINAL_ERROR_RAW
            ),
            "legs": {"anchor_to_q0": tracking},
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--anchor-raw", type=int, nargs=6, required=True, metavar="RAW"
    )
    parser.add_argument(
        "--duration-ms",
        type=int,
        default=None,
        help="생략하면 추종 계약을 통과하는 최소 시간을 탐색한다",
    )
    parser.add_argument(
        "--tracking-rate-raw-s",
        type=float,
        default=CONSERVATIVE_TRACKING_RATE_RAW_S,
        help="이 leg 의 추종률 가정(raw/s). 기본값은 보수적 50",
    )
    repository_root = Path(__file__).resolve().parents[1]
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = build_plan(
        args.calibration,
        args.contract,
        tuple(args.anchor_raw),
        args.duration_ms,
        args.tracking_rate_raw_s,
    )
    encoded = (
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    ).encode()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(encoded)

    profile = document["analytic_profile"]
    tracking = document["physical_tracking_model"]["legs"]["anchor_to_q0"]
    print(f"MOTION12_Q0_RETURN_PLAN={args.output}")
    print(f"STATUS={document['status']}")
    print(f"DURATION_MS={profile['duration_ms']}")
    print(f"DURATION_AUTOSELECTED={int(profile['duration_selected_automatically'])}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(
        "MAXIMUM_SAMPLE_STEP_RAD="
        f"{document['resampling']['maximum_sample_step_rad']:.9f}"
    )
    print(f"MODELED_PEAK_ERROR_RAW={tracking['maximum_peak_error_raw']:.3f}")
    print(
        f"MODELED_TERMINAL_ERROR_RAW={tracking['maximum_terminal_error_raw']:.3f}"
    )
    print(f"EXECUTION_API_USED={int(document['execution_api_used'])}")
    print(f"MOTION_AUTHORIZED={int(document['motion_authorized'])}")
    print(f"SHA256={sha256(encoded).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
