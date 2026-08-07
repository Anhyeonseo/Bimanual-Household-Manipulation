#!/usr/bin/env python3
"""갓 계획된 collision-checked segment 를 buffered leg 로 변환한다. plan-only.

Motion-13 은 SHA 로 고정된 manifest 의 phase 만 실행할 수 있다. 그것으로는
과거에 검토된 경로를 재생하는 것밖에 못 한다. Pick 자세를 새로 계산하거나
(A4 offset 재계측), Top 카메라가 찾은 위치로 가려면(A4.5) **매 회 새로 계획한
경로**를 실행해야 한다.

이 도구는 `ros_moveit_plan_pregrasp_segments.py` 가 방금 만든 segment 파일을
받아 같은 buffered leg 로 만든다. 검증 규율은 manifest 경로와 동일하다.

**왜 segment 파일이어야 하는가.**

`ros_moveit_plan_grasp.py` 는 endpoint 만 저장한다(`trajectory_point_count` 는
남지만 점은 남지 않는다). 그것만으로는 MoveIt 이 검사한 경로를 재생할 수
없다. segment 파일은 경계된 관절 스텝의 체인을 담고 있고, 각 스텝이 개별로
계획·검사됐다. Motion-13 이 안전했던 근거가 그것이다.

이 도구는 endpoint 계획을 직접 받지 않는다. 받으면 검사되지 않은 경로를
직선으로 이어 실행하게 된다.
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
)
from plan_buffered_pick_place_leg import (
    ANCHOR_DEVIATION_LIMIT_RAW,
    MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S,
    PLAN_TICK_MS,
    select_duration_ms,
    simulate_stage_tracking,
)


STATUS = "BUFFERED_SEGMENT_LEG_PLAN_ONLY_PASS"
PHASE = "motion14_fresh_segment_leg"
STRAIGHT_LINE_TOLERANCE_RAD = 1.0e-9


def load_segment_route(
    path: Path,
    expected_sha256: str,
    arm_names: tuple[str, ...],
) -> dict[str, object]:
    """방금 만든 segment 파일을 manifest phase 와 같은 규율로 검증한다.

    SHA 는 소스에 박지 않고 운영자가 실행 시점에 넘긴다. 이 파일은 매 회
    새로 만들어지므로 고정 상수로 둘 수 없다. 대신 계획과 실행이 **같은
    digest** 를 요구하게 해 그 사이에 파일이 바뀌지 못하게 한다.
    """
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"segment plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))

    status = document.get("status")
    if not isinstance(status, str) or not status.endswith(
        "_SEGMENT_PLAN_ONLY_PASS"
    ):
        raise ValueError(f"segment plan status is not a pass: {status}")
    for flag in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
    ):
        if document.get(flag) is not False:
            raise ValueError(f"segment plan must keep {flag}=false")
    if tuple(document.get("joint_names", ())) != arm_names:
        raise ValueError("segment plan joint order does not match calibration")

    segments = document.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("segment plan contains no segments")
    for index, segment in enumerate(segments, start=1):
        if segment.get("success") is not True:
            raise ValueError(f"segment {index} did not plan successfully")
        if segment.get("moveit_error_code") != 1:
            raise ValueError(f"segment {index} has a non-success error code")

    step_limit = document.get("max_joint_step_rad")
    if not isinstance(step_limit, (int, float)) or not 0.0 < step_limit <= 0.30:
        raise ValueError("segment plan step limit is outside (0, 0.30]")
    for index, segment in enumerate(segments, start=1):
        delta = segment.get("maximum_joint_delta_rad")
        if not isinstance(delta, (int, float)) or delta > step_limit:
            raise ValueError(
                f"segment {index} delta {delta} exceeds its own step limit"
            )

    # 체인 연속성. 앞 segment 의 목표가 뒤 segment 의 시작이어야 한다.
    for index, (previous, following) in enumerate(
        zip(segments[:-1], segments[1:]), start=1
    ):
        end = tuple(float(v) for v in previous["target_positions_rad"])
        start = tuple(float(v) for v in following["expected_start_positions_rad"])
        if any(abs(a - b) > 1.0e-12 for a, b in zip(end, start, strict=True)):
            raise ValueError(f"segment chain breaks after segment {index}")

    start = tuple(
        float(v) for v in segments[0]["expected_start_positions_rad"]
    )
    end = tuple(float(v) for v in segments[-1]["target_positions_rad"])
    if len(start) != 5 or len(end) != 5:
        raise ValueError("segment endpoints must contain five arm joints")

    # 이 계획기는 경유점을 버리고 minimum-jerk 하나로 잇는다. 그것이
    # 허용되려면 segment 체인이 관절공간 직선이어야 한다. 아니면 MoveIt 이
    # 검사한 경로를 벗어난다.
    for index, segment in enumerate(segments, start=1):
        fraction = index / len(segments)
        values = tuple(float(v) for v in segment["target_positions_rad"])
        for a, b, value in zip(start, end, values, strict=True):
            if abs(value - (a + fraction * (b - a))) > STRAIGHT_LINE_TOLERANCE_RAD:
                raise ValueError(
                    "segment chain is not a straight joint-space path; "
                    "a buffered leg cannot collapse it"
                )

    return {
        "document": document,
        "digest": digest,
        "start": start,
        "target": end,
        "segment_count": len(segments),
        "step_limit_rad": float(step_limit),
    }


def build_plan(
    calibration_path: Path,
    contract_path: Path,
    segments_path: Path,
    segments_sha256: str,
    anchor_raw: tuple[int, ...],
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> dict[str, object]:
    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized=false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("buffered physical execution must be commissioned")
    uart_candidate = contract["servo_uart_receive_candidate"]
    if uart_candidate["motion_authorized"] is not False:
        raise ValueError("servo UART candidate must keep motion_authorized=false")
    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")

    arm_names = tuple(calibration.ros_joint_names[:5])
    route = load_segment_route(segments_path, segments_sha256, arm_names)
    document = route["document"]

    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_anchor = anchor_rad[:5]
    preserved_gripper_rad = anchor_rad[5]

    expected_start_raw = radians_to_raw(
        calibration, tuple(route["start"]) + (preserved_gripper_rad,)
    )
    deviation = [
        abs(actual - expected)
        for actual, expected in zip(
            anchor_raw[:5], expected_start_raw[:5], strict=True
        )
    ]
    if max(deviation) > ANCHOR_DEVIATION_LIMIT_RAW:
        raise ValueError(
            "anchor is off the freshly planned route: "
            f"deviation_raw={deviation} limit={ANCHOR_DEVIATION_LIMIT_RAW}"
        )

    target_rad = tuple(route["target"])
    target_raw = radians_to_raw(
        calibration, target_rad + (preserved_gripper_rad,)
    )
    actual_raw = tuple(float(value) for value in anchor_raw)
    duration_ms = select_duration_ms(
        actual_raw, tuple(anchor_raw), target_raw, tracking_rate_raw_s
    )
    if duration_ms % SAMPLE_PERIOD_MS != 0:
        raise ValueError("duration must be a whole number of 20 ms samples")
    tracking = simulate_stage_tracking(
        actual_raw,
        tuple(anchor_raw),
        target_raw,
        duration_ms,
        tracking_rate_raw_s,
    )
    if tracking["maximum_peak_error_raw"] > MAXIMUM_MODELED_PEAK_ERROR_RAW:
        raise ValueError("modeled peak tracking error exceeds the contract")
    if (
        tracking["maximum_terminal_error_raw"]
        > MAXIMUM_MODELED_TERMINAL_ERROR_RAW
    ):
        raise ValueError("modeled terminal tracking error exceeds the contract")
    tracking.pop("actual_after_raw", None)

    elapsed_values = range(0, duration_ms + SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS)
    points = tuple(
        TrajectoryPointData(
            positions=tuple(
                a + minimum_jerk_unit_progress(elapsed_ms / duration_ms) * (b - a)
                for a, b in zip(arm_anchor, target_rad, strict=True)
            ),
            time_from_start_ns=elapsed_ms * 1_000_000,
        )
        for elapsed_ms in elapsed_values
    )

    trajectory = validate_buffered_trajectory(
        arm_names,
        points,
        arm_names,
        {name: calibration.ros_radian_limits[name] for name in arm_names},
        arm_anchor,
        {name: 0.5 for name in arm_names},
        {name: 1.0 for name in arm_names},
        start_tolerance_rad={
            name: (0.055 if name.endswith("shoulder_joint") else 0.050)
            for name in arm_names
        },
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
    maximum_sample_step = max(
        abs(current - previous)
        for a, b in zip(samples_rad[:-1], samples_rad[1:], strict=True)
        for previous, current in zip(a[:5], b[:5], strict=True)
    )
    firmware_raw_trace = simulate_firmware_output_raw(calibration, plan.samples)
    if firmware_raw_trace[0] != tuple(anchor_raw):
        raise ValueError("firmware output does not start at the anchor")
    if firmware_raw_trace[-1] != tuple(target_raw):
        raise ValueError("firmware output does not finish at the segment target")
    if any(row[5] != anchor_raw[5] for row in firmware_raw_trace):
        raise ValueError("gripper must stay preserved across a buffered leg")

    return {
        "schema_version": 1,
        "status": STATUS,
        "phase": PHASE,
        "firmware_version": uart_candidate["firmware_version"],
        "capabilities": "0x00000FFF",
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": sha256_file(calibration_path),
        "contract_sha256": sha256_file(contract_path),
        "contract_status": contract["status"],
        "firmware_deployment_gate": {
            "candidate_status": uart_candidate["status"],
            "deployed": uart_candidate["deployed"],
            "motion_authorized": uart_candidate["motion_authorized"],
        },
        "segment_route": {
            "path": str(segments_path),
            "sha256": route["digest"],
            "status": document["status"],
            "target_name": document.get("target_name"),
            "source_plan": document.get("source_plan"),
            "segment_count": route["segment_count"],
            "max_joint_step_rad": route["step_limit_rad"],
            "collision_checked_in_this_session": True,
        },
        "joint_names": list(calibration.ros_joint_names),
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
            "expected_start_rad": list(route["start"]),
            "expected_start_raw": list(expected_start_raw),
            "deviation_raw": deviation,
            "deviation_limit_raw": ANCHOR_DEVIATION_LIMIT_RAW,
        },
        "target": {
            "positions_rad": list(target_rad) + [preserved_gripper_rad],
            "raw": list(target_raw),
            "gripper_preserved": True,
        },
        "analytic_profile": {
            "kind": "single_leg_quintic_minimum_jerk_over_checked_segments",
            "polynomial": "10t^3-15t^4+6t^5",
            "duration_ms": duration_ms,
            "duration_selected_automatically": True,
            "waypoint_period_ms": SAMPLE_PERIOD_MS,
            "waypoint_count": len(points),
        },
        "resampling": {
            "period_ms": SAMPLE_PERIOD_MS,
            "sample_count": len(plan.samples),
            "duration_ms": duration_ms,
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
        "physical_tracking_model": {
            "kind": "per_axis_rate_limited_minimum_jerk_follower",
            # 실행기가 이 값을 읽어 계획을 그대로 재계산한다. 계획과 다른
            # rate 로 재계산하면 재현이 깨져 실행이 거부된다.
            "conservative_rate_raw_s": float(tracking_rate_raw_s),
            "default_rate_raw_s": CONSERVATIVE_TRACKING_RATE_RAW_S,
            "maximum_authorized_rate_raw_s": (
                MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S
            ),
            "maximum_allowed_peak_error_raw": MAXIMUM_MODELED_PEAK_ERROR_RAW,
            "maximum_allowed_terminal_error_raw": (
                MAXIMUM_MODELED_TERMINAL_ERROR_RAW
            ),
            "legs": {"anchor_to_segment_target": tracking},
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
            "start_raw": list(firmware_raw_trace[0]),
            "final_raw": list(firmware_raw_trace[-1]),
        },
        "execution_api_used": False,
        "buffered_frame_encoded": False,
        "robot_target_available": False,
        "motion_authorized": False,
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--segments", type=Path, required=True)
    parser.add_argument("--segments-sha256", required=True)
    parser.add_argument(
        "--anchor-raw", type=int, nargs=6, required=True, metavar="RAW"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=root
        / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--contract",
        type=Path,
        default=root
        / "ros2_ws/src/single_arm_bridge/config/buffered_trajectory_contract.json",
    )
    # 속도 손잡이. 기본값은 2026-08-04 의 보수적 가정이고, 올리려면 그
    # 근거를 같은 세션의 실측으로 남겨야 한다.
    parser.add_argument(
        "--tracking-rate-raw-s",
        type=float,
        default=CONSERVATIVE_TRACKING_RATE_RAW_S,
    )
    arguments = parser.parse_args()
    if not arguments.plan_only:
        parser.error("--plan-only is required; this tool never executes")
    if not (
        0.0
        < arguments.tracking_rate_raw_s
        <= MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S
    ):
        parser.error(
            "tracking rate must be in (0, "
            f"{MAXIMUM_AUTHORIZED_TRACKING_RATE_RAW_S:g}] raw/s"
        )
    return arguments


def main() -> int:
    arguments = parse_args()
    document = build_plan(
        arguments.calibration,
        arguments.contract,
        arguments.segments,
        arguments.segments_sha256,
        tuple(arguments.anchor_raw),
        arguments.tracking_rate_raw_s,
    )
    text = json.dumps(document, indent=2, sort_keys=True) + "\n"
    arguments.output.write_text(text, encoding="utf-8")

    route = document["segment_route"]
    print(f"MOTION14_SEGMENT_LEG_PLAN={arguments.output}")
    print(f"STATUS={document['status']}")
    print(f"SEGMENT_ROUTE={route['path']}")
    print(f"SEGMENT_STATUS={route['status']}  target={route['target_name']}")
    print(f"SEGMENT_COUNT={route['segment_count']}")
    print(f"ANCHOR_DEVIATION_RAW={document['anchor']['deviation_raw']}")
    print(f"TARGET_RAW={document['target']['raw']}")
    print(f"DURATION_MS={document['analytic_profile']['duration_ms']}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(
        "TRACKING_RATE_RAW_S="
        f"{document['physical_tracking_model']['conservative_rate_raw_s']:g}"
    )
    tracking = document["physical_tracking_model"]["legs"][
        "anchor_to_segment_target"
    ]
    print(f"MODELED_PEAK_ERROR_RAW={tracking['maximum_peak_error_raw']:.3f}")
    print(
        "MODELED_TERMINAL_ERROR_RAW="
        f"{tracking['maximum_terminal_error_raw']:.3f}"
    )
    print(f"EXECUTION_API_USED={int(document['execution_api_used'])}")
    print(f"MOTION_AUTHORIZED={int(document['motion_authorized'])}")
    print(f"SHA256={sha256(text.encode('utf-8')).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
