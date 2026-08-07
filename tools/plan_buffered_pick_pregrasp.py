#!/usr/bin/env python3
"""Create a deterministic, non-executable Motion-11 Pick pregrasp plan."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path

from single_arm_bridge.action_validation import (
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (
    MAXIMUM_BATCH_SAMPLES,
    SAMPLE_PERIOD_MS,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.buffered_trajectory import (
    load_buffered_trajectory_contract,
)
from single_arm_bridge.calibration import ArmCalibration, load_calibration

from plan_buffered_q0_roundtrip import (
    minimum_jerk_unit_progress,
    radians_to_raw,
    sha256_file,
    simulate_admission_batches,
    simulate_firmware_output_raw,
)


STATUS = "BUFFERED_PICK_PREGRASP_PLAN_ONLY_PASS"
PHASE = "motion11_pick_pregrasp"
FIRMWARE_VERSION = "0x00022C00"
CAPABILITIES = "0x00000FFF"
PLAN_TICK_MS = 100_000
ANCHOR_TO_Q0_DURATION_MS = 12_000
Q0_TO_PREGRASP_DURATION_MS = 35_000
TOTAL_DURATION_MS = ANCHOR_TO_Q0_DURATION_MS + Q0_TO_PREGRASP_DURATION_MS
FIRMWARE_OUTPUT_PERIOD_MS = 5
Q0_RAW = (2048, 2048, 2048, 2048, 2048)
TRACKING_SIMULATION_PERIOD_MS = 1
CONSERVATIVE_TRACKING_RATE_RAW_S = 50.0
MAXIMUM_MODELED_PEAK_ERROR_RAW = 100.0
MAXIMUM_MODELED_TERMINAL_ERROR_RAW = 30.0
EXPECTED_SOURCE_ROUTE_SHA256 = (
    "da5f3b3fc8200cbc4713e2fcf05d5b54387929ec399377ebc68ce1722587549f"
)


def repository_relative_path(path: Path) -> str:
    """Return a stable repository-relative path for plan evidence."""
    repository_root = Path(__file__).resolve().parents[1]
    try:
        return path.resolve().relative_to(repository_root).as_posix()
    except ValueError as error:
        raise ValueError("source route must be inside the repository") from error


def load_source_route(path: Path) -> tuple[tuple[str, ...], tuple[float, ...], int]:
    if sha256_file(path) != EXPECTED_SOURCE_ROUTE_SHA256:
        raise ValueError("source Pick pregrasp route sha256 mismatch")
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != "PREGRASP_SEGMENT_PLAN_ONLY_PASS":
        raise ValueError("source route status is not approved")
    if document.get("target_name") != "pregrasp":
        raise ValueError("source route target is not pregrasp")
    if document.get("motion_authorized") is not False:
        raise ValueError("source route must keep motion_authorized=false")
    if document.get("execution_api_used") is not False:
        raise ValueError("source route must keep execution_api_used=false")

    names = tuple(document.get("joint_names", ()))
    segments = document.get("segments")
    if len(names) != 5 or not isinstance(segments, list) or len(segments) != 12:
        raise ValueError("source route must contain five joints and 12 segments")
    if any(
        not isinstance(segment, dict)
        or segment.get("index") != index
        or segment.get("success") is not True
        or segment.get("moveit_error_code") != 1
        for index, segment in enumerate(segments, start=1)
    ):
        raise ValueError("source route contains a failed or reindexed segment")

    target = tuple(float(value) for value in segments[-1]["target_positions_rad"])
    if len(target) != 5 or not all(math.isfinite(value) for value in target):
        raise ValueError("source pregrasp target is invalid")
    for index, segment in enumerate(segments, start=1):
        fraction = index / len(segments)
        values = tuple(float(value) for value in segment["target_positions_rad"])
        if len(values) != 5 or any(
            not math.isclose(value, fraction * final, abs_tol=3.0e-16)
            for value, final in zip(values, target, strict=True)
        ):
            raise ValueError("source route is not the reviewed straight joint path")
    return names, target, len(segments)


def path_positions(
    elapsed_ms: int,
    anchor: tuple[float, ...],
    target: tuple[float, ...],
) -> tuple[float, ...]:
    if not 0 <= elapsed_ms <= TOTAL_DURATION_MS:
        raise ValueError("elapsed time is outside the Motion-11 route")
    if elapsed_ms <= ANCHOR_TO_Q0_DURATION_MS:
        progress = minimum_jerk_unit_progress(
            elapsed_ms / ANCHOR_TO_Q0_DURATION_MS
        )
        return tuple(value * (1.0 - progress) for value in anchor)
    progress = minimum_jerk_unit_progress(
        (elapsed_ms - ANCHOR_TO_Q0_DURATION_MS)
        / Q0_TO_PREGRASP_DURATION_MS
    )
    return tuple(progress * value for value in target)


def finite_difference_metrics(
    samples_rad: tuple[tuple[float, ...], ...],
) -> dict[str, float]:
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
    jerks = tuple(
        tuple(
            (current - previous) / period_s
            for previous, current in zip(a, b, strict=True)
        )
        for a, b in zip((zero, *accelerations[:-1]), accelerations, strict=True)
    )

    def maximum(rows) -> float:
        return max(abs(value) for row in rows for value in row)

    q0_index = ANCHOR_TO_Q0_DURATION_MS // SAMPLE_PERIOD_MS
    return {
        "maximum_velocity_rad_s": maximum(velocities),
        "maximum_acceleration_rad_s2": maximum(accelerations),
        "maximum_jerk_rad_s3": maximum(jerks),
        "start_segment_velocity_rad_s": max(abs(v) for v in velocities[0]),
        "q0_inbound_velocity_rad_s": max(
            abs(v) for v in velocities[q0_index - 1]
        ),
        "q0_outbound_velocity_rad_s": max(
            abs(v) for v in velocities[q0_index]
        ),
        "final_segment_velocity_rad_s": max(abs(v) for v in velocities[-1]),
    }


def simulate_rate_limited_tracking(
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    duration_ms: int,
    tracking_rate_raw_s: float = CONSERVATIVE_TRACKING_RATE_RAW_S,
) -> dict[str, object]:
    """Model conservative physical tracking against a minimum-jerk leg."""
    if len(start_raw) != 6 or len(target_raw) != 6:
        raise ValueError("tracking simulation requires six-axis endpoints")
    maximum_step = (
        tracking_rate_raw_s
        * TRACKING_SIMULATION_PERIOD_MS
        / 1000.0
    )
    actual = [float(value) for value in start_raw]
    peak_errors = [0.0] * 6
    step_count = duration_ms // TRACKING_SIMULATION_PERIOD_MS
    for step in range(1, step_count + 1):
        progress = minimum_jerk_unit_progress(step / step_count)
        for index, (start, target) in enumerate(
            zip(start_raw, target_raw, strict=True)
        ):
            command = start + progress * (target - start)
            error = command - actual[index]
            actual[index] += max(-maximum_step, min(maximum_step, error))
            peak_errors[index] = max(
                peak_errors[index],
                abs(command - actual[index]),
            )
    terminal_errors = [
        abs(target - value)
        for target, value in zip(target_raw, actual, strict=True)
    ]
    return {
        "duration_ms": duration_ms,
        "peak_error_raw": peak_errors,
        "terminal_error_raw": terminal_errors,
        "maximum_peak_error_raw": max(peak_errors),
        "maximum_terminal_error_raw": max(terminal_errors),
    }


def build_plan(
    calibration_path: Path,
    contract_path: Path,
    source_route_path: Path,
    anchor_raw: tuple[int, ...],
) -> dict[str, object]:
    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized=false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("buffered physical execution must be commissioned")
    uart_candidate = contract["servo_uart_receive_candidate"]
    if uart_candidate["firmware_version"] != FIRMWARE_VERSION:
        raise ValueError("servo UART candidate firmware version is inconsistent")
    if uart_candidate["motion_authorized"] is not False:
        raise ValueError("servo UART candidate must keep motion_authorized=false")
    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")

    source_names, target, source_segment_count = load_source_route(
        source_route_path
    )
    arm_names = tuple(calibration.ros_joint_names[:5])
    if source_names != arm_names:
        raise ValueError("source route joint order does not match calibration")

    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_anchor = anchor_rad[:5]
    elapsed_values = range(0, TOTAL_DURATION_MS + SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS)
    points = tuple(
        TrajectoryPointData(
            positions=path_positions(elapsed_ms, arm_anchor, target),
            time_from_start_ns=elapsed_ms * 1_000_000,
        )
        for elapsed_ms in elapsed_values
    )

    position_limits = {
        name: calibration.ros_radian_limits[name] for name in arm_names
    }
    velocity_limits = {name: 0.5 for name in arm_names}
    acceleration_limits = {name: 1.0 for name in arm_names}
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
        velocity_limits,
        acceleration_limits,
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
    dynamics = finite_difference_metrics(samples_rad)
    maximum_sample_step = max(
        abs(current - previous)
        for a, b in zip(samples_rad[:-1], samples_rad[1:], strict=True)
        for previous, current in zip(a[:5], b[:5], strict=True)
    )
    firmware_raw_trace = simulate_firmware_output_raw(calibration, plan.samples)
    raw_steps = tuple(
        tuple(current - previous for previous, current in zip(a, b, strict=True))
        for a, b in zip(
            firmware_raw_trace[:-1],
            firmware_raw_trace[1:],
            strict=True,
        )
    )
    target_with_gripper = target + (anchor_rad[5],)
    target_raw = radians_to_raw(calibration, target_with_gripper)
    q0_with_gripper = (0.0,) * 5 + (anchor_rad[5],)
    q0_raw = radians_to_raw(calibration, q0_with_gripper)
    if q0_raw[:5] != Q0_RAW:
        raise ValueError("calibration does not map arm q0 to raw 2048")
    tracking_legs = {
        "anchor_to_q0": simulate_rate_limited_tracking(
            anchor_raw,
            q0_raw,
            ANCHOR_TO_Q0_DURATION_MS,
        ),
        "q0_to_pregrasp": simulate_rate_limited_tracking(
            q0_raw,
            target_raw,
            Q0_TO_PREGRASP_DURATION_MS,
        ),
    }
    maximum_modeled_peak_error = max(
        leg["maximum_peak_error_raw"] for leg in tracking_legs.values()
    )
    maximum_modeled_terminal_error = max(
        leg["maximum_terminal_error_raw"] for leg in tracking_legs.values()
    )
    if maximum_modeled_peak_error > MAXIMUM_MODELED_PEAK_ERROR_RAW:
        raise ValueError("modeled physical tracking peak error is too large")
    if maximum_modeled_terminal_error > MAXIMUM_MODELED_TERMINAL_ERROR_RAW:
        raise ValueError("modeled physical tracking terminal error is too large")

    return {
        "schema_version": 1,
        "status": STATUS,
        "phase": PHASE,
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "buffered_frame_encoded": False,
        "firmware_version": FIRMWARE_VERSION,
        "firmware_deployment_gate": {
            "candidate_status": uart_candidate["status"],
            "deployed": uart_candidate["deployed"],
            "motion_authorized": False,
        },
        "capabilities": CAPABILITIES,
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": sha256_file(calibration_path),
        "contract_status": contract["status"],
        "contract_sha256": sha256_file(contract_path),
        "joint_names": list(calibration.ros_joint_names),
        "source_route": {
            "path": repository_relative_path(source_route_path),
            "sha256": EXPECTED_SOURCE_ROUTE_SHA256,
            "segment_count": source_segment_count,
            "collision_checked": True,
            "targets_exactly_collinear_from_q0": True,
        },
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
        },
        "q0_transition": {
            "time_from_start_ms": ANCHOR_TO_Q0_DURATION_MS,
            "raw_with_preserved_gripper": list(q0_raw),
            "settle_wait_ms": 0,
        },
        "target": {
            "name": "pregrasp",
            "positions_rad": list(target_with_gripper),
            "raw": list(target_raw),
            "gripper_preserved": True,
        },
        "analytic_profile": {
            "kind": "two_leg_quintic_minimum_jerk",
            "polynomial": "10t^3-15t^4+6t^5",
            "anchor_to_q0_duration_ms": ANCHOR_TO_Q0_DURATION_MS,
            "q0_to_pregrasp_duration_ms": Q0_TO_PREGRASP_DURATION_MS,
            "total_duration_ms": TOTAL_DURATION_MS,
            "waypoint_period_ms": SAMPLE_PERIOD_MS,
            "waypoint_count": len(points),
            "q0_settle_wait_ms": 0,
        },
        "waypoints": [
            {
                "time_from_start_ms": point.time_from_start_ns // 1_000_000,
                "positions_rad": list(point.positions),
            }
            for point in points
        ],
        "resampling": {
            "period_ms": SAMPLE_PERIOD_MS,
            "duration_ms": trajectory.duration_ms,
            "sample_count": len(plan.samples),
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
            "velocity_rad_s": velocity_limits,
            "acceleration_rad_s2": acceleration_limits,
            "finite_difference": dynamics,
        },
        "physical_tracking_model": {
            "kind": "per_axis_rate_limited_minimum_jerk_follower",
            "simulation_period_ms": TRACKING_SIMULATION_PERIOD_MS,
            "conservative_rate_raw_s": CONSERVATIVE_TRACKING_RATE_RAW_S,
            "measured_rate_evidence_raw_s": {
                "left_shoulder_joint": 60.0,
                "left_wrist_flex_joint": 60.8,
            },
            "maximum_allowed_peak_error_raw": MAXIMUM_MODELED_PEAK_ERROR_RAW,
            "maximum_allowed_terminal_error_raw": (
                MAXIMUM_MODELED_TERMINAL_ERROR_RAW
            ),
            "maximum_modeled_peak_error_raw": maximum_modeled_peak_error,
            "maximum_modeled_terminal_error_raw": (
                maximum_modeled_terminal_error
            ),
            "legs": tracking_legs,
        },
        "firmware_output_simulation": {
            "executor_step_period_ms": 1,
            "servo_sync_write_period_ms": FIRMWARE_OUTPUT_PERIOD_MS,
            "output_count": len(firmware_raw_trace),
            "maximum_arm_step_raw": max(
                abs(value) for row in raw_steps for value in row[:5]
            ),
            "start_raw": list(firmware_raw_trace[0]),
            "q0_raw": list(
                firmware_raw_trace[
                    ANCHOR_TO_Q0_DURATION_MS // FIRMWARE_OUTPUT_PERIOD_MS
                ]
            ),
            "final_raw": list(firmware_raw_trace[-1]),
        },
        "queue_contract": {
            "maximum_batch_samples": MAXIMUM_BATCH_SAMPLES,
            "admission_batch_sizes": batch_sizes,
            "simulation_terminal": admission,
        },
    }


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws" / "src" / "single_arm_bridge"
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--anchor-raw", type=int, nargs=6, required=True)
    parser.add_argument(
        "--source-route",
        required=True,
        type=Path,
    )
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
    if not arguments.plan_only:
        parser.error("--plan-only is required; execution is not available")

    document = build_plan(
        arguments.calibration,
        arguments.contract,
        arguments.source_route,
        tuple(arguments.anchor_raw),
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = sha256(arguments.output.read_bytes()).hexdigest()
    print(f"MOTION11_PICK_PREGRASP_PLAN={arguments.output}")
    print(f"STATUS={document['status']}")
    print(f"DURATION_MS={document['resampling']['duration_ms']}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(f"MAXIMUM_SAMPLE_STEP_RAD={document['resampling']['maximum_sample_step_rad']:.9f}")
    print("EXECUTION_API_USED=0")
    print("MOTION_AUTHORIZED=0")
    print(f"SHA256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
