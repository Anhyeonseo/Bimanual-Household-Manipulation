#!/usr/bin/env python3
"""Create a deterministic, non-executable buffered arm q0 round-trip plan."""

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
    LOW_WATERMARK_SAMPLES,
    MAXIMUM_BATCH_SAMPLES,
    REFILL_TARGET_SAMPLES,
    SAMPLE_PERIOD_MS,
    STARTUP_PRIME_SAMPLES,
    STARTUP_PRIME_MINIMUM_ELAPSED_MS,
    BufferedAdapterState,
    BufferedBatchScheduler,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.buffered_trajectory import (
    load_buffered_trajectory_contract,
)
from single_arm_bridge.calibration import ArmCalibration, load_calibration


STATUS = "BUFFERED_Q0_ROUNDTRIP_PLAN_ONLY_PASS"
FIRMWARE_VERSION = "0x00022100"
CAPABILITIES = "0x00000FFF"
PLAN_TICK_MS = 100_000
Q0_RAW = (2048, 2048, 2048, 2048, 2048)
ROUND_TRIP_DURATION_MS = 4_200
HALF_TRIP_DURATION_MS = ROUND_TRIP_DURATION_MS // 2
FIRMWARE_OUTPUT_PERIOD_MS = 5
TURN_URAD = 6_283_185
RAW_UNITS_PER_TURN = 4_096


def minimum_jerk_unit_progress(unit_time: float) -> float:
    """Return 10t^3 - 15t^4 + 6t^5 on the closed unit interval."""

    if not 0.0 <= unit_time <= 1.0:
        raise ValueError("minimum-jerk unit time must be within 0..1")
    return unit_time**3 * (
        10.0 + unit_time * (-15.0 + 6.0 * unit_time)
    )


def q0_progress(elapsed_ms: int) -> float:
    """Symmetric analytic minimum-jerk progress for anchor -> q0 -> anchor."""

    if not 0 <= elapsed_ms <= ROUND_TRIP_DURATION_MS:
        raise ValueError("elapsed time is outside the q0 round-trip")
    if elapsed_ms <= HALF_TRIP_DURATION_MS:
        return minimum_jerk_unit_progress(
            elapsed_ms / HALF_TRIP_DURATION_MS
        )
    return 1.0 - minimum_jerk_unit_progress(
        (elapsed_ms - HALF_TRIP_DURATION_MS) / HALF_TRIP_DURATION_MS
    )


DENSE_WAYPOINTS = tuple(
    (elapsed_ms, q0_progress(elapsed_ms))
    for elapsed_ms in range(
        0,
        ROUND_TRIP_DURATION_MS + SAMPLE_PERIOD_MS,
        SAMPLE_PERIOD_MS,
    )
)


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def radians_to_raw(
    calibration: ArmCalibration,
    positions_rad: tuple[float, ...],
) -> tuple[int, ...]:
    if len(positions_rad) != len(calibration.joints):
        raise ValueError("position count does not match calibration")
    return tuple(
        round(
            joint.zero_raw
            + joint.direction
            * position
            * 4096.0
            / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            calibration.joints,
            positions_rad,
            strict=True,
        )
    )


def round_divide(numerator: int, denominator: int) -> int:
    """Match the signed round-to-nearest helper used by STM32 calibration."""

    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def urad_to_raw(calibration: ArmCalibration, positions_urad) -> tuple[int, ...]:
    if len(positions_urad) != len(calibration.joints):
        raise ValueError("microradian position count does not match calibration")
    result = []
    for joint, position_urad in zip(
        calibration.joints,
        positions_urad,
        strict=True,
    ):
        raw_delta = round_divide(
            int(position_urad) * RAW_UNITS_PER_TURN,
            TURN_URAD,
        )
        raw = joint.zero_raw + joint.direction * raw_delta
        if not joint.minimum_raw <= raw <= joint.maximum_raw:
            raise ValueError(f"{joint.name}: interpolated raw target is unsafe")
        result.append(raw)
    return tuple(result)


def simulate_firmware_output_raw(
    calibration: ArmCalibration,
    samples,
) -> tuple[tuple[int, ...], ...]:
    """Reproduce the 5 ms STM32 interpolation and urad-to-raw conversion."""

    if len(samples) < 2:
        raise ValueError("firmware output simulation requires two samples")
    trace = [urad_to_raw(calibration, samples[0].positions_urad)]
    for previous, current in zip(samples[:-1], samples[1:], strict=True):
        for elapsed_ms in range(
            FIRMWARE_OUTPUT_PERIOD_MS,
            SAMPLE_PERIOD_MS + FIRMWARE_OUTPUT_PERIOD_MS,
            FIRMWARE_OUTPUT_PERIOD_MS,
        ):
            interpolated = tuple(
                start + int((end - start) * elapsed_ms / SAMPLE_PERIOD_MS)
                for start, end in zip(
                    previous.positions_urad,
                    current.positions_urad,
                    strict=True,
                )
            )
            trace.append(urad_to_raw(calibration, interpolated))
    return tuple(trace)


def finite_difference_metrics(samples_rad) -> dict[str, float]:
    sample_period_s = SAMPLE_PERIOD_MS / 1000.0
    velocities = tuple(
        tuple(
            (current - previous) / sample_period_s
            for previous, current in zip(
                previous_sample[:5],
                current_sample[:5],
                strict=True,
            )
        )
        for previous_sample, current_sample in zip(
            samples_rad[:-1],
            samples_rad[1:],
            strict=True,
        )
    )
    zero = (0.0,) * 5
    accelerations = tuple(
        tuple(
            (current - previous) / sample_period_s
            for previous, current in zip(
                previous_velocity,
                current_velocity,
                strict=True,
            )
        )
        for previous_velocity, current_velocity in zip(
            (zero, *velocities[:-1]),
            velocities,
            strict=True,
        )
    )
    jerks = tuple(
        tuple(
            (current - previous) / sample_period_s
            for previous, current in zip(
                previous_acceleration,
                current_acceleration,
                strict=True,
            )
        )
        for previous_acceleration, current_acceleration in zip(
            (zero, *accelerations[:-1]),
            accelerations,
            strict=True,
        )
    )

    def maximum_absolute(rows) -> float:
        return max(abs(value) for row in rows for value in row)

    midpoint = HALF_TRIP_DURATION_MS // SAMPLE_PERIOD_MS
    return {
        "maximum_velocity_rad_s": maximum_absolute(velocities),
        "maximum_acceleration_rad_s2": maximum_absolute(accelerations),
        "maximum_jerk_rad_s3": maximum_absolute(jerks),
        "start_segment_velocity_rad_s": max(abs(value) for value in velocities[0]),
        "q0_inbound_velocity_rad_s": max(
            abs(value) for value in velocities[midpoint - 1]
        ),
        "q0_outbound_velocity_rad_s": max(
            abs(value) for value in velocities[midpoint]
        ),
        "final_segment_velocity_rad_s": max(abs(value) for value in velocities[-1]),
    }


def simulate_admission_batches(plan) -> tuple[list[int], dict[str, object]]:
    scheduler = BufferedBatchScheduler(plan)
    batch_sizes: list[int] = []

    first = scheduler.next_batch(current_tick_ms=PLAN_TICK_MS)
    if first is None:
        raise RuntimeError("startup prime first batch was not produced")
    batch_sizes.append(first.sample_count)
    scheduler.acknowledge_batch(
        status_code=0,
        accepted_samples=first.accepted_samples_after_ack,
        applied_samples=0,
        queued_samples=first.accepted_samples_after_ack,
    )

    second = scheduler.next_batch(current_tick_ms=PLAN_TICK_MS + STARTUP_PRIME_MINIMUM_ELAPSED_MS)
    if second is None:
        raise RuntimeError("startup prime second batch was not produced")
    batch_sizes.append(second.sample_count)
    scheduler.acknowledge_batch(
        status_code=0,
        accepted_samples=second.accepted_samples_after_ack,
        applied_samples=0,
        queued_samples=second.accepted_samples_after_ack,
    )

    while scheduler.snapshot().accepted_samples < len(plan.samples):
        snapshot = scheduler.snapshot()
        applied = snapshot.accepted_samples - LOW_WATERMARK_SAMPLES
        scheduler.record_clock_progress(applied)
        current_tick = (
            plan.samples[0].apply_tick_ms
            + max(0, applied - 1) * SAMPLE_PERIOD_MS
        ) & 0xFFFFFFFF
        batch = scheduler.next_batch(current_tick_ms=current_tick)
        if batch is None:
            raise RuntimeError("refill batch was not produced at low watermark")
        batch_sizes.append(batch.sample_count)
        scheduler.acknowledge_batch(
            status_code=0,
            accepted_samples=batch.accepted_samples_after_ack,
            applied_samples=applied,
            queued_samples=batch.accepted_samples_after_ack - applied,
        )

    snapshot = scheduler.snapshot()
    if snapshot.state is not BufferedAdapterState.INPUT_COMPLETE:
        raise RuntimeError("admission simulation did not reach input_complete")
    if snapshot.accepted_samples != len(plan.samples):
        raise RuntimeError("admission simulation did not accept every sample")
    if snapshot.safe_stop_required:
        raise RuntimeError("admission simulation unexpectedly requested safe stop")

    return batch_sizes, {
        "state": snapshot.state.value,
        "accepted_samples": snapshot.accepted_samples,
        "applied_samples": snapshot.applied_samples,
        "queued_samples": snapshot.queued_samples,
        "safe_stop_required": snapshot.safe_stop_required,
        "success_without_firmware_terminal": False,
    }


def build_plan(
    calibration_path: Path,
    contract_path: Path,
    anchor_raw: tuple[int, ...],
) -> dict[str, object]:
    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("buffered physical execution must be commissioned")
    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")

    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_names = tuple(calibration.ros_joint_names[:5])
    arm_anchor = anchor_rad[:5]
    q0_positions = tuple(0.0 for _ in arm_names)
    points = tuple(
        TrajectoryPointData(
            positions=tuple(
                start + fraction * (target - start)
                for start, target in zip(
                    arm_anchor,
                    q0_positions,
                    strict=True,
                )
            ),
            time_from_start_ns=time_ms * 1_000_000,
        )
        for time_ms, fraction in DENSE_WAYPOINTS
    )

    position_limits = {
        name: calibration.ros_radian_limits[name]
        for name in arm_names
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
        for previous_sample, current_sample in zip(
            samples_rad[:-1],
            samples_rad[1:],
            strict=True,
        )
        for previous, current in zip(
            previous_sample[:5],
            current_sample[:5],
            strict=True,
        )
    )
    q0_with_gripper = q0_positions + (anchor_rad[5],)
    q0_raw = radians_to_raw(calibration, q0_with_gripper)
    if q0_raw[:5] != Q0_RAW:
        raise ValueError("calibration does not map arm q0 to raw 2048")
    maximum_q0_error_raw = max(
        abs(value - expected)
        for value, expected in zip(q0_raw[:5], Q0_RAW, strict=True)
    )
    firmware_raw_trace = simulate_firmware_output_raw(
        calibration,
        plan.samples,
    )
    raw_steps = tuple(
        tuple(
            current - previous
            for previous, current in zip(
                previous_output,
                current_output,
                strict=True,
            )
        )
        for previous_output, current_output in zip(
            firmware_raw_trace[:-1],
            firmware_raw_trace[1:],
            strict=True,
        )
    )
    arm_names = tuple(calibration.ros_joint_names[:5])
    per_axis_raw_output = {
        name: {
            "maximum_step_raw": max(
                abs(output_step[index]) for output_step in raw_steps
            ),
            "unchanged_output_ratio": sum(
                output_step[index] == 0 for output_step in raw_steps
            ) / len(raw_steps),
        }
        for index, name in enumerate(arm_names)
    }

    return {
        "schema_version": 1,
        "status": STATUS,
        "phase": "motion10_q0_roundtrip",
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "buffered_frame_encoded": False,
        "firmware_version": FIRMWARE_VERSION,
        "capabilities": CAPABILITIES,
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": sha256_file(calibration_path),
        "contract_status": contract["status"],
        "contract_sha256": sha256_file(contract_path),
        "joint_names": list(calibration.ros_joint_names),
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
        },
        "q0": {
            "arm_positions_rad": list(q0_positions),
            "raw_with_preserved_gripper": list(q0_raw),
            "maximum_arm_error_raw": maximum_q0_error_raw,
            "gripper_preserved": True,
        },
        "waypoints": [
            {
                "time_from_start_ms": time_ms,
                "q0_progress": fraction,
                "positions_rad": list(point.positions),
            }
            for (time_ms, fraction), point in zip(
                DENSE_WAYPOINTS,
                points,
                strict=True,
            )
        ],
        "analytic_profile": {
            "kind": "symmetric_quintic_minimum_jerk",
            "polynomial": "10t^3-15t^4+6t^5",
            "half_trip_duration_ms": HALF_TRIP_DURATION_MS,
            "waypoint_period_ms": SAMPLE_PERIOD_MS,
            "waypoint_count": len(DENSE_WAYPOINTS),
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
        },
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
            "segment_velocities_rad_s": [
                list(values)
                for values in trajectory.segment_velocities_rad_s
            ],
        },
        "firmware_output_simulation": {
            "executor_step_period_ms": 1,
            "servo_sync_write_period_ms": FIRMWARE_OUTPUT_PERIOD_MS,
            "output_count": len(firmware_raw_trace),
            "maximum_arm_step_raw": max(
                abs(value)
                for output_step in raw_steps
                for value in output_step[:5]
            ),
            "per_axis": per_axis_raw_output,
            "start_raw": list(firmware_raw_trace[0]),
            "q0_raw": list(
                firmware_raw_trace[
                    HALF_TRIP_DURATION_MS // FIRMWARE_OUTPUT_PERIOD_MS
                ]
            ),
            "final_raw": list(firmware_raw_trace[-1]),
        },
        "queue_contract": {
            "startup_prime_samples": STARTUP_PRIME_SAMPLES,
            "low_watermark_samples": LOW_WATERMARK_SAMPLES,
            "refill_target_samples": REFILL_TARGET_SAMPLES,
            "maximum_batch_samples": MAXIMUM_BATCH_SAMPLES,
            "admission_batch_sizes": batch_sizes,
            "simulation_terminal": admission,
        },
        "round_trip": {
            "q0_time_from_start_ms": HALF_TRIP_DURATION_MS,
            "final_positions_rad": list(trajectory.ordered_points[-1]),
            "maximum_return_error_rad": max(
                abs(final - start)
                for final, start in zip(
                    trajectory.ordered_points[-1],
                    arm_anchor,
                    strict=True,
                )
            ),
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
        tuple(arguments.anchor_raw),
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"BUFFERED_Q0_ROUNDTRIP_PLAN={arguments.output}")
    print(f"STATUS={document['status']}")
    print(f"DURATION_MS={document['resampling']['duration_ms']}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(f"MAXIMUM_SAMPLE_STEP_RAD={document['resampling']['maximum_sample_step_rad']:.9f}")
    print("EXECUTION_API_USED=0")
    print("MOTION_AUTHORIZED=0")
    print(f"SHA256={sha256_file(arguments.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
