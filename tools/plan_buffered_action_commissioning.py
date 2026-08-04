#!/usr/bin/env python3
"""Create one deterministic, host-only buffered Action commissioning plan."""

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


STATUS = "BUFFERED_ACTION_COMMISSIONING_PLAN_ONLY_PASS"
FIRMWARE_VERSION = "0x00022100"
CAPABILITIES = "0x00000FFF"
PLAN_TICK_MS = 100_000
WAYPOINTS = (
    (0, 0.00),
    (200, 0.25),
    (400, 0.75),
    (600, 1.00),
    (800, 0.75),
    (1_000, 0.25),
    (1_200, 0.00),
)


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _radians_to_raw(
    calibration: ArmCalibration,
    positions_rad: tuple[float, ...],
) -> tuple[int, ...]:
    if len(positions_rad) != len(calibration.joints):
        raise ValueError("position count does not match calibration")
    return tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * 3.141592653589793)
        )
        for joint, position in zip(
            calibration.joints,
            positions_rad,
            strict=True,
        )
    )


def _simulate_admission_batches(plan) -> tuple[list[int], dict[str, object]]:
    scheduler = BufferedBatchScheduler(plan)
    batch_sizes: list[int] = []
    applied = 0

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
    requested_deltas_rad: dict[str, float],
) -> dict[str, object]:
    calibration = load_calibration(calibration_path)
    contract = load_buffered_trajectory_contract(contract_path)
    if contract["motion_authorized"] is not False:
        raise ValueError("contract must keep motion_authorized false")
    if contract["physical_execution_candidate"]["deployed"] is not True:
        raise ValueError("Action physical execution must be commissioned")

    if len(anchor_raw) != 6:
        raise ValueError("anchor must contain six raw positions")
    anchor_rad = tuple(calibration.raw_feedback_to_radians(anchor_raw))
    arm_names = tuple(calibration.ros_joint_names[:5])
    arm_anchor = anchor_rad[:5]
    deltas = tuple(requested_deltas_rad.get(name, 0.0) for name in arm_names)

    points = tuple(
        TrajectoryPointData(
            positions=tuple(
                start + fraction * delta
                for start, delta in zip(arm_anchor, deltas, strict=True)
            ),
            time_from_start_ns=time_ms * 1_000_000,
        )
        for time_ms, fraction in WAYPOINTS
    )
    limits = {
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
        limits,
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
    batch_sizes, admission = _simulate_admission_batches(plan)

    sample_positions_rad = tuple(
        tuple(value / 1_000_000.0 for value in sample.positions_urad)
        for sample in plan.samples
    )
    maximum_sample_step = max(
        abs(current - previous)
        for previous_sample, current_sample in zip(
            sample_positions_rad[:-1],
            sample_positions_rad[1:],
            strict=True,
        )
        for previous, current in zip(
            previous_sample[:5],
            current_sample[:5],
            strict=True,
        )
    )
    apex_positions = points[3].positions + (anchor_rad[5],)
    apex_raw = _radians_to_raw(calibration, apex_positions)

    return {
        "schema_version": 1,
        "status": STATUS,
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "buffered_frame_encoded": False,
        "firmware_version": FIRMWARE_VERSION,
        "capabilities": CAPABILITIES,
        "calibration_hash": f"0x{calibration.calibration_hash:08X}",
        "calibration_sha256": _sha256(calibration_path),
        "contract_status": contract["status"],
        "contract_sha256": _sha256(contract_path),
        "joint_names": list(calibration.ros_joint_names),
        "anchor": {
            "raw": list(anchor_raw),
            "positions_rad": list(anchor_rad),
        },
        "requested_deltas_rad": {
            name: requested_deltas_rad.get(name, 0.0)
            for name in arm_names
        },
        "apex": {
            "raw": list(apex_raw),
            "positions_rad": list(apex_positions),
            "delta_raw": [
                target - start
                for target, start in zip(apex_raw, anchor_raw, strict=True)
            ],
        },
        "waypoints": [
            {
                "time_from_start_ms": time_ms,
                "positions_rad": list(point.positions),
            }
            for (time_ms, _), point in zip(WAYPOINTS, points, strict=True)
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
                    "apply_offset_ms": (
                        sample.apply_tick_ms - PLAN_TICK_MS
                    ),
                    "positions_urad": list(sample.positions_urad),
                }
                for sample in plan.samples
            ],
        },
        "dynamic_limits": {
            "velocity_rad_s": velocity_limits,
            "acceleration_rad_s2": acceleration_limits,
            "segment_velocities_rad_s": [
                list(values) for values in trajectory.segment_velocities_rad_s
            ],
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
    parser.add_argument("--base-delta-rad", type=float, default=0.015)
    parser.add_argument("--shoulder-delta-rad", type=float, default=0.015)
    parser.add_argument("--wrist-roll-delta-rad", type=float, default=0.030)
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

    deltas = {
        "left_base_joint": arguments.base_delta_rad,
        "left_shoulder_joint": arguments.shoulder_delta_rad,
        "left_wrist_roll_joint": arguments.wrist_roll_delta_rad,
    }
    document = build_plan(
        arguments.calibration,
        arguments.contract,
        tuple(arguments.anchor_raw),
        deltas,
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = _sha256(arguments.output)
    print(f"BUFFERED_ACTION_PLAN_ONLY={arguments.output}")
    print(f"STATUS={document['status']}")
    print(f"SAMPLES={document['resampling']['sample_count']}")
    print(
        "BATCHES="
        + ",".join(
            str(value)
            for value in document["queue_contract"]["admission_batch_sizes"]
        )
    )
    print("EXECUTION_API_USED=0")
    print("MOTION_AUTHORIZED=0")
    print(f"SHA256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
