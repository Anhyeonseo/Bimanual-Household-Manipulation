#!/usr/bin/env python3
"""Execute one tightly bounded 300 ms buffered arm-joint delta.

This commissioning tool is intentionally separate from the ROS Action server.
It sends exactly the 9+7 startup-prime frames, accepts one matching extended
terminal, requires two consecutive six-axis settle snapshots, and physically
disables all servos before closing the serial port.  It never clears a fault or
retries a command frame.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time
from typing import NamedTuple

from single_arm_bridge.action_validation import (
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (
    BufferedAdapterState,
    BufferedBatchScheduler,
    prepare_buffered_execution_plan,
)
from single_arm_bridge.buffered_transport_driver import BufferedTransportDriver
from single_arm_bridge.calibration import ArmCalibration, load_calibration
from single_arm_bridge.hardware_identity import validate_hardware_identity
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.transport import ActuatorTransport


EXPECTED_FIRMWARE = 0x00022100
EXPECTED_CALIBRATION = 0xB317C672
EXPECTED_CAPABILITIES = 0x00000FFF
DURATION_MS = 300
MAXIMUM_ABSOLUTE_DELTA_RAD = 0.03
MINIMUM_OBSERVABLE_COMMAND_RAW = 16
MINIMUM_DIRECTIONAL_PROGRESS_RAW = 10
SELECTED_JOINT_TARGET_TOLERANCE_RAW = 8
OTHER_AXIS_SETTLE_TOLERANCE_RAW = 30
SETTLE_REQUIRED_CONSECUTIVE = 2
SETTLE_TIMEOUT_S = 1.5
TERMINAL_TIMEOUT_S = 2.0
CONFIRMATION = "EXECUTE_BUFFERED_SINGLE_JOINT_ONCE"


class ObservableMotionMetrics(NamedTuple):
    planned_delta_raw: int
    observed_delta_raw: int
    target_error_raw: int
    maximum_other_axis_error_raw: int
    direction_ok: bool
    progress_ok: bool
    target_ok: bool
    other_axes_ok: bool

    @property
    def passed(self) -> bool:
        return (
            self.direction_ok
            and self.progress_ok
            and self.target_ok
            and self.other_axes_ok
        )


def format_buffered_terminal(response) -> str:
    """Return every field needed to distinguish success from HOLD causes."""

    result = response.result
    return (
        "BUFFERED_TERMINAL_RECEIVED "
        f"frame_sequence={response.frame_sequence} "
        f"request_sequence={result.request_sequence} "
        f"status={result.status_code} safety_state={result.safety_state} "
        f"detail={result.detail} sample_count={result.sample_count} "
        f"apply_tick={result.apply_tick_ms} "
        f"calibration=0x{result.calibration_hash:08X} "
        f"executor_state={result.executor_state} "
        f"terminal_reason={result.terminal_reason} "
        f"safe_stop_required={result.safe_stop_required} "
        f"queue_result={result.queue_result} "
        f"queued={result.queued_samples} "
        f"peak_queued={result.peak_queued_samples} "
        f"accepted={result.accepted_samples} "
        f"applied={result.applied_samples}"
    )


def radians_to_raw_positions(
    calibration: ArmCalibration,
    positions_rad: tuple[float, ...],
) -> tuple[int, ...]:
    if len(positions_rad) != len(calibration.joints):
        raise ValueError("position count does not match calibration")
    result = []
    for joint, position in zip(
        calibration.joints,
        positions_rad,
        strict=True,
    ):
        raw = round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * math.pi)
        )
        if not joint.minimum_raw <= raw <= joint.maximum_raw:
            raise ValueError(
                f"{joint.name}: target raw {raw} outside "
                f"{joint.minimum_raw}..{joint.maximum_raw}"
            )
        result.append(raw)
    return tuple(result)


def build_execution_plan(
    calibration: ArmCalibration,
    raw_positions: tuple[int, ...],
    *,
    joint_name: str,
    delta_rad: float,
    current_tick_ms: int,
):
    arm_names = tuple(calibration.ros_joint_names[:5])
    if joint_name not in arm_names:
        raise ValueError("joint must be one of the five arm joints")
    if (
        not math.isfinite(delta_rad)
        or delta_rad == 0.0
        or abs(delta_rad) > MAXIMUM_ABSOLUTE_DELTA_RAD
    ):
        raise ValueError("delta must be non-zero and at most 0.03 rad")

    positions = tuple(calibration.raw_feedback_to_radians(raw_positions))
    start_arm = positions[:5]
    target_arm = list(start_arm)
    target_arm[arm_names.index(joint_name)] += delta_rad
    target_arm_tuple = tuple(target_arm)

    trajectory = validate_buffered_trajectory(
        arm_names,
        (
            TrajectoryPointData(start_arm, 0),
            TrajectoryPointData(
                target_arm_tuple,
                DURATION_MS * 1_000_000,
            ),
        ),
        arm_names,
        {name: calibration.ros_radian_limits[name] for name in arm_names},
        start_arm,
        {name: 0.2 for name in arm_names},
        {name: 1.0 for name in arm_names},
        start_tolerance_rad=0.0,
    )
    plan = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=positions[5],
        current_tick_ms=current_tick_ms,
    )
    target_raw = radians_to_raw_positions(
        calibration,
        (*target_arm_tuple, positions[5]),
    )
    require_observable_command(
        raw_positions,
        target_raw,
        joint_index=arm_names.index(joint_name),
    )
    return plan, target_raw


def require_observable_command(
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    *,
    joint_index: int,
) -> int:
    if len(start_raw) != len(target_raw):
        raise ValueError("start/target raw position count mismatch")
    if not 0 <= joint_index < len(start_raw):
        raise ValueError("selected joint index is outside the raw position vector")

    planned_delta_raw = target_raw[joint_index] - start_raw[joint_index]
    if abs(planned_delta_raw) < MINIMUM_OBSERVABLE_COMMAND_RAW:
        raise ValueError(
            "commissioning delta is not physically observable: "
            f"planned_delta_raw={planned_delta_raw} "
            f"minimum={MINIMUM_OBSERVABLE_COMMAND_RAW}"
        )
    return planned_delta_raw


def preflight_execution_request(
    calibration: ArmCalibration,
    raw_positions: tuple[int, ...],
    *,
    joint_name: str,
    delta_rad: float,
) -> None:
    """Reject an unsafe current pose or target before ARM/ENABLE."""

    build_execution_plan(
        calibration,
        raw_positions,
        joint_name=joint_name,
        delta_rad=delta_rad,
        current_tick_ms=0,
    )


def maximum_settle_error_raw(snapshot, target_raw: tuple[int, ...]) -> int:
    if len(snapshot.joints) != len(target_raw):
        raise RuntimeError("diagnostic joint count mismatch")
    if any(not joint.torque_enabled for joint in snapshot.joints):
        raise RuntimeError("torque disabled before settle verification")
    return max(
        abs(joint.position_raw - target)
        for joint, target in zip(snapshot.joints, target_raw, strict=True)
    )


def observable_motion_metrics(
    snapshot,
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    *,
    joint_index: int,
) -> ObservableMotionMetrics:
    if len(snapshot.joints) != len(start_raw) or len(start_raw) != len(target_raw):
        raise RuntimeError("diagnostic/start/target position count mismatch")
    if any(not joint.torque_enabled for joint in snapshot.joints):
        raise RuntimeError("torque disabled before observable motion verification")

    planned_delta_raw = require_observable_command(
        start_raw,
        target_raw,
        joint_index=joint_index,
    )
    selected_position_raw = snapshot.joints[joint_index].position_raw
    observed_delta_raw = selected_position_raw - start_raw[joint_index]
    target_error_raw = abs(selected_position_raw - target_raw[joint_index])
    maximum_other_axis_error_raw = max(
        (
            abs(joint.position_raw - target)
            for index, (joint, target) in enumerate(
                zip(snapshot.joints, target_raw, strict=True)
            )
            if index != joint_index
        ),
        default=0,
    )

    return ObservableMotionMetrics(
        planned_delta_raw=planned_delta_raw,
        observed_delta_raw=observed_delta_raw,
        target_error_raw=target_error_raw,
        maximum_other_axis_error_raw=maximum_other_axis_error_raw,
        direction_ok=(planned_delta_raw * observed_delta_raw) > 0,
        progress_ok=(
            abs(observed_delta_raw) >= MINIMUM_DIRECTIONAL_PROGRESS_RAW
        ),
        target_ok=(target_error_raw <= SELECTED_JOINT_TARGET_TOLERANCE_RAW),
        other_axes_ok=(
            maximum_other_axis_error_raw <= OTHER_AXIS_SETTLE_TOLERANCE_RAW
        ),
    )


def require_observable_settled(
    transport: ActuatorTransport,
    start_raw: tuple[int, ...],
    target_raw: tuple[int, ...],
    *,
    joint_index: int,
) -> tuple[object, ObservableMotionMetrics]:
    deadline = time.monotonic() + SETTLE_TIMEOUT_S
    consecutive = 0
    last_metrics = None
    while time.monotonic() < deadline:
        snapshot = transport.get_diagnostics()
        metrics = observable_motion_metrics(
            snapshot,
            start_raw,
            target_raw,
            joint_index=joint_index,
        )
        last_metrics = metrics
        if metrics.passed:
            consecutive += 1
            if consecutive >= SETTLE_REQUIRED_CONSECUTIVE:
                return snapshot, metrics
        else:
            consecutive = 0
        time.sleep(0.1)

    if last_metrics is None:
        raise RuntimeError("observable motion verification captured no diagnostics")
    raise RuntimeError(
        "buffered motion failed observable physical gate: "
        f"planned_delta_raw={last_metrics.planned_delta_raw} "
        f"observed_delta_raw={last_metrics.observed_delta_raw} "
        f"target_error_raw={last_metrics.target_error_raw} "
        "maximum_other_axis_error_raw="
        f"{last_metrics.maximum_other_axis_error_raw} "
        f"direction_ok={last_metrics.direction_ok} "
        f"progress_ok={last_metrics.progress_ok} "
        f"target_ok={last_metrics.target_ok} "
        f"other_axes_ok={last_metrics.other_axes_ok}"
    )


def require_disabled(snapshot) -> None:
    enabled = [joint.servo_id for joint in snapshot.joints if joint.torque_enabled]
    if enabled:
        raise RuntimeError(f"physical DISABLE failed for servo IDs {enabled}")


def disable_after_attempt(
    transport: ActuatorTransport,
    *,
    verify_readback: bool,
) -> None:
    """Physically disable, reading six axes only while heartbeat is legal.

    A failed motion first enters SAFE_STOP, whose latch intentionally rejects
    heartbeat-backed diagnostics until reset or CLEAR_FAULT.  The failure path
    therefore requires the DISABLE acknowledgement but never clears the latch.
    """

    transport.disable()
    if not verify_readback:
        print("PHYSICAL_DISABLE_ACK_PASS_LATCH_PRESERVED")
        return
    disabled = transport.get_diagnostics()
    require_disabled(disabled)
    print("PHYSICAL_DISABLE_6AXIS_PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--joint", required=True)
    parser.add_argument("--delta-rad", type=float, required=True)
    parser.add_argument("--confirmation", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("explicit execution confirmation token is required")

    import serial

    calibration = load_calibration(args.calibration)
    if calibration.calibration_hash != EXPECTED_CALIBRATION:
        raise RuntimeError("unexpected calibration file hash")

    port = open_exclusive_serial(serial, args.device, 115200, 0.4)
    transport = ActuatorTransport(port, response_timeout_s=0.4)
    binary_mode_entered = False
    motion_authority_attempted = False
    execution_succeeded = False
    primary_error: BaseException | None = None
    try:
        hello = transport.enter_binary_mode()
        binary_mode_entered = True
        validate_hardware_identity(hello, calibration.calibration_hash)
        if (
            hello.firmware_version != EXPECTED_FIRMWARE
            or hello.capabilities != EXPECTED_CAPABILITIES
        ):
            raise RuntimeError("unexpected buffered execution identity")

        before = transport.get_state(include_positions=True)
        if before.raw_positions is None:
            raise RuntimeError("fresh start position feedback is missing")

        preflight_execution_request(
            calibration,
            before.raw_positions,
            joint_name=args.joint,
            delta_rad=args.delta_rad,
        )
        print("BUFFERED_PRE_ARM_POSE_AND_TARGET_GATE=PASS")

        motion_authority_attempted = True
        transport.arm_and_enable(calibration.calibration_hash)
        enabled = transport.get_state(include_positions=True)
        if enabled.raw_positions is None:
            raise RuntimeError("enabled fresh position feedback is missing")
        heartbeat = transport.heartbeat()

        plan, target_raw = build_execution_plan(
            calibration,
            enabled.raw_positions,
            joint_name=args.joint,
            delta_rad=args.delta_rad,
            current_tick_ms=heartbeat.last_heartbeat_ms,
        )
        joint_index = tuple(calibration.ros_joint_names).index(args.joint)
        planned_delta_raw = require_observable_command(
            enabled.raw_positions,
            target_raw,
            joint_index=joint_index,
        )
        scheduler = BufferedBatchScheduler(plan)
        driver = BufferedTransportDriver(scheduler, transport)

        print(
            "BUFFERED_EXECUTION_REQUEST "
            f"joint={args.joint} delta_rad={args.delta_rad:.6f} "
            f"duration_ms={DURATION_MS} samples={len(plan.samples)} "
            f"anchor_tick={plan.anchor_tick_ms} "
            f"start_raw={enabled.raw_positions[joint_index]} "
            f"target_raw={target_raw[joint_index]} "
            f"planned_delta_raw={planned_delta_raw}"
        )
        first = driver.service_once(current_tick_ms=heartbeat.last_heartbeat_ms)
        second_clock = transport.heartbeat()
        second = driver.service_once(
            current_tick_ms=second_clock.last_heartbeat_ms
        )
        if first is None or second is None or driver.commands_sent != 2:
            raise RuntimeError("startup prime did not use exactly two frames")
        if driver.service_once(
            current_tick_ms=second_clock.last_heartbeat_ms
        ) is not None:
            raise RuntimeError("unexpected third buffered command frame")

        deadline = time.monotonic() + TERMINAL_TIMEOUT_S
        terminal = None
        while time.monotonic() < deadline:
            transport.heartbeat()
            responses = transport.drain_buffered_motion_results()
            if len(responses) > 1:
                raise RuntimeError("multiple buffered terminals received")
            if responses:
                terminal = responses[0]
                break
            time.sleep(0.02)
        if terminal is None:
            raise TimeoutError("buffered terminal timeout")
        print(format_buffered_terminal(terminal))
        driver.observe_terminal(terminal)
        if scheduler.state is not BufferedAdapterState.SUCCEEDED:
            snapshot = scheduler.snapshot()
            raise RuntimeError(
                "buffered terminal did not succeed: "
                f"state={snapshot.state.value} reason={snapshot.reason} "
                f"accepted={snapshot.accepted_samples} "
                f"applied={snapshot.applied_samples} "
                f"queued={snapshot.queued_samples} "
                f"safe_stop_required={snapshot.safe_stop_required}"
            )

        print(
            "BUFFERED_FIRMWARE_TERMINAL_PASS "
            f"accepted={terminal.result.accepted_samples} "
            f"applied={terminal.result.applied_samples} "
            f"maximum_apply_lateness_ms={terminal.result.detail}"
        )
        snapshot, metrics = require_observable_settled(
            transport,
            enabled.raw_positions,
            target_raw,
            joint_index=joint_index,
        )
        print(
            "BUFFERED_OBSERVABLE_MOTION_PASS "
            f"planned_delta_raw={metrics.planned_delta_raw} "
            f"observed_delta_raw={metrics.observed_delta_raw} "
            f"target_error_raw={metrics.target_error_raw} "
            "maximum_other_axis_error_raw="
            f"{metrics.maximum_other_axis_error_raw} "
            f"direction_ok={metrics.direction_ok} "
            f"minimum_progress_raw={MINIMUM_DIRECTIONAL_PROGRESS_RAW} "
            f"selected_target_tolerance_raw="
            f"{SELECTED_JOINT_TARGET_TOLERANCE_RAW}"
        )
        print(
            "BUFFERED_SETTLED_POSITIONS="
            + ",".join(str(joint.position_raw) for joint in snapshot.joints)
        )
        execution_succeeded = True
    except BaseException as exc:
        primary_error = exc
        if motion_authority_attempted:
            try:
                transport.safe_stop()
            except BaseException as stop_error:
                print(f"SAFE_STOP_ERROR={stop_error}")
    finally:
        disable_error: BaseException | None = None
        try:
            if binary_mode_entered:
                try:
                    disable_after_attempt(
                        transport,
                        verify_readback=execution_succeeded,
                    )
                except BaseException as exc:
                    disable_error = exc
        finally:
            port.close()

        if disable_error is not None:
            if primary_error is not None:
                raise RuntimeError(
                    "buffered execution failed and physical DISABLE "
                    f"verification also failed: execution={primary_error}; "
                    f"disable={disable_error}"
                ) from disable_error
            raise RuntimeError(
                f"physical DISABLE verification failed: {disable_error}"
            ) from disable_error
        if primary_error is not None:
            raise primary_error

    if not execution_succeeded:
        raise RuntimeError("buffered execution did not reach success")
    print("BUFFERED_SINGLE_JOINT_ONCE_PASS_NO_RETRY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
