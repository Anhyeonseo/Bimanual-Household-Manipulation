#!/usr/bin/env python3
"""Send one SHA-pinned buffered FollowJointTrajectory goal without retry."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import time
from typing import Any, Callable

from single_arm_bridge.action_validation import (
    TrajectoryPointData,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_action_adapter import (
    prepare_buffered_execution_plan,
)
from single_arm_bridge.calibration import ArmCalibration, load_calibration


PLAN_STATUS = "BUFFERED_ACTION_COMMISSIONING_PLAN_ONLY_PASS"
EXPECTED_FIRMWARE = "0x00022100"
EXPECTED_CAPABILITIES = "0x00000FFF"
EXPECTED_CALIBRATION = "0x8AD27897"
ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
CONFIRMATION = "EXECUTE_MOTION9_BUFFERED_ACTION_ONCE"
STATE_TIMEOUT_S = 5.0
ACTION_SERVER_TIMEOUT_S = 10.0
ACTION_RESULT_TIMEOUT_S = 10.0
START_TOLERANCES_RAD = (0.050, 0.055, 0.050, 0.050, 0.050)
POST_RETURN_TOLERANCES_RAD = START_TOLERANCES_RAD
ACTION_STATUS_SUCCEEDED = 4
FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL = 0
# buffered_action_execution.ExecutionOutcome 은 성공 terminal 뒤에
# "; {startup}; {lateness}" 로 진단을 덧붙인다. 두 필수 수치 뒤의 나머지는
# 통째로 증거로 보존한다. 형식 일치는
# tests/test_buffered_terminal_format_contract.py 가 양쪽 소스를 파싱해 강제한다.
TERMINAL_PATTERN = re.compile(
    r"^buffered trajectory completed; "
    r"maximum_apply_lateness_ms=(\d+) "
    r"post_settle_max_error_raw=(\d+)"
    r"(?:; (?P<diagnostics>.*))?$"
)


@dataclass(frozen=True, slots=True)
class PlanWaypoint:
    time_from_start_ms: int
    positions_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class CommissioningPlan:
    path: Path
    sha256: str
    arm_joint_names: tuple[str, ...]
    anchor_positions_rad: tuple[float, ...]
    waypoints: tuple[PlanWaypoint, ...]
    duration_ms: int
    sample_count: int


@dataclass(frozen=True, slots=True)
class TerminalEvidence:
    action_status: int
    error_code: int
    maximum_apply_lateness_ms: int
    post_settle_max_error_raw: int
    error_string: str
    terminal_diagnostics: str | None = None


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _finite_vector(
    value: Any,
    count: int,
    label: str,
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must contain exactly {count} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def _raw_from_radians(
    calibration: ArmCalibration,
    positions_rad: tuple[float, ...],
) -> tuple[int, ...]:
    return tuple(
        round(
            joint.zero_raw
            + joint.direction * position * 4096.0 / (2.0 * math.pi)
        )
        for joint, position in zip(
            calibration.joints,
            positions_rad,
            strict=True,
        )
    )


def load_commissioning_plan(
    path: Path,
    expected_sha256: str,
    calibration_path: Path,
    contract_path: Path,
) -> CommissioningPlan:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("status") != PLAN_STATUS:
        raise ValueError("plan status is not the reviewed commissioning status")
    for field in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
        "buffered_frame_encoded",
    ):
        if document.get(field) is not False:
            raise ValueError(f"plan must keep {field}=false")
    if document.get("firmware_version") != EXPECTED_FIRMWARE:
        raise ValueError("plan firmware identity mismatch")
    if document.get("capabilities") != EXPECTED_CAPABILITIES:
        raise ValueError("plan capability identity mismatch")
    if document.get("calibration_hash") != EXPECTED_CALIBRATION:
        raise ValueError("plan calibration identity mismatch")
    if document.get("calibration_sha256") != sha256_file(calibration_path):
        raise ValueError("plan calibration file sha256 mismatch")
    if document.get("contract_sha256") != sha256_file(contract_path):
        raise ValueError("plan buffered contract sha256 mismatch")

    calibration = load_calibration(calibration_path)
    all_names = tuple(calibration.ros_joint_names)
    if tuple(document.get("joint_names", ())) != all_names:
        raise ValueError("plan joint order does not match calibration")
    arm_names = all_names[:5]

    anchor = document.get("anchor")
    if not isinstance(anchor, dict):
        raise ValueError("plan anchor is invalid")
    anchor_raw = anchor.get("raw")
    if (
        not isinstance(anchor_raw, list)
        or len(anchor_raw) != 6
        or any(isinstance(value, bool) or not isinstance(value, int) for value in anchor_raw)
    ):
        raise ValueError("plan anchor raw must contain six integers")
    anchor_positions = _finite_vector(
        anchor.get("positions_rad"),
        6,
        "plan anchor positions",
    )
    recomputed_anchor = tuple(
        calibration.raw_feedback_to_radians(tuple(anchor_raw))
    )
    if any(
        not math.isclose(recorded, recomputed, abs_tol=1.0e-12)
        for recorded, recomputed in zip(
            anchor_positions,
            recomputed_anchor,
            strict=True,
        )
    ):
        raise ValueError("plan anchor radians are inconsistent with raw")

    requested = document.get("requested_deltas_rad")
    expected_deltas = {
        arm_names[0]: 0.015,
        arm_names[1]: 0.015,
        arm_names[2]: 0.0,
        arm_names[3]: 0.0,
        arm_names[4]: 0.030,
    }
    if not isinstance(requested, dict) or set(requested) != set(expected_deltas):
        raise ValueError("plan requested delta joint set is invalid")
    if any(
        not math.isclose(float(requested[name]), value, abs_tol=1.0e-12)
        for name, value in expected_deltas.items()
    ):
        raise ValueError("plan requested deltas do not match reviewed values")

    waypoint_documents = document.get("waypoints")
    if not isinstance(waypoint_documents, list) or len(waypoint_documents) < 2:
        raise ValueError("plan requires at least two waypoints")
    waypoints = tuple(
        PlanWaypoint(
            time_from_start_ms=int(item["time_from_start_ms"]),
            positions_rad=_finite_vector(
                item.get("positions_rad"),
                5,
                "waypoint positions",
            ),
        )
        for item in waypoint_documents
        if isinstance(item, dict)
    )
    if len(waypoints) != len(waypoint_documents):
        raise ValueError("plan contains an invalid waypoint")
    if tuple(point.time_from_start_ms for point in waypoints) != (
        0,
        200,
        400,
        600,
        800,
        1000,
        1200,
    ):
        raise ValueError("plan waypoint timing does not match reviewed route")
    if any(
        not math.isclose(actual, expected, abs_tol=1.0e-12)
        for actual, expected in zip(
            waypoints[0].positions_rad,
            anchor_positions[:5],
            strict=True,
        )
    ):
        raise ValueError("plan first waypoint does not match anchor")
    if any(
        not math.isclose(actual, expected, abs_tol=1.0e-12)
        for actual, expected in zip(
            waypoints[-1].positions_rad,
            anchor_positions[:5],
            strict=True,
        )
    ):
        raise ValueError("plan does not return to its anchor")

    points = tuple(
        TrajectoryPointData(
            positions=waypoint.positions_rad,
            time_from_start_ns=waypoint.time_from_start_ms * 1_000_000,
        )
        for waypoint in waypoints
    )
    limits = {
        name: calibration.ros_radian_limits[name]
        for name in arm_names
    }
    trajectory = validate_buffered_trajectory(
        arm_names,
        points,
        arm_names,
        limits,
        anchor_positions[:5],
        {name: 0.5 for name in arm_names},
        {name: 1.0 for name in arm_names},
        start_tolerance_rad=dict(zip(
            arm_names,
            START_TOLERANCES_RAD,
            strict=True,
        )),
    )
    recomputed = prepare_buffered_execution_plan(
        trajectory,
        calibration,
        preserved_gripper_rad=anchor_positions[5],
        current_tick_ms=100_000,
    )
    resampling = document.get("resampling")
    if not isinstance(resampling, dict):
        raise ValueError("plan resampling evidence is missing")
    if resampling.get("period_ms") != 20:
        raise ValueError("plan sample period must be 20 ms")
    if resampling.get("duration_ms") != trajectory.duration_ms:
        raise ValueError("plan duration evidence mismatch")
    if resampling.get("sample_count") != len(recomputed.samples):
        raise ValueError("plan sample count evidence mismatch")
    recorded_samples = resampling.get("samples")
    if not isinstance(recorded_samples, list) or len(recorded_samples) != len(
        recomputed.samples
    ):
        raise ValueError("plan resampled samples are incomplete")
    for recorded, sample in zip(recorded_samples, recomputed.samples, strict=True):
        if recorded != {
            "index": sample.sample_index,
            "elapsed_ms": sample.trajectory_elapsed_ms,
            "apply_offset_ms": sample.apply_tick_ms - 100_000,
            "positions_urad": list(sample.positions_urad),
        }:
            raise ValueError("plan resampled sample evidence mismatch")

    apex = document.get("apex")
    if not isinstance(apex, dict):
        raise ValueError("plan apex evidence is missing")
    apex_positions = _finite_vector(
        apex.get("positions_rad"),
        6,
        "plan apex positions",
    )
    if list(_raw_from_radians(calibration, apex_positions)) != apex.get("raw"):
        raise ValueError("plan apex raw evidence mismatch")
    if apex.get("delta_raw") != [10, 10, 0, 0, 20, 0]:
        raise ValueError("plan apex raw delta is not the reviewed route")

    queue = document.get("queue_contract")
    if not isinstance(queue, dict):
        raise ValueError("plan queue evidence is missing")
    if queue.get("admission_batch_sizes") != [9, 7, 6, 6, 6, 6, 6, 6, 6, 3]:
        raise ValueError("plan queue admission shape mismatch")
    terminal = queue.get("simulation_terminal")
    if (
        not isinstance(terminal, dict)
        or terminal.get("state") != "input_complete"
        or terminal.get("success_without_firmware_terminal") is not False
        or terminal.get("safe_stop_required") is not False
    ):
        raise ValueError("plan terminal simulation evidence is invalid")
    round_trip = document.get("round_trip")
    if (
        not isinstance(round_trip, dict)
        or float(round_trip.get("maximum_return_error_rad", math.inf)) != 0.0
    ):
        raise ValueError("plan round-trip error must be exactly zero")

    return CommissioningPlan(
        path=path,
        sha256=digest,
        arm_joint_names=arm_names,
        anchor_positions_rad=anchor_positions[:5],
        waypoints=waypoints,
        duration_ms=trajectory.duration_ms,
        sample_count=len(recomputed.samples),
    )


def positions_from_joint_state(
    message: Any,
    expected_names: tuple[str, ...],
) -> tuple[float, ...]:
    if len(message.name) != len(set(message.name)):
        raise ValueError("joint state contains duplicate names")
    if len(message.name) != len(message.position):
        raise ValueError("joint state name and position counts differ")
    by_name = dict(zip(message.name, message.position, strict=True))
    missing = [name for name in expected_names if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing {','.join(missing)}")
    positions = tuple(float(by_name[name]) for name in expected_names)
    if not all(math.isfinite(position) for position in positions):
        raise ValueError("joint state contains a non-finite position")
    return positions


def validate_fresh_start(
    current: tuple[float, ...],
    anchor: tuple[float, ...],
    tolerances: tuple[float, ...] = START_TOLERANCES_RAD,
) -> float:
    if len(current) != 5 or len(anchor) != 5 or len(tolerances) != 5:
        raise ValueError("fresh start vectors must contain five joints")
    errors = tuple(
        abs(actual - expected)
        for actual, expected in zip(current, anchor, strict=True)
    )
    violations = [
        (index, error, tolerance)
        for index, (error, tolerance) in enumerate(
            zip(errors, tolerances, strict=True)
        )
        if error > tolerance + 1.0e-12
    ]
    if violations:
        index, error, tolerance = max(
            violations,
            key=lambda item: item[1] / item[2],
        )
        raise ValueError(
            f"fresh start mismatch joint_index={index} "
            f"error={error:.6f} tolerance={tolerance:.6f}"
        )
    return max(errors)


def build_goal_spec(plan: CommissioningPlan) -> dict[str, object]:
    return {
        "joint_names": list(plan.arm_joint_names),
        "points": [
            {
                "positions": list(waypoint.positions_rad),
                "time_from_start_ms": waypoint.time_from_start_ms,
            }
            for waypoint in plan.waypoints
        ],
    }


def build_goal(plan: CommissioningPlan) -> Any:
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    goal = FollowJointTrajectory.Goal()
    specification = build_goal_spec(plan)
    goal.trajectory.joint_names = specification["joint_names"]
    for item in specification["points"]:
        point = JointTrajectoryPoint()
        point.positions = item["positions"]
        time_from_start_ms = item["time_from_start_ms"]
        point.time_from_start.sec = time_from_start_ms // 1000
        point.time_from_start.nanosec = (
            time_from_start_ms % 1000
        ) * 1_000_000
        goal.trajectory.points.append(point)
    return goal


def validate_action_terminal(
    action_status: int,
    result: Any,
) -> TerminalEvidence:
    if action_status != ACTION_STATUS_SUCCEEDED:
        raise RuntimeError(
            f"buffered Action did not succeed status={action_status}"
        )
    if result.error_code != FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL:
        raise RuntimeError(
            f"buffered Action error_code={result.error_code} "
            f"reason={result.error_string}"
        )
    match = TERMINAL_PATTERN.fullmatch(result.error_string)
    if match is None:
        raise RuntimeError("buffered Action terminal evidence is missing")
    maximum_lateness = int(match.group(1))
    settle_error = int(match.group(2))
    if not 0 <= maximum_lateness <= 5:
        raise RuntimeError("maximum apply lateness is outside 0..5 ms")
    if not 0 <= settle_error <= 30:
        raise RuntimeError("post-settle error is outside 0..30 raw")
    return TerminalEvidence(
        action_status=action_status,
        error_code=result.error_code,
        maximum_apply_lateness_ms=maximum_lateness,
        post_settle_max_error_raw=settle_error,
        error_string=result.error_string,
        terminal_diagnostics=match.group("diagnostics"),
    )


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("ROS future timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not future.done():
        raise TimeoutError("ROS future did not complete")
    result = future.result()
    if result is None:
        raise RuntimeError("ROS future completed without a result")
    return result


def wait_joint_state(
    node: Any,
    expected_names: tuple[str, ...],
    timeout_s: float = STATE_TIMEOUT_S,
) -> tuple[float, ...]:
    import rclpy
    from sensor_msgs.msg import JointState

    messages: list[JointState] = []
    subscription = node.create_subscription(
        JointState,
        JOINT_STATES_TOPIC,
        messages.append,
        10,
    )
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and not messages:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("joint state timed out")
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        return positions_from_joint_state(messages[-1], expected_names)
    finally:
        node.destroy_subscription(subscription)


def send_goal_once(
    node: Any,
    action_client: Any,
    goal: Any,
    *,
    result_timeout_s: float = ACTION_RESULT_TIMEOUT_S,
    wait: Callable[[Node, Any, float], Any] = wait_future,
) -> tuple[int, Any]:
    send_future = action_client.send_goal_async(goal)
    goal_handle = wait(node, send_future, ACTION_SERVER_TIMEOUT_S)
    if not goal_handle.accepted:
        raise RuntimeError("buffered Action goal was rejected")
    try:
        response = wait(
            node,
            goal_handle.get_result_async(),
            result_timeout_s,
        )
    except TimeoutError:
        try:
            wait(node, goal_handle.cancel_goal_async(), 3.0)
        except Exception:
            pass
        raise
    return response.status, response.result


def main() -> int:
    from control_msgs.action import FollowJointTrajectory
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    root = Path(__file__).resolve().parents[1]
    package = root / "ros2_ws" / "src" / "single_arm_bridge"
    parser = argparse.ArgumentParser()
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
        parser.error("exact one-shot execution confirmation is required")

    plan = load_commissioning_plan(
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
    node = Node("motion9_buffered_action_commissioning_once")
    action_client = ActionClient(
        node,
        FollowJointTrajectory,
        ACTION_NAME,
    )
    try:
        current = wait_joint_state(node, plan.arm_joint_names)
        start_error = validate_fresh_start(
            current,
            plan.anchor_positions_rad,
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
        )
        evidence = validate_action_terminal(status, result)
        final_positions = wait_joint_state(node, plan.arm_joint_names)
        return_error = validate_fresh_start(
            final_positions,
            plan.anchor_positions_rad,
            POST_RETURN_TOLERANCES_RAD,
        )
        print(
            "ACTION_TERMINAL_PASS "
            f"status={evidence.action_status} "
            f"error_code={evidence.error_code} "
            f"maximum_apply_lateness_ms="
            f"{evidence.maximum_apply_lateness_ms} "
            f"post_settle_max_error_raw="
            f"{evidence.post_settle_max_error_raw}"
        )
        print(f"ROUND_TRIP_MAX_ERROR_RAD={return_error:.6f}")
        print("AUTOMATIC_RETRY_COUNT=0")
        print("MOTION9_BUFFERED_ACTION_ONCE_PASS")
        return 0
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
