#!/usr/bin/env python3
"""Execute one validated bounded pregrasp or grasp segment."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from tools.ros_moveit_execute_once import (  # noqa: E402
    ACTION_NAME,
    ARM_CONTROLLER,
    ARM_JOINTS,
    Preset,
    build_goal,
    wait_future,
)
from tools.ros_moveit_plan_pregrasp_segments import (  # noqa: E402
    DEFAULT_MAX_JOINT_STEP_RAD,
    arm_limits,
    validate_positions,
)


JOINT_STATES_TOPIC = "/joint_states"
START_TOLERANCE_RAD = 0.05
SHOULDER_TOLERANCE_RAD = 0.055
START_TOLERANCES_RAD = (
    START_TOLERANCE_RAD,
    SHOULDER_TOLERANCE_RAD,
    START_TOLERANCE_RAD,
    START_TOLERANCE_RAD,
    START_TOLERANCE_RAD,
)
STATE_TIMEOUT_S = 3.0
EXECUTION_DURATION_S = 2
DIAGNOSTICS_SERVICE = "/get_servo_diagnostics"
POST_SETTLE_WAIT_S = 2.0
POST_SETTLE_TOLERANCE_RAD = 0.05
POST_SETTLE_TOLERANCES_RAD = START_TOLERANCES_RAD
EXPECTED_PROTOCOL_VERSION = 1
EXPECTED_CALIBRATION_HASH = "0xB317C672"
EXPECTED_SERVO_IDENTITIES = tuple(
    enumerate((*ARM_JOINTS, "left_gripper_joint"), start=1)
)
MAX_SHOULDER_TEMPERATURE_C = 50


@dataclass(frozen=True, slots=True)
class Segment:
    index: int
    target_name: str
    expected_start: tuple[float, ...]
    target: tuple[float, ...]
    max_joint_step_rad: float


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _finite_joint_vector(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != len(ARM_JOINTS):
        raise ValueError(f"{label} must contain exactly 5 values")
    vector = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in vector):
        raise ValueError(f"{label} must contain only finite values")
    return vector


def load_segment(path: Path, index: int, expected_sha256: str) -> Segment:
    digest = sha256_file(path)
    if digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"plan sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={digest}"
        )

    document = json.loads(path.read_text(encoding="utf-8"))
    status_targets = {
        "PREGRASP_SEGMENT_PLAN_ONLY_PASS": "pregrasp",
        "GRASP_SEGMENT_PLAN_ONLY_PASS": "grasp",
    }
    status = document.get("status")
    if status not in status_targets:
        raise ValueError("source is not a supported segment PLAN_ONLY_PASS")
    target_name = document.get("target_name", status_targets[status])
    if target_name != status_targets[status]:
        raise ValueError("source target_name and status are inconsistent")
    if document.get("execution_api_used") is not False:
        raise ValueError("source must prove execution_api_used=false")
    if document.get("motion_authorized") is not False:
        raise ValueError("source must remain motion_authorized=false")
    if document.get("robot_target_available") is not False:
        raise ValueError("source must remain robot_target_available=false")
    if tuple(document.get("joint_names", ())) != ARM_JOINTS:
        raise ValueError("source joint order does not match the left arm")

    interpolation_maximum = float(
        document.get("interpolation_joint_step_rad")
    )
    maximum = float(document.get("max_joint_step_rad"))
    if (
        not math.isfinite(interpolation_maximum)
        or not math.isfinite(maximum)
        or interpolation_maximum <= 0.0
        or interpolation_maximum > maximum
        or maximum > DEFAULT_MAX_JOINT_STEP_RAD
    ):
        raise ValueError(
            "source step contract must satisfy "
            "0 < interpolation <= execution <= 0.30 rad"
        )

    segments = document.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("source has no segments")
    matches = [item for item in segments if item.get("index") == index]
    if len(matches) != 1:
        raise ValueError("requested segment index is not unique")
    if any(item.get("success") is not True for item in segments):
        raise ValueError("source contains an unsuccessful segment")

    selected = matches[0]
    expected_start = _finite_joint_vector(
        selected.get("expected_start_positions_rad"),
        "expected start",
    )
    target = _finite_joint_vector(
        selected.get("target_positions_rad"),
        "target",
    )
    actual_step = max(
        abs(goal - current)
        for current, goal in zip(expected_start, target, strict=True)
    )
    recorded_step = float(selected.get("maximum_joint_delta_rad"))
    if not math.isclose(actual_step, recorded_step, abs_tol=1e-9):
        raise ValueError("segment maximum_joint_delta_rad is inconsistent")
    if actual_step > interpolation_maximum + 1e-9:
        raise ValueError("segment exceeds the source interpolation step")

    return Segment(index, target_name, expected_start, target, maximum)


def positions_from_joint_state(message: JointState) -> tuple[float, ...]:
    if len(message.name) != len(set(message.name)):
        raise ValueError("joint state contains duplicate names")
    if len(message.name) != len(message.position):
        raise ValueError("joint state name and position counts differ")
    by_name = dict(zip(message.name, message.position, strict=True))
    missing = [name for name in ARM_JOINTS if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing {','.join(missing)}")
    positions = tuple(float(by_name[name]) for name in ARM_JOINTS)
    if not all(math.isfinite(position) for position in positions):
        raise ValueError("joint state contains a non-finite position")
    return positions


def validate_current_start(
    current: tuple[float, ...],
    expected: tuple[float, ...],
    tolerances_rad: tuple[float, ...] | float = START_TOLERANCES_RAD,
) -> float:
    if len(current) != len(ARM_JOINTS) or len(expected) != len(ARM_JOINTS):
        raise ValueError("current and expected start must contain 5 joints")
    if isinstance(tolerances_rad, (int, float)):
        tolerances = (float(tolerances_rad),) * len(ARM_JOINTS)
    else:
        tolerances = tuple(float(value) for value in tolerances_rad)
    if len(tolerances) != len(ARM_JOINTS):
        raise ValueError("joint tolerances must contain 5 values")
    if not all(math.isfinite(value) and value > 0.0 for value in tolerances):
        raise ValueError("joint tolerances must be finite and positive")
    errors = [
        abs(actual - planned)
        for actual, planned in zip(current, expected, strict=True)
    ]
    maximum_error = max(errors)
    violations = [
        (index, error, tolerance)
        for index, (error, tolerance) in enumerate(
            zip(errors, tolerances, strict=True)
        )
        if error > tolerance + 1e-12
    ]
    if violations:
        index, error, tolerance = max(
            violations,
            key=lambda item: item[1] / item[2],
        )
        raise ValueError(
            f"current start mismatch joint={ARM_JOINTS[index]} "
            f"error={error:.6f} tolerance={tolerance:.6f}"
        )
    return maximum_error


def validate_actual_step(
    current: tuple[float, ...],
    target: tuple[float, ...],
    maximum_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> float:
    if len(current) != len(ARM_JOINTS) or len(target) != len(ARM_JOINTS):
        raise ValueError("current and target must contain 5 joints")
    actual_step = max(
        abs(goal - position)
        for position, goal in zip(current, target, strict=True)
    )
    if actual_step > maximum_rad + 1e-9:
        raise ValueError(
            f"actual current-to-target step {actual_step:.6f} exceeds "
            f"{maximum_rad:.6f} rad"
        )
    return actual_step


def validate_post_settle_soft_abort(
    action_status: int,
    error_code: int,
    current: tuple[float, ...],
    target: tuple[float, ...],
    diagnostics_success: bool,
) -> float:
    if action_status != GoalStatus.STATUS_ABORTED:
        raise ValueError("post-settle reconciliation requires Action ABORTED")
    if error_code != MoveItErrorCodes.CONTROL_FAILED:
        raise ValueError("post-settle reconciliation requires CONTROL_FAILED")
    if diagnostics_success is not True:
        raise ValueError("post-settle diagnostics did not pass")
    return validate_current_start(
        current,
        target,
        POST_SETTLE_TOLERANCES_RAD,
    )


def parse_diagnostics_gate(message: str) -> int:
    try:
        document = json.loads(message)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError("servo diagnostics message is not valid JSON") from error
    if not isinstance(document, dict):
        raise ValueError("servo diagnostics document must be an object")
    if document.get("protocol_version") != EXPECTED_PROTOCOL_VERSION:
        raise ValueError("servo diagnostics protocol identity mismatch")
    if document.get("joint_count") != len(EXPECTED_SERVO_IDENTITIES):
        raise ValueError("servo diagnostics joint count mismatch")
    if document.get("calibration_hash") != EXPECTED_CALIBRATION_HASH:
        raise ValueError("servo diagnostics calibration identity mismatch")

    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != len(
        EXPECTED_SERVO_IDENTITIES
    ):
        raise ValueError("servo diagnostics must contain all 6 axes")
    by_name: dict[str, dict[str, Any]] = {}
    for joint in joints:
        if not isinstance(joint, dict) or not isinstance(joint.get("name"), str):
            raise ValueError("servo diagnostics contains an invalid axis")
        name = joint["name"]
        if name in by_name:
            raise ValueError(f"servo diagnostics duplicate axis: {name}")
        by_name[name] = joint

    shoulder_temperature: int | None = None
    for servo_id, name in EXPECTED_SERVO_IDENTITIES:
        joint = by_name.get(name)
        if joint is None or joint.get("servo_id") != servo_id:
            raise ValueError(
                f"servo diagnostics identity mismatch: {name}/ID{servo_id}"
            )
        if joint.get("torque_enabled") is not True:
            raise ValueError(f"servo diagnostics torque disabled: {name}")
        temperature = joint.get("temperature_c")
        if (
            isinstance(temperature, bool)
            or not isinstance(temperature, (int, float))
            or not math.isfinite(float(temperature))
        ):
            raise ValueError(f"servo diagnostics invalid temperature: {name}")
        if name == "left_shoulder_joint":
            shoulder_temperature = int(temperature)

    if set(by_name) != {name for _, name in EXPECTED_SERVO_IDENTITIES}:
        raise ValueError("servo diagnostics contains an unexpected axis")
    if shoulder_temperature is None:
        raise ValueError("servo diagnostics is missing Shoulder temperature")
    if shoulder_temperature >= MAX_SHOULDER_TEMPERATURE_C:
        raise ValueError(
            "Shoulder temperature gate failed: "
            f"temperature={shoulder_temperature}C "
            f"limit=<{MAX_SHOULDER_TEMPERATURE_C}C"
        )
    return shoulder_temperature


def wait_for_joint_state(node: Any, timeout_s: float) -> JointState:
    messages: list[JointState] = []
    subscription = node.create_subscription(
        JointState,
        JOINT_STATES_TOPIC,
        lambda message: messages.append(message),
        10,
    )
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not messages:
            raise TimeoutError(f"no fresh joint state on {JOINT_STATES_TOPIC}")
        return messages[-1]
    finally:
        node.destroy_subscription(subscription)


def require_healthy_diagnostics(node: Any) -> int:
    client = node.create_client(Trigger, DIAGNOSTICS_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=STATE_TIMEOUT_S):
            raise TimeoutError(f"service unavailable: {DIAGNOSTICS_SERVICE}")
        response = wait_future(
            node,
            client.call_async(Trigger.Request()),
            timeout_s=STATE_TIMEOUT_S,
        )
        if response.success is not True:
            raise RuntimeError(
                f"servo diagnostics rejected: {response.message}"
            )
        return parse_diagnostics_gate(response.message)
    finally:
        node.destroy_client(client)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Execute one bounded pregrasp or grasp segment only when the plan "
            "digest and fresh state pass all fail-closed gates. Never retries."
        )
    )
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--segment", required=True, type=int)
    parser.add_argument(
        "--execute-once",
        action="store_true",
        help="required acknowledgement that exactly one physical goal will be sent",
    )
    args = parser.parse_args()
    if args.segment < 1:
        parser.error("--segment must be at least 1")
    if len(args.plan_sha256) != 64:
        parser.error("--plan-sha256 must contain exactly 64 hex characters")
    try:
        int(args.plan_sha256, 16)
    except ValueError:
        parser.error("--plan-sha256 must be hexadecimal")
    if not args.execute_once:
        parser.error("--execute-once is required; no goal was sent")
    return args


def main() -> int:
    args = parse_args()
    try:
        segment = load_segment(args.plan, args.segment, args.plan_sha256)
        limits = arm_limits(args.calibration)
        validate_positions("expected start", segment.expected_start, limits)
        validate_positions("target", segment.target, limits)
        preset = Preset(
            ARM_CONTROLLER,
            ARM_JOINTS,
            segment.target,
            EXECUTION_DURATION_S,
        )
    except Exception as error:
        print(f"BOUNDED_SEGMENT_EXECUTE_PRECHECK_FAIL reason={error}")
        return 2

    rclpy.init()
    node = rclpy.create_node("so101_moveit_execute_bounded_segment_once")
    client = ActionClient(node, ExecuteTrajectory, ACTION_NAME)
    try:
        current = positions_from_joint_state(
            wait_for_joint_state(node, STATE_TIMEOUT_S)
        )
        maximum_start_error = validate_current_start(
            current,
            segment.expected_start,
        )
        actual_step = validate_actual_step(
            current,
            segment.target,
            segment.max_joint_step_rad,
        )
        shoulder_temperature = require_healthy_diagnostics(node)
        print(
            "SEGMENT_PRE_EXECUTION_DIAGNOSTICS_PASS "
            f"segment={segment.index} axes=6 "
            f"shoulder_temperature_c={shoulder_temperature} "
            f"shoulder_temperature_limit_c="
            f"{MAX_SHOULDER_TEMPERATURE_C}"
        )
        if not client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError(f"Action server unavailable: {ACTION_NAME}")

        prefix = segment.target_name.upper()
        print(
            f"{prefix}_SEGMENT_EXECUTE_REQUEST "
            f"segment={segment.index} "
            f"max_start_error_rad={maximum_start_error:.6f} "
            f"actual_step_rad={actual_step:.6f} "
            f"positions={','.join(f'{value:.6f}' for value in segment.target)} "
            f"duration_ms={EXECUTION_DURATION_S * 1000}"
        )
        goal_handle = wait_future(
            node,
            client.send_goal_async(build_goal(preset)),
            timeout_s=5.0,
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"{prefix}_SEGMENT_EXECUTE_GOAL_REJECTED")
        print(f"{prefix}_SEGMENT_EXECUTE_GOAL_ACCEPTED")

        wrapped_result = wait_future(
            node,
            goal_handle.get_result_async(),
            timeout_s=EXECUTION_DURATION_S + 8.0,
        )
        error_value = int(wrapped_result.result.error_code.val)
        print(
            f"{prefix}_SEGMENT_EXECUTE_RESULT "
            f"status={wrapped_result.status} error_code={error_value}"
        )
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and error_value == MoveItErrorCodes.SUCCESS
        ):
            shoulder_temperature = require_healthy_diagnostics(node)
            print(
                f"{prefix}_SEGMENT_POST_EXECUTION_DIAGNOSTICS_PASS "
                f"segment={segment.index} axes=6 "
                f"shoulder_temperature_c={shoulder_temperature} "
                f"shoulder_temperature_limit_c="
                f"{MAX_SHOULDER_TEMPERATURE_C}"
            )
        elif (
            wrapped_result.status == GoalStatus.STATUS_ABORTED
            and error_value == MoveItErrorCodes.CONTROL_FAILED
        ):
            # A non-latching deadline miss must never trigger a resend. Read
            # fresh post-settle state and six-axis diagnostics instead.
            time.sleep(POST_SETTLE_WAIT_S)
            settled = positions_from_joint_state(
                wait_for_joint_state(node, STATE_TIMEOUT_S)
            )
            shoulder_temperature = require_healthy_diagnostics(node)
            settled_error = validate_post_settle_soft_abort(
                wrapped_result.status,
                error_value,
                settled,
                segment.target,
                True,
            )
            print(
                f"{prefix}_SEGMENT_SOFT_ABORT_POST_SETTLE_ACCEPTED "
                f"segment={segment.index} "
                f"max_error_rad={settled_error:.6f} retries=0 "
                f"shoulder_temperature_c={shoulder_temperature}"
            )
        elif wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{prefix}_SEGMENT_EXECUTE_ACTION_NOT_SUCCEEDED")
        else:
            raise RuntimeError(
                f"{prefix}_SEGMENT_EXECUTE_ERROR_CODE_{error_value}"
            )
        print(
            f"{prefix}_SEGMENT_EXECUTE_PASS segment={segment.index} retries=0"
        )
        return 0
    except Exception as error:
        print(f"BOUNDED_SEGMENT_EXECUTE_FAIL reason={error}")
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
