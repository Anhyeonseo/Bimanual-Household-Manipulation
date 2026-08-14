#!/usr/bin/env python3
"""Execute exactly one hash-pinned Stage 7 pick/place phase.

The tool is deliberately phase-bounded.  It validates the complete plan-only
manifest, checks a fresh robot state and servo diagnostics, executes only the
requested phase, and stops immediately after the first rejected or failed
command.  It never retries physical motion.
"""

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

from tools.joint_calibration import (  # noqa: E402
    calibration_hash,
    load_calibration,
    urad_to_raw,
)
from tools.ros_moveit_execute_once import (  # noqa: E402
    ACTION_NAME,
    ARM_CONTROLLER,
    ARM_JOINTS,
    GRIPPER_CONTROLLER,
    GRIPPER_JOINT,
    Preset,
    build_goal,
    wait_future,
)
from tools.ros_moveit_plan_pregrasp_segments import (  # noqa: E402
    arm_limits,
    validate_positions,
)


MANIFEST_STATUS = "FULL_PICK_PLACE_PLAN_ONLY_PASS"
DIAGNOSTICS_SERVICE = "/get_servo_diagnostics"
JOINT_STATES_TOPIC = "/joint_states"
START_TOLERANCE_RAD = 0.05
FINAL_TOLERANCE_RAD = 0.05
STATE_TIMEOUT_S = 3.0
SERVICE_TIMEOUT_S = 5.0
MINIMUM_VOLTAGE_V = 11.5
MAXIMUM_TEMPERATURE_C = 55
MINIMUM_CONTACT_LOAD_RAW = 20
MINIMUM_CONTACT_POSITION_ERROR_RAW = 10

PHASE_ORDER = (
    "q0_to_pick_pregrasp",
    "pick_pregrasp_to_grasp",
    "pick_close",
    "pick_grasp_to_lift20",
    "lift_to_place_pregrasp",
    "place_pregrasp_to_place",
    "place_release",
    "place_to_retreat",
    "place_pregrasp_to_q0",
)
CONTACT_REQUIRED_BEFORE = frozenset(
    {
        "pick_grasp_to_lift20",
        "lift_to_place_pregrasp",
        "place_pregrasp_to_place",
        "place_release",
    }
)
CONTACT_REQUIRED_AFTER = frozenset(
    {
        "pick_close",
        "pick_grasp_to_lift20",
        "lift_to_place_pregrasp",
        "place_pregrasp_to_place",
    }
)
OPEN_REQUIRED_AFTER = frozenset(
    {
        "place_release",
        "place_to_retreat",
        "place_pregrasp_to_q0",
    }
)
OPEN_GRIPPER_PHASES = frozenset(
    {
        "q0_to_pick_pregrasp",
        "pick_pregrasp_to_grasp",
        "pick_close",
        "place_to_retreat",
        "place_pregrasp_to_q0",
    }
)
CLOSED_GRIPPER_PHASES = frozenset(
    {
        "pick_grasp_to_lift20",
        "lift_to_place_pregrasp",
        "place_pregrasp_to_place",
        "place_release",
    }
)

EXPECTED_P_GAINS = (16, 64, 56, 16, 16, 16)
EXPECTED_TORQUE_LIMITS = (400, 900, 800, 400, 250, 150)


@dataclass(frozen=True, slots=True)
class PhasePlan:
    name: str
    steps: tuple[dict[str, Any], ...]
    expected_arm_start: tuple[float, ...]
    expected_arm_end: tuple[float, ...]
    maximum_joint_step_rad: float
    calibration_hash: str


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _vector(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != len(ARM_JOINTS):
        raise ValueError(f"{label} must contain exactly 5 values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def _resolve_repository_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a repository-relative path")
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise ValueError(f"{label} must remain inside the repository")
    resolved = (REPO_ROOT / candidate).resolve()
    if REPO_ROOT.resolve() not in resolved.parents:
        raise ValueError(f"{label} resolves outside the repository")
    return resolved


def _validate_manifest_sources(document: dict[str, Any]) -> None:
    summaries = document.get("phase_summaries")
    if not isinstance(summaries, list) or len(summaries) != 7:
        raise ValueError("manifest must contain seven arm phase summaries")
    expected_arm_phases = tuple(
        phase
        for phase in PHASE_ORDER
        if phase not in {"pick_close", "place_release"}
    )
    if tuple(item.get("name") for item in summaries) != expected_arm_phases:
        raise ValueError("manifest arm phase order is invalid")
    for summary in summaries:
        source = _resolve_repository_path(summary.get("source"), "phase source")
        expected_digest = summary.get("source_sha256")
        if not isinstance(expected_digest, str) or len(expected_digest) != 64:
            raise ValueError("phase source_sha256 is invalid")
        actual_digest = sha256_file(source)
        if actual_digest.lower() != expected_digest.lower():
            raise ValueError(
                f"phase source sha256 mismatch source={source} "
                f"expected={expected_digest.lower()} actual={actual_digest}"
            )


def load_phase(
    manifest_path: Path,
    expected_sha256: str,
    phase_name: str,
    calibration_path: Path,
) -> PhasePlan:
    actual_digest = sha256_file(manifest_path)
    if actual_digest.lower() != expected_sha256.lower():
        raise ValueError(
            f"manifest sha256 mismatch expected={expected_sha256.lower()} "
            f"actual={actual_digest}"
        )
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1:
        raise ValueError("unsupported manifest schema")
    if document.get("status") != MANIFEST_STATUS:
        raise ValueError("manifest is not FULL_PICK_PLACE_PLAN_ONLY_PASS")
    for field in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
        "automatic_execution_permitted",
    ):
        if document.get(field) is not False:
            raise ValueError(f"manifest must prove {field}=false")
    if tuple(document.get("joint_names", ())) != ARM_JOINTS:
        raise ValueError("manifest joint order does not match the left arm")
    if phase_name not in PHASE_ORDER:
        raise ValueError(f"unsupported phase {phase_name}")

    maximum_step = float(document.get("maximum_joint_step_rad"))
    if not math.isfinite(maximum_step) or not 0.0 < maximum_step <= 0.18:
        raise ValueError("manifest maximum joint step is outside (0, 0.18]")

    calibration = load_calibration(calibration_path)
    actual_calibration_hash = f"0x{calibration_hash(calibration):08X}"
    manifest_calibration_hash = document.get("calibration_hash")
    if manifest_calibration_hash != actual_calibration_hash:
        raise ValueError(
            "manifest and local calibration hashes differ "
            f"manifest={manifest_calibration_hash} "
            f"local={actual_calibration_hash}"
        )
    _validate_manifest_sources(document)

    steps = document.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("manifest contains no command steps")
    if [step.get("index") for step in steps] != list(range(1, len(steps) + 1)):
        raise ValueError("manifest command indices are not contiguous")
    if int(document.get("command_step_count", -1)) != len(steps):
        raise ValueError("manifest command_step_count is inconsistent")
    if [step.get("phase") for step in steps if step.get("manual_gate_required")] != list(
        PHASE_ORDER
    ):
        raise ValueError("manifest manual phase gates are incomplete or out of order")

    selected: list[dict[str, Any]] = []
    last_arm = (0.0,) * len(ARM_JOINTS)
    selected_start: tuple[float, ...] | None = None
    selected_end: tuple[float, ...] | None = None
    arm_count = 0
    for step in steps:
        kind = step.get("kind")
        if kind == "arm":
            start = _vector(step.get("start_positions_rad"), "arm start")
            target = _vector(step.get("target_positions_rad"), "arm target")
            if any(
                not math.isclose(a, b, abs_tol=1e-9)
                for a, b in zip(start, last_arm, strict=True)
            ):
                raise ValueError("manifest arm chain is discontinuous")
            actual_step = max(
                abs(target_value - start_value)
                for start_value, target_value in zip(start, target, strict=True)
            )
            recorded_step = float(step.get("maximum_joint_delta_rad"))
            if not math.isclose(actual_step, recorded_step, abs_tol=1e-9):
                raise ValueError("manifest arm step delta is inconsistent")
            if actual_step > maximum_step + 1e-9:
                raise ValueError("manifest arm step exceeds maximum")
            last_arm = target
            arm_count += 1
        elif kind == "gripper":
            target = float(step.get("target_position_rad"))
            expected = 0.13 if step.get("phase") == "pick_close" else 0.06
            if not math.isclose(target, expected, abs_tol=1e-9):
                raise ValueError("manifest gripper target is invalid")
        else:
            raise ValueError("manifest contains an unsupported command kind")

        if step.get("phase") == phase_name:
            if selected_start is None:
                selected_start = last_arm if kind == "gripper" else start
            selected_end = last_arm
            selected.append(step)

    if arm_count != int(document.get("arm_segment_count", -1)):
        raise ValueError("manifest arm_segment_count is inconsistent")
    if last_arm != (0.0,) * len(ARM_JOINTS):
        raise ValueError("manifest does not return the arm to q0")
    if not selected or selected_start is None or selected_end is None:
        raise ValueError(f"manifest contains no steps for phase {phase_name}")

    return PhasePlan(
        name=phase_name,
        steps=tuple(selected),
        expected_arm_start=selected_start,
        expected_arm_end=selected_end,
        maximum_joint_step_rad=maximum_step,
        calibration_hash=actual_calibration_hash,
    )


def resume_arm_phase(
    phase: PhasePlan,
    completed_arm_steps: int,
) -> PhasePlan:
    if completed_arm_steps == 0:
        return phase
    if completed_arm_steps < 0:
        raise ValueError("completed arm steps cannot be negative")
    if any(step.get("kind") != "arm" for step in phase.steps):
        raise ValueError("checkpoint resume is supported only for arm-only phases")
    if completed_arm_steps >= len(phase.steps):
        raise ValueError("completed arm steps must leave at least one command")
    checkpoint = _vector(
        phase.steps[completed_arm_steps - 1].get("target_positions_rad"),
        "resume checkpoint",
    )
    return PhasePlan(
        name=phase.name,
        steps=phase.steps[completed_arm_steps:],
        expected_arm_start=checkpoint,
        expected_arm_end=phase.expected_arm_end,
        maximum_joint_step_rad=phase.maximum_joint_step_rad,
        calibration_hash=phase.calibration_hash,
    )


def positions_from_joint_state(
    message: JointState,
) -> tuple[tuple[float, ...], float]:
    if len(message.name) != len(set(message.name)):
        raise ValueError("joint state contains duplicate names")
    if len(message.name) != len(message.position):
        raise ValueError("joint state name and position counts differ")
    by_name = dict(zip(message.name, message.position, strict=True))
    required = (*ARM_JOINTS, GRIPPER_JOINT)
    missing = [name for name in required if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing {','.join(missing)}")
    arm = tuple(float(by_name[name]) for name in ARM_JOINTS)
    gripper = float(by_name[GRIPPER_JOINT])
    if not all(math.isfinite(value) for value in (*arm, gripper)):
        raise ValueError("joint state contains a non-finite position")
    return arm, gripper


def validate_pose(
    current: tuple[float, ...],
    expected: tuple[float, ...],
    tolerance_rad: float,
    label: str,
) -> float:
    if len(current) != len(ARM_JOINTS) or len(expected) != len(ARM_JOINTS):
        raise ValueError("arm poses must contain exactly five joints")
    errors = [
        abs(actual - target)
        for actual, target in zip(current, expected, strict=True)
    ]
    maximum = max(errors)
    if maximum > tolerance_rad:
        index = errors.index(maximum)
        raise ValueError(
            f"{label} mismatch joint={ARM_JOINTS[index]} "
            f"error={maximum:.6f} tolerance={tolerance_rad:.6f}"
        )
    return maximum


def validate_actual_step(
    current: tuple[float, ...],
    target: tuple[float, ...],
    maximum_rad: float,
) -> float:
    maximum = max(
        abs(goal - actual)
        for actual, goal in zip(current, target, strict=True)
    )
    if maximum > maximum_rad + 1e-9:
        raise ValueError(
            f"actual current-to-target step {maximum:.6f} exceeds "
            f"{maximum_rad:.6f} rad"
        )
    return maximum


def validate_gripper_position(
    current: float,
    expected: float,
    tolerance_rad: float = START_TOLERANCE_RAD,
) -> float:
    error = abs(current - expected)
    if error > tolerance_rad:
        raise ValueError(
            f"gripper phase-state mismatch error={error:.6f} "
            f"tolerance={tolerance_rad:.6f}"
        )
    return error


def parse_diagnostics_message(
    message: str,
    expected_calibration_hash: str,
    require_contact: bool,
    require_open: bool,
    gripper_closed_raw: int,
    gripper_open_raw: int,
) -> dict[str, Any]:
    document = json.loads(message)
    if document.get("protocol_version") != 1:
        raise ValueError("diagnostics protocol_version is not 1")
    if document.get("joint_count") != 6:
        raise ValueError("diagnostics joint_count is not 6")
    if document.get("calibration_hash") != expected_calibration_hash:
        raise ValueError("diagnostics calibration hash mismatch")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("diagnostics must contain six joints")

    names = (*ARM_JOINTS, GRIPPER_JOINT)
    for index, (sample, name, p_gain, torque_limit) in enumerate(
        zip(
            joints,
            names,
            EXPECTED_P_GAINS,
            EXPECTED_TORQUE_LIMITS,
            strict=True,
        )
    ):
        if sample.get("name") != name or sample.get("servo_id") != index + 1:
            raise ValueError(f"diagnostics joint identity mismatch at index {index}")
        if sample.get("torque_enabled") is not True:
            raise ValueError(f"{name} torque is not enabled")
        if int(sample.get("p_gain", -1)) != p_gain:
            raise ValueError(f"{name} P gain mismatch")
        if int(sample.get("torque_limit_raw", -1)) != torque_limit:
            raise ValueError(f"{name} torque limit mismatch")
        if float(sample.get("voltage_v", 0.0)) < MINIMUM_VOLTAGE_V:
            raise ValueError(f"{name} voltage is below {MINIMUM_VOLTAGE_V:.1f} V")
        if int(sample.get("temperature_c", 999)) > MAXIMUM_TEMPERATURE_C:
            raise ValueError(
                f"{name} temperature exceeds {MAXIMUM_TEMPERATURE_C} C"
            )

    gripper = joints[-1]
    if require_contact:
        if int(gripper.get("goal_position_raw", -1)) != gripper_closed_raw:
            raise ValueError("gripper closed goal readback mismatch")
        load = int(gripper.get("load_magnitude_raw", 0))
        position_error = abs(
            int(gripper.get("position_raw", 0))
            - int(gripper.get("goal_position_raw", 0))
        )
        if (
            load < MINIMUM_CONTACT_LOAD_RAW
            and position_error < MINIMUM_CONTACT_POSITION_ERROR_RAW
        ):
            raise ValueError("gripper has no retained-contact evidence")
    if require_open:
        if int(gripper.get("goal_position_raw", -1)) != gripper_open_raw:
            raise ValueError("gripper open goal readback mismatch")
    return document


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


def get_diagnostics(
    node: Any,
    client: Any,
    phase: PhasePlan,
    calibration: dict[str, Any],
    boundary: str,
) -> dict[str, Any]:
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
        raise TimeoutError(f"service unavailable: {DIAGNOSTICS_SERVICE}")
    response = wait_future(
        node,
        client.call_async(Trigger.Request()),
        timeout_s=SERVICE_TIMEOUT_S,
    )
    if response.success is not True:
        raise RuntimeError(f"servo diagnostics rejected: {response.message}")
    require_contact = (
        phase.name in CONTACT_REQUIRED_BEFORE
        if boundary == "before"
        else phase.name in CONTACT_REQUIRED_AFTER
    )
    require_open = boundary == "after" and phase.name in OPEN_REQUIRED_AFTER
    closed_raw = urad_to_raw(calibration, 5, 130_000)
    open_raw = urad_to_raw(calibration, 5, 60_000)
    return parse_diagnostics_message(
        response.message,
        phase.calibration_hash,
        require_contact,
        require_open,
        closed_raw,
        open_raw,
    )


def execute_goal_once(
    node: Any,
    client: Any,
    preset: Preset,
    phase_name: str,
    step_index: int,
) -> None:
    goal_handle = wait_future(
        node,
        client.send_goal_async(build_goal(preset)),
        timeout_s=5.0,
    )
    if not goal_handle.accepted:
        raise RuntimeError(
            f"phase={phase_name} step={step_index} goal rejected"
        )
    wrapped_result = wait_future(
        node,
        goal_handle.get_result_async(),
        timeout_s=float(preset.duration_s) + 8.0,
    )
    error_value = int(wrapped_result.result.error_code.val)
    print(
        "PICK_PLACE_PHASE_STEP_RESULT "
        f"phase={phase_name} step={step_index} "
        f"status={wrapped_result.status} error_code={error_value}"
    )
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(
            f"phase={phase_name} step={step_index} Action did not succeed"
        )
    if error_value != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"phase={phase_name} step={step_index} "
            f"MoveIt error={error_value}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Execute exactly one hash-pinned full-pick-place phase. "
            "Every invocation requires a fresh explicit acknowledgement and "
            "never retries."
        )
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--phase", required=True, choices=PHASE_ORDER)
    parser.add_argument(
        "--resume-after-arm-steps",
        type=int,
        default=0,
        help=(
            "resume an arm-only phase after this many completed phase-local "
            "steps; current joints must match the hash-pinned checkpoint"
        ),
    )
    parser.add_argument(
        "--execute-phase-once",
        action="store_true",
        help="required acknowledgement for exactly one selected phase",
    )
    args = parser.parse_args()
    if len(args.manifest_sha256) != 64:
        parser.error("--manifest-sha256 must contain exactly 64 hex characters")
    try:
        int(args.manifest_sha256, 16)
    except ValueError:
        parser.error("--manifest-sha256 must be hexadecimal")
    if not args.execute_phase_once:
        parser.error("--execute-phase-once is required; no goal was sent")
    if args.resume_after_arm_steps < 0:
        parser.error("--resume-after-arm-steps cannot be negative")
    return args


def main() -> int:
    args = parse_args()
    try:
        phase = load_phase(
            args.manifest,
            args.manifest_sha256,
            args.phase,
            args.calibration,
        )
        phase = resume_arm_phase(phase, args.resume_after_arm_steps)
        limits = arm_limits(args.calibration)
        validate_positions("phase start", phase.expected_arm_start, limits)
        validate_positions("phase end", phase.expected_arm_end, limits)
        calibration = load_calibration(args.calibration)
    except Exception as error:
        print(f"PICK_PLACE_PHASE_PRECHECK_FAIL reason={error}")
        return 2

    rclpy.init()
    node = rclpy.create_node("so101_execute_pick_place_phase_once")
    action_client = ActionClient(node, ExecuteTrajectory, ACTION_NAME)
    diagnostics_client = node.create_client(Trigger, DIAGNOSTICS_SERVICE)
    try:
        if not action_client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError(f"Action server unavailable: {ACTION_NAME}")
        current_arm, current_gripper = positions_from_joint_state(
            wait_for_joint_state(node, STATE_TIMEOUT_S)
        )
        start_error = validate_pose(
            current_arm,
            phase.expected_arm_start,
            START_TOLERANCE_RAD,
            "phase start",
        )
        expected_gripper = 0.06 if phase.name in OPEN_GRIPPER_PHASES else 0.13
        gripper_start_error = validate_gripper_position(
            current_gripper,
            expected_gripper,
        )
        get_diagnostics(node, diagnostics_client, phase, calibration, "before")
        print(
            "PICK_PLACE_PHASE_EXECUTE_REQUEST "
            f"phase={phase.name} commands={len(phase.steps)} "
            f"resume_after_arm_steps={args.resume_after_arm_steps} "
            f"max_start_error_rad={start_error:.6f} "
            f"gripper_start_error_rad={gripper_start_error:.6f} retries=0"
        )

        for step in phase.steps:
            current_arm, _ = positions_from_joint_state(
                wait_for_joint_state(node, STATE_TIMEOUT_S)
            )
            if step["kind"] == "arm":
                expected_start = _vector(
                    step["start_positions_rad"],
                    "arm start",
                )
                target = _vector(step["target_positions_rad"], "arm target")
                validate_pose(
                    current_arm,
                    expected_start,
                    START_TOLERANCE_RAD,
                    "segment start",
                )
                actual_step = validate_actual_step(
                    current_arm,
                    target,
                    phase.maximum_joint_step_rad,
                )
                print(
                    "PICK_PLACE_PHASE_STEP_REQUEST "
                    f"phase={phase.name} step={step['index']} kind=arm "
                    f"actual_step_rad={actual_step:.6f}"
                )
                preset = Preset(
                    ARM_CONTROLLER,
                    ARM_JOINTS,
                    target,
                    int(step["duration_s"]),
                )
                execute_goal_once(
                    node,
                    action_client,
                    preset,
                    phase.name,
                    int(step["index"]),
                )
                final_arm, _ = positions_from_joint_state(
                    wait_for_joint_state(node, STATE_TIMEOUT_S)
                )
                validate_pose(
                    final_arm,
                    target,
                    FINAL_TOLERANCE_RAD,
                    "segment final",
                )
            else:
                target = float(step["target_position_rad"])
                print(
                    "PICK_PLACE_PHASE_STEP_REQUEST "
                    f"phase={phase.name} step={step['index']} kind=gripper "
                    f"target_rad={target:.6f}"
                )
                preset = Preset(
                    GRIPPER_CONTROLLER,
                    (GRIPPER_JOINT,),
                    (target,),
                    int(step["duration_s"]),
                )
                execute_goal_once(
                    node,
                    action_client,
                    preset,
                    phase.name,
                    int(step["index"]),
                )

        final_arm, final_gripper = positions_from_joint_state(
            wait_for_joint_state(node, STATE_TIMEOUT_S)
        )
        final_error = validate_pose(
            final_arm,
            phase.expected_arm_end,
            FINAL_TOLERANCE_RAD,
            "phase final",
        )
        expected_final_gripper = (
            0.13 if phase.name in CONTACT_REQUIRED_AFTER else 0.06
            if phase.name in OPEN_REQUIRED_AFTER
            else expected_gripper
        )
        gripper_final_error = validate_gripper_position(
            final_gripper,
            expected_final_gripper,
        )
        get_diagnostics(node, diagnostics_client, phase, calibration, "after")
        print(
            "PICK_PLACE_PHASE_EXECUTE_PASS "
            f"phase={phase.name} commands={len(phase.steps)} "
            f"max_final_error_rad={final_error:.6f} "
            f"gripper_final_error_rad={gripper_final_error:.6f} retries=0"
        )
        return 0
    except Exception as error:
        print(
            "PICK_PLACE_PHASE_EXECUTE_FAIL "
            f"phase={phase.name} reason={error} subsequent_commands=0"
        )
        return 1
    finally:
        action_client.destroy()
        node.destroy_client(diagnostics_client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
