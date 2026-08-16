#!/usr/bin/env python3
"""Execute one freshly generated camera-routed plan through the resident adapter."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for source_path in (
    ROOT / "tools",
    ROOT / "ros2_ws" / "src" / "single_arm_bridge",
):
    if str(source_path) not in sys.path:
        sys.path.insert(0, str(source_path))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState  # noqa: E402
from so101_interfaces.msg import BimanualJointFeedback  # noqa: E402
from so101_interfaces.srv import BimanualStreamCommand  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from top_pick_place_application import (  # noqa: E402
    ARM_JOINTS_BY_SIDE,
    ARM_TERMINAL_TOLERANCE_RAD,
    BIMANUAL_ARM_INDICES,
    CANONICAL_JOINTS,
    CONTACT_THRESHOLD_RAW,
    RELEASE_TOLERANCE_RAW,
    RAW_STEP_RAD,
    bimanual_q0_target,
    interpolate_finite_points,
    residual_raw,
    sha256_file,
    split_finite_targets,
    step_target,
    validate_bimanual_q0,
)


CONFIRMATION = "RUN_TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE"
OPEN_GRASP_HEIGHT_CHECK_CONFIRMATION = (
    "RUN_TOP_CAMERA_OPEN_GRASP_HEIGHT_CHECK_ONCE"
)
RIGHT_PLACE_CONFIRMATION = "RUN_RIGHT_PLACE_HEIGHT_CHECK_ONCE"
OWNER = "top_camera_pick_place_application"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"
EXPECTED_FIRMWARES = frozenset(("0x00024809",))
EXPECTED_PLAN_STATUS = "DYNAMIC_TOP_PICK_PLACE_PLAN_ONLY_PASS"
MAXIMUM_PLAN_AGE_S = 300.0
MAXIMUM_FALLBACK_FEEDBACK_AGE_MS = 150
MAXIMUM_LOCKED_WRIST_ROLL_RAD = 1.0e-6
MAXIMUM_ENDPOINT_RESIDUAL_M = 0.0021
HOMING_INTERMEDIATE_TOLERANCE_RAD = 0.05
BIMANUAL_Q0_TOLERANCE_RAD = ARM_TERMINAL_TOLERANCE_RAD
MAXIMUM_HOMING_SUBLEGS = 12
CONTINUOUS_SAMPLE_PERIOD_MS = 50
CONTINUOUS_FIRST_POINT_MS = 80
CONTINUOUS_COMMAND_RATE_RAD_S = 200.0 * RAW_STEP_RAD
EXPECTED_BASELINE_PICK_GRASP_OFFSET_M = 0.011
EXPECTED_PREVIOUS_PICK_GRASP_OFFSET_M = 0.002
EXPECTED_PICK_GRASP_OFFSET_M = -0.001
EXPECTED_PICK_GRASP_DOWNWARD_ADJUSTMENT_M = 0.003
EXPECTED_PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M = 0.012
EXPECTED_LEFT_SCREEN_X_CORRECTION_M = 0.01372
EXPECTED_LEFT_SCREEN_X_CORRECTION_REASON = (
    "operator_requested_left_target_13_72mm_screen_right"
)
EXPECTED_RIGHT_SCREEN_X_CORRECTION_M = -0.02947
EXPECTED_RIGHT_SCREEN_X_CORRECTION_REASON = (
    "operator_requested_right_target_29_47mm_screen_left"
)
EXPECTED_GRIPPER_OPEN_TARGET_RAW = 2048
EXPECTED_GRIPPER_OPEN_TARGET_RAD = (
    2048 - EXPECTED_GRIPPER_OPEN_TARGET_RAW
) * RAW_STEP_RAD
EXPECTED_GRIPPER_CLOSE_TARGET_RAW = 1948
EXPECTED_GRIPPER_CLOSE_TARGET_RAD = (
    2048 - EXPECTED_GRIPPER_CLOSE_TARGET_RAW
) * RAW_STEP_RAD
ENDPOINT_SEQUENCE = (
    "pick_pregrasp",
    "pick_grasp",
    "pick_lift",
    "place_pregrasp",
    "place_grasp",
)
REQUIRED_ENDPOINTS = frozenset(ENDPOINT_SEQUENCE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", default="")
    parser.add_argument("--right-place-validation", default="")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--open-grasp-height-check", action="store_true")
    parser.add_argument("--hold-at-grasp-s", type=float, default=8.0)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--plan",
        type=Path,
        default=(
            ROOT
            / "artifacts/top_pick_place/2026-08-14/dynamic_plan_run01.json"
        ),
    )
    parser.add_argument(
        "--plan-sha256",
        default="",
        help="required for execution; printed by the plan-only tool",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            ROOT
            / "artifacts/top_pick_place/2026-08-14/application_once_run01.json"
        ),
    )
    args = parser.parse_args()
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if not 2.0 <= args.hold_at_grasp_s <= 15.0:
        parser.error("--hold-at-grasp-s must be within 2..15 seconds")
    if not args.validate_only:
        expected_confirmation = (
            OPEN_GRASP_HEIGHT_CHECK_CONFIRMATION
            if args.open_grasp_height_check
            else CONFIRMATION
        )
        if args.confirmation != expected_confirmation:
            parser.error(
                f"execution requires --confirmation {expected_confirmation}"
            )
        if len(args.plan_sha256) != 64:
            parser.error("execution requires --plan-sha256 with 64 hex characters")
    return args


def load_dynamic_plan(path: Path, expected_sha256: str, validate_only: bool):
    actual_sha256 = sha256_file(path)
    if expected_sha256 and actual_sha256 != expected_sha256.lower():
        raise RuntimeError(
            "dynamic plan sha256 mismatch: "
            f"expected={expected_sha256.lower()} actual={actual_sha256}"
        )
    plan = json.loads(path.read_text(encoding="utf-8"))
    routing = plan.get("routing", {})
    side = routing.get("selected_arm")
    if (
        plan.get("schema_version") != 12
        or plan.get("status") != EXPECTED_PLAN_STATUS
        or plan.get("execution_api_used") is not False
        or plan.get("motion_authorized") is not False
        or plan.get("automatic_execution_permitted") is not False
        or side not in ARM_JOINTS_BY_SIDE
        or tuple(plan.get("joint_names", ())) != ARM_JOINTS_BY_SIDE[side]
        or routing.get("rule") != "source_image_center_x"
        or routing.get("nonselected_arm_behavior") != "hold_bimanual_q0"
        or not isinstance(plan.get("steps"), list)
        or not plan["steps"]
    ):
        raise RuntimeError("dynamic camera plan contract is invalid")
    lateral = plan.get("lateral_adjustment", {})
    target_lock = plan.get("target_lock", {})
    expected_correction_m = (
        EXPECTED_LEFT_SCREEN_X_CORRECTION_M
        if side == "left"
        else EXPECTED_RIGHT_SCREEN_X_CORRECTION_M
    )
    expected_correction_reason = (
        EXPECTED_LEFT_SCREEN_X_CORRECTION_REASON
        if side == "left"
        else EXPECTED_RIGHT_SCREEN_X_CORRECTION_REASON
    )
    unit = lateral.get("direction_unit_workcell_xy", ())
    delta = lateral.get("delta_workcell_xy_m", ())
    observed = lateral.get("observed_target_xy_m", ())
    corrected = lateral.get("corrected_target_xy_m", ())
    homography = lateral.get("homography", {})
    homography_path = Path(str(homography.get("path", "")))
    if (
        lateral.get("applied") is not True
        or lateral.get("selected_arm") != side
        or lateral.get("screen_axis") != "positive_image_x"
        or not math.isclose(
            float(
                lateral.get(
                    "operator_requested_screen_x_correction_m", math.nan
                )
            ),
            expected_correction_m,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(lateral.get("command_correction_m", math.nan)),
            expected_correction_m,
            abs_tol=1e-9,
        )
        or lateral.get("reason") != expected_correction_reason
        or len(unit) != 2
        or len(delta) != 2
        or len(observed) != 2
        or len(corrected) != 2
        or not math.isclose(
            math.hypot(*map(float, unit)), 1.0, abs_tol=1e-9
        )
        or not all(
            math.isclose(
                float(delta[index]),
                float(unit[index]) * expected_correction_m,
                abs_tol=1e-9,
            )
            for index in range(2)
        )
        or not all(
            math.isclose(
                float(corrected[index]),
                float(observed[index]) + float(delta[index]),
                abs_tol=1e-9,
            )
            for index in range(2)
        )
        or not math.isclose(
            float(target_lock.get("x_m", math.nan)),
            float(corrected[0]),
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(target_lock.get("y_m", math.nan)),
            float(corrected[1]),
            abs_tol=1e-9,
        )
        or not homography_path.is_file()
        or sha256_file(homography_path) != str(homography.get("sha256", ""))
    ):
        raise RuntimeError(
            f"dynamic camera plan {side} lateral contract is invalid"
        )

    pick_offsets = plan.get("pick_offsets_m", {})
    height_adjustment = plan.get("height_adjustment", {})
    if (
        not math.isclose(
            float(pick_offsets.get("grasp", math.nan)),
            EXPECTED_PICK_GRASP_OFFSET_M,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(height_adjustment.get("baseline_grasp_offset_m", math.nan)),
            EXPECTED_BASELINE_PICK_GRASP_OFFSET_M,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(height_adjustment.get("previous_grasp_offset_m", math.nan)),
            EXPECTED_PREVIOUS_PICK_GRASP_OFFSET_M,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(height_adjustment.get("selected_grasp_offset_m", math.nan)),
            EXPECTED_PICK_GRASP_OFFSET_M,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(height_adjustment.get("downward_adjustment_m", math.nan)),
            EXPECTED_PICK_GRASP_DOWNWARD_ADJUSTMENT_M,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(
                height_adjustment.get(
                    "cumulative_downward_adjustment_m", math.nan
                )
            ),
            EXPECTED_PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M,
            abs_tol=1e-9,
        )
        or height_adjustment.get("reason")
        != "operator_observed_run10_grasp_still_too_high"
    ):
        raise RuntimeError("dynamic camera plan grasp-height contract is invalid")

    endpoints = plan.get("endpoints")
    if not isinstance(endpoints, dict) or set(endpoints) != REQUIRED_ENDPOINTS:
        raise RuntimeError("dynamic camera plan endpoint set is invalid")

    gripper_contract = plan.get("gripper_contract", {})
    pick_open_steps = [
        step
        for step in plan["steps"]
        if step.get("kind") == "gripper"
        and step.get("phase") == "pick_open"
    ]
    place_release_steps = [
        step
        for step in plan["steps"]
        if step.get("kind") == "gripper"
        and step.get("phase") == "place_release"
    ]
    pick_close_steps = [
        step
        for step in plan["steps"]
        if step.get("kind") == "gripper"
        and step.get("phase") == "pick_close"
    ]
    if (
        gripper_contract.get("preopen_required") is not True
        or gripper_contract.get("open_phase") != "before_approach"
        or int(gripper_contract.get("open_target_raw", -1))
        != EXPECTED_GRIPPER_OPEN_TARGET_RAW
        or not math.isclose(
            float(gripper_contract.get("open_target_rad", math.nan)),
            EXPECTED_GRIPPER_OPEN_TARGET_RAD,
            abs_tol=1e-9,
        )
        or int(gripper_contract.get("close_target_raw", -1))
        != EXPECTED_GRIPPER_CLOSE_TARGET_RAW
        or not math.isclose(
            float(gripper_contract.get("close_target_rad", math.nan)),
            EXPECTED_GRIPPER_CLOSE_TARGET_RAD,
            abs_tol=1e-9,
        )
        or int(gripper_contract.get("contact_threshold_raw", -1))
        != CONTACT_THRESHOLD_RAW
        or int(gripper_contract.get("empty_grasp_observed_residual_raw", -1))
        != 2
        or int(gripper_contract.get("held_object_observed_residual_raw", -1))
        != 8
        or int(
            gripper_contract.get(
                "expected_held_residual_after_target_change_raw", -1
            )
        )
        != 23
        or len(pick_open_steps) != 1
        or plan["steps"][0] is not pick_open_steps[0]
        or not math.isclose(
            float(
                pick_open_steps[0].get("target_position_rad", math.nan)
            ),
            EXPECTED_GRIPPER_OPEN_TARGET_RAD,
            abs_tol=1e-9,
        )
        or len(place_release_steps) != 1
        or not math.isclose(
            float(
                place_release_steps[0].get(
                    "target_position_rad", math.nan
                )
            ),
            EXPECTED_GRIPPER_OPEN_TARGET_RAD,
            abs_tol=1e-9,
        )
        or len(pick_close_steps) != 1
        or not math.isclose(
            float(
                pick_close_steps[0].get("target_position_rad", math.nan)
            ),
            EXPECTED_GRIPPER_CLOSE_TARGET_RAD,
            abs_tol=1e-9,
        )
    ):
        raise RuntimeError("dynamic camera plan gripper contract is invalid")
    for name in ENDPOINT_SEQUENCE:
        endpoint = endpoints[name]
        geometry = endpoint.get("grasp_geometry", {})
        wrist_reference = float(
            endpoint.get("wrist_roll_reference_rad", math.nan)
        )
        wrist_delta = float(
            endpoint.get("wrist_roll_delta_rad", math.inf)
        )
        final_positions = endpoint.get("final_joint_positions_rad", ())
        endpoint_residual = float(
            endpoint.get("plan_residual_norm_m", math.inf)
        )
        if (
            endpoint.get("wrist_roll_yaw_correction_applied") is not False
            or endpoint.get("wrist_roll_policy") != "hold_bimanual_q0"
            or endpoint.get("wrist_roll_locked") is not True
            or geometry.get("relationship")
            != "informational_only_wrist_locked_at_q0"
            or len(final_positions) != 5
            or not math.isfinite(wrist_reference)
            or abs(wrist_reference) > MAXIMUM_LOCKED_WRIST_ROLL_RAD
            or not math.isfinite(wrist_delta)
            or abs(wrist_delta) > MAXIMUM_LOCKED_WRIST_ROLL_RAD
            or abs(float(final_positions[4]))
            > MAXIMUM_LOCKED_WRIST_ROLL_RAD
            or not math.isfinite(endpoint_residual)
            or endpoint_residual > MAXIMUM_ENDPOINT_RESIDUAL_M
        ):
            raise RuntimeError(
                f"dynamic camera plan q0 wrist-lock contract is invalid: {name}"
            )

    for step in plan["steps"]:
        if step.get("kind") != "arm":
            continue
        target_positions = step.get("target_positions_rad", ())
        if len(target_positions) != 5:
            raise RuntimeError("dynamic camera arm step shape is invalid")
        wrist_roll = float(target_positions[4])
        if (
            not math.isfinite(wrist_roll)
            or abs(wrist_roll) > MAXIMUM_LOCKED_WRIST_ROLL_RAD
        ):
            raise RuntimeError(
                "dynamic camera plan attempts to move the locked wrist: "
                f"step={step.get('index')} value={wrist_roll:.6f}rad"
            )

    generated_at = float(plan.get("generated_at_unix_s", 0.0))
    age_s = time.time() - generated_at
    if not validate_only and (age_s < 0.0 or age_s > MAXIMUM_PLAN_AGE_S):
        raise RuntimeError(
            f"dynamic camera plan is stale: age={age_s:.1f}s "
            f"limit={MAXIMUM_PLAN_AGE_S:.1f}s"
        )
    return plan, side, actual_sha256, age_s


def call(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}") from error
    return future.result()


def status_document(node: Node, client, timeout_s: float) -> dict:
    response = call(node, client, Trigger.Request(), timeout_s)
    document = json.loads(response.message)
    if not response.success or document.get("state") == "faulted":
        raise RuntimeError(f"resident adapter is unhealthy: {response}")
    return document


def feedback_positions(
    feedback: BimanualJointFeedback,
    *,
    label: str,
) -> tuple[float, ...]:
    if tuple(feedback.joint_names) != CANONICAL_JOINTS:
        raise RuntimeError(f"unexpected {label} feedback joint order")
    positions = tuple(float(value) for value in feedback.positions)
    ages = tuple(int(value) for value in feedback.sample_age_ms)
    if (
        len(positions) != 12
        or len(ages) != 12
        or not all(math.isfinite(value) for value in positions)
        or int(feedback.present_mask) != 0x0FFF
    ):
        raise RuntimeError(f"incomplete {label} feedback snapshot")
    maximum_age_ms = max(ages)
    if maximum_age_ms > MAXIMUM_FALLBACK_FEEDBACK_AGE_MS:
        raise RuntimeError(
            f"stale {label} feedback: maximum_age_ms={maximum_age_ms}"
        )
    return positions


def status_prepared_positions(
    document: dict,
    *,
    label: str,
    expected_epoch: int,
    require_torque_hold: bool,
) -> tuple[float, ...]:
    if int(document.get("prepared_epoch", -1)) != expected_epoch:
        raise RuntimeError(f"{label} prepared epoch mismatch")
    if require_torque_hold and document.get("torque_hold_active") is not True:
        raise RuntimeError(f"{label} status does not prove torque hold")
    values = document.get("prepared_positions_rad")
    if (
        not isinstance(values, list)
        or len(values) != 12
        or not all(math.isfinite(float(value)) for value in values)
    ):
        raise RuntimeError(f"{label} status has no complete prepared anchor")
    return tuple(float(value) for value in values)


def wait_for_anchor(
    node: Node,
    storage: list[JointState],
    feedback_messages: list[BimanualJointFeedback],
    timeout_s: float,
):
    deadline = time.monotonic() + timeout_s
    while not storage and not feedback_messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if storage:
        anchor = storage[-1]
        if tuple(anchor.name) != CANONICAL_JOINTS:
            raise RuntimeError(f"unexpected resident joint order: {anchor.name}")
        if len(anchor.position) != 12:
            raise RuntimeError("resident anchor does not contain 12 positions")
        return (
            tuple(float(value) for value in anchor.position),
            "resident_anchor",
            None,
        )
    if feedback_messages:
        feedback = feedback_messages[-1]
        return (
            feedback_positions(feedback, label="startup"),
            "resident_fresh_feedback_fallback",
            feedback,
        )
    raise RuntimeError(
        f"timeout waiting for {ANCHOR_TOPIC} or fresh {FEEDBACK_TOPIC}"
    )

def terminal_anchor_settle_evidence(
    measured_positions: tuple[float, ...],
    *,
    target_positions: tuple[float, ...],
    joint_indices,
    label: str,
    measurement_source: str,
) -> dict:
    """Validate the measured terminal anchor emitted at READY transition.

    F8.9 does not declare a finite trajectory complete immediately after the
    final setpoint. Firmware first requires twelve consecutive measured joint
    pairs inside its terminal-settle bound. The resident adapter then validates
    one complete, <=150 ms-old 12-axis snapshot and publishes that measured
    vector as a new anchor while changing ACTIVE -> READY.

    Tracking intentionally stops after that handoff. Later feedback-topic
    messages repeat the same sample with an increasing age, so they are not
    independent post-settle observations. Use the fresh terminal anchor as the
    completion evidence. The application uses the same 46.020 mrad arm bound
    rather than contradicting a firmware-proven terminal result.
    """
    indices = tuple(int(index) for index in joint_indices)
    if len(measured_positions) != 12 or not all(
        math.isfinite(float(value)) for value in measured_positions
    ):
        raise RuntimeError(f"invalid {label} terminal anchor: {measured_positions}")
    terminal_error = max(
        abs(measured_positions[index] - target_positions[index])
        for index in indices
    )
    if terminal_error > ARM_TERMINAL_TOLERANCE_RAD:
        raise RuntimeError(
            f"{label} terminal anchor error={terminal_error:.6f}rad"
        )
    return {
        "source": measurement_source,
        "firmware_consecutive_joint_pairs": 12,
        "resident_terminal_feedback_validated": True,
        "application_terminal_error_rad": terminal_error,
        "application_tolerance_rad": ARM_TERMINAL_TOLERANCE_RAD,
    }


def trajectory_point(positions, offset_ms: int):
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = offset_ms // 1000
    point.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return point


def finite_request(start, target):
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.START_FINITE
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINTS)
    request.points = [
        trajectory_point(positions, offset)
        for offset, positions in interpolate_finite_points(start, target)
    ]
    return request


def continuous_finite_request(
    start: tuple[float, ...],
    targets: list[tuple[float, ...]],
):
    if not targets:
        raise RuntimeError("continuous finite route requires a target")
    points: list[JointTrajectoryPoint] = []
    offset_ms = CONTINUOUS_FIRST_POINT_MS - CONTINUOUS_SAMPLE_PERIOD_MS
    segment_start = tuple(start)
    for segment_target in targets:
        largest = max(
            abs(end - begin)
            for begin, end in zip(segment_start, segment_target, strict=True)
        )
        sample_count = max(
            1,
            math.ceil(
                largest
                / (
                    CONTINUOUS_COMMAND_RATE_RAD_S
                    * CONTINUOUS_SAMPLE_PERIOD_MS
                    / 1000.0
                )
            ),
        )
        for sample_index in range(1, sample_count + 1):
            fraction = sample_index / sample_count
            positions = tuple(
                begin + (end - begin) * fraction
                for begin, end in zip(
                    segment_start, segment_target, strict=True
                )
            )
            offset_ms += CONTINUOUS_SAMPLE_PERIOD_MS
            points.append(trajectory_point(positions, offset_ms))
        segment_start = tuple(segment_target)
    if len(points) == 1:
        offset_ms += CONTINUOUS_SAMPLE_PERIOD_MS
        points.append(trajectory_point(targets[-1], offset_ms))
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.START_FINITE
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINTS)
    request.points = points
    return request


def continuous_actions(plan: dict) -> list[dict]:
    """Group camera waypoints into the proven three arm legs."""
    actions: list[dict] = []
    arm_steps: list[dict] = []

    def flush_arm() -> None:
        if not arm_steps:
            return
        phases = [str(step["phase"]) for step in arm_steps]
        actions.append(
            {
                "kind": "arm_route",
                "label": f"{phases[0]}..{phases[-1]}",
                "steps": list(arm_steps),
            }
        )
        arm_steps.clear()

    for step in plan["steps"]:
        if step["kind"] == "arm":
            arm_steps.append(step)
            continue
        flush_arm()
        actions.append(
            {
                "kind": "gripper",
                "label": str(step["phase"]),
                "steps": [step],
            }
        )
    flush_arm()
    kinds = tuple(action["kind"] for action in actions)
    labels = tuple(action["label"] for action in actions)
    if kinds != (
        "gripper",
        "arm_route",
        "gripper",
        "arm_route",
        "gripper",
        "arm_route",
    ) or labels[0] != "pick_open" or labels[2] != "pick_close" or labels[4] != "place_release":
        raise RuntimeError(f"unexpected continuous action partition: {labels}")
    return actions


def response_document(response) -> dict:
    return {
        "accepted": bool(response.accepted),
        "adapter_state": str(response.adapter_state),
        "arbiter_epoch": int(response.arbiter_epoch),
        "diagnostic": str(response.diagnostic),
    }


def wait_until_ready(
    node: Node,
    status_client,
    terminal_anchors: list[JointState],
    feedback_messages: list[BimanualJointFeedback],
    *,
    epoch: int,
    timeout_s: float,
):
    deadline = time.monotonic() + timeout_s
    history = []
    while time.monotonic() < deadline:
        document = status_document(node, status_client, timeout_s)
        history.append(document)
        if (
            document.get("state") == "ready"
            and document.get("owner") == OWNER
            and document.get("arbiter_epoch") == epoch
        ):
            if "prepared_positions_rad" in document:
                measured = status_prepared_positions(
                    document,
                    label="terminal",
                    expected_epoch=epoch,
                    require_torque_hold=True,
                )
                topic_feedback = (
                    feedback_messages[-1] if feedback_messages else None
                )
                return (
                    history,
                    measured,
                    topic_feedback,
                    "resident_status_terminal_anchor",
                )
            anchor_deadline = min(deadline, time.monotonic() + 1.0)
            while not terminal_anchors and time.monotonic() < anchor_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if terminal_anchors:
                anchor = terminal_anchors[-1]
                if (
                    tuple(anchor.name) != CANONICAL_JOINTS
                    or len(anchor.position) != 12
                    or not all(
                        math.isfinite(float(value)) for value in anchor.position
                    )
                ):
                    raise RuntimeError(f"invalid measured terminal anchor: {anchor}")
                measured = tuple(float(value) for value in anchor.position)
                topic_feedback = (
                    feedback_messages[-1] if feedback_messages else None
                )
                return (
                    history,
                    measured,
                    topic_feedback,
                    "resident_terminal_anchor",
                )

            # READY proves firmware terminal settling. If cross-host DDS drops
            # the transient-local anchor, require a newly received complete and
            # fresh feedback sample instead of using an ACTIVE-era cache.
            feedback_messages.clear()
            feedback_deadline = min(deadline, time.monotonic() + 2.0)
            while not feedback_messages and time.monotonic() < feedback_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not feedback_messages:
                raise RuntimeError(
                    "finite leg completed without terminal anchor or fresh feedback"
                )
            topic_feedback = feedback_messages[-1]
            measured = feedback_positions(topic_feedback, label="terminal")
            return (
                history,
                measured,
                topic_feedback,
                "resident_ready_fresh_feedback_fallback",
            )
        if document.get("state") not in ("active", "ready"):
            raise RuntimeError(f"unexpected resident state: {document}")
        time.sleep(0.02)
    raise RuntimeError(f"timeout waiting for finite epoch={epoch}: {history[-1:]}")


def stop_request():
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.STOP
    request.owner = OWNER
    return request


def main() -> int:
    args = parse_args()
    plan, selected_arm, plan_sha256, plan_age_s = load_dynamic_plan(
        args.plan, args.plan_sha256, args.validate_only
    )
    if args.validate_only:
        result = {
            "schema_version": 5,
            "record_kind": "top_camera_resident_pick_place_once",
            "operator_confirmation": args.confirmation,
            "validate_only": True,
            "plan": str(args.plan),
            "plan_sha256": plan_sha256,
            "plan_age_s": plan_age_s,
            "selected_arm": selected_arm,
            "routing": plan["routing"],
            "target_lock": plan["target_lock"],
            "homing": {
                "measurement_required_at_execution": True,
                "motion_commands": 0,
            },
            "legs": [],
            "automatic_retry_count": 0,
            "resident_services_called": 0,
            "motion_commands": 0,
            "overall_verdict": "TOP_PICK_PLACE_DYNAMIC_VALIDATE_ONLY_PASS",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = sha256(args.output.read_bytes()).hexdigest()
        print(
            "TOP_PICK_PLACE_DYNAMIC_VALIDATE_ONLY_PASS "
            f"selected_arm={selected_arm} motion_commands=0 "
            f"resident_services_called=0 output={args.output} sha256={digest}"
        )
        return 0
    if (
        not args.open_grasp_height_check
        and selected_arm == "right"
        and plan.get("place_target_source", {}).get(
            "right_arm_physical_place_validation_required"
        )
        and args.right_place_validation != RIGHT_PLACE_CONFIRMATION
    ):
        raise RuntimeError(
            "right-arm execution requires --right-place-validation "
            f"{RIGHT_PLACE_CONFIRMATION} for the supervised first height check"
        )

    rclpy.init()
    node = Node("top_camera_resident_pick_place_once")
    transient_qos = QoSProfile(depth=1)
    transient_qos.reliability = ReliabilityPolicy.RELIABLE
    transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    feedback_qos = QoSProfile(depth=20)
    feedback_qos.reliability = ReliabilityPolicy.RELIABLE
    anchors: list[JointState] = []
    feedback_messages: list[BimanualJointFeedback] = []
    node.create_subscription(
        JointState, ANCHOR_TOPIC, anchors.append, transient_qos
    )
    node.create_subscription(
        BimanualJointFeedback,
        FEEDBACK_TOPIC,
        feedback_messages.append,
        feedback_qos,
    )
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    refresh_anchor_client = node.create_client(
        Trigger, REFRESH_ANCHOR_SERVICE
    )
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)

    motion_started = False
    successful_hold = False
    resident_ready_for_hold = False
    result = {
        "schema_version": 5,
        "record_kind": "top_camera_resident_pick_place_once",
        "operator_confirmation": args.confirmation,
        "validate_only": bool(args.validate_only),
        "plan": str(args.plan),
        "plan_sha256": plan_sha256,
        "plan_age_s": plan_age_s,
        "selected_arm": selected_arm,
        "routing": plan["routing"],
        "target_lock": plan["target_lock"],
        "homing": {"legs": []},
        "legs": [],
        "automatic_retry_count": 0,
        "right_place_height_check_mode": (
            "supervised_first_execution"
            if selected_arm == "right" and not args.open_grasp_height_check
            else None
        ),
        "open_grasp_height_check": bool(args.open_grasp_height_check),
        "hold_at_grasp_s": (
            float(args.hold_at_grasp_s)
            if args.open_grasp_height_check
            else None
        ),
    }
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (REFRESH_ANCHOR_SERVICE, refresh_anchor_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        initial = status_document(node, status_client, args.timeout_s)
        initial_owner = initial.get("owner")
        initial_epoch = int(initial.get("arbiter_epoch", -1))
        if (
            initial.get("state") != "ready"
            or initial_owner not in (None, OWNER)
            or (initial_owner is None and initial_epoch != 0)
            or initial.get("firmware_version") not in EXPECTED_FIRMWARES
            or initial.get("motion_authorized") is not True
        ):
            raise RuntimeError(f"unexpected resident initial state: {initial}")
        anchors.clear()
        feedback_messages.clear()
        if initial_owner is None:
            refresh_response = call(
                node,
                refresh_anchor_client,
                Trigger.Request(),
                args.timeout_s,
            )
            if not refresh_response.success:
                raise RuntimeError(
                    f"resident anchor refresh failed: {refresh_response}"
                )
            anchor_refresh = json.loads(refresh_response.message)
            if "prepared_positions_rad" in anchor_refresh:
                startup_anchor = status_prepared_positions(
                    anchor_refresh,
                    label="unarmed refresh",
                    expected_epoch=initial_epoch,
                    require_torque_hold=False,
                )
                startup_feedback = None
                initial_position_source = "resident_refresh_service_anchor"
            else:
                # Backward-compatible path while an older Pi resident is still
                # running. A newly deployed resident returns the anchor directly
                # in the reliable refresh service response.
                feedback_messages.clear()
                startup_anchor, startup_measurement_source, startup_feedback = (
                    wait_for_anchor(
                        node,
                        anchors,
                        feedback_messages,
                        args.timeout_s,
                    )
                )
                initial_position_source = (
                    "resident_immediate_pre_motion_anchor"
                    if startup_measurement_source == "resident_anchor"
                    else "resident_immediate_pre_motion_fresh_feedback_fallback"
                )
            anchor_torque_state = "off"
        else:
            # The resident keeps the latest READY terminal measurement in its
            # prepared state. Recover it through the reliable status service so
            # repeat runs do not depend on transient-local DDS delivery or an
            # aging feedback cache.
            startup_anchor = status_prepared_positions(
                initial,
                label="same-owner startup",
                expected_epoch=initial_epoch,
                require_torque_hold=True,
            )
            startup_feedback = None
            initial_position_source = "resident_armed_status_terminal_anchor"
            anchor_refresh = {
                "skipped": True,
                "reason": "same_owner_armed_ready_reuses_status_terminal_anchor",
                "state": initial["state"],
                "owner": initial_owner,
                "arbiter_epoch": initial_epoch,
                "torque_enabled": True,
            }
            anchor_torque_state = "hold"
        if startup_feedback is not None:
            result["startup_feedback_age_max_ms"] = max(
                int(value) for value in startup_feedback.sample_age_ms
            )
        current_positions = startup_anchor
        result["initial_status"] = initial
        result["startup_anchor_rad"] = list(startup_anchor)
        result["initial_position_source"] = initial_position_source
        result["anchor_refresh"] = anchor_refresh

        print(
            "TOP_PICK_PLACE_FRESH_ANCHOR_PASS "
            f"source={initial_position_source} torque={anchor_torque_state}"
        )

        print(
            "TOP_PICK_PLACE_DYNAMIC_PLAN_GATE_PASS "
            f"selected_arm={selected_arm} plan_age_s={plan_age_s:.1f} "
            f"pixel_x={plan['routing']['center_x_px']:.1f}/"
            f"{plan['routing']['image_width_px']}"
        )
        if args.validate_only:
            q0_target = bimanual_q0_target(current_positions)
            q0_residual = max(
                abs(current_positions[index]) for index in BIMANUAL_ARM_INDICES
            )
            q0_subleg_count = len(
                split_finite_targets(current_positions, q0_target)
            )
            result["homing"].update(
                {
                    "required": q0_residual > 0.03,
                    "maximum_initial_residual_rad": q0_residual,
                    "subleg_count": q0_subleg_count,
                    "grippers_preserved": True,
                    "motion_commands": 0,
                }
            )
            result["overall_verdict"] = "TOP_PICK_PLACE_DYNAMIC_VALIDATE_ONLY_PASS"
            print(
                "TOP_PICK_PLACE_DYNAMIC_VALIDATE_ONLY_PASS "
                f"motion_commands=0 bimanual_q0_initial_residual_rad={q0_residual:.6f} "
                f"planned_homing_sublegs={q0_subleg_count}"
            )
        else:
            epoch = initial_epoch
            commanded = current_positions
            q0_target = bimanual_q0_target(commanded)
            initial_q0_residual = max(
                abs(commanded[index]) for index in BIMANUAL_ARM_INDICES
            )
            q0_request = continuous_finite_request(commanded, [q0_target])
            result["homing"].update(
                {
                    "maximum_initial_residual_rad": initial_q0_residual,
                    "subleg_count": 1,
                    "target_positions_rad": list(q0_target),
                    "grippers_preserved": True,
                    "torque_gap_before_pick_place": False,
                    "execution_style": "one_continuous_finite_leg",
                    "trajectory_points": len(q0_request.points),
                }
            )
            print(
                "TOP_PICK_PLACE_BIMANUAL_Q0_CONTINUOUS_PRECHECK_PASS "
                f"maximum_initial_residual_rad={initial_q0_residual:.6f} "
                f"points={len(q0_request.points)} grippers=hold"
            )
            resident_ready_for_hold = False
            motion_started = True
            anchors.clear()
            feedback_messages.clear()
            response = call(
                node, command_client, q0_request, args.timeout_s
            )
            expected_epoch = epoch + 1
            if (
                not response.accepted
                or response.adapter_state != "active"
                or int(response.arbiter_epoch) != expected_epoch
            ):
                raise RuntimeError(
                    f"continuous bimanual q0 rejected: {response}"
                )
            q0_duration_s = (
                q0_request.points[-1].time_from_start.sec
                + q0_request.points[-1].time_from_start.nanosec / 1e9
            )
            history, q0_measured, feedback, q0_settle_source = (
                wait_until_ready(
                node,
                status_client,
                anchors,
                feedback_messages,
                epoch=expected_epoch,
                    timeout_s=max(args.timeout_s, q0_duration_s + 5.0),
                )
            )
            resident_ready_for_hold = True
            q0_settle = terminal_anchor_settle_evidence(
                q0_measured,
                target_positions=q0_target,
                joint_indices=BIMANUAL_ARM_INDICES,
                label="bimanual q0",
                measurement_source=q0_settle_source,
            )
            q0_residual = validate_bimanual_q0(q0_measured)
            result["homing"]["legs"] = [
                {
                    "epoch": expected_epoch,
                    "start_positions_rad": list(commanded),
                    "target_positions_rad": list(q0_target),
                    "terminal_positions_rad": list(q0_measured),
                    "terminal_error_rad": q0_residual,
                    "trajectory_points": len(q0_request.points),
                    "status_samples": len(history),
                    "post_settle": q0_settle,
                }
            ]
            result["homing"]["maximum_final_residual_rad"] = q0_residual
            print(
                "TOP_PICK_PLACE_BIMANUAL_Q0_CONTINUOUS_HOLD_PASS "
                f"maximum_residual_rad={q0_residual:.6f} "
                f"settle_source={q0_settle_source} "
                f"epoch={expected_epoch} torque_hold=true"
            )
            epoch = expected_epoch
            commanded = q0_measured
            opposite_hold = (
                q0_measured[6:]
                if selected_arm == "left"
                else q0_measured[:6]
            )
            arm_indices = range(5) if selected_arm == "left" else range(6, 11)
            gripper_index = 5 if selected_arm == "left" else 11
            actions = continuous_actions(plan)
            result["execution_style"] = (
                "open_grasp_height_check_with_safe_pregrasp_return"
                if args.open_grasp_height_check
                else "proven_three_continuous_arm_legs_with_gripper_stops"
            )
            for action_index, action in enumerate(actions, start=1):
                targets: list[tuple[float, ...]] = []
                planned = commanded
                for step in action["steps"]:
                    planned = step_target(
                        planned, step, opposite_hold, arm=selected_arm
                    )
                    targets.append(planned)
                request = continuous_finite_request(commanded, targets)
                resident_ready_for_hold = False
                anchors.clear()
                feedback_messages.clear()
                response = call(
                    node, command_client, request, args.timeout_s
                )
                expected_epoch = epoch + 1
                if (
                    not response.accepted
                    or response.adapter_state != "active"
                    or int(response.arbiter_epoch) != expected_epoch
                ):
                    raise RuntimeError(
                        f"continuous action {action_index} rejected: {response}"
                    )
                duration_s = (
                    request.points[-1].time_from_start.sec
                    + request.points[-1].time_from_start.nanosec / 1e9
                )
                history, measured, feedback, settle_source = wait_until_ready(
                    node,
                    status_client,
                    anchors,
                    feedback_messages,
                    epoch=expected_epoch,
                    timeout_s=max(args.timeout_s, duration_s + 5.0),
                )
                resident_ready_for_hold = True
                post_settle = None
                if action["kind"] == "arm_route":
                    post_settle = terminal_anchor_settle_evidence(
                        measured,
                        target_positions=targets[-1],
                        joint_indices=arm_indices,
                        label=str(action["label"]),
                        measurement_source=settle_source,
                    )
                terminal_error = max(
                    abs(measured[index] - targets[-1][index])
                    for index in arm_indices
                )
                if (
                    action["kind"] == "arm_route"
                    and terminal_error > ARM_TERMINAL_TOLERANCE_RAD
                ):
                    raise RuntimeError(
                        "continuous arm terminal error "
                        f"action={action_index} label={action['label']} "
                        f"error={terminal_error:.6f}rad"
                    )
                gripper_gap_raw = None
                if action["label"] == "pick_close":
                    gripper_gap_raw = residual_raw(
                        targets[-1][gripper_index], measured[gripper_index]
                    )
                    if gripper_gap_raw < CONTACT_THRESHOLD_RAW:
                        raise RuntimeError(
                            "pick contact not detected: "
                            f"residual_gap_raw={gripper_gap_raw}"
                        )
                if action["label"] in ("pick_open", "place_release"):
                    gripper_gap_raw = residual_raw(
                        targets[-1][gripper_index], measured[gripper_index]
                    )
                    if gripper_gap_raw > RELEASE_TOLERANCE_RAW:
                        raise RuntimeError(
                            f"{action['label']} did not settle: "
                            f"residual_gap_raw={gripper_gap_raw}"
                        )
                result["legs"].append(
                    {
                        "action_index": action_index,
                        "action_count": (
                            3 if args.open_grasp_height_check else len(actions)
                        ),
                        "kind": action["kind"],
                        "label": action["label"],
                        "source_step_indices": [
                            int(step["index"]) for step in action["steps"]
                        ],
                        "epoch": expected_epoch,
                        "start_response": response_document(response),
                        "trajectory_points": len(request.points),
                        "duration_ms": round(duration_s * 1000.0),
                        "terminal_positions_rad": list(measured),
                        "arm_terminal_error_rad": terminal_error,
                        "gripper_residual_raw": gripper_gap_raw,
                        "last_topic_feedback_age_max_ms": (
                            max(int(value) for value in feedback.sample_age_ms)
                            if feedback is not None
                            else None
                        ),
                        "status_samples": len(history),
                        "post_settle": post_settle,
                    }
                )
                epoch = expected_epoch
                commanded = measured
                print(
                    "TOP_PICK_PLACE_CONTINUOUS_ACTION_PASS "
                    f"arm={selected_arm} action={action_index}/"
                    f"{3 if args.open_grasp_height_check else len(actions)} "
                    f"label={action['label']} "
                    f"points={len(request.points)} epoch={epoch} "
                    f"arm_error_mrad={terminal_error * 1000.0:.3f} "
                    "settle_source="
                    f"{settle_source}"
                )

                if args.open_grasp_height_check and action_index == 2:
                    print(
                        "TOP_OPEN_GRASP_HEIGHT_CHECK_HOLD "
                        f"seconds={args.hold_at_grasp_s:.1f} "
                        "gripper_commands_after_open=0 close_commands=0"
                    )
                    time.sleep(args.hold_at_grasp_s)
                    pregrasp_target = step_target(
                        commanded,
                        {
                            "kind": "arm",
                            "target_positions_rad": plan["endpoints"]
                            ["pick_pregrasp"]["final_joint_positions_rad"],
                        },
                        opposite_hold,
                        arm=selected_arm,
                    )
                    q0_return_target = bimanual_q0_target(pregrasp_target)
                    return_request = continuous_finite_request(
                        commanded,
                        [pregrasp_target, q0_return_target],
                    )
                    resident_ready_for_hold = False
                    anchors.clear()
                    feedback_messages.clear()
                    response = call(
                        node, command_client, return_request, args.timeout_s
                    )
                    return_epoch = epoch + 1
                    if (
                        not response.accepted
                        or response.adapter_state != "active"
                        or int(response.arbiter_epoch) != return_epoch
                    ):
                        raise RuntimeError(
                            f"open grasp height-check return rejected: {response}"
                        )
                    return_duration_s = (
                        return_request.points[-1].time_from_start.sec
                        + return_request.points[-1].time_from_start.nanosec / 1e9
                    )
                    (
                        return_history,
                        return_measured,
                        return_feedback,
                        return_settle_source,
                    ) = wait_until_ready(
                        node,
                        status_client,
                        anchors,
                        feedback_messages,
                        epoch=return_epoch,
                        timeout_s=max(
                            args.timeout_s,
                            return_duration_s + 5.0,
                        ),
                    )
                    resident_ready_for_hold = True
                    return_residual = validate_bimanual_q0(return_measured)
                    return_settle = terminal_anchor_settle_evidence(
                        return_measured,
                        target_positions=q0_return_target,
                        joint_indices=BIMANUAL_ARM_INDICES,
                        label="open grasp height-check safe return",
                        measurement_source=return_settle_source,
                    )
                    result["legs"].append(
                        {
                            "action_index": 3,
                            "action_count": 3,
                            "kind": "arm_route",
                            "label": "open_grasp_to_pregrasp_to_q0",
                            "source_step_indices": [],
                            "epoch": return_epoch,
                            "start_response": response_document(response),
                            "trajectory_points": len(return_request.points),
                            "duration_ms": round(return_duration_s * 1000.0),
                            "terminal_positions_rad": list(return_measured),
                            "arm_terminal_error_rad": return_residual,
                            "gripper_residual_raw": None,
                            "last_topic_feedback_age_max_ms": (
                                max(
                                    int(value)
                                    for value in return_feedback.sample_age_ms
                                )
                                if return_feedback is not None
                                else None
                            ),
                            "status_samples": len(return_history),
                            "post_settle": return_settle,
                        }
                    )
                    epoch = return_epoch
                    commanded = return_measured
                    result["height_check_close_commands"] = 0
                    result["height_check_return_via_pregrasp"] = True
                    print(
                        "TOP_OPEN_GRASP_HEIGHT_CHECK_RETURN_PASS "
                        f"epoch={epoch} q0_residual_rad={return_residual:.6f} "
                        f"settle_source={return_settle_source}"
                    )
                    break

            final = status_document(node, status_client, args.timeout_s)
            if (
                final.get("state") != "ready"
                or final.get("owner") != OWNER
                or int(final.get("arbiter_epoch", -1)) != epoch
            ):
                raise RuntimeError(f"unexpected final resident state: {final}")
            successful_hold = True
            result["final_status"] = final
            result["torque_hold_active"] = True
            result["coordinated_stop_sent"] = False
            if selected_arm == "right" and not args.open_grasp_height_check:
                result["right_place_height_check_motion_completed"] = True
                result["right_place_height_operator_observation_required"] = True
            result["overall_verdict"] = (
                "TOP_CAMERA_OPEN_GRASP_HEIGHT_CHECK_PASS_HOLDING"
                if args.open_grasp_height_check
                else "TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE_PASS_HOLDING"
            )

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = sha256(args.output.read_bytes()).hexdigest()
        print(
            f"{result['overall_verdict']} selected_arm={selected_arm} "
            f"legs={len(result['legs'])} "
            f"torque_hold={str(successful_hold).lower()} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    except Exception as error:
        result["overall_verdict"] = "TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE_FAIL"
        result["failure"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if motion_started and not successful_hold:
            hold_preserved = False
            if resident_ready_for_hold:
                try:
                    hold_status = status_document(
                        node, status_client, min(args.timeout_s, 2.0)
                    )
                    if (
                        hold_status.get("state") == "ready"
                        and hold_status.get("owner") == OWNER
                    ):
                        hold_preserved = True
                        result["overall_verdict"] = (
                            "TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE_FAIL_HOLDING"
                        )
                        result["failure_hold_preserved"] = True
                        result["failure_hold_status"] = hold_status
                        result["torque_hold_active"] = True
                        result["coordinated_stop_sent"] = False
                        print(
                            "TOP_PICK_PLACE_FAILURE_HOLD_PRESERVED "
                            f"epoch={hold_status.get('arbiter_epoch')} "
                            "torque_hold=true"
                        )
                except Exception as hold_error:
                    result["failure_hold_check_error"] = (
                        f"{type(hold_error).__name__}: {hold_error}"
                    )
            if not hold_preserved:
                try:
                    stopped = call(
                        node,
                        command_client,
                        stop_request(),
                        min(args.timeout_s, 2.0),
                    )
                    result["failure_stop_response"] = response_document(stopped)
                    result["coordinated_stop_sent"] = True
                    result["torque_hold_active"] = False
                except Exception as stop_error:
                    result["failure_stop_error"] = (
                        f"{type(stop_error).__name__}: {stop_error}"
                    )
        if result:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
