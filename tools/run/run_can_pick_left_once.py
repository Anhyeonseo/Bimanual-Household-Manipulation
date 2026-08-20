#!/usr/bin/env python3
"""왼팔 캔 파지 계획을 resident 로 한 번 실행한다.

세 가지 모드가 있고 이 순서로만 승격한다.

1. `--validate-only`
   resident service client 를 **만들지도 부르지도 않는다.** 계획 SHA 와 계약만
   검사하고 `motion_commands=0 resident_services_called=0` 을 기록한다.

2. `--open-grasp-height-check`
   조를 열고 grasp 자세까지 내려가 **닫지 않고** 멈춘 뒤 pregrasp 를 거쳐 q0 로
   돌아온다. 캔에 닿기 전에 파지 높이와 개방 폭을 눈으로 확인하는 단계다.
   실제 파지 전에 높이와 개방 폭을 확인하는 commissioning 단계다.

3. 전체 실행
   위 두 단계를 통과한 뒤에만 한다.

jaw gap, contact, release 값은 계획에 고정된 캔 전용 계약값만 사용한다.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState  # noqa: E402
from so101_interfaces.msg import BimanualJointFeedback  # noqa: E402
from so101_interfaces.srv import BimanualStreamCommand  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from tools.lib.desk_task_runtime import (  # noqa: E402
    ARM_JOINTS_BY_SIDE,
    ARM_TERMINAL_TOLERANCE_RAD,
    BIMANUAL_ARM_INDICES,
    CANONICAL_JOINTS,
    RAW_STEP_RAD,
    bimanual_q0_target,
    residual_raw,
    sha256_file,
    step_target,
    validate_bimanual_q0,
)

OWNER = "can_pick_left_application"
CONFIRMATION = "RUN_CAN_PICK_LEFT_ONCE"
HEIGHT_CHECK_CONFIRMATION = "RUN_CAN_OPEN_GRASP_HEIGHT_CHECK_ONCE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"
EXPECTED_PLAN_STATUS = "CAN_PICK_LEFT_PLAN_ONLY_PASS"
EXPECTED_FIRMWARES = ("0x00024809",)
SIDE = "left"
GRIPPER_INDEX = 5
ARM_INDICES = tuple(range(5))
MAXIMUM_PLAN_AGE_S = 300.0
CONTINUOUS_SAMPLE_PERIOD_MS = 50
CONTINUOUS_FIRST_POINT_MS = 80
CONTINUOUS_COMMAND_RATE_RAD_S = 200.0 * RAW_STEP_RAD
REQUIRED_ENDPOINTS = frozenset(("pick_pregrasp", "pick_grasp", "pick_lift"))


class CanExecutionError(RuntimeError):
    """실행 전제가 성립하지 않는다."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--open-grasp-height-check",
        action="store_true",
        help="조를 열고 grasp 까지 내려가되 닫지 않는다",
    )
    parser.add_argument("--confirmation", default="")
    parser.add_argument("--hold-at-grasp-s", type=float, default=6.0)
    parser.add_argument("--timeout-s", type=float, default=25.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.validate_only and args.open_grasp_height_check:
        parser.error("--validate-only and --open-grasp-height-check are exclusive")
    if not args.validate_only:
        expected = (
            HEIGHT_CHECK_CONFIRMATION
            if args.open_grasp_height_check
            else CONFIRMATION
        )
        if args.confirmation != expected:
            parser.error(
                f"motion requires --confirmation {expected}; support both arms "
                "and clear the workspace first"
            )
    if not 0.0 <= args.hold_at_grasp_s <= 60.0:
        parser.error("hold at grasp must be within 0..60 s")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing artifact: {args.output}")
    return args


def load_can_plan(path: Path, expected_sha256: str) -> dict:
    """계획을 SHA 로 고정하고 캔 계약을 전부 검사한다.

    여기서 걸러야 하는 것은 **개방 안 된 그리퍼로 캔에 내려가는 계획**과
    **roll 을 안 푼 계획**이다. 둘 다 통과하면 하드웨어에서만 드러난다.
    """
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise CanExecutionError(
            f"plan sha256 mismatch: expected={expected_sha256.lower()} "
            f"actual={actual}"
        )
    plan = json.loads(path.read_text(encoding="utf-8"))
    if (
        plan.get("schema_version") != 1
        or plan.get("record_kind") != "can_pick_left_plan_only"
        or plan.get("status") != EXPECTED_PLAN_STATUS
        or plan.get("execution_api_used") is not False
        or plan.get("motion_authorized") is not False
        or plan.get("automatic_execution_permitted") is not False
        or plan.get("selected_arm") != SIDE
        or tuple(plan.get("joint_names", ())) != ARM_JOINTS_BY_SIDE[SIDE]
        or not isinstance(plan.get("steps"), list)
        or not plan["steps"]
    ):
        raise CanExecutionError("can pick plan contract is invalid")

    age_s = time.time() - float(plan.get("generated_at_unix_s", 0.0))
    if not 0.0 <= age_s <= MAXIMUM_PLAN_AGE_S:
        raise CanExecutionError(
            f"plan age {age_s:.1f} s is outside 0..{MAXIMUM_PLAN_AGE_S:.0f} s; "
            "make a fresh plan"
        )

    endpoints = plan.get("endpoints")
    if not isinstance(endpoints, dict) or set(endpoints) != REQUIRED_ENDPOINTS:
        raise CanExecutionError("can pick plan endpoint set is invalid")

    gripper = plan.get("gripper_contract", {})
    for name in (
        "open_gap_mm",
        "open_command_rad",
        "grasp_gap_mm",
        "grasp_command_rad",
        "contact_threshold_raw",
        "release_tolerance_raw",
    ):
        if gripper.get(name) is None:
            raise CanExecutionError(
                f"plan gripper contract is not commissioned: {name} is null"
            )
    if gripper.get("provenance") in (None, "", "not_measured"):
        raise CanExecutionError(
            "plan gripper contract has no measurement provenance"
        )
    if float(gripper["open_gap_mm"]) <= float(gripper["grasp_gap_mm"]):
        raise CanExecutionError("open gap must exceed the grasp gap")
    if float(gripper["open_gap_mm"]) < float(
        gripper["minimum_open_gap_for_tolerance_mm"]
    ):
        raise CanExecutionError(
            "open gap does not cover the declared crossing tolerance"
        )
    # 개방은 raw 가 커지는 쪽 = semantic rad 가 작아지는 쪽이다. 이 부호가
    # 뒤집힌 계획은 조를 닫으면서 캔으로 내려간다.
    if float(gripper["open_command_rad"]) >= float(
        gripper["grasp_command_rad"]
    ):
        raise CanExecutionError(
            "open command must be more negative than the grasp command; "
            "the opening direction in this plan is inverted"
        )

    limits = plan.get("acceptance_limits", {})
    grasp_endpoint = endpoints["pick_grasp"]
    crossing = float(grasp_endpoint.get("crossing_error_rad", math.inf))
    tolerance = float(limits.get("crossing_tolerance_rad", 0.0))
    if not 0.0 < tolerance or crossing > tolerance:
        raise CanExecutionError(
            f"grasp crossing error {math.degrees(crossing):.3f} deg exceeds "
            f"the tolerance {math.degrees(tolerance):.3f} deg"
        )
    if grasp_endpoint.get("wrist_roll_policy") != (
        "nearest_in_limit_branch_then_joint_position_crossing_solve"
    ):
        raise CanExecutionError(
            "grasp endpoint did not use the in-limit nearest-branch roll solver"
        )

    descent = plan.get("descent_check", {})
    if (
        descent.get("vertical_only") is not True
        or float(descent.get("wrist_roll_span_rad", math.inf)) > 1.0e-9
        or float(descent.get("lateral_travel_m", math.inf))
        > float(limits.get("position_tolerance_m", 0.0))
    ):
        raise CanExecutionError("plan descent is not vertical with a locked roll")

    kinds = [step.get("kind") for step in plan["steps"]]
    if kinds.count("gripper") != 2 or kinds[0] != "gripper":
        raise CanExecutionError(
            "can pick needs exactly one open before approach and one close"
        )
    return plan


def can_actions(plan: dict, *, height_check: bool) -> list[dict]:
    """계획 단계를 연속 leg 로 묶는다.

    place가 없으므로 gripper/arm/gripper/arm 네 구간만 허용한다.
    `height_check` 면 닫기 이후를 통째로 버리고 pregrasp 복귀로 대체하므로
    여기서는 닫기 직전까지만 낸다.
    """
    actions: list[dict] = []
    arm_steps: list[dict] = []

    def flush() -> None:
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
        flush()
        actions.append(
            {
                "kind": "gripper",
                "label": str(step["phase"]),
                "steps": [step],
            }
        )
    flush()

    labels = [action["label"] for action in actions]
    kinds = [action["kind"] for action in actions]
    if (
        kinds != ["gripper", "arm_route", "gripper", "arm_route"]
        or labels[0] != "pick_open"
        or labels[2] != "pick_close"
    ):
        raise CanExecutionError(f"unexpected can action partition: {labels}")
    if height_check:
        # 캔을 물지 않는다. 개방과 하강까지만 낸다.
        return actions[:2]
    return actions


def call(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise CanExecutionError("service response timeout")
    error = future.exception()
    if error is not None:
        raise CanExecutionError(f"service call failed: {error}") from error
    return future.result()


def status_document(node: Node, client, timeout_s: float) -> dict:
    response = call(node, client, Trigger.Request(), timeout_s)
    if not response.success:
        raise CanExecutionError(f"status service rejected: {response.message}")
    document = json.loads(response.message)
    if not isinstance(document, dict):
        raise CanExecutionError("status response is not an object")
    return document


def prepared_positions(
    document: dict,
    *,
    label: str,
    expected_epoch: int,
    require_torque_hold: bool,
) -> tuple[float, ...]:
    if int(document.get("prepared_epoch", -1)) != expected_epoch:
        raise CanExecutionError(f"{label} prepared epoch mismatch")
    if require_torque_hold and document.get("torque_hold_active") is not True:
        raise CanExecutionError(f"{label} status does not prove torque hold")
    values = document.get("prepared_positions_rad")
    if (
        not isinstance(values, list)
        or len(values) != 12
        or not all(math.isfinite(float(value)) for value in values)
    ):
        raise CanExecutionError(f"{label} status has no complete prepared anchor")
    return tuple(float(value) for value in values)


def trajectory_point(positions, offset_ms: int) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = offset_ms // 1000
    point.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return point


def continuous_finite_request(start, targets):
    """관절 속도를 200 raw/s 로 제한한 연속 finite route 를 만든다."""
    if not targets:
        raise CanExecutionError("continuous finite route requires a target")
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
            points.append(
                trajectory_point(
                    tuple(
                        begin + (end - begin) * fraction
                        for begin, end in zip(
                            segment_start, segment_target, strict=True
                        )
                    ),
                    offset_ms + CONTINUOUS_SAMPLE_PERIOD_MS * sample_index,
                )
            )
        offset_ms += CONTINUOUS_SAMPLE_PERIOD_MS * sample_count
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


def stop_request():
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.STOP
    request.owner = OWNER
    return request


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
    *,
    epoch: int,
    timeout_s: float,
) -> tuple[list[dict], tuple[float, ...]]:
    deadline = time.monotonic() + timeout_s
    history: list[dict] = []
    while time.monotonic() < deadline:
        document = status_document(node, status_client, timeout_s)
        history.append(document)
        if (
            document.get("state") == "ready"
            and document.get("owner") == OWNER
            and document.get("arbiter_epoch") == epoch
        ):
            measured = prepared_positions(
                document,
                label=f"epoch {epoch} terminal",
                expected_epoch=epoch,
                require_torque_hold=True,
            )
            return history, measured
        if document.get("state") not in ("active", "ready"):
            raise CanExecutionError(f"unexpected resident state: {document}")
        time.sleep(0.02)
    raise CanExecutionError(
        f"timeout waiting for finite epoch={epoch}: {history[-1:]}"
    )


def leg_duration_s(request) -> float:
    last = request.points[-1].time_from_start
    return last.sec + last.nanosec / 1e9


def validate_only_document(plan: dict, plan_path: Path, digest: str) -> dict:
    gripper = plan["gripper_contract"]
    grasp = plan["endpoints"]["pick_grasp"]
    return {
        "schema_version": 1,
        "record_kind": "can_pick_left_validate_only",
        "status": "CAN_PICK_LEFT_VALIDATE_ONLY_PASS",
        "plan": {"path": str(plan_path), "sha256": digest},
        "selected_arm": SIDE,
        "motion_commands": 0,
        "resident_services_called": 0,
        "resident_clients_created": 0,
        "execution_api_used": False,
        "automatic_retry_count": 0,
        "checked": {
            "plan_schema": True,
            "plan_sha256": True,
            "plan_age": True,
            "gripper_commissioned": True,
            "open_clears_the_can": True,
            "opening_direction": True,
            "crossing_within_tolerance": True,
            "roll_branch_policy": True,
            "descent_vertical_with_locked_roll": True,
        },
        "evidence": {
            "can_axis_yaw_deg": math.degrees(
                float(plan["target_lock"]["can_axis_yaw_rad"])
            ),
            "wrist_roll_deg": math.degrees(float(grasp["wrist_roll_rad"])),
            "wrist_roll_rotation_from_q0_deg": math.degrees(
                float(grasp["wrist_roll_rotation_from_q0_rad"])
            ),
            "wrist_roll_branch_index": grasp["wrist_roll_branch_index"],
            "wrist_roll_branch_count": grasp["wrist_roll_branch_count"],
            "crossing_error_deg": math.degrees(
                float(grasp["crossing_error_rad"])
            ),
            "approach_tilt_deg": grasp["approach_tilt_from_vertical_deg"],
            "open_gap_mm": gripper["open_gap_mm"],
            "grasp_gap_mm": gripper["grasp_gap_mm"],
            "minimum_open_gap_for_tolerance_mm": gripper[
                "minimum_open_gap_for_tolerance_mm"
            ],
            "required_jaw_width_at_achieved_error_mm": gripper[
                "required_jaw_width_at_achieved_error_mm"
            ],
        },
    }


def main() -> int:
    args = parse_args()
    plan = load_can_plan(args.plan, args.plan_sha256)

    if args.validate_only:
        document = validate_only_document(
            plan, args.plan, args.plan_sha256.lower()
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        evidence = document["evidence"]
        print(
            "CAN_PICK_LEFT_VALIDATE_ONLY_PASS motion_commands=0 "
            "resident_services_called=0 "
            f"can_yaw_deg={evidence['can_axis_yaw_deg']:+.2f} "
            f"roll_deg={evidence['wrist_roll_deg']:+.2f} "
            f"rotation_deg={evidence['wrist_roll_rotation_from_q0_deg']:+.2f} "
            f"branch={evidence['wrist_roll_branch_index']}"
            f"/{evidence['wrist_roll_branch_count']} "
            f"crossing_deg={evidence['crossing_error_deg']:.3f} "
            f"open_mm={evidence['open_gap_mm']:.1f} "
            f"needs_mm={evidence['required_jaw_width_at_achieved_error_mm']:.1f} "
            f"output={args.output}"
        )
        return 0

    gripper_contract = plan["gripper_contract"]
    contact_threshold_raw = int(gripper_contract["contact_threshold_raw"])
    release_tolerance_raw = int(gripper_contract["release_tolerance_raw"])
    actions = can_actions(plan, height_check=args.open_grasp_height_check)

    rclpy.init()
    node = Node("can_pick_left_application")
    anchor_qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    anchors: list[JointState] = []
    feedback_messages: list[BimanualJointFeedback] = []
    node.create_subscription(
        JointState, ANCHOR_TOPIC, anchors.append, anchor_qos
    )
    node.create_subscription(
        BimanualJointFeedback, FEEDBACK_TOPIC, feedback_messages.append, 10
    )
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)
    refresh_client = node.create_client(Trigger, REFRESH_ANCHOR_SERVICE)

    result: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "can_pick_left_execute_once",
        "selected_arm": SIDE,
        "mode": (
            "open_grasp_height_check"
            if args.open_grasp_height_check
            else "full_pick"
        ),
        "operator_confirmation": args.confirmation,
        "plan": {"path": str(args.plan), "sha256": args.plan_sha256.lower()},
        "automatic_retry_count": 0,
        "legs": [],
        "homing": {},
    }
    motion_commands = 0
    resident_ready_for_hold = False
    started = False
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
            (REFRESH_ANCHOR_SERVICE, refresh_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise CanExecutionError(f"service unavailable: {name}")

        initial = status_document(node, status_client, args.timeout_s)
        initial_owner = initial.get("owner")
        epoch = int(initial.get("arbiter_epoch", -1))
        if (
            initial.get("state") != "ready"
            or initial_owner not in (None, OWNER)
            or (initial_owner is None and epoch != 0)
            or initial.get("motion_authorized") is not True
            or initial.get("fault_diagnostic") is not None
            or initial.get("firmware_version") not in EXPECTED_FIRMWARES
        ):
            raise CanExecutionError(f"unexpected resident initial state: {initial}")
        result["initial_status"] = initial

        if initial_owner is None:
            response = call(
                node, refresh_client, Trigger.Request(), args.timeout_s
            )
            if not response.success:
                raise CanExecutionError(f"anchor refresh failed: {response}")
            refreshed = json.loads(response.message)
            source = (
                refreshed if "prepared_positions_rad" in refreshed else initial
            )
            commanded = prepared_positions(
                source,
                label="startup",
                expected_epoch=epoch,
                require_torque_hold=False,
            )
        else:
            commanded = prepared_positions(
                initial,
                label="resumed",
                expected_epoch=epoch,
                require_torque_hold=True,
            )

        # 1) 양팔 q0 로 모은다. 반대 팔은 이후 내내 여기서 고정된다.
        q0_target = bimanual_q0_target(commanded)
        q0_request = continuous_finite_request(commanded, [q0_target])
        started = True
        motion_commands += 1
        response = call(node, command_client, q0_request, args.timeout_s)
        expected_epoch = epoch + 1
        if (
            not response.accepted
            or response.adapter_state != "active"
            or int(response.arbiter_epoch) != expected_epoch
        ):
            raise CanExecutionError(f"bimanual q0 rejected: {response}")
        resident_ready_for_hold = False
        history, measured = wait_until_ready(
            node,
            status_client,
            epoch=expected_epoch,
            timeout_s=max(args.timeout_s, leg_duration_s(q0_request) + 5.0),
        )
        resident_ready_for_hold = True
        q0_residual = validate_bimanual_q0(measured)
        result["homing"] = {
            "epoch": expected_epoch,
            "terminal_positions_rad": list(measured),
            "maximum_final_residual_rad": q0_residual,
            "status_samples": len(history),
        }
        print(
            "CAN_PICK_LEFT_BIMANUAL_Q0_HOLD_PASS "
            f"maximum_residual_rad={q0_residual:.6f} epoch={expected_epoch}"
        )
        epoch = expected_epoch
        commanded = measured
        opposite_hold = measured[6:]

        # 2) 계획 leg 를 순서대로 낸다. 자동 재시도는 없다.
        for action_index, action in enumerate(actions, start=1):
            targets: list[tuple[float, ...]] = []
            planned = commanded
            for step in action["steps"]:
                planned = step_target(planned, step, opposite_hold, arm=SIDE)
                targets.append(planned)
            request = continuous_finite_request(commanded, targets)
            motion_commands += 1
            resident_ready_for_hold = False
            response = call(node, command_client, request, args.timeout_s)
            expected_epoch = epoch + 1
            if (
                not response.accepted
                or response.adapter_state != "active"
                or int(response.arbiter_epoch) != expected_epoch
            ):
                raise CanExecutionError(
                    f"action {action_index} ({action['label']}) rejected: "
                    f"{response_document(response)}"
                )
            history, measured = wait_until_ready(
                node,
                status_client,
                epoch=expected_epoch,
                timeout_s=max(args.timeout_s, leg_duration_s(request) + 5.0),
            )
            resident_ready_for_hold = True

            terminal_error = max(
                abs(measured[index] - targets[-1][index])
                for index in ARM_INDICES
            )
            if (
                action["kind"] == "arm_route"
                and terminal_error > ARM_TERMINAL_TOLERANCE_RAD
            ):
                raise CanExecutionError(
                    f"arm terminal error action={action_index} "
                    f"label={action['label']} error={terminal_error:.6f} rad"
                )
            gripper_gap_raw = None
            if action["label"] == "pick_open":
                gripper_gap_raw = residual_raw(
                    targets[-1][GRIPPER_INDEX], measured[GRIPPER_INDEX]
                )
                if gripper_gap_raw > release_tolerance_raw:
                    raise CanExecutionError(
                        "gripper did not reach the commanded opening: "
                        f"residual_raw={gripper_gap_raw} exceeds "
                        f"{release_tolerance_raw}"
                    )
            if action["label"] == "pick_close":
                gripper_gap_raw = residual_raw(
                    targets[-1][GRIPPER_INDEX], measured[GRIPPER_INDEX]
                )
                if gripper_gap_raw < contact_threshold_raw:
                    raise CanExecutionError(
                        "can contact not detected: "
                        f"residual_raw={gripper_gap_raw} is below the "
                        f"commissioned threshold {contact_threshold_raw}"
                    )
            result["legs"].append(
                {
                    "action_index": action_index,
                    "action_count": len(actions),
                    "kind": action["kind"],
                    "label": action["label"],
                    "source_step_indices": [
                        int(step["index"]) for step in action["steps"]
                    ],
                    "epoch": expected_epoch,
                    "start_response": response_document(response),
                    "trajectory_points": len(request.points),
                    "duration_ms": round(leg_duration_s(request) * 1000.0),
                    "terminal_positions_rad": list(measured),
                    "arm_terminal_error_rad": terminal_error,
                    "gripper_residual_raw": gripper_gap_raw,
                    "status_samples": len(history),
                }
            )
            print(
                "CAN_PICK_LEFT_ACTION_PASS "
                f"action={action_index}/{len(actions)} "
                f"label={action['label']} epoch={expected_epoch} "
                f"arm_error_mrad={terminal_error * 1000.0:.3f} "
                f"gripper_residual_raw={gripper_gap_raw}"
            )
            epoch = expected_epoch
            commanded = measured

            if args.open_grasp_height_check and action_index == len(actions):
                # 캔에 닿지 않은 채 파지 자세를 보여준 뒤 되돌아간다.
                print(
                    "CAN_OPEN_GRASP_HEIGHT_CHECK_HOLD "
                    f"seconds={args.hold_at_grasp_s:.1f} close_commands=0"
                )
                time.sleep(args.hold_at_grasp_s)
                pregrasp_target = step_target(
                    commanded,
                    {
                        "kind": "arm",
                        "target_positions_rad": plan["endpoints"][
                            "pick_pregrasp"
                        ]["final_joint_positions_rad"],
                    },
                    opposite_hold,
                    arm=SIDE,
                )
                return_request = continuous_finite_request(
                    commanded,
                    [pregrasp_target, bimanual_q0_target(pregrasp_target)],
                )
                motion_commands += 1
                resident_ready_for_hold = False
                response = call(
                    node, command_client, return_request, args.timeout_s
                )
                expected_epoch = epoch + 1
                if (
                    not response.accepted
                    or response.adapter_state != "active"
                    or int(response.arbiter_epoch) != expected_epoch
                ):
                    raise CanExecutionError(
                        f"height-check return rejected: "
                        f"{response_document(response)}"
                    )
                history, measured = wait_until_ready(
                    node,
                    status_client,
                    epoch=expected_epoch,
                    timeout_s=max(
                        args.timeout_s, leg_duration_s(return_request) + 5.0
                    ),
                )
                resident_ready_for_hold = True
                epoch = expected_epoch
                commanded = measured
                result["legs"].append(
                    {
                        "action_index": len(actions) + 1,
                        "kind": "arm_route",
                        "label": "height_check_return_to_q0",
                        "epoch": expected_epoch,
                        "terminal_positions_rad": list(measured),
                        "status_samples": len(history),
                    }
                )
                print(
                    "CAN_OPEN_GRASP_HEIGHT_CHECK_RETURN_PASS "
                    f"epoch={expected_epoch} close_commands=0"
                )

        result["status"] = (
            "CAN_OPEN_GRASP_HEIGHT_CHECK_PASS"
            if args.open_grasp_height_check
            else "CAN_PICK_LEFT_EXECUTE_PASS"
        )
        result["motion_commands"] = motion_commands
        result["terminal_state"] = "ready_torque_hold"
        result["stop_sent"] = False
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = sha256(args.output.read_bytes()).hexdigest()
        print(
            f"{result['status']} arm={SIDE} "
            f"legs={len(result['legs'])} motion_commands={motion_commands} "
            f"automatic_retry_count=0 terminal=ready_torque_hold "
            f"output={args.output} sha256={digest}"
        )
        return 0
    except Exception as error:
        result["status"] = "CAN_PICK_LEFT_FAILED"
        result["failure"] = f"{type(error).__name__}: {error}"
        result["motion_commands"] = motion_commands
        # 정상 완료한 leg 뒤의 판정 실패는 현재 자세 torque hold 를 보존한다.
        # 팔이 중력으로 넘어지지 않게 하기 위해서다. resident 가 ready 가
        # 아니면 그때만 STOP 을 낸다.
        if started and not resident_ready_for_hold:
            try:
                call(node, command_client, stop_request(), min(args.timeout_s, 3.0))
                result["stop_sent"] = True
            except Exception:  # noqa: BLE001
                result["stop_sent"] = False
        else:
            result["stop_sent"] = False
        result["terminal_state"] = (
            "ready_torque_hold" if resident_ready_for_hold else "stopped_or_unknown"
        )
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except Exception:  # noqa: BLE001
            pass
        print(
            f"CAN_PICK_LEFT_FAILED {result['failure']} "
            f"motion_commands={motion_commands} "
            f"stop_sent={str(result['stop_sent']).lower()} "
            f"terminal={result['terminal_state']} "
            f"output={args.output}"
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
