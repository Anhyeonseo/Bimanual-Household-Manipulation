#!/usr/bin/env python3
"""Run one supervised current->clear->current->clear resident roundtrip.

Only a fresh, SHA-pinned MoveIt plan-only artifact is accepted.  The MoveIt
waypoints are replayed at a conservative host rate; the tool never substitutes
a direct joint-space shortcut.  Two fresh Top images are captured at the two
clear arrivals, then both arms are stopped.  Visual occlusion review remains a
separate operator decision.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from so101_interfaces.srv import BimanualStreamCommand  # noqa: E402

CANONICAL_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
    "left_gripper_joint",
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
    "right_gripper_joint",
)
RAW_STEP_RAD = 2.0 * math.pi / 4096.0

BOTH_ARM_JOINTS = tuple(
    name for name in CANONICAL_JOINTS if not name.endswith("gripper_joint")
)


CONFIRMATION = "RUN_OBSERVE_CLEAR_ROUNDTRIP_ONCE"
OWNER = "observe_clear_roundtrip_operator"
EXPECTED_PLAN_STATUS = "OBSERVE_CLEAR_PLAN_ONLY_PASS"
EXPECTED_FIRMWARES = ("0x00024809",)
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
REFRESH_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
TOP_IMAGE_TOPIC = "/camera/top/image_raw"
MAXIMUM_PLAN_AGE_S = 600.0
START_MATCH_LIMIT_RAD = 0.015
TERMINAL_LIMIT_RAD = 0.05
CLEAR_REPEATABILITY_LIMIT_RAD = 0.03
SAMPLE_PERIOD_MS = 50
FIRST_POINT_MS = 100
COMMAND_RATE_RAD_S = 150.0 * RAW_STEP_RAD


class ObserveClearExecutionError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_plan(path: Path, expected_sha256: str, now: float | None = None) -> dict:
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ObserveClearExecutionError(
            f"plan sha256 mismatch: expected={expected_sha256} actual={actual}"
        )
    plan = json.loads(path.read_text(encoding="utf-8"))
    if (
        plan.get("schema_version") != 1
        or plan.get("record_kind") != "observe_clear_plan_only"
        or plan.get("status") != EXPECTED_PLAN_STATUS
        or plan.get("motion_authorized") is not False
        or plan.get("automatic_execution_permitted") is not False
        or plan.get("execution_api_used") is not False
        or plan.get("motion_commands") != 0
        or plan.get("planning_group") != "both_arms"
        or plan.get("planning_scene", {}).get("collision_object_id")
        != "validated_worktable_region"
        or plan.get("planning_scene", {}).get("apply_planning_scene_success") is not True
        or plan.get("planning_scene", {}).get("temporary_exceptions_restored_before_strict_validation") is not True
        or plan.get("planning_scene", {}).get("strict_unapproved_contact_count") != 0
        or int(plan.get("planning_scene", {}).get("strict_path_sample_count", 0)) <= 1
        or float(plan.get("planning_scene", {}).get("strict_maximum_exception_depth_m", math.inf))
        > float(plan.get("planning_scene", {}).get("strict_maximum_exception_depth_limit_m", 0.0))
        or tuple(plan.get("joint_names", ())) != CANONICAL_JOINTS
        or set(plan.get("arm_joint_names", ())) != set(BOTH_ARM_JOINTS)
        or not plan.get("trajectory")
    ):
        raise ObserveClearExecutionError("OBSERVE_CLEAR plan contract is invalid")
    age = (time.time() if now is None else now) - float(
        plan.get("generated_at_unix_s", 0.0)
    )
    if not 0.0 <= age <= MAXIMUM_PLAN_AGE_S:
        raise ObserveClearExecutionError(f"plan age is outside the 600 s gate: {age:.1f}")
    for key in ("start_positions_rad", "target_positions_rad"):
        values = plan.get(key)
        if (
            not isinstance(values, list)
            or len(values) != 12
            or not all(math.isfinite(float(value)) for value in values)
        ):
            raise ObserveClearExecutionError(f"plan {key} is invalid")
    return plan


def arm_route_as_full_positions(plan: dict, grippers: tuple[float, float]):
    names = tuple(plan["arm_joint_names"])
    route = []
    for item in plan["trajectory"]:
        positions = item.get("positions_rad")
        if (
            not isinstance(positions, list)
            or len(positions) != len(names)
            or not all(math.isfinite(float(value)) for value in positions)
        ):
            raise ObserveClearExecutionError("plan trajectory point is invalid")
        by_name = dict(zip(names, positions, strict=True))
        full = []
        for name in CANONICAL_JOINTS:
            if name == "left_gripper_joint":
                full.append(float(grippers[0]))
            elif name == "right_gripper_joint":
                full.append(float(grippers[1]))
            else:
                full.append(float(by_name[name]))
        route.append(tuple(full))
    return tuple(route)


def trajectory_point(positions, offset_ms: int) -> JointTrajectoryPoint:
    result = JointTrajectoryPoint()
    result.positions = list(positions)
    result.time_from_start.sec = offset_ms // 1000
    result.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return result


def finite_route_request(start, route):
    if not route:
        raise ObserveClearExecutionError("resident route is empty")
    points = []
    previous = tuple(start)
    offset_ms = FIRST_POINT_MS - SAMPLE_PERIOD_MS
    for target in route:
        largest = max(
            abs(float(end) - float(begin))
            for begin, end in zip(previous, target, strict=True)
        )
        count = max(
            1,
            math.ceil(
                largest / (COMMAND_RATE_RAD_S * SAMPLE_PERIOD_MS / 1000.0)
            ),
        )
        for index in range(1, count + 1):
            fraction = index / count
            offset_ms += SAMPLE_PERIOD_MS
            points.append(
                trajectory_point(
                    tuple(
                        begin + (end - begin) * fraction
                        for begin, end in zip(previous, target, strict=True)
                    ),
                    offset_ms,
                )
            )
        previous = tuple(target)
    if len(points) == 1:
        offset_ms += SAMPLE_PERIOD_MS
        points.append(trajectory_point(route[-1], offset_ms))
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


def call(node, client, request, timeout_s):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.exception() is not None:
        raise ObserveClearExecutionError("resident service call failed or timed out")
    return future.result()


def status_document(node, client, timeout_s):
    response = call(node, client, Trigger.Request(), timeout_s)
    if not response.success:
        raise ObserveClearExecutionError(f"status rejected: {response.message}")
    return json.loads(response.message)


def prepared_positions(document: dict) -> tuple[float, ...]:
    values = document.get("prepared_positions_rad")
    if (
        not isinstance(values, list)
        or len(values) != 12
        or not all(math.isfinite(float(value)) for value in values)
    ):
        raise ObserveClearExecutionError("resident status has no complete anchor")
    return tuple(float(value) for value in values)


def wait_ready(node, client, epoch: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    history = []
    while time.monotonic() < deadline:
        document = status_document(node, client, min(timeout_s, 5.0))
        history.append(document)
        if (
            document.get("state") == "ready"
            and document.get("owner") == OWNER
            and int(document.get("arbiter_epoch", -1)) == epoch
            and document.get("torque_hold_active") is True
        ):
            return history, prepared_positions(document)
        if document.get("state") not in ("active", "ready"):
            raise ObserveClearExecutionError(f"unexpected resident state: {document}")
        time.sleep(0.03)
    raise ObserveClearExecutionError(f"timeout waiting for epoch={epoch}")


def save_fresh_top_image(node, images: list[Image], output: Path, timeout_s: float):
    images.clear()
    deadline = time.monotonic() + timeout_s
    while not images and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not images:
        raise ObserveClearExecutionError(f"timeout waiting for {TOP_IMAGE_TOPIC}")
    message = images[-1]
    if (
        message.encoding != "rgb8"
        or int(message.width) != 1280
        or int(message.height) != 960
        or int(message.step) != 3840
    ):
        raise ObserveClearExecutionError(
            f"unexpected Top image contract: {message.width}x{message.height} {message.encoding}"
        )
    rgb = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(960, 1280, 3)
    if not cv2.imwrite(str(output), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise ObserveClearExecutionError(f"failed to save Top image: {output}")
    return {
        "path": str(output.resolve()),
        "sha256": sha256_file(output),
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--image-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error(
            f"confirmation mismatch; use {CONFIRMATION} only with both workspaces clear"
        )
    if args.output.exists():
        parser.error(f"refusing to overwrite artifact: {args.output}")
    if args.image_directory.exists() and any(args.image_directory.iterdir()):
        parser.error("image directory must be absent or empty")
    if not 1.0 <= args.settle_s <= 10.0:
        parser.error("settle must be within 1..10 s")
    return args


def main() -> int:
    args = parse_args()
    plan = load_plan(args.plan, args.plan_sha256)
    args.image_directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("observe_clear_roundtrip_operator")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    images: list[Image] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
    # camera_manager publishes image_raw with SensorDataQoS (best effort).
    node.create_subscription(
        Image,
        TOP_IMAGE_TOPIC,
        images.append,
        qos_profile_sensor_data,
    )
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)
    refresh_client = node.create_client(Trigger, REFRESH_SERVICE)
    motion_started = False
    stopped = False
    result = {"legs": [], "images": []}
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
            (REFRESH_SERVICE, refresh_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise ObserveClearExecutionError(f"service unavailable: {name}")
        initial = status_document(node, status_client, args.timeout_s)
        if (
            initial.get("state") != "ready"
            or initial.get("owner") is not None
            or int(initial.get("arbiter_epoch", -1)) != 0
            or initial.get("motion_authorized") is not True
            or initial.get("fault_diagnostic") is not None
            or initial.get("firmware_version") not in EXPECTED_FIRMWARES
            or initial.get("torque_hold_active") is not False
        ):
            raise ObserveClearExecutionError(f"unexpected initial status: {initial}")
        refreshed = call(node, refresh_client, Trigger.Request(), args.timeout_s)
        if not refreshed.success:
            raise ObserveClearExecutionError(f"anchor refresh failed: {refreshed.message}")
        refresh_document = json.loads(refreshed.message)
        live_start = prepared_positions(refresh_document)
        planned_start = tuple(float(value) for value in plan["start_positions_rad"])
        start_error = max(
            abs(live_start[index] - planned_start[index])
            for index, name in enumerate(CANONICAL_JOINTS)
            if name in BOTH_ARM_JOINTS
        )
        if start_error > START_MATCH_LIMIT_RAD:
            raise ObserveClearExecutionError(
                f"live start no longer matches the plan: {start_error:.6f} rad"
            )
        route = arm_route_as_full_positions(
            plan, (live_start[5], live_start[11])
        )
        routes = (
            route,
            tuple(reversed(route[:-1])) + (live_start,),
            route,
        )
        expected_positions = (
            tuple(float(value) for value in plan["target_positions_rad"]),
            live_start,
            tuple(float(value) for value in plan["target_positions_rad"]),
        )
        measured_clear = []
        commanded = live_start
        for index, (leg_route, expected) in enumerate(
            zip(routes, expected_positions, strict=True), start=1
        ):
            request = finite_route_request(commanded, leg_route)
            motion_started = True
            response = call(node, command_client, request, args.timeout_s)
            if (
                not response.accepted
                or response.adapter_state != "active"
                or int(response.arbiter_epoch) != index
            ):
                raise ObserveClearExecutionError(f"roundtrip leg {index} rejected: {response}")
            duration = request.points[-1].time_from_start.sec + (
                request.points[-1].time_from_start.nanosec / 1e9
            )
            history, commanded = wait_ready(
                node, status_client, index, max(args.timeout_s, duration + 8.0)
            )
            residual = max(
                abs(commanded[joint_index] - expected[joint_index])
                for joint_index, name in enumerate(CANONICAL_JOINTS)
                if name in BOTH_ARM_JOINTS
            )
            if residual > TERMINAL_LIMIT_RAD:
                raise ObserveClearExecutionError(
                    f"roundtrip leg {index} terminal residual {residual:.6f} rad"
                )
            result["legs"].append({
                "index": index,
                "epoch": index,
                "resident_point_count": len(request.points),
                "duration_s": duration,
                "terminal_positions_rad": list(commanded),
                "maximum_terminal_residual_rad": residual,
                "status_samples": len(history),
            })
            if index in (1, 3):
                time.sleep(args.settle_s)
                result["images"].append(
                    save_fresh_top_image(
                        node,
                        images,
                        args.image_directory / f"observe_clear_arrival_{1 if index == 1 else 2}.png",
                        args.timeout_s,
                    )
                )
                measured_clear.append(commanded)
                print(f"OBSERVE_CLEAR_ARRIVAL_CAPTURED arrival={len(measured_clear)}/2 epoch={index}")
        repeatability = max(
            abs(measured_clear[1][index] - measured_clear[0][index])
            for index, name in enumerate(CANONICAL_JOINTS)
            if name in BOTH_ARM_JOINTS
        )
        if repeatability > CLEAR_REPEATABILITY_LIMIT_RAD:
            raise ObserveClearExecutionError(
                f"OBSERVE_CLEAR repeatability {repeatability:.6f} rad exceeds limit"
            )
        stop = call(node, command_client, stop_request(), args.timeout_s)
        if not stop.accepted or stop.adapter_state != "stopped":
            raise ObserveClearExecutionError(f"coordinated STOP rejected: {stop}")
        stopped = True
        final = status_document(node, status_client, args.timeout_s)
        result.update({
            "schema_version": 1,
            "record_kind": "observe_clear_roundtrip_once",
            "status": "OBSERVE_CLEAR_ROUNDTRIP_CAPTURED_AWAITING_VISUAL_REVIEW",
            "motion_authorized": False,
            "automatic_retry_count": 0,
            "operator_confirmation": args.confirmation,
            "plan": {"path": str(args.plan.resolve()), "sha256": args.plan_sha256.lower()},
            "initial_status": initial,
            "live_start_positions_rad": list(live_start),
            "maximum_plan_start_error_rad": start_error,
            "clear_repeatability_max_rad": repeatability,
            "coordinated_stop_verified": True,
            "final_status": final,
            "visual_towel_occlusion_reviewed": False,
        })
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(
            "OBSERVE_CLEAR_ROUNDTRIP_CAPTURED_AWAITING_VISUAL_REVIEW "
            f"repeatability_rad={repeatability:.6f} state={final.get('state')} "
            f"output={args.output} sha256={sha256_file(args.output)}"
        )
        return 0
    finally:
        if motion_started and not stopped:
            try:
                call(node, command_client, stop_request(), min(args.timeout_s, 2.0))
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
