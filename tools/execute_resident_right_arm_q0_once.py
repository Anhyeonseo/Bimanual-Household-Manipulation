#!/usr/bin/env python3
"""Move the right arm five-axis chain to q0 through bounded resident finite legs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES

from so101_interfaces.msg import BimanualJointFeedback
from so101_interfaces.srv import BimanualStreamCommand

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectoryPoint


CONFIRMATION = "RESIDENT_RIGHT_ARM_Q0_ONCE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"
OWNER = "resident_right_arm_q0_operator"
RIGHT_ARM_INDICES = (6, 7, 8, 9, 10)
Q0_RAD = 0.0
Q0_TOLERANCE_RAD = 0.03
MAXIMUM_SUBLEG_DELTA_RAD = 0.075
POINT_OFFSETS_MS = (100, 200, 300, 400)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root
            / "artifacts/resident_adapter/2026-08-14/right_arm_q0_once_run01.json"
        ),
    )
    return parser.parse_args()


def call(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}") from error
    return future.result()


def wait_for_anchor(
    node: Node,
    storage: list[JointState],
    timeout_s: float,
) -> JointState:
    deadline = time.monotonic() + timeout_s
    while not storage and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not storage:
        raise RuntimeError(f"timeout waiting for {ANCHOR_TOPIC}")
    return storage[-1]


def wait_for_q0_feedback(
    node: Node,
    storage: list[BimanualJointFeedback],
    timeout_s: float,
) -> BimanualJointFeedback:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not storage:
            continue
        feedback = storage[-1]
        if (
            tuple(feedback.joint_names) == CANONICAL_JOINT_NAMES
            and int(feedback.present_mask) == 0x0FFF
            and max(int(age) for age in feedback.sample_age_ms) <= 150
            and all(math.isfinite(float(value)) for value in feedback.positions)
        ):
            return feedback
    raise RuntimeError(f"timeout waiting for fresh {FEEDBACK_TOPIC}")


def status_document(node: Node, client, timeout_s: float) -> dict:
    response = call(node, client, Trigger.Request(), timeout_s)
    document = json.loads(response.message)
    if not response.success or document.get("state") == "faulted":
        raise RuntimeError(f"resident adapter is unhealthy: {response}")
    return document


def wait_until_ready(
    node: Node,
    client,
    *,
    owner: str,
    epoch: int,
    timeout_s: float,
) -> list[dict]:
    deadline = time.monotonic() + timeout_s
    history = []
    while time.monotonic() < deadline:
        document = status_document(node, client, timeout_s)
        history.append(document)
        if (
            document.get("state") == "ready"
            and document.get("owner") == owner
            and document.get("arbiter_epoch") == epoch
        ):
            return history
        if document.get("state") not in ("active", "ready"):
            raise RuntimeError(f"unexpected resident state: {document}")
        time.sleep(0.05)
    raise RuntimeError(
        f"timeout waiting for finite leg {epoch} completion: {history[-1:]}"
    )


def point(positions: tuple[float, ...], offset_ms: int):
    result = JointTrajectoryPoint()
    result.positions = list(positions)
    result.time_from_start.sec = offset_ms // 1000
    result.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return result


def finite_request(
    start_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
):
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.START_FINITE
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINT_NAMES)
    request.points = []
    for offset, fraction in zip(
        POINT_OFFSETS_MS,
        (0.25, 0.5, 0.75, 1.0),
        strict=True,
    ):
        positions = tuple(
            start + ((target - start) * fraction)
            for start, target in zip(
                start_positions,
                target_positions,
                strict=True,
            )
        )
        request.points.append(point(positions, offset))
    return request


def response_document(response) -> dict:
    return {
        "accepted": bool(response.accepted),
        "adapter_state": response.adapter_state,
        "arbiter_epoch": int(response.arbiter_epoch),
        "diagnostic": response.diagnostic,
    }


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; support both arms and clear their workspace"
        )
    if args.timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be positive")

    rclpy.init()
    node = Node("resident_right_arm_q0_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    feedback_messages: list[BimanualJointFeedback] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
    node.create_subscription(
        BimanualJointFeedback,
        FEEDBACK_TOPIC,
        feedback_messages.append,
        20,
    )
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(
        BimanualStreamCommand,
        COMMAND_SERVICE,
    )
    motion_request_sent = False
    stop_accepted = False
    try:
        for service_name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {service_name}")

        anchor = wait_for_anchor(node, anchors, args.timeout_s)
        positions = tuple(anchor.position)
        if (
            tuple(anchor.name) != CANONICAL_JOINT_NAMES
            or len(positions) != 12
            or not all(math.isfinite(value) for value in positions)
        ):
            raise RuntimeError(f"invalid resident anchor: {anchor}")

        initial = status_document(node, status_client, args.timeout_s)
        if (
            initial.get("state") != "ready"
            or initial.get("owner") is not None
            or initial.get("arbiter_epoch") != 0
            or initial.get("motion_authorized") is not True
            or initial.get("firmware_version")
            not in ("0x00024806", "0x00024807")
        ):
            raise RuntimeError(f"unexpected initial resident state: {initial}")

        target = list(positions)
        for index in RIGHT_ARM_INDICES:
            target[index] = Q0_RAD
        target_positions = tuple(target)
        maximum_initial_residual = max(
            abs(positions[index] - Q0_RAD) for index in RIGHT_ARM_INDICES
        )
        subleg_count = max(
            1, math.ceil(maximum_initial_residual / MAXIMUM_SUBLEG_DELTA_RAD)
        )
        subtargets = tuple(
            target_positions
            if index == subleg_count
            else tuple(
                start + ((end - start) * index / subleg_count)
                for start, end in zip(positions, target_positions, strict=True)
            )
            for index in range(1, subleg_count + 1)
        )
        print(
            "RESIDENT_RIGHT_ARM_Q0_PRECHECK_PASS "
            f"maximum_initial_residual_rad={maximum_initial_residual:.6f} "
            f"sublegs={subleg_count} opposite_arm=hold grippers=hold"
        )

        legs = []
        start_positions = positions
        for expected_epoch, end_positions in enumerate(subtargets, start=1):
            motion_request_sent = True
            started = call(
                node,
                command_client,
                finite_request(start_positions, end_positions),
                args.timeout_s,
            )
            started_document = response_document(started)
            if (
                not started.accepted
                or started.adapter_state != "active"
                or started.arbiter_epoch != expected_epoch
            ):
                raise RuntimeError(
                    f"finite leg {expected_epoch} rejected: {started}"
                )
            history = wait_until_ready(
                node,
                status_client,
                owner=OWNER,
                epoch=expected_epoch,
                timeout_s=args.timeout_s,
            )
            legs.append(
                {
                    "epoch": expected_epoch,
                    "target_positions_rad": list(end_positions),
                    "start_response": started_document,
                    "status_history": history,
                }
            )
            print(
                "RESIDENT_RIGHT_ARM_Q0_LEG_PASS "
                f"epoch={expected_epoch}/{subleg_count}"
            )
            start_positions = end_positions

        terminal_feedback = wait_for_q0_feedback(
            node, feedback_messages, args.timeout_s
        )
        final_right_positions = tuple(
            float(terminal_feedback.positions[index])
            for index in RIGHT_ARM_INDICES
        )
        maximum_final_residual = max(
            abs(value) for value in final_right_positions
        )
        if maximum_final_residual > Q0_TOLERANCE_RAD:
            raise RuntimeError(
                "right arm did not settle at q0: "
                f"maximum_residual_rad={maximum_final_residual:.6f} "
                f"limit={Q0_TOLERANCE_RAD:.6f}"
            )
        print(
            "RESIDENT_RIGHT_ARM_Q0_FEEDBACK_PASS "
            f"positions_rad={list(final_right_positions)} "
            f"maximum_residual_rad={maximum_final_residual:.6f}"
        )

        stop_request = BimanualStreamCommand.Request()
        stop_request.operation = BimanualStreamCommand.Request.STOP
        stop_request.owner = OWNER
        stopped = call(
            node,
            command_client,
            stop_request,
            args.timeout_s,
        )
        stop_accepted = bool(stopped.accepted)
        if (
            not stopped.accepted
            or stopped.adapter_state != "stopped"
            or stopped.arbiter_epoch != subleg_count
        ):
            raise RuntimeError(f"coordinated stop rejected: {stopped}")
        final = status_document(node, status_client, args.timeout_s)
        if (
            final.get("state") != "stopped"
            or final.get("owner") != OWNER
            or final.get("arbiter_epoch") != subleg_count
        ):
            raise RuntimeError(f"unexpected final resident state: {final}")

        document = {
            "schema_version": 1,
            "record_kind": "resident_right_arm_q0_once",
            "overall_verdict": (
                "RESIDENT_RIGHT_ARM_Q0_ONCE_PASS"
            ),
            "operator_confirmation": args.confirmation,
            "firmware_version": initial["firmware_version"],
            "joint_names": list(anchor.name),
            "anchor_positions_rad": list(positions),
            "point_offsets_ms": list(POINT_OFFSETS_MS),
            "target_positions_rad": list(target_positions),
            "right_arm_indices": list(RIGHT_ARM_INDICES),
            "maximum_initial_residual_rad": maximum_initial_residual,
            "maximum_subleg_delta_rad": MAXIMUM_SUBLEG_DELTA_RAD,
            "subleg_count": subleg_count,
            "terminal_right_positions_rad": list(final_right_positions),
            "maximum_final_residual_rad": maximum_final_residual,
            "q0_tolerance_rad": Q0_TOLERANCE_RAD,
            "initial_status": initial,
            "legs": legs,
            "stop_response": response_document(stopped),
            "final_status": final,
            "coordinated_stop_verified": True,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "RESIDENT_RIGHT_ARM_Q0_ONCE_PASS "
            f"sublegs={subleg_count} "
            f"maximum_final_residual_rad={maximum_final_residual:.6f} "
            f"state={final['state']} output={args.output} sha256={digest}"
        )
        return 0
    finally:
        if motion_request_sent and not stop_accepted:
            try:
                emergency = BimanualStreamCommand.Request()
                emergency.operation = BimanualStreamCommand.Request.STOP
                emergency.owner = OWNER
                call(node, command_client, emergency, min(args.timeout_s, 2.0))
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
