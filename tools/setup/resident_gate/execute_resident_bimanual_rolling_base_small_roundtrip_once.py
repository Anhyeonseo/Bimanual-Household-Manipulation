#!/usr/bin/env python3
"""Exercise a moving resident rolling horizon and non-stopping splice."""

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


CONFIRMATION = "RESIDENT_BIMANUAL_ROLLING_BASE_SMALL_ROUNDTRIP_ONCE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"
OWNER = "resident_rolling_base_roundtrip_validator"
START_OFFSETS_MS = (80, 130, 180)
KEEPALIVE_OFFSETS_MS = (280, 330)
KEEPALIVE_COUNT = 3
KEEPALIVE_INTERVAL_S = 0.05
FINAL_SETTLE_S = 0.08
BASE_DELTA_RAD = 0.03
BASE_INDICES = (0, 6)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root
            / "artifacts/resident_adapter/2026-08-14/"
            "rolling_base_small_roundtrip_run01.json"
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


def status_document(node: Node, client, timeout_s: float) -> dict:
    response = call(node, client, Trigger.Request(), timeout_s)
    document = json.loads(response.message)
    if not response.success or document.get("state") == "faulted":
        raise RuntimeError(f"resident adapter is unhealthy: {response}")
    return document


def point(positions: tuple[float, ...], offset_ms: int):
    result = JointTrajectoryPoint()
    result.positions = list(positions)
    result.time_from_start.sec = offset_ms // 1000
    result.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return result


def stream_request(
    operation: int,
    position_sets: tuple[tuple[float, ...], ...],
    offsets_ms: tuple[int, ...],
    *,
    splice_offset_ms: int = 0,
):
    request = BimanualStreamCommand.Request()
    request.operation = operation
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINT_NAMES)
    request.splice_offset_ms = splice_offset_ms
    if len(position_sets) != len(offsets_ms):
        raise ValueError("position_sets and offsets_ms length mismatch")
    request.points = [
        point(positions, offset)
        for positions, offset in zip(
            position_sets,
            offsets_ms,
            strict=True,
        )
    ]
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
    node = Node("resident_bimanual_rolling_base_small_roundtrip_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
    feedback_qos = QoSProfile(depth=20)
    feedback_qos.reliability = ReliabilityPolicy.RELIABLE
    feedback_messages: list[BimanualJointFeedback] = []
    node.create_subscription(
        BimanualJointFeedback,
        FEEDBACK_TOPIC,
        feedback_messages.append,
        feedback_qos,
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
            not in ("0x00024806", "0x00024809")
        ):
            raise RuntimeError(f"unexpected initial resident state: {initial}")

        target = list(positions)
        for index in BASE_INDICES:
            target[index] += BASE_DELTA_RAD
        target_positions = tuple(target)
        outbound = tuple(
            tuple(
                start + ((end - start) * fraction)
                for start, end in zip(
                    positions,
                    target_positions,
                    strict=True,
                )
            )
            for fraction in (1.0 / 3.0, 2.0 / 3.0, 1.0)
        )
        target_hold = (target_positions, target_positions)
        return_targets = (positions, positions)
        anchor_hold = (positions, positions)

        request_type = BimanualStreamCommand.Request
        command_specs = (
            (
                "start_open",
                request_type.START_OPEN,
                START_OFFSETS_MS,
                outbound,
                0,
                1,
            ),
            (
                "append_1",
                request_type.APPEND,
                (200, 250),
                target_hold,
                0,
                1,
            ),
            (
                "splice",
                request_type.SPLICE,
                (150, 200),
                return_targets,
                100,
                2,
            ),
            (
                "append_2",
                request_type.APPEND,
                (220, 270),
                anchor_hold,
                0,
                2,
            ),
        )
        commands = []
        for (
            label,
            operation,
            offsets,
            position_sets,
            splice_offset_ms,
            expected_epoch,
        ) in command_specs:
            motion_request_sent = True
            response = call(
                node,
                command_client,
                stream_request(
                    operation,
                    position_sets,
                    offsets,
                    splice_offset_ms=splice_offset_ms,
                ),
                args.timeout_s,
            )
            response_data = response_document(response)
            if (
                not response.accepted
                or response.adapter_state != "active"
                or response.arbiter_epoch != expected_epoch
            ):
                raise RuntimeError(f"{label} rejected: {response}")
            commands.append(
                {
                    "label": label,
                    "operation": int(operation),
                    "offsets_ms": list(offsets),
                    "splice_offset_ms": splice_offset_ms,
                    "response": response_data,
                }
            )

        for index in range(KEEPALIVE_COUNT):
            time.sleep(KEEPALIVE_INTERVAL_S)
            response = call(
                node,
                command_client,
                stream_request(
                    request_type.APPEND,
                    (positions, positions),
                    KEEPALIVE_OFFSETS_MS,
                ),
                args.timeout_s,
            )
            if (
                not response.accepted
                or response.adapter_state != "active"
                or response.arbiter_epoch != 2
            ):
                raise RuntimeError(
                    f"keepalive append {index + 1} rejected: {response}"
                )
            commands.append(
                {
                    "label": f"keepalive_append_{index + 1}",
                    "operation": int(request_type.APPEND),
                    "offsets_ms": list(KEEPALIVE_OFFSETS_MS),
                    "response": response_document(response),
                }
            )

        time.sleep(FINAL_SETTLE_S)
        active = status_document(node, status_client, args.timeout_s)
        if (
            active.get("state") != "active"
            or active.get("owner") != OWNER
            or active.get("arbiter_epoch") != 2
        ):
            raise RuntimeError(f"unexpected rolling state: {active}")

        active_feedback = [
            message
            for message in feedback_messages
            if message.completed_pairs > 0
        ]
        if not active_feedback:
            raise RuntimeError("no measured feedback arrived during motion")
        latest_feedback = active_feedback[-1]
        if (
            tuple(latest_feedback.joint_names) != CANONICAL_JOINT_NAMES
            or latest_feedback.present_mask != 0x0FFF
            or len(latest_feedback.positions) != 12
            or len(latest_feedback.sample_age_ms) != 12
            or max(latest_feedback.sample_age_ms) > 150
        ):
            raise RuntimeError(
                f"measured feedback is incomplete or stale: {latest_feedback}"
            )
        maximum_observed_base_delta_rad = max(
            max(
                abs(message.positions[index] - positions[index])
                for index in BASE_INDICES
            )
            for message in active_feedback
        )
        if maximum_observed_base_delta_rad < 0.005:
            raise RuntimeError(
                "measured feedback did not observe the commanded base motion"
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
            or stopped.arbiter_epoch != 2
        ):
            raise RuntimeError(f"coordinated stop rejected: {stopped}")
        final = status_document(node, status_client, args.timeout_s)
        if (
            final.get("state") != "stopped"
            or final.get("owner") != OWNER
            or final.get("arbiter_epoch") != 2
        ):
            raise RuntimeError(f"unexpected final resident state: {final}")

        document = {
            "schema_version": 1,
            "record_kind": "resident_bimanual_rolling_base_small_roundtrip_once",
            "overall_verdict": (
                "RESIDENT_BIMANUAL_ROLLING_BASE_SMALL_ROUNDTRIP_ONCE_PASS"
            ),
            "operator_confirmation": args.confirmation,
            "firmware_version": initial["firmware_version"],
            "joint_names": list(anchor.name),
            "anchor_positions_rad": list(positions),
            "commanded_motion_delta_rad": [
                BASE_DELTA_RAD if index in BASE_INDICES else 0.0
                for index in range(12)
            ],
            "splice_offset_ms": 100,
            "final_settle_s": FINAL_SETTLE_S,
            "initial_status": initial,
            "commands": commands,
            "active_status": active,
            "stop_response": response_document(stopped),
            "final_status": final,
            "feedback_samples": len(active_feedback),
            "feedback_completed_pairs": int(
                latest_feedback.completed_pairs
            ),
            "feedback_sample_age_ms": [
                int(value) for value in latest_feedback.sample_age_ms
            ],
            "feedback_maximum_sample_age_ms": int(
                max(latest_feedback.sample_age_ms)
            ),
            "maximum_observed_base_delta_rad": (
                maximum_observed_base_delta_rad
            ),
            "coordinated_stop_verified": True,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "RESIDENT_BIMANUAL_ROLLING_BASE_SMALL_ROUNDTRIP_ONCE_PASS "
            f"delta_rad={BASE_DELTA_RAD:.6f} "
            f"feedback_samples={len(active_feedback)} "
            f"feedback_age_max_ms={max(latest_feedback.sample_age_ms)} "
            "commands=7 epochs=1,1,2,2,2,2,2 "
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
