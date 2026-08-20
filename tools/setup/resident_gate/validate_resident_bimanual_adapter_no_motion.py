#!/usr/bin/env python3
"""Validate the resident Pi adapter without authorizing motion."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
BRIDGE_SOURCE = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
if str(BRIDGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(BRIDGE_SOURCE))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES

from so101_interfaces.msg import BimanualJointFeedback
from so101_interfaces.srv import BimanualStreamCommand

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectoryPoint


CONFIRMATION = "RESIDENT_BIMANUAL_ADAPTER_NO_MOTION"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            ROOT
            / "artifacts/resident_adapter/2026-08-14/no_motion_run01.json"
        ),
    )
    return parser.parse_args()


def duration_point(
    positions: tuple[float, ...],
    offset_ms: int,
) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = offset_ms // 1000
    point.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return point


def call(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}") from error
    return future.result()


def wait_for_message(node: Node, storage: list, topic: str, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while not storage and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not storage:
        raise RuntimeError(f"timeout waiting for {topic}")
    return storage[-1]


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; launch the adapter with "
            "motion_authorized:=false"
        )
    if args.timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be positive")

    rclpy.init()
    node = Node("resident_bimanual_adapter_no_motion_validator")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
    feedback_qos = QoSProfile(depth=1)
    feedback_qos.reliability = ReliabilityPolicy.RELIABLE
    feedback_messages: list[BimanualJointFeedback] = []
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
    command_client = node.create_client(
        BimanualStreamCommand,
        COMMAND_SERVICE,
    )
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (REFRESH_ANCHOR_SERVICE, refresh_anchor_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        initial_status = call(
            node,
            status_client,
            Trigger.Request(),
            args.timeout_s,
        )
        initial_document = json.loads(initial_status.message)
        if (
            not initial_status.success
            or initial_document.get("state") != "ready"
            or initial_document.get("owner") is not None
            or initial_document.get("arbiter_epoch") != 0
            or initial_document.get("motion_authorized") is not False
            or initial_document.get("firmware_version")
            != "0x00024809"
        ):
            raise RuntimeError(
                f"unexpected initial adapter status: {initial_status}"
            )

        anchors.clear()
        feedback_messages.clear()
        refreshed = call(
            node,
            refresh_anchor_client,
            Trigger.Request(),
            args.timeout_s,
        )
        if not refreshed.success:
            raise RuntimeError(f"anchor refresh failed: {refreshed}")
        refresh_document = json.loads(refreshed.message)
        if (
            refresh_document.get("state") != "ready"
            or refresh_document.get("firmware_version") != "0x00024809"
            or refresh_document.get("joint_count") != 12
            or refresh_document.get("torque_enabled") is not False
        ):
            raise RuntimeError(
                f"unexpected anchor refresh response: {refreshed}"
            )
        anchor = wait_for_message(
            node, anchors, ANCHOR_TOPIC, args.timeout_s
        )
        feedback = wait_for_message(
            node, feedback_messages, FEEDBACK_TOPIC, args.timeout_s
        )
        if (
            tuple(anchor.name) != CANONICAL_JOINT_NAMES
            or len(anchor.position) != 12
        ):
            raise RuntimeError(f"invalid anchor joint state: {anchor}")
        if (
            tuple(feedback.joint_names) != CANONICAL_JOINT_NAMES
            or len(feedback.positions) != 12
            or len(feedback.sample_age_ms) != 12
            or feedback.present_mask != 0x0FFF
        ):
            raise RuntimeError(f"invalid measured feedback: {feedback}")

        request = BimanualStreamCommand.Request()
        request.operation = BimanualStreamCommand.Request.START_OPEN
        request.owner = "no_motion_validator"
        request.joint_names = list(CANONICAL_JOINT_NAMES)
        positions = tuple(anchor.position)
        request.points = [
            duration_point(positions, 80),
            duration_point(positions, 130),
        ]
        rejected = call(node, command_client, request, args.timeout_s)
        if (
            rejected.accepted
            or rejected.adapter_state != "ready"
            or rejected.arbiter_epoch != 0
            or "motion_authorized is false" not in rejected.diagnostic
        ):
            raise RuntimeError(
                f"motion-disabled command was not rejected: {rejected}"
            )

        final_status = call(
            node,
            status_client,
            Trigger.Request(),
            args.timeout_s,
        )
        final_document = json.loads(final_status.message)
        if (
            not final_status.success
            or final_document != initial_document
        ):
            raise RuntimeError(
                "adapter state changed after rejected motion command: "
                f"{final_status}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "resident_bimanual_adapter_no_motion",
            "overall_verdict": "RESIDENT_BIMANUAL_ADAPTER_NO_MOTION_PASS",
            "motion_authorized": False,
            "torque_command_attempted": False,
            "firmware_version": initial_document["firmware_version"],
            "joint_names": list(anchor.name),
            "anchor_positions_rad": [float(value) for value in anchor.position],
            "measured_positions_rad": [
                float(value) for value in feedback.positions
            ],
            "sample_age_ms": [
                int(value) for value in feedback.sample_age_ms
            ],
            "present_mask": int(feedback.present_mask),
            "firmware_tick_ms": int(feedback.firmware_tick_ms),
            "completed_pairs": int(feedback.completed_pairs),
            "initial_status": initial_document,
            "anchor_refresh": refresh_document,
            "rejected_command": {
                "accepted": rejected.accepted,
                "adapter_state": rejected.adapter_state,
                "arbiter_epoch": rejected.arbiter_epoch,
                "diagnostic": rejected.diagnostic,
            },
            "final_status": final_document,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "RESIDENT_BIMANUAL_ADAPTER_NO_MOTION_PASS "
            f"joints={len(anchor.name)} state={final_document['state']} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
