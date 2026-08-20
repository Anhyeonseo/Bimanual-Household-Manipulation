#!/usr/bin/env python3
"""Exercise resident START_OPEN, APPEND, SPLICE, APPEND at one pose."""

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

from so101_interfaces.srv import BimanualStreamCommand

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectoryPoint


CONFIRMATION = "RESIDENT_BIMANUAL_ROLLING_HORIZON_NO_MOTION_ONCE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
OWNER = "resident_rolling_no_motion_validator"
START_OFFSETS_MS = (80, 130, 180)
KEEPALIVE_OFFSETS_MS = (280, 330)
KEEPALIVE_COUNT = 3
KEEPALIVE_INTERVAL_S = 0.05


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
            "rolling_horizon_no_motion_run01.json"
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
    positions: tuple[float, ...],
    offsets_ms: tuple[int, ...],
    *,
    splice_offset_ms: int = 0,
):
    request = BimanualStreamCommand.Request()
    request.operation = operation
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINT_NAMES)
    request.splice_offset_ms = splice_offset_ms
    request.points = [point(positions, offset) for offset in offsets_ms]
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
    node = Node("resident_bimanual_rolling_horizon_no_motion_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
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

        request_type = BimanualStreamCommand.Request
        command_specs = (
            ("start_open", request_type.START_OPEN, START_OFFSETS_MS, 0, 1),
            ("append_1", request_type.APPEND, (200, 250), 0, 1),
            ("splice", request_type.SPLICE, (150, 200), 100, 2),
            ("append_2", request_type.APPEND, (220, 270), 0, 2),
        )
        commands = []
        for (
            label,
            operation,
            offsets,
            splice_offset_ms,
            expected_epoch,
        ) in command_specs:
            motion_request_sent = True
            response = call(
                node,
                command_client,
                stream_request(
                    operation,
                    positions,
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
                    positions,
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

        active = status_document(node, status_client, args.timeout_s)
        if (
            active.get("state") != "active"
            or active.get("owner") != OWNER
            or active.get("arbiter_epoch") != 2
        ):
            raise RuntimeError(f"unexpected rolling state: {active}")

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
            "record_kind": "resident_bimanual_rolling_horizon_no_motion_once",
            "overall_verdict": (
                "RESIDENT_BIMANUAL_ROLLING_HORIZON_NO_MOTION_ONCE_PASS"
            ),
            "operator_confirmation": args.confirmation,
            "firmware_version": initial["firmware_version"],
            "joint_names": list(anchor.name),
            "anchor_positions_rad": list(positions),
            "commanded_motion_delta_rad": [0.0] * 12,
            "initial_status": initial,
            "commands": commands,
            "active_status": active,
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
            "RESIDENT_BIMANUAL_ROLLING_HORIZON_NO_MOTION_ONCE_PASS "
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
