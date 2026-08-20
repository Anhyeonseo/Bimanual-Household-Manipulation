#!/usr/bin/env python3
"""Run two resident finite holds at the measured 12-axis pose, then stop."""

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


CONFIRMATION = "RESIDENT_BIMANUAL_CURRENT_POSE_HOLD_TWICE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
OWNER = "resident_hold_validator"
POINT_OFFSETS_MS = (80, 130, 180)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--ready-soak-s", type=float, default=2.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root
            / "artifacts/resident_adapter/2026-08-14/current_pose_hold_twice_run01.json"
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


def finite_request(positions: tuple[float, ...]):
    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.START_FINITE
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINT_NAMES)
    request.points = [point(positions, offset) for offset in POINT_OFFSETS_MS]
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
    if not 1.0 <= args.ready_soak_s <= 10.0:
        raise SystemExit("--ready-soak-s must be within 1.0..10.0")

    rclpy.init()
    node = Node("resident_bimanual_current_pose_hold_twice")
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

        legs = []
        for expected_epoch in (1, 2):
            motion_request_sent = True
            started = call(
                node,
                command_client,
                finite_request(positions),
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
                    "start_response": started_document,
                    "status_history": history,
                }
            )

        # Exercise the independent resident heartbeat in armed READY without
        # application traffic. This is deliberately longer than the 500 ms
        # MCU watchdog and catches periodic-callback starvation before a real
        # trajectory is attempted.
        time.sleep(args.ready_soak_s)
        ready_soak_status = status_document(
            node,
            status_client,
            args.timeout_s,
        )
        if (
            ready_soak_status.get("state") != "ready"
            or ready_soak_status.get("owner") != OWNER
            or ready_soak_status.get("arbiter_epoch") != 2
            or ready_soak_status.get("torque_hold_active") is not True
        ):
            raise RuntimeError(
                f"armed READY soak failed: {ready_soak_status}"
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
            "record_kind": "resident_bimanual_current_pose_hold_twice",
            "overall_verdict": (
                "RESIDENT_BIMANUAL_CURRENT_POSE_HOLD_TWICE_PASS"
            ),
            "operator_confirmation": args.confirmation,
            "firmware_version": initial["firmware_version"],
            "joint_names": list(anchor.name),
            "anchor_positions_rad": list(positions),
            "point_offsets_ms": list(POINT_OFFSETS_MS),
            "commanded_motion_delta_rad": [0.0] * 12,
            "ready_soak_s": args.ready_soak_s,
            "ready_soak_status": ready_soak_status,
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
            "RESIDENT_BIMANUAL_CURRENT_POSE_HOLD_TWICE_PASS "
            "legs=2 epochs=1,2 commanded_delta_rad=0 "
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
