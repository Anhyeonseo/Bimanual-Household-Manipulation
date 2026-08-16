#!/usr/bin/env python3
"""Move only selected resident gripper axes for supervised can commissioning."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_interfaces.srv import BimanualStreamCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES


STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/joint_states"
OWNER = "can_gripper_commissioning"
CONFIRMATION = "COMMISSION_CAN_GRIPPER_PROBE_ONCE"
RAW_STEP_RAD = 2.0 * math.pi / 4096.0
SAMPLE_PERIOD_MS = 50
FIRST_POINT_MS = 80
PROBE_MINIMUM_RAW = 1948
PROBE_MAXIMUM_RAW = 2009
GRIPPER_INDICES = {"left": 5, "right": 11}
ARM_INDICES = tuple(index for index in range(12) if index not in (5, 11))
ARM_MOTION_LIMIT_RAD = 0.02


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), required=True)
    parser.add_argument("--target-raw", type=int, required=True)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--duration-ms", type=int, default=1500)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--label", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/can_to_bin/gripper_probe_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error(
            "confirmation mismatch; support both arms and clear the grippers"
        )
    if not PROBE_MINIMUM_RAW <= args.target_raw <= PROBE_MAXIMUM_RAW:
        parser.error(
            f"target raw must be within {PROBE_MINIMUM_RAW}..{PROBE_MAXIMUM_RAW}"
        )
    if not 800 <= args.duration_ms <= 2500 or args.duration_ms % 50 != 0:
        parser.error(
            "duration must be a multiple of 50 ms within 800..2500 ms"
        )
    if args.timeout_s <= 0.0:
        parser.error("timeout must be positive")
    if not args.label.strip():
        parser.error("label must not be empty")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing artifact: {args.output}")
    return args


def semantic_raw_to_rad(raw: int) -> float:
    return (2048 - int(raw)) * RAW_STEP_RAD


def semantic_rad_to_raw(position_rad: float) -> int:
    if not math.isfinite(position_rad):
        raise ValueError("gripper position must be finite")
    return round(2048 - position_rad / RAW_STEP_RAD)


def target_positions(
    current: tuple[float, ...],
    side: str,
    target_raw: int,
) -> tuple[float, ...]:
    if len(current) != 12 or not all(math.isfinite(value) for value in current):
        raise ValueError("current feedback must contain 12 finite positions")
    if side not in ("left", "right", "both"):
        raise ValueError("side must be left, right, or both")
    if not PROBE_MINIMUM_RAW <= int(target_raw) <= PROBE_MAXIMUM_RAW:
        raise ValueError("target raw is outside the bounded probe range")
    result = list(current)
    indices = (5, 11) if side == "both" else (GRIPPER_INDICES[side],)
    for index in indices:
        result[index] = semantic_raw_to_rad(target_raw)
    return tuple(result)


def interpolate_probe(
    start: tuple[float, ...],
    target: tuple[float, ...],
    duration_ms: int,
) -> list[tuple[int, tuple[float, ...]]]:
    if len(start) != 12 or len(target) != 12:
        raise ValueError("probe interpolation requires 12-axis endpoints")
    if (
        duration_ms < SAMPLE_PERIOD_MS
        or duration_ms % SAMPLE_PERIOD_MS != 0
    ):
        raise ValueError("probe duration must be a positive sample multiple")
    count = duration_ms // SAMPLE_PERIOD_MS
    output: list[tuple[int, tuple[float, ...]]] = []
    for index in range(1, count + 1):
        fraction = index / count
        positions = tuple(
            begin + (end - begin) * fraction
            for begin, end in zip(start, target, strict=True)
        )
        offset_ms = FIRST_POINT_MS + (index - 1) * SAMPLE_PERIOD_MS
        output.append((offset_ms, positions))
    output.append(
        (
            FIRST_POINT_MS + count * SAMPLE_PERIOD_MS,
            tuple(target),
        )
    )
    return output


def trajectory_point(
    positions: tuple[float, ...],
    offset_ms: int,
) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = offset_ms // 1000
    point.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return point


def call(node: Node, client: Any, request: Any, timeout_s: float) -> Any:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("service response timeout")
    error = future.exception()
    if error is not None:
        raise RuntimeError(f"service call failed: {error}") from error
    return future.result()


def status_document(node: Node, client: Any, timeout_s: float) -> dict[str, Any]:
    response = call(node, client, Trigger.Request(), timeout_s)
    if not response.success:
        raise RuntimeError(f"status service rejected: {response.message}")
    document = json.loads(response.message)
    if not isinstance(document, dict):
        raise RuntimeError("status response is not an object")
    return document


def wait_feedback(
    node: Node,
    storage: list[JointState],
    timeout_s: float,
) -> JointState:
    storage.clear()
    deadline = time.monotonic() + timeout_s
    while not storage and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not storage:
        raise RuntimeError(f"timeout waiting for {FEEDBACK_TOPIC}")
    message = storage[-1]
    if (
        tuple(message.name) != CANONICAL_JOINT_NAMES
        or len(message.position) != 12
        or not all(math.isfinite(value) for value in message.position)
    ):
        raise RuntimeError("resident joint feedback is invalid")
    return message


def wait_until_ready(
    node: Node,
    client: Any,
    expected_epoch: int,
    timeout_s: float,
) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_s
    history: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        document = status_document(node, client, timeout_s)
        history.append(document)
        if (
            document.get("state") == "ready"
            and document.get("owner") == OWNER
            and document.get("arbiter_epoch") == expected_epoch
        ):
            return history
        if document.get("state") not in ("active", "ready"):
            raise RuntimeError(f"resident probe failed closed: {document}")
        time.sleep(0.05)
    raise RuntimeError(f"probe completion timeout: {history[-1:]}")


def response_document(response: Any) -> dict[str, Any]:
    return {
        "accepted": bool(response.accepted),
        "adapter_state": response.adapter_state,
        "arbiter_epoch": int(response.arbiter_epoch),
        "diagnostic": response.diagnostic,
    }


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Node("commission_can_gripper_probe_once")
    feedback: list[JointState] = []
    node.create_subscription(JointState, FEEDBACK_TOPIC, feedback.append, 1)
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)
    request_sent = False
    explicitly_rejected = False
    completed = False
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        initial = status_document(node, status_client, args.timeout_s)
        if (
            initial.get("state") != "ready"
            or initial.get("owner") not in (None, OWNER)
            or initial.get("motion_authorized") is not True
            or initial.get("fault_diagnostic") is not None
            or initial.get("firmware_version") != "0x00024809"
        ):
            raise RuntimeError(f"unexpected initial resident state: {initial}")

        before_message = wait_feedback(node, feedback, args.timeout_s)
        before = tuple(float(value) for value in before_message.position)
        target = target_positions(before, args.side, args.target_raw)
        expected_epoch = int(initial["arbiter_epoch"]) + 1

        request = BimanualStreamCommand.Request()
        request.operation = BimanualStreamCommand.Request.START_FINITE
        request.owner = OWNER
        request.joint_names = list(CANONICAL_JOINT_NAMES)
        request.points = [
            trajectory_point(positions, offset_ms)
            for offset_ms, positions in interpolate_probe(
                before, target, args.duration_ms
            )
        ]
        request_sent = True
        started = call(node, command_client, request, args.timeout_s)
        if not started.accepted:
            explicitly_rejected = True
            raise RuntimeError(f"gripper probe rejected: {started}")
        if (
            started.adapter_state != "active"
            or started.arbiter_epoch != expected_epoch
        ):
            raise RuntimeError(f"gripper probe rejected: {started}")

        history = wait_until_ready(
            node,
            status_client,
            expected_epoch,
            args.timeout_s,
        )
        after_message = wait_feedback(node, feedback, args.timeout_s)
        after = tuple(float(value) for value in after_message.position)
        arm_motion = max(abs(after[index] - before[index]) for index in ARM_INDICES)
        if arm_motion > ARM_MOTION_LIMIT_RAD:
            raise RuntimeError(
                f"uncommanded arm motion {arm_motion:.6f} rad exceeds limit"
            )

        before_raw = {
            side: semantic_rad_to_raw(before[index])
            for side, index in GRIPPER_INDICES.items()
        }
        after_raw = {
            side: semantic_rad_to_raw(after[index])
            for side, index in GRIPPER_INDICES.items()
        }
        selected = ("left", "right") if args.side == "both" else (args.side,)
        residual_raw = {
            side: abs(after_raw[side] - args.target_raw) for side in selected
        }

        document = {
            "schema_version": 1,
            "record_kind": "can_gripper_resident_probe_once",
            "status": "CAN_GRIPPER_RESIDENT_PROBE_PASS",
            "label": args.label,
            "operator_confirmation": args.confirmation,
            "firmware_version": initial["firmware_version"],
            "owner": OWNER,
            "side": args.side,
            "target_raw": args.target_raw,
            "target_rad": semantic_raw_to_rad(args.target_raw),
            "duration_ms": args.duration_ms,
            "joint_names": list(CANONICAL_JOINT_NAMES),
            "before_positions_rad": list(before),
            "after_positions_rad": list(after),
            "before_gripper_raw": before_raw,
            "after_gripper_raw": after_raw,
            "selected_residual_raw": residual_raw,
            "maximum_uncommanded_arm_motion_rad": arm_motion,
            "automatic_retry_count": 0,
            "motion_commands": 1,
            "initial_status": initial,
            "start_response": response_document(started),
            "status_history": history,
            "terminal_state": "ready_torque_hold",
            "stop_sent": False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        completed = True
        print(
            "CAN_GRIPPER_RESIDENT_PROBE_PASS "
            f"label={args.label} side={args.side} target_raw={args.target_raw} "
            f"after_raw={after_raw} residual_raw={residual_raw} "
            f"arm_motion_rad={arm_motion:.6f} epoch={expected_epoch} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        if request_sent and not completed and not explicitly_rejected:
            try:
                stop = BimanualStreamCommand.Request()
                stop.operation = BimanualStreamCommand.Request.STOP
                stop.owner = OWNER
                call(node, command_client, stop, min(args.timeout_s, 2.0))
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
