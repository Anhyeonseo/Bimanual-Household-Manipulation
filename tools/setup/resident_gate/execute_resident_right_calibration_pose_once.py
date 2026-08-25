#!/usr/bin/env python3
"""Move the right arm slowly to one previously observed calibration pose."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

CONFIRMATION = "RESIDENT_RIGHT_CALIBRATION_POSE_ONCE"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
OWNER = "resident_right_calibration_operator"
RIGHT_ARM_INDICES = (6, 7, 8, 9, 10)
POINT_OFFSETS_MS = (100, 200, 300, 400)
MAXIMUM_SUBLEG_DELTA_RAD = 0.04
MAXIMUM_ALLOWED_TOTAL_DELTA_RAD = 0.45
# Captures use the measured terminal anchor, not the requested waypoint.  This
# remains a fail-closed gross-settle bound while allowing ordinary SO-101
# position quantization and loaded gear residual at a visibility waypoint.
FINAL_RESIDUAL_LIMIT_RAD = 0.05


def load_capture_target(path: Path) -> tuple[str, tuple[float, ...]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    capture = document.get("capture", {})
    if document.get("status") != "STATIONARY_READ_ONLY_CAPTURE_PASS":
        raise ValueError("target must come from a passed read-only capture")
    if bool(document.get("motion_authorized", False)):
        raise ValueError("target capture must be motion_authorized=false")
    if capture.get("arm") != "right":
        raise ValueError("target capture must describe the right arm")
    values = tuple(float(value) for value in capture.get("measured_arm_rad", ()))
    if len(values) != 5 or not all(math.isfinite(value) for value in values):
        raise ValueError("target capture must contain five finite joint values")
    if float(capture.get("pnp_rms_px_max", math.inf)) > float(
        capture.get("pnp_rms_px_limit", 0.0)
    ):
        raise ValueError("target capture PnP check did not pass")
    if tuple(capture.get("detected_marker_ids", ())) != (0, 1, 2, 3):
        raise ValueError("target capture did not observe the complete GridBoard")
    capture_id = str(capture.get("id", "")).strip()
    if not capture_id:
        raise ValueError("target capture id is missing")
    return capture_id, values


def compose_bimanual_target(
    anchor: tuple[float, ...],
    right_target: tuple[float, ...],
) -> tuple[float, ...]:
    if len(anchor) != 12 or len(right_target) != 5:
        raise ValueError("invalid anchor or target joint count")
    result = list(anchor)
    for index, value in zip(RIGHT_ARM_INDICES, right_target, strict=True):
        result[index] = value
    return tuple(result)


def segmented_targets(
    start: tuple[float, ...],
    target: tuple[float, ...],
) -> tuple[tuple[float, ...], ...]:
    maximum_delta = max(abs(end - begin) for begin, end in zip(start, target, strict=True))
    count = max(1, math.ceil(maximum_delta / MAXIMUM_SUBLEG_DELTA_RAD))
    return tuple(
        tuple(
            begin + ((end - begin) * index / count)
            for begin, end in zip(start, target, strict=True)
        )
        for index in range(1, count + 1)
    )


def initial_session_epoch(
    document: dict,
    *,
    resume_held_session: bool,
    precheck_only: bool,
) -> int:
    if (
        document.get("state") != "ready"
        or document.get("fault_diagnostic") is not None
    ):
        raise ValueError(f"resident session is not healthy READY: {document}")
    epoch = int(document.get("arbiter_epoch", -1))
    if resume_held_session:
        if (
            document.get("motion_authorized") is not True
            or document.get("owner") != OWNER
            or epoch <= 0
            or document.get("torque_hold_active") is not True
        ):
            raise ValueError(
                f"resident held session cannot be resumed: {document}"
            )
        return epoch
    if (
        document.get("owner") is not None
        or epoch != 0
        or document.get("torque_hold_active") is not False
        or (
            not precheck_only
            and document.get("motion_authorized") is not True
        )
    ):
        raise ValueError(f"unexpected fresh resident session: {document}")
    return 0


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--target-capture", required=True, type=Path)
    parser.add_argument(
        "--precheck-only",
        action="store_true",
        help="Validate the live anchor and target without sending motion",
    )
    parser.add_argument(
        "--resume-held-session",
        action="store_true",
        help="Continue this operator's existing armed READY session",
    )
    parser.add_argument(
        "--leave-torque-hold-active",
        action="store_true",
        help="Keep the successful final READY torque hold for the next command",
    )
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--capture-soak-s", type=float, default=10.0)
    parser.add_argument(
        "--capture-completion-file",
        type=Path,
        help=(
            "Keep the final torque hold until this newly created capture "
            "file appears, bounded by --capture-soak-s"
        ),
    )
    parser.add_argument(
        "--capture-skip-file",
        type=Path,
        help=(
            "Treat creation of this operator signal file as a visibility "
            "skip while preserving an explicitly requested held session"
        ),
    )
    parser.add_argument(
        "--maximum-total-joint-delta-rad",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root
        / "artifacts/resident_adapter/2026-08-25/right_calibration_pose_once.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error("confirmation mismatch; clear both arm workspaces")
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if args.precheck_only and args.leave_torque_hold_active:
        parser.error(
            "--precheck-only cannot be combined with "
            "--leave-torque-hold-active"
        )
    if args.precheck_only and args.resume_held_session:
        parser.error(
            "--precheck-only cannot be combined with --resume-held-session"
        )
    if not 1.0 <= args.capture_soak_s <= 120.0:
        parser.error("--capture-soak-s must be within 1.0..120.0")
    if (
        args.capture_skip_file is not None
        and args.capture_completion_file is None
    ):
        parser.error(
            "--capture-skip-file requires --capture-completion-file"
        )
    if not 0.04 <= args.maximum_total_joint_delta_rad <= MAXIMUM_ALLOWED_TOTAL_DELTA_RAD:
        parser.error(
            "--maximum-total-joint-delta-rad must be within 0.04.."
            f"{MAXIMUM_ALLOWED_TOTAL_DELTA_RAD}"
        )
    return args


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


def wait_for_anchor(node: Node, storage: list[JointState], timeout_s: float) -> JointState:
    deadline = time.monotonic() + timeout_s
    while not storage and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not storage:
        raise RuntimeError(f"timeout waiting for {ANCHOR_TOPIC}")
    return storage[-1]


def wait_until_ready(
    node: Node,
    client,
    *,
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
            and document.get("owner") == OWNER
            and document.get("arbiter_epoch") == epoch
        ):
            return history
        if document.get("state") not in ("active", "ready"):
            raise RuntimeError(f"unexpected resident state: {document}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for leg {epoch}: {history[-1:]}")


def point(positions: tuple[float, ...], offset_ms: int) -> JointTrajectoryPoint:
    result = JointTrajectoryPoint()
    result.positions = list(positions)
    result.time_from_start.sec = offset_ms // 1000
    result.time_from_start.nanosec = (offset_ms % 1000) * 1_000_000
    return result


def finite_request(
    start: tuple[float, ...],
    target: tuple[float, ...],
):
    from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES
    from so101_interfaces.srv import BimanualStreamCommand

    request = BimanualStreamCommand.Request()
    request.operation = BimanualStreamCommand.Request.START_FINITE
    request.owner = OWNER
    request.joint_names = list(CANONICAL_JOINT_NAMES)
    request.points = [
        point(
            tuple(
                begin + ((end - begin) * fraction)
                for begin, end in zip(start, target, strict=True)
            ),
            offset,
        )
        for fraction, offset in zip(
            (0.25, 0.5, 0.75, 1.0),
            POINT_OFFSETS_MS,
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


def require_armed_ready_hold(document: dict, epoch: int) -> None:
    if (
        document.get("state") != "ready"
        or document.get("owner") != OWNER
        or document.get("arbiter_epoch") != epoch
        or document.get("torque_hold_active") is not True
    ):
        raise RuntimeError(f"armed READY hold failed: {document}")


def main() -> int:
    from single_arm_bridge.bimanual_stream_adapter import CANONICAL_JOINT_NAMES
    from so101_interfaces.srv import BimanualStreamCommand

    args = parse_args()
    capture_id, right_target = load_capture_target(args.target_capture)
    rclpy.init()
    node = Node("resident_right_calibration_pose_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    anchors: list[JointState] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, qos)
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)
    motion_request_sent = False
    stop_accepted = False
    hold_left_active = False
    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")
        anchor_message = wait_for_anchor(node, anchors, args.timeout_s)
        anchor = tuple(float(value) for value in anchor_message.position)
        if (
            tuple(anchor_message.name) != CANONICAL_JOINT_NAMES
            or len(anchor) != 12
            or not all(math.isfinite(value) for value in anchor)
        ):
            raise RuntimeError(f"invalid resident anchor: {anchor_message}")
        initial = status_document(node, status_client, args.timeout_s)
        try:
            starting_epoch = initial_session_epoch(
                initial,
                resume_held_session=args.resume_held_session,
                precheck_only=args.precheck_only,
            )
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        if args.resume_held_session:
            prepared = initial.get("prepared_positions_rad")
            if (
                not isinstance(prepared, list)
                or len(prepared) != 12
                or not all(math.isfinite(float(value)) for value in prepared)
            ):
                raise RuntimeError(
                    f"held session has no valid prepared anchor: {initial}"
                )
            anchor = tuple(float(value) for value in prepared)

        target = compose_bimanual_target(anchor, right_target)
        capture_completion_file = (
            args.capture_completion_file.resolve()
            if args.capture_completion_file is not None
            else None
        )
        capture_skip_file = (
            args.capture_skip_file.resolve()
            if args.capture_skip_file is not None
            else None
        )
        if (
            capture_completion_file is not None
            and capture_completion_file.exists()
        ):
            raise RuntimeError(
                "capture completion file already exists before motion: "
                f"{capture_completion_file}"
            )
        if capture_skip_file is not None and capture_skip_file.exists():
            raise RuntimeError(
                "capture skip file already exists before motion: "
                f"{capture_skip_file}"
            )
        maximum_total_delta = max(
            abs(target[index] - anchor[index]) for index in RIGHT_ARM_INDICES
        )
        if maximum_total_delta > args.maximum_total_joint_delta_rad:
            raise RuntimeError(
                "target exceeds approved total joint delta: "
                f"value={maximum_total_delta:.6f} "
                f"limit={args.maximum_total_joint_delta_rad:.6f}"
            )
        subtargets = segmented_targets(anchor, target)
        final_epoch = starting_epoch + len(subtargets)
        print(
            "RESIDENT_RIGHT_CALIBRATION_PRECHECK_PASS "
            f"capture={capture_id} maximum_total_delta_rad={maximum_total_delta:.6f} "
            f"sublegs={len(subtargets)} starting_epoch={starting_epoch} "
            f"final_epoch={final_epoch} left_arm=hold grippers=hold"
        )
        if args.precheck_only:
            print(
                "RESIDENT_RIGHT_CALIBRATION_PRECHECK_ONLY_PASS "
                "motion_request_sent=false torque_enabled=false "
                f"motion_authorized={str(bool(initial.get('motion_authorized'))).lower()}"
            )
            return 0

        legs = []
        measured_start = anchor
        for leg_index, planned_target in enumerate(subtargets, start=1):
            epoch = starting_epoch + leg_index
            motion_request_sent = True
            started = call(
                node,
                command_client,
                finite_request(measured_start, planned_target),
                args.timeout_s,
            )
            if (
                not started.accepted
                or started.adapter_state != "active"
                or started.arbiter_epoch != epoch
            ):
                raise RuntimeError(f"finite leg {epoch} rejected: {started}")
            history = wait_until_ready(
                node,
                status_client,
                epoch=epoch,
                timeout_s=args.timeout_s,
            )
            terminal = history[-1]
            measured_start = tuple(
                float(value) for value in terminal["prepared_positions_rad"]
            )
            legs.append(
                {
                    "epoch": epoch,
                    "planned_target_positions_rad": list(planned_target),
                    "terminal_measured_positions_rad": list(measured_start),
                    "start_response": response_document(started),
                }
            )
            print(
                "RESIDENT_RIGHT_CALIBRATION_LEG_PASS "
                f"leg={leg_index}/{len(subtargets)} epoch={epoch}"
            )

        final_right = tuple(measured_start[index] for index in RIGHT_ARM_INDICES)
        maximum_final_residual = max(
            abs(measured - requested)
            for measured, requested in zip(final_right, right_target, strict=True)
        )
        if maximum_final_residual > FINAL_RESIDUAL_LIMIT_RAD:
            raise RuntimeError(
                "right calibration pose did not settle: "
                f"residual={maximum_final_residual:.6f} "
                f"limit={FINAL_RESIDUAL_LIMIT_RAD:.6f}"
            )
        print(
            "RESIDENT_RIGHT_CALIBRATION_READY_FOR_CAPTURE "
            f"capture={capture_id} residual_rad={maximum_final_residual:.6f} "
            f"epoch={final_epoch} soak_s={args.capture_soak_s:.1f}"
        )
        capture_skipped = False
        if capture_completion_file is None:
            time.sleep(args.capture_soak_s)
            ready_soak = status_document(node, status_client, args.timeout_s)
            require_armed_ready_hold(ready_soak, final_epoch)
        else:
            deadline = time.monotonic() + args.capture_soak_s
            ready_soak = status_document(node, status_client, args.timeout_s)
            while not capture_completion_file.is_file():
                require_armed_ready_hold(ready_soak, final_epoch)
                if capture_skip_file is not None and capture_skip_file.is_file():
                    capture_skipped = True
                    print(
                        "RESIDENT_RIGHT_CALIBRATION_CAPTURE_VISIBILITY_SKIPPED "
                        f"signal={capture_skip_file}"
                    )
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "capture completion file did not appear before the "
                        f"bounded hold timeout: {capture_completion_file}"
                    )
                time.sleep(0.2)
                ready_soak = status_document(
                    node,
                    status_client,
                    args.timeout_s,
                )
            require_armed_ready_hold(ready_soak, final_epoch)
            if not capture_skipped:
                print(
                    "RESIDENT_RIGHT_CALIBRATION_CAPTURE_FILE_CONFIRMED "
                    f"path={capture_completion_file}"
                )

        stopped = None
        if args.leave_torque_hold_active:
            final = status_document(node, status_client, args.timeout_s)
            require_armed_ready_hold(final, final_epoch)
        else:
            stop_request = BimanualStreamCommand.Request()
            stop_request.operation = BimanualStreamCommand.Request.STOP
            stop_request.owner = OWNER
            stopped = call(node, command_client, stop_request, args.timeout_s)
            stop_accepted = bool(stopped.accepted)
            if not stopped.accepted or stopped.adapter_state != "stopped":
                raise RuntimeError(f"coordinated stop rejected: {stopped}")
            final = status_document(node, status_client, args.timeout_s)
        document = {
            "schema_version": 1,
            "record_kind": "resident_right_calibration_pose_once",
            "overall_verdict": (
                "RESIDENT_RIGHT_CALIBRATION_POSE_VISIBILITY_SKIPPED_HOLD_ACTIVE"
                if capture_skipped and args.leave_torque_hold_active
                else "RESIDENT_RIGHT_CALIBRATION_POSE_HOLD_ACTIVE"
                if args.leave_torque_hold_active
                else "RESIDENT_RIGHT_CALIBRATION_POSE_PASS"
            ),
            "target_capture": str(args.target_capture.resolve()),
            "target_capture_id": capture_id,
            "resumed_held_session": args.resume_held_session,
            "starting_epoch": starting_epoch,
            "final_epoch": final_epoch,
            "initial_positions_rad": list(anchor),
            "requested_right_positions_rad": list(right_target),
            "terminal_right_positions_rad": list(final_right),
            "maximum_total_delta_rad": maximum_total_delta,
            "maximum_subleg_delta_rad": MAXIMUM_SUBLEG_DELTA_RAD,
            "maximum_final_residual_rad": maximum_final_residual,
            "capture_soak_s": args.capture_soak_s,
            "capture_completion_file": (
                str(capture_completion_file)
                if capture_completion_file is not None
                else None
            ),
            "capture_skip_file": (
                str(capture_skip_file)
                if capture_skip_file is not None
                else None
            ),
            "capture_visibility_skipped": capture_skipped,
            "legs": legs,
            "ready_soak_status": ready_soak,
            "stop_response": (
                response_document(stopped) if stopped is not None else None
            ),
            "final_status": final,
            "coordinated_stop_verified": stopped is not None,
            "torque_hold_left_active": args.leave_torque_hold_active,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        if args.leave_torque_hold_active:
            hold_left_active = True
            hold_verdict = (
                "RESIDENT_RIGHT_CALIBRATION_POSE_"
                "VISIBILITY_SKIPPED_HOLD_ACTIVE"
                if capture_skipped
                else "RESIDENT_RIGHT_CALIBRATION_POSE_HOLD_ACTIVE"
            )
            print(
                f"{hold_verdict} capture={capture_id} "
                f"epoch={final_epoch} owner={OWNER} "
                f"output={args.output} sha256={digest}"
            )
            return 0
        print(
            "RESIDENT_RIGHT_CALIBRATION_POSE_PASS "
            f"capture={capture_id} sublegs={len(subtargets)} state={final['state']} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        if (
            (motion_request_sent or args.resume_held_session)
            and not stop_accepted
            and not hold_left_active
        ):
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
