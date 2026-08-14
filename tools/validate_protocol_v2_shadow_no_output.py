#!/usr/bin/env python3
"""Validate real dual-arm feedback as a v2 anchor without servo goal output."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_protocol_v2 import (
    ARM_MASK_BOTH,
    StreamBatchV2,
    StreamExecutorStateV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamStatusCodeV2,
    StreamTerminalReasonV2,
)
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "PROTOCOL_V2_SHADOW_BOTH_ARMS_TORQUE_OFF_NO_GOAL_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00023F01
EXPECTED_CAPABILITIES = 0x07802000
EXPECTED_CALIBRATION_HASH = 0x2D90167E
HOST_BAUD = 921_600
RAW_UNITS_PER_TURN = 4096
TURN_URAD = 6_283_185
FEEDBACK_LIMIT_MARGIN_RAW = 30


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--left-calibration",
        type=Path,
        default=root
        / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--right-calibration",
        type=Path,
        default=root / "config/right_arm_calibration.candidate.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/protocol_v2/shadow_no_output.json",
    )
    return parser.parse_args()


def tick(base: int, offset: int) -> int:
    return (base + offset) & 0xFFFFFFFF


def policy(horizon: int) -> StreamPolicyV2:
    return StreamPolicyV2(
        minimum_start_samples=2,
        minimum_lead_ms=20,
        horizon_end_tick=horizon,
        maximum_lead_ms=400,
        command_timeout_ms=500,
        maximum_apply_lateness_ms=5,
        tracking_error_limit_urad=(90_000,) * 12,
        maximum_step_urad_per_tick=(9_000,) * 12,
        arm_mask=ARM_MASK_BOTH,
    )


def require_accepted(status: object, label: str) -> None:
    values = asdict(status)  # type: ignore[arg-type]
    if values["status_code"] != StreamStatusCodeV2.OK:
        raise RuntimeError(f"{label} rejected: {status}")


def round_divide(numerator: int, denominator: int) -> int:
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def expected_anchor(
    left_path: Path,
    right_path: Path,
    positions_raw: tuple[int, ...],
) -> tuple[int, ...]:
    documents = tuple(
        json.loads(path.read_text(encoding="utf-8"))
        for path in (left_path, right_path)
    )
    output: list[int] = []
    for arm, (document, slot) in enumerate(zip(documents, ("left", "right"))):
        if document.get("arm_slot") != slot:
            raise RuntimeError(f"{slot} calibration has the wrong arm_slot")
        joints = document.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            raise RuntimeError(f"{slot} calibration must contain six joints")
        for joint, raw in zip(joints, positions_raw[arm * 6 : arm * 6 + 6]):
            feedback_minimum = max(
                0, joint["minimum_raw"] - FEEDBACK_LIMIT_MARGIN_RAW
            )
            feedback_maximum = min(
                4095, joint["maximum_raw"] + FEEDBACK_LIMIT_MARGIN_RAW
            )
            if not feedback_minimum <= raw <= feedback_maximum:
                raise RuntimeError(
                    f"{slot} servo {joint['id']} raw position is outside "
                    "the bounded feedback margin"
                )
            positive_delta = (
                (raw - joint["zero_raw"]) * joint["positive_raw_direction"]
            )
            output.append(
                round_divide(positive_delta * TURN_URAD, RAW_UNITS_PER_TURN)
            )
    return tuple(output)


def expected_executor_anchor(
    left_path: Path,
    right_path: Path,
    positions_raw: tuple[int, ...],
) -> tuple[int, ...]:
    documents = tuple(
        json.loads(path.read_text(encoding="utf-8"))
        for path in (left_path, right_path)
    )
    output: list[int] = []
    for arm, document in enumerate(documents):
        for joint, raw in zip(
            document["joints"], positions_raw[arm * 6 : arm * 6 + 6]
        ):
            command_raw = min(
                joint["maximum_raw"], max(joint["minimum_raw"], raw)
            )
            positive_delta = (
                (command_raw - joint["zero_raw"])
                * joint["positive_raw_direction"]
            )
            output.append(
                round_divide(positive_delta * TURN_URAD, RAW_UNITS_PER_TURN)
            )
    return tuple(output)


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("confirmation mismatch; this gate requires torque-off shadow")

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    try:
        transport = StreamValidationTransportV2(port)
        hello = transport.enter_binary_mode()
        if (
            hello.firmware_version != EXPECTED_FIRMWARE_VERSION
            or hello.protocol_version != 2
            or hello.joint_count != 12
            or hello.left_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.right_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.capabilities != EXPECTED_CAPABILITIES
            or hello.stop_latched
        ):
            raise RuntimeError(f"unexpected shadow identity: {hello}")

        start = transport.heartbeat()
        snapshot = transport.prepare_shadow()
        if (
            snapshot.status_code != 0
            or snapshot.left_present_mask != 0x3F
            or snapshot.right_present_mask != 0x3F
            or len(snapshot.positions_raw) != 12
            or len(snapshot.anchor_positions_urad) != 12
        ):
            raise RuntimeError(f"shadow preparation failed: {snapshot}")
        independently_computed_anchor = expected_anchor(
            args.left_calibration,
            args.right_calibration,
            snapshot.positions_raw,
        )
        if snapshot.anchor_positions_urad != independently_computed_anchor:
            raise RuntimeError(
                "firmware shadow anchor does not match host calibration: "
                f"firmware={snapshot.anchor_positions_urad} "
                f"host={independently_computed_anchor}"
            )
        executor_anchor = expected_executor_anchor(
            args.left_calibration,
            args.right_calibration,
            snapshot.positions_raw,
        )

        base = transport.heartbeat().last_heartbeat_ms
        horizon = tick(base, 180)
        opened = transport.open_stream(policy(horizon))
        require_accepted(opened, "shadow_open")
        apply_ticks = tuple(tick(base, offset) for offset in (120, 150, 180))
        appended = transport.append(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=31,
                samples=tuple(
                    StreamSampleV2(value, executor_anchor)
                    for value in apply_ticks
                ),
            )
        )
        require_accepted(appended, "shadow_append")
        time.sleep(0.24)
        terminal = transport.get_executor_diagnostics()
        if (
            terminal.state is not StreamExecutorStateV2.SUCCEEDED
            or terminal.terminal_reason
            is not StreamTerminalReasonV2.PLANNED_HORIZON
            or terminal.accepted_samples != 3
            or terminal.applied_samples != 3
            or terminal.safe_stop_required
        ):
            raise RuntimeError(f"shadow terminal mismatch: {terminal}")

        end = transport.get_state()
        rejected_delta = (
            end.rejected_frame_count - start.rejected_frame_count
        ) & 0xFFFFFFFF
        if end.stop_latched or end.status_code != 0 or rejected_delta != 0:
            raise RuntimeError(
                "shadow final state mismatch: "
                f"latched={int(end.stop_latched)} status={end.status_code} "
                f"rejected_delta={rejected_delta}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "protocol_v2_real_feedback_shadow_no_output",
            "overall_verdict": "PROTOCOL_V2_SHADOW_NO_OUTPUT_PASS",
            "motion_authorized": False,
            "executor_goal_output_connected": False,
            "executor_output_discarded": True,
            "torque_off_verified_before_capture": True,
            "feedback_sampling": "left_then_right_sequential",
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "snapshot": asdict(snapshot),
            "independently_computed_anchor_urad": independently_computed_anchor,
            "strict_limit_executor_anchor_urad": executor_anchor,
            "calibrations": {
                "left": {
                    "path": str(args.left_calibration),
                    "sha256": hashlib.sha256(
                        args.left_calibration.read_bytes()
                    ).hexdigest(),
                },
                "right": {
                    "path": str(args.right_calibration),
                    "sha256": hashlib.sha256(
                        args.right_calibration.read_bytes()
                    ).hexdigest(),
                },
            },
            "open": asdict(opened),
            "append": asdict(appended),
            "terminal": asdict(terminal),
            "state_end": asdict(end),
            "rejected_frame_delta": rejected_delta,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "PROTOCOL_V2_SHADOW_NO_OUTPUT_PASS "
            f"firmware=0x{hello.firmware_version:08X} "
            f"raw={list(snapshot.positions_raw)} "
            f"anchor_urad={list(snapshot.anchor_positions_urad)} "
            f"applied={terminal.applied_samples} "
            f"lateness_ms={terminal.maximum_apply_lateness_ms} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
