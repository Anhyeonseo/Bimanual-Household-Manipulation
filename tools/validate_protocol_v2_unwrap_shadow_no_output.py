#!/usr/bin/env python3
"""Validate explicit-branch J1-W shadow coordinates with no servo output."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.joint_unwrap import (
    JointUnwrapper,
    nearest_unwrapped_raw,
)
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


CONFIRMATION = "J1W_UNWRAP_SHADOW_BOTH_ARMS_TORQUE_OFF_NO_GOAL_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00024000
EXPECTED_CAPABILITIES = 0x0F802000
EXPECTED_CALIBRATION_HASH = 0x2D90167E
HOST_BAUD = 921_600
RAW_UNITS_PER_TURN = 4096
TURN_URAD = 6_283_185


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--reference-sha256", required=True)
    parser.add_argument(
        "--maximum-reference-delta-raw", type=int, default=32
    )
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
        default=root / "artifacts/protocol_v2/j1w_unwrap_shadow.json",
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


def load_reference(path: Path, expected_sha256: str) -> tuple[int, ...]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise RuntimeError(
            f"branch reference SHA mismatch expected={expected_sha256} "
            f"actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        document.get("status") != "J1W_BRANCH_REFERENCE_CAPTURE_PASS"
        or document.get("motion_authorized") is not False
    ):
        raise RuntimeError("branch reference artifact did not pass")
    values = tuple(document.get("reference_unwrapped_raw", ()))
    if len(values) != 12 or not all(isinstance(value, int) for value in values):
        raise RuntimeError("branch reference must contain 12 integers")
    return values


def load_joint_mappings(left_path: Path, right_path: Path) -> tuple[dict, ...]:
    output: list[dict] = []
    for path, arm_slot in ((left_path, "left"), (right_path, "right")):
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("arm_slot") != arm_slot:
            raise RuntimeError(f"{path}: wrong arm_slot")
        joints = document.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            raise RuntimeError(f"{path}: expected six joints")
        output.extend(joints)
    return tuple(output)


def expected_unwrapped_snapshot(
    raw: tuple[int, ...],
    references: tuple[int, ...],
    maximum_delta: int,
) -> tuple[int, ...]:
    return tuple(
        nearest_unwrapped_raw(value, reference, maximum_delta)
        for value, reference in zip(raw, references, strict=True)
    )


def expected_anchor(
    unwrapped_raw: tuple[int, ...], joints: tuple[dict, ...]
) -> tuple[int, ...]:
    return tuple(
        round_divide(
            (raw - joint["zero_raw"])
            * joint["positive_raw_direction"]
            * TURN_URAD,
            RAW_UNITS_PER_TURN,
        )
        for raw, joint in zip(unwrapped_raw, joints, strict=True)
    )


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("confirmation mismatch")
    if not 1 <= args.maximum_reference_delta_raw < 2048:
        raise SystemExit("maximum reference delta must be within 1..2047")
    references = load_reference(args.reference, args.reference_sha256)
    joints = load_joint_mappings(
        args.left_calibration, args.right_calibration
    )

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
            raise RuntimeError(f"unexpected J1-W identity: {hello}")

        start = transport.heartbeat()
        snapshot = transport.prepare_shadow(
            references,
            args.maximum_reference_delta_raw,
        )
        if (
            snapshot.status_code != 0
            or snapshot.left_present_mask != 0x3F
            or snapshot.right_present_mask != 0x3F
            or len(snapshot.positions_raw) != 12
            or len(snapshot.unwrapped_positions_raw) != 12
            or len(snapshot.anchor_positions_urad) != 12
        ):
            raise RuntimeError(f"J1-W shadow preparation failed: {snapshot}")
        host_unwrapped = expected_unwrapped_snapshot(
            snapshot.positions_raw,
            references,
            args.maximum_reference_delta_raw,
        )
        if snapshot.unwrapped_positions_raw != host_unwrapped:
            raise RuntimeError(
                "firmware unwrapped raw does not match host: "
                f"firmware={snapshot.unwrapped_positions_raw} "
                f"host={host_unwrapped}"
            )
        host_anchor = expected_anchor(host_unwrapped, joints)
        if snapshot.anchor_positions_urad != host_anchor:
            raise RuntimeError(
                "firmware unwrapped anchor does not match host: "
                f"firmware={snapshot.anchor_positions_urad} "
                f"host={host_anchor}"
            )
        if any(
            unwrapped % RAW_UNITS_PER_TURN != raw
            for raw, unwrapped in zip(
                snapshot.positions_raw,
                snapshot.unwrapped_positions_raw,
                strict=True,
            )
        ):
            raise RuntimeError("unwrapped raw modulo does not match feedback")

        models = tuple(JointUnwrapper() for _ in range(12))
        for index, model in enumerate(models):
            model.bind(
                snapshot.positions_raw[index],
                references[index],
                args.maximum_reference_delta_raw,
            )
        continuous_update_samples = 3
        for _ in range(continuous_update_samples):
            snapshot = transport.prepare_shadow()
            if (
                snapshot.status_code != 0
                or snapshot.left_present_mask != 0x3F
                or snapshot.right_present_mask != 0x3F
            ):
                raise RuntimeError(
                    f"J1-W continuous update failed: {snapshot}"
                )
            host_unwrapped = tuple(
                model.update(raw)
                for model, raw in zip(
                    models, snapshot.positions_raw, strict=True
                )
            )
            if snapshot.unwrapped_positions_raw != host_unwrapped:
                raise RuntimeError(
                    "continuous unwrap mismatch: "
                    f"firmware={snapshot.unwrapped_positions_raw} "
                    f"host={host_unwrapped}"
                )
            host_anchor = expected_anchor(host_unwrapped, joints)
            if snapshot.anchor_positions_urad != host_anchor:
                raise RuntimeError(
                    "continuous anchor mismatch: "
                    f"firmware={snapshot.anchor_positions_urad} "
                    f"host={host_anchor}"
                )

        base = transport.heartbeat().last_heartbeat_ms
        horizon = tick(base, 180)
        opened = transport.open_stream(policy(horizon))
        require_accepted(opened, "j1w_shadow_open")
        apply_ticks = tuple(tick(base, offset) for offset in (120, 150, 180))
        appended = transport.append(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=41,
                samples=tuple(
                    StreamSampleV2(value, host_anchor)
                    for value in apply_ticks
                ),
            )
        )
        require_accepted(appended, "j1w_shadow_append")
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
            raise RuntimeError(f"J1-W shadow terminal mismatch: {terminal}")

        end = transport.get_state()
        rejected_delta = (
            end.rejected_frame_count - start.rejected_frame_count
        ) & 0xFFFFFFFF
        if end.stop_latched or end.status_code != 0 or rejected_delta != 0:
            raise RuntimeError(
                "J1-W final state mismatch: "
                f"latched={int(end.stop_latched)} status={end.status_code} "
                f"rejected_delta={rejected_delta}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "protocol_v2_unwrapped_feedback_shadow_no_output",
            "overall_verdict": "J1W_UNWRAP_SHADOW_NO_OUTPUT_PASS",
            "motion_authorized": False,
            "executor_goal_output_connected": False,
            "executor_output_discarded": True,
            "torque_off_verified_before_capture": True,
            "branch_reference": {
                "path": str(args.reference),
                "sha256": args.reference_sha256.lower(),
                "maximum_delta_raw": args.maximum_reference_delta_raw,
                "values": references,
            },
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "snapshot": asdict(snapshot),
            "independently_computed_unwrapped_raw": host_unwrapped,
            "independently_computed_anchor_urad": host_anchor,
            "continuous_update_samples": continuous_update_samples,
            "open": asdict(opened),
            "append": asdict(appended),
            "terminal": asdict(terminal),
            "state_end": asdict(end),
            "rejected_frame_delta": rejected_delta,
        }
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(output)
        print(
            "J1W_UNWRAP_SHADOW_NO_OUTPUT_PASS "
            f"firmware=0x{hello.firmware_version:08X} "
            f"raw={list(snapshot.positions_raw)} "
            f"unwrapped={list(snapshot.unwrapped_positions_raw)} "
            f"applied={terminal.applied_samples} "
            f"lateness_ms={terminal.maximum_apply_lateness_ms} "
            f"output={output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
