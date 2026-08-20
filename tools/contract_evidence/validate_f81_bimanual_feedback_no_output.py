#!/usr/bin/env python3
"""Validate the F8.1 measured feedback snapshot with all torque disabled."""

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
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "F81_BIMANUAL_FEEDBACK_NO_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00024800
EXPECTED_PROTOCOL_VERSION = 2
EXPECTED_JOINT_COUNT = 12
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0xEFFFFFFF
HOST_BAUD = 921_600
COMPLETE_MASK = 0x0FFF


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root
            / "artifacts/f81/2026-08-14/feedback_no_output_run01.json"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; this check keeps both arms torque-disabled"
        )

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    try:
        transport = StreamValidationTransportV2(port)
        hello = transport.enter_binary_mode()
        if (
            hello.firmware_version != EXPECTED_FIRMWARE_VERSION
            or hello.protocol_version != EXPECTED_PROTOCOL_VERSION
            or hello.joint_count != EXPECTED_JOINT_COUNT
            or hello.left_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.right_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.capabilities != EXPECTED_CAPABILITIES
            or hello.stop_latched
        ):
            raise RuntimeError(f"unexpected F8.1 identity: {hello}")

        start = transport.heartbeat()
        dispatch_before = transport.get_dispatch_diagnostics()
        shadow = transport.prepare_shadow()
        first = transport.get_feedback_snapshot()
        time.sleep(0.05)
        second = transport.get_feedback_snapshot()
        dispatch_after = transport.get_dispatch_diagnostics()
        end = transport.get_state()

        if (
            start.stop_latched
            or start.status_code != 0
            or shadow.status_code != 0
            or shadow.joint_count != EXPECTED_JOINT_COUNT
            or shadow.left_present_mask != 0x3F
            or shadow.right_present_mask != 0x3F
        ):
            raise RuntimeError(f"F8.1 shadow preparation failed: {shadow}")
        for label, snapshot in (("first", first), ("second", second)):
            if (
                snapshot.status_code != 0
                or snapshot.joint_count != EXPECTED_JOINT_COUNT
                or snapshot.present_mask != COMPLETE_MASK
                or len(snapshot.positions_urad) != EXPECTED_JOINT_COUNT
                or len(snapshot.sample_age_ms) != EXPECTED_JOINT_COUNT
            ):
                raise RuntimeError(f"{label} feedback is incomplete: {snapshot}")
        if first.positions_urad != shadow.anchor_positions_urad:
            raise RuntimeError("feedback does not match the measured shadow anchor")
        if second.positions_urad != first.positions_urad:
            raise RuntimeError("torque-off feedback changed without a new sample")
        if any(
            later < earlier
            for earlier, later in zip(
                first.sample_age_ms,
                second.sample_age_ms,
                strict=True,
            )
        ):
            raise RuntimeError("feedback sample age regressed")
        for label, dispatch in (
            ("before", dispatch_before),
            ("after", dispatch_after),
        ):
            if (
                dispatch.status_code != 0
                or dispatch.active
                or dispatch.faulted
                or not dispatch.ready
                or dispatch.launch_count != 0
                or dispatch.completed_count != 0
                or dispatch.failure_count != 0
            ):
                raise RuntimeError(
                    f"{label} dispatch does not prove zero output: {dispatch}"
                )
        if end.stop_latched or end.status_code != 0:
            raise RuntimeError(f"unhealthy final state: {end}")

        document = {
            "schema_version": 1,
            "record_kind": "f81_bimanual_feedback_no_output",
            "overall_verdict": "F81_BIMANUAL_FEEDBACK_NO_OUTPUT_PASS",
            "motion_authorized": False,
            "torque_enabled": False,
            "dma_launches": 0,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "state_start": asdict(start),
            "dispatch_before": asdict(dispatch_before),
            "shadow": asdict(shadow),
            "feedback_first": asdict(first),
            "feedback_second": asdict(second),
            "dispatch_after": asdict(dispatch_after),
            "state_end": asdict(end),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "F81_BIMANUAL_FEEDBACK_NO_OUTPUT_PASS "
            f"firmware=0x{hello.firmware_version:08X} "
            f"present_mask=0x{second.present_mask:03X} "
            f"sample_age_ms={list(second.sample_age_ms)} "
            f"launches={dispatch_after.launch_count} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
