#!/usr/bin/env python3
"""Validate F7 identity, automatic 12-axis anchoring, and zero DMA output."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "F7_BIMANUAL_DMA_NO_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00024604
EXPECTED_PROTOCOL_VERSION = 2
EXPECTED_JOINT_COUNT = 12
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0xEFFFFFFF
HOST_BAUD = 921_600


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/f7/2026-08-14/no_output_run01.json",
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
            raise RuntimeError(f"unexpected F7 identity: {hello}")

        start = transport.heartbeat()
        before = transport.get_dispatch_diagnostics()
        snapshot = transport.prepare_shadow()
        after = transport.get_dispatch_diagnostics()
        end = transport.get_state()

        if (
            start.stop_latched
            or start.status_code != 0
            or snapshot.status_code != 0
            or snapshot.joint_count != EXPECTED_JOINT_COUNT
            or snapshot.left_present_mask != 0x3F
            or snapshot.right_present_mask != 0x3F
            or len(snapshot.positions_raw) != EXPECTED_JOINT_COUNT
            or len(snapshot.unwrapped_positions_raw) != EXPECTED_JOINT_COUNT
            or len(snapshot.anchor_positions_urad) != EXPECTED_JOINT_COUNT
        ):
            raise RuntimeError(f"automatic F7 anchor failed: {snapshot}")
        for label, diagnostics in (("before", before), ("after", after)):
            if (
                diagnostics.status_code != 0
                or diagnostics.active
                or diagnostics.faulted
                or not diagnostics.ready
                or diagnostics.launch_count != 0
                or diagnostics.completed_count != 0
                or diagnostics.failure_count != 0
            ):
                raise RuntimeError(
                    f"{label} dispatch state does not prove zero output: "
                    f"{diagnostics}"
                )
        if end.stop_latched or end.status_code != 0:
            raise RuntimeError(f"unhealthy final state: {end}")

        document = {
            "schema_version": 1,
            "record_kind": "f7_bimanual_dma_no_output",
            "overall_verdict": "F7_BIMANUAL_DMA_NO_OUTPUT_PASS",
            "motion_authorized": False,
            "torque_enabled": False,
            "dma_launches": 0,
            "automatic_operational_limit_branch_binding": True,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "state_start": asdict(start),
            "dispatch_before": asdict(before),
            "shadow": asdict(snapshot),
            "dispatch_after": asdict(after),
            "state_end": asdict(end),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "F7_BIMANUAL_DMA_NO_OUTPUT_PASS "
            f"firmware=0x{hello.firmware_version:08X} "
            f"raw={list(snapshot.positions_raw)} "
            f"unwrapped={list(snapshot.unwrapped_positions_raw)} "
            f"launches={after.launch_count} output={args.output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
