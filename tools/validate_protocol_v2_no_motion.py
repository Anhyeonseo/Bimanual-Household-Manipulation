#!/usr/bin/env python3
"""Validate protocol-v2 framing and stream semantics with servo 12 V off."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_protocol_v2 import (
    ARM_MASK_BOTH,
    StreamBatchV2,
    StreamContractResultV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamStatusCodeV2,
)
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "PROTOCOL_V2_VALIDATE_NO_MOTION_BOTH_SERVO_12V_OFF"
EXPECTED_FIRMWARE_VERSION = 0x00023D00
EXPECTED_PROTOCOL_VERSION = 2
EXPECTED_JOINT_COUNT = 12
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0x01802000
HOST_BAUD = 921_600
SAFE_DISABLED_STATE = 1
ZERO_POSITIONS = (0,) * EXPECTED_JOINT_COUNT


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/protocol_v2/validation_no_motion.json",
    )
    return parser.parse_args()


def _tick(base: int, offset: int) -> int:
    return (base + offset) & 0xFFFFFFFF


def _status_document(status: object) -> dict[str, object]:
    return asdict(status)  # type: ignore[arg-type]


def _require_no_motion(status: object, label: str) -> None:
    values = asdict(status)  # type: ignore[arg-type]
    if values["status_code"] != StreamStatusCodeV2.VALIDATION_ONLY:
        raise RuntimeError(f"{label} was not accepted by validation-only route")
    if values["contract_result"] != StreamContractResultV2.OK:
        raise RuntimeError(f"{label} contract result was not OK")
    if values["safety_state"] != SAFE_DISABLED_STATE:
        raise RuntimeError(f"{label} left SAFE_DISABLED")
    for field in (
        "execution_queue_samples",
        "accepted_samples",
        "applied_samples",
    ):
        if values[field] != 0:
            raise RuntimeError(f"{label} lost no-motion proof: {field}")


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; both servo 12 V domains must be off"
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
            raise RuntimeError(f"unexpected protocol-v2 identity: {hello}")

        start = transport.heartbeat()
        if start.stop_latched or start.status_code != 0:
            raise RuntimeError("protocol-v2 candidate is unhealthy before test")
        base_tick = start.last_heartbeat_ms
        horizon = _tick(base_tick, 1_000)
        policy = StreamPolicyV2(
            minimum_start_samples=2,
            minimum_lead_ms=20,
            horizon_end_tick=horizon,
            maximum_lead_ms=400,
            command_timeout_ms=500,
            maximum_apply_lateness_ms=5,
            tracking_error_limit_urad=(90_000,) * EXPECTED_JOINT_COUNT,
            maximum_step_urad_per_tick=(9_000,) * EXPECTED_JOINT_COUNT,
            arm_mask=ARM_MASK_BOTH,
        )
        opened = transport.open_stream(policy)
        _require_no_motion(opened, "stream_open")

        append_ticks = tuple(_tick(base_tick, offset) for offset in (250, 270, 290))
        appended = transport.append(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=7,
                samples=tuple(
                    StreamSampleV2(tick, ZERO_POSITIONS)
                    for tick in append_ticks
                ),
            )
        )
        _require_no_motion(appended, "append")

        spliced = transport.splice(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=8,
                splice_at_tick=append_ticks[1],
                samples=(
                    StreamSampleV2(append_ticks[1], ZERO_POSITIONS),
                    StreamSampleV2(append_ticks[2], ZERO_POSITIONS),
                ),
            )
        )
        _require_no_motion(spliced, "splice")

        rejected = transport.open_stream(replace(policy, minimum_lead_ms=19))
        if (
            rejected.status_code != StreamStatusCodeV2.CONTRACT_REJECTED
            or rejected.contract_result
            != StreamContractResultV2.MINIMUM_LEAD_TOO_SMALL
            or rejected.execution_queue_samples != 0
            or rejected.accepted_samples != 0
            or rejected.applied_samples != 0
        ):
            raise RuntimeError("loosened minimum lead was not rejected safely")

        end = transport.get_state()
        rejected_delta = (
            end.rejected_frame_count - start.rejected_frame_count
        ) & 0xFFFFFFFF
        if end.stop_latched or end.status_code != 0 or rejected_delta != 1:
            raise RuntimeError(
                "protocol-v2 final state mismatch: "
                f"latched={int(end.stop_latched)} status={end.status_code} "
                f"rejected_delta={rejected_delta}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "protocol_v2_validation_no_motion",
            "overall_verdict": "PROTOCOL_V2_VALIDATION_NO_MOTION_PASS",
            "validation_only": True,
            "motion_authorized": False,
            "both_servo_12v_confirmed_off": True,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "heartbeat_start": asdict(start),
            "stream_open": _status_document(opened),
            "append": _status_document(appended),
            "splice": _status_document(spliced),
            "expected_rejection": _status_document(rejected),
            "state_end": asdict(end),
            "rejected_frame_delta": rejected_delta,
            "no_motion_proof": {
                "safety_state": SAFE_DISABLED_STATE,
                "execution_queue_samples": 0,
                "accepted_samples": 0,
                "applied_samples": 0,
            },
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "PROTOCOL_V2_VALIDATION_NO_MOTION_PASS "
            f"firmware=0x{hello.firmware_version:08X} protocol=2 joints=12 "
            f"open={opened.status_code} append={appended.status_code} "
            f"splice={spliced.status_code} rejected_delta={rejected_delta} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
