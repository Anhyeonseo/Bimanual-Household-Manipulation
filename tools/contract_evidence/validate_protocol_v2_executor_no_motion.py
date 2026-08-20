#!/usr/bin/env python3
"""Validate the source-agnostic v2 executor with both servo 12 V rails off."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
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
    StreamContractResultV2,
    StreamExecutorDiagnosticsV2,
    StreamExecutorStateV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamStatusCodeV2,
    StreamTerminalReasonV2,
)
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "PROTOCOL_V2_EXECUTOR_NO_MOTION_BOTH_SERVO_12V_OFF"
EXPECTED_FIRMWARE_VERSION = 0x00023E00
EXPECTED_PROTOCOL_VERSION = 2
EXPECTED_JOINT_COUNT = 12
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0x03802000
HOST_BAUD = 921_600
SAFE_DISABLED_STATE = 1
ZERO_POSITIONS = (0,) * EXPECTED_JOINT_COUNT


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/protocol_v2/executor_no_motion.json",
    )
    return parser.parse_args()


def _tick(base: int, offset: int) -> int:
    return (base + offset) & 0xFFFFFFFF


def _policy(*, horizon: int, command_timeout_ms: int) -> StreamPolicyV2:
    return StreamPolicyV2(
        minimum_start_samples=2,
        minimum_lead_ms=20,
        horizon_end_tick=horizon,
        maximum_lead_ms=400,
        command_timeout_ms=command_timeout_ms,
        maximum_apply_lateness_ms=5,
        tracking_error_limit_urad=(90_000,) * EXPECTED_JOINT_COUNT,
        maximum_step_urad_per_tick=(9_000,) * EXPECTED_JOINT_COUNT,
        arm_mask=ARM_MASK_BOTH,
    )


def _require_accepted(status: object, label: str) -> None:
    values = asdict(status)  # type: ignore[arg-type]
    if values["status_code"] != StreamStatusCodeV2.OK:
        raise RuntimeError(
            f"{label} was not accepted by the executor route: {status}"
        )
    if values["contract_result"] != StreamContractResultV2.OK:
        raise RuntimeError(f"{label} contract result was not OK")
    if values["safety_state"] != SAFE_DISABLED_STATE:
        raise RuntimeError(f"{label} left SAFE_DISABLED")


def _require_terminal(
    diagnostics: StreamExecutorDiagnosticsV2,
    *,
    state: StreamExecutorStateV2,
    reason: StreamTerminalReasonV2,
    accepted: int,
    applied: int,
    splices: int,
    safe_stop_required: bool,
    label: str,
) -> None:
    if (
        diagnostics.state is not state
        or diagnostics.terminal_reason is not reason
        or diagnostics.accepted_samples != accepted
        or diagnostics.applied_samples != applied
        or diagnostics.splice_count != splices
        or diagnostics.queued_samples != 0
        or diagnostics.safe_stop_required is not safe_stop_required
        or diagnostics.control_outputs == 0
    ):
        raise RuntimeError(f"{label} terminal mismatch: {diagnostics}")


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
            raise RuntimeError(f"unexpected executor candidate identity: {hello}")

        start = transport.heartbeat()
        if start.stop_latched or start.status_code != 0:
            raise RuntimeError("executor candidate is unhealthy before test")

        finite_base = start.last_heartbeat_ms
        finite_horizon = _tick(finite_base, 220)
        finite_policy = _policy(
            horizon=finite_horizon,
            command_timeout_ms=500,
        )
        finite_open = transport.open_stream(finite_policy)
        _require_accepted(finite_open, "finite_open")
        finite_ticks = tuple(
            _tick(finite_base, offset) for offset in (140, 180, 220)
        )
        finite_append = transport.append(
            StreamBatchV2(
                horizon_end_tick=finite_horizon,
                arbiter_epoch=11,
                samples=tuple(
                    StreamSampleV2(tick, ZERO_POSITIONS)
                    for tick in finite_ticks
                ),
            )
        )
        _require_accepted(finite_append, "finite_append")
        finite_splice = transport.splice(
            StreamBatchV2(
                horizon_end_tick=finite_horizon,
                arbiter_epoch=12,
                splice_at_tick=finite_ticks[1],
                samples=(
                    StreamSampleV2(finite_ticks[1], ZERO_POSITIONS),
                    StreamSampleV2(finite_ticks[2], ZERO_POSITIONS),
                ),
            )
        )
        _require_accepted(finite_splice, "finite_splice")
        time.sleep(0.28)
        finite_terminal = transport.get_executor_diagnostics()
        _require_terminal(
            finite_terminal,
            state=StreamExecutorStateV2.SUCCEEDED,
            reason=StreamTerminalReasonV2.PLANNED_HORIZON,
            accepted=5,
            applied=3,
            splices=1,
            safe_stop_required=False,
            label="finite",
        )

        open_start = transport.heartbeat()
        open_base = open_start.last_heartbeat_ms
        # The firmware hard cap for an open stream is 100 ms.  Put both
        # samples comfortably inside that window; their ticks are based on
        # the heartbeat immediately before OPEN and APPEND.
        open_policy = _policy(horizon=0, command_timeout_ms=100)
        open_opened = transport.open_stream(open_policy)
        _require_accepted(open_opened, "open_stream")
        open_ticks = tuple(
            _tick(open_base, offset) for offset in (70, 90)
        )
        open_append = transport.append(
            StreamBatchV2(
                horizon_end_tick=0,
                arbiter_epoch=21,
                samples=tuple(
                    StreamSampleV2(tick, ZERO_POSITIONS)
                    for tick in open_ticks
                ),
            )
        )
        _require_accepted(open_append, "open_append")
        time.sleep(0.22)
        timeout_terminal = transport.get_executor_diagnostics()
        _require_terminal(
            timeout_terminal,
            state=StreamExecutorStateV2.HOLD,
            reason=StreamTerminalReasonV2.COMMAND_TIMEOUT,
            accepted=2,
            applied=2,
            splices=0,
            safe_stop_required=True,
            label="open_timeout",
        )

        rejected = transport.open_stream(
            replace(open_policy, minimum_lead_ms=19)
        )
        if (
            rejected.status_code != StreamStatusCodeV2.CONTRACT_REJECTED
            or rejected.contract_result
            != StreamContractResultV2.MINIMUM_LEAD_TOO_SMALL
        ):
            raise RuntimeError("loosened minimum lead was not rejected")

        end = transport.get_state()
        rejected_delta = (
            end.rejected_frame_count - start.rejected_frame_count
        ) & 0xFFFFFFFF
        if end.stop_latched or end.status_code != 0 or rejected_delta != 1:
            raise RuntimeError(
                "executor final state mismatch: "
                f"latched={int(end.stop_latched)} status={end.status_code} "
                f"rejected_delta={rejected_delta}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "protocol_v2_executor_no_motion",
            "overall_verdict": "PROTOCOL_V2_EXECUTOR_NO_MOTION_PASS",
            "validation_only": True,
            "synthetic_anchor": True,
            "discarded_executor_output": True,
            "motion_authorized": False,
            "both_servo_12v_confirmed_off": True,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "finite": {
                "open": asdict(finite_open),
                "append": asdict(finite_append),
                "splice": asdict(finite_splice),
                "terminal": asdict(finite_terminal),
            },
            "open_timeout": {
                "open": asdict(open_opened),
                "append": asdict(open_append),
                "terminal": asdict(timeout_terminal),
            },
            "expected_rejection": asdict(rejected),
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
            "PROTOCOL_V2_EXECUTOR_NO_MOTION_PASS "
            f"firmware=0x{hello.firmware_version:08X} "
            f"finite={finite_terminal.state.name}/"
            f"{finite_terminal.applied_samples} "
            f"open={timeout_terminal.state.name}/"
            f"{timeout_terminal.applied_samples} "
            f"rejected_delta={rejected_delta} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
