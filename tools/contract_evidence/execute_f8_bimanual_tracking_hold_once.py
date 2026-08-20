#!/usr/bin/env python3
"""Send one finite absolute 12-axis stream that holds the measured pose."""

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
    StreamContractResultV2,
    StreamExecutorStateV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamStatusCodeV2,
    StreamTerminalReasonV2,
)
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "F8_BIMANUAL_TRACKING_HOLD_ONCE"
EXPECTED_FIRMWARE_VERSION = 0x00024700
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0xEFFFFFFF
HOST_BAUD = 921_600
JOINT_COUNT = 12


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/f8/2026-08-14/tracking_hold_run01.json",
    )
    return parser.parse_args()


def tick(base: int, offset: int) -> int:
    return (base + offset) & 0xFFFFFFFF


def require_accepted(status: object, label: str) -> None:
    values = asdict(status)  # type: ignore[arg-type]
    if (
        values["status_code"] != StreamStatusCodeV2.OK
        or values["contract_result"] != StreamContractResultV2.OK
        or values["arm_mask"] != ARM_MASK_BOTH
    ):
        raise RuntimeError(f"{label} rejected: {status}")


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit(
            "confirmation mismatch; support both arms and clear their workspace"
        )

    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    transport = StreamValidationTransportV2(port)
    armed = False
    stopped = None
    try:
        hello = transport.enter_binary_mode()
        if (
            hello.firmware_version != EXPECTED_FIRMWARE_VERSION
            or hello.protocol_version != 2
            or hello.joint_count != JOINT_COUNT
            or hello.left_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.right_calibration_hash != EXPECTED_CALIBRATION_HASH
            or hello.capabilities != EXPECTED_CAPABILITIES
            or hello.stop_latched
        ):
            raise RuntimeError(f"unexpected F8 identity: {hello}")

        shadow = transport.prepare_shadow()
        if (
            shadow.status_code != 0
            or shadow.left_present_mask != 0x3F
            or shadow.right_present_mask != 0x3F
            or len(shadow.anchor_positions_urad) != JOINT_COUNT
        ):
            raise RuntimeError(f"automatic anchor failed: {shadow}")
        before = transport.get_dispatch_diagnostics()
        if before.faulted or before.active or not before.ready:
            raise RuntimeError(f"dispatch is not ready: {before}")

        prearm = transport.heartbeat()
        if prearm.stop_latched or prearm.status_code != 0:
            raise RuntimeError(f"pre-arm heartbeat failed: {prearm}")

        transport.arm(EXPECTED_CALIBRATION_HASH)
        armed = True
        postarm = transport.heartbeat()
        if postarm.stop_latched or postarm.status_code != 0:
            raise RuntimeError(f"post-arm heartbeat failed: {postarm}")
        enabled = transport.enable()
        base = transport.heartbeat().last_heartbeat_ms
        horizon = tick(base, 180)
        policy = StreamPolicyV2(
            minimum_start_samples=2,
            minimum_lead_ms=20,
            horizon_end_tick=horizon,
            maximum_lead_ms=400,
            command_timeout_ms=500,
            maximum_apply_lateness_ms=5,
            tracking_error_limit_urad=(90_000,) * JOINT_COUNT,
            maximum_step_urad_per_tick=(9_000,) * JOINT_COUNT,
            arm_mask=ARM_MASK_BOTH,
        )
        opened = transport.open_stream(policy)
        require_accepted(opened, "open")
        sample_ticks = tuple(tick(base, offset) for offset in (80, 130, 180))
        appended = transport.append(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=1,
                samples=tuple(
                    StreamSampleV2(
                        apply_tick=sample_tick,
                        positions_urad=shadow.anchor_positions_urad,
                    )
                    for sample_tick in sample_ticks
                ),
                arm_mask=ARM_MASK_BOTH,
            )
        )
        require_accepted(appended, "append")

        deadline = time.monotonic() + 0.32
        while time.monotonic() < deadline:
            time.sleep(0.025)
            state = transport.heartbeat()
            if state.stop_latched or state.status_code != 0:
                failure_executor = transport.get_executor_diagnostics()
                failure_dispatch = transport.get_dispatch_diagnostics()
                raise RuntimeError(
                    "hold heartbeat failed: "
                    f"state={state}; executor={failure_executor}; "
                    f"dispatch={failure_dispatch}"
                )

        executor = transport.get_executor_diagnostics()
        dispatch = transport.get_dispatch_diagnostics()
        tracking = transport.get_tracking_diagnostics()
        if (
            executor.state is not StreamExecutorStateV2.SUCCEEDED
            or executor.terminal_reason is not StreamTerminalReasonV2.PLANNED_HORIZON
            or executor.safe_stop_required
            or executor.applied_samples != 3
        ):
            raise RuntimeError(f"executor terminal mismatch: {executor}")
        if (
            dispatch.active
            or dispatch.faulted
            or not dispatch.ready
            or dispatch.launch_count <= before.launch_count
            or dispatch.completed_count != dispatch.launch_count
            or dispatch.failure_count != before.failure_count
            or dispatch.maximum_start_skew_us > 500
            or dispatch.maximum_launch_lateness_us >= 5_000
        ):
            raise RuntimeError(f"paired DMA diagnostics failed: {dispatch}")

        launch_delta = dispatch.launch_count - before.launch_count
        if (
            tracking.status_code != 0
            or not tracking.active
            or tracking.pending
            or tracking.requested_pairs != launch_delta
            or tracking.completed_pairs != launch_delta
            or tracking.failed_pairs != 0
            or tracking.maximum_reply_latency_ms >= 5
            or tracking.next_joint != launch_delta % 6
            or max(tracking.maximum_tracking_error_urad) > 90_000
        ):
            raise RuntimeError(f"route-time tracking diagnostics failed: {tracking}")

        stopped = transport.safe_stop()
        armed = False
        after_stop = transport.get_dispatch_diagnostics()
        tracking_after_stop = transport.get_tracking_diagnostics()
        if (
            not stopped.stop_latched
            or stopped.status_code != 0
            or after_stop.failure_count != dispatch.failure_count
            or tracking_after_stop.active
            or tracking_after_stop.pending
            or tracking_after_stop.completed_pairs != tracking.completed_pairs
        ):
            raise RuntimeError(
                f"normal coordinated stop mismatch: {stopped}, {after_stop}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "f8_bimanual_tracking_hold_once",
            "overall_verdict": "F8_BIMANUAL_TRACKING_HOLD_ONCE_PASS",
            "automatic_motion": False,
            "commanded_motion_delta_urad": [0] * JOINT_COUNT,
            "operator_confirmation": args.confirmation,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "shadow": asdict(shadow),
            "state_prearm": asdict(prearm),
            "state_postarm": asdict(postarm),
            "state_enable": asdict(enabled),
            "open": asdict(opened),
            "append": asdict(appended),
            "executor": asdict(executor),
            "dispatch_before": asdict(before),
            "dispatch_terminal": asdict(dispatch),
            "tracking_terminal": asdict(tracking),
            "safe_stop": asdict(stopped),
            "tracking_after_stop": asdict(tracking_after_stop),
            "dispatch_after_stop": asdict(after_stop),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "F8_BIMANUAL_TRACKING_HOLD_ONCE_PASS "
            f"launches={launch_delta} "
            f"completed={dispatch.completed_count - before.completed_count} "
            f"tracking_pairs={tracking.completed_pairs} "
            f"tracking_latency_max_ms={tracking.maximum_reply_latency_ms} "
            f"tracking_error_max_urad={max(tracking.maximum_tracking_error_urad)} "
            f"start_skew_max_us={dispatch.maximum_start_skew_us} "
            f"launch_lateness_max_us={dispatch.maximum_launch_lateness_us} "
            f"torque_disabled=true output={args.output} sha256={digest}"
        )
        return 0
    finally:
        if armed:
            try:
                transport.safe_stop()
            except Exception:
                pass
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
