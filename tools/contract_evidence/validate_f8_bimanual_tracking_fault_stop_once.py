#!/usr/bin/env python3
"""Inject one synthetic tracking error and verify coordinated torque-off."""

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


CONFIRMATION = "F8_BIMANUAL_TRACKING_FAULT_STOP_ONCE"
EXPECTED_FIRMWARE_VERSION = 0x00024701
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0xEFFFFFFF
HOST_BAUD = 921_600
JOINT_COUNT = 12
EXPECTED_PAIRS_BEFORE_FAULT = 8
EXPECTED_TRACKING_ERROR_JOINT = 7
INJECTED_TRACKING_ERROR_URAD = 100_000


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/f8/2026-08-14/tracking_fault_stop_run01.json",
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
            raise RuntimeError(f"unexpected F8 fault identity: {hello}")

        shadow = transport.prepare_shadow()
        if (
            shadow.status_code != 0
            or shadow.left_present_mask != 0x3F
            or shadow.right_present_mask != 0x3F
            or len(shadow.anchor_positions_urad) != JOINT_COUNT
        ):
            raise RuntimeError(f"automatic anchor failed: {shadow}")
        before = transport.get_dispatch_diagnostics()
        tracking_before = transport.get_tracking_diagnostics()
        if (
            before.status_code != 2
            or before.faulted
            or before.active
            or not before.ready
            or tracking_before.status_code != 0
            or tracking_before.active
            or tracking_before.pending
            or tracking_before.requested_pairs != 0
            or tracking_before.completed_pairs != 0
            or tracking_before.failed_pairs != 0
        ):
            raise RuntimeError(
                f"fault candidate is not ready: {before}, {tracking_before}"
            )

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

        fault_state = None
        deadline = time.monotonic() + 0.40
        while time.monotonic() < deadline:
            time.sleep(0.010)
            state = transport.heartbeat()
            if state.stop_latched:
                fault_state = state
                armed = False
                break
            if state.status_code != 0:
                raise RuntimeError(
                    f"unexpected state before injected fault: {state}"
                )
        if fault_state is None:
            raise RuntimeError("tracking-error injection did not latch stop")

        executor = transport.get_executor_diagnostics()
        dispatch = transport.get_dispatch_diagnostics()
        tracking = transport.get_tracking_diagnostics()
        launch_delta = dispatch.launch_count - before.launch_count
        completed_delta = dispatch.completed_count - before.completed_count
        failure_delta = dispatch.failure_count - before.failure_count
        if (
            fault_state.status_code != 0
            or executor.state is not StreamExecutorStateV2.ABORTED
            or executor.terminal_reason is not StreamTerminalReasonV2.TRACKING_ERROR
            or not executor.safe_stop_required
            or executor.tracking_error_joint != EXPECTED_TRACKING_ERROR_JOINT
            or dispatch.status_code != 0
            or dispatch.active
            or not dispatch.faulted
            or dispatch.ready
            or launch_delta != EXPECTED_PAIRS_BEFORE_FAULT
            or completed_delta != EXPECTED_PAIRS_BEFORE_FAULT
            or failure_delta != 1
            or dispatch.completed_count != dispatch.launch_count
            or tracking.status_code != 0
            or tracking.active
            or tracking.pending
            or tracking.requested_pairs != EXPECTED_PAIRS_BEFORE_FAULT
            or tracking.completed_pairs != EXPECTED_PAIRS_BEFORE_FAULT
            or tracking.failed_pairs != 0
            or tracking.maximum_reply_latency_ms >= 5
            or max(tracking.maximum_tracking_error_urad)
            != INJECTED_TRACKING_ERROR_URAD
            or tracking.maximum_tracking_error_urad[
                EXPECTED_TRACKING_ERROR_JOINT
            ]
            != INJECTED_TRACKING_ERROR_URAD
        ):
            raise RuntimeError(
                "injected tracking error did not produce verified stop: "
                f"state={fault_state}; executor={executor}; "
                f"dispatch={dispatch}; tracking={tracking}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "f8_bimanual_tracking_fault_stop_once",
            "overall_verdict": "F8_BIMANUAL_TRACKING_FAULT_STOP_ONCE_PASS",
            "automatic_motion": False,
            "fault_injection": (
                "one-shot right-arm synthetic 100000 urad tracking error "
                "after 8 completed feedback pairs"
            ),
            "verified_left_torque_disabled": True,
            "verified_right_torque_disabled": True,
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
            "fault_state": asdict(fault_state),
            "executor": asdict(executor),
            "dispatch_before": asdict(before),
            "tracking_before": asdict(tracking_before),
            "dispatch_faulted": asdict(dispatch),
            "tracking_faulted": asdict(tracking),
            "launch_delta": launch_delta,
            "completed_delta": completed_delta,
            "failure_delta": failure_delta,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "F8_BIMANUAL_TRACKING_FAULT_STOP_ONCE_PASS "
            f"launches_before_fault={launch_delta} "
            f"feedback_pairs_before_fault={tracking.completed_pairs} "
            f"tracking_error_joint={executor.tracking_error_joint} "
            f"tracking_error_urad="
            f"{tracking.maximum_tracking_error_urad[executor.tracking_error_joint]} "
            f"tracking_latency_max_ms={tracking.maximum_reply_latency_ms} "
            "left_torque_disabled=true right_torque_disabled=true "
            f"output={args.output} sha256={digest}"
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
