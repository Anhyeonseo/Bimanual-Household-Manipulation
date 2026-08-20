#!/usr/bin/env python3
"""Run one small same-sign bimanual base-joint roundtrip from the measured pose."""

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


CONFIRMATION = "F7_BIMANUAL_BASE_SMALL_ROUNDTRIP_ONCE"
EXPECTED_FIRMWARE_VERSION = 0x00024604
EXPECTED_CALIBRATION_HASH = 0x2D90167E
EXPECTED_CAPABILITIES = 0xEFFFFFFF
HOST_BAUD = 921_600
JOINT_COUNT = 12


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--delta-rad", type=float, default=0.03)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/f7/2026-08-14/base_small_roundtrip_run01.json",
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
    if not 0.0 < args.delta_rad <= 0.03:
        raise SystemExit("delta-rad must be in (0, 0.03]")
    delta_urad = int(round(args.delta_rad * 1_000_000.0))

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
            raise RuntimeError(f"unexpected F7 identity: {hello}")

        shadow = transport.prepare_shadow()
        if (
            shadow.status_code != 0
            or shadow.left_present_mask != 0x3F
            or shadow.right_present_mask != 0x3F
            or len(shadow.anchor_positions_urad) != JOINT_COUNT
        ):
            raise RuntimeError(f"automatic anchor failed: {shadow}")
        anchor = tuple(shadow.anchor_positions_urad)
        target = list(anchor)
        target[0] += delta_urad
        target[6] += delta_urad
        target = tuple(target)
        before = transport.get_dispatch_diagnostics()
        if before.faulted or before.active or not before.ready:
            raise RuntimeError(f"dispatch is not ready: {before}")

        prearm = transport.heartbeat()
        if prearm.stop_latched or prearm.status_code != 0:
            raise RuntimeError(f"pre-arm heartbeat failed: {prearm}")

        print(
            "F7_BIMANUAL_BASE_SMALL_ROUNDTRIP_PRECHECK_PASS "
            f"delta_rad={args.delta_rad:.6f} delta_urad={delta_urad} "
            f"left={anchor[0]}->{target[0]} right={anchor[6]}->{target[6]}"
        )
        transport.arm(EXPECTED_CALIBRATION_HASH)
        armed = True
        postarm = transport.heartbeat()
        if postarm.stop_latched or postarm.status_code != 0:
            raise RuntimeError(f"post-arm heartbeat failed: {postarm}")
        enabled = transport.enable()
        base = transport.heartbeat().last_heartbeat_ms
        horizon = tick(base, 360)
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
        sample_ticks = tuple(tick(base, offset) for offset in (80, 220, 360))
        appended = transport.append(
            StreamBatchV2(
                horizon_end_tick=horizon,
                arbiter_epoch=1,
                samples=tuple(
                    StreamSampleV2(
                        apply_tick=sample_tick,
                        positions_urad=(anchor, target, anchor)[index],
                    )
                    for index, sample_tick in enumerate(sample_ticks)
                ),
                arm_mask=ARM_MASK_BOTH,
            )
        )
        require_accepted(appended, "append")

        deadline = time.monotonic() + 0.52
        while time.monotonic() < deadline:
            time.sleep(0.025)
            state = transport.heartbeat()
            if state.stop_latched or state.status_code != 0:
                failure_executor = transport.get_executor_diagnostics()
                failure_dispatch = transport.get_dispatch_diagnostics()
                raise RuntimeError(
                    "roundtrip heartbeat failed: "
                    f"state={state}; executor={failure_executor}; "
                    f"dispatch={failure_dispatch}"
                )

        executor = transport.get_executor_diagnostics()
        dispatch = transport.get_dispatch_diagnostics()
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

        stopped = transport.safe_stop()
        armed = False
        after_stop = transport.get_dispatch_diagnostics()
        if (
            not stopped.stop_latched
            or stopped.status_code != 0
            or after_stop.failure_count != dispatch.failure_count
        ):
            raise RuntimeError(
                f"normal coordinated stop mismatch: {stopped}, {after_stop}"
            )

        document = {
            "schema_version": 1,
            "record_kind": "f7_bimanual_base_small_roundtrip_once",
            "overall_verdict": "F7_BIMANUAL_BASE_SMALL_ROUNDTRIP_ONCE_PASS",
            "automatic_motion": True,
            "commanded_peak_delta_urad": [
                delta_urad if index in (0, 6) else 0
                for index in range(JOINT_COUNT)
            ],
            "target_positions_urad": target,
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
            "safe_stop": asdict(stopped),
            "dispatch_after_stop": asdict(after_stop),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        print(
            "F7_BIMANUAL_BASE_SMALL_ROUNDTRIP_ONCE_PASS "
            f"delta_rad={args.delta_rad:.6f} "
            f"launches={dispatch.launch_count - before.launch_count} "
            f"completed={dispatch.completed_count - before.completed_count} "
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
