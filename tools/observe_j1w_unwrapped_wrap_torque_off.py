#!/usr/bin/env python3
"""Observe both shoulder wrap crossings through the J1-W no-output route."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import serial

from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.joint_unwrap import JointUnwrapper
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "OBSERVE_J1W_BOTH_SHOULDER_WRAPS_TORQUE_OFF_NO_OUTPUT"
EXPECTED_FIRMWARE_VERSION = 0x00024000
EXPECTED_CAPABILITIES = 0x0F802000
EXPECTED_CALIBRATION_HASH = 0x2D90167E
HOST_BAUD = 921_600
RAW_MODULUS = 4096
HALF_TURN_RAW = RAW_MODULUS // 2
SHOULDER_INDICES = (1, 7)
SHOULDER_NAMES = ("left_shoulder_joint", "right_shoulder_joint")


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--reference-sha256", required=True)
    parser.add_argument("--maximum-reference-delta-raw", type=int, default=32)
    parser.add_argument("--duration-s", type=float, default=90.0)
    parser.add_argument("--maximum-sample-step-raw", type=int, default=1024)
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/joint_ranges/j1w_both_shoulder_wraps.json",
    )
    return parser.parse_args()


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


def verify_snapshot_shape(snapshot: object) -> None:
    for name in (
        "positions_raw",
        "unwrapped_positions_raw",
        "anchor_positions_urad",
    ):
        values = tuple(getattr(snapshot, name))
        if len(values) != 12:
            raise RuntimeError(f"snapshot {name} must contain 12 values")
    if (
        snapshot.status_code != 0
        or snapshot.left_present_mask != 0x3F
        or snapshot.right_present_mask != 0x3F
    ):
        raise RuntimeError(f"J1-W snapshot failed: {snapshot}")


def main() -> int:
    args = parse_args()
    if args.confirmation != CONFIRMATION:
        raise SystemExit("confirmation mismatch")
    if not 1 <= args.maximum_reference_delta_raw < HALF_TURN_RAW:
        raise SystemExit("maximum reference delta must be within 1..2047")
    if not 15.0 <= args.duration_s <= 300.0:
        raise SystemExit("--duration-s must be within 15..300")
    if not 1 <= args.maximum_sample_step_raw < HALF_TURN_RAW:
        raise SystemExit("maximum sample step must be within 1..2047")

    references = load_reference(args.reference, args.reference_sha256)
    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    records: list[dict[str, object]] = []

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

        first = transport.prepare_shadow(
            references,
            args.maximum_reference_delta_raw,
        )
        verify_snapshot_shape(first)
        models = tuple(JointUnwrapper() for _ in range(12))
        for index, model in enumerate(models):
            expected = model.bind(
                first.positions_raw[index],
                references[index],
                args.maximum_reference_delta_raw,
            )
            if expected != first.unwrapped_positions_raw[index]:
                raise RuntimeError(
                    f"initial unwrap mismatch joint={index} "
                    f"host={expected} "
                    f"firmware={first.unwrapped_positions_raw[index]}"
                )

        previous_selected_raw = {
            index: first.positions_raw[index] for index in SHOULDER_INDICES
        }
        wrap_counts = {index: 0 for index in SHOULDER_INDICES}
        minima = {
            index: first.unwrapped_positions_raw[index]
            for index in SHOULDER_INDICES
        }
        maxima = dict(minima)
        maximum_steps = {index: 0 for index in SHOULDER_INDICES}
        sample_count = 1
        records.append(
            {
                "elapsed_s": 0.0,
                "raw": [
                    first.positions_raw[index] for index in SHOULDER_INDICES
                ],
                "unwrapped_raw": [
                    first.unwrapped_positions_raw[index]
                    for index in SHOULDER_INDICES
                ],
            }
        )
        print(
            "J1W_WRAP_OBSERVER_READY "
            f"duration_s={args.duration_s:.1f} "
            "joints=left_shoulder_joint,right_shoulder_joint "
            "torque=OFF output=DISCONNECTED; move each shoulder slowly "
            "across 4095<->0 at least once",
            flush=True,
        )

        started = time.monotonic()
        while time.monotonic() - started < args.duration_s:
            snapshot = transport.prepare_shadow()
            verify_snapshot_shape(snapshot)
            expected_unwrapped: list[int] = []
            for index, model in enumerate(models):
                expected = model.update(snapshot.positions_raw[index])
                expected_unwrapped.append(expected)
                if expected != snapshot.unwrapped_positions_raw[index]:
                    raise RuntimeError(
                        f"continuous unwrap mismatch joint={index} "
                        f"host={expected} "
                        f"firmware={snapshot.unwrapped_positions_raw[index]}"
                    )

            for index in SHOULDER_INDICES:
                raw = snapshot.positions_raw[index]
                raw_difference = raw - previous_selected_raw[index]
                if abs(raw_difference) > HALF_TURN_RAW:
                    wrap_counts[index] += 1
                step = abs(
                    snapshot.unwrapped_positions_raw[index]
                    - models[index].unwrapped_raw
                )
                # The model has already consumed the same sample, so derive
                # the physical step from consecutive stored records instead.
                if records:
                    previous_unwrapped = records[-1]["unwrapped_raw"][
                        SHOULDER_INDICES.index(index)
                    ]
                    step = abs(
                        snapshot.unwrapped_positions_raw[index]
                        - previous_unwrapped
                    )
                if step > args.maximum_sample_step_raw:
                    raise RuntimeError(
                        f"sample step too large joint={index} step={step}"
                    )
                maximum_steps[index] = max(maximum_steps[index], step)
                minima[index] = min(
                    minima[index], snapshot.unwrapped_positions_raw[index]
                )
                maxima[index] = max(
                    maxima[index], snapshot.unwrapped_positions_raw[index]
                )
                previous_selected_raw[index] = raw

            sample_count += 1
            records.append(
                {
                    "elapsed_s": round(time.monotonic() - started, 3),
                    "raw": [
                        snapshot.positions_raw[index]
                        for index in SHOULDER_INDICES
                    ],
                    "unwrapped_raw": [
                        snapshot.unwrapped_positions_raw[index]
                        for index in SHOULDER_INDICES
                    ],
                }
            )
            if sample_count == 2 or sample_count % 10 == 0:
                print(
                    "J1W_WRAP_SAMPLE "
                    f"count={sample_count} "
                    f"raw={records[-1]['raw']} "
                    f"unwrapped={records[-1]['unwrapped_raw']} "
                    f"wraps={[wrap_counts[index] for index in SHOULDER_INDICES]}",
                    flush=True,
                )

        missing = tuple(
            SHOULDER_NAMES[position]
            for position, index in enumerate(SHOULDER_INDICES)
            if wrap_counts[index] < 1
        )
        if missing:
            raise RuntimeError(
                "no physical wrap crossing observed for " + ",".join(missing)
            )

        end = transport.get_state()
        if end.stop_latched or end.status_code != 0:
            raise RuntimeError(f"J1-W terminal state failed: {end}")
        document = {
            "schema_version": 1,
            "record_kind": "j1w_both_shoulder_wrap_observation",
            "overall_verdict": "J1W_BOTH_SHOULDER_WRAP_NO_OUTPUT_PASS",
            "motion_authorized": False,
            "executor_goal_output_connected": False,
            "torque_off_verified_on_every_snapshot": True,
            "operator_motion_only": True,
            "device": device,
            "baud": HOST_BAUD,
            "hello": asdict(hello),
            "reference": {
                "path": str(args.reference),
                "sha256": args.reference_sha256.lower(),
                "values": references,
                "maximum_delta_raw": args.maximum_reference_delta_raw,
            },
            "sample_count": sample_count,
            "shoulders": {
                name: {
                    "index": index,
                    "wrap_count": wrap_counts[index],
                    "minimum_unwrapped_raw": minima[index],
                    "maximum_unwrapped_raw": maxima[index],
                    "maximum_sample_step_raw": maximum_steps[index],
                }
                for name, index in zip(
                    SHOULDER_NAMES, SHOULDER_INDICES, strict=True
                )
            },
            "state_end": asdict(end),
            "records": records,
        }
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        digest = file_sha256(output)
        print(
            "J1W_BOTH_SHOULDER_WRAP_NO_OUTPUT_PASS "
            f"samples={sample_count} "
            f"wraps={[wrap_counts[index] for index in SHOULDER_INDICES]} "
            f"output={output} sha256={digest}"
        )
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
