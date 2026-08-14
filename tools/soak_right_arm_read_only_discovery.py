#!/usr/bin/env python3
"""Repeat the right-arm discovery service without issuing any write operation.

The service is firmware-gated to request only STS Present_Position reads.  This
tool never calls ARM, ENABLE, DISABLE, CLEAR_FAULT, a trajectory Action, or a
setpoint API.  It records the returned raw positions and communication failures
to establish a baseline before any right-arm calibration or motion work.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any


SERVICE = "/discover_right_arm_read_only"
SERVICE_TIMEOUT_S = 5.0


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--interval-s", type=float, default=0.05)
    parser.add_argument(
        "--expect-position-raw",
        type=int,
        help=(
            "require every read position to be this raw value; use 2048 to "
            "record the shared physical Q0 convention"
        ),
    )
    parser.add_argument(
        "--position-tolerance-raw",
        type=int,
        default=10,
        help="absolute raw tolerance used with --expect-position-raw (default: 10)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts" / "right_arm" / "read_only_soak.json",
    )
    return parser.parse_args()


def wait_future(node: Any, future: Any, timeout_s: float) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("right-arm discovery service timed out")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    if response is None:
        raise RuntimeError("right-arm discovery service returned no response")
    return response


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval_s < 0.0:
        raise SystemExit("--interval-s must be non-negative")
    if args.expect_position_raw is not None and not 0 <= args.expect_position_raw <= 4095:
        raise SystemExit("--expect-position-raw must be within 0..4095")
    if args.position_tolerance_raw < 0:
        raise SystemExit("--position-tolerance-raw must be non-negative")

    import rclpy
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("right_arm_read_only_soak")
    try:
        client = node.create_client(Trigger, SERVICE)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {SERVICE}")

        records: list[dict[str, Any]] = []
        for index in range(args.samples):
            started = time.monotonic()
            response = wait_future(
                node, client.call_async(Trigger.Request()), SERVICE_TIMEOUT_S
            )
            elapsed_ms = (time.monotonic() - started) * 1_000.0
            try:
                payload = json.loads(response.message)
            except json.JSONDecodeError as error:
                raise RuntimeError(
                    f"discovery response was not JSON: {response.message}"
                ) from error
            record = {
                "sample": index + 1,
                "service_success": bool(response.success),
                "elapsed_ms": round(elapsed_ms, 3),
                "present_mask": payload.get("present_mask"),
                "positions_raw": payload.get("positions_raw"),
                "read_statuses": payload.get("read_statuses"),
                "transaction_count": payload.get("transaction_count"),
                "failure_count": payload.get("failure_count"),
            }
            if args.expect_position_raw is not None:
                positions = record["positions_raw"]
                record["position_matches_expected"] = (
                    isinstance(positions, list)
                    and len(positions) == 6
                    and all(
                        abs(value - args.expect_position_raw)
                        <= args.position_tolerance_raw
                        for value in positions
                    )
                )
            records.append(record)
            print(
                "RIGHT_READ_ONLY_SAMPLE "
                f"index={record['sample']} success={record['service_success']} "
                f"mask={record['present_mask']} failures={record['failure_count']} "
                f"elapsed_ms={record['elapsed_ms']:.3f}"
            )
            if index + 1 < args.samples:
                time.sleep(args.interval_s)

        successful = [record for record in records if record["service_success"]]
        all_positions = [
            record["positions_raw"] for record in successful
            if isinstance(record["positions_raw"], list) and len(record["positions_raw"]) == 6
        ]
        ranges = (
            [max(values[index] for values in all_positions) - min(values[index] for values in all_positions)
             for index in range(6)]
            if all_positions else None
        )
        result = {
            "kind": "right_arm_read_only_discovery_soak",
            "commanded_operations": ["READ present_position"],
            "samples_requested": args.samples,
            "samples_successful": len(successful),
            "samples_failed": args.samples - len(successful),
            "all_present": all(
                record["service_success"]
                and record["present_mask"] == "0x3F"
                and record["read_statuses"] == [0, 0, 0, 0, 0, 0]
                and record["failure_count"] == 0
                for record in records
            ),
            "position_range_raw": ranges,
            "expected_position_raw": args.expect_position_raw,
            "position_tolerance_raw": (
                args.position_tolerance_raw
                if args.expect_position_raw is not None else None
            ),
            "all_expected_positions": (
                all(
                    record.get("position_matches_expected", False)
                    for record in records
                ) if args.expect_position_raw is not None else None
            ),
            "records": records,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            "RIGHT_ARM_READ_ONLY_SOAK "
            f"samples={args.samples} successful={len(successful)} "
            f"all_present={result['all_present']} "
            f"all_expected_positions={result['all_expected_positions']} "
            f"position_range_raw={ranges} output={args.output}"
        )
        q0_matches = result["all_expected_positions"]
        return 0 if result["all_present"] and q0_matches is not False else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
