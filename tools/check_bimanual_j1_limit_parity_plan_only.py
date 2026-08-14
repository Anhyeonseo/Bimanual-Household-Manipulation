#!/usr/bin/env python3
"""Verify approved J1-L arm limits in host math and no-output firmware."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any


ARM_JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
ARM_SLOTS = ("left", "right")
DIRECTIONS = (1, 1, -1, -1, 1)
ZERO_RAW = 2048
RAW_UNITS_PER_TURN = 4096
TURN_URAD = 6283185
GRIPPER_SENTINEL = (-6400000, 6400000)
STATUS = "J1_L_FIRMWARE_HOST_PARITY_PLAN_ONLY_PASS"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bound_json(path: Path, expected_sha256: str, label: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise ValueError(
            f"{label} SHA mismatch expected={expected_sha256.lower()} actual={actual}"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def resolve_source(root: Path, source: str) -> Path:
    path = Path(source)
    return path if path.is_absolute() else root / path


def round_divide(numerator: int, denominator: int) -> int:
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def raw_to_urad(raw: int, direction: int) -> int:
    return round_divide(
        (raw - ZERO_RAW) * direction * TURN_URAD, RAW_UNITS_PER_TURN
    )


def parse_firmware_limits(source: str) -> tuple[tuple[int, int], ...]:
    try:
        body = source.split("j1l_shadow_limits[", 1)[1].split("};", 1)[0]
    except IndexError as error:
        raise ValueError("firmware J1-L limit table is missing") from error
    limits = tuple(
        (int(lower), int(upper))
        for lower, upper in re.findall(r"\{\s*(-?\d+),\s*(-?\d+)\s*\}", body)
    )
    if len(limits) != 12:
        raise ValueError(f"firmware J1-L table has {len(limits)} entries")
    return limits


def check_parity(root: Path, approved: dict[str, Any]) -> dict[str, Any]:
    if approved.get("status") != "J1_L_ARM_5_APPROVED_FOR_PARITY_ONLY":
        raise ValueError("J1-L approval status changed")
    if approved.get("operator_approved") is not True:
        raise ValueError("J1-L candidate is not operator-approved")
    if approved.get("motion_authorized") is not False:
        raise ValueError("J1-L approval must keep motion_authorized=false")
    if approved.get("runtime_change_authorized") is not False:
        raise ValueError("J1-L approval must keep runtime changes disabled")

    inputs: dict[str, dict[str, str]] = approved["inputs"]
    bound_inputs: dict[str, Any] = {}
    for label, binding in inputs.items():
        path = resolve_source(root, binding["path"])
        bound_inputs[label] = load_bound_json(
            path, binding["sha256"], label
        )
    derived = bound_inputs["derived_candidate"]
    coverage = bound_inputs["left_nominal_route_coverage"]
    if coverage.get("status") != "J1_LEFT_NOMINAL_ROUTE_COVERAGE_PASS":
        raise ValueError("left nominal route coverage did not pass")

    expected_firmware: list[tuple[int, int]] = []
    projections: dict[str, Any] = {}
    for arm in ARM_SLOTS:
        projections[arm] = {}
        for index, joint_name in enumerate(ARM_JOINTS):
            joint = approved["arms"][arm][joint_name]
            candidate_joint = derived["arms"][arm]["joints"][joint_name]
            minimum_raw = int(joint["minimum_unwrapped_raw"])
            maximum_raw = int(joint["maximum_unwrapped_raw"])
            if (minimum_raw, maximum_raw) != (
                int(candidate_joint["candidate_minimum_unwrapped_raw"]),
                int(candidate_joint["candidate_maximum_unwrapped_raw"]),
            ):
                raise ValueError(f"{arm} {joint_name}: approved raw limits drifted")
            endpoints = (
                raw_to_urad(minimum_raw, DIRECTIONS[index]),
                raw_to_urad(maximum_raw, DIRECTIONS[index]),
            )
            expected = (min(endpoints), max(endpoints))
            stored = (int(joint["minimum_urad"]), int(joint["maximum_urad"]))
            if stored != expected:
                raise ValueError(f"{arm} {joint_name}: approved urad drifted")
            expected_firmware.append(expected)
            projections[arm][joint_name] = {
                "minimum_unwrapped_raw": minimum_raw,
                "maximum_unwrapped_raw": maximum_raw,
                "minimum_urad": expected[0],
                "maximum_urad": expected[1],
                "minimum_rad": expected[0] / 1_000_000.0,
                "maximum_rad": expected[1] / 1_000_000.0,
            }
        expected_firmware.append(GRIPPER_SENTINEL)

    firmware_path = root / (
        "firmware/stm32_g474_single_arm/Core/Src/"
        "bimanual_operational_limits.c"
    )
    firmware_limits = parse_firmware_limits(
        firmware_path.read_text(encoding="utf-8")
    )
    if firmware_limits != tuple(expected_firmware):
        raise ValueError("firmware J1-L limits do not match approved candidate")

    return {
        "status": STATUS,
        "motion_authorized": False,
        "runtime_change_authorized": False,
        "execution_api_used": False,
        "approval_scope": approved["approval_scope"],
        "arm_joint_count": 10,
        "gripper_limit_status": "VALIDATION_SENTINELS_NOT_APPROVED_LIMITS",
        "left_nominal_route_coverage": True,
        "projections": projections,
        "parity": {
            "firmware_unwrapped_urad": True,
            "host_unwrapped_rad": True,
            "urdf": False,
            "moveit": False,
            "isaac": False,
        },
        "remaining_blockers": [
            "gripper semantic mapping",
            "URDF/MoveIt/Isaac candidate projection",
            "physical q0 and inter-base model alignment",
            "right-arm representative route coverage",
            "J2 bounded active validation",
        ],
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--approved",
        type=Path,
        default=root / "config/bimanual_j1_operational_limits.approved.json",
    )
    parser.add_argument("--approved-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.set_defaults(root=root)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")
    approved = load_bound_json(
        args.approved, args.approved_sha256, "approved J1-L manifest"
    )
    report = check_parity(args.root, approved)
    report["inputs"] = {
        "approved": {
            "path": str(args.approved),
            "sha256": args.approved_sha256.lower(),
        }
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    digest = file_sha256(output)
    print(
        f"STATUS={STATUS} arm_joints=10 grippers=blocked "
        f"motion_authorized=false output={output} sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
