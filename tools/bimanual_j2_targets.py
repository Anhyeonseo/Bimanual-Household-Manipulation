"""Pure J2 target and bounded-step contract shared by plan and execution tools."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


STATUS = "J2_ARM_AXIS_TARGETS_PLAN_ONLY_PASS"
ARM_JOINTS = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
DIRECTIONS = (1, 1, -1, -1, 1)
FRACTIONS = {25: (1, 4), 50: (1, 2), 75: (3, 4)}
ZERO_RAW = 2048
RAW_UNITS_PER_TURN = 4096
TURN_URAD = 6_283_185
MINIMUM_STEP_RAW = 8
MAXIMUM_STEP_RAW = 20
SETTLE_TOLERANCE_RAW = 10


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bound_json(path: Path, expected_sha256: str, label: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise ValueError(
            f"{label} SHA mismatch expected={expected_sha256.lower()} actual={actual}"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def round_divide(numerator: int, denominator: int) -> int:
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def raw_to_urad(raw: int, direction: int) -> int:
    return round_divide(
        (raw - ZERO_RAW) * direction * TURN_URAD,
        RAW_UNITS_PER_TURN,
    )


def fractional_target_raw(endpoint_raw: int, numerator: int, denominator: int) -> int:
    return ZERO_RAW + round_divide(
        (endpoint_raw - ZERO_RAW) * numerator,
        denominator,
    )


def bounded_step_raw(
    position_raw: int,
    target_raw: int,
    tolerance_raw: int = SETTLE_TOLERANCE_RAW,
) -> int:
    """Return one legal R2.1 jog that approaches the target without a short tail."""

    remaining = target_raw - position_raw
    magnitude = abs(remaining)
    if magnitude <= tolerance_raw:
        return 0
    if magnitude <= MAXIMUM_STEP_RAW:
        step = magnitude
    else:
        step = MAXIMUM_STEP_RAW
        tail = magnitude - step
        if 0 < tail < MINIMUM_STEP_RAW:
            step = magnitude - MINIMUM_STEP_RAW
    if not MINIMUM_STEP_RAW <= step <= MAXIMUM_STEP_RAW:
        raise ValueError(f"cannot form legal bounded step for remaining={remaining}")
    return step if remaining > 0 else -step


def derive_targets(approved: dict[str, Any]) -> dict[str, Any]:
    if approved.get("status") != "J1_L_ARM_5_APPROVED_FOR_PARITY_ONLY":
        raise ValueError("approved J1-L status changed")
    if approved.get("operator_approved") is not True:
        raise ValueError("J1-L limits are not operator-approved")
    if approved.get("motion_authorized") is not False:
        raise ValueError("J1-L approval unexpectedly authorized motion")
    if approved.get("runtime_change_authorized") is not False:
        raise ValueError("J1-L approval unexpectedly authorized runtime changes")

    arms: dict[str, Any] = {}
    for arm in ("left", "right"):
        joints: dict[str, Any] = {}
        for index, joint_name in enumerate(ARM_JOINTS):
            limit = approved["arms"][arm][joint_name]
            minimum = int(limit["minimum_unwrapped_raw"])
            maximum = int(limit["maximum_unwrapped_raw"])
            if not minimum < ZERO_RAW < maximum:
                raise ValueError(f"{arm}_{joint_name}: q0 is not inside J1-L")
            directions: dict[str, Any] = {}
            for side, endpoint in (("lower", minimum), ("upper", maximum)):
                targets: dict[str, Any] = {}
                previous_distance = 0
                for percent, (numerator, denominator) in FRACTIONS.items():
                    raw = fractional_target_raw(endpoint, numerator, denominator)
                    distance = abs(raw - ZERO_RAW)
                    if not minimum < raw < maximum or distance <= previous_distance:
                        raise ValueError(
                            f"{arm}_{joint_name}_{side}_{percent}: invalid target"
                        )
                    targets[str(percent)] = {
                        "target_unwrapped_raw": raw,
                        "target_urad": raw_to_urad(raw, DIRECTIONS[index]),
                        "distance_from_q0_raw": distance,
                    }
                    previous_distance = distance
                directions[side] = targets
            joints[joint_name] = {
                "servo_id": index + 1,
                "coordinate": limit["coordinate"],
                "q0_unwrapped_raw": ZERO_RAW,
                "approved_minimum_unwrapped_raw": minimum,
                "approved_maximum_unwrapped_raw": maximum,
                "directions": directions,
            }
        joints["gripper"] = {
            "servo_id": 6,
            "status": "BLOCKED_SEMANTIC_GRIPPER_MAPPING_REQUIRED",
        }
        arms[arm] = {"joints": joints}

    return {
        "schema_version": 1,
        "record_kind": "bimanual_j2_axis_targets_plan_only",
        "status": STATUS,
        "motion_authorized": False,
        "runtime_change_authorized": False,
        "execution_api_used": False,
        "q0_unwrapped_raw": ZERO_RAW,
        "fractions_percent": list(FRACTIONS),
        "endpoint_commands_forbidden": True,
        "multi_joint_commands_forbidden": True,
        "maximum_jog_step_raw": MAXIMUM_STEP_RAW,
        "settle_tolerance_raw": SETTLE_TOLERANCE_RAW,
        "arms": arms,
    }


def select_target(
    document: dict[str, Any],
    arm: str,
    joint: str,
    direction: str,
    fraction_percent: int,
) -> dict[str, Any]:
    if document.get("status") != STATUS:
        raise ValueError("J2 target artifact did not pass")
    if document.get("motion_authorized") is not False:
        raise ValueError("J2 target artifact unexpectedly authorized motion")
    if arm not in ("left", "right"):
        raise ValueError("arm must be left or right")
    if joint not in ARM_JOINTS:
        raise ValueError("joint must be one of the five arm joints")
    if direction not in ("lower", "upper"):
        raise ValueError("direction must be lower or upper")
    if fraction_percent not in FRACTIONS:
        raise ValueError("fraction must be 25, 50, or 75")
    joint_record = document["arms"][arm]["joints"][joint]
    target = joint_record["directions"][direction][str(fraction_percent)]
    return {
        "arm": arm,
        "joint": joint,
        "servo_id": int(joint_record["servo_id"]),
        "direction": direction,
        "fraction_percent": fraction_percent,
        "q0_unwrapped_raw": int(joint_record["q0_unwrapped_raw"]),
        "approved_minimum_unwrapped_raw": int(
            joint_record["approved_minimum_unwrapped_raw"]
        ),
        "approved_maximum_unwrapped_raw": int(
            joint_record["approved_maximum_unwrapped_raw"]
        ),
        "target_unwrapped_raw": int(target["target_unwrapped_raw"]),
        "target_urad": int(target["target_urad"]),
        "distance_from_q0_raw": int(target["distance_from_q0_raw"]),
    }
