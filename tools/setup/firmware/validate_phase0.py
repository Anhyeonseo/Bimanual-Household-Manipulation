#!/usr/bin/env python3
"""Validate measured SO-ARM101 Phase 0 hardware-baseline data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


EXPECTED_JOINTS = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
}
EXPECTED_IDS = set(range(1, 7))
FEEDBACK_FIELDS = ("position", "speed", "load", "voltage")


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _require_bool(errors: list[str], path: str, value: Any, expected: bool | None = None) -> None:
    if not isinstance(value, bool):
        errors.append(f"{path}: boolean measurement is required")
    elif expected is not None and value is not expected:
        errors.append(f"{path}: expected {expected}, got {value}")


def _validate_power(data: dict[str, Any], arm_name: str, errors: list[str]) -> None:
    power = data.get("power", {}).get(arm_name)
    if not isinstance(power, dict):
        errors.append(f"power.{arm_name}: object is required")
        return
    for field in ("rated_voltage_v", "rated_current_a", "no_load_v", "enabled_idle_v", "moving_min_v"):
        value = power.get(field)
        if not _is_number(value) or value <= 0:
            errors.append(f"power.{arm_name}.{field}: positive numeric value is required")


def _validate_arm(data: dict[str, Any], arm_name: str, errors: list[str]) -> None:
    arm = data.get("arms", {}).get(arm_name)
    if not isinstance(arm, dict):
        errors.append(f"arms.{arm_name}: object is required")
        return

    _require_bool(errors, f"arms.{arm_name}.unexpected_motion_on_powerup", arm.get("unexpected_motion_on_powerup"), expected=False)
    for field in ("adapter_reconnect_ok", "settings_persist_after_power_cycle", "wrong_id_silent"):
        _require_bool(errors, f"arms.{arm_name}.{field}", arm.get(field), expected=True)

    servos = arm.get("servos")
    if not isinstance(servos, list):
        errors.append(f"arms.{arm_name}.servos: list is required")
        return
    ids = {servo.get("id") for servo in servos if isinstance(servo, dict)}
    if len(servos) != 6 or ids != EXPECTED_IDS:
        errors.append(f"arms.{arm_name}.servos: exactly IDs 1..6 are required; got {sorted(ids, key=str)}")

    joint_names: list[str] = []
    for index, servo in enumerate(servos):
        path = f"arms.{arm_name}.servos[{index}]"
        if not isinstance(servo, dict):
            errors.append(f"{path}: object is required")
            continue

        joint_name = servo.get("joint_name")
        if joint_name not in EXPECTED_JOINTS:
            errors.append(f"{path}.joint_name: one of {sorted(EXPECTED_JOINTS)} is required")
        else:
            joint_names.append(joint_name)

        _require_bool(errors, f"{path}.ping_ok", servo.get("ping_ok"), expected=True)
        _require_bool(errors, f"{path}.raw_increases_with_positive_joint", servo.get("raw_increases_with_positive_joint"))
        _require_bool(errors, f"{path}.abnormal_heat_or_noise", servo.get("abnormal_heat_or_noise"), expected=False)
        _require_bool(errors, f"{path}.communication_errors", servo.get("communication_errors"), expected=False)

        raw_current = servo.get("raw_current")
        raw_min = servo.get("raw_safe_min")
        raw_max = servo.get("raw_safe_max")
        raw_values = (raw_current, raw_min, raw_max)
        if not all(isinstance(value, int) and not isinstance(value, bool) for value in raw_values):
            errors.append(f"{path}: raw_current/raw_safe_min/raw_safe_max integers are required")
        elif not raw_min < raw_max:
            errors.append(f"{path}: raw_safe_min must be smaller than raw_safe_max")
        elif not raw_min <= raw_current <= raw_max:
            errors.append(f"{path}.raw_current: value is outside the recorded safe range")

        feedback = servo.get("feedback")
        if not isinstance(feedback, dict):
            errors.append(f"{path}.feedback: object is required")
        else:
            for field in FEEDBACK_FIELDS:
                _require_bool(errors, f"{path}.feedback.{field}", feedback.get(field), expected=True)

    if set(joint_names) != EXPECTED_JOINTS or len(joint_names) != 6:
        errors.append(f"arms.{arm_name}.servos: each expected joint must appear exactly once; got {sorted(joint_names)}")


def validate_baseline(data: dict[str, Any], selected_arm: str = "all") -> list[str]:
    """Return errors; an empty list means the selected Phase 0 gate passes."""
    errors: list[str] = []
    if data.get("schema_version") != 1:
        errors.append("schema_version: expected 1")
    if not isinstance(data.get("measured_at"), str) or not data["measured_at"].strip():
        errors.append("measured_at: non-empty ISO-8601 string is required")
    if not isinstance(data.get("operator"), str) or not data["operator"].strip():
        errors.append("operator: non-empty string is required")

    arms = ("left", "right") if selected_arm == "all" else (selected_arm,)
    for arm_name in arms:
        _validate_power(data, arm_name, errors)
        _validate_arm(data, arm_name, errors)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=Path("hardware/phase0_baseline.json"))
    parser.add_argument("--arm", choices=("left", "right", "all"), default="all")
    args = parser.parse_args()
    try:
        data = json.loads(args.path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[ERROR] Could not read {args.path}: {exc}")
        return 2

    errors = validate_baseline(data, args.arm)
    if errors:
        print(f"[FAIL] Phase 0 {args.arm} gate has {len(errors)} incomplete or unsafe fields:")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(f"[PASS] Phase 0 {args.arm} hardware baseline is complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
