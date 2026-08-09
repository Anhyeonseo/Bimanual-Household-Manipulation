"""Single-arm joint-unit conversion shared by host tests and tooling."""

from __future__ import annotations

import json
from pathlib import Path
import struct
from typing import Any

from tools.actuator_protocol import crc32c


DEFAULT_CONFIG = Path("config/single_arm_calibration.json")


class CalibrationError(ValueError):
    """Raised when calibration data or a converted target is invalid."""


def load_calibration(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    calibration = json.loads(path.read_text(encoding="utf-8"))
    validate_calibration(calibration)
    return calibration


def validate_calibration(calibration: dict[str, Any]) -> None:
    if calibration.get("schema_version") != 1:
        raise CalibrationError("unsupported calibration schema")
    if calibration.get("raw_units_per_turn") != 4096:
        raise CalibrationError("STS3215 raw_units_per_turn must be 4096")
    if calibration.get("turn_urad") != 6_283_185:
        raise CalibrationError("turn_urad must be 6283185")

    joints = calibration.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise CalibrationError("exactly six joints are required")

    for expected_id, joint in enumerate(joints, start=1):
        if joint.get("id") != expected_id:
            raise CalibrationError("joint IDs must be ordered 1 through 6")
        if joint.get("positive_raw_direction") not in (-1, 1):
            raise CalibrationError("positive_raw_direction must be -1 or 1")
        minimum = joint.get("minimum_raw")
        zero = joint.get("zero_raw")
        maximum = joint.get("maximum_raw")
        if not all(isinstance(value, int) for value in (minimum, zero, maximum)):
            raise CalibrationError("raw calibration values must be integers")
        if not 0 <= minimum <= zero <= maximum <= 4095:
            raise CalibrationError("raw range must contain zero_raw within 0..4095")


def calibration_hash(calibration: dict[str, Any]) -> int:
    """Match the firmware's explicitly serialized calibration CRC-32C."""

    validate_calibration(calibration)
    serialized = bytearray()
    for joint in calibration["joints"]:
        serialized.extend(
            struct.pack(
                "<BHHHbBB",
                joint["id"],
                joint["zero_raw"],
                joint["minimum_raw"],
                joint["maximum_raw"],
                joint["positive_raw_direction"],
                joint["p_gain"],
                joint["d_gain"],
            )
        )
    return crc32c(bytes(serialized))


def _round_divide(numerator: int, denominator: int) -> int:
    if denominator <= 0:
        raise ValueError("denominator must be positive")
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def urad_to_raw(
    calibration: dict[str, Any], joint_index: int, position_urad: int
) -> int:
    validate_calibration(calibration)
    if not 0 <= joint_index < 6:
        raise CalibrationError("joint_index must be in 0..5")
    if not isinstance(position_urad, int):
        raise CalibrationError("position_urad must be an integer")

    joint = calibration["joints"][joint_index]
    raw_delta = _round_divide(
        position_urad * calibration["raw_units_per_turn"],
        calibration["turn_urad"],
    )
    raw = joint["zero_raw"] + joint["positive_raw_direction"] * raw_delta
    if not joint["minimum_raw"] <= raw <= joint["maximum_raw"]:
        raise CalibrationError(
            f"joint {joint['id']} target {position_urad} urad converts to "
            f"raw {raw}, outside {joint['minimum_raw']}..{joint['maximum_raw']}"
        )
    return raw


def raw_to_urad(
    calibration: dict[str, Any], joint_index: int, raw_position: int
) -> int:
    validate_calibration(calibration)
    if not 0 <= joint_index < 6:
        raise CalibrationError("joint_index must be in 0..5")

    joint = calibration["joints"][joint_index]
    if not joint["minimum_raw"] <= raw_position <= joint["maximum_raw"]:
        raise CalibrationError("raw position is outside the configured safe range")
    positive_raw_delta = (
        raw_position - joint["zero_raw"]
    ) * joint["positive_raw_direction"]
    return _round_divide(
        positive_raw_delta * calibration["turn_urad"],
        calibration["raw_units_per_turn"],
    )

