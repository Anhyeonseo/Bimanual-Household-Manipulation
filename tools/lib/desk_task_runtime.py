"""Shared safety helpers for desk-organization manipulation tasks."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from statistics import median
from typing import Iterable, Sequence
import math

LEFT_ARM_JOINTS = (
    "left_base_joint", "left_shoulder_joint", "left_elbow_joint",
    "left_wrist_flex_joint", "left_wrist_roll_joint",
)
RIGHT_ARM_JOINTS = (
    "right_base_joint", "right_shoulder_joint", "right_elbow_joint",
    "right_wrist_flex_joint", "right_wrist_roll_joint",
)
CANONICAL_JOINTS = (
    *LEFT_ARM_JOINTS, "left_gripper_joint",
    *RIGHT_ARM_JOINTS, "right_gripper_joint",
)
ARM_JOINTS_BY_SIDE = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}
BIMANUAL_ARM_INDICES = (0, 1, 2, 3, 4, 6, 7, 8, 9, 10)
ARM_TERMINAL_TOLERANCE_RAD = 0.046020
RAW_STEP_RAD = 2.0 * math.pi / 4096.0
TARGET_LOCK_SPREAD_M = 0.003
RIGHT_BASE_TRANSLATION_IN_WORKCELL_M = (0.0, -0.232064146, 0.0)
RIGHT_BASE_RPY_IN_WORKCELL_RAD = (0.0, 0.0, 0.0)


class DeskTaskContractError(RuntimeError):
    """A desk-task input failed before motion was allowed."""


@dataclass(frozen=True, slots=True)
class BaseTargetSample:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    confidence: float


@dataclass(frozen=True, slots=True)
class LockedTarget:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    minimum_confidence: float
    maximum_position_spread_m: float
    sample_count: int


def workspace_coordinates_for_arm(
    x_m: float, y_m: float, arm: str, z_m: float = 0.0063
) -> tuple[float, float]:
    """Express a workcell target in the selected arm's base frame."""
    if arm not in ARM_JOINTS_BY_SIDE:
        raise DeskTaskContractError(f"unsupported arm: {arm}")
    values = (float(x_m), float(y_m), float(z_m))
    if not all(math.isfinite(value) for value in values):
        raise DeskTaskContractError("workspace coordinates must be finite")
    if arm == "left":
        return values[0], values[1]
    roll, pitch, yaw = RIGHT_BASE_RPY_IN_WORKCELL_RAD
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    root_from_base = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    delta = tuple(
        value - origin
        for value, origin in zip(
            values, RIGHT_BASE_TRANSLATION_IN_WORKCELL_M, strict=True
        )
    )
    base = tuple(
        sum(root_from_base[row][column] * delta[row] for row in range(3))
        for column in range(3)
    )
    return base[0], base[1]


def lock_target(samples: Iterable[BaseTargetSample]) -> LockedTarget:
    """Median-lock at least five finite observations of one object."""
    values = tuple(samples)
    if len(values) < 5:
        raise DeskTaskContractError("at least five valid target samples are required")
    for sample in values:
        if not all(
            math.isfinite(value)
            for value in (
                sample.x_m, sample.y_m, sample.z_m,
                sample.yaw_rad, sample.confidence,
            )
        ):
            raise DeskTaskContractError("target samples must be finite")
    center = tuple(
        median(getattr(sample, field) for sample in values)
        for field in ("x_m", "y_m", "z_m")
    )
    spread = max(
        math.dist((sample.x_m, sample.y_m, sample.z_m), center)
        for sample in values
    )
    if spread > TARGET_LOCK_SPREAD_M:
        raise DeskTaskContractError(
            f"target lock is unstable: spread={spread:.6f}m"
        )
    sin_sum = sum(math.sin(2.0 * sample.yaw_rad) for sample in values)
    cos_sum = sum(math.cos(2.0 * sample.yaw_rad) for sample in values)
    return LockedTarget(
        x_m=center[0], y_m=center[1], z_m=center[2],
        yaw_rad=0.5 * math.atan2(sin_sum, cos_sum),
        minimum_confidence=min(sample.confidence for sample in values),
        maximum_position_spread_m=spread,
        sample_count=len(values),
    )


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def bimanual_q0_target(positions: Sequence[float]) -> tuple[float, ...]:
    """Set both arm chains to q0 while preserving both grippers."""
    if len(positions) != 12 or not all(math.isfinite(value) for value in positions):
        raise DeskTaskContractError("resident position must contain 12 finite joints")
    target = [float(value) for value in positions]
    for index in BIMANUAL_ARM_INDICES:
        target[index] = 0.0
    return tuple(target)


def validate_bimanual_q0(positions: Sequence[float]) -> float:
    """Require both arm chains at q0 and return the maximum residual."""
    target = bimanual_q0_target(positions)
    residual = max(abs(float(positions[index])) for index in BIMANUAL_ARM_INDICES)
    if residual > ARM_TERMINAL_TOLERANCE_RAD:
        raise DeskTaskContractError(
            f"both arms are not at q0: maximum residual={residual:.6f}rad"
        )
    assert target[5] == float(positions[5]) and target[11] == float(positions[11])
    return residual


def step_target(
    current: Sequence[float], step: dict,
    opposite_hold: Sequence[float], arm: str = "left",
) -> tuple[float, ...]:
    """Apply one reviewed arm or gripper step to a 12-axis hold target."""
    if arm not in ARM_JOINTS_BY_SIDE:
        raise DeskTaskContractError(f"unsupported arm: {arm}")
    if len(current) != 12 or len(opposite_hold) != 6:
        raise DeskTaskContractError("12-axis current and 6-axis hold required")
    target = [float(value) for value in current]
    arm_slice = slice(0, 5) if arm == "left" else slice(6, 11)
    gripper_index = 5 if arm == "left" else 11
    hold_slice = slice(6, 12) if arm == "left" else slice(0, 6)
    if step.get("kind") == "arm":
        arm_positions = tuple(
            float(value) for value in step.get("target_positions_rad", ())
        )
        if len(arm_positions) != 5:
            raise DeskTaskContractError("arm step must contain five positions")
        target[arm_slice] = arm_positions
    elif step.get("kind") == "gripper":
        target[gripper_index] = float(step["target_position_rad"])
    else:
        raise DeskTaskContractError(f"unsupported manifest step: {step}")
    target[hold_slice] = [float(value) for value in opposite_hold]
    if not all(math.isfinite(value) for value in target):
        raise DeskTaskContractError("manifest produced a non-finite target")
    return tuple(target)


def residual_raw(commanded_rad: float, measured_rad: float) -> int:
    return round(abs(float(commanded_rad) - float(measured_rad)) / RAW_STEP_RAD)
