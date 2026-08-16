#!/usr/bin/env python3
"""Pure contracts for camera-routed single-arm resident Pick/Place.

The source-image x pixel selects exactly one arm.  Dynamic planning and
resident execution remain separate gates; a perception observation never
authorizes motion by itself.  Legacy pinned-route helpers remain here for the
previously verified left-arm evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
from statistics import median
from typing import Iterable, Sequence


LEFT_ARM_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
RIGHT_ARM_JOINTS = (
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
)
CANONICAL_JOINTS = (
    *LEFT_ARM_JOINTS,
    "left_gripper_joint",
    *RIGHT_ARM_JOINTS,
    "right_gripper_joint",
)
ARM_JOINTS_BY_SIDE = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}
BIMANUAL_ARM_INDICES = (0, 1, 2, 3, 4, 6, 7, 8, 9, 10)
DEFAULT_CAMERA_CENTER_DEADBAND_PX = 40.0
RIGHT_BASE_TRANSLATION_IN_WORKCELL_M = (0.0, -0.232064146, 0.0)
RIGHT_BASE_RPY_IN_WORKCELL_RAD = (0.0, 0.0, 0.0)

EXPECTED_MANIFEST_SHA256 = (
    "94f6eb82eab531276518ab2409b1fbf97e43b7e72b8768f0c3ddfafc7028a297"
)
EXPECTED_ROUTE_REPORT_SHA256 = (
    "175e1c15afba8dfedc0e32a25d0bad67f65629e7a7d688a47f2d281e6e1d0866"
)
EXPECTED_EXECUTION_EVIDENCE_SHA256 = (
    "2a52bb03bb8411601a00d6ea895173f6f0c7816a8eb07d3e750de810a6171ba5"
)
EXPECTED_PICK_PLAN_SHA256 = (
    "c932b38daa22bb7253295fc17d3fac3fe6919052f42ec94c84d08667d70a3dd6"
)
EXPECTED_LIFT_PLAN_SHA256 = (
    "9d282e533f862596c9ad9eb178d8c3ada8c71a27abd7252886bdb5d6c8203bbd"
)
EXPECTED_PLACE_PLAN_SHA256 = (
    "39eae1f89d2ec9b0944227ec86eef61603450e45d67c0716013ba7df0730f9f5"
)

PICK_GRASP_OFFSET_M = 0.011
TARGET_POSITION_TOLERANCE_M = 0.005
TARGET_YAW_TOLERANCE_RAD = math.radians(10.0)
TARGET_LOCK_SPREAD_M = 0.003
# Match the F8 firmware's measured arm terminal-settle contract exactly:
# 30 encoder raw * (2*pi/4096) ~= 0.046020 rad. The route-time tracking
# limit remains independently tighter than unsafe motion at 0.09 rad.
ARM_TERMINAL_TOLERANCE_RAD = 0.046020
LEFT_Q0_TOLERANCE_RAD = ARM_TERMINAL_TOLERANCE_RAD
CONTACT_THRESHOLD_RAW = 14
RELEASE_TOLERANCE_RAW = 30
RAW_STEP_RAD = 2.0 * math.pi / 4096.0
FINITE_POINT_OFFSETS_MS = (50, 100, 150, 200, 250, 300, 350, 400)
MAXIMUM_FINITE_DELTA_RAD = 0.12


class TopPickPlaceContractError(RuntimeError):
    """An input failed before any application motion is allowed."""


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


def select_arm_for_pixel(
    center_x_px: float,
    image_width_px: int,
    deadband_px: float = DEFAULT_CAMERA_CENTER_DEADBAND_PX,
) -> str:
    """Route one object by its source-image x coordinate.

    The center band is deliberately rejected.  Board coordinates are not used
    here because homography axis conventions must never silently swap the arms.
    """
    center_x_px = float(center_x_px)
    image_width_px = int(image_width_px)
    deadband_px = float(deadband_px)
    if image_width_px <= 0:
        raise TopPickPlaceContractError("image width must be positive")
    if (
        not math.isfinite(center_x_px)
        or center_x_px < 0.0
        or center_x_px >= image_width_px
    ):
        raise TopPickPlaceContractError("object center x is outside the image")
    if (
        not math.isfinite(deadband_px)
        or deadband_px < 0.0
        or deadband_px >= image_width_px
    ):
        raise TopPickPlaceContractError("camera center deadband is invalid")
    midpoint = image_width_px / 2.0
    half_deadband = deadband_px / 2.0
    if center_x_px < midpoint - half_deadband:
        return "left"
    if center_x_px > midpoint + half_deadband:
        return "right"
    raise TopPickPlaceContractError(
        "object center is inside the camera routing deadband: "
        f"x={center_x_px:.3f}px center={midpoint:.3f}px "
        f"deadband={deadband_px:.3f}px"
    )


def require_consistent_arm_selection(sides: Iterable[str]) -> str:
    values = tuple(sides)
    if not values or any(value not in ARM_JOINTS_BY_SIDE for value in values):
        raise TopPickPlaceContractError("arm selection samples are invalid")
    unique = set(values)
    if len(unique) != 1:
        raise TopPickPlaceContractError(
            f"camera routing changed during target lock: {sorted(unique)}"
        )
    return values[0]


def workspace_coordinates_for_arm(
    x_m: float, y_m: float, arm: str, z_m: float = 0.0063
) -> tuple[float, float]:
    """Express a workcell target in the selected arm's calibrated base frame."""
    if arm not in ARM_JOINTS_BY_SIDE:
        raise TopPickPlaceContractError(f"unsupported arm: {arm}")
    x_m = float(x_m)
    y_m = float(y_m)
    z_m = float(z_m)
    if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
        raise TopPickPlaceContractError("workspace coordinates must be finite")
    if arm == "left":
        return x_m, y_m

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
            (x_m, y_m, z_m),
            RIGHT_BASE_TRANSLATION_IN_WORKCELL_M,
            strict=True,
        )
    )
    base = tuple(
        sum(root_from_base[row][column] * delta[row] for row in range(3))
        for column in range(3)
    )
    return base[0], base[1]


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _load_pinned(path: Path, expected_sha256: str, label: str) -> dict:
    actual = sha256_file(path)
    if actual != expected_sha256:
        raise TopPickPlaceContractError(
            f"{label} sha256 mismatch: expected={expected_sha256} actual={actual}"
        )
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TopPickPlaceContractError(f"{label} must contain a JSON object")
    return value


def load_application_inputs(
    manifest_path: Path,
    report_path: Path,
    evidence_path: Path,
    pick_plan_path: Path,
) -> tuple[dict, tuple[float, float, float, float]]:
    """Pin the exact route and the successful physical execution evidence."""
    manifest = _load_pinned(
        manifest_path, EXPECTED_MANIFEST_SHA256, "route manifest"
    )
    report = _load_pinned(
        report_path, EXPECTED_ROUTE_REPORT_SHA256, "route report"
    )
    evidence = _load_pinned(
        evidence_path, EXPECTED_EXECUTION_EVIDENCE_SHA256, "execution evidence"
    )
    pick_plan = _load_pinned(
        pick_plan_path, EXPECTED_PICK_PLAN_SHA256, "pick plan"
    )
    if (
        manifest.get("status") != "FULL_PICK_PLACE_PLAN_ONLY_PASS"
        or manifest.get("execution_api_used") is not False
        or manifest.get("calibration_hash") != "0x2D90167E"
        or tuple(manifest.get("joint_names", ())) != LEFT_ARM_JOINTS
        or not isinstance(manifest.get("steps"), list)
        or not manifest["steps"]
    ):
        raise TopPickPlaceContractError("route manifest contract is invalid")
    if (
        report.get("nominal_moveit_collision_check_passed") is not True
        or report.get("tracking_envelope_route_matches_inputs") is not True
        or report.get("manifest", {}).get("sha256") != EXPECTED_MANIFEST_SHA256
    ):
        raise TopPickPlaceContractError("route report does not match the route")
    expected_plan_hashes = {
        "pick": EXPECTED_PICK_PLAN_SHA256,
        "pick_lift": EXPECTED_LIFT_PLAN_SHA256,
        "place": EXPECTED_PLACE_PLAN_SHA256,
    }
    if (
        evidence.get("status") != "PICK_PLACE_ONCE_RESIDENT_COMPLETE"
        or evidence.get("automatic_retry_count") != 0
        or evidence.get("plan_sha256") != expected_plan_hashes
    ):
        raise TopPickPlaceContractError(
            "physical execution evidence does not match the application route"
        )
    plans = pick_plan.get("plans")
    if not isinstance(plans, list):
        raise TopPickPlaceContractError("pick plan has no plan list")
    matches = [
        item
        for item in plans
        if isinstance(item, dict)
        and item.get("name") == "grasp"
        and item.get("success") is True
    ]
    if len(matches) != 1:
        raise TopPickPlaceContractError("pick plan has no unique grasp target")
    target = tuple(float(value) for value in matches[0].get("target_m", ()))
    if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise TopPickPlaceContractError("pick target is incomplete")
    expected_object_center = (
        target[0],
        target[1],
        target[2] - PICK_GRASP_OFFSET_M,
        float(matches[0]["yaw_rad"]),
    )
    return manifest, expected_object_center


def lock_target(samples: Iterable[BaseTargetSample]) -> LockedTarget:
    values = tuple(samples)
    if len(values) < 5:
        raise TopPickPlaceContractError("at least five valid target samples are required")
    for sample in values:
        if not all(
            math.isfinite(value)
            for value in (
                sample.x_m,
                sample.y_m,
                sample.z_m,
                sample.yaw_rad,
                sample.confidence,
            )
        ):
            raise TopPickPlaceContractError("target samples must be finite")
    center = tuple(
        median(getattr(sample, field) for sample in values)
        for field in ("x_m", "y_m", "z_m")
    )
    spread = max(
        math.dist((sample.x_m, sample.y_m, sample.z_m), center)
        for sample in values
    )
    if spread > TARGET_LOCK_SPREAD_M:
        raise TopPickPlaceContractError(
            f"target lock is unstable: spread={spread:.6f}m"
        )
    # The detector reports an undirected long axis.  Circular averaging on 2*yaw
    # keeps +pi/2 and -pi/2 adjacent instead of averaging them to zero.
    sin_sum = sum(math.sin(2.0 * sample.yaw_rad) for sample in values)
    cos_sum = sum(math.cos(2.0 * sample.yaw_rad) for sample in values)
    yaw = 0.5 * math.atan2(sin_sum, cos_sum)
    return LockedTarget(
        x_m=center[0],
        y_m=center[1],
        z_m=center[2],
        yaw_rad=yaw,
        minimum_confidence=min(sample.confidence for sample in values),
        maximum_position_spread_m=spread,
        sample_count=len(values),
    )


def validate_locked_target(
    locked: LockedTarget,
    expected_object_center_m: Sequence[float],
) -> tuple[float, float]:
    if len(expected_object_center_m) != 4:
        raise TopPickPlaceContractError("expected object center must be xyz+yaw")
    error = math.dist(
        (locked.x_m, locked.y_m, locked.z_m),
        tuple(float(value) for value in expected_object_center_m[:3]),
    )
    if error > TARGET_POSITION_TOLERANCE_M:
        raise TopPickPlaceContractError(
            "camera target does not match the proven route: "
            f"error={error:.6f}m limit={TARGET_POSITION_TOLERANCE_M:.6f}m"
        )
    expected_yaw = float(expected_object_center_m[3])
    yaw_error = abs(
        ((locked.yaw_rad - expected_yaw + math.pi / 2.0) % math.pi)
        - math.pi / 2.0
    )
    if yaw_error > TARGET_YAW_TOLERANCE_RAD:
        raise TopPickPlaceContractError(
            "camera target yaw does not match the proven route: "
            f"error={yaw_error:.6f}rad limit={TARGET_YAW_TOLERANCE_RAD:.6f}rad"
        )
    return error, yaw_error


def bimanual_q0_target(positions: Sequence[float]) -> tuple[float, ...]:
    """Set both five-axis arm chains to q0 while preserving both grippers."""
    if len(positions) != 12 or not all(math.isfinite(value) for value in positions):
        raise TopPickPlaceContractError("resident position must contain 12 finite joints")
    target = [float(value) for value in positions]
    for index in BIMANUAL_ARM_INDICES:
        target[index] = 0.0
    return tuple(target)


def validate_bimanual_q0(positions: Sequence[float]) -> float:
    """Require both five-axis arm chains at q0 and return maximum residual."""
    target = bimanual_q0_target(positions)
    residual = max(abs(float(positions[index])) for index in BIMANUAL_ARM_INDICES)
    if residual > LEFT_Q0_TOLERANCE_RAD:
        raise TopPickPlaceContractError(
            f"both arms are not at q0: maximum residual={residual:.6f}rad"
        )
    assert target[5] == float(positions[5]) and target[11] == float(positions[11])
    return residual


def validate_selected_q0_anchor(positions: Sequence[float], arm: str) -> None:
    if arm not in ARM_JOINTS_BY_SIDE:
        raise TopPickPlaceContractError(f"unsupported arm: {arm}")
    if len(positions) != 12 or not all(math.isfinite(value) for value in positions):
        raise TopPickPlaceContractError("resident anchor must contain 12 finite joints")
    arm_positions = positions[:5] if arm == "left" else positions[6:11]
    residual = max(abs(float(value)) for value in arm_positions)
    if residual > LEFT_Q0_TOLERANCE_RAD:
        raise TopPickPlaceContractError(
            f"{arm} arm is not at q0: maximum residual={residual:.6f}rad"
        )


def validate_q0_anchor(positions: Sequence[float]) -> None:
    """Backward-compatible left-arm q0 gate for the proven legacy route."""
    validate_selected_q0_anchor(positions, "left")


def step_target(
    current: Sequence[float],
    step: dict,
    opposite_hold: Sequence[float],
    arm: str = "left",
) -> tuple[float, ...]:
    if arm not in ARM_JOINTS_BY_SIDE:
        raise TopPickPlaceContractError(f"unsupported arm: {arm}")
    if len(current) != 12 or len(opposite_hold) != 6:
        raise TopPickPlaceContractError(
            "12-axis current and 6-axis opposite-arm hold required"
        )
    target = [float(value) for value in current]
    arm_slice = slice(0, 5) if arm == "left" else slice(6, 11)
    gripper_index = 5 if arm == "left" else 11
    hold_slice = slice(6, 12) if arm == "left" else slice(0, 6)
    if step.get("kind") == "arm":
        arm_positions = tuple(
            float(value) for value in step.get("target_positions_rad", ())
        )
        if len(arm_positions) != 5:
            raise TopPickPlaceContractError("arm step must contain five positions")
        target[arm_slice] = arm_positions
    elif step.get("kind") == "gripper":
        target[gripper_index] = float(step["target_position_rad"])
    else:
        raise TopPickPlaceContractError(f"unsupported manifest step: {step}")
    target[hold_slice] = [float(value) for value in opposite_hold]
    if not all(math.isfinite(value) for value in target):
        raise TopPickPlaceContractError("manifest produced a non-finite target")
    return tuple(target)

def split_finite_targets(
    start: Sequence[float], target: Sequence[float]
) -> tuple[tuple[float, ...], ...]:
    """Split one reviewed checkpoint without changing its straight joint path.

    0.12 rad over 400 ms is 0.30 rad/s (approximately 195.6 raw/s),
    below the 200 raw/s rate used by the physically successful route.
    """
    if len(start) != 12 or len(target) != 12:
        raise TopPickPlaceContractError("finite splitting requires 12 axes")
    largest = max(
        abs(float(b) - float(a)) for a, b in zip(start, target, strict=True)
    )
    count = max(1, math.ceil(largest / MAXIMUM_FINITE_DELTA_RAD))
    return tuple(
        tuple(float(value) for value in target)
        if index == count
        else tuple(
            float(a) + (float(b) - float(a)) * index / count
            for a, b in zip(start, target, strict=True)
        )
        for index in range(1, count + 1)
    )


def interpolate_finite_points(
    start: Sequence[float], target: Sequence[float]
) -> tuple[tuple[int, tuple[float, ...]], ...]:
    if len(start) != 12 or len(target) != 12:
        raise TopPickPlaceContractError("finite interpolation requires 12 axes")
    result = []
    final_offset = FINITE_POINT_OFFSETS_MS[-1]
    for offset in FINITE_POINT_OFFSETS_MS:
        fraction = offset / final_offset
        positions = tuple(
            float(a) + (float(b) - float(a)) * fraction
            for a, b in zip(start, target, strict=True)
        )
        result.append((offset, positions))
    return tuple(result)


def residual_raw(commanded_rad: float, measured_rad: float) -> int:
    return round(abs(float(commanded_rad) - float(measured_rad)) / RAW_STEP_RAD)
