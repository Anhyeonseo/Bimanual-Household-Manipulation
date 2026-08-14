#!/usr/bin/env python3
"""Pure, plan-only contract for grasping a fallen can and standing it upright.

This module deliberately has no ROS publisher, service client, serial transport, or
motion executor. It turns validated perception into a task-space contract that a
later MoveIt/resident adapter integration must satisfy. The separation matters:
the current five-axis MoveIt configuration uses position-only IK and therefore
cannot by itself prove that a can axis became vertical.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Literal


class LyingCanContractError(RuntimeError):
    """A perception or planning precondition is not safe enough to continue."""


def wrap_undirected_axis(angle_rad: float) -> float:
    """Normalize an undirected line angle to (-pi/2, pi/2]."""
    if not math.isfinite(angle_rad):
        raise LyingCanContractError("axis yaw must be finite")
    wrapped = (angle_rad + math.pi / 2.0) % math.pi - math.pi / 2.0
    return math.pi / 2.0 if wrapped <= -math.pi / 2.0 else wrapped


def undirected_axis_error(a_rad: float, b_rad: float) -> float:
    """Smallest absolute error between two undirected axes."""
    return abs(wrap_undirected_axis(a_rad - b_rad))


def nearest_gripper_crossing_yaw(
    can_axis_yaw_rad: float,
    current_finger_yaw_rad: float,
) -> dict[str, float]:
    """Return the nearest gripper closing-line yaw crossing the can at 90 deg.

    finger_yaw follows grasp_yaw_kinematics: it is the line along which the
    fingers open/close, not the tool approach direction. A parallel jaw grasp
    of a cylinder must put this line perpendicular to the cylinder long axis.
    Both lines are undirected, so the shortest equivalent branch is always
    within 90 degrees and never asks for a gratuitous full revolution.
    """
    can_axis = wrap_undirected_axis(can_axis_yaw_rad)
    current = wrap_undirected_axis(current_finger_yaw_rad)
    target = wrap_undirected_axis(can_axis + math.pi / 2.0)
    delta = wrap_undirected_axis(target - current)
    return {
        "can_axis_yaw_rad": can_axis,
        "current_finger_yaw_rad": current,
        "target_finger_yaw_rad": target,
        "required_delta_rad": delta,
    }


@dataclass(frozen=True)
class CanPerceptionPolicy:
    target_class: str
    maximum_frame_age_s: float
    minimum_confidence: float
    minimum_aspect_ratio: float
    expected_length_m: float
    expected_diameter_m: float
    length_tolerance_m: float
    diameter_tolerance_m: float
    routing_deadband_px: float
    maximum_top_wrist_center_disagreement_m: float
    maximum_top_wrist_yaw_disagreement_rad: float
    maximum_wrist_correction_m: float
    maximum_wrist_yaw_correction_rad: float

    def __post_init__(self) -> None:
        positive = (
            "maximum_frame_age_s",
            "minimum_aspect_ratio",
            "expected_length_m",
            "expected_diameter_m",
            "length_tolerance_m",
            "diameter_tolerance_m",
            "routing_deadband_px",
            "maximum_top_wrist_center_disagreement_m",
            "maximum_top_wrist_yaw_disagreement_rad",
            "maximum_wrist_correction_m",
            "maximum_wrist_yaw_correction_rad",
        )
        if not self.target_class:
            raise ValueError("target_class is required")
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be within (0, 1]")
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.expected_length_m <= self.expected_diameter_m:
            raise ValueError("a lying can must be longer than its diameter")
        if self.minimum_aspect_ratio <= 1.0:
            raise ValueError("minimum_aspect_ratio must distinguish a long axis")


@dataclass(frozen=True)
class CanObservation:
    source: Literal["top", "wrist"]
    class_name: str
    detection_count: int
    confidence: float
    frame_age_s: float
    center_x_m: float
    center_y_m: float
    long_axis_yaw_rad: float
    major_axis_m: float
    minor_axis_m: float
    center_x_px: float
    center_y_px: float
    image_width_px: int
    image_height_px: int
    footprint_inside: bool = True
    image_fully_visible: bool = True
    # Direction from can center toward its physical bottom along the undirected
    # long axis. A standard can must stand on its bottom, so this is mandatory
    # before upright rotation. It is expected to come from a wrist close-up
    # end classifier (pull-tab/top versus bottom), not from OBB geometry alone.
    bottom_end_sign: Literal[-1, 1] | None = None


def validate_observation(
    policy: CanPerceptionPolicy,
    observation: CanObservation,
) -> CanObservation:
    if observation.detection_count != 1:
        raise LyingCanContractError(
            f"{observation.source} detection_count={observation.detection_count}; "
            "exactly one can is required"
        )
    if observation.class_name != policy.target_class:
        raise LyingCanContractError(
            f"unexpected class {observation.class_name!r}; "
            f"expected {policy.target_class!r}"
        )
    numeric = (
        observation.confidence,
        observation.frame_age_s,
        observation.center_x_m,
        observation.center_y_m,
        observation.long_axis_yaw_rad,
        observation.major_axis_m,
        observation.minor_axis_m,
        observation.center_x_px,
        observation.center_y_px,
    )
    if not all(math.isfinite(value) for value in numeric):
        raise LyingCanContractError("observation contains a non-finite value")
    if observation.frame_age_s < 0.0:
        raise LyingCanContractError("frame age cannot be negative")
    if observation.frame_age_s > policy.maximum_frame_age_s:
        raise LyingCanContractError(
            f"{observation.source} frame is stale: "
            f"{observation.frame_age_s:.3f}s"
        )
    if observation.confidence < policy.minimum_confidence:
        raise LyingCanContractError(
            f"{observation.source} confidence {observation.confidence:.3f} is "
            f"below {policy.minimum_confidence:.3f}"
        )
    if observation.image_width_px <= 0 or observation.image_height_px <= 0:
        raise LyingCanContractError("image dimensions must be positive")
    if not 0.0 <= observation.center_x_px < observation.image_width_px:
        raise LyingCanContractError("center_x_px is outside the source image")
    if not 0.0 <= observation.center_y_px < observation.image_height_px:
        raise LyingCanContractError("center_y_px is outside the source image")
    if not observation.footprint_inside or not observation.image_fully_visible:
        raise LyingCanContractError(
            "the complete can footprint must be calibrated and visible"
        )
    if observation.minor_axis_m <= 0.0:
        raise LyingCanContractError("minor_axis_m must be positive")
    if observation.major_axis_m < observation.minor_axis_m:
        raise LyingCanContractError(
            "major_axis_m must not be shorter than minor_axis_m"
        )
    aspect_ratio = observation.major_axis_m / observation.minor_axis_m
    if aspect_ratio < policy.minimum_aspect_ratio:
        raise LyingCanContractError(
            f"can long axis is ambiguous: aspect_ratio={aspect_ratio:.3f}"
        )
    if (
        abs(observation.major_axis_m - policy.expected_length_m)
        > policy.length_tolerance_m
    ):
        raise LyingCanContractError(
            "observed can length is outside the commissioned model"
        )
    if (
        abs(observation.minor_axis_m - policy.expected_diameter_m)
        > policy.diameter_tolerance_m
    ):
        raise LyingCanContractError(
            "observed can diameter is outside the commissioned model"
        )
    wrap_undirected_axis(observation.long_axis_yaw_rad)
    return observation


def select_arm(observation: CanObservation, deadband_px: float) -> str:
    midpoint = observation.image_width_px / 2.0
    half = deadband_px / 2.0
    if observation.center_x_px < midpoint - half:
        return "left"
    if observation.center_x_px > midpoint + half:
        return "right"
    raise LyingCanContractError(
        "can center is inside the arm-routing deadband"
    )


def validate_wrist_refinement(
    policy: CanPerceptionPolicy,
    top: CanObservation,
    wrist: CanObservation,
) -> dict[str, float]:
    validate_observation(policy, top)
    validate_observation(policy, wrist)
    center_error = math.hypot(
        wrist.center_x_m - top.center_x_m,
        wrist.center_y_m - top.center_y_m,
    )
    yaw_error = undirected_axis_error(
        wrist.long_axis_yaw_rad,
        top.long_axis_yaw_rad,
    )
    if center_error > policy.maximum_top_wrist_center_disagreement_m:
        raise LyingCanContractError(
            f"top/wrist center disagreement {center_error:.4f}m exceeds limit"
        )
    if yaw_error > policy.maximum_top_wrist_yaw_disagreement_rad:
        raise LyingCanContractError(
            f"top/wrist yaw disagreement {yaw_error:.4f}rad exceeds limit"
        )
    if center_error > policy.maximum_wrist_correction_m:
        raise LyingCanContractError(
            "required wrist XY correction exceeds the bounded servo window"
        )
    if yaw_error > policy.maximum_wrist_yaw_correction_rad:
        raise LyingCanContractError(
            "required wrist yaw correction exceeds the bounded servo window"
        )
    if wrist.bottom_end_sign not in (-1, 1):
        raise LyingCanContractError(
            "wrist close-up did not identify the physical can bottom; "
            "upright rotation is ambiguous"
        )
    return {
        "correction_x_m": wrist.center_x_m - top.center_x_m,
        "correction_y_m": wrist.center_y_m - top.center_y_m,
        "correction_norm_m": center_error,
        "yaw_correction_rad": wrap_undirected_axis(
            wrist.long_axis_yaw_rad - top.long_axis_yaw_rad
        ),
        "yaw_error_rad": yaw_error,
    }


def build_task_space_plan(
    policy: CanPerceptionPolicy,
    top: CanObservation,
    wrist: CanObservation,
    current_finger_yaw_rad: float,
) -> dict[str, object]:
    """Create a non-executable task-space plan after both vision gates pass."""
    correction = validate_wrist_refinement(policy, top, wrist)
    arm = select_arm(top, policy.routing_deadband_px)
    yaw = nearest_gripper_crossing_yaw(
        wrist.long_axis_yaw_rad,
        current_finger_yaw_rad,
    )
    if (
        abs(yaw["required_delta_rad"])
        > policy.maximum_wrist_yaw_correction_rad
    ):
        raise LyingCanContractError(
            "nearest equivalent wrist rotation exceeds the commissioned bound"
        )
    image_axis = wrap_undirected_axis(top.long_axis_yaw_rad)
    image_orientation = (
        "horizontal"
        if abs(math.cos(image_axis)) >= abs(math.sin(image_axis))
        else "vertical"
    )
    stages = [
        {
            "name": "top_lock",
            "motion": False,
            "gate": "exactly_one_full_can_obb",
        },
        {
            "name": "high_pregrasp",
            "motion": True,
            "approach": "table_normal_from_above",
            "tool_yaw_rad": yaw["target_finger_yaw_rad"],
        },
        {
            "name": "wrist_lock_and_refine",
            "motion": True,
            "control_frame": "selected_wrist_camera_relative_xy_yaw",
            "z_correction": False,
        },
        {
            "name": "vertical_descend",
            "motion": True,
            "direction": "negative_table_normal",
            "floor_sweep": False,
        },
        {"name": "close_gripper", "motion": True},
        {
            "name": "lift_clear",
            "motion": True,
            "direction": "positive_table_normal",
        },
        {
            "name": "rotate_can_upright",
            "motion": True,
            "required_can_axis_pitch_change_rad": math.pi / 2.0,
            "bottom_end_sign": wrist.bottom_end_sign,
            "orientation_must_be_verified": True,
        },
        {
            "name": "lower_until_bottom_supported",
            "motion": True,
            "contact_model": "table_plane_plus_known_can_length",
        },
        {"name": "release", "motion": True},
        {"name": "vertical_retreat", "motion": True},
        {
            "name": "return_q0_hold",
            "motion": True,
            "torque_hold": True,
        },
    ]
    return {
        "schema_version": 1,
        "status": "TASK_SPACE_PLAN_ONLY_PASS",
        "motion_commands": 0,
        "selected_arm": arm,
        "image_orientation_diagnostic": image_orientation,
        "top_observation": asdict(top),
        "wrist_observation": asdict(wrist),
        "wrist_refinement": correction,
        "gripper_yaw": yaw,
        "upright_contract": {
            "lying_axis_is_horizontal_in_3d_for_every_image_yaw": True,
            "required_pitch_change_rad": math.pi / 2.0,
            "bottom_identification_required": True,
            "position_only_ik_is_sufficient": False,
        },
        "stages": stages,
        "execution_authorized": False,
    }
