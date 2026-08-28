"""Canonical two-fold phase geometry: bimanual first, single-arm second.

The widest, most compliant 300 mm edge is constrained at two separated
points.  After that fold halves the unsupported edge length and creates a
two-layer rectangle, the second orthogonal fold uses one midpoint grasp while
the inactive arm remains at the independently validated clear pose.

This module only builds task poses.  Full-FK constraints, gripper collision
modes, joint limits, planning-scene collision checks, and execution lockout
are owned by the shared plan-only gate.
"""

from __future__ import annotations

import math
from typing import Sequence

from tools.lib.towel_task_pose_planning import (
    TowelPlanningError,
    CandidateSpec,
    FIRST_LAYER_TCP_Z_OFFSET_M,
    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
    MAXIMUM_APPROACH_TILT_RAD,
    NOMINAL_TOWEL_SIDE_M,
    OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD,
    OBSERVE_CLEAR_TCP_BY_ARM_M,
    PhaseSpec,
    PREGRASP_CLEARANCE_M,
    SECOND_LAYER_TCP_Z_OFFSET_M,
    finite_vector,
    task_pose,
)
from tools.lib.grasp_yaw_kinematics import wrap_half_turn


FIRST_EDGE_ENDPOINT_INSET_M = 0.015
FIRST_FOLD_NORMAL_INSET_M = 0.040
SECOND_FOLD_NORMAL_INSET_M = 0.030
SECOND_RELEASE_TCP_Z_OFFSET_M = 0.040
FIRST_ARC_SAMPLE_COUNT = 17
SECOND_ARC_SAMPLE_COUNT = 9
RETREAT_CLEARANCE_M = 0.050
DEPARTURE_FRACTIONS = tuple(index / 20.0 for index in range(1, 21))


def _blend_yaw(start: float, target: float, fraction: float) -> float:
    return wrap_half_turn(start + fraction * wrap_half_turn(target - start))


def _departure_phases(
    *,
    prefix: str,
    arm: str,
    target_xyz: tuple[float, float, float],
    target_yaw: float,
    layer: str,
) -> tuple[PhaseSpec, ...]:
    origin = OBSERVE_CLEAR_TCP_BY_ARM_M[arm]
    origin_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD[arm]
    phases = []
    for index, fraction in enumerate(DEPARTURE_FRACTIONS, start=1):
        xyz = tuple(
            start + fraction * (target - start)
            for start, target in zip(origin, target_xyz, strict=True)
        )
        pose = task_pose(
            f"{prefix}_{index:02d}_{arm}",
            arm,
            xyz,
            _blend_yaw(origin_yaw, target_yaw, fraction),
            "pregrasp_open",
            layer,
            (
                MAXIMUM_APPROACH_TILT_RAD
                if math.isclose(fraction, 1.0, abs_tol=1.0e-12)
                else MAXIMUM_ATTACHED_TRANSFER_TILT_RAD
            ),
        )
        phases.append(PhaseSpec(pose.name, (pose,)))
    return tuple(phases)


def _arc_coordinate(
    start: float,
    target: float,
    start_z: float,
    target_z: float,
    progress: float,
) -> tuple[float, float]:
    center = 0.5 * (start + target)
    radius = abs(start - center)
    sign = math.copysign(1.0, start - center)
    angle = math.pi * progress
    coordinate = center + sign * radius * math.cos(angle)
    base_z = start_z + progress * (target_z - start_z)
    return coordinate, base_z + radius * math.sin(angle)


def build_bimanual_first_fold(
    bounds: Sequence[float], table_z_m: float
) -> tuple[tuple[PhaseSpec, ...], tuple[float, float, float, float]]:
    """Fold the near/negative-X edge to positive X using two endpoints."""
    left, right, bottom, top = finite_vector(bounds, 4, "towel bounds")
    center_x = 0.5 * (left + right)
    start_x = left + FIRST_FOLD_NORMAL_INSET_M
    target_x = right - FIRST_FOLD_NORMAL_INSET_M
    contact_z = table_z_m + FIRST_LAYER_TCP_Z_OFFSET_M
    jaw_yaw = 0.0
    grasp_y_by_arm = {
        "left": top - FIRST_EDGE_ENDPOINT_INSET_M,
        "right": bottom + FIRST_EDGE_ENDPOINT_INSET_M,
    }
    pregrasp_by_arm = {
        arm: (start_x, grasp_y, contact_z + PREGRASP_CLEARANCE_M)
        for arm, grasp_y in grasp_y_by_arm.items()
    }
    phases: list[PhaseSpec] = []
    # Move the near-side/right arm first, then hold it at pregrasp while the
    # far-side/left arm departs.  This avoids one large simultaneous transit.
    for arm in ("right", "left"):
        phases.extend(
            _departure_phases(
                prefix="first_departure",
                arm=arm,
                target_xyz=pregrasp_by_arm[arm],
                target_yaw=jaw_yaw,
                layer="one_layer",
            )
        )
    phases.append(
        PhaseSpec(
            "first_contact",
            tuple(
                task_pose(
                    f"first_contact_{arm}",
                    arm,
                    (start_x, grasp_y_by_arm[arm], contact_z),
                    jaw_yaw,
                    "contact",
                    "one_layer",
                )
                for arm in ("left", "right")
            ),
            attachment_event="attach_two_single_layer_edge_patches_after_dual_contact_gate",
        )
    )
    for sample_index in range(1, FIRST_ARC_SAMPLE_COUNT):
        progress = sample_index / (FIRST_ARC_SAMPLE_COUNT - 1)
        x, z = _arc_coordinate(
            start_x, target_x, contact_z, contact_z, progress
        )
        semantic = (
            "attached_laydown"
            if sample_index == FIRST_ARC_SAMPLE_COUNT - 1
            else "attached_transfer"
        )
        phases.append(
            PhaseSpec(
                f"first_fold_{sample_index:02d}",
                tuple(
                    task_pose(
                        f"first_fold_{sample_index:02d}_{arm}",
                        arm,
                        (x, grasp_y_by_arm[arm], z),
                        jaw_yaw,
                        semantic,
                        "one_layer",
                    )
                    for arm in ("left", "right")
                ),
                attachment_event=(
                    "release_both_edge_patches_after_laydown_gate"
                    if sample_index == FIRST_ARC_SAMPLE_COUNT - 1
                    else None
                ),
            )
        )
    phases.append(
        PhaseSpec(
            "first_retreat",
            tuple(
                task_pose(
                    f"first_retreat_{arm}",
                    arm,
                    (target_x, grasp_y_by_arm[arm], contact_z + RETREAT_CLEARANCE_M),
                    jaw_yaw,
                    "released_retreat",
                    "one_layer",
                )
                for arm in ("left", "right")
            ),
        )
    )
    phases.append(PhaseSpec("first_reobserve_clear", (), clear_pose=True))
    return tuple(phases), (center_x, right, bottom, top)


def build_single_arm_second_fold(
    first_footprint: Sequence[float],
    table_z_m: float,
    *,
    active_arm: str,
    direction: str,
) -> tuple[tuple[PhaseSpec, ...], tuple[float, float, float, float]]:
    """Fold one short moving edge at its midpoint with the nearest arm."""
    if active_arm not in {"left", "right"}:
        raise TowelPlanningError(f"invalid second active arm: {active_arm}")
    left, right, bottom, top = finite_vector(
        first_footprint, 4, "first footprint"
    )
    center_y = 0.5 * (bottom + top)
    grasp_x = 0.5 * (left + right)
    if direction == "right_to_left":
        start_y = bottom + SECOND_FOLD_NORMAL_INSET_M
        target_y = top - SECOND_FOLD_NORMAL_INSET_M
        final = (left, right, center_y, top)
    elif direction == "left_to_right":
        start_y = top - SECOND_FOLD_NORMAL_INSET_M
        target_y = bottom + SECOND_FOLD_NORMAL_INSET_M
        final = (left, right, bottom, center_y)
    else:
        raise TowelPlanningError(f"invalid second direction: {direction}")
    contact_z = table_z_m + SECOND_LAYER_TCP_Z_OFFSET_M
    release_z = table_z_m + SECOND_RELEASE_TCP_Z_OFFSET_M
    jaw_yaw = math.pi / 2.0
    pregrasp = (grasp_x, start_y, contact_z + PREGRASP_CLEARANCE_M)
    phases: list[PhaseSpec] = list(
        _departure_phases(
            prefix="second_departure",
            arm=active_arm,
            target_xyz=pregrasp,
            target_yaw=jaw_yaw,
            layer="two_layer_bundle",
        )
    )
    phases.append(
        PhaseSpec(
            "second_contact",
            (
                task_pose(
                    f"second_contact_{active_arm}",
                    active_arm,
                    (grasp_x, start_y, contact_z),
                    jaw_yaw,
                    "contact",
                    "two_layer_bundle",
                ),
            ),
            attachment_event="attach_moving_edge_midpoint_bundle_after_contact_gate",
        )
    )
    for sample_index in range(1, SECOND_ARC_SAMPLE_COUNT):
        progress = sample_index / (SECOND_ARC_SAMPLE_COUNT - 1)
        y, z = _arc_coordinate(
            start_y, target_y, contact_z, release_z, progress
        )
        semantic = (
            "attached_laydown"
            if sample_index == SECOND_ARC_SAMPLE_COUNT - 1
            else "attached_transfer"
        )
        phases.append(
            PhaseSpec(
                f"second_fold_{sample_index:02d}",
                (
                    task_pose(
                        f"second_fold_{sample_index:02d}_{active_arm}",
                        active_arm,
                        (grasp_x, y, z),
                        jaw_yaw,
                        semantic,
                        "two_layer_bundle",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    ),
                ),
                attachment_event=(
                    "release_midpoint_bundle_after_laydown_gate"
                    if sample_index == SECOND_ARC_SAMPLE_COUNT - 1
                    else None
                ),
            )
        )
    phases.extend(
        (
            PhaseSpec(
                "second_retreat",
                (
                    task_pose(
                        f"second_retreat_{active_arm}",
                        active_arm,
                        (
                            grasp_x,
                            target_y,
                            release_z + RETREAT_CLEARANCE_M,
                        ),
                        jaw_yaw,
                        "released_retreat",
                        "two_layer_bundle",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    ),
                ),
            ),
            PhaseSpec(
                "second_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm=active_arm,
            ),
        )
    )
    return tuple(phases), final


def build_bimanual_then_single_candidates(
    towel_bounds: Sequence[float], table_z_m: float
) -> tuple[CandidateSpec, ...]:
    """Return the canonical topology and bounded second-fold alternatives."""
    bounds = finite_vector(towel_bounds, 4, "towel bounds")
    left, right, bottom, top = bounds
    if not left < right or not bottom < top:
        raise TowelPlanningError("towel bounds must have positive area")
    if not math.isclose(right - left, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise TowelPlanningError("towel x span must be nominal 300 mm")
    if not math.isclose(top - bottom, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise TowelPlanningError("towel y span must be nominal 300 mm")
    if not math.isfinite(table_z_m):
        raise TowelPlanningError("table z must be finite")
    if not 0.0 < MAXIMUM_APPROACH_TILT_RAD < math.pi:
        raise TowelPlanningError("shared approach cone is invalid")

    first_phases, first_footprint = build_bimanual_first_fold(bounds, table_z_m)
    candidates = []
    # Nearest-arm alternatives are ordered first; MoveIt still decides by
    # passing the complete pose, collision, and path gates.
    for direction, arms in (
        ("right_to_left", ("right", "left")),
        ("left_to_right", ("left", "right")),
    ):
        for active_arm in arms:
            second_phases, final = build_single_arm_second_fold(
                first_footprint,
                table_z_m,
                active_arm=active_arm,
                direction=direction,
            )
            candidates.append(
                CandidateSpec(
                    candidate_id=(
                        "first_bimanual_robot_near_to_far__second_"
                        f"{active_arm}_{direction}_edge_midpoint"
                    ),
                    first_arm_assignment="left_high_y_right_low_y",
                    first_axis="x",
                    first_direction="robot_near_to_far",
                    second_axis="y",
                    second_direction=direction,
                    second_active_arm=active_arm,
                    first_fold_phases=first_phases,
                    second_fold_phases=second_phases,
                    first_expected_footprint_xyxy_m=first_footprint,
                    final_expected_footprint_xyxy_m=final,
                )
            )
    return tuple(candidates)
