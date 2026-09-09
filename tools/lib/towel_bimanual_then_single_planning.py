"""Canonical towel-fold phase geometry.

The widest, most compliant 300 mm edge is constrained at two separated
points.  After that fold halves the unsupported edge length and creates a
two-layer rectangle, the preferred second orthogonal fold uses two separated
top-down U-pinches.  Each U-pinch doubles the local two-layer bundle to four
layers between the jaws.  A legacy single-midpoint builder remains for comparing the
superseded candidate and for reading old plan artifacts.

This module only builds task poses.  Full-FK constraints, gripper collision
modes, joint limits, planning-scene collision checks, and execution lockout
are owned by the shared plan-only gate.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Sequence

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

if TYPE_CHECKING:
    from tools.lib.towel_second_fold_correction import SecondFoldCorrectionPlan


FIRST_EDGE_ENDPOINT_INSET_M = 0.015
FIRST_FOLD_NORMAL_INSET_M = 0.015
# Keep the two-layer pinch close to the observed free edge.  A 30 mm inset
# made the grasp point itself become a built-in 30 mm fold error.  Fifteen
# millimetres matches the accepted first-fold edge contract while leaving
# enough cloth inside the jaw for the measured rubber-pad grasp.
SECOND_FOLD_NORMAL_INSET_M = 0.015
# The collision sweep over the accepted S1 footprint found a left-arm
# midpoint pinch that is valid with both the open jaw and the measured
# four-layer closed command.  Continuous Isaac contact showed that translating
# the terminal jaw 5 mm along its achieved moving-face inward direction puts
# both S1 layers on both jaw faces.  Apply that correction only during the
# final descent so the already-qualified 30-degree departure branch is kept.
SECOND_SINGLE_ARM_CONTACT_TCP_Z_OFFSET_M = 0.008
SECOND_SINGLE_ARM_PREGRASP_JAW_YAW_RAD = math.radians(30.0)
SECOND_SINGLE_ARM_JAW_YAW_RAD = math.radians(30.0)
SECOND_SINGLE_ARM_TRANSFER_JAW_YAW_RAD = math.radians(30.0)
SECOND_SINGLE_ARM_PRECONTACT_DESCENT_SAMPLE_COUNT = 10
SECOND_SINGLE_ARM_POST_PINCH_VERTICAL_LIFT_M = 0.010
SECOND_SINGLE_ARM_ACCEPTED_S1_CONTACT_CORRECTION_XYZ_M = (
    -0.002709,
    -0.002145,
    -0.003544,
)
SECOND_TRANSFER_TCP_Z_OFFSET_M = 0.040
# A second fold is not complete while the carried edge is still airborne.
# The exact contact height makes the moving jaw penetrate the validated table
# by 1.184 mm at the far pose.  Two millimetres of clearance preserves a
# supported laydown without the old 24 mm airborne release.
SECOND_RELEASE_TCP_Z_OFFSET_M = SECOND_LAYER_TCP_Z_OFFSET_M + 0.002
SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD = math.radians(30.0)
# The accepted S1 footprint cannot be carried through the complete S2 arc at
# both geometric endpoints.  These asymmetric insets keep 116 mm of the
# measured ~156 mm edge constrained while staying on the physical SO101 IK
# branches.  A physical dry-run identified the natural assignment: the left
# arm takes the robot-near/lower-X point with a vertical-in-top-view jaw, and
# the right arm takes the robot-far/higher-X point with a horizontal jaw.  The
# The actual-contact 63x63 S1 result settles 14--17 mm farther toward high X
# than the older coarse checkpoint.  The continuous-approach contact result
# then showed the far/right fixed pad 1.14 mm outside the two-layer closing
# corridor.  Moving the right TCP 5 mm toward low X and 5 mm down places both
# S1 topology halves inside both finite faces on that achieved cloth shape.
# A 45 mm right inset applies the X component while retaining at least 90 mm
# between the reviewed bimanual pinches.
# Collision approval is still owned by the MoveIt/FCL plan-only gate.
SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M = 0.015
SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M = 0.045
SECOND_BIMANUAL_MINIMUM_GRASP_SEPARATION_M = 0.090
# The accepted 63x63 actual-contact S1 checkpoint puts the two-layer boundary
# lower than the old coarse checkpoint at both asymmetric grasp sites.  The
# left midpoint remains valid.  For the right arm, the later continuous-
# approach surface (rather than the rejected teleported surface) requires the
# matching Z component of the finite-face corridor correction above.  Strict
# FCL found 0.054 mm of moving-jaw/table overlap at the raw -5 mm correction;
# the saved continuous-approach surface keeps all four contacts through a
# 0.50 mm rise (and loses them at 0.75 mm), so use that upper valid endpoint.
# The following synchronized vertical chords still own the approach path.
SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M = -0.01225
SECOND_BIMANUAL_RIGHT_CONTACT_HEIGHT_ADDITION_M = -0.01195
SECOND_BIMANUAL_PRECONTACT_DESCENT_SAMPLE_COUNT = 10
# The carried two-layer edge lands on the stationary two-layer half.  The old
# two-layer-plus-2 mm release height makes the left circular moving jaw enter
# the table by up to 2.01 mm in the bimanual terminal orientation.  Add 3 mm
# only at laydown; this represents the four-layer stack and leaves the XY fold
# target unchanged.
SECOND_BIMANUAL_RELEASE_HEIGHT_ADDITION_M = 0.003
# The old 30 degree constraint, rather than a physical joint limit, rejected
# the higher-X grasp late in the arc.  A diagonal pinch is physically valid
# for the real circular SO101 jaw; 50 degrees passes full FK without changing
# the reviewed joint envelope.
SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD = math.radians(50.0)
FIRST_ARC_SAMPLE_COUNT = 17
# Ten samples retain the previously collision-qualified 30-degree transfer
# branch; the yaw-0 transfer experiment is rejected because its IK disconnects
# one third of the way through the fold.
SECOND_ARC_SAMPLE_COUNT = 10
RETREAT_CLEARANCE_M = 0.050
# The off-midpoint S2 grasp crosses a narrow five-axis IK branch transition.
# Forty task-space waypoints keep each MoveIt joint chord inside the existing
# 4 mm TCP-path-deviation gate without weakening that gate.
DEPARTURE_FRACTIONS = tuple(index / 40.0 for index in range(1, 41))
SECOND_CORRECTION_LIFT_M = 0.008
SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD = math.radians(20.0)
SECOND_CORRECTION_PAD_ALIGNMENT_SAMPLES = 8
SECOND_CORRECTION_PAD_TILT_ALIGNMENT_LIMITS_RAD = tuple(
    math.radians(value) for value in (30.0, 28.0, 26.0, 24.0, 22.0, 20.0)
)
# A table-horizontal normal exactly perpendicular to the observed free edge is
# unreachable at the correction workspace point with the five-axis right arm.
# The nearest robust side-pinch branch is diagonal in the table plane: it keeps
# a 0.707 component across the edge while allowing the real fixed rubber face
# to stay within 20 degrees of horizontal.
# Keep the reachable, nearly table-horizontal +135-degree fixed-to-moving
# normal.  The overfold planner now places this grasp near the low-X end of
# the observed free edge, so the long curved moving-jaw body extends toward
# -X outside the towel instead of crossing the lower bundle from edge centre.
SECOND_CORRECTION_FIXED_PAD_NORMAL_YAW_RAD = 3.0 * math.pi / 4.0
SECOND_STABILIZER_TCP_Z_OFFSET_M = 0.025
SECOND_STABILIZER_FINAL_QUARTER_FRACTION = 0.55
SECOND_STABILIZER_ROUTE_CLEARANCE_M = 0.100
SECOND_STABILIZER_OUTSIDE_MARGIN_M = 0.035
SECOND_STABILIZER_LOW_ROUTE_Z_OFFSET_M = 0.040
SECOND_STABILIZER_ROUTE_SEGMENT_SAMPLES = (8, 8, 8, 12, 4)
# Keep the receiving right jaw far enough from the left jaw that already holds
# the laid-down free edge.  The accepted S2 left grasp is 20 mm left of the
# footprint centre; placing the receiver 40 mm right of centre gives 60 mm of
# TCP separation while remaining well inside the 156 mm folded edge.
SECOND_HANDOFF_ALONG_EDGE_OFFSET_M = 0.040


def _blend_yaw(start: float, target: float, fraction: float) -> float:
    return wrap_half_turn(start + fraction * wrap_half_turn(target - start))


def _blend_directed_yaw(start: float, target: float, fraction: float) -> float:
    delta = (target - start + math.pi) % (2.0 * math.pi) - math.pi
    return start + fraction * delta


def _departure_phases(
    *,
    prefix: str,
    arm: str,
    target_xyz: tuple[float, float, float],
    target_yaw: float,
    layer: str,
    maximum_finger_tilt_rad: float = math.pi / 2.0,
    enforce_finger_yaw: bool = True,
    fixed_pad_normal_yaw_rad: float | None = None,
    maximum_fixed_pad_normal_tilt_rad: float = math.pi / 2.0,
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
            maximum_finger_tilt_rad,
            enforce_finger_yaw=enforce_finger_yaw,
            fixed_pad_normal_yaw_rad=fixed_pad_normal_yaw_rad,
            maximum_fixed_pad_normal_tilt_rad=(
                maximum_fixed_pad_normal_tilt_rad
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
    along_edge_offset_m: float = 0.0,
    contact_correction_xyz_m: Sequence[float] = (0.0, 0.0, 0.0),
) -> tuple[tuple[PhaseSpec, ...], tuple[float, float, float, float]]:
    """Fold one short moving edge with the nearest arm."""
    if active_arm not in {"left", "right"}:
        raise TowelPlanningError(f"invalid second active arm: {active_arm}")
    left, right, bottom, top = finite_vector(
        first_footprint, 4, "first footprint"
    )
    center_y = 0.5 * (bottom + top)
    if not math.isfinite(along_edge_offset_m):
        raise TowelPlanningError("second-fold along-edge offset must be finite")
    correction_x, correction_y, correction_z = finite_vector(
        contact_correction_xyz_m, 3, "second-fold contact correction"
    )
    nominal_grasp_x = 0.5 * (left + right) + along_edge_offset_m
    grasp_x = nominal_grasp_x + correction_x
    if not left + SECOND_FOLD_NORMAL_INSET_M <= grasp_x <= right - SECOND_FOLD_NORMAL_INSET_M:
        raise TowelPlanningError(
            "second-fold along-edge grasp must remain inside the folded footprint"
        )
    if direction == "right_to_left":
        nominal_start_y = bottom + SECOND_FOLD_NORMAL_INSET_M
        start_y = nominal_start_y + correction_y
        target_y = top - SECOND_FOLD_NORMAL_INSET_M
        final = (left, right, center_y, top)
    elif direction == "left_to_right":
        nominal_start_y = top - SECOND_FOLD_NORMAL_INSET_M
        start_y = nominal_start_y + correction_y
        target_y = bottom + SECOND_FOLD_NORMAL_INSET_M
        final = (left, right, bottom, center_y)
    else:
        raise TowelPlanningError(f"invalid second direction: {direction}")
    nominal_contact_z = table_z_m + SECOND_SINGLE_ARM_CONTACT_TCP_Z_OFFSET_M
    contact_z = nominal_contact_z + correction_z
    transfer_end_z = table_z_m + SECOND_TRANSFER_TCP_Z_OFFSET_M
    release_z = table_z_m + SECOND_RELEASE_TCP_Z_OFFSET_M
    pregrasp_jaw_yaw = SECOND_SINGLE_ARM_PREGRASP_JAW_YAW_RAD
    jaw_yaw = SECOND_SINGLE_ARM_JAW_YAW_RAD
    pregrasp = (
        nominal_grasp_x,
        nominal_start_y,
        nominal_contact_z + PREGRASP_CLEARANCE_M,
    )
    phases: list[PhaseSpec] = list(
        _departure_phases(
            prefix="second_departure",
            arm=active_arm,
            target_xyz=pregrasp,
            target_yaw=pregrasp_jaw_yaw,
            layer="two_layer_bundle",
        )
    )
    for sample_index in range(
        1, SECOND_SINGLE_ARM_PRECONTACT_DESCENT_SAMPLE_COUNT
    ):
        progress = (
            sample_index / SECOND_SINGLE_ARM_PRECONTACT_DESCENT_SAMPLE_COUNT
        )
        phases.append(
            PhaseSpec(
                f"second_precontact_{sample_index:02d}",
                (
                    task_pose(
                        f"second_precontact_{sample_index:02d}_{active_arm}",
                        active_arm,
                        (
                            pregrasp[0]
                            + progress * (grasp_x - pregrasp[0]),
                            pregrasp[1]
                            + progress * (start_y - pregrasp[1]),
                            pregrasp[2]
                            + progress * (contact_z - pregrasp[2]),
                        ),
                        pregrasp_jaw_yaw
                        + progress * (jaw_yaw - pregrasp_jaw_yaw),
                        "pregrasp_open",
                        "two_layer_bundle",
                        maximum_finger_tilt_rad=(
                            SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD
                        ),
                    ),
                ),
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
                    maximum_finger_tilt_rad=(
                        SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD
                    ),
                ),
            ),
            attachment_event="attach_moving_edge_midpoint_bundle_after_contact_gate",
        )
    )
    phases.append(
        PhaseSpec(
            "second_fold_01",
            (
                task_pose(
                    f"second_fold_01_{active_arm}",
                    active_arm,
                    (
                        grasp_x,
                        start_y,
                        contact_z + SECOND_SINGLE_ARM_POST_PINCH_VERTICAL_LIFT_M,
                    ),
                    SECOND_SINGLE_ARM_TRANSFER_JAW_YAW_RAD,
                    "attached_lift",
                    "two_layer_bundle",
                    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD,
                ),
            ),
        )
    )
    for sample_index in range(1, SECOND_ARC_SAMPLE_COUNT):
        progress = sample_index / (SECOND_ARC_SAMPLE_COUNT - 1)
        y, z = _arc_coordinate(
            start_y, target_y, contact_z, transfer_end_z, progress
        )
        phases.append(
            PhaseSpec(
                f"second_fold_{sample_index + 1:02d}",
                (
                    task_pose(
                        f"second_fold_{sample_index + 1:02d}_{active_arm}",
                        active_arm,
                        (grasp_x, y, z),
                        SECOND_SINGLE_ARM_TRANSFER_JAW_YAW_RAD,
                        "attached_transfer",
                        "two_layer_bundle",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                        SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD,
                    ),
                ),
            )
        )
    phases.append(
        PhaseSpec(
            f"second_fold_{SECOND_ARC_SAMPLE_COUNT + 1:02d}",
            (
                task_pose(
                    f"second_fold_{SECOND_ARC_SAMPLE_COUNT + 1:02d}_{active_arm}",
                    active_arm,
                    (grasp_x, target_y, release_z),
                    SECOND_SINGLE_ARM_TRANSFER_JAW_YAW_RAD,
                    "attached_laydown",
                    "two_layer_bundle",
                    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD,
                ),
            ),
            attachment_event="release_midpoint_bundle_after_laydown_gate",
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
                        SECOND_SINGLE_ARM_TRANSFER_JAW_YAW_RAD,
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


def build_bimanual_second_fold(
    first_footprint: Sequence[float],
    table_z_m: float,
    *,
    direction: str,
) -> tuple[tuple[PhaseSpec, ...], tuple[float, float, float, float]]:
    """Fold the short edge using two four-layer top-down U-pinches.

    Both arms retain their own cloth patch through the same sampled arc and
    release only after both patches reach the supported laydown pose.  This
    removes the free rotation of the legacy midpoint grasp and avoids trying
    to isolate an upper two-layer bundle after all four layers overlap.
    """
    left, right, bottom, top = finite_vector(
        first_footprint, 4, "first footprint"
    )
    if not left < right or not bottom < top:
        raise TowelPlanningError("first footprint must have positive area")
    if not math.isfinite(table_z_m):
        raise TowelPlanningError("table z must be finite")

    center_y = 0.5 * (bottom + top)
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

    grasp_x_by_arm = {
        "left": left + SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M,
        "right": right - SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M,
    }
    jaw_yaw_by_arm = {
        "left": math.pi / 2.0,
        "right": 0.0,
    }
    separation = grasp_x_by_arm["right"] - grasp_x_by_arm["left"]
    if separation < SECOND_BIMANUAL_MINIMUM_GRASP_SEPARATION_M:
        raise TowelPlanningError(
            "folded edge is too short for the reviewed bimanual S2 grasp"
        )

    contact_z = table_z_m + SECOND_LAYER_TCP_Z_OFFSET_M
    contact_z_by_arm = {
        "left": contact_z + SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M,
        "right": contact_z + SECOND_BIMANUAL_RIGHT_CONTACT_HEIGHT_ADDITION_M,
    }
    transfer_end_z = table_z_m + SECOND_TRANSFER_TCP_Z_OFFSET_M
    release_z = (
        table_z_m
        + SECOND_RELEASE_TCP_Z_OFFSET_M
        + SECOND_BIMANUAL_RELEASE_HEIGHT_ADDITION_M
    )
    pregrasp_by_arm = {
        arm: (grasp_x, start_y, contact_z + PREGRASP_CLEARANCE_M)
        for arm, grasp_x in grasp_x_by_arm.items()
    }
    phases: list[PhaseSpec] = []
    # Stage the robot-far/right arm first, then bring the left arm into the
    # near point.  This is the non-crossing physical assignment; unlike the
    # rejected same-yaw candidate, each wrist stays on its natural side.
    for arm in ("right", "left"):
        phases.extend(
            _departure_phases(
                prefix="second_bimanual_departure",
                arm=arm,
                target_xyz=pregrasp_by_arm[arm],
                target_yaw=jaw_yaw_by_arm[arm],
                layer="two_layer_bundle",
                maximum_finger_tilt_rad=(
                    SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD
                ),
            )
        )
    # A direct 50 mm pregrasp-to-contact transition has valid endpoints, but
    # its five-axis joint interpolation bows a moving jaw through the table.
    # Short synchronized vertical chords preserve the selected IK branches
    # and keep the TCP paths on the intended descent line.
    for sample_index in range(
        1, SECOND_BIMANUAL_PRECONTACT_DESCENT_SAMPLE_COUNT
    ):
        progress = (
            sample_index / SECOND_BIMANUAL_PRECONTACT_DESCENT_SAMPLE_COUNT
        )
        phases.append(
            PhaseSpec(
                f"second_bimanual_precontact_{sample_index:02d}",
                tuple(
                    task_pose(
                        f"second_bimanual_precontact_{sample_index:02d}_{arm}",
                        arm,
                        (
                            grasp_x_by_arm[arm],
                            start_y,
                            pregrasp_by_arm[arm][2]
                            + progress
                            * (
                                contact_z_by_arm[arm]
                                - pregrasp_by_arm[arm][2]
                            ),
                        ),
                        jaw_yaw_by_arm[arm],
                        "pregrasp_open",
                        "two_layer_bundle",
                        MAXIMUM_APPROACH_TILT_RAD,
                        SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
                    )
                    for arm in ("left", "right")
                ),
            )
        )
    phases.append(
        PhaseSpec(
            "second_bimanual_contact",
            tuple(
                task_pose(
                    f"second_bimanual_contact_{arm}",
                    arm,
                    (grasp_x_by_arm[arm], start_y, contact_z_by_arm[arm]),
                    jaw_yaw_by_arm[arm],
                    "contact",
                    "two_layer_bundle",
                    maximum_finger_tilt_rad=(
                        SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD
                    ),
                )
                for arm in ("left", "right")
            ),
            attachment_event=(
                "attach_two_four_layer_u_pinches_after_dual_contact_gate"
            ),
        )
    )
    for sample_index in range(1, SECOND_ARC_SAMPLE_COUNT):
        progress = sample_index / (SECOND_ARC_SAMPLE_COUNT - 1)
        y, z = _arc_coordinate(
            start_y, target_y, contact_z, transfer_end_z, progress
        )
        phases.append(
            PhaseSpec(
                f"second_bimanual_fold_{sample_index:02d}",
                tuple(
                    task_pose(
                        f"second_bimanual_fold_{sample_index:02d}_{arm}",
                        arm,
                        (grasp_x_by_arm[arm], y, z),
                        jaw_yaw_by_arm[arm],
                        "attached_transfer",
                        "two_layer_bundle",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                        SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
                    )
                    for arm in ("left", "right")
                ),
            )
        )
    phases.append(
        PhaseSpec(
            f"second_bimanual_fold_{SECOND_ARC_SAMPLE_COUNT:02d}",
            tuple(
                task_pose(
                    f"second_bimanual_fold_{SECOND_ARC_SAMPLE_COUNT:02d}_{arm}",
                    arm,
                    (grasp_x_by_arm[arm], target_y, release_z),
                    jaw_yaw_by_arm[arm],
                    "attached_laydown",
                    "two_layer_bundle",
                    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
                )
                for arm in ("left", "right")
            ),
            attachment_event=(
                "release_two_four_layer_u_pinches_after_dual_laydown_gate"
            ),
        )
    )
    phases.append(
        PhaseSpec(
            "second_bimanual_retreat",
            tuple(
                task_pose(
                    f"second_bimanual_retreat_{arm}",
                    arm,
                    (
                        grasp_x_by_arm[arm],
                        target_y,
                        release_z + RETREAT_CLEARANCE_M,
                    ),
                    jaw_yaw_by_arm[arm],
                    "released_retreat",
                    "two_layer_bundle",
                    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                    SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
                )
                for arm in ("left", "right")
            ),
        )
    )
    phases.append(PhaseSpec("second_bimanual_reobserve_clear", (), clear_pose=True))
    return tuple(phases), final


def build_right_arm_second_fold_correction(
    plan: "SecondFoldCorrectionPlan", table_z_m: float
) -> tuple[PhaseSpec, ...]:
    """Build one bounded right-arm correction followed by clear reobservation.

    The edge is lifted only 8 mm before translating.  This separates the
    grasped upper edge from table friction without recreating a large arch.
    Every correction step returns both arms to the clear observation state;
    a caller must observe again before requesting another step.
    """
    if not plan.required or plan.grasp_xy_m is None or plan.target_xy_m is None:
        raise TowelPlanningError("a required metric correction plan is needed")
    if plan.arm != "right" or not plan.reobserve_after_step:
        raise TowelPlanningError("second-fold correction must use right-arm reobservation")
    grasp_x, grasp_y = finite_vector(plan.grasp_xy_m, 2, "correction grasp")
    target_x, target_y = finite_vector(plan.target_xy_m, 2, "correction target")
    edge_push = (
        plan.contact_mode == "exposed_overhang_open_jaw_fixed_pad_edge_push"
    )
    if not edge_push and plan.contact_mode != "top_bundle_edge_side_pinch":
        raise TowelPlanningError("unsupported second-fold correction contact mode")
    contact_z = float(table_z_m) + SECOND_LAYER_TCP_Z_OFFSET_M
    lifted_z = contact_z + SECOND_CORRECTION_LIFT_M
    approach_yaw = math.pi / 2.0
    pad_normal_yaw = SECOND_CORRECTION_FIXED_PAD_NORMAL_YAW_RAD
    pregrasp = (grasp_x, grasp_y, contact_z + PREGRASP_CLEARANCE_M)
    # A direct low diagonal to this low-X patch selects a low-shoulder IK branch
    # that intersects the overhead camera mount. Reuse the first 20 samples
    # of the previously strict-validated clear departure to reach its safe
    # high-shoulder gateway, then rise, translate above the towel, move over
    # the target, and descend. The 20+6+6+4+4 samples preserve the stable
    # 40-phase replay contract and keep the long jaw away from cloth.
    clear_xyz = OBSERVE_CLEAR_TCP_BY_ARM_M["right"]
    clear_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD["right"]
    camera_clear_gateway = (
        0.2078333543553543,
        -0.2748407988031619,
        0.02524647084744109,
    )
    high_route_z = float(table_z_m) + 0.105
    route_segments = (
        (camera_clear_gateway, 20),
        ((0.220, camera_clear_gateway[1], float(table_z_m) + 0.085), 6),
        ((grasp_x, clear_xyz[1], high_route_z), 6),
        ((grasp_x, grasp_y, high_route_z), 4),
        (pregrasp, 4),
    )
    phases: list[PhaseSpec] = []
    segment_start_xyz = clear_xyz
    departure_index = 0
    for segment_target_xyz, samples in route_segments:
        for segment_index in range(1, samples + 1):
            fraction = segment_index / samples
            departure_index += 1
            xyz = tuple(
                start + fraction * (target - start)
                for start, target in zip(
                    segment_start_xyz, segment_target_xyz, strict=True
                )
            )
            yaw = _blend_yaw(
                clear_yaw, approach_yaw, departure_index / 40.0
            )
            phase_name = (
                f"second_correction_departure_{departure_index:02d}_right"
            )
            pose = task_pose(
                phase_name,
                "right",
                xyz,
                yaw,
                "pregrasp_open",
                "upper_bundle_edge",
                (
                    MAXIMUM_APPROACH_TILT_RAD
                    if departure_index == 40
                    else MAXIMUM_ATTACHED_TRANSFER_TILT_RAD
                ),
            )
            phases.append(PhaseSpec(phase_name, (pose,)))
        segment_start_xyz = segment_target_xyz
    # The position-only departure ends with its fixed-pad normal near +90
    # degrees.  Jumping directly to the diagonal -45-degree pinch branch at
    # the same TCP point makes a large joint-space chord whose interpolated
    # TCP bows just beyond the strict 4 mm path limit.  First rotate the pad
    # normal in bounded increments with a loose tilt cone.  Then tighten the
    # pad tilt from 30 to 20 degrees in separate steps; coupling the last yaw
    # increment to the final 20-degree cone causes another discontinuous IK
    # branch jump.  The final phase name stays stable for replay contracts.
    for index in range(1, SECOND_CORRECTION_PAD_ALIGNMENT_SAMPLES + 1):
        fraction = index / SECOND_CORRECTION_PAD_ALIGNMENT_SAMPLES
        alignment_yaw = _blend_directed_yaw(
            approach_yaw, pad_normal_yaw, fraction
        )
        phase_name = f"second_correction_pad_yaw_align_{index:02d}"
        pose = task_pose(
            f"{phase_name}_right",
            "right",
            pregrasp,
            alignment_yaw,
            "pregrasp_pad_align",
            "upper_bundle_edge",
            MAXIMUM_APPROACH_TILT_RAD,
            enforce_finger_yaw=False,
            fixed_pad_normal_yaw_rad=alignment_yaw,
            maximum_fixed_pad_normal_tilt_rad=math.pi / 2.0,
        )
        phases.append(PhaseSpec(phase_name, (pose,)))
    for index, tilt_limit in enumerate(
        SECOND_CORRECTION_PAD_TILT_ALIGNMENT_LIMITS_RAD, start=1
    ):
        final_alignment = (
            index == len(SECOND_CORRECTION_PAD_TILT_ALIGNMENT_LIMITS_RAD)
        )
        phase_name = (
            "second_correction_pad_align"
            if final_alignment
            else f"second_correction_pad_tilt_align_{index:02d}"
        )
        pose = task_pose(
            f"{phase_name}_right",
            "right",
            pregrasp,
            pad_normal_yaw,
            "pregrasp_pad_align",
            "upper_bundle_edge",
            MAXIMUM_APPROACH_TILT_RAD,
            enforce_finger_yaw=False,
            fixed_pad_normal_yaw_rad=pad_normal_yaw,
            maximum_fixed_pad_normal_tilt_rad=tilt_limit,
        )
        phases.append(PhaseSpec(phase_name, (pose,)))
    if edge_push:
        # Keep the historical six-phase tail names so old replay readers can
        # validate a fixed-length plan.  In this contact mode the former
        # lift/laydown phases are an open-jaw preload and hold; there is no
        # attachment event, jaw closure, or kinematic cloth retention.
        phases.extend(
            (
                PhaseSpec(
                    "second_correction_contact",
                    (
                        task_pose(
                            "second_correction_contact_right",
                            "right",
                            (grasp_x, grasp_y, contact_z),
                            pad_normal_yaw,
                            "pregrasp_open",
                            "exposed_upper_edge",
                            enforce_finger_yaw=False,
                            fixed_pad_normal_yaw_rad=pad_normal_yaw,
                            maximum_fixed_pad_normal_tilt_rad=(
                                SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                            ),
                        ),
                    ),
                ),
                PhaseSpec(
                    "second_correction_lift",
                    (
                        task_pose(
                            "second_correction_lift_right",
                            "right",
                            (target_x, target_y, contact_z),
                            pad_normal_yaw,
                            "pregrasp_open",
                            "exposed_upper_edge",
                            enforce_finger_yaw=False,
                            fixed_pad_normal_yaw_rad=pad_normal_yaw,
                            maximum_fixed_pad_normal_tilt_rad=(
                                SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                            ),
                        ),
                    ),
                ),
                PhaseSpec(
                    "second_correction_translate",
                    (
                        task_pose(
                            "second_correction_translate_right",
                            "right",
                            (target_x, target_y, contact_z),
                            pad_normal_yaw,
                            "pregrasp_open",
                            "exposed_upper_edge",
                            enforce_finger_yaw=False,
                            fixed_pad_normal_yaw_rad=pad_normal_yaw,
                            maximum_fixed_pad_normal_tilt_rad=(
                                SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                            ),
                        ),
                    ),
                ),
                PhaseSpec(
                    "second_correction_laydown",
                    (
                        task_pose(
                            "second_correction_laydown_right",
                            "right",
                            (target_x, target_y, contact_z),
                            pad_normal_yaw,
                            "pregrasp_open",
                            "exposed_upper_edge",
                            enforce_finger_yaw=False,
                            fixed_pad_normal_yaw_rad=pad_normal_yaw,
                            maximum_fixed_pad_normal_tilt_rad=(
                                SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                            ),
                        ),
                    ),
                ),
                PhaseSpec(
                    "second_correction_retreat",
                    (
                        task_pose(
                            "second_correction_retreat_right",
                            "right",
                            (target_x, target_y, contact_z + RETREAT_CLEARANCE_M),
                            approach_yaw,
                            "released_retreat",
                            "upper_bundle_edge",
                        ),
                    ),
                ),
                PhaseSpec(
                    "second_correction_reobserve_clear",
                    (),
                    clear_pose=True,
                    clear_arm="right",
                ),
            )
        )
    else:
        phases.extend(
            (
            PhaseSpec(
                "second_correction_contact",
                (
                    task_pose(
                        "second_correction_contact_right",
                        "right",
                        (grasp_x, grasp_y, contact_z),
                        pad_normal_yaw,
                        "contact",
                        "upper_bundle_edge",
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
                attachment_event="attach_observed_upper_edge_after_contact_gate",
            ),
            PhaseSpec(
                "second_correction_lift",
                (
                    task_pose(
                        "second_correction_lift_right",
                        "right",
                        (grasp_x, grasp_y, lifted_z),
                        pad_normal_yaw,
                        "attached_lift",
                        "upper_bundle_edge",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
            ),
            PhaseSpec(
                "second_correction_translate",
                (
                    task_pose(
                        "second_correction_translate_right",
                        "right",
                        (target_x, target_y, lifted_z),
                        pad_normal_yaw,
                        "attached_correction",
                        "upper_bundle_edge",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
            ),
            PhaseSpec(
                "second_correction_laydown",
                (
                    task_pose(
                        "second_correction_laydown_right",
                        "right",
                        (target_x, target_y, contact_z),
                        pad_normal_yaw,
                        "attached_laydown",
                        "upper_bundle_edge",
                        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
                attachment_event="release_upper_edge_after_laydown_gate",
            ),
            PhaseSpec(
                "second_correction_retreat",
                (
                    task_pose(
                        "second_correction_retreat_right",
                        "right",
                        (target_x, target_y, contact_z + RETREAT_CLEARANCE_M),
                        approach_yaw,
                        "released_retreat",
                        "upper_bundle_edge",
                    ),
                ),
            ),
            PhaseSpec(
                "second_correction_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm="right",
            ),
            )
        )
    return tuple(phases)


def build_right_arm_second_fold_stabilizer(
    first_footprint: Sequence[float], table_z_m: float
) -> tuple[PhaseSpec, ...]:
    """Pin the laid-down S2 bundle while the left gripper releases.

    The right arm contacts the interior of the expected four-layer footprint,
    away from both the fold seam and the left-arm laydown point.  Runtime code
    must prove bilateral jaw contact before retaining a single contacted cloth
    particle; these poses alone never claim a grasp.
    """
    left, right, bottom, top = finite_vector(
        first_footprint, 4, "first footprint"
    )
    center_x = 0.5 * (left + right)
    fold_line_y = 0.5 * (bottom + top)
    contact_y = bottom + SECOND_STABILIZER_FINAL_QUARTER_FRACTION * (
        fold_line_y - bottom
    )
    contact_z = float(table_z_m) + SECOND_STABILIZER_TCP_Z_OFFSET_M
    jaw_yaw = math.pi / 2.0
    clear_xyz = OBSERVE_CLEAR_TCP_BY_ARM_M["right"]
    clear_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD["right"]
    route_z = float(table_z_m) + SECOND_STABILIZER_ROUTE_CLEARANCE_M
    pregrasp = (center_x, contact_y, route_z)
    low_route_z = float(table_z_m) + SECOND_STABILIZER_LOW_ROUTE_Z_OFFSET_M
    outside_y = bottom - SECOND_STABILIZER_OUTSIDE_MARGIN_M
    outside_entry_x = clear_xyz[0] + 0.050
    outside_transfer_x = left - 0.054
    route_segments = (
        ((outside_entry_x, outside_y, low_route_z), clear_yaw),
        ((outside_transfer_x, outside_y, low_route_z), clear_yaw),
        ((outside_transfer_x, outside_y, route_z), clear_yaw),
        ((center_x, contact_y, route_z), jaw_yaw),
        (pregrasp, jaw_yaw),
    )
    phases: list[PhaseSpec] = []
    segment_start_xyz = clear_xyz
    segment_start_yaw = clear_yaw
    departure_index = 0
    for (segment_target_xyz, segment_target_yaw), samples in zip(
        route_segments, SECOND_STABILIZER_ROUTE_SEGMENT_SAMPLES, strict=True
    ):
        for segment_index in range(1, samples + 1):
            fraction = segment_index / samples
            departure_index += 1
            xyz = tuple(
                start + fraction * (target - start)
                for start, target in zip(
                    segment_start_xyz, segment_target_xyz, strict=True
                )
            )
            pose = task_pose(
                f"second_stabilizer_departure_{departure_index:02d}_right",
                "right",
                xyz,
                _blend_yaw(segment_start_yaw, segment_target_yaw, fraction),
                "pregrasp_open",
                "four_layer_bundle",
                (
                    MAXIMUM_APPROACH_TILT_RAD
                    if departure_index == sum(
                        SECOND_STABILIZER_ROUTE_SEGMENT_SAMPLES
                    )
                    else MAXIMUM_ATTACHED_TRANSFER_TILT_RAD
                ),
            )
            phases.append(PhaseSpec(pose.name, (pose,)))
        segment_start_xyz = segment_target_xyz
        segment_start_yaw = segment_target_yaw
    phases.extend(
        (
            PhaseSpec(
                "second_stabilizer_contact",
                (
                    task_pose(
                        "second_stabilizer_contact_right",
                        "right",
                        (center_x, contact_y, contact_z),
                        jaw_yaw,
                        "contact",
                        "four_layer_bundle",
                        maximum_finger_tilt_rad=(
                            SECOND_FOLD_MAXIMUM_FINGER_TILT_RAD
                        ),
                    ),
                ),
                attachment_event=(
                    "attach_right_stabilizer_after_actual_contact_gate"
                ),
            ),
            PhaseSpec(
                "second_stabilizer_retreat",
                (
                    task_pose(
                        "second_stabilizer_retreat_right",
                        "right",
                        (
                            center_x,
                            contact_y,
                            contact_z + RETREAT_CLEARANCE_M,
                        ),
                        jaw_yaw,
                        "released_retreat",
                        "four_layer_bundle",
                    ),
                ),
                attachment_event=(
                    "release_right_stabilizer_after_left_clear_gate"
                ),
            ),
            PhaseSpec(
                "second_stabilizer_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm="right",
            ),
        )
    )
    return tuple(phases)


def build_right_arm_second_fold_edge_handoff(
    first_footprint: Sequence[float], table_z_m: float
) -> tuple[PhaseSpec, ...]:
    """Receive the laid-down two-layer free edge before the left jaw opens.

    This is an edge pinch, not an interior press.  Runtime execution must
    prove opposing-jaw contact across the two S1 topology halves and reject
    any contact with the stationary lower S2 bundle.  The right arm then
    holds this edge while the left arm releases and clears, avoiding reliance
    on persistent cloth static friction during the handoff.
    """
    left, right, bottom, top = finite_vector(
        first_footprint, 4, "first footprint"
    )
    center_x = 0.5 * (left + right)
    contact_x = center_x + SECOND_HANDOFF_ALONG_EDGE_OFFSET_M
    if not left + SECOND_FOLD_NORMAL_INSET_M <= contact_x <= right - SECOND_FOLD_NORMAL_INSET_M:
        raise TowelPlanningError("second-fold handoff grasp leaves the folded edge")
    contact_y = bottom + SECOND_FOLD_NORMAL_INSET_M
    contact_z = float(table_z_m) + SECOND_LAYER_TCP_Z_OFFSET_M
    approach_yaw = math.pi / 2.0
    pad_normal_yaw = SECOND_CORRECTION_FIXED_PAD_NORMAL_YAW_RAD
    clear_xyz = OBSERVE_CLEAR_TCP_BY_ARM_M["right"]
    clear_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD["right"]
    route_z = float(table_z_m) + SECOND_STABILIZER_ROUTE_CLEARANCE_M
    low_route_z = float(table_z_m) + SECOND_STABILIZER_LOW_ROUTE_Z_OFFSET_M
    outside_y = bottom - SECOND_STABILIZER_OUTSIDE_MARGIN_M
    outside_entry_x = clear_xyz[0] + 0.050
    outside_transfer_x = left - 0.054
    pregrasp = (contact_x, contact_y, route_z)
    route_segments = (
        ((outside_entry_x, outside_y, low_route_z), clear_yaw),
        ((outside_transfer_x, outside_y, low_route_z), clear_yaw),
        ((outside_transfer_x, outside_y, route_z), clear_yaw),
        ((contact_x, contact_y, route_z), approach_yaw),
        (pregrasp, approach_yaw),
    )
    phases: list[PhaseSpec] = []
    segment_start_xyz = clear_xyz
    segment_start_yaw = clear_yaw
    departure_index = 0
    for (segment_target_xyz, segment_target_yaw), samples in zip(
        route_segments, SECOND_STABILIZER_ROUTE_SEGMENT_SAMPLES, strict=True
    ):
        for segment_index in range(1, samples + 1):
            fraction = segment_index / samples
            departure_index += 1
            xyz = tuple(
                start + fraction * (target - start)
                for start, target in zip(
                    segment_start_xyz, segment_target_xyz, strict=True
                )
            )
            pose = task_pose(
                f"second_handoff_departure_{departure_index:02d}_right",
                "right",
                xyz,
                _blend_yaw(segment_start_yaw, segment_target_yaw, fraction),
                "pregrasp_open",
                "upper_bundle_edge",
                (
                    MAXIMUM_APPROACH_TILT_RAD
                    if departure_index == sum(SECOND_STABILIZER_ROUTE_SEGMENT_SAMPLES)
                    else MAXIMUM_ATTACHED_TRANSFER_TILT_RAD
                ),
                enforce_finger_yaw=False,
                fixed_pad_normal_yaw_rad=pad_normal_yaw,
                maximum_fixed_pad_normal_tilt_rad=(
                    SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                ),
            )
            phases.append(PhaseSpec(pose.name, (pose,)))
        segment_start_xyz = segment_target_xyz
        segment_start_yaw = segment_target_yaw
    phases.extend(
        (
            PhaseSpec(
                "second_handoff_contact",
                (
                    task_pose(
                        "second_handoff_contact_right",
                        "right",
                        (contact_x, contact_y, contact_z),
                        approach_yaw,
                        "contact",
                        "upper_bundle_edge",
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
                attachment_event=(
                    "attach_right_upper_edge_after_opposing_layer_contact_gate"
                ),
            ),
            PhaseSpec(
                "second_handoff_hold",
                (
                    task_pose(
                        "second_handoff_hold_right",
                        "right",
                        (contact_x, contact_y, contact_z),
                        approach_yaw,
                        "attached_hold",
                        "upper_bundle_edge",
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
            ),
            PhaseSpec(
                "second_handoff_release",
                (
                    task_pose(
                        "second_handoff_release_right",
                        "right",
                        (contact_x, contact_y, contact_z),
                        approach_yaw,
                        "released_in_place",
                        "upper_bundle_edge",
                        enforce_finger_yaw=False,
                        fixed_pad_normal_yaw_rad=pad_normal_yaw,
                        maximum_fixed_pad_normal_tilt_rad=(
                            SECOND_CORRECTION_MAXIMUM_FIXED_PAD_NORMAL_TILT_RAD
                        ),
                    ),
                ),
                attachment_event="release_right_upper_edge_after_left_clear_gate",
            ),
            PhaseSpec(
                "second_handoff_retreat",
                (
                    task_pose(
                        "second_handoff_retreat_right",
                        "right",
                        (contact_x, contact_y, contact_z + RETREAT_CLEARANCE_M),
                        approach_yaw,
                        "released_retreat",
                        "upper_bundle_edge",
                    ),
                ),
            ),
            PhaseSpec(
                "second_handoff_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm="right",
            ),
        )
    )
    return tuple(phases)


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
