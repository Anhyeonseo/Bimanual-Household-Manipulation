"""Task-pose geometry and IK for the asymmetric two-fold R0 gate.

The SO-101 arm has five controlled arm joints, so it cannot satisfy an
arbitrary six-degree-of-freedom pose.  This module instead solves the task
constraints that matter for a towel grasp: TCP XYZ, the jaw-opening line in
the table plane, and a downward approach cone.  The resulting full 6D FK is
measured and recorded; a position-only solution is never accepted.

No ROS publisher, controller, resident service, or execution API is imported
here.  MoveIt collision planning is performed by the plan-only CLI.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from typing import Iterable, Mapping, Sequence

import numpy as np
from scipy.optimize import least_squares

from tools.lib.grasp_yaw_kinematics import (
    GraspYawKinematics,
    wrap_half_turn,
)


NOMINAL_TOWEL_SIDE_M = 0.300
PERIMETER_MARGIN_M = 0.030
# The physical table starts 140 mm in front of the base and the 300 mm towel
# must retain a 30 mm camera/workspace perimeter.  A literal outer-edge TCP is
# therefore outside both arms' task-pose envelope.  The coarse single-arm
# grasp uses a 40 mm patch inset; endpoint and correction grasps retain the
# smaller 15 mm inset.  Cloth S1/real validation must still prove that the
# 40 mm patch can carry the outer edge.
FIRST_GRASP_INSET_M = 0.040
SECOND_GRASP_INSET_M = 0.060
SECOND_PREGRASP_CLEARANCE_M = 0.010
SECOND_TRANSFER_STAGGER_M = 0.020
EDGE_GRASP_INSET_M = 0.015
FIRST_GRASP_LATERAL_OFFSET_M = 0.045
PREGRASP_CLEARANCE_M = 0.035
LIFT_CLEARANCE_M = 0.080
ARC_BASE_CLEARANCE_M = 0.025
CORRECTION_LIMIT_M = 0.030
FIRST_LAYER_TCP_Z_OFFSET_M = 0.015
SECOND_LAYER_TCP_Z_OFFSET_M = 0.016
FIRST_JAW_YAW_RAD = math.pi / 4.0
SECOND_RIGHT_JAW_YAW_RAD = 0.0
SECOND_LEFT_JAW_YAW_RAD = math.pi / 4.0

# Full-FK measurements of the pinned r0g URDF at the validated OBSERVE_CLEAR
# joint state.  The first plan-only departure is interpolated from these
# points so MoveIt never has to bridge two distant IK branches in one segment.
# The runner independently hashes both the URDF and towel contract before
# these R0-specific route points are accepted.
OBSERVE_CLEAR_TCP_BY_ARM_M = {
    "left": (0.09189654987886778, 0.057628143069419525, -0.0011709585623834173),
    "right": (0.07137517071060179, -0.2909524465958105, 0.004492941694882178),
}
OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD = {
    "left": -0.7536239764425656,
    "right": 0.7182052256208875,
}
DEPARTURE_FRACTIONS = (
    0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05,
    0.075, 0.10, 0.1125, 0.125, 0.1375, 0.15, 0.175, 0.1875,
    0.20, 0.2125, 0.225, 0.2375, 0.25, 0.2625, 0.275,
    0.35, 0.50, 0.70, 1.00,
)

MAXIMUM_TCP_POSITION_ERROR_M = 0.0025
MAXIMUM_JAW_YAW_ERROR_RAD = math.radians(4.0)
MAXIMUM_APPROACH_TILT_RAD = math.radians(70.0)
MINIMUM_JOINT_LIMIT_MARGIN_RAD = 0.025
IK_RANDOM_SEED_COUNT = 18
IK_MAXIMUM_BRANCHES = 4


class AsymmetricPlanningError(RuntimeError):
    """A task-pose candidate cannot satisfy the R0 contract."""


@dataclass(frozen=True, slots=True)
class TaskPose:
    name: str
    arm: str
    xyz_m: tuple[float, float, float]
    jaw_yaw_rad: float
    semantic: str
    layer: str


@dataclass(frozen=True, slots=True)
class PhaseSpec:
    name: str
    targets: tuple[TaskPose, ...]
    clear_pose: bool = False
    clear_arm: str | None = None
    reuse_target_of: str | None = None
    reverse_of: str | None = None
    path_cache_key: str | None = None
    attachment_event: str | None = None


@dataclass(frozen=True, slots=True)
class CandidateSpec:
    candidate_id: str
    first_active_arm: str
    first_axis: str
    first_direction: str
    second_axis: str
    second_direction: str
    second_assignment: str
    first_fold_phases: tuple[PhaseSpec, ...]
    second_fold_phases: tuple[PhaseSpec, ...]
    first_expected_footprint_xyxy_m: tuple[float, float, float, float]
    final_expected_footprint_xyxy_m: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class CorrectionProbe:
    probe_id: str
    primitive: str
    arm: str
    corner: str
    offset_xy_m: tuple[float, float]
    phases: tuple[PhaseSpec, ...]


def _finite_vector(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise AsymmetricPlanningError(f"{name} must contain {length} finite values")
    return result


def towel_bounds_from_worktable(
    span_m: Sequence[float], origin_xy_m: Sequence[float]
) -> tuple[float, float, float, float]:
    """Center one nominal towel inside the independently validated table area."""
    span_x, span_y = _finite_vector(span_m, 2, "worktable span")
    origin_x, origin_y = _finite_vector(origin_xy_m, 2, "worktable origin")
    required = NOMINAL_TOWEL_SIDE_M + 2.0 * PERIMETER_MARGIN_M
    if span_x < required or span_y < required:
        raise AsymmetricPlanningError(
            "validated worktable does not contain the towel and perimeter margin"
        )
    center_x = origin_x + span_x / 2.0
    center_y = origin_y + span_y / 2.0
    half = NOMINAL_TOWEL_SIDE_M / 2.0
    return center_x - half, center_x + half, center_y - half, center_y + half


def _pose(
    name: str,
    arm: str,
    xyz: Sequence[float],
    jaw_yaw: float,
    semantic: str,
    layer: str,
) -> TaskPose:
    if arm not in {"left", "right"}:
        raise AsymmetricPlanningError(f"invalid arm: {arm}")
    point = _finite_vector(xyz, 3, f"{name} xyz")
    if not math.isfinite(jaw_yaw):
        raise AsymmetricPlanningError(f"{name} jaw yaw must be finite")
    return TaskPose(
        name=name,
        arm=arm,
        xyz_m=point,  # type: ignore[arg-type]
        jaw_yaw_rad=wrap_half_turn(float(jaw_yaw)),
        semantic=semantic,
        layer=layer,
    )


def _single_arm_first_fold(
    bounds: tuple[float, float, float, float],
    table_z_m: float,
    arm: str,
) -> tuple[PhaseSpec, ...]:
    left, right, bottom, top = bounds
    center_x = 0.5 * (left + right)
    center_y = 0.5 * (bottom + top)
    # The calibrated homography maps screen-right almost exactly to workcell
    # -Y.  The operator-reviewed right-to-left fold is therefore bottom-to-top
    # in this metric frame, not +X-to-X as the discarded exploration assumed.
    grasp_x = center_x
    start_y = bottom + FIRST_GRASP_INSET_M
    target_y = top - FIRST_GRASP_INSET_M
    contact_z = table_z_m + FIRST_LAYER_TCP_Z_OFFSET_M
    departure_target = (0.140, start_y - 0.050, table_z_m + 0.025)
    departure_origin = OBSERVE_CLEAR_TCP_BY_ARM_M[arm]
    departure_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD[arm]
    yaw_delta = wrap_half_turn(FIRST_JAW_YAW_RAD - departure_yaw)

    def departure_pose(name: str, fraction: float) -> TaskPose:
        # The preferred right arm has a narrow shoulder/jaw contact band near
        # 42 degrees while the TCP is still folded close to OBSERVE_CLEAR.
        # Keep its measured clear yaw through the first 10 percent, then turn
        # to the 45-degree grasp direction over four verified task-space
        # waypoints after the TCP has moved away from the shoulder.
        if arm == "right" and fraction <= 0.10:
            jaw_yaw = departure_yaw
        elif arm == "right" and fraction <= 0.15:
            blend = (fraction - 0.10) / 0.05
            jaw_yaw = departure_yaw + blend * (
                FIRST_JAW_YAW_RAD - departure_yaw
            )
        elif arm == "right":
            jaw_yaw = FIRST_JAW_YAW_RAD
        else:
            jaw_yaw = departure_yaw + fraction * yaw_delta
        return _pose(
            name,
            arm,
            tuple(
                origin + fraction * (target - origin)
                for origin, target in zip(
                    departure_origin, departure_target, strict=True
                )
            ),
            jaw_yaw,
            "released_retreat",
            "one_layer",
        )

    points: list[TaskPose] = [
        *(departure_pose(f"first_departure_{index:02d}", fraction)
          for index, fraction in enumerate(DEPARTURE_FRACTIONS, start=1)),
        _pose(
            "first_departure_workspace", arm,
            (0.230, start_y - 0.025, contact_z + 0.030),
            FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
        ),
        _pose(
            "first_pregrasp", arm,
            (grasp_x, start_y, contact_z + PREGRASP_CLEARANCE_M),
            FIRST_JAW_YAW_RAD, "pregrasp_open", "one_layer",
        ),
        _pose(
            "first_contact", arm, (grasp_x, start_y, contact_z),
            FIRST_JAW_YAW_RAD, "contact", "one_layer",
        ),
        _pose(
            "first_lift", arm,
            (grasp_x, start_y, contact_z + 0.040),
            FIRST_JAW_YAW_RAD, "attached_lift", "one_layer",
        ),
    ]
    # A geometric semicircle peaks at 150 mm and drives the 5-axis wrist-flex
    # onto its lower limit on the placement half.  The physically intended
    # primitive is instead a low lift-over-place: lift the grasped patch,
    # traverse above the towel, then let the free outer edge drape while the
    # TCP descends.  Surface-cloth/real validation still owns whether the cloth
    # follows this collision-free arm envelope.
    transfer_points = (
        (start_y + 0.039, contact_z + 0.040),
        (start_y + 0.089, contact_z + 0.045),
        (center_y + 0.009, contact_z + 0.033),
        (center_y + 0.049, contact_z + 0.033),
        (target_y, contact_z + 0.033),
    )
    for index, (transfer_y, transfer_z) in enumerate(transfer_points, start=1):
        points.append(
            _pose(
                f"first_transfer_{index}", arm,
                (grasp_x, transfer_y, transfer_z),
                FIRST_JAW_YAW_RAD, "attached_transfer", "one_layer",
            )
        )
    points.extend(
        [
            _pose(
                "first_descend", arm,
                (grasp_x, target_y, contact_z + 0.015),
                FIRST_JAW_YAW_RAD, "attached_transfer", "one_layer",
            ),
            _pose(
                "first_laydown", arm, (grasp_x, target_y, contact_z),
                FIRST_JAW_YAW_RAD, "attached_laydown", "one_layer",
            ),
            _pose(
                "first_retreat", arm,
                (grasp_x, target_y, contact_z + 0.063),
                FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
            ),
            _pose(
                "first_return_1", arm,
                (grasp_x, center_y + 0.049, contact_z + 0.050),
                FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
            ),
            _pose(
                "first_return_2", arm,
                (grasp_x, start_y + 0.089, contact_z + 0.045),
                FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
            ),
            _pose(
                "first_return_3", arm,
                (grasp_x, start_y + 0.039, contact_z + 0.040),
                FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
            ),
            _pose(
                "first_return_4", arm,
                (0.230, start_y - 0.025, contact_z + 0.030),
                FIRST_JAW_YAW_RAD, "released_retreat", "one_layer",
            ),
            *(departure_pose(f"first_return_departure_{index:02d}", fraction)
              for index, fraction in enumerate(
                  reversed(DEPARTURE_FRACTIONS), start=1
              )),
        ]
    )
    phases = []
    for item in points:
        event = None
        if item.name == "first_contact":
            event = "attach_single_layer_patch_after_contact_gate"
        elif item.name == "first_laydown":
            event = "release_after_laydown_gate"
        phases.append(PhaseSpec(item.name, (item,), attachment_event=event))
    phases.append(PhaseSpec("first_reobserve_clear", (), clear_pose=True))
    return tuple(phases)


def _second_bimanual_fold(
    first_footprint: tuple[float, float, float, float],
    table_z_m: float,
    direction: str,
    assignment: str,
) -> tuple[tuple[PhaseSpec, ...], tuple[float, float, float, float]]:
    left, right, bottom, top = first_footprint
    center_x = 0.5 * (left + right)
    if direction == "positive_to_negative":
        start_x = right - SECOND_GRASP_INSET_M
        target_x = left + SECOND_GRASP_INSET_M
        final = (left, center_x, bottom, top)
    elif direction == "negative_to_positive":
        start_x = left + SECOND_GRASP_INSET_M
        target_x = right - SECOND_GRASP_INSET_M
        final = (center_x, right, bottom, top)
    else:
        raise AsymmetricPlanningError(f"invalid second direction: {direction}")
    # Use the two physical endpoints of the already halved edge.  An inset of
    # 15 mm produced a measured cross-gripper mesh overlap during the central
    # transfer; keeping the TCPs at the endpoint lines gives 150 mm separation
    # while the open jaw still overlaps cloth inward from each boundary.
    endpoint_y = (bottom, top)
    if assignment == "left_to_high_y":
        arm_for_endpoint = ("right", "left")
    elif assignment == "left_to_low_y":
        arm_for_endpoint = ("left", "right")
    else:
        raise AsymmetricPlanningError(f"invalid second assignment: {assignment}")

    contact_z = table_z_m + SECOND_LAYER_TCP_Z_OFFSET_M
    direction_sign = -1.0 if direction == "positive_to_negative" else 1.0

    def paired(
        name: str,
        x: float,
        z: float,
        semantic: str,
        stagger_m: float = 0.0,
    ) -> tuple[TaskPose, ...]:
        return tuple(
            _pose(
                f"{name}_{arm}",
                arm,
                (
                    x + (
                        direction_sign * stagger_m
                        if arm == "right"
                        else -direction_sign * stagger_m
                    ),
                    y,
                    z,
                ),
                (
                    SECOND_LEFT_JAW_YAW_RAD
                    if arm == "left"
                    else SECOND_RIGHT_JAW_YAW_RAD
                ),
                semantic,
                "two_layer_bundle",
            )
            for y, arm in zip(endpoint_y, arm_for_endpoint, strict=True)
        )

    # Reuse the collision-mapped right-arm departure before asking both arms
    # to approach the folded towel.  The first footprint retains the original
    # top edge, so its pre-fold bottom is recovered exactly here.
    original_bottom = 2.0 * bottom - top
    original_bounds = (left, right, original_bottom, top)
    first_route = tuple(
        phase.targets[0]
        for phase in _single_arm_first_fold(
            original_bounds, table_z_m, "right"
        )
        if phase.name.startswith("first_departure")
    )
    right_departure_targets = tuple(
        _pose(
            target.name.replace("first_", "second_", 1),
            "right",
            target.xyz_m,
            target.jaw_yaw_rad,
            "pregrasp_open",
            "two_layer_bundle",
        )
        for target in first_route
    )
    pregrasp_targets = paired(
        "second_pregrasp",
        start_x,
        contact_z + SECOND_PREGRASP_CLEARANCE_M,
        "pregrasp_open",
    )
    pregrasp_by_arm = {target.arm: target for target in pregrasp_targets}
    left_y = pregrasp_by_arm["left"].xyz_m[1]
    left_clear_origin = OBSERVE_CLEAR_TCP_BY_ARM_M["left"]
    left_clear_yaw = OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD["left"]
    left_near_target = (0.140, left_y + 0.011, table_z_m + 0.030)
    left_near_departure = tuple(
        (
            tuple(
                origin + fraction * (target - origin)
                for origin, target in zip(
                    left_clear_origin, left_near_target, strict=True
                )
            ),
            left_clear_yaw,
        )
        for fraction in (0.05, 0.10, 0.20, 0.35, 0.50, 0.70, 1.00)
    )
    left_departure_targets = tuple(
        _pose(
            f"second_left_departure_{index:02d}",
            "left",
            xyz,
            yaw,
            "pregrasp_open",
            "two_layer_bundle",
        )
        for index, (xyz, yaw) in enumerate(
            left_near_departure
            + (
                ((0.180, left_y + 0.006, table_z_m + 0.050), -0.75362398),
                ((0.230, left_y + 0.001, table_z_m + 0.055), -0.300),
                ((0.280, left_y, table_z_m + 0.055), 0.200),
                ((0.340, left_y, table_z_m + 0.045), SECOND_LEFT_JAW_YAW_RAD),
            ),
            start=1,
        )
    )

    def second_return_name(index: int, target: TaskPose) -> str:
        if target.name.endswith("workspace"):
            return "second_return_workspace"
        return f"second_return_departure_{index:02d}"

    phases = [
        *(
            PhaseSpec(target.name, (target,))
            for target in right_departure_targets
        ),
        PhaseSpec(
            "second_pregrasp_right",
            (pregrasp_by_arm["right"],),
        ),
        *(
            PhaseSpec(target.name, (target,))
            for target in left_departure_targets
        ),
        PhaseSpec(
            "second_pregrasp_left",
            (pregrasp_by_arm["left"],),
        ),
        PhaseSpec(
            "second_contact",
            paired("second_contact", start_x, contact_z, "contact"),
            attachment_event="attach_two_endpoint_bundle_patches_after_dual_contact_gate",
        ),
        PhaseSpec(
            "second_lift",
            paired(
                "second_lift", start_x,
                contact_z + 0.040, "attached_lift",
            ),
        ),
    ]
    transfer_xs = tuple(
        start_x + fraction * (target_x - start_x)
        for fraction in (0.20, 0.40, 0.50, 0.80, 1.00)
    )
    transfer_zs = (
        contact_z + 0.030,
        contact_z + 0.005,
        contact_z + 0.005,
        contact_z + 0.020,
        contact_z + 0.015,
    )
    transfer_staggers = (
        SECOND_TRANSFER_STAGGER_M,
        SECOND_TRANSFER_STAGGER_M,
        SECOND_TRANSFER_STAGGER_M,
        SECOND_TRANSFER_STAGGER_M,
        SECOND_TRANSFER_STAGGER_M,
    )
    for index, (transfer_x, transfer_z, stagger_m) in enumerate(
        zip(transfer_xs, transfer_zs, transfer_staggers, strict=True), start=1
    ):
        phases.append(
            PhaseSpec(
                f"second_transfer_{index}",
                paired(
                    f"second_transfer_{index}",
                    transfer_x,
                    transfer_z,
                    "attached_transfer",
                    stagger_m,
                ),
            )
        )
    phases.extend(
        [
            PhaseSpec(
                "second_laydown",
                paired(
                    "second_laydown",
                    target_x,
                    contact_z,
                    "attached_laydown",
                    SECOND_TRANSFER_STAGGER_M,
                ),
                attachment_event="release_both_patches_after_laydown_gate",
            ),
            PhaseSpec(
                "second_retreat",
                paired(
                    "second_retreat", target_x,
                    contact_z + SECOND_PREGRASP_CLEARANCE_M,
                    "released_retreat",
                    SECOND_TRANSFER_STAGGER_M,
                ),
            ),
            PhaseSpec(
                "second_left_return_lift",
                (
                    _pose(
                        "second_left_return_lift",
                        "left",
                        (
                            target_x
                            - direction_sign * SECOND_TRANSFER_STAGGER_M,
                            pregrasp_by_arm["left"].xyz_m[1],
                            contact_z + 0.050,
                        ),
                        SECOND_LEFT_JAW_YAW_RAD,
                        "released_retreat",
                        "two_layer_bundle",
                    ),
                ),
            ),
            PhaseSpec(
                "second_left_return_over_start",
                (
                    _pose(
                        "second_left_return_over_start",
                        "left",
                        (
                            start_x,
                            pregrasp_by_arm["left"].xyz_m[1],
                            contact_z + 0.050,
                        ),
                        SECOND_LEFT_JAW_YAW_RAD,
                        "released_retreat",
                        "two_layer_bundle",
                    ),
                ),
            ),
            PhaseSpec(
                "second_left_return_pregrasp",
                (
                    _pose(
                        "second_left_return_pregrasp",
                        "left",
                        pregrasp_by_arm["left"].xyz_m,
                        SECOND_LEFT_JAW_YAW_RAD,
                        "released_retreat",
                        "two_layer_bundle",
                    ),
                ),
                reuse_target_of="second_pregrasp_left",
            ),
            PhaseSpec(
                "second_left_return_pregrasp_segment",
                (
                    _pose(
                        "second_left_return_pregrasp_segment",
                        "left",
                        left_departure_targets[-1].xyz_m,
                        left_departure_targets[-1].jaw_yaw_rad,
                        "released_retreat",
                        "two_layer_bundle",
                    ),
                ),
                reverse_of="second_pregrasp_left",
            ),
            *(
                PhaseSpec(
                    f"second_left_return_departure_{index:02d}",
                    (
                        _pose(
                            f"second_left_return_departure_{index:02d}",
                            "left",
                            destination.xyz_m,
                            destination.jaw_yaw_rad,
                            "released_retreat",
                            "two_layer_bundle",
                        ),
                    ),
                    reverse_of=source.name,
                )
                for index, (source, destination) in enumerate(
                    zip(
                        reversed(left_departure_targets[1:]),
                        reversed(left_departure_targets[:-1]),
                        strict=True,
                    ),
                    start=1,
                )
            ),
            PhaseSpec(
                "second_left_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm="left",
                reverse_of=left_departure_targets[0].name,
            ),
            *(
                PhaseSpec(
                    second_return_name(index, target),
                    (
                        _pose(
                            second_return_name(index, target),
                            "right",
                            target.xyz_m,
                            target.jaw_yaw_rad,
                            "released_retreat",
                            "two_layer_bundle",
                        ),
                    ),
                )
                for index, target in enumerate(
                    reversed(right_departure_targets), start=1
                )
            ),
            PhaseSpec(
                "second_right_reobserve_clear",
                (),
                clear_pose=True,
                clear_arm="right",
            ),
        ]
    )
    return tuple(phases), final


def build_asymmetric_candidates(
    towel_bounds: Sequence[float], table_z_m: float
) -> tuple[CandidateSpec, ...]:
    """Enumerate the approved right-edge single-arm fold strategy.

    The first direction is intentionally fixed to right-to-left after the
    operator's physical-structure review.  Arm choice, second-fold direction,
    and endpoint assignment remain candidates and must be decided by actual
    task-pose and collision planning.
    """
    bounds = _finite_vector(towel_bounds, 4, "towel bounds")
    left, right, bottom, top = bounds
    if not left < right or not bottom < top:
        raise AsymmetricPlanningError("towel bounds must have positive area")
    if not math.isclose(right - left, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise AsymmetricPlanningError("towel x span must be nominal 300 mm")
    if not math.isclose(top - bottom, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise AsymmetricPlanningError("towel y span must be nominal 300 mm")
    if not math.isfinite(table_z_m):
        raise AsymmetricPlanningError("table z must be finite")

    first_footprint = (left, right, 0.5 * (bottom + top), top)
    result = []
    for first_arm in ("right", "left"):
        first_phases = _single_arm_first_fold(bounds, table_z_m, first_arm)
        for second_direction in ("positive_to_negative", "negative_to_positive"):
            for assignment in ("left_to_high_y", "left_to_low_y"):
                second_phases, final = _second_bimanual_fold(
                    first_footprint, table_z_m, second_direction, assignment
                )
                result.append(
                    CandidateSpec(
                        candidate_id=(
                            f"first_{first_arm}_screen_right_to_left__second_"
                            f"{second_direction}__{assignment}"
                        ),
                        first_active_arm=first_arm,
                        first_axis="y",
                        first_direction="negative_to_positive",
                        second_axis="x",
                        second_direction=second_direction,
                        second_assignment=assignment,
                        first_fold_phases=first_phases,
                        second_fold_phases=second_phases,
                        first_expected_footprint_xyxy_m=first_footprint,
                        final_expected_footprint_xyxy_m=final,
                    )
                )
    return tuple(result)


def build_correction_probes(
    first_footprint: Sequence[float], table_z_m: float
) -> tuple[CorrectionProbe, ...]:
    """Build both signed extrema for each bounded post-first-fold correction.

    ``micro_drag`` is restricted to the fold-normal Y axis.  The lifting
    correction handles edge-tangent X error.  Each is checked on two reachable
    patches of the placed edge, yielding eight independent clear-to-clear
    probes.
    """
    left, right, bottom, top = _finite_vector(
        first_footprint, 4, "first footprint"
    )
    # Use two diagonally separated patches that each arm can approach with
    # useful joint margin.  These are deliberately not literal corners: the
    # outermost 40 mm remains cloth carried by the grasped patch and is left
    # to the later surface-cloth/real gate.
    corners = (
        # The left arm's lifted +30 mm X extreme loses its task-pose branch
        # at a 55 mm inset.  Seventy-five millimetres preserves the full
        # signed correction envelope without pretending the literal corner
        # is reachable by this five-axis chain.
        ("high_x_patch", "left", (right - 0.075, top - EDGE_GRASP_INSET_M)),
        ("low_x_patch", "right", (left + 0.055, bottom + EDGE_GRASP_INSET_M)),
    )
    contact_z = table_z_m + FIRST_LAYER_TCP_Z_OFFSET_M
    probes = []
    for corner, arm, (start_x, start_y) in corners:
        for primitive, offsets in (
            ("micro_drag", ((0.0, -CORRECTION_LIMIT_M), (0.0, CORRECTION_LIMIT_M))),
            ("lift_pull_place", ((-CORRECTION_LIMIT_M, 0.0), (CORRECTION_LIMIT_M, 0.0))),
        ):
            for offset_x, offset_y in offsets:
                sign = "neg" if offset_x + offset_y < 0.0 else "pos"
                probe_id = f"{primitive}_{corner}_{sign}"
                pregrasp = _pose(
                    f"{probe_id}_pregrasp", arm,
                    (start_x, start_y, contact_z + PREGRASP_CLEARANCE_M),
                    0.0, "pregrasp_open", "one_layer",
                )
                contact = _pose(
                    f"{probe_id}_contact", arm,
                    (start_x, start_y, contact_z),
                    0.0, "contact", "one_layer",
                )
                target_z = (
                    contact_z
                    if primitive == "micro_drag"
                    else contact_z + 0.030
                )
                phases = [
                    PhaseSpec(
                        f"{probe_id}_pregrasp",
                        (pregrasp,),
                        path_cache_key=f"correction_pregrasp_{corner}_{arm}",
                    ),
                    PhaseSpec(
                        f"{probe_id}_contact", (contact,),
                        attachment_event="attach_single_layer_patch_after_contact_gate",
                    ),
                ]
                if primitive == "lift_pull_place":
                    phases.append(
                        PhaseSpec(
                            f"{probe_id}_lift",
                            (
                                _pose(
                                    f"{probe_id}_lift", arm,
                                    (start_x, start_y, target_z),
                                    0.0, "attached_lift", "one_layer",
                                ),
                            ),
                        )
                    )
                phases.append(
                    PhaseSpec(
                        f"{probe_id}_target",
                        (
                            _pose(
                                f"{probe_id}_target", arm,
                                (start_x + offset_x, start_y + offset_y, target_z),
                                0.0, "attached_correction", "one_layer",
                            ),
                        ),
                        attachment_event=(
                            "release_after_correction_gate"
                            if primitive == "micro_drag" else None
                        ),
                    )
                )
                if primitive == "lift_pull_place":
                    phases.append(
                        PhaseSpec(
                            f"{probe_id}_laydown",
                            (
                                _pose(
                                    f"{probe_id}_laydown", arm,
                                    (start_x + offset_x, start_y + offset_y, contact_z),
                                    0.0, "attached_laydown", "one_layer",
                                ),
                            ),
                            attachment_event="release_after_correction_gate",
                        )
                    )
                phases.extend(
                    [
                        PhaseSpec(
                            f"{probe_id}_retreat",
                            (
                                _pose(
                                    f"{probe_id}_retreat", arm,
                                    (
                                        start_x + offset_x,
                                        start_y + offset_y,
                                        contact_z + PREGRASP_CLEARANCE_M,
                                    ),
                                    0.0, "released_retreat", "one_layer",
                                ),
                            ),
                        ),
                        PhaseSpec(
                            f"{probe_id}_return_pregrasp",
                            (
                                _pose(
                                    f"{probe_id}_return_pregrasp",
                                    arm,
                                    pregrasp.xyz_m,
                                    pregrasp.jaw_yaw_rad,
                                    "released_retreat",
                                    "one_layer",
                                ),
                            ),
                            reuse_target_of=f"{probe_id}_pregrasp",
                        ),
                        PhaseSpec(
                            f"{probe_id}_reobserve_clear",
                            (),
                            clear_pose=True,
                            clear_arm=arm,
                            reverse_of=f"{probe_id}_pregrasp",
                        ),
                    ]
                )
                probes.append(
                    CorrectionProbe(
                        probe_id=probe_id,
                        primitive=primitive,
                        arm=arm,
                        corner=corner,
                        offset_xy_m=(offset_x, offset_y),
                        phases=tuple(phases),
                    )
                )
    return tuple(probes)


def _jaw_yaw(axis: np.ndarray) -> float:
    return wrap_half_turn(math.atan2(float(axis[1]), float(axis[0])))


def evaluate_task_pose(
    kinematics: GraspYawKinematics,
    pose: TaskPose,
    positions_rad: Sequence[float],
    lower_rad: Sequence[float],
    upper_rad: Sequence[float],
) -> dict[str, object]:
    q = np.asarray(_finite_vector(positions_rad, 5, "arm positions"))
    lower = np.asarray(_finite_vector(lower_rad, 5, "lower bounds"))
    upper = np.asarray(_finite_vector(upper_rad, 5, "upper bounds"))
    if np.any(q < lower) or np.any(q > upper):
        raise AsymmetricPlanningError(f"{pose.name} is outside joint bounds")
    by_name = dict(zip(kinematics.arm_joints, q, strict=True))
    rotation, xyz = kinematics.tcp_pose_in_root(by_name)
    approach = kinematics.approach_axis_in_root(by_name)
    finger = kinematics.finger_axis_in_root(by_name)
    target = np.asarray(pose.xyz_m)
    position_error = float(np.linalg.norm(xyz - target))
    finger_yaw = _jaw_yaw(finger)
    yaw_error = abs(wrap_half_turn(finger_yaw - pose.jaw_yaw_rad))
    downward_dot = float(np.clip(-approach[2], -1.0, 1.0))
    approach_tilt = float(math.acos(downward_dot))
    margins = np.minimum(q - lower, upper - q)
    minimum_margin = float(np.min(margins))
    passed = (
        position_error <= MAXIMUM_TCP_POSITION_ERROR_M
        and yaw_error <= MAXIMUM_JAW_YAW_ERROR_RAD
        and approach_tilt <= MAXIMUM_APPROACH_TILT_RAD
        and minimum_margin >= MINIMUM_JOINT_LIMIT_MARGIN_RAD
    )
    return {
        "task_pose_pass": passed,
        "target_xyz_m": list(pose.xyz_m),
        "actual_xyz_m": [float(value) for value in xyz],
        "tcp_position_error_m": position_error,
        "tcp_rotation_matrix": [[float(value) for value in row] for row in rotation],
        "approach_axis_workcell": [float(value) for value in approach],
        "approach_tilt_from_down_rad": approach_tilt,
        "approach_tilt_from_down_deg": math.degrees(approach_tilt),
        "finger_axis_workcell": [float(value) for value in finger],
        "target_jaw_yaw_rad": pose.jaw_yaw_rad,
        "actual_jaw_yaw_rad": finger_yaw,
        "jaw_yaw_error_rad": yaw_error,
        "minimum_joint_limit_margin_rad": minimum_margin,
        "joint_limit_margins_rad": [float(value) for value in margins],
        "positions_rad": [float(value) for value in q],
    }


def solve_task_pose_branches(
    kinematics: GraspYawKinematics,
    pose: TaskPose,
    lower_rad: Sequence[float],
    upper_rad: Sequence[float],
    preferred_positions_rad: Sequence[float],
    fallback_positions_rad: Sequence[float],
) -> tuple[dict[str, object], ...]:
    """Solve deterministic task-pose branches and return passing ones only."""
    lower = np.asarray(_finite_vector(lower_rad, 5, "lower bounds"))
    upper = np.asarray(_finite_vector(upper_rad, 5, "upper bounds"))
    preferred = np.asarray(
        _finite_vector(preferred_positions_rad, 5, "preferred positions")
    )
    fallback = np.asarray(
        _finite_vector(fallback_positions_rad, 5, "fallback positions")
    )
    if np.any(lower >= upper):
        raise AsymmetricPlanningError("joint bounds must satisfy lower < upper")
    preferred = np.clip(preferred, lower, upper)
    fallback = np.clip(fallback, lower, upper)
    midpoint = 0.5 * (lower + upper)
    digest = hashlib.sha256(
        f"{pose.name}|{pose.arm}|{pose.xyz_m}|{pose.jaw_yaw_rad}".encode()
    ).digest()
    rng = np.random.default_rng(int.from_bytes(digest[:8], "little"))
    seeds = [preferred, fallback, midpoint]
    seeds.extend(
        lower + (upper - lower) * rng.random(5)
        for _ in range(IK_RANDOM_SEED_COUNT)
    )
    target = np.asarray(pose.xyz_m)
    span = upper - lower
    maximum_approach_tilt = MAXIMUM_APPROACH_TILT_RAD

    def residual(q: np.ndarray, continuity_anchor: np.ndarray) -> np.ndarray:
        by_name = dict(zip(kinematics.arm_joints, q, strict=True))
        _, xyz = kinematics.tcp_pose_in_root(by_name)
        finger = kinematics.finger_axis_in_root(by_name)
        approach = kinematics.approach_axis_in_root(by_name)
        yaw_error = wrap_half_turn(_jaw_yaw(finger) - pose.jaw_yaw_rad)
        downward_dot = float(np.clip(-approach[2], -1.0, 1.0))
        approach_tilt = math.acos(downward_dot)
        cone_violation = max(0.0, approach_tilt - maximum_approach_tilt)
        return np.concatenate(
            (
                (xyz - target) / 0.0020,
                np.asarray((yaw_error / math.radians(2.0),)),
                np.asarray((cone_violation / math.radians(2.0),)),
                # The arm is underactuated for arbitrary 6D pose control.
                # Inside the approved downward cone, prefer the continuous
                # branch instead of inventing a needless vertical-wrist
                # objective that can flip shoulder/elbow posture.
                0.025 * (q - continuity_anchor) / span,
                0.002 * (q - midpoint) / span,
            )
        )

    accepted: list[dict[str, object]] = []
    for seed in seeds:
        result = least_squares(
            lambda q, anchor=seed: residual(q, anchor),
            seed,
            bounds=(lower, upper),
            max_nfev=900,
            xtol=1.0e-11,
            ftol=1.0e-11,
            gtol=1.0e-11,
        )
        evaluation = evaluate_task_pose(
            kinematics, pose, result.x, lower, upper
        )
        if not evaluation["task_pose_pass"]:
            continue
        q = np.asarray(evaluation["positions_rad"])
        if any(
            np.max(np.abs(q - np.asarray(item["positions_rad"]))) < 1.0e-4
            for item in accepted
        ):
            continue
        evaluation["solver_cost"] = float(result.cost)
        evaluation["solver_optimality"] = float(result.optimality)
        evaluation["distance_from_preferred_rad"] = float(
            np.linalg.norm(q - preferred)
        )
        accepted.append(evaluation)
    accepted.sort(
        key=lambda item: (
            float(item["distance_from_preferred_rad"]),
            float(item["tcp_position_error_m"]),
            float(item["jaw_yaw_error_rad"]),
            -float(item["minimum_joint_limit_margin_rad"]),
        )
    )
    return tuple(accepted[:IK_MAXIMUM_BRANCHES])


def phase_to_dict(phase: PhaseSpec) -> dict[str, object]:
    return {
        "name": phase.name,
        "clear_pose": phase.clear_pose,
        "clear_arm": phase.clear_arm,
        "reuse_target_of": phase.reuse_target_of,
        "reverse_of": phase.reverse_of,
        "path_cache_key": phase.path_cache_key,
        "attachment_event": phase.attachment_event,
        "targets": [asdict(target) for target in phase.targets],
    }


def validate_phase_contract(phases: Iterable[PhaseSpec]) -> None:
    names = []
    attached = False
    for phase in phases:
        if phase.name in names:
            raise AsymmetricPlanningError(f"duplicate phase name: {phase.name}")
        names.append(phase.name)
        if phase.clear_pose and phase.targets:
            raise AsymmetricPlanningError("clear phase cannot contain task targets")
        if phase.clear_arm is not None and (
            not phase.clear_pose or phase.clear_arm not in {"left", "right"}
        ):
            raise AsymmetricPlanningError(
                "clear_arm requires a clear phase and a valid arm"
            )
        if phase.reverse_of is not None:
            active_arms = {target.arm for target in phase.targets}
            valid_clear_reverse = phase.clear_pose and phase.clear_arm is not None
            valid_task_reverse = not phase.clear_pose and len(active_arms) == 1
            if not (valid_clear_reverse or valid_task_reverse):
                raise AsymmetricPlanningError(
                    "reverse_of requires a single-arm clear or task phase"
                )
            if phase.reverse_of not in names[:-1]:
                raise AsymmetricPlanningError(
                    f"reverse_of must name an earlier phase: {phase.reverse_of}"
                )
        if phase.reuse_target_of is not None:
            if phase.clear_pose or not phase.targets:
                raise AsymmetricPlanningError(
                    "reuse_target_of requires a non-clear task phase"
                )
            if phase.reuse_target_of not in names[:-1]:
                raise AsymmetricPlanningError(
                    "reuse_target_of must name an earlier phase: "
                    f"{phase.reuse_target_of}"
                )
        if phase.path_cache_key is not None and (
            not phase.path_cache_key or phase.clear_pose or not phase.targets
        ):
            raise AsymmetricPlanningError(
                "path_cache_key requires a named non-clear task phase"
            )
        for target in phase.targets:
            if target.arm not in {"left", "right"}:
                raise AsymmetricPlanningError(f"invalid target arm: {target.arm}")
        event = phase.attachment_event or ""
        if event.startswith("attach"):
            if attached:
                raise AsymmetricPlanningError("attachment cannot be nested")
            attached = True
        if event.startswith("release"):
            if not attached:
                raise AsymmetricPlanningError("release requires an attachment")
            attached = False
    if attached:
        raise AsymmetricPlanningError("phase sequence ended while cloth was attached")
