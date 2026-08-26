"""Shared task-pose geometry and IK for the canonical towel-fold gate.

The SO-101 arm has five controlled arm joints, so it cannot satisfy an
arbitrary six-degree-of-freedom pose.  This module instead solves the task
constraints that matter for a towel grasp: TCP XYZ, the jaw-opening line in
the table plane, and an explicit phase-specific approach cone.  New contacts
use a downward cone; attached transfer may use the wider declared envelope.
The resulting full 6D FK is measured and recorded; a position-only solution
is never accepted.

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
EDGE_GRASP_INSET_M = 0.015
PREGRASP_CLEARANCE_M = 0.035
CORRECTION_LIMIT_M = 0.030
FIRST_LAYER_TCP_Z_OFFSET_M = 0.015
SECOND_LAYER_TCP_Z_OFFSET_M = 0.016

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
MAXIMUM_TCP_POSITION_ERROR_M = 0.0025
MAXIMUM_JAW_YAW_ERROR_RAD = math.radians(4.0)
MAXIMUM_APPROACH_TILT_RAD = math.radians(70.0)
MAXIMUM_ATTACHED_TRANSFER_TILT_RAD = math.radians(90.0)
MINIMUM_JOINT_LIMIT_MARGIN_RAD = 0.025
IK_RANDOM_SEED_COUNT = 18
IK_MAXIMUM_BRANCHES = 4


class TowelPlanningError(RuntimeError):
    """A task-pose candidate cannot satisfy the R0 contract."""


@dataclass(frozen=True, slots=True)
class TaskPose:
    name: str
    arm: str
    xyz_m: tuple[float, float, float]
    jaw_yaw_rad: float
    semantic: str
    layer: str
    maximum_approach_tilt_rad: float = MAXIMUM_APPROACH_TILT_RAD


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
    first_arm_assignment: str
    first_axis: str
    first_direction: str
    second_axis: str
    second_direction: str
    second_active_arm: str
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
        raise TowelPlanningError(f"{name} must contain {length} finite values")
    return result


def towel_bounds_from_worktable(
    span_m: Sequence[float], origin_xy_m: Sequence[float]
) -> tuple[float, float, float, float]:
    """Center one nominal towel inside the independently validated table area."""
    span_x, span_y = _finite_vector(span_m, 2, "worktable span")
    origin_x, origin_y = _finite_vector(origin_xy_m, 2, "worktable origin")
    required = NOMINAL_TOWEL_SIDE_M + 2.0 * PERIMETER_MARGIN_M
    if span_x < required or span_y < required:
        raise TowelPlanningError(
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
    maximum_approach_tilt_rad: float = MAXIMUM_APPROACH_TILT_RAD,
) -> TaskPose:
    if arm not in {"left", "right"}:
        raise TowelPlanningError(f"invalid arm: {arm}")
    point = _finite_vector(xyz, 3, f"{name} xyz")
    if not math.isfinite(jaw_yaw):
        raise TowelPlanningError(f"{name} jaw yaw must be finite")
    if (
        not math.isfinite(maximum_approach_tilt_rad)
        or not 0.0 < maximum_approach_tilt_rad <= math.pi
    ):
        raise TowelPlanningError(
            f"{name} maximum approach tilt must be in (0, pi]"
        )
    return TaskPose(
        name=name,
        arm=arm,
        xyz_m=point,  # type: ignore[arg-type]
        jaw_yaw_rad=wrap_half_turn(float(jaw_yaw)),
        semantic=semantic,
        layer=layer,
        maximum_approach_tilt_rad=float(maximum_approach_tilt_rad),
    )


# Public construction helpers shared by alternative fold topologies.  The
# task-pose solver and MoveIt gate stay topology-agnostic; only the phase
# builder decides how many arms grasp each fold edge.
finite_vector = _finite_vector
task_pose = _pose


def build_correction_probes(
    first_footprint: Sequence[float], table_z_m: float
) -> tuple[CorrectionProbe, ...]:
    """Build both signed extrema for each bounded post-first-fold correction.

    ``micro_drag`` is restricted to the first fold's normal X axis.  The
    lifting correction handles edge-tangent Y error.  Each is checked on two
    reachable
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
        ("high_y_patch", "left", (right - 0.075, top - EDGE_GRASP_INSET_M)),
        ("low_y_patch", "right", (left + 0.055, bottom + EDGE_GRASP_INSET_M)),
    )
    contact_z = table_z_m + FIRST_LAYER_TCP_Z_OFFSET_M
    probes = []
    for corner, arm, (start_x, start_y) in corners:
        for primitive, offsets in (
            (
                "micro_drag",
                ((-CORRECTION_LIMIT_M, 0.0), (CORRECTION_LIMIT_M, 0.0)),
            ),
            (
                "lift_pull_place",
                ((0.0, -CORRECTION_LIMIT_M), (0.0, CORRECTION_LIMIT_M)),
            ),
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
        raise TowelPlanningError(f"{pose.name} is outside joint bounds")
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
        and approach_tilt <= pose.maximum_approach_tilt_rad
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
        raise TowelPlanningError("joint bounds must satisfy lower < upper")
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
    maximum_approach_tilt = pose.maximum_approach_tilt_rad

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
            raise TowelPlanningError(f"duplicate phase name: {phase.name}")
        names.append(phase.name)
        if phase.clear_pose and phase.targets:
            raise TowelPlanningError("clear phase cannot contain task targets")
        if phase.clear_arm is not None and (
            not phase.clear_pose or phase.clear_arm not in {"left", "right"}
        ):
            raise TowelPlanningError(
                "clear_arm requires a clear phase and a valid arm"
            )
        if phase.reverse_of is not None:
            active_arms = {target.arm for target in phase.targets}
            valid_clear_reverse = phase.clear_pose and phase.clear_arm is not None
            valid_task_reverse = not phase.clear_pose and len(active_arms) == 1
            if not (valid_clear_reverse or valid_task_reverse):
                raise TowelPlanningError(
                    "reverse_of requires a single-arm clear or task phase"
                )
            if phase.reverse_of not in names[:-1]:
                raise TowelPlanningError(
                    f"reverse_of must name an earlier phase: {phase.reverse_of}"
                )
        if phase.reuse_target_of is not None:
            if phase.clear_pose or not phase.targets:
                raise TowelPlanningError(
                    "reuse_target_of requires a non-clear task phase"
                )
            if phase.reuse_target_of not in names[:-1]:
                raise TowelPlanningError(
                    "reuse_target_of must name an earlier phase: "
                    f"{phase.reuse_target_of}"
                )
        if phase.path_cache_key is not None and (
            not phase.path_cache_key or phase.clear_pose or not phase.targets
        ):
            raise TowelPlanningError(
                "path_cache_key requires a named non-clear task phase"
            )
        for target in phase.targets:
            if target.arm not in {"left", "right"}:
                raise TowelPlanningError(f"invalid target arm: {target.arm}")
            if (
                not math.isfinite(target.maximum_approach_tilt_rad)
                or not 0.0 < target.maximum_approach_tilt_rad <= math.pi
            ):
                raise TowelPlanningError(
                    f"invalid approach cone for target: {target.name}"
                )
        event = phase.attachment_event or ""
        if event.startswith("attach"):
            if attached:
                raise TowelPlanningError("attachment cannot be nested")
            attached = True
        if event.startswith("release"):
            if not attached:
                raise TowelPlanningError("release requires an attachment")
            attached = False
    if attached:
        raise TowelPlanningError("phase sequence ended while cloth was attached")
