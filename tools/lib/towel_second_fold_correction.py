"""Fail-closed geometry for the left-to-right second towel fold.

The nominal second fold is performed by the left arm.  After release and a
clear top-camera observation, the right arm may move only the free edge of
the upper two-layer bundle.  A plain towel silhouette is not sufficient to
identify that edge, so correction planning requires an explicit metric
free-edge observation and rejects low-confidence or occluded input.

This module contains geometry only.  It does not import ROS, issue robot
commands, or claim that a camera edge detector or cloth grasp has passed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


S1_RESULT_KIND = "towel_isaac_s1_vertex_patch_place_release_result"
S1_ACCEPTED_STATUS_PREFIX = "S1_ISAACLAB_NOMINAL_HALF_FOLD_ACCEPTED_"
S1_SUMMARY_STATUS = "R2_S1_FIRST_FOLD_ACCEPTED_WITHIN_55_45"
SECOND_FOLD_ACTIVE_ARM = "left"
SECOND_FOLD_DIRECTION = "left_to_right"
SECOND_FOLD_CORRECTION_ARM = "right"
# Correction grasps an already folded two-layer free edge.  Reusing the
# 15 mm one-layer inset targets only the inner S1 layer once the edge fans
# slightly.  Half the 6 mm contact-face width keeps the pad on the boundary
# while avoiding a numerically exact silhouette tangent.
SECOND_FOLD_EDGE_INSET_M = 0.003
# The raw S2 checkpoint proves that its two upper layers separate vertically
# near the outside corners, while an inward pinch makes the long moving jaw
# cross the stationary lower bundle.  For an overfold, keep both jaws open and
# keep the long moving jaw outside and use the compact 2.2 mm fixed rubber pad
# as a pusher from outside the exposed overhang.
SECOND_FOLD_OVERFOLD_FIXED_PAD_X_INSET_M = 0.030
SECOND_FOLD_OVERFOLD_FIXED_PAD_OUTSIDE_OFFSET_M = 0.004
SECOND_FOLD_OVERFOLD_MAXIMUM_PUSH_STEP_M = 0.003
# Do not push to a numerically exact zero residual.  Stopping with a small
# visible overhang avoids crossing the stationary edge and is comfortably
# inside the camera acceptance tolerance.
SECOND_FOLD_OVERFOLD_PUSH_STOP_MARGIN_M = 0.003
SECOND_FOLD_TARGET_TOLERANCE_M = 0.008
SECOND_FOLD_MAXIMUM_CORRECTION_M = 0.030
SECOND_FOLD_MAXIMUM_RECOVERABLE_RESIDUAL_M = 0.080
SECOND_FOLD_MINIMUM_EDGE_CONFIDENCE = 0.80
MINIMUM_FIRST_FOLD_SHORT_SIDE_M = 0.12
MAXIMUM_FIRST_FOLD_SHORT_SIDE_M = 0.18
MINIMUM_FIRST_FOLD_LONG_SIDE_M = 0.27
MAXIMUM_FIRST_FOLD_LONG_SIDE_M = 0.33


class SecondFoldPlanningError(RuntimeError):
    """The second-fold observation or correction cannot be accepted."""


@dataclass(frozen=True, slots=True)
class AcceptedFirstFoldFootprint:
    bounds_xyxy_m: tuple[float, float, float, float]
    source_result_sha256: str
    source_result_path: str
    node_count: int

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class SecondFoldEdgeObservation:
    """Top-view metric observation after the left arm has released.

    ``stationary_right_edge_y_m`` is the remembered/observed right boundary
    of the stationary lower bundle. ``moving_free_edge_y_m`` is the visible
    edge of the upper bundle that was moved from left to right.
    """

    moving_free_edge_y_m: float
    stationary_right_edge_y_m: float
    moving_edge_x_span_m: tuple[float, float]
    confidence: float
    settled: bool
    clear_pose_verified: bool
    moving_edge_center_x_m: float | None = None


@dataclass(frozen=True, slots=True)
class SecondFoldCorrectionPlan:
    required: bool
    status: str
    arm: str
    signed_edge_residual_m: float
    correction_delta_y_m: float
    correction_direction: str | None
    grasp_xy_m: tuple[float, float] | None
    target_xy_m: tuple[float, float] | None
    contact_mode: str | None
    requires_top_bundle_layer_separation: bool
    step_limited: bool
    expected_remaining_residual_m: float
    reobserve_after_step: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class OpposingJawTwoLayerContact:
    """Topology-aware contact state for a two-layer edge pinch.

    A valid two-layer pinch does not require one cloth particle to touch both
    jaws.  Instead, each opposing jaw may touch the outward face of a
    different S1 topology half.  Any jaw contact on the stationary lower S2
    bundle still fails the gate.
    """

    fixed_upper_particles: tuple[int, ...]
    moving_upper_particles: tuple[int, ...]
    lower_bundle_particles: tuple[int, ...]
    fixed_contacts_first_half: bool
    fixed_contacts_second_half: bool
    moving_contacts_first_half: bool
    moving_contacts_second_half: bool
    opposing_assignment: str | None
    clean_two_layer_pinch: bool

    @property
    def upper_bundle_particles(self) -> tuple[int, ...]:
        return tuple(
            sorted(set(self.fixed_upper_particles) | set(self.moving_upper_particles))
        )

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def classify_opposing_jaw_two_layer_contact(
    fixed_jaw_particles: Iterable[int],
    moving_jaw_particles: Iterable[int],
    *,
    grid_side: int,
    upper_row_minimum: int,
) -> OpposingJawTwoLayerContact:
    """Classify a clean opposing-face pinch of the folded two-layer edge."""
    if grid_side < 2 or not 0 < upper_row_minimum < grid_side:
        raise SecondFoldPlanningError("two-layer contact topology is invalid")

    particle_count = grid_side * grid_side

    def normalized(values: Iterable[int], name: str) -> tuple[int, ...]:
        result = tuple(sorted({int(value) for value in values}))
        if any(index < 0 or index >= particle_count for index in result):
            raise SecondFoldPlanningError(f"{name} contains an invalid particle index")
        return result

    fixed = normalized(fixed_jaw_particles, "fixed jaw contacts")
    moving = normalized(moving_jaw_particles, "moving jaw contacts")
    fixed_upper = tuple(
        index for index in fixed if index // grid_side >= upper_row_minimum
    )
    moving_upper = tuple(
        index for index in moving if index // grid_side >= upper_row_minimum
    )
    lower = tuple(
        sorted(
            {
                index
                for index in (*fixed, *moving)
                if index // grid_side < upper_row_minimum
            }
        )
    )
    half = grid_side // 2
    fixed_first = any(index % grid_side < half for index in fixed_upper)
    fixed_second = any(index % grid_side >= half for index in fixed_upper)
    moving_first = any(index % grid_side < half for index in moving_upper)
    moving_second = any(index % grid_side >= half for index in moving_upper)
    first_assignment = fixed_first and moving_second
    reverse_assignment = fixed_second and moving_first
    assignment = None
    if first_assignment and reverse_assignment:
        assignment = "both_opposing_assignments"
    elif first_assignment:
        assignment = "fixed_first_moving_second"
    elif reverse_assignment:
        assignment = "fixed_second_moving_first"

    return OpposingJawTwoLayerContact(
        fixed_upper_particles=fixed_upper,
        moving_upper_particles=moving_upper,
        lower_bundle_particles=lower,
        fixed_contacts_first_half=fixed_first,
        fixed_contacts_second_half=fixed_second,
        moving_contacts_first_half=moving_first,
        moving_contacts_second_half=moving_second,
        opposing_assignment=assignment,
        clean_two_layer_pinch=not lower and assignment is not None,
    )


def _finite(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise SecondFoldPlanningError(f"{name} must be finite")
    return result


def _file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def load_accepted_first_fold_footprint(
    result_path: Path, summary_path: Path
) -> AcceptedFirstFoldFootprint:
    """Load an S1 terminal shape only when its hash is in the accepted summary."""
    if not result_path.is_file() or not summary_path.is_file():
        raise SecondFoldPlanningError("accepted S1 result and summary are required")
    result = json.loads(result_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(result, dict) or not isinstance(summary, dict):
        raise SecondFoldPlanningError("S1 result and summary roots must be objects")
    if (
        summary.get("status") != S1_SUMMARY_STATUS
        or int(summary.get("completed_runs", 0)) < 3
    ):
        raise SecondFoldPlanningError("S1 summary is not the accepted three-run gate")
    result_digest = _file_sha256(result_path)
    accepted_digests = {
        str(value)
        for key, value in summary.get("sources", {}).items()
        if key.endswith("result_sha256")
    }
    if result_digest not in accepted_digests:
        raise SecondFoldPlanningError("S1 result hash is not listed in the accepted summary")
    if (
        result.get("record_kind") != S1_RESULT_KIND
        or not str(result.get("status", "")).startswith(S1_ACCEPTED_STATUS_PREFIX)
        or result.get("motion_authorized") is not False
    ):
        raise SecondFoldPlanningError("source is not an accepted simulation-only S1 result")
    nodes = result.get("final_cloth_shape_local_m_env_0")
    if not isinstance(nodes, list) or len(nodes) < 4:
        raise SecondFoldPlanningError("S1 result does not contain a terminal cloth shape")
    parsed: list[tuple[float, float, float]] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, list) or len(node) != 3:
            raise SecondFoldPlanningError(f"S1 node {index} is not XYZ")
        parsed.append(
            tuple(_finite(value, f"S1 node {index}") for value in node)  # type: ignore[arg-type]
        )
    xs = [node[0] for node in parsed]
    ys = [node[1] for node in parsed]
    bounds = (min(xs), max(xs), min(ys), max(ys))
    short_side = bounds[1] - bounds[0]
    long_side = bounds[3] - bounds[2]
    if not MINIMUM_FIRST_FOLD_SHORT_SIDE_M <= short_side <= MAXIMUM_FIRST_FOLD_SHORT_SIDE_M:
        raise SecondFoldPlanningError("accepted S1 short side is outside 120..180 mm")
    if not MINIMUM_FIRST_FOLD_LONG_SIDE_M <= long_side <= MAXIMUM_FIRST_FOLD_LONG_SIDE_M:
        raise SecondFoldPlanningError("accepted S1 long side is outside 270..330 mm")
    return AcceptedFirstFoldFootprint(
        bounds_xyxy_m=bounds,
        source_result_sha256=result_digest,
        source_result_path=str(result_path.resolve()),
        node_count=len(parsed),
    )


def plan_right_arm_second_fold_correction(
    observation: SecondFoldEdgeObservation,
    *,
    tolerance_m: float = SECOND_FOLD_TARGET_TOLERANCE_M,
    maximum_correction_m: float = SECOND_FOLD_MAXIMUM_CORRECTION_M,
) -> SecondFoldCorrectionPlan:
    """Plan one bounded right-arm free-edge correction after clear observation.

    A positive residual means the moved upper edge stopped left of the lower
    right edge (underfold) and must move farther right.  A negative residual
    means the upper bundle overhangs and must return left.
    """
    moving_y = _finite(observation.moving_free_edge_y_m, "moving free edge y")
    stationary_y = _finite(
        observation.stationary_right_edge_y_m, "stationary right edge y"
    )
    x0, x1 = (
        _finite(value, "moving edge x span")
        for value in observation.moving_edge_x_span_m
    )
    observed_center_x = (
        0.5 * (x0 + x1)
        if observation.moving_edge_center_x_m is None
        else _finite(observation.moving_edge_center_x_m, "moving edge center x")
    )
    confidence = _finite(observation.confidence, "edge confidence")
    tolerance = _finite(tolerance_m, "correction tolerance")
    maximum = _finite(maximum_correction_m, "maximum correction")
    if x0 >= x1:
        raise SecondFoldPlanningError("moving edge x span must have positive width")
    if not x0 <= observed_center_x <= x1:
        raise SecondFoldPlanningError("moving edge center x must lie inside its span")
    if x1 - x0 <= 2.0 * SECOND_FOLD_EDGE_INSET_M:
        raise SecondFoldPlanningError("moving edge x span is too short for a safe patch")
    if not 0.0 <= confidence <= 1.0:
        raise SecondFoldPlanningError("edge confidence must be in [0, 1]")
    if confidence < SECOND_FOLD_MINIMUM_EDGE_CONFIDENCE:
        raise SecondFoldPlanningError("moving free-edge confidence is below 0.80")
    if not observation.settled or not observation.clear_pose_verified:
        raise SecondFoldPlanningError("settled cloth and clear arms are required")
    if tolerance <= 0.0 or maximum < tolerance:
        raise SecondFoldPlanningError("correction limits are inconsistent")

    residual = moving_y - stationary_y
    if abs(residual) <= tolerance:
        return SecondFoldCorrectionPlan(
            required=False,
            status="SECOND_FOLD_WITHIN_TOLERANCE",
            arm=SECOND_FOLD_CORRECTION_ARM,
            signed_edge_residual_m=residual,
            correction_delta_y_m=0.0,
            correction_direction=None,
            grasp_xy_m=None,
            target_xy_m=None,
            contact_mode=None,
            requires_top_bundle_layer_separation=False,
            step_limited=False,
            expected_remaining_residual_m=residual,
            reobserve_after_step=False,
        )
    if abs(residual) > SECOND_FOLD_MAXIMUM_RECOVERABLE_RESIDUAL_M:
        raise SecondFoldPlanningError(
            "second-fold edge residual exceeds the recoverable 80 mm envelope"
        )

    underfold = residual > 0.0
    if underfold:
        # An underfold must be pulled toward -Y, so retain the opposing-jaw
        # two-layer pinch at the camera-observed edge centre.
        delta_y = max(-maximum, -residual)
        grasp_y = moving_y + SECOND_FOLD_EDGE_INSET_M
        contact_mode = "top_bundle_edge_side_pinch"
        requires_layer_separation = True
    else:
        # An overfold exposes its upper edge beyond the stationary bundle.
        # Approach from -Y with an open jaw and use only a bounded 3 mm step.
        # The camera must reobserve after every step: cloth travel is not
        # assumed to equal TCP travel.  The offset places the compact fixed
        # pad just outside the edge while the long moving jaw stays on the
        # low-X exterior side of the towel.
        desired_push = max(0.0, -residual - SECOND_FOLD_OVERFOLD_PUSH_STOP_MARGIN_M)
        delta_y = min(
            maximum,
            SECOND_FOLD_OVERFOLD_MAXIMUM_PUSH_STEP_M,
            desired_push,
        )
        grasp_y = moving_y - SECOND_FOLD_OVERFOLD_FIXED_PAD_OUTSIDE_OFFSET_M
        contact_mode = "exposed_overhang_open_jaw_fixed_pad_edge_push"
        requires_layer_separation = False
    expected_remaining_residual = residual + delta_y
    grasp_x = (
        observed_center_x
        if underfold
        else x0 + SECOND_FOLD_OVERFOLD_FIXED_PAD_X_INSET_M
    )
    return SecondFoldCorrectionPlan(
        required=True,
        status=(
            "SECOND_FOLD_UNDERFOLD_CORRECTION_REQUIRED"
            if underfold
            else "SECOND_FOLD_OVERFOLD_CORRECTION_REQUIRED"
        ),
        arm=SECOND_FOLD_CORRECTION_ARM,
        signed_edge_residual_m=residual,
        correction_delta_y_m=delta_y,
        correction_direction="toward_right" if delta_y < 0.0 else "toward_left",
        grasp_xy_m=(grasp_x, grasp_y),
        target_xy_m=(grasp_x, grasp_y + delta_y),
        contact_mode=contact_mode,
        requires_top_bundle_layer_separation=requires_layer_separation,
        step_limited=(
            desired_push > delta_y
            if not underfold
            else residual > maximum
        ),
        expected_remaining_residual_m=expected_remaining_residual,
        reobserve_after_step=True,
    )
