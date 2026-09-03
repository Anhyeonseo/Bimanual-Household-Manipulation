"""Plan a gravity-canonicalized first fold for the dual SO-101 arms.

The previous first-fold arc tried to turn a flat panel over while the cloth
was already in table/self contact.  That can trap an internal Z fold.  This
candidate changes the cloth topology before the precision laydown:

1. pinch the robot-near edge at both endpoints;
2. lift vertically until the complete free edge clears the table;
3. accelerate the suspended panel away from the robot, then keep moving it
   forward while lowering so inertia trails and straightens the free edge;
4. continue moving the held edge away from the robot, depositing a
   deliberately larger supported lower layer with a no-tension allowance;
5. reverse the arm direction and lay the shorter held layer over it without
   recovering that allowance.

Mid-fold top-view imagery cannot reliably separate the vertical panel from the
table-contacting panel.  The route therefore uses the known grasp-to-free-edge
length and a small open-loop allowance instead of a hidden-boundary estimate.

Only task-space poses are produced here.  Cloth contact, free-edge vision,
MoveIt collision checking, and execution authorization remain separate gates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from tools.lib.towel_bimanual_then_single_planning import (
    FIRST_EDGE_ENDPOINT_INSET_M,
    build_bimanual_first_fold,
)
from tools.lib.towel_task_pose_planning import (
    EDGE_GRASP_INSET_M,
    FIRST_LAYER_TCP_Z_OFFSET_M,
    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
    NOMINAL_TOWEL_SIDE_M,
    PhaseSpec,
    PREGRASP_CLEARANCE_M,
    TowelPlanningError,
    finite_vector,
    task_pose,
)


# The TCP starts 15 mm above the tabletop at cloth contact.  A grasp centered
# 15 mm inside a 300 mm edge leaves 285 mm of cloth below it.  Twenty
# millimetres of additional clearance keeps the outer edge off the table.
HANGING_LENGTH_FROM_GRASP_M = NOMINAL_TOWEL_SIDE_M - EDGE_GRASP_INSET_M
FREE_EDGE_CLEARANCE_M = 0.020
# The nominal 285 mm cloth length is not its vertical projection in the
# coupled-VBD simulation.  With the measured material and the actual jaw
# contact, the final 12 mm feed left the free-edge median 48 mm above the
# table when the TCP was at the nominal geometric touchdown.  Offline
# privileged calibration puts the end of that feed at table contact when the
# preceding touchdown TCP is 234 mm above the flat-cloth contact TCP.  This is
# a fixed trajectory calibration, not a mid-action camera measurement.
CALIBRATED_TOUCHDOWN_TCP_HEIGHT_M = 0.234
# With the selected measured-bend Newton material, the free-edge median
# settles 44 mm farther from the robot than
# the held TCP after the fixed touchdown feed.  Mid-action top view is
# occluded, so this repeatable simulator offset is part of the open-loop
# placement calibration.
CALIBRATED_POST_FEED_FREE_EDGE_X_OFFSET_M = 0.044
POST_RELEASE_CLEARANCE_Z_OFFSET_M = 0.205
POST_RELEASE_CLEARANCE_SAMPLE_COUNT = 4
POST_RELEASE_OUTBOARD_FRACTION = 0.75
SUSPEND_LIFT_SAMPLE_COUNT = 7
# A stopped vertical descent left the free panel directly under the jaws and
# repeatedly trapped its bottom in a Z fold.  Start a modest high sweep, then
# continue the same motion through several descent samples.  These samples are
# executed without per-waypoint settling by the Isaac replay runner.
SUSPENDED_FORWARD_PREBIAS_M = 0.005
SUSPENDED_FORWARD_SWEEP_SAMPLE_COUNT = 3
SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT = 4
# Once the free edge first touches the table, keep the held edge at the same
# height and pull forward.  This deliberately makes the loose edge slide on
# the measured-friction tabletop and straightens the hanging panel before any
# slack is deposited.  Lowering during this segment created the visible Z.
TOUCHDOWN_FORWARD_CONTINUATION_M = 0.036
FORM_L_SAMPLE_COUNT = 9
FORM_L_EARLY_ADVANCE_SAMPLE_COUNT = 5
FORM_L_LOW_SWEEP_SAMPLE_COUNT = 3
TOUCHDOWN_SLACK_FEED_SAMPLE_COUNT = 4
LAYDOWN_SAMPLE_COUNT = 9
# Put exactly 50% of the cloth on the table before direction reversal.  Fold
# quality is accepted within a 55/45 envelope after the unobstructed
# post-release observation; 55/45 is not the commanded geometry.  The former
# 60/40 layout required a second top-down grasp in the overlap, where physics
# showed that the jaw either missed or pinched both cloth layers.
SUPPORTED_LOWER_LAYER_FRACTION = 0.50
# The constant-height surface drag moves the free edge about 30 mm before it
# anchors.  The previous geometric 50/50 reversal consequently left roughly
# 165 mm in the returned upper layer and 135 mm below.  Depositing half of
# that measured 30 mm transfer before reversal produced the best validated
# 145/155 mm result.  A subsequent 11.3 mm interpolation trial did not improve
# the ratio and increased paired overlap error, so 15 mm remains the baseline.
SURFACE_DRAG_HALF_FOLD_COMPENSATION_M = 0.015
# Keep a small no-tension allowance while the target 50% lower layer is fed.
FORWARD_LAY_SLACK_M = 0.012
# Advance from the held-edge grasp to the calibrated free-edge landing plus
# the supported lower-layer span.  Omitting the 44 mm landing offset made the
# nominal 180 mm lower layer only about 106 mm long and forced the surplus
# cloth into a second (Z) fold.
FORWARD_LAY_ADVANCE_M = (
    CALIBRATED_POST_FEED_FREE_EDGE_X_OFFSET_M
    + SUPPORTED_LOWER_LAYER_FRACTION * NOMINAL_TOWEL_SIDE_M
    + SURFACE_DRAG_HALF_FOLD_COMPENSATION_M
    - FORWARD_LAY_SLACK_M
)
OVER_CENTER_SAMPLE_COUNT = 3
# Stop the held layer deliberately short so a small lower-layer strip remains
# visible from the top.  This makes the residual direction observable without
# making a second overlap grasp part of the nominal first-fold path.  The
# historical phase name is retained for replay compatibility.
CONTROLLED_UNDERFOLD_MARGIN_M = 0.002
MINIMUM_EXPOSED_LOWER_STRIP_M = 0.001
MINIMUM_UPPER_EDGE_SUPPORT_DEPTH_M = 0.040
# At a near-50/50 laydown the two free edges overlap.  Separating the held
# upper edge by one two-layer contact thickness before opening prevents the
# moving-jaw proxy from driving into the lower layer.  Keep the released arm at
# this height while withdrawing sideways; descending an open jaw would recreate
# the same collision.
PREOPEN_EDGE_SEPARATION_M = 0.006
PREOPEN_EDGE_SEPARATION_SAMPLE_COUNT = 2
RELEASE_SIDE_WITHDRAWAL_M = 0.040
RELEASE_SIDE_WITHDRAWAL_SAMPLE_COUNT = 4


@dataclass(frozen=True, slots=True)
class SuspendedGravityFoldSpec:
    phases: tuple[PhaseSpec, ...]
    initial_grasp_x_m: float
    free_edge_anchor_x_m: float
    fold_line_x_m: float
    final_held_grasp_x_m: float
    final_held_edge_x_m: float
    exposed_lower_strip_m: float
    upper_edge_support_depth_m: float
    suspend_tcp_z_m: float
    free_edge_touchdown_tcp_z_m: float
    touchdown_slack_feed_m: float
    l_shape_tcp_z_m: float
    supported_lower_layer_fraction: float
    forward_lay_slack_m: float
    laydown_tcp_z_m: float
    expected_footprint_xyxy_m: tuple[float, float, float, float]
    requires_free_edge_observation: bool = False
    requires_post_release_correction: bool = False


def _bimanual_phase(
    name: str,
    *,
    x_m: float,
    y_by_arm_m: dict[str, float],
    z_m: float,
    semantic: str,
    attachment_event: str | None = None,
) -> PhaseSpec:
    return PhaseSpec(
        name,
        tuple(
            task_pose(
                f"{name}_{arm}",
                arm,
                (x_m, y_by_arm_m[arm], z_m),
                0.0,
                semantic,
                "one_layer",
                MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
            )
            for arm in ("left", "right")
        ),
        attachment_event=attachment_event,
    )


def build_suspended_gravity_first_fold(
    bounds: Sequence[float],
    table_z_m: float,
    *,
    contact_tcp_z_offset_m: float = FIRST_LAYER_TCP_Z_OFFSET_M,
    laydown_tcp_z_offset_m: float | None = None,
) -> SuspendedGravityFoldSpec:
    """Build the near-edge lift, L formation, and return laydown candidate."""
    left, right, bottom, top = finite_vector(bounds, 4, "towel bounds")
    if not left < right or not bottom < top:
        raise TowelPlanningError("towel bounds must have positive area")
    if not math.isclose(right - left, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise TowelPlanningError("suspended fold requires a nominal 300 mm X span")
    if not math.isclose(top - bottom, NOMINAL_TOWEL_SIDE_M, abs_tol=1.0e-9):
        raise TowelPlanningError("suspended fold requires a nominal 300 mm Y span")
    if not math.isfinite(table_z_m):
        raise TowelPlanningError("table z must be finite")
    if (
        not math.isfinite(contact_tcp_z_offset_m)
        or contact_tcp_z_offset_m <= 0.0
        or contact_tcp_z_offset_m > FIRST_LAYER_TCP_Z_OFFSET_M
    ):
        raise TowelPlanningError(
            "gravity contact TCP offset must be in (0, 0.015] m"
        )
    if laydown_tcp_z_offset_m is None:
        laydown_tcp_z_offset_m = contact_tcp_z_offset_m
    if (
        not math.isfinite(laydown_tcp_z_offset_m)
        or laydown_tcp_z_offset_m < contact_tcp_z_offset_m
        or laydown_tcp_z_offset_m > FIRST_LAYER_TCP_Z_OFFSET_M
    ):
        raise TowelPlanningError(
            "gravity laydown TCP offset must be between contact offset and "
            "0.015 m"
        )

    contact_z = table_z_m + contact_tcp_z_offset_m
    laydown_z = table_z_m + laydown_tcp_z_offset_m
    grasp_x = left + EDGE_GRASP_INSET_M
    anchor_x = grasp_x + CALIBRATED_POST_FEED_FREE_EDGE_X_OFFSET_M
    # Preserve the calibrated touchdown position.  Moving this endpoint 30 mm
    # forward shortened the supported lower panel and compressed the surplus
    # into the fold.  Forward speed continues through the touchdown feed rather
    # than shifting the entire lower-layer geometry.
    touchdown_x = anchor_x
    supported_lower_length = (
        SUPPORTED_LOWER_LAYER_FRACTION * NOMINAL_TOWEL_SIDE_M
        + SURFACE_DRAG_HALF_FOLD_COMPENSATION_M
    )
    fold_x = grasp_x + FORWARD_LAY_ADVANCE_M
    vertical_panel_length = HANGING_LENGTH_FROM_GRASP_M - supported_lower_length
    if vertical_panel_length <= FORWARD_LAY_SLACK_M:
        raise TowelPlanningError("supported lower layer leaves no held panel")
    return_turn_radius = vertical_panel_length - FORWARD_LAY_SLACK_M
    laydown_clearance = laydown_z - contact_z
    laydown_grasp_x = fold_x - math.sqrt(
        return_turn_radius**2 - laydown_clearance**2
    )
    final_grasp_x = laydown_grasp_x + CONTROLLED_UNDERFOLD_MARGIN_M
    # The physical held edge extends one grasp inset toward the robot.  It must
    # land well inside the supported lower layer, while leaving a visible strip
    # between itself and the anchored free edge for post-release assessment.
    final_held_edge_x = final_grasp_x - EDGE_GRASP_INSET_M
    exposed_lower_strip = final_held_edge_x - anchor_x
    upper_edge_support_depth = fold_x - final_held_edge_x
    if exposed_lower_strip < MINIMUM_EXPOSED_LOWER_STRIP_M:
        raise TowelPlanningError("raw fold does not expose enough lower strip")
    if upper_edge_support_depth < MINIMUM_UPPER_EDGE_SUPPORT_DEPTH_M:
        raise TowelPlanningError("held edge is not placed deeply on lower support")
    suspend_z = contact_z + HANGING_LENGTH_FROM_GRASP_M + FREE_EDGE_CLEARANCE_M
    touchdown_z = contact_z + CALIBRATED_TOUCHDOWN_TCP_HEIGHT_M
    l_shape_z = contact_z + vertical_panel_length - FORWARD_LAY_SLACK_M
    # Do not feed vertical slack immediately after first contact.  Maintaining
    # the touchdown height supplies the tension needed for a short surface
    # drag; the following form-L phases introduce the lower-panel slack while
    # continuing forward.
    slack_feed_z = touchdown_z
    y_by_arm = {
        "left": top - FIRST_EDGE_ENDPOINT_INSET_M,
        "right": bottom + FIRST_EDGE_ENDPOINT_INSET_M,
    }

    # Reuse only the independently tested clear->pregrasp->vertical-contact
    # prefix.  The superseded flat-table fold arc is intentionally discarded.
    old_phases, _ = build_bimanual_first_fold(bounds, table_z_m)
    contact_index = next(
        index for index, phase in enumerate(old_phases) if phase.name == "first_contact"
    )
    phases = list(old_phases[:contact_index])
    phases.append(
        _bimanual_phase(
            "first_contact",
            x_m=grasp_x,
            y_by_arm_m=y_by_arm,
            z_m=contact_z,
            semantic="contact",
            attachment_event=(
                "attach_two_single_layer_edge_patches_after_dual_contact_gate"
            ),
        )
    )

    for index in range(1, SUSPEND_LIFT_SAMPLE_COUNT + 1):
        progress = index / SUSPEND_LIFT_SAMPLE_COUNT
        z_m = contact_z + progress * (suspend_z - contact_z)
        phases.append(
            _bimanual_phase(
                f"first_suspend_lift_{index:02d}",
                x_m=grasp_x,
                y_by_arm_m=y_by_arm,
                z_m=z_m,
                semantic="attached_lift",
            )
        )

    sweep_start_x = grasp_x
    sweep_end_x = grasp_x + SUSPENDED_FORWARD_PREBIAS_M
    for index in range(1, SUSPENDED_FORWARD_SWEEP_SAMPLE_COUNT + 1):
        # Quadratic spacing starts gently and increases forward speed without
        # introducing a lateral or vertical impulse at full suspension.
        progress = (index / SUSPENDED_FORWARD_SWEEP_SAMPLE_COUNT) ** 2
        phases.append(
            _bimanual_phase(
                f"first_suspended_forward_sweep_{index:02d}",
                x_m=sweep_start_x + progress * (sweep_end_x - sweep_start_x),
                y_by_arm_m=y_by_arm,
                z_m=suspend_z,
                semantic="attached_lift",
            )
        )

    for index in range(1, SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT + 1):
        progress = index / SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT
        # Front-load horizontal travel while the free edge is still airborne;
        # keep vertical progress linear so the final contact is gentle.
        horizontal_progress = math.sqrt(progress)
        phases.append(
            _bimanual_phase(
                (
                    "first_free_edge_touchdown"
                    if index == SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT
                    else f"first_suspended_forward_descent_{index:02d}"
                ),
                x_m=sweep_end_x
                + horizontal_progress * (touchdown_x - sweep_end_x),
                y_by_arm_m=y_by_arm,
                z_m=suspend_z + progress * (touchdown_z - suspend_z),
                semantic="attached_laydown",
            )
        )

    # Keep advancing at constant height after the loose edge reaches the table.
    # The resulting low surface drag uses table friction to straighten the
    # bottom edge instead of depositing a vertical strip that curls into a Z.
    feed_end_x = touchdown_x + TOUCHDOWN_FORWARD_CONTINUATION_M
    for index in range(1, TOUCHDOWN_SLACK_FEED_SAMPLE_COUNT + 1):
        progress = index / TOUCHDOWN_SLACK_FEED_SAMPLE_COUNT
        phases.append(
            _bimanual_phase(
                f"first_touchdown_slack_feed_{index:02d}",
                x_m=touchdown_x
                + progress * TOUCHDOWN_FORWARD_CONTINUATION_M,
                y_by_arm_m=y_by_arm,
                z_m=slack_feed_z,
                semantic="attached_laydown",
            )
        )

    # Deposit the larger lower layer with a gradually introduced allowance.  This is
    # based on measured towel length and does not require a top-view estimate
    # of the hidden table-contact boundary.
    early_advance_x = feed_end_x + (5.0 / 7.0) * (fold_x - feed_end_x)
    early_advance_z = slack_feed_z + (5.0 / 7.0) * (
        l_shape_z - slack_feed_z
    )
    for index in range(1, FORM_L_EARLY_ADVANCE_SAMPLE_COUNT + 1):
        progress = index / 7.0
        phases.append(
            _bimanual_phase(
                f"first_form_l_{index:02d}",
                x_m=feed_end_x + progress * (fold_x - feed_end_x),
                y_by_arm_m=y_by_arm,
                z_m=slack_feed_z + progress * (l_shape_z - slack_feed_z),
                semantic="attached_laydown",
            )
        )
    # The measured slip gain rose from below 7% to 51--66% in the final two
    # forward samples.  Pause horizontal motion at that transition, lower to
    # the final lay height to add contact, then finish with a low sweep.
    phases.append(
        _bimanual_phase(
            "first_form_l_06",
            x_m=early_advance_x,
            y_by_arm_m=y_by_arm,
            z_m=l_shape_z,
            semantic="attached_laydown",
        )
    )
    for sweep_index in range(1, FORM_L_LOW_SWEEP_SAMPLE_COUNT + 1):
        progress = sweep_index / FORM_L_LOW_SWEEP_SAMPLE_COUNT
        phases.append(
            _bimanual_phase(
                f"first_form_l_{sweep_index + 6:02d}",
                x_m=early_advance_x
                + progress * (fold_x - early_advance_x),
                y_by_arm_m=y_by_arm,
                z_m=l_shape_z,
                semantic="attached_laydown",
            )
        )

    # Reverse direction after the lower half has been deposited.  The cloth
    # panel, not the wrist, performs the quarter turn.  The terminal
    # angle may stop just short of 90 degrees so the Isaac jaw remains above
    # the table despite the measured right-arm frame-height discrepancy.  The
    # contact pose itself is intentionally unchanged so the pinch is not made
    # less reliable.
    laydown_terminal_angle = math.acos(laydown_clearance / return_turn_radius)
    for index in range(1, LAYDOWN_SAMPLE_COUNT + 1):
        progress = index / LAYDOWN_SAMPLE_COUNT
        angle = laydown_terminal_angle * progress
        # Keep the allowance through release.  Recovering it made the upper
        # surface taut and lifted the middle into a 49 mm arch.
        turn_radius = return_turn_radius
        x_m = fold_x - turn_radius * math.sin(angle)
        z_m = contact_z + turn_radius * math.cos(angle)
        phases.append(
            _bimanual_phase(
                f"first_gravity_laydown_{index:02d}",
                x_m=x_m,
                y_by_arm_m=y_by_arm,
                z_m=z_m,
                semantic="attached_laydown",
            )
        )

    # Do not demand perfect corner alignment from the raw fold.  Stop the held
    # inset 2 mm short while staying at contact height, leaving the lower layer
    # visible and making the residual direction observable after release.  The
    # first_gravity_overcenter_* identifier remains only for compatibility with
    # the existing replay consumers.
    for index in range(1, OVER_CENTER_SAMPLE_COUNT + 1):
        progress = index / OVER_CENTER_SAMPLE_COUNT
        phases.append(
            _bimanual_phase(
                f"first_gravity_overcenter_{index:02d}",
                x_m=laydown_grasp_x
                + progress * (final_grasp_x - laydown_grasp_x),
                y_by_arm_m=y_by_arm,
                z_m=laydown_z,
                semantic="attached_laydown",
            )
        )

    release_z = laydown_z + PREOPEN_EDGE_SEPARATION_M
    for index in range(1, PREOPEN_EDGE_SEPARATION_SAMPLE_COUNT + 1):
        progress = index / PREOPEN_EDGE_SEPARATION_SAMPLE_COUNT
        phases.append(
            _bimanual_phase(
                f"first_gravity_preopen_clearance_{index:02d}",
                x_m=final_grasp_x,
                y_by_arm_m=y_by_arm,
                z_m=laydown_z + progress * PREOPEN_EDGE_SEPARATION_M,
                semantic="attached_release_clearance",
                attachment_event=(
                    "release_both_edge_patches_after_gravity_laydown_gate"
                    if index == PREOPEN_EDGE_SEPARATION_SAMPLE_COUNT
                    else None
                ),
            )
        )

    retreat_z = contact_z + PREGRASP_CLEARANCE_M
    clearance_z = table_z_m + POST_RELEASE_CLEARANCE_Z_OFFSET_M
    # Once the jaws are open, leave the towel footprint by moving straight up.
    # The former sideways sweep crossed the newly folded edge and visibly
    # disturbed it despite a successful release.
    phases.append(
        _bimanual_phase(
            "first_gravity_retreat",
            x_m=final_grasp_x,
            y_by_arm_m=y_by_arm,
            z_m=retreat_z,
            semantic="released_retreat",
        )
    )
    for index in range(1, POST_RELEASE_CLEARANCE_SAMPLE_COUNT + 1):
        progress = index / POST_RELEASE_CLEARANCE_SAMPLE_COUNT
        phases.append(
            _bimanual_phase(
                f"first_gravity_clearance_lift_{index:02d}",
                x_m=final_grasp_x,
                y_by_arm_m=y_by_arm,
                z_m=retreat_z + progress * (clearance_z - retreat_z),
                semantic="released_retreat",
            )
        )
    for index in range(1, POST_RELEASE_CLEARANCE_SAMPLE_COUNT + 1):
        progress = index / POST_RELEASE_CLEARANCE_SAMPLE_COUNT
        outboard_target_x = final_grasp_x + POST_RELEASE_OUTBOARD_FRACTION * (
            fold_x - final_grasp_x
        )
        phases.append(
            _bimanual_phase(
                f"first_gravity_clearance_outboard_{index:02d}",
                x_m=final_grasp_x
                + progress * (outboard_target_x - final_grasp_x),
                y_by_arm_m=y_by_arm,
                z_m=clearance_z,
                semantic="released_retreat",
            )
        )
    # Use a topology-specific name so the legacy flat-fold staged return
    # (which rotates wrist-roll before wrist-flex) is not inherited by this
    # different retreat posture.
    phases.append(
        PhaseSpec("first_gravity_reobserve_clear", (), clear_pose=True)
    )

    return SuspendedGravityFoldSpec(
        phases=tuple(phases),
        initial_grasp_x_m=grasp_x,
        free_edge_anchor_x_m=anchor_x,
        fold_line_x_m=fold_x,
        final_held_grasp_x_m=final_grasp_x,
        final_held_edge_x_m=final_held_edge_x,
        exposed_lower_strip_m=exposed_lower_strip,
        upper_edge_support_depth_m=upper_edge_support_depth,
        suspend_tcp_z_m=suspend_z,
        free_edge_touchdown_tcp_z_m=touchdown_z,
        touchdown_slack_feed_m=0.0,
        l_shape_tcp_z_m=l_shape_z,
        supported_lower_layer_fraction=SUPPORTED_LOWER_LAYER_FRACTION,
        forward_lay_slack_m=FORWARD_LAY_SLACK_M,
        laydown_tcp_z_m=laydown_z,
        expected_footprint_xyxy_m=(
            anchor_x,
            fold_x,
            bottom,
            top,
        ),
    )
