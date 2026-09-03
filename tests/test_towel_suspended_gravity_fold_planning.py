from __future__ import annotations

import math

import pytest

from tools.lib.towel_suspended_gravity_fold_planning import (
    CALIBRATED_POST_FEED_FREE_EDGE_X_OFFSET_M,
    CALIBRATED_TOUCHDOWN_TCP_HEIGHT_M,
    FORM_L_SAMPLE_COUNT,
    FORWARD_LAY_ADVANCE_M,
    FORWARD_LAY_SLACK_M,
    HANGING_LENGTH_FROM_GRASP_M,
    LAYDOWN_SAMPLE_COUNT,
    CONTROLLED_UNDERFOLD_MARGIN_M,
    OVER_CENTER_SAMPLE_COUNT,
    RELEASE_SIDE_WITHDRAWAL_M,
    SUPPORTED_LOWER_LAYER_FRACTION,
    SURFACE_DRAG_HALF_FOLD_COMPENSATION_M,
    SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT,
    SUSPENDED_FORWARD_PREBIAS_M,
    SUSPENDED_FORWARD_SWEEP_SAMPLE_COUNT,
    SUSPEND_LIFT_SAMPLE_COUNT,
    TOUCHDOWN_SLACK_FEED_SAMPLE_COUNT,
    build_suspended_gravity_first_fold,
)
from tools.lib.towel_task_pose_planning import (
    TowelPlanningError,
    validate_phase_contract,
)


def _phase(spec, name):
    return next(phase for phase in spec.phases if phase.name == name)


def test_suspended_fold_lifts_complete_free_edge_before_touchdown() -> None:
    table_z = -0.005
    spec = build_suspended_gravity_first_fold((0.0, 0.3, -0.3, 0.0), table_z)
    validate_phase_contract(spec.phases)

    contact_z = table_z + 0.015
    assert HANGING_LENGTH_FROM_GRASP_M == pytest.approx(0.285)
    assert spec.suspend_tcp_z_m == pytest.approx(contact_z + 0.305)
    assert CALIBRATED_TOUCHDOWN_TCP_HEIGHT_M == pytest.approx(0.234)
    assert spec.free_edge_touchdown_tcp_z_m == pytest.approx(contact_z + 0.234)
    assert spec.suspend_tcp_z_m - HANGING_LENGTH_FROM_GRASP_M > table_z

    lift = _phase(spec, f"first_suspend_lift_{SUSPEND_LIFT_SAMPLE_COUNT:02d}")
    touchdown = _phase(spec, "first_free_edge_touchdown")
    assert [target.xyz_m[0] for target in lift.targets] == pytest.approx(
        [0.015, 0.015]
    )
    assert [target.xyz_m[0] for target in touchdown.targets] == pytest.approx(
        [0.059, 0.059]
    )
    sweep_x = [
        _phase(spec, f"first_suspended_forward_sweep_{index:02d}")
        .targets[0]
        .xyz_m[0]
        for index in range(1, SUSPENDED_FORWARD_SWEEP_SAMPLE_COUNT + 1)
    ]
    assert sweep_x == sorted(sweep_x)
    assert sweep_x[-1] == pytest.approx(0.015 + SUSPENDED_FORWARD_PREBIAS_M)
    descent = [
        _phase(spec, f"first_suspended_forward_descent_{index:02d}").targets[0]
        for index in range(1, SUSPENDED_FORWARD_DESCENT_SAMPLE_COUNT)
    ] + [touchdown.targets[0]]
    assert all(
        later.xyz_m[0] > earlier.xyz_m[0]
        and later.xyz_m[2] < earlier.xyz_m[2]
        for earlier, later in zip(descent, descent[1:])
    )


def test_forward_lay_drags_at_constant_height_before_depositing_slack() -> None:
    spec = build_suspended_gravity_first_fold((0.0, 0.3, -0.3, 0.0), -0.005)
    contact_z = 0.010
    phase_names = [phase.name for phase in spec.phases]
    assert phase_names.index("first_free_edge_touchdown") < phase_names.index(
        "first_touchdown_slack_feed_01"
    ) < phase_names.index("first_form_l_01")
    feed = _phase(
        spec,
        f"first_touchdown_slack_feed_{TOUCHDOWN_SLACK_FEED_SAMPLE_COUNT:02d}",
    ).targets[0]
    touchdown = _phase(spec, "first_free_edge_touchdown").targets[0]
    assert feed.xyz_m == pytest.approx((0.095, -0.015, 0.244))
    assert feed.xyz_m[2] == pytest.approx(touchdown.xyz_m[2])
    assert spec.touchdown_slack_feed_m == pytest.approx(0.0)

    for index in range(1, 6):
        target = _phase(spec, f"first_form_l_{index:02d}").targets[0]
        progress = index / 7.0
        assert target.xyz_m[0] == pytest.approx(
            0.095 + progress * (0.212 - 0.095)
        )
        assert target.xyz_m[2] == pytest.approx(
            feed.xyz_m[2] + progress * (spec.l_shape_tcp_z_m - feed.xyz_m[2])
        )

    mid_feed = _phase(spec, "first_form_l_06").targets[0]
    assert mid_feed.xyz_m[0] == pytest.approx(
        _phase(spec, "first_form_l_05").targets[0].xyz_m[0]
    )
    assert mid_feed.xyz_m[2] == pytest.approx(spec.l_shape_tcp_z_m)
    for index in range(7, FORM_L_SAMPLE_COUNT + 1):
        assert _phase(spec, f"first_form_l_{index:02d}").targets[0].xyz_m[2] == (
            pytest.approx(spec.l_shape_tcp_z_m)
        )

    final = _phase(spec, f"first_form_l_{FORM_L_SAMPLE_COUNT:02d}").targets[0]
    assert final.xyz_m == pytest.approx((0.212, -0.015, 0.118))
    assert spec.supported_lower_layer_fraction == pytest.approx(0.50)
    assert SURFACE_DRAG_HALF_FOLD_COMPENSATION_M == pytest.approx(0.015)
    assert CALIBRATED_POST_FEED_FREE_EDGE_X_OFFSET_M == pytest.approx(0.044)
    assert spec.free_edge_anchor_x_m == pytest.approx(0.059)
    assert FORWARD_LAY_ADVANCE_M == pytest.approx(0.197)
    assert spec.forward_lay_slack_m == pytest.approx(0.012)
    assert spec.final_held_edge_x_m == pytest.approx(0.091)
    assert spec.exposed_lower_strip_m == pytest.approx(0.032)
    assert spec.upper_edge_support_depth_m == pytest.approx(0.121)


def test_laydown_returns_held_edge_to_anchored_free_edge_without_z_path() -> None:
    spec = build_suspended_gravity_first_fold((0.0, 0.3, -0.3, 0.0), -0.005)
    contact_z = 0.010
    radius = 0.108
    previous_x = math.inf
    previous_z = math.inf
    for index in range(1, LAYDOWN_SAMPLE_COUNT + 1):
        phase = _phase(spec, f"first_gravity_laydown_{index:02d}")
        target = phase.targets[0]
        assert target.xyz_m[0] <= previous_x
        assert target.xyz_m[2] <= previous_z
        assert math.hypot(
            target.xyz_m[0] - spec.fold_line_x_m,
            target.xyz_m[2] - contact_z,
        ) == pytest.approx(radius)
        previous_x = target.xyz_m[0]
        previous_z = target.xyz_m[2]

    final_phase = _phase(spec, f"first_gravity_laydown_{LAYDOWN_SAMPLE_COUNT:02d}")
    assert final_phase.targets[0].xyz_m == pytest.approx((0.104, -0.015, 0.010))
    assert final_phase.attachment_event is None
    overcenter = _phase(
        spec, f"first_gravity_overcenter_{OVER_CENTER_SAMPLE_COUNT:02d}"
    )
    assert overcenter.targets[0].xyz_m == pytest.approx(
        (0.106, -0.015, 0.010)
    )
    assert overcenter.attachment_event is None
    preopen = _phase(spec, "first_gravity_preopen_clearance_02")
    assert preopen.targets[0].xyz_m == pytest.approx((0.106, -0.015, 0.016))
    assert preopen.attachment_event == (
        "release_both_edge_patches_after_gravity_laydown_gate"
    )
    assert CONTROLLED_UNDERFOLD_MARGIN_M == pytest.approx(0.002)
    assert spec.expected_footprint_xyxy_m == pytest.approx(
        (0.059, 0.212, -0.3, 0.0)
    )
    assert spec.requires_free_edge_observation is False
    assert spec.requires_post_release_correction is False

    clearance_lift = _phase(spec, "first_gravity_clearance_lift_04")
    clearance_outboard = _phase(spec, "first_gravity_clearance_outboard_04")
    assert clearance_lift.targets[0].xyz_m == pytest.approx(
        (0.106, -0.015, 0.200)
    )
    assert clearance_outboard.targets[0].xyz_m == pytest.approx(
        (0.1855, -0.015, 0.200)
    )
    assert all(
        not phase.name.startswith("first_gravity_release_sideways_")
        for phase in spec.phases
    )
    retreat = _phase(spec, "first_gravity_retreat")
    assert retreat.targets[0].xyz_m == pytest.approx((0.106, -0.015, 0.045))
    assert retreat.targets[1].xyz_m == pytest.approx((0.106, -0.285, 0.045))


def test_suspended_fold_rejects_non_nominal_towel_or_bad_table() -> None:
    with pytest.raises(TowelPlanningError, match="300 mm X"):
        build_suspended_gravity_first_fold((0.0, 0.29, -0.3, 0.0), -0.005)
    with pytest.raises(TowelPlanningError, match="table z"):
        build_suspended_gravity_first_fold(
            (0.0, 0.3, -0.3, 0.0), float("nan")
        )
def test_laydown_clearance_does_not_raise_the_contact_pose() -> None:
    table_z = -0.005
    spec = build_suspended_gravity_first_fold(
        (0.0, 0.3, -0.3, 0.0),
        table_z,
        contact_tcp_z_offset_m=0.009,
        laydown_tcp_z_offset_m=0.015,
    )

    contact = _phase(spec, "first_contact").targets[0]
    final_laydown = _phase(
        spec, f"first_gravity_laydown_{LAYDOWN_SAMPLE_COUNT:02d}"
    ).targets[0]
    overcenter = _phase(
        spec, f"first_gravity_overcenter_{OVER_CENTER_SAMPLE_COUNT:02d}"
    ).targets[0]

    assert contact.xyz_m[2] == pytest.approx(table_z + 0.009)
    assert final_laydown.xyz_m[2] == pytest.approx(table_z + 0.015)
    assert overcenter.xyz_m[2] == pytest.approx(table_z + 0.015)
    assert math.hypot(
        final_laydown.xyz_m[0] - spec.fold_line_x_m,
        final_laydown.xyz_m[2] - (table_z + 0.009),
    ) == pytest.approx(0.108)
    assert spec.laydown_tcp_z_m == pytest.approx(table_z + 0.015)

    with pytest.raises(TowelPlanningError, match="laydown TCP offset"):
        build_suspended_gravity_first_fold(
            (0.0, 0.3, -0.3, 0.0),
            table_z,
            contact_tcp_z_offset_m=0.009,
            laydown_tcp_z_offset_m=0.008,
        )
