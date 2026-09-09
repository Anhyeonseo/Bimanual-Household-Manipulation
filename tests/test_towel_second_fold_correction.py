import json
from pathlib import Path

import pytest

from tools.lib.towel_second_fold_correction import (
    SecondFoldEdgeObservation,
    SecondFoldPlanningError,
    classify_opposing_jaw_two_layer_contact,
    load_accepted_first_fold_footprint,
    plan_right_arm_second_fold_correction,
)


ROOT = Path(__file__).resolve().parents[1]
SUMMARY = (
    ROOT
    / "artifacts/bimanual/planning/"
    "towel_first_fold_surface_drag_r2_s1_summary.json"
)
HEADLESS = (
    ROOT
    / "tmp/towel_first_fold_surface_drag_half_comp15_measured_headless_20260903.json"
)


def observation(moving_y: float, stationary_y: float = -0.275) -> SecondFoldEdgeObservation:
    return SecondFoldEdgeObservation(
        moving_free_edge_y_m=moving_y,
        stationary_right_edge_y_m=stationary_y,
        moving_edge_x_span_m=(0.234, 0.390),
        confidence=0.90,
        settled=True,
        clear_pose_verified=True,
    )


def test_accepted_s1_shape_is_hash_locked_and_has_measured_footprint():
    footprint = load_accepted_first_fold_footprint(HEADLESS, SUMMARY)
    left, right, bottom, top = footprint.bounds_xyxy_m
    assert footprint.node_count == 1024
    assert right - left == pytest.approx(0.15632355, abs=1.0e-7)
    assert top - bottom == pytest.approx(0.30454152, abs=1.0e-7)


def test_unlisted_s1_result_is_rejected(tmp_path: Path):
    modified = json.loads(HEADLESS.read_text(encoding="utf-8"))
    modified["status"] += "_MODIFIED"
    path = tmp_path / "modified.json"
    path.write_text(json.dumps(modified), encoding="utf-8")
    with pytest.raises(SecondFoldPlanningError, match="hash"):
        load_accepted_first_fold_footprint(path, SUMMARY)


def test_small_second_fold_residual_needs_no_correction():
    plan = plan_right_arm_second_fold_correction(observation(-0.270))
    assert plan.required is False
    assert plan.status == "SECOND_FOLD_WITHIN_TOLERANCE"
    assert plan.reobserve_after_step is False


def test_underfold_moves_top_bundle_farther_right_with_edge_side_pinch():
    plan = plan_right_arm_second_fold_correction(observation(-0.255))
    assert plan.required is True
    assert plan.arm == "right"
    assert plan.signed_edge_residual_m == pytest.approx(0.020)
    assert plan.correction_delta_y_m == pytest.approx(-0.020)
    assert plan.correction_direction == "toward_right"
    assert plan.contact_mode == "top_bundle_edge_side_pinch"
    assert plan.requires_top_bundle_layer_separation is True
    assert plan.step_limited is False
    assert plan.expected_remaining_residual_m == pytest.approx(0.0)
    assert plan.grasp_xy_m == pytest.approx((0.312, -0.252))


def test_observed_edge_median_center_overrides_asymmetric_span_midpoint():
    candidate = observation(-0.255)
    candidate = SecondFoldEdgeObservation(
        moving_free_edge_y_m=candidate.moving_free_edge_y_m,
        stationary_right_edge_y_m=candidate.stationary_right_edge_y_m,
        moving_edge_x_span_m=(0.200, 0.340),
        confidence=candidate.confidence,
        settled=candidate.settled,
        clear_pose_verified=candidate.clear_pose_verified,
        moving_edge_center_x_m=0.285,
    )
    plan = plan_right_arm_second_fold_correction(candidate)
    assert plan.grasp_xy_m == pytest.approx((0.285, -0.252))


def test_overfold_pushes_exposed_top_bundle_toward_stationary_edge():
    plan = plan_right_arm_second_fold_correction(observation(-0.290))
    assert plan.required is True
    assert plan.signed_edge_residual_m == pytest.approx(-0.015)
    assert plan.correction_delta_y_m == pytest.approx(0.003)
    assert plan.correction_direction == "toward_left"
    assert plan.contact_mode == "exposed_overhang_open_jaw_fixed_pad_edge_push"
    assert plan.requires_top_bundle_layer_separation is False
    assert plan.reobserve_after_step is True
    assert plan.grasp_xy_m == pytest.approx((0.264, -0.294))
    assert plan.target_xy_m == pytest.approx((0.264, -0.291))
    assert plan.expected_remaining_residual_m == pytest.approx(-0.012)
    assert plan.step_limited is True


def test_overfold_uses_low_x_patch_to_keep_long_open_jaw_outside_cloth():
    candidate = SecondFoldEdgeObservation(
        moving_free_edge_y_m=-0.290,
        stationary_right_edge_y_m=-0.275,
        moving_edge_x_span_m=(0.200, 0.340),
        confidence=0.90,
        settled=True,
        clear_pose_verified=True,
        moving_edge_center_x_m=0.285,
    )
    plan = plan_right_arm_second_fold_correction(candidate)
    assert plan.grasp_xy_m == pytest.approx((0.230, -0.294))


@pytest.mark.parametrize(
    "candidate",
    [
        SecondFoldEdgeObservation(-0.255, -0.275, (0.234, 0.390), 0.79, True, True),
        SecondFoldEdgeObservation(-0.255, -0.275, (0.234, 0.390), 0.90, False, True),
        SecondFoldEdgeObservation(-0.255, -0.275, (0.234, 0.390), 0.90, True, False),
    ],
)
def test_correction_rejects_unreliable_camera_state(candidate):
    with pytest.raises(SecondFoldPlanningError):
        plan_right_arm_second_fold_correction(candidate)


def test_correction_larger_than_30_mm_is_split_then_reobserved():
    plan = plan_right_arm_second_fold_correction(observation(-0.220))
    assert plan.signed_edge_residual_m == pytest.approx(0.055)
    assert plan.correction_delta_y_m == pytest.approx(-0.030)
    assert plan.step_limited is True
    assert plan.expected_remaining_residual_m == pytest.approx(0.025)
    assert plan.reobserve_after_step is True


def test_correction_larger_than_recoverable_envelope_is_rejected():
    with pytest.raises(SecondFoldPlanningError, match="80 mm"):
        plan_right_arm_second_fold_correction(observation(-0.190))


def test_two_layer_gate_accepts_opposing_jaws_on_different_s1_halves():
    contact = classify_opposing_jaw_two_layer_contact(
        [20 * 32 + 7],
        [21 * 32 + 24],
        grid_side=32,
        upper_row_minimum=16,
    )
    assert contact.clean_two_layer_pinch is True
    assert contact.opposing_assignment == "fixed_first_moving_second"
    assert contact.upper_bundle_particles == (20 * 32 + 7, 21 * 32 + 24)


def test_two_layer_gate_accepts_reverse_opposing_face_assignment():
    contact = classify_opposing_jaw_two_layer_contact(
        [22 * 32 + 20],
        [23 * 32 + 4],
        grid_side=32,
        upper_row_minimum=16,
    )
    assert contact.clean_two_layer_pinch is True
    assert contact.opposing_assignment == "fixed_second_moving_first"


def test_two_layer_gate_rejects_same_jaw_touching_both_layers():
    contact = classify_opposing_jaw_two_layer_contact(
        [20 * 32 + 7, 21 * 32 + 24],
        [],
        grid_side=32,
        upper_row_minimum=16,
    )
    assert contact.clean_two_layer_pinch is False
    assert contact.opposing_assignment is None


def test_two_layer_gate_rejects_any_stationary_lower_bundle_contact():
    contact = classify_opposing_jaw_two_layer_contact(
        [20 * 32 + 7, 15 * 32 + 9],
        [21 * 32 + 24],
        grid_side=32,
        upper_row_minimum=16,
    )
    assert contact.clean_two_layer_pinch is False
    assert contact.lower_bundle_particles == (15 * 32 + 9,)
