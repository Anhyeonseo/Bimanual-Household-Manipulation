from pathlib import Path

import pytest

from tools.lib.so101_gripper_geometry import load_gripper_geometry_candidate


ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = ROOT / "config/so101_gripper_geometry.candidate.json"


def test_q0_gap_and_opposite_coordinate_directions_are_locked() -> None:
    candidate = load_gripper_geometry_candidate(CANDIDATE)
    assert candidate.q0_gap_mm == {"left": 16.7, "right": 16.7}
    assert candidate.q0_gap_uncertainty_mm == pytest.approx(0.5)
    assert candidate.fixed_jaw_rubber_pad_thickness_m == pytest.approx(0.0022)
    assert candidate.moving_jaw_has_matching_rubber_pad is False
    assert candidate.measurements_include_fixed_jaw_rubber_pad is True
    assert candidate.model_q_at_physical_q0_rad == pytest.approx(0.186588)
    assert candidate.model_q_range_rad == pytest.approx((0.179888, 0.1932944))
    assert candidate.project_to_model(0.0) == pytest.approx(0.186588)
    assert candidate.project_to_model(0.1) < candidate.project_to_model(0.0)
    assert candidate.model_to_project(candidate.project_to_model(0.123)) == pytest.approx(
        0.123
    )


def test_operator_selected_operational_grasp_commands_are_preserved() -> None:
    candidate = load_gripper_geometry_candidate(CANDIDATE)
    assert candidate.grasp_project_rad == {
        "left": {1: pytest.approx(0.251573), 4: pytest.approx(0.210155)},
        "right": {1: pytest.approx(0.188680), 4: pytest.approx(0.128854)},
    }
    assert candidate.grasp_model_rad("left", 1) == pytest.approx(-0.064985)
    assert candidate.grasp_model_rad("right", 1) == pytest.approx(-0.002092)
    assert candidate.release_model_rad == pytest.approx(0.186588)
    assert candidate.model_limits_rad("left") == pytest.approx(
        (-0.083393, 2.041171)
    )
    assert candidate.model_limits_rad("right") == pytest.approx(
        (-0.029703, 2.105598)
    )


def test_generic_rubber_candidate_is_not_misrepresented_as_measured() -> None:
    candidate = load_gripper_geometry_candidate(CANDIDATE)
    assert candidate.rubber_static_friction == pytest.approx(1.0)
    assert candidate.rubber_dynamic_friction == pytest.approx(0.8)
    assert candidate.rubber_static_friction >= candidate.rubber_dynamic_friction
