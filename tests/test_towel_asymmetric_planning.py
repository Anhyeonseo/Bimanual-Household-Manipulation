from __future__ import annotations

import math
from pathlib import Path

import pytest

from tools.lib.grasp_yaw_kinematics import GraspYawKinematics
from tools.lib.towel_asymmetric_planning import (
    AsymmetricPlanningError,
    PhaseSpec,
    TaskPose,
    build_asymmetric_candidates,
    build_correction_probes,
    evaluate_task_pose,
    towel_bounds_from_worktable,
    validate_phase_contract,
)


ROOT = Path(__file__).resolve().parents[1]
REGISTERED_URDF_WITH_XML_DECLARATION = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
)
PLAN_ONLY_TOOL = ROOT / "tools/run/plan_towel_asymmetric_sequence_once.py"


def test_towel_is_centered_inside_validated_table_with_margin():
    bounds = towel_bounds_from_worktable(
        (0.37729576435909806, 0.37151273763179776),
        (0.14043161353744507, -0.3070740004777908),
    )
    left, right, bottom, top = bounds
    assert right - left == pytest.approx(0.300)
    assert top - bottom == pytest.approx(0.300)
    assert left - 0.14043161353744507 >= 0.030
    assert 0.14043161353744507 + 0.37729576435909806 - right >= 0.030


def test_table_without_required_perimeter_margin_fails_closed():
    with pytest.raises(AsymmetricPlanningError, match="perimeter margin"):
        towel_bounds_from_worktable((0.350, 0.400), (0.0, 0.0))


def test_candidate_family_keeps_operator_reviewed_first_fold_direction():
    candidates = build_asymmetric_candidates((0.0, 0.3, -0.3, 0.0), -0.005)
    assert len(candidates) == 8
    assert {item.first_active_arm for item in candidates} == {"left", "right"}
    assert {item.first_axis for item in candidates} == {"y"}
    assert {item.first_direction for item in candidates} == {"negative_to_positive"}
    assert {item.second_axis for item in candidates} == {"x"}
    assert {item.second_direction for item in candidates} == {
        "positive_to_negative",
        "negative_to_positive",
    }
    assert {item.second_assignment for item in candidates} == {
        "left_to_low_y",
        "left_to_high_y",
    }
    for item in candidates:
        validate_phase_contract(item.first_fold_phases)
        validate_phase_contract(item.second_fold_phases)
        assert item.first_fold_phases[-1].clear_pose is True
        assert item.second_fold_phases[-1].clear_pose is True
        assert item.first_expected_footprint_xyxy_m == pytest.approx(
            (0.0, 0.3, -0.15, 0.0)
        )
        left, right, bottom, top = item.final_expected_footprint_xyxy_m
        assert (right - left) * (top - bottom) == pytest.approx(0.0225)


def test_first_fold_is_single_arm_and_second_fold_is_endpoint_bimanual():
    candidate = build_asymmetric_candidates((0.0, 0.3, -0.3, 0.0), -0.005)[0]
    first_target_arms = {
        target.arm
        for phase in candidate.first_fold_phases
        for target in phase.targets
    }
    assert first_target_arms == {candidate.first_active_arm}
    second_contact = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_contact"
    )
    assert {target.arm for target in second_contact.targets} == {"left", "right"}
    points = sorted(target.xyz_m[1] for target in second_contact.targets)
    assert points[1] - points[0] == pytest.approx(0.150)
    second_laydown = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_laydown"
    )
    laydown_x = sorted(target.xyz_m[0] for target in second_laydown.targets)
    assert laydown_x[1] - laydown_x[0] == pytest.approx(0.040)
    left_return = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_left_reobserve_clear"
    )
    left_return_pregrasp = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_left_return_pregrasp"
    )
    left_departures = [
        phase for phase in candidate.second_fold_phases
        if phase.name.startswith("second_left_departure_")
    ]
    assert len(left_departures) == 11
    assert left_return_pregrasp.reuse_target_of == "second_pregrasp_left"
    assert left_return.clear_arm == "left"
    assert left_return.reverse_of == "second_left_departure_01"


def test_reverse_path_must_reference_an_earlier_single_arm_clear_phase():
    with pytest.raises(AsymmetricPlanningError, match="earlier phase"):
        validate_phase_contract(
            (
                PhaseSpec(
                    "invalid_reverse",
                    (),
                    clear_pose=True,
                    clear_arm="left",
                    reverse_of="missing_approach",
                ),
            )
        )


def test_correction_envelope_has_both_signed_extrema_on_both_corners():
    candidate = build_asymmetric_candidates((0.0, 0.3, -0.3, 0.0), -0.005)[0]
    probes = build_correction_probes(
        candidate.first_expected_footprint_xyxy_m, -0.005
    )
    assert len(probes) == 8
    assert {probe.primitive for probe in probes} == {
        "micro_drag",
        "lift_pull_place",
    }
    assert {probe.corner for probe in probes} == {"high_x_patch", "low_x_patch"}
    assert {probe.arm for probe in probes} == {"left", "right"}
    assert max(abs(value) for probe in probes for value in probe.offset_xy_m) == pytest.approx(0.030)
    for probe in probes:
        validate_phase_contract(probe.phases)
        assert probe.phases[-1].clear_pose is True
        assert probe.phases[-1].clear_arm == probe.arm
        assert probe.phases[-1].reverse_of == f"{probe.probe_id}_pregrasp"
        assert probe.phases[-2].reuse_target_of == f"{probe.probe_id}_pregrasp"
        assert probe.phases[0].path_cache_key == (
            f"correction_pregrasp_{probe.corner}_{probe.arm}"
        )


def test_registered_xml_declaration_urdf_loads_and_full_fk_is_checked():
    kinematics = GraspYawKinematics(
        REGISTERED_URDF_WITH_XML_DECLARATION, prefix="left_"
    )
    q = (0.38475717, 2.72363531, 2.24841306, -0.29532839, -1.15082774)
    by_name = dict(zip(kinematics.arm_joints, q, strict=True))
    _, xyz = kinematics.tcp_pose_in_root(by_name)
    finger = kinematics.finger_axis_in_root(by_name)
    jaw_yaw = math.atan2(float(finger[1]), float(finger[0]))
    pose = TaskPose(
        name="known_full_fk_pose",
        arm="left",
        xyz_m=tuple(float(value) for value in xyz),
        jaw_yaw_rad=jaw_yaw,
        semantic="test",
        layer="one_layer",
    )
    lower = (-1.633689, -0.228563, -0.681087, -0.515418, -2.241146)
    upper = (1.523243, 3.281185, 2.702874, 2.880816, 1.211845)
    result = evaluate_task_pose(kinematics, pose, q, lower, upper)
    assert result["task_pose_pass"] is True
    assert result["tcp_position_error_m"] == pytest.approx(0.0, abs=1.0e-12)
    assert result["jaw_yaw_error_rad"] == pytest.approx(0.0, abs=1.0e-12)
    assert len(result["tcp_rotation_matrix"]) == 3


def test_plan_only_tool_has_no_execution_or_resident_motion_client():
    source = PLAN_ONLY_TOOL.read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "BimanualStreamCommand" not in source
    assert "/bimanual_stream_adapter/command" not in source
    assert '"motion_commands": 0' in source
    assert '"execution_api_used": False' in source
    assert '"arbitrary_exact_6d_pose_claimed": False' in source
    assert 'request.group_name = collision_check_group' in source
    assert '"attached_lift"' in source
    assert '"released_retreat"' in source
    assert 'if "minimum_joint_limit_margin_rad" in evaluation' in source
