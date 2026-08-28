from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from tools.lib.grasp_yaw_kinematics import GraspYawKinematics
from tools.lib.towel_task_pose_planning import (
    CORRECTION_DEPARTURE_FRACTIONS,
    MAXIMUM_APPROACH_TILT_RAD,
    MAXIMUM_ATTACHED_TRANSFER_TILT_RAD,
    TowelPlanningError,
    PhaseSpec,
    TaskPose,
    build_correction_probes,
    evaluate_task_pose,
    point_segment_distance_m,
    towel_bounds_from_worktable,
    validate_phase_contract,
)
from tools.lib.towel_bimanual_then_single_planning import (
    build_bimanual_then_single_candidates,
)


ROOT = Path(__file__).resolve().parents[1]
REGISTERED_URDF_WITH_XML_DECLARATION = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
)
PLAN_ONLY_TOOL = ROOT / "tools/run/plan_towel_fold_sequence_once.py"
KINEMATIC_DIAGNOSTIC_TOOL = (
    ROOT / "tools/run/diagnose_towel_fold_kinematics.py"
)
PLAN_ONLY_LAUNCH = (
    ROOT / "ros2_ws/src/so101_bringup/launch/towel_fold_plan_only.launch.py"
)
MOVEIT_RVIZ_CONFIG = (
    ROOT / "ros2_ws/src/so101_moveit_config/config/moveit.rviz"
)


def test_tcp_path_distance_is_zero_on_adjacent_task_chord():
    assert point_segment_distance_m(
        (0.5, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.0)


def test_tcp_path_distance_measures_lateral_moveit_deviation():
    assert point_segment_distance_m(
        (0.5, 0.003, 0.004), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)


def test_tcp_path_distance_clamps_before_and_after_segment():
    assert point_segment_distance_m(
        (-0.003, 0.004, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)
    assert point_segment_distance_m(
        (1.003, 0.004, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    ) == pytest.approx(0.005)


def test_tcp_path_distance_rejects_nonfinite_or_wrong_shape():
    with pytest.raises(TowelPlanningError, match="three finite XYZ"):
        point_segment_distance_m((0.0, 0.0), (0.0,) * 3, (1.0,) * 3)
    with pytest.raises(TowelPlanningError, match="must be finite"):
        point_segment_distance_m(
            (float("nan"), 0.0, 0.0), (0.0,) * 3, (1.0,) * 3
        )


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
    with pytest.raises(TowelPlanningError, match="perimeter margin"):
        towel_bounds_from_worktable((0.350, 0.400), (0.0, 0.0))


def test_candidate_family_uses_bimanual_first_and_orthogonal_single_second():
    candidates = build_bimanual_then_single_candidates(
        (0.0, 0.3, -0.3, 0.0), -0.005
    )
    assert len(candidates) == 4
    assert {item.first_arm_assignment for item in candidates} == {
        "left_high_y_right_low_y"
    }
    assert {item.first_axis for item in candidates} == {"x"}
    assert {item.first_direction for item in candidates} == {"robot_near_to_far"}
    assert {item.second_axis for item in candidates} == {"y"}
    assert {item.second_direction for item in candidates} == {
        "right_to_left",
        "left_to_right",
    }
    assert {item.second_active_arm for item in candidates} == {"right", "left"}
    for item in candidates:
        validate_phase_contract(item.first_fold_phases)
        validate_phase_contract(item.second_fold_phases)
        assert item.first_fold_phases[-1].clear_pose is True
        assert item.second_fold_phases[-1].clear_pose is True
        assert item.first_expected_footprint_xyxy_m == pytest.approx(
            (0.15, 0.3, -0.3, 0.0)
        )
        left, right, bottom, top = item.final_expected_footprint_xyxy_m
        assert (right - left) * (top - bottom) == pytest.approx(0.0225)


def test_first_fold_is_bimanual_and_second_fold_is_one_midpoint():
    candidate = build_bimanual_then_single_candidates(
        (0.0, 0.3, -0.3, 0.0), -0.005
    )[0]
    first_target_arms = {
        target.arm
        for phase in candidate.first_fold_phases
        for target in phase.targets
    }
    assert first_target_arms == {"left", "right"}
    first_contact = next(
        phase for phase in candidate.first_fold_phases
        if phase.name == "first_contact"
    )
    assert {target.arm for target in first_contact.targets} == {"left", "right"}
    contact_y = sorted(target.xyz_m[1] for target in first_contact.targets)
    assert contact_y[1] - contact_y[0] == pytest.approx(0.270)
    second_contact = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_contact"
    )
    assert len(second_contact.targets) == 1
    assert second_contact.targets[0].arm == "right"
    assert second_contact.targets[0].xyz_m[:2] == pytest.approx((0.225, -0.270))
    assert candidate.second_fold_phases[-1].clear_arm == "right"


def test_second_contact_stays_low_but_release_uses_reachable_height():
    table_z = -0.005
    candidate = build_bimanual_then_single_candidates(
        (0.0, 0.3, -0.3, 0.0), table_z
    )[0]
    contact = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_contact"
    ).targets[0]
    laydown = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_fold_08"
    ).targets[0]
    retreat = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_retreat"
    ).targets[0]
    assert contact.xyz_m[2] == pytest.approx(table_z + 0.016)
    assert laydown.xyz_m[2] == pytest.approx(table_z + 0.040)
    assert retreat.xyz_m[2] == pytest.approx(table_z + 0.090)
    assert contact.maximum_approach_tilt_rad == pytest.approx(
        MAXIMUM_APPROACH_TILT_RAD
    )
    assert laydown.maximum_approach_tilt_rad == pytest.approx(
        MAXIMUM_ATTACHED_TRANSFER_TILT_RAD
    )


def test_reverse_path_must_reference_an_earlier_single_arm_clear_phase():
    with pytest.raises(TowelPlanningError, match="earlier phase"):
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
    candidate = build_bimanual_then_single_candidates(
        (0.0, 0.3, -0.3, 0.0), -0.005
    )[0]
    probes = build_correction_probes(
        candidate.first_expected_footprint_xyxy_m, -0.005
    )
    assert len(probes) == 8
    assert {probe.primitive for probe in probes} == {
        "micro_drag",
        "lift_pull_place",
    }
    assert {probe.corner for probe in probes} == {"high_y_patch", "low_y_patch"}
    assert {probe.arm for probe in probes} == {"left", "right"}
    assert max(abs(value) for probe in probes for value in probe.offset_xy_m) == pytest.approx(0.030)
    for probe in probes:
        validate_phase_contract(probe.phases)
        departure_phases = tuple(
            phase
            for phase in probe.phases
            if phase.path_cache_key is not None
            and phase.path_cache_key.startswith("correction_pregrasp_")
        )
        assert len(departure_phases) == 2 * len(CORRECTION_DEPARTURE_FRACTIONS)
        assert probe.phases[-1].clear_pose is True
        assert probe.phases[-1].clear_arm == probe.arm
        assert probe.phases[-1].reverse_of == (
            departure_phases[0].name if probe.arm == "left" else None
        )
        return_pregrasp = next(
            phase
            for phase in probe.phases
            if phase.name == f"{probe.probe_id}_return_pregrasp"
        )
        assert return_pregrasp.reuse_target_of == f"{probe.probe_id}_pregrasp"
        assert probe.phases[0].path_cache_key == (
            f"correction_pregrasp_{probe.corner}_{probe.arm}_gateway_01"
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
    assert '"maximum_dense_tcp_path_deviation_m_by_arm"' in source
    assert "MAXIMUM_DENSE_TCP_PATH_DEVIATION_M = 0.004" in source


def test_full_fk_diagnostic_is_explicitly_not_a_moveit_or_collision_pass():
    source = KINEMATIC_DIAGNOSTIC_TOOL.read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "BimanualStreamCommand" not in source
    assert '"motion_commands": 0' in source
    assert '"moveit_segment_planning_checked": False' in source
    assert '"transition_collision_checked": False' in source
    assert '"physical_fold_success_checked": False' in source


def test_towel_launch_disables_every_moveit_execution_path():
    source = PLAN_ONLY_LAUNCH.read_text(encoding="utf-8")
    assert '"allow_trajectory_execution": "false"' in source
    assert "MoveGroupExecuteTrajectoryAction" in source
    assert "MoveGroupMoveAction" in source


def test_moveit_rviz_enables_canonical_towel_marker_topic():
    document = yaml.safe_load(MOVEIT_RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = document["Visualization Manager"]["Displays"]
    marker = next(
        item for item in displays
        if item.get("Class") == "rviz_default_plugins/MarkerArray"
    )
    assert marker["Marker Topic"]["Value"] == "/towel_fold_markers"
    assert marker["Value"] is True
