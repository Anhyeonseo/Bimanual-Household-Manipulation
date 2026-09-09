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
    SECOND_LAYER_TCP_Z_OFFSET_M,
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
    SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M,
    SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M,
    SECOND_BIMANUAL_MINIMUM_GRASP_SEPARATION_M,
    SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M,
    SECOND_BIMANUAL_RIGHT_CONTACT_HEIGHT_ADDITION_M,
    SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
    SECOND_BIMANUAL_RELEASE_HEIGHT_ADDITION_M,
    SECOND_RELEASE_TCP_Z_OFFSET_M,
    SECOND_SINGLE_ARM_CONTACT_TCP_Z_OFFSET_M,
    SECOND_SINGLE_ARM_JAW_YAW_RAD,
    build_bimanual_second_fold,
    build_right_arm_second_fold_edge_handoff,
    build_bimanual_then_single_candidates,
    build_right_arm_second_fold_correction,
    build_right_arm_second_fold_stabilizer,
)
from tools.lib.towel_second_fold_correction import SecondFoldCorrectionPlan


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


def test_second_fold_correction_is_bounded_lift_translate_laydown_then_reobserve():
    plan = SecondFoldCorrectionPlan(
        required=True,
        status="SECOND_FOLD_UNDERFOLD_CORRECTION_REQUIRED",
        arm="right",
        signed_edge_residual_m=0.052,
        correction_delta_y_m=-0.030,
        correction_direction="toward_right",
        grasp_xy_m=(0.31, -0.205),
        target_xy_m=(0.31, -0.235),
        contact_mode="top_bundle_edge_side_pinch",
        requires_top_bundle_layer_separation=True,
        step_limited=True,
        expected_remaining_residual_m=0.022,
        reobserve_after_step=True,
    )
    phases = build_right_arm_second_fold_correction(plan, -0.005)
    validate_phase_contract(phases)
    names = [phase.name for phase in phases]
    departures = phases[:40]
    assert [phase.name for phase in departures] == [
        f"second_correction_departure_{index:02d}_right"
        for index in range(1, 41)
    ]
    assert departures[19].targets[0].xyz_m == pytest.approx(
        (0.2078333544, -0.2748407988, 0.0252464708)
    )
    assert departures[25].targets[0].xyz_m == pytest.approx(
        (0.220, -0.2748407988, 0.080)
    )
    assert departures[31].targets[0].xyz_m == pytest.approx(
        (0.31, -0.2909524466, 0.100)
    )
    assert departures[35].targets[0].xyz_m == pytest.approx((0.31, -0.205, 0.100))
    assert departures[39].targets[0].xyz_m == pytest.approx((0.31, -0.205, 0.046))
    assert names[-7] == "second_correction_pad_align"
    assert names[-6:] == [
        "second_correction_contact",
        "second_correction_lift",
        "second_correction_translate",
        "second_correction_laydown",
        "second_correction_retreat",
        "second_correction_reobserve_clear",
    ]
    contact = phases[-6].targets[0]
    lifted = phases[-5].targets[0]
    translated = phases[-4].targets[0]
    assert lifted.xyz_m[2] - contact.xyz_m[2] == pytest.approx(0.008)
    assert translated.xyz_m[1] - lifted.xyz_m[1] == pytest.approx(-0.030)
    assert phases[-1].clear_pose is True


def test_second_fold_stabilizer_uses_right_arm_away_from_left_laydown():
    phases = build_right_arm_second_fold_stabilizer(
        (0.234, 0.390, -0.275, 0.029), -0.005
    )
    validate_phase_contract(phases)
    assert phases[-3].name == "second_stabilizer_contact"
    assert phases[-2].name == "second_stabilizer_retreat"
    assert phases[-1].name == "second_stabilizer_reobserve_clear"
    contact = phases[-3].targets[0]
    assert contact.arm == "right"
    assert contact.layer == "four_layer_bundle"
    assert contact.xyz_m[1] > -0.260
    assert contact.xyz_m[1] < -0.123
    assert phases[-3].attachment_event == (
        "attach_right_stabilizer_after_actual_contact_gate"
    )
    assert phases[-2].attachment_event == (
        "release_right_stabilizer_after_left_clear_gate"
    )


def test_second_fold_edge_handoff_pinches_free_edge_away_from_left_grasp():
    footprint = (0.234, 0.390, -0.275, 0.029)
    phases = build_right_arm_second_fold_edge_handoff(footprint, -0.005)
    validate_phase_contract(phases)
    assert len(phases) == 45
    assert phases[-5].name == "second_handoff_contact"
    assert phases[-4].name == "second_handoff_hold"
    assert phases[-3].name == "second_handoff_release"
    assert phases[-2].name == "second_handoff_retreat"
    assert phases[-1].name == "second_handoff_reobserve_clear"
    contact = phases[-5].targets[0]
    assert contact.arm == "right"
    assert contact.layer == "upper_bundle_edge"
    assert contact.xyz_m[0] == pytest.approx(0.352)
    assert contact.xyz_m[1] == pytest.approx(-0.260)
    # Current S2 left grasp is centre - 20 mm, leaving 60 mm TCP separation.
    assert contact.xyz_m[0] - (0.5 * (footprint[0] + footprint[1]) - 0.020) == pytest.approx(0.060)
    assert phases[-5].attachment_event == (
        "attach_right_upper_edge_after_opposing_layer_contact_gate"
    )
    assert phases[-3].attachment_event == (
        "release_right_upper_edge_after_left_clear_gate"
    )


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
    assert second_contact.targets[0].xyz_m[:2] == pytest.approx((0.225, -0.285))
    assert candidate.second_fold_phases[-1].clear_arm == "right"


def test_preferred_second_fold_uses_four_layer_u_pinches_with_both_arms():
    footprint = (0.2335726, 0.3898962, -0.2750562, 0.0294853)
    phases, final = build_bimanual_second_fold(
        footprint, -0.005, direction="left_to_right"
    )
    validate_phase_contract(phases)

    contact = next(
        phase for phase in phases if phase.name == "second_bimanual_contact"
    )
    by_arm = {target.arm: target for target in contact.targets}
    assert set(by_arm) == {"left", "right"}
    assert by_arm["left"].xyz_m[:2] == pytest.approx(
        (
            footprint[0] + SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M,
            footprint[3] - 0.015,
        )
    )
    assert by_arm["right"].xyz_m[:2] == pytest.approx(
        (
            footprint[1] - SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M,
            footprint[3] - 0.015,
        )
    )
    assert (
        by_arm["right"].xyz_m[0] - by_arm["left"].xyz_m[0]
        >= SECOND_BIMANUAL_MINIMUM_GRASP_SEPARATION_M
    )
    assert by_arm["left"].xyz_m[2] == pytest.approx(
        -0.005
        + SECOND_LAYER_TCP_Z_OFFSET_M
        + SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M
    )
    assert by_arm["right"].xyz_m[2] == pytest.approx(
        -0.005
        + SECOND_LAYER_TCP_Z_OFFSET_M
        + SECOND_BIMANUAL_RIGHT_CONTACT_HEIGHT_ADDITION_M
    )
    assert by_arm["left"].jaw_yaw_rad == pytest.approx(math.pi / 2.0)
    assert by_arm["right"].jaw_yaw_rad == pytest.approx(0.0)
    assert all(
        target.layer == "two_layer_bundle" for target in contact.targets
    )
    assert all(
        target.maximum_finger_tilt_rad
        == pytest.approx(SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD)
        for target in contact.targets
    )
    assert contact.attachment_event == (
        "attach_two_four_layer_u_pinches_after_dual_contact_gate"
    )

    laydown = next(
        phase
        for phase in phases
        if phase.attachment_event
        == "release_two_four_layer_u_pinches_after_dual_laydown_gate"
    )
    assert {target.arm for target in laydown.targets} == {"left", "right"}
    assert [target.xyz_m[1] for target in laydown.targets] == pytest.approx(
        [footprint[2] + 0.015, footprint[2] + 0.015]
    )
    assert [target.xyz_m[2] for target in laydown.targets] == pytest.approx(
        [
            -0.005
            + SECOND_RELEASE_TCP_Z_OFFSET_M
            + SECOND_BIMANUAL_RELEASE_HEIGHT_ADDITION_M
        ]
        * 2
    )
    assert phases[-2].name == "second_bimanual_retreat"
    assert phases[-1].name == "second_bimanual_reobserve_clear"
    assert phases[-1].clear_pose is True
    assert phases[-1].clear_arm is None
    assert final == pytest.approx(
        (footprint[0], footprint[1], footprint[2], 0.5 * (footprint[2] + footprint[3]))
    )


def test_bimanual_second_fold_rejects_edge_too_short_for_safe_separation():
    with pytest.raises(TowelPlanningError, match="too short"):
        build_bimanual_second_fold(
            (0.0, 0.120, -0.3, 0.0), -0.005, direction="left_to_right"
        )


def test_left_to_right_second_fold_uses_left_edge_midpoint_and_left_arm():
    candidate = next(
        item
        for item in build_bimanual_then_single_candidates(
            (0.0, 0.3, -0.3, 0.0), -0.005
        )
        if item.second_active_arm == "left"
        and item.second_direction == "left_to_right"
    )
    contact = next(
        phase
        for phase in candidate.second_fold_phases
        if phase.name == "second_contact"
    ).targets[0]
    laydown = next(
        phase
        for phase in candidate.second_fold_phases
        if phase.attachment_event
        == "release_midpoint_bundle_after_laydown_gate"
    ).targets[0]

    assert contact.arm == "left"
    assert contact.xyz_m[:2] == pytest.approx((0.225, -0.015))
    assert laydown.xyz_m[:2] == pytest.approx((0.225, -0.285))
    assert candidate.final_expected_footprint_xyxy_m == pytest.approx(
        (0.15, 0.3, -0.3, -0.15)
    )
    assert candidate.second_fold_phases[-1].clear_arm == "left"


def test_second_contact_and_release_use_supported_two_layer_height():
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
        if phase.attachment_event
        == "release_midpoint_bundle_after_laydown_gate"
    ).targets[0]
    retreat = next(
        phase for phase in candidate.second_fold_phases
        if phase.name == "second_retreat"
    ).targets[0]
    assert contact.xyz_m[2] == pytest.approx(
        table_z + SECOND_SINGLE_ARM_CONTACT_TCP_Z_OFFSET_M
    )
    assert laydown.xyz_m[2] == pytest.approx(table_z + 0.018)
    assert retreat.xyz_m[2] == pytest.approx(table_z + 0.068)
    assert contact.maximum_approach_tilt_rad == pytest.approx(
        MAXIMUM_APPROACH_TILT_RAD
    )
    assert contact.jaw_yaw_rad == pytest.approx(SECOND_SINGLE_ARM_JAW_YAW_RAD)
    assert sum(
        phase.name.startswith("second_precontact_")
        for phase in candidate.second_fold_phases
    ) == 9
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


def test_fixed_pad_normal_keeps_its_directed_opposing_face_assignment():
    kinematics = GraspYawKinematics(
        REGISTERED_URDF_WITH_XML_DECLARATION, prefix="right_"
    )
    q = (0.0022137155, 2.2643454984, 1.3548553085, 0.0806338801, 0.9223087051)
    by_name = dict(zip(kinematics.arm_joints, q, strict=True))
    _, xyz = kinematics.tcp_pose_in_root(by_name)
    fixed_normal = kinematics.fixed_jaw_pad_normal_in_root(by_name)
    fixed_normal_yaw = math.atan2(float(fixed_normal[1]), float(fixed_normal[0]))
    lower = (-1.442, -0.290, -0.729, -0.598, -1.993)
    upper = (1.454, 3.283, 2.686, 2.563, 1.414)
    matching = TaskPose(
        name="matching_fixed_pad_direction",
        arm="right",
        xyz_m=tuple(float(value) for value in xyz),
        jaw_yaw_rad=0.0,
        semantic="test",
        layer="upper_bundle_edge",
        enforce_finger_yaw=False,
        fixed_pad_normal_yaw_rad=fixed_normal_yaw,
    )
    reversed_direction = TaskPose(
        name="reversed_fixed_pad_direction",
        arm="right",
        xyz_m=matching.xyz_m,
        jaw_yaw_rad=0.0,
        semantic="test",
        layer="upper_bundle_edge",
        enforce_finger_yaw=False,
        fixed_pad_normal_yaw_rad=fixed_normal_yaw + math.pi,
    )
    matching_result = evaluate_task_pose(
        kinematics, matching, q, lower, upper
    )
    reversed_result = evaluate_task_pose(
        kinematics, reversed_direction, q, lower, upper
    )
    assert matching_result["task_pose_pass"] is True
    assert matching_result["fixed_pad_normal_yaw_error_rad"] == pytest.approx(0.0)
    assert reversed_result["task_pose_pass"] is False
    assert reversed_result["fixed_pad_normal_yaw_error_rad"] == pytest.approx(math.pi)


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
    assert source.count('"attached_transfer"') >= 2
    assert '"released_retreat"' in source
    assert 'if "minimum_joint_limit_margin_rad" in evaluation' in source
    assert '"maximum_dense_tcp_path_deviation_m_by_arm"' in source
    assert "MAXIMUM_DENSE_TCP_PATH_DEVIATION_M = 0.004" in source
    assert 'phase.name.startswith(\n                "second_bimanual_fold_"' in source
    assert 'segment["bimanual_second_fold_exact_chord"] = True' in source
    assert 'segment["bimanual_second_contact_exact_chord"] = True' in source
    assert 'f"{phase_name}_route_right_arm", current, right_clear' in source
    assert 'so101_gripper_s2_four_layer.candidate.json' in source
    assert 'second_layer_gripper["four_layer_project_contact_target_rad"]' in source
    assert 'modes["four_layer_contact"] = mixed("four_layer_contact")' in source
    assert 'second_layer_gripper["two_layer_project_contact_target_rad"]' not in source


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
