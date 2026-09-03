from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.lib import towel_isaac_s0
from tools.lib.towel_isaac_s0 import TowelIsaacS0Error, validate_s0_contract
from tools.lib.towel_isaac_collision import (
    classify_contact_pair,
    classify_contact_separation,
    expanded_phase_waypoints,
    interpolation_step_count,
    normalized_prim_path,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/towel_isaac_s0.json"
MATERIAL_CONFIG = ROOT / "config/towel_isaac_s1_material.json"
GRIPPER_CONFIG = ROOT / "config/so101_gripper_geometry.candidate.json"
FIRST_FOLD_SUMMARY = (
    ROOT
    / "artifacts/bimanual/planning/"
    "towel_first_fold_surface_drag_r2_s1_summary.json"
)


def test_repository_s0_contract_accepts_latest_canonical_plan_artifact() -> None:
    document = json.loads(CONTRACT.read_text(encoding="utf-8"))
    assert document["expected_strategy"]["first_direction"] == "robot_near_to_far"
    assert document["expected_strategy"]["second_direction"] == "right_to_left"
    assert document["expected_strategy"]["second_active_arm"] == "right"
    report = validate_s0_contract(CONTRACT, ROOT)
    assert report["status"] == "R2_S0_HOST_PREFLIGHT_PASS_SIMULATION_NOT_RUN"
    assert report["planning_segment_count"] == 846
    assert report["strict_state_sample_count"] == 12552
    assert report["simulation_executed"] is False
    plan_path = ROOT / document["sources"]["plan"]["path"]
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    phase_names = [
        phase["name"]
        for fold in ("first_fold", "second_fold")
        for phase in plan["selected_candidate"][fold]
    ]
    assert not any("route_shoulder_clearance" in name for name in phase_names)
    assert sum("route_base_shoulder_clearance" in name for name in phase_names) == 2
    assert sum(
        "route_partial_restore_base_before_target" in name for name in phase_names
    ) == 2


def test_s0_contract_never_authorizes_motion(tmp_path: Path) -> None:
    document = json.loads(CONTRACT.read_text(encoding="utf-8"))
    document["motion_authorized"] = True
    candidate = tmp_path / "unsafe.json"
    candidate.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(TowelIsaacS0Error, match="authorization/status"):
        validate_s0_contract(candidate, ROOT)


def test_s0_contract_rejects_strategy_drift(tmp_path: Path) -> None:
    document = json.loads(CONTRACT.read_text(encoding="utf-8"))
    document["expected_strategy"]["second_direction"] = "left_to_right"
    candidate = tmp_path / "stale.json"
    candidate.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(TowelIsaacS0Error, match="strategy mismatch"):
        validate_s0_contract(candidate, ROOT)


def test_s0_contract_rejects_unsafe_relay_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_load_yaml = towel_isaac_s0._load_yaml

    def load_yaml_with_relay_drift(path: Path):
        document = original_load_yaml(path)
        if path.name == "towel_task_contract.candidate.yaml":
            document["fold_policy"]["second_relay_fallback"] = {
                "strategy": "airborne_simultaneous_handover"
            }
        return document

    monkeypatch.setattr(towel_isaac_s0, "_load_yaml", load_yaml_with_relay_drift)
    with pytest.raises(TowelIsaacS0Error, match="bounded fold policy"):
        validate_s0_contract(CONTRACT, ROOT)


def test_collision_contact_classification_preserves_real_obstacles() -> None:
    env = "/World/envs/env_3"
    assert classify_contact_pair(
        f"{env}/Robot/workcell_base_link", f"{env}/Table"
    ) == "allowed_robot_mount_table_contact"
    assert classify_contact_pair(
        f"{env}/Robot/left_lower_arm_link",
        f"{env}/Robot/left_wrist_link",
    ) == "allowed_adjacent_robot_links"
    assert classify_contact_pair(
        f"{env}/Robot/workcell_base_link",
        f"{env}/Robot/left_upper_arm_link",
    ) == "allowed_srdf_disabled_robot_links"
    assert classify_contact_pair(
        f"{env}/Robot/left_gripper_link",
        f"{env}/Robot/right_gripper_link",
    ) == "forbidden_robot_self_collision"
    assert classify_contact_pair(
        f"{env}/Robot/right_wrist_link", f"{env}/Table"
    ) == "forbidden_robot_table_collision"
    assert classify_contact_pair(
        f"{env}/Robot/left_shoulder_link",
        f"{env}/Robot/left_gripper_link",
    ) == "bounded_shallow_mesh_contact"
    assert classify_contact_pair(
        f"{env}/Robot/workcell_base_link", f"{env}/ContactProbe"
    ) == "sensor_liveness_robot_probe_contact"
    assert classify_contact_pair(
        f"{env}/ContactProbe", f"{env}/Table"
    ) == "sensor_liveness_probe_table_contact"
    assert normalized_prim_path(f"{env}/Robot/right_wrist_link") == (
        "{ENV}/Robot/right_wrist_link"
    )


def test_collision_nested_physx_paths_honor_pinned_moveit_mesh_contract() -> None:
    env = "/World/envs/env_0"
    shoulder = f"{env}/Robot/Geometry/workcell_base_link/left_shoulder_link"
    gripper = (
        f"{shoulder}/left_upper_arm_link/left_lower_arm_link/"
        "left_wrist_link/left_gripper_link"
    )
    assert classify_contact_separation(shoulder, gripper, -0.0039) == (
        "accepted_moveit_bounded_shallow_mesh_contact"
    )
    assert classify_contact_separation(shoulder, gripper, -0.0041) == (
        "accepted_moveit_bounded_shallow_mesh_contact"
    )
    wrist = f"{gripper.rsplit('/', 1)[0]}"
    assert classify_contact_separation(shoulder, wrist, 0.001) == (
        "allowed_physx_contact_offset_proximity"
    )
    assert classify_contact_separation(shoulder, wrist, -0.001) == (
        "forbidden_robot_self_collision"
    )


def test_collision_interpolation_bounds_joint_increment() -> None:
    assert interpolation_step_count([0.0, 0.0], [0.041, -0.001]) == 3
    assert interpolation_step_count([0.0], [0.0]) == 1
    with pytest.raises(ValueError, match="equally sized"):
        interpolation_step_count([0.0], [0.0, 1.0])


def test_collision_expands_partial_moveit_trajectory_with_bounded_residual() -> None:
    phase = {
        "joint_positions_rad": [0.2, 0.3, 0.4],
        "start_positions_rad": [0.0, 0.3, 0.4],
        "trajectory_joint_names": ["joint_a"],
        "trajectory_positions_rad": [[0.1], [0.1995]],
    }
    assert expanded_phase_waypoints(
        phase,
        canonical_joint_names=["joint_a", "joint_b", "joint_c"],
        current_positions_rad=[0.0005, 0.3, 0.4],
    ) == [[0.1, 0.3, 0.4], [0.1995, 0.3, 0.4]]


def test_collision_rejects_discontinuous_or_stale_strict_trajectory() -> None:
    phase = {
        "joint_positions_rad": [0.2, 0.3],
        "start_positions_rad": [0.0, 0.3],
        "trajectory_joint_names": ["joint_a"],
        "trajectory_positions_rad": [[0.198]],
    }
    with pytest.raises(ValueError, match="start is discontinuous"):
        expanded_phase_waypoints(
            phase,
            canonical_joint_names=["joint_a", "joint_b"],
            current_positions_rad=[0.002, 0.3],
        )
    with pytest.raises(ValueError, match="terminal state"):
        expanded_phase_waypoints(
            phase,
            canonical_joint_names=["joint_a", "joint_b"],
            current_positions_rad=[0.0, 0.3],
        )


def test_collision_rejects_unknown_strict_trajectory_joint() -> None:
    phase = {
        "joint_positions_rad": [0.2],
        "start_positions_rad": [0.0],
        "trajectory_joint_names": ["missing_joint"],
        "trajectory_positions_rad": [[0.2]],
    }
    with pytest.raises(ValueError, match="unknown joint"):
        expanded_phase_waypoints(
            phase,
            canonical_joint_names=["joint_a"],
            current_positions_rad=[0.0],
        )


def test_isaac_collision_runner_remains_simulation_only() -> None:
    source = (
        ROOT / "tools/setup/isaac/run_towel_s0_collision_replay.py"
    ).read_text(encoding="utf-8")
    assert "S0_ISAACLAB_TRANSITION_COLLISION_PASS" in source
    assert '"motion_commands": 0' in source
    assert "enabled_self_collisions=True" in source
    for forbidden in (
        "rclpy",
        "create_publisher",
        "create_client",
        "ActionClient",
        "send_goal_async",
        "arm_and_enable",
        "torque_enable",
    ):
        assert forbidden not in source


def test_all_s0_gui_runners_reset_proxy_with_wxyz_quaternion() -> None:
    runners = (
        "run_towel_s0_articulation_replay.py",
        "run_towel_s0_camera_fov.py",
        "run_towel_s0_collision_replay.py",
    )
    for name in runners:
        source = (ROOT / "tools/setup/isaac" / name).read_text(encoding="utf-8")
        assert 'source["rigid_proxy_pose_xyz_yaw_rad"]' in source
        assert "torch.cos(half_yaw), zeros, zeros, torch.sin(half_yaw)" in source


def test_s1_surface_drop_settle_runner_is_fail_closed_and_simulation_only() -> None:
    source = (
        ROOT / "tools/setup/isaac/run_towel_s1_surface_drop_settle.py"
    ).read_text(encoding="utf-8")
    assert "PhysxSurfaceDeformableBodyMaterialCfg" in source
    assert "material_physical_fidelity_validated\": False" in source
    assert '"s1_completed": False' in source
    assert '"motion_commands": 0' in source
    assert "MAXIMUM_TABLE_PENETRATION_M" in source
    assert "MAXIMUM_TABLE_HOVER_CLEARANCE_M" in source
    assert "MAXIMUM_ENVIRONMENT_DIVERGENCE_M" in source
    assert "MAXIMUM_TABLE_RESET_ERROR_M" in source
    assert "replicate_physics=False" in source
    assert "scene.reset()" in source
    for forbidden in (
        "rclpy",
        "create_publisher",
        "create_client",
        "ActionClient",
        "send_goal_async",
        "arm_and_enable",
        "torque_enable",
    ):
        assert forbidden not in source


def test_s1_vertex_patch_lift_runner_is_fail_closed_and_simulation_only() -> None:
    source = (
        ROOT / "tools/setup/isaac/run_towel_s1_vertex_patch_lift.py"
    ).read_text(encoding="utf-8")
    assert "OmniPhysicsVtxXformAttachment" in source
    assert '"newton_state_constraint_used": newton_state_retention_used' in source
    assert '"actual_contact_particles_by_side"' in source
    assert '"proximity_fallback_used": False' in source
    assert '"TowelAttachmentFrame"' in source
    assert "orientation_xyzw" in source
    assert '"vertex_patch_attachment_lift_smoke_passed": legacy_attachment_used' in source
    assert '"contact_gated_retention_lift_smoke_passed"' in source
    assert '"contact_gated_q0_release_smoke_passed"' in source
    assert '"s1_completed": False' in source
    assert '"motion_commands": 0' in source
    assert "MINIMUM_PATCH_POINT_COUNT" in source
    assert "MAXIMUM_ATTACHMENT_SNAP_M" in source
    assert "MINIMUM_LIFT_M" in source
    assert "MAXIMUM_ATTACHMENT_PATCH_ENVIRONMENT_DIVERGENCE_M" in source
    for material_field in (
        '"mass_kg": CLOTH_MASS_KG',
        '"static_friction": CLOTH_STATIC_FRICTION',
        '"dynamic_friction": CLOTH_DYNAMIC_FRICTION',
        '"poissons_ratio": CLOTH_POISSONS_RATIO',
    ):
        assert material_field in source
    assert '"full_dynamic_cloth_shape_determinism_checked": False' in source
    assert "first_contact" in source
    assert "first_fold_01" in source
    assert "first_fold_16" in source
    assert "first_retreat" in source
    assert "disable_runtime_attachments" in source
    assert '"place_and_release_checked": args.place_release' in source
    assert '"self_contact_smoke_passed": args.self_contact' in source
    assert '"self_collision_checked": args.self_contact' in source
    assert "minimum_nonlocal_node_separation_m" in source
    assert "MINIMUM_SELF_CONTACT_NONLOCAL_NODE_SEPARATION_M" in source
    assert "enable_external_forces_every_iteration=args.self_contact" in source
    assert "enable_enhanced_determinism=args.self_contact" in source
    assert "surface_bend_stiffness=CLOTH_SURFACE_BEND_STIFFNESS_PA" in source
    assert "bend_damping=CLOTH_BEND_DAMPING_S_INV" in source
    assert "RELEASE_SHAPE_VIDEO_FPS = 24.0" in source
    assert "RELEASE_SHAPE_DISPLACEMENT_THRESHOLD_M = 0.001" in source
    assert "RELEASE_SHAPE_CONSECUTIVE_VIDEO_FRAMES = 5" in source
    assert '"maximum_node_speed_is_settle_gate": False' in source
    assert '"table_penetration_checked_only_within_xy_footprint": True' in source
    assert "nodes_over_table_footprint" in source
    assert "load_gripper_geometry_candidate" in source
    assert "PINCH_PROJECT_GRIPPER_JOINT_POSITIONS_RAD" in source
    assert "PINCH_MODEL_GRIPPER_JOINT_POSITIONS_RAD" in source
    assert "write_scripted_arm_state_and_drive_targets" in source
    # Arm links are written only for reset. A second, gripper-only write is
    # permitted during release so Newton cannot wrap the bounded jaw joint.
    assert source.count("write_joint_state_to_sim_index(") == 2
    assert "lock_gripper_state=True" in source
    assert 'joint_names_expr=[".*_gripper_joint"]' in source
    assert 'effort_limit_sim=2.0' in source
    assert 'stiffness=20.0' in source
    assert '"arm_state_overwritten_after_scene_reset": False' in source
    assert '"gripper_state_constrained_during_release": args.place_release' in source
    assert "_disable_imported_gripper_mesh_collisions" in source
    assert "_author_rubber_material" in source
    assert '"contact-gated-retention"' in source
    assert 'default="contact-gated-retention"' in source
    assert "self_collision=False" in source
    assert "enable_cloth_self_collision_after_pinch" in source
    assert "MAXIMUM_GRIPPER_CLOSING_RESIDUAL_RAD" in source
    assert "sim.pause()" in source
    assert '"final_cloth_shape_local_m_env_0"' in source
    assert '"scripted_attachment_used": scripted_attachment_used' in source
    assert '"physical_frictional_grasp_checked": args.grasp_mode == "frictional"' in source
    assert "FRICTIONAL_PREGRASP_ARM_POSITIONS_RAD" in source
    assert "FRICTIONAL_CONTACT_ARM_POSITIONS_RAD" in source
    assert "MAXIMUM_FRICTIONAL_APPROACH_TILT_RAD" in source
    assert "outside the table-contact allowance" in source
    for forbidden in (
        "rclpy",
        "create_publisher",
        "create_client",
        "ActionClient",
        "send_goal_async",
        "arm_and_enable",
        "torque_enable",
    ):
        assert forbidden not in source


def test_s1_full_shape_repeat_gate_is_fail_closed() -> None:
    source = (
        ROOT / "tools/setup/isaac/validate_towel_s1_full_shape_determinism.py"
    ).read_text(encoding="utf-8")
    assert 'PASS_STATUS = "S1_FULL_SHAPE_REPEAT_DETERMINISM_PASS"' in source
    assert "DEFAULT_MAXIMUM_NODE_DISTANCE_M = 0.001" in source
    assert 'len(shape) != 1024' in source
    assert '"identity_match": True' in source
    assert '"contact_gated_vertical_pinch": True' in source
    assert '"full_shape_repeat_determinism_passed": True' in source
    assert '"motion_commands": 0' in source


def test_s1_first_fold_summary_records_the_selected_repeatable_baseline() -> None:
    document = json.loads(FIRST_FOLD_SUMMARY.read_text(encoding="utf-8"))
    assert document["status"] == "R2_S1_FIRST_FOLD_ACCEPTED_WITHIN_55_45"
    assert document["motion_authorized"] is False
    assert document["automatic_execution_permitted"] is False
    assert document["completed_runs"] == 3
    assert document["trajectory"]["surface_drag_half_fold_compensation_m"] == (
        pytest.approx(0.015)
    )
    metrics = document["worst_case_metrics"]
    assert metrics["maximum_layer_fraction"] <= metrics[
        "maximum_layer_fraction_limit"
    ]
    assert metrics["maximum_p95_paired_vertex_xy_error_m"] <= metrics[
        "maximum_p95_paired_vertex_xy_error_limit_m"
    ]
    assert metrics["maximum_final_cloth_height_m"] <= metrics[
        "maximum_final_cloth_height_limit_m"
    ]
    assert metrics["maximum_first_fold_footprint_width_m"] <= metrics[
        "maximum_first_fold_footprint_width_limit_m"
    ]
    assert metrics["terminal_curl_amplitude_m"] == pytest.approx(0.0)
    assert metrics["terminal_curl_fraction"] == pytest.approx(0.0)
    assert metrics["maximum_repeat_node_distance_m"] <= metrics[
        "maximum_repeat_node_distance_limit_m"
    ]
    assert document["rejected_refinement"]["surface_drag_half_fold_compensation_m"] == (
        pytest.approx(0.0113)
    )


def test_s1_material_candidate_preserves_measurements_and_motion_lock() -> None:
    document = json.loads(MATERIAL_CONFIG.read_text(encoding="utf-8"))
    assert document["status"] == "R2_S1_MATERIAL_CALIBRATED_CANDIDATE"
    assert document["motion_authorized"] is False
    assert document["physical_fidelity_validated"] is False
    measurements = document["measurements"]
    assert measurements["mass"]["difference_readings_g"] == [50.0, 60.0, 60.0]
    assert measurements["mass"]["mean_g"] == pytest.approx(56.6666666667)
    assert measurements["thickness"]["four_layer_readings_mm"] == [
        20.3,
        18.1,
        17.6,
        17.2,
        18.1,
    ]
    assert measurements["thickness"]["one_layer_bulk_median_mm"] == pytest.approx(
        4.525
    )
    friction = measurements["towel_table_friction"]
    assert friction["onset_coefficient_mean"] == pytest.approx(0.7011952191)
    assert friction["sliding_coefficient_mean"] == pytest.approx(0.7370517928)
    edge_drop = measurements["edge_release_drop"]
    assert edge_drop["lift_height_mm"] == pytest.approx(100.1)
    assert edge_drop["trial_count"] == 3
    assert edge_drop["video_fps"] == pytest.approx(24.0)
    assert edge_drop["motion_onset_frames"] == [140, 278, 393]
    assert edge_drop["near_static_frames"] == [144, 283, 397]
    assert edge_drop["release_to_near_static_s"] == pytest.approx(
        [0.1666666667, 0.2083333333, 0.1666666667]
    )
    assert edge_drop["release_to_near_static_mean_s"] == pytest.approx(0.1805555556)
    assert edge_drop["video_stored_in_repository"] is False
    bending = measurements["cantilever_bending"]
    assert bending["target_angle_deg"] == pytest.approx(45.0)
    assert bending["overhang_readings_mm"] == [36.1, 36.6, 36.3]
    assert bending["overhang_mean_mm"] == pytest.approx(36.3333333333)
    assert bending["derived_peirce_like_bending_length_mm"] == pytest.approx(
        17.6934977919
    )
    assert bending["derived_value_is_calibration_target_not_certified_test"] is True
    assert len(document["calibration_trials"]) == 3
    assert all(
        trial["status"] == "REJECTED_SELF_CONTACT_SETTLE_FAIL"
        for trial in document["calibration_trials"]
    )
    model = document["model_candidate"]
    assert model["mass_kg"] == pytest.approx(0.0566666667)
    assert model["surface_thickness_m"] == pytest.approx(0.003)
    assert model["surface_bend_stiffness_pa"] == pytest.approx(2250.0)
    assert model["bend_damping_s_inv"] == pytest.approx(500.0)
    assert model["static_friction"] == model["dynamic_friction"] == pytest.approx(0.74)
    calibration = document["dedicated_scene_calibration"]
    assert calibration["cantilever"]["matched"] is True
    assert calibration["cantilever"]["target_angle_deg"] == pytest.approx(45.0)
    assert calibration["edge_release"]["matched"] is True
    assert calibration["edge_release"]["match_rule"] == "uncertainty_intervals_overlap"


def test_s1_gripper_candidate_preserves_q0_and_operator_command_choice() -> None:
    document = json.loads(GRIPPER_CONFIG.read_text(encoding="utf-8"))
    assert document["simulation_only"] is True
    assert document["motion_authorized"] is False
    q0 = document["q0_measurement"]
    assert q0["left_gap_mm"] == q0["right_gap_mm"] == pytest.approx(16.7)
    assert q0["estimated_uncertainty_mm"] == pytest.approx(0.5)
    assert document["geometry"][
        "detailed_stl_model_q_at_physical_q0_rad"
    ] == pytest.approx(0.186588)
    left_one = document["grasp_commands"]["left"]["one_layer"]
    assert left_one["validated_hold_anchor_rad"] == pytest.approx(0.246971)
    assert left_one["operational_candidate_rad"] == pytest.approx(0.251573)
    assert left_one["use_operational_candidate"] is True
    assert left_one["operational_candidate_revalidated"] is False
    assert document["generic_rubber_cloth_material_candidate"]["measured"] is False


def test_s1_material_calibration_runner_matches_measurement_geometry_without_motion() -> None:
    source = (
        ROOT / "tools/setup/isaac/run_towel_s1_material_calibration.py"
    ).read_text(encoding="utf-8")
    assert 'choices=("cantilever", "edge-release")' in source
    assert "CANTILEVER_OVERHANG_TARGET_M = 0.0363333333333" in source
    assert "CANTILEVER_ANGLE_TARGET_DEG = 45.0" in source
    assert "CANTILEVER_FRAME_DISPLACEMENT_THRESHOLD_M = 0.0010" in source
    assert "CANTILEVER_SHAPE_CONSECUTIVE_VIDEO_FRAMES = 5" in source
    assert "EDGE_RELEASE_LIFT_HEIGHT_M = 0.1001" in source
    assert "EDGE_RELEASE_LIFT_PREPARATION_S = 3.0" in source
    assert "EDGE_RELEASE_TARGET_SETTLE_S = 0.1805555556" in source
    assert "VIDEO_FPS = 24.0" in source
    assert "EDGE_RELEASE_OBSERVATION_THRESHOLDS_M" in source
    assert "surface_bend_stiffness=SURFACE_BEND_STIFFNESS_PA" in source
    assert "bend_damping=BEND_DAMPING_S_INV" in source
    assert "two_3x3_corner_patches_on_one_edge" in source
    assert "write_nodal_state_to_sim_index(release_state_w)" in source
    assert "UsdPhysics.PrismaticJoint.Define" in source
    assert 'CreateAxisAttr("Z")' in source
    assert '"world_prismatic_force_drive"' in source
    assert "def settle_shape_at_video_rate(" in source
    assert '"first_quiet_frame_in_confirmed_consecutive_run"' in source
    assert '"target_and_simulated_24_fps_uncertainty_intervals_overlap"' in source
    assert 'self_collision=args.experiment == "edge-release"' in source
    assert '"material_physical_fidelity_validated": False' in source
    assert '"maximum_vertex_speed_is_match_gate": False' in source
    assert '"s1_completed": False' in source
    assert '"motion_commands": 0' in source
    for forbidden in (
        "rclpy",
        "create_publisher",
        "create_client",
        "ActionClient",
        "send_goal_async",
        "arm_and_enable",
        "torque_enable",
    ):
        assert forbidden not in source
