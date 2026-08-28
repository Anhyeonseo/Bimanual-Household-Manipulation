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
    assert '"direct_articulation_link_attachment": True' in source
    assert '"TowelAttachmentFrame"' in source
    assert "orientation_xyzw" in source
    assert '"vertex_patch_attachment_lift_smoke_passed": True' in source
    assert '"vertex_patch_place_release_smoke_passed": args.place_release' in source
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
