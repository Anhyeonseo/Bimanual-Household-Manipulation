from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_top_object_pose_exports_source_pixel_geometry() -> None:
    message = (
        ROOT / "ros2_ws/src/so101_interfaces/msg/TopObjectPose.msg"
    ).read_text(encoding="utf-8")
    node = (
        ROOT
        / "ros2_ws/src/so101_top_perception/so101_top_perception/node.py"
    ).read_text(encoding="utf-8")
    for field in (
        "float32 center_x_px",
        "float32 center_y_px",
        "uint32 image_width_px",
        "uint32 image_height_px",
    ):
        assert field in message
    assert 'pose["raw_center_px"][0]' in node
    assert "output.image_width_px = int(message.width)" in node


def test_dual_moveit_has_independent_left_and_right_groups() -> None:
    root = ET.parse(
        ROOT / "ros2_ws/src/so101_moveit_config/config/so101_dual.srdf"
    ).getroot()
    groups = {group.attrib["name"]: group for group in root.findall("group")}
    assert set(groups) == {"left_arm", "right_arm"}
    assert groups["left_arm"].find("chain").attrib == {
        "base_link": "left_base_link",
        "tip_link": "left_gripper_frame_link",
    }
    assert groups["right_arm"].find("chain").attrib == {
        "base_link": "right_base_link",
        "tip_link": "right_gripper_frame_link",
    }
    kinematics = yaml.safe_load(
        (
            ROOT
            / "ros2_ws/src/so101_moveit_config/config/kinematics_dual.yaml"
        ).read_text(encoding="utf-8")
    )
    assert set(kinematics) == {"left_arm", "right_arm"}


def test_dual_moveit_limits_match_operator_approved_arm_limits() -> None:
    approved = json.loads(
        (ROOT / "config/bimanual_operational_limits.json").read_text(
            encoding="utf-8"
        )
    )
    moveit = yaml.safe_load(
        (
            ROOT
            / "ros2_ws/src/so101_moveit_config/config/joint_limits_dual.yaml"
        ).read_text(encoding="utf-8")
    )["joint_limits"]
    for side in ("left", "right"):
        for short_name in (
            "base",
            "shoulder",
            "elbow",
            "wrist_flex",
            "wrist_roll",
        ):
            expected = approved["arms"][side][short_name]
            actual = moveit[f"{side}_{short_name}_joint"]
            assert actual["min_position"] == pytest.approx(
                expected["minimum_urad"] / 1e6
            )
            assert actual["max_position"] == pytest.approx(
                expected["maximum_urad"] / 1e6
            )


def test_dynamic_planner_routes_by_pixels_and_remains_plan_only() -> None:
    source = (ROOT / "tools/plan_top_camera_pick_place_once.py").read_text(
        encoding="utf-8"
    )
    assert "select_arm_for_pixel" in source
    assert 'f"{side}_arm"' in source
    assert 'f"{side}_gripper_frame_link"' in source
    assert '"nonselected_arm_behavior": "hold_bimanual_q0"' in source
    assert "workspace_coordinates_for_arm" in source
    assert "GraspYawKinematics" in source
    assert "least_squares" in source
    assert "solve_endpoint_pose_with_locked_wrist" in source
    assert "locked_wrist_roll = 0.0" in source
    assert '"wrist_roll_yaw_correction_applied": False' in source
    assert '"wrist_roll_policy": "hold_bimanual_q0"' in source
    assert '"schema_version": 9' in source
    assert "BASELINE_PICK_GRASP_OFFSET_M = 0.011" in source
    assert "PREVIOUS_PICK_GRASP_OFFSET_M = 0.005" in source
    assert "PICK_GRASP_OFFSET_M = 0.002" in source
    assert "PICK_GRASP_CUMULATIVE_DOWNWARD_ADJUSTMENT_M" in source
    assert '"height_adjustment"' in source
    assert "DYNAMIC_PICK_HEIGHT_OFFSET_PASS" in source
    assert "GRIPPER_OPEN_TARGET_RAW = 2009" in source
    assert "GRIPPER_CLOSE_TARGET_RAW = 1948" in source
    assert '"phase": "pick_open"' in source
    assert '"gripper_contract"' in source
    assert "DYNAMIC_PICK_GRIPPER_CONTRACT_PASS" in source
    assert "DYNAMIC_PICK_WRIST_BRANCH_PASS" in source
    assert "--plan-only is required; this tool has no execution client" in source
    assert "BimanualStreamCommand" not in source


def test_dynamic_plan_opens_before_approach_and_closes_at_grasp() -> None:
    path = ROOT / "tools/plan_top_camera_pick_place_once.py"
    spec = importlib.util.spec_from_file_location(
        "plan_top_camera_pick_place_gripper_order_test", path
    )
    planner = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = planner
    spec.loader.exec_module(planner)

    segment = {
        "target_positions_rad": [0.0] * 5,
        "maximum_joint_delta_rad": 0.0,
    }
    steps = planner.steps_from_phases(
        [
            {"name": "q0_to_pick_pregrasp", "segments": [segment]},
            {"name": "pick_grasp_to_lift", "segments": [segment]},
            {"name": "place_grasp_to_retreat", "segments": [segment]},
        ]
    )

    assert [step["phase"] for step in steps] == [
        "pick_open",
        "q0_to_pick_pregrasp",
        "pick_close",
        "pick_grasp_to_lift",
        "place_release",
        "place_grasp_to_retreat",
    ]
    assert steps[0]["target_position_rad"] == pytest.approx(
        (2048 - 2009) * planner.RAW_STEP_RAD
    )
    assert steps[2]["target_position_rad"] == pytest.approx(
        (2048 - 1948) * planner.RAW_STEP_RAD
    )


def test_endpoint_solver_locks_wrist_roll_at_bimanual_q0() -> None:
    path = ROOT / "tools/plan_top_camera_pick_place_once.py"
    spec = importlib.util.spec_from_file_location(
        "plan_top_camera_pick_place_locked_wrist_test", path
    )
    planner = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = planner
    spec.loader.exec_module(planner)

    joint_names = ("base", "shoulder", "elbow", "flex", "roll")

    class FakeKinematics:
        
        def tcp_position(self, positions):
            return np.array(
                [positions[name] for name in joint_names[:3]], dtype=float
            )

        
        def finger_yaw(self, positions):
            return float(positions["roll"] + 0.67)

    result = planner.solve_endpoint_pose_with_locked_wrist(
        FakeKinematics(),
        "left",
        joint_names,
        (0.1, 0.2, 0.3, 0.4, -2.2),
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (0.1, 0.2, 0.3),
        0.0,
        np.array([-3.0, -3.0, -3.0, -3.0, -2.3]),
        np.array([3.0, 3.0, 3.0, 3.0, 1.21]),
    )

    assert result["positions_rad"][4] == pytest.approx(0.0)
    assert result["wrist_roll_reference_rad"] == pytest.approx(0.0)
    assert result["wrist_roll_delta_rad"] == pytest.approx(0.0)
    assert result["position_residual_m"] < 1.0e-9


def test_bimanual_bringup_is_separate_from_legacy_left_bringup() -> None:
    dual = ROOT / "ros2_ws/src/so101_bringup/launch/external_bimanual_moveit.launch.py"
    legacy = ROOT / "ros2_ws/src/so101_bringup/launch/external_stm32_moveit.launch.py"
    assert dual.is_file()
    assert "dual_move_group.launch.py" in dual.read_text(encoding="utf-8")
    assert "dual_move_group.launch.py" not in legacy.read_text(encoding="utf-8")


def test_resident_feedback_preserves_private_topic_and_adds_moveit_alias() -> None:
    source = (
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bimanual_stream_node.py"
    ).read_text(encoding="utf-8")
    assert '"~/joint_states"' in source
    assert '"/joint_states"' in source
    assert "self._moveit_joint_state_publisher.publish(joint_state)" in source


def test_dual_moveit_bringup_disables_moveit_execution() -> None:
    source = (
        ROOT
        / "ros2_ws/src/so101_bringup/launch/external_bimanual_moveit.launch.py"
    ).read_text(encoding="utf-8")
    assert '"allow_trajectory_execution": "false"' in source
