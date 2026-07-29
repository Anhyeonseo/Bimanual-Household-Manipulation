import importlib.util
import sys
import unittest
from pathlib import Path


BRINGUP_LAUNCH = Path(
    "ros2_ws/src/so101_bringup/launch/external_stm32_moveit.launch.py"
)
MOVE_GROUP_LAUNCH = Path(
    "ros2_ws/src/so101_moveit_config/launch/external_move_group.launch.py"
)
TOOLS_ROOT = Path("tools")
sys.path.insert(0, str(TOOLS_ROOT))

launch_spec = importlib.util.spec_from_file_location(
    "external_stm32_moveit_launch",
    BRINGUP_LAUNCH,
)
launch_module = importlib.util.module_from_spec(launch_spec)
assert launch_spec.loader is not None
launch_spec.loader.exec_module(launch_module)

move_group_spec = importlib.util.spec_from_file_location(
    "external_move_group_launch",
    MOVE_GROUP_LAUNCH,
)
move_group_module = importlib.util.module_from_spec(move_group_spec)
assert move_group_spec.loader is not None
move_group_spec.loader.exec_module(move_group_module)

tool_spec = importlib.util.spec_from_file_location(
    "ros_moveit_execute_once",
    TOOLS_ROOT / "ros_moveit_execute_once.py",
)
tool_module = importlib.util.module_from_spec(tool_spec)
assert tool_spec.loader is not None
sys.modules[tool_spec.name] = tool_module
tool_spec.loader.exec_module(tool_module)


class ExternalMoveItLaunchTests(unittest.TestCase):
    def test_external_launch_starts_only_four_moveit_includes(self) -> None:
        actions = launch_module._moveit_actions()
        self.assertEqual(len(actions), 4)
        self.assertTrue(
            all(
                action.__class__.__name__ == "IncludeLaunchDescription"
                for action in actions
            )
        )

    def test_external_launch_has_no_backend_or_motion_argument(self) -> None:
        description = launch_module.generate_launch_description()
        names = {
            entity.name
            for entity in description.entities
            if hasattr(entity, "name") and entity.name
        }
        self.assertNotIn("backend", names)
        self.assertNotIn("allow_motion", names)
        self.assertIn("use_rviz", names)

    def test_rviz_include_is_conditioned_by_use_rviz(self) -> None:
        actions = launch_module._moveit_actions()
        self.assertIsNone(actions[0].condition)
        self.assertIsNone(actions[1].condition)
        self.assertIsNone(actions[2].condition)
        self.assertIsNotNone(actions[3].condition)

    def test_external_move_group_uses_bounded_registration_tolerance(self) -> None:
        config = move_group_module._moveit_config()
        tolerance = config.trajectory_execution[
            "trajectory_execution"
        ]["allowed_start_tolerance"]
        self.assertEqual(tolerance, 0.45)
        self.assertGreater(
            tolerance,
            tool_module.PRESETS["register-base-040"].positions[0],
        )
        self.assertLess(tolerance, 0.50)
        execution = config.trajectory_execution["trajectory_execution"]
        self.assertEqual(
            execution["allowed_execution_duration_scaling"],
            1.2,
        )
        self.assertEqual(
            execution["allowed_goal_duration_margin"],
            1.0,
        )


class MoveItExecuteOnceTests(unittest.TestCase):
    def test_presets_are_fixed_single_point_safe_contracts(self) -> None:
        self.assertEqual(
            tuple(tool_module.PRESETS),
            (
                "home",
                "representative",
                "visible",
                "register-base-002",
                "register-base-006",
                "register-base-010",
                "register-base-020",
                "register-base-030",
                "register-base-035",
                "register-base-040",
                "register-pose-03",
                "register-pose-04",
                "register-pose-05",
                "register-pose-05b",
                "register-pose-05c",
                "register-pose-05d",
                "register-pose-05e",
                "diagnose-shoulder-low",
                "gripper-safe",
            ),
        )
        representative = tool_module.PRESETS["representative"]
        self.assertEqual(representative.positions, (0.05,) * 5)
        visible = tool_module.PRESETS["visible"]
        self.assertEqual(visible.positions, (0.10,) * 5)
        gripper = tool_module.PRESETS["gripper-safe"]
        self.assertEqual(gripper.positions, (0.08,))
        self.assertEqual(
            tool_module.PRESETS["register-base-002"].positions,
            (0.02, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-006"].positions,
            (0.06, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-010"].positions,
            (0.10, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-020"].positions,
            (0.20, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-030"].positions,
            (0.30, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-035"].positions,
            (0.35, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-base-040"].positions,
            (0.40, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-03"].positions,
            (0.45, 0.10, 0.05, 0.05, 0.05),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-03"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-04"].positions,
            (0.47, 0.14, 0.04, 0.14, 0.08),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-04"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05"].positions,
            (0.48, 0.14, 0.14, 0.04, 0.16),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05b"].positions,
            (0.49, 0.04, 0.10, 0.04, 0.16),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05b"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05c"].positions,
            (0.49, 0.04, 0.10, 0.04, 0.08),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05c"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05d"].positions,
            (0.45, 0.08, 0.12, 0.12, 0.08),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05d"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05e"].positions,
            (0.40, 0.14, 0.04, 0.14, 0.08),
        )
        self.assertEqual(
            tool_module.PRESETS["register-pose-05e"].duration_s,
            2,
        )
        self.assertEqual(
            tool_module.PRESETS["diagnose-shoulder-low"].positions,
            (0.47, 0.05, 0.04, 0.14, 0.08),
        )
        self.assertEqual(
            tool_module.PRESETS["diagnose-shoulder-low"].duration_s,
            2,
        )

    def test_every_preset_passes_hardware_calibration_preflight(self) -> None:
        for preset in tool_module.PRESETS.values():
            tool_module.validate_preset_against_hardware_calibration(preset)

    def test_duration_outside_bridge_contract_fails_preflight(self) -> None:
        invalid = tool_module.Preset(
            tool_module.ARM_CONTROLLER,
            tool_module.ARM_JOINTS,
            (0.45, 0.10, 0.05, 0.05, 0.05),
            3,
        )
        with self.assertRaisesRegex(ValueError, "300..2000 ms"):
            tool_module.validate_preset_against_hardware_calibration(invalid)

    def test_negative_elbow_fails_hardware_calibration_preflight(self) -> None:
        invalid = tool_module.Preset(
            tool_module.ARM_CONTROLLER,
            tool_module.ARM_JOINTS,
            (0.40, 0.0, -0.35, 0.0, 0.0),
            2,
        )
        with self.assertRaisesRegex(ValueError, "left_elbow_joint"):
            tool_module.validate_preset_against_hardware_calibration(invalid)

    def test_goal_contains_exactly_one_point_and_one_controller(self) -> None:
        preset = tool_module.PRESETS["representative"]
        goal = tool_module.build_goal(preset)
        self.assertEqual(goal.controller_names, [preset.controller])
        trajectory = goal.trajectory.joint_trajectory
        self.assertEqual(trajectory.joint_names, list(preset.joint_names))
        self.assertEqual(len(trajectory.points), 1)
        self.assertEqual(
            tuple(trajectory.points[0].positions),
            preset.positions,
        )
        self.assertEqual(
            trajectory.points[0].time_from_start.sec,
            preset.duration_s,
        )


if __name__ == "__main__":
    unittest.main()
