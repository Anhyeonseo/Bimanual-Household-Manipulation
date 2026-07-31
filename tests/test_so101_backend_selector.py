import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))
LAUNCH_FILE = Path(
    "ros2_ws/src/so101_bringup/launch/so101_moveit.launch.py"
)

spec = importlib.util.spec_from_file_location(
    "so101_moveit_launch",
    LAUNCH_FILE,
)
launch_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(launch_module)


class BackendSelectorTests(unittest.TestCase):
    def test_exact_supported_backend_is_normalized(self) -> None:
        for value, expected in (
            ("mock", "mock"),
            (" ISAAC ", "isaac"),
            ("STM32", "stm32"),
        ):
            with self.subTest(value=value):
                self.assertEqual(
                    launch_module._validate_backend(value),
                    expected,
                )

    def test_empty_unknown_or_multiple_backend_is_rejected(self) -> None:
        for value in ("", "hardware", "mock,isaac", "mock isaac"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "expected exactly one",
                ):
                    launch_module._validate_backend(value)

    def test_each_backend_builds_exactly_one_provider(self) -> None:
        with patch.object(
            launch_module,
            "_stm32_parameters",
            return_value=[],
        ):
            mock_actions = launch_module._backend_actions(
                "mock",
                "false",
                123,
            )
            isaac_actions = launch_module._backend_actions(
                "isaac",
                "false",
                123,
            )
            stm32_actions = launch_module._backend_actions(
                "stm32",
                "false",
                123,
            )

        self.assertEqual(len(mock_actions), 1)
        self.assertEqual(len(isaac_actions), 1)
        self.assertEqual(len(stm32_actions), 1)
        self.assertEqual(
            isaac_actions[0].node_package,
            "so101_isaac_bridge",
        )
        self.assertEqual(
            stm32_actions[0].node_package,
            "single_arm_bridge",
        )

    def test_stm32_receives_only_validated_lease_owner_pid(self) -> None:
        with patch.object(
            launch_module,
            "_stm32_parameters",
            return_value=[],
        ):
            action = launch_module._backend_actions(
                "stm32",
                "false",
                4321,
            )[0]

        environment = {
            key[0].text: value[0].text
            for key, value in action.additional_env
        }
        self.assertEqual(
            environment["SO101_BACKEND_LEASE_OWNER_PID"],
            "4321",
        )

    def test_motion_default_remains_read_only(self) -> None:
        description = launch_module.generate_launch_description()
        arguments = {
            entity.name: entity.default_value[0].text
            for entity in description.entities
            if hasattr(entity, "name") and entity.name
        }
        self.assertEqual(arguments["backend"], "mock")
        self.assertEqual(arguments["allow_motion"], "false")
        self.assertEqual(arguments["use_rviz"], "true")

    def test_rviz_can_be_disabled_for_headless_validation(self) -> None:
        actions = launch_module._common_actions("mock", "false")

        rviz_action = actions[-1]
        self.assertIsNotNone(rviz_action.condition)


if __name__ == "__main__":
    unittest.main()
