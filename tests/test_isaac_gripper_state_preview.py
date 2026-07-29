import importlib.util
import math
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path("tools/isaac_preview_gripper_state.py")
SPEC = importlib.util.spec_from_file_location(
    "isaac_preview_gripper_state",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class IsaacGripperStatePreviewTest(unittest.TestCase):
    def test_preview_is_explicitly_simulation_only(self) -> None:
        self.assertTrue(MODULE.SIMULATION_ONLY)
        self.assertFalse(MODULE.MOTION_AUTHORIZED)

    def test_closed_project_q0_maps_to_isaac_minus_ten_degrees(self) -> None:
        self.assertAlmostEqual(
            MODULE.requested_isaac_position("closed"),
            math.radians(-10.0),
        )

    def test_open_maps_to_isaac_one_hundred_degrees(self) -> None:
        self.assertAlmostEqual(
            MODULE.requested_isaac_position("open"),
            math.radians(100.0),
            places=5,
        )

    def test_unknown_state_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            MODULE.requested_isaac_position("toggle")


if __name__ == "__main__":
    unittest.main()
