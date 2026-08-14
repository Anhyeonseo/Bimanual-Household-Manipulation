import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.motion_goal_arbiter import MotionGoalArbiter  # noqa: E402


class MotionGoalArbiterTests(unittest.TestCase):
    def test_arm_owner_blocks_gripper(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("arm"))
        self.assertFalse(arbiter.try_reserve("gripper"))
        self.assertEqual(arbiter.owner, "arm")

    def test_gripper_owner_blocks_arm(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("gripper"))
        self.assertFalse(arbiter.try_reserve("arm"))
        self.assertEqual(arbiter.owner, "gripper")

    def test_diagnostics_owner_blocks_motion_goals(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("diagnostics"))
        self.assertFalse(arbiter.try_reserve("arm"))
        self.assertFalse(arbiter.try_reserve("gripper"))
        self.assertTrue(arbiter.release("diagnostics"))
        self.assertIsNone(arbiter.owner)

    def test_right_arm_read_only_discovery_blocks_motion_goals(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("right_arm_discovery"))
        self.assertFalse(arbiter.try_reserve("arm"))
        self.assertFalse(arbiter.try_reserve("gripper"))
        self.assertTrue(arbiter.release("right_arm_discovery"))

    def test_bimanual_read_only_feedback_has_exclusive_bus_ownership(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("bimanual_read_only_feedback"))
        self.assertFalse(arbiter.try_reserve("right_arm_discovery"))
        self.assertFalse(arbiter.try_reserve("arm"))
        self.assertTrue(arbiter.release("bimanual_read_only_feedback"))
        self.assertIsNone(arbiter.owner)

    def test_right_arm_torque_enable_has_exclusive_bus_ownership(self) -> None:
        arbiter = MotionGoalArbiter()

        self.assertTrue(arbiter.try_reserve("right_arm_torque_enable"))
        self.assertFalse(arbiter.try_reserve("right_arm_jog"))
        self.assertFalse(arbiter.try_reserve("arm"))
        self.assertTrue(arbiter.release("right_arm_torque_enable"))
        self.assertIsNone(arbiter.owner)

    def test_only_current_owner_can_release(self) -> None:
        arbiter = MotionGoalArbiter()
        arbiter.try_reserve("arm")

        self.assertFalse(arbiter.release("gripper"))
        self.assertEqual(arbiter.owner, "arm")
        self.assertTrue(arbiter.release("arm"))
        self.assertIsNone(arbiter.owner)

    def test_unknown_owner_is_rejected(self) -> None:
        arbiter = MotionGoalArbiter()

        with self.assertRaisesRegex(ValueError, "invalid motion owner"):
            arbiter.try_reserve("unknown")


if __name__ == "__main__":
    unittest.main()
