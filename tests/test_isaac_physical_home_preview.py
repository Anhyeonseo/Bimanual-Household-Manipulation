import importlib.util
import math
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path("tools/setup/isaac/isaac_preview_physical_home.py")
SPEC = importlib.util.spec_from_file_location(
    "isaac_preview_physical_home",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class IsaacPhysicalHomePreviewTest(unittest.TestCase):
    def test_preview_is_explicitly_simulation_only(self) -> None:
        self.assertTrue(MODULE.SIMULATION_ONLY)
        self.assertFalse(MODULE.MOTION_AUTHORIZED)

    def test_registration_pose_has_five_arm_joints_and_no_gripper(self) -> None:
        candidate = MODULE.upstream_registration_pose()
        self.assertEqual(len(candidate), 5)
        self.assertNotIn("left_gripper_joint", candidate)
        self.assertEqual(candidate["left_base_joint"], 0.0)
        self.assertAlmostEqual(
            candidate["left_shoulder_joint"],
            math.radians(90.0),
        )
        self.assertAlmostEqual(
            candidate["left_elbow_joint"],
            math.radians(-55.0),
        )
        self.assertAlmostEqual(
            candidate["left_wrist_flex_joint"],
            math.radians(-64.898281239),
        )
        self.assertAlmostEqual(
            candidate["left_wrist_roll_joint"],
            -math.pi / 2.0,
        )

    def test_project_candidate_is_mapped_to_isaac_names_and_signs(self) -> None:
        project = MODULE.upstream_registration_pose()
        isaac = MODULE.upstream_registration_pose_isaac()
        self.assertEqual(
            set(isaac),
            {
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            },
        )
        self.assertAlmostEqual(
            isaac["shoulder_lift"],
            -project["left_shoulder_joint"],
        )
        self.assertAlmostEqual(
            isaac["wrist_roll"],
            math.pi / 2.0,
        )

    def test_base_link_is_forced_to_front(self) -> None:
        paths = [
            "/robot/Geometry/base_link/shoulder_link/gripper_link",
            "/robot/Geometry/base_link/shoulder_link",
            "/robot/Geometry/base_link",
        ]
        ordered = MODULE.base_first_link_paths(paths)
        self.assertEqual(ordered[0], "/robot/Geometry/base_link")
        self.assertEqual(set(ordered), set(paths))

    def test_preview_usd_authors_base_link_first(self) -> None:
        robot_usda = Path(
            "isaac_sim/assets/so101_new_calib/payloads/robot.usda"
        ).read_text()
        link_block = robot_usda.split(
            "prepend rel isaac:physics:robotLinks = [", 1
        )[1].split("]", 1)[0]
        first_target = next(
            line.strip()
            for line in link_block.splitlines()
            if line.strip().startswith("</")
        )
        self.assertEqual(
            first_target,
            "</so101_new_calib/Geometry/base_link>,",
        )


if __name__ == "__main__":
    unittest.main()
