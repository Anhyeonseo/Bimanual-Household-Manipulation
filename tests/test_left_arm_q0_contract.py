import json
import math
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MACRO_PATH = ROOT / "ros2_ws/src/so101_description/urdf/so101_arm_macro.xacro"
SRDF_PATH = ROOT / "ros2_ws/src/so101_moveit_config/config/so101_left.srdf"
CALIBRATION_PATH = ROOT / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json"
MAPPING_PATH = ROOT / "ros2_ws/src/so101_isaac_bridge/so101_isaac_bridge/mapping.py"
ISAAC_BASE_PATH = ROOT / "isaac_sim/assets/so101_new_calib/payloads/base.usda"
ISAAC_PHYSICS_PATH = (
    ROOT / "isaac_sim/assets/so101_new_calib/payloads/Physics/physics.usda"
)


UPSTREAM = {
    "shoulder_joint": {
        "rpy": (-1.5708, -1.5708, 0.0),
        "q_home": math.radians(90.0),
        "limits": (-1.74533, 1.74533),
    },
    "elbow_joint": {
        "rpy": (-3.63608e-16, 8.74301e-16, 1.5708),
        "q_home": math.radians(-55.0),
        "limits": (-1.69, 1.69),
    },
    "wrist_flex_joint": {
        "rpy": (4.02456e-15, 8.67362e-16, -1.5708),
        "q_home": math.radians(-64.898281239),
        "limits": (-1.65806, 1.65806),
    },
    "wrist_roll_joint": {
        "rpy": (1.5708, 0.0486795, 3.14159),
        "q_home": math.radians(-90.0),
        "limits": (-2.84121, 2.74385),
    },
}

EXPECTED_PROJECT_AXES = {
    "base_joint": (0.0, 0.0, 1.0),
    "shoulder_joint": (0.0, 0.0, 1.0),
    "elbow_joint": (0.0, 0.0, -1.0),
    "wrist_flex_joint": (0.0, 0.0, -1.0),
    "wrist_roll_joint": (0.0, 0.0, 1.0),
}


def rotation_from_rpy(rpy):
    roll, pitch, yaw = rpy
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, math.cos(roll), -math.sin(roll)], [0.0, math.sin(roll), math.cos(roll)]])
    ry = np.asarray([[math.cos(pitch), 0.0, math.sin(pitch)], [0.0, 1.0, 0.0], [-math.sin(pitch), 0.0, math.cos(pitch)]])
    rz = np.asarray([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def axis_minus_z_rotation(value):
    return rotation_from_rpy((0.0, 0.0, -value))


def rotation_from_quaternion_wxyz(quaternion):
    w, x, y, z = quaternion
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def parse_usd_tuple(block, attribute):
    match = re.search(rf"{re.escape(attribute)} = \(([^)]+)\)", block)
    if match is None:
        raise AssertionError(f"Missing USD attribute: {attribute}")
    return tuple(float(value.strip()) for value in match.group(1).split(","))


class LeftArmQ0ContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.macro = ET.parse(MACRO_PATH).getroot()
        cls.joints = {
            element.attrib["name"].replace("${prefix}", ""): element
            for element in cls.macro.findall(".//joint")
        }

    def test_new_q0_matches_visually_accepted_upstream_pose(self) -> None:
        for name, reference in UPSTREAM.items():
            with self.subTest(joint=name):
                new_rpy = tuple(float(value) for value in self.joints[name].find("origin").attrib["rpy"].split())
                expected = rotation_from_rpy(reference["rpy"]) @ axis_minus_z_rotation(reference["q_home"])
                np.testing.assert_allclose(rotation_from_rpy(new_rpy), expected, atol=2e-12)

    def test_joint_limits_are_shifted_into_new_coordinates(self) -> None:
        for name, reference in UPSTREAM.items():
            with self.subTest(joint=name):
                limit = self.joints[name].find("limit").attrib
                shifted_lower = reference["limits"][0] - reference["q_home"]
                shifted_upper = reference["limits"][1] - reference["q_home"]
                if EXPECTED_PROJECT_AXES[name][2] > 0.0:
                    expected_lower = -shifted_upper
                    expected_upper = -shifted_lower
                else:
                    expected_lower = shifted_lower
                    expected_upper = shifted_upper
                self.assertAlmostEqual(float(limit["lower"]), expected_lower)
                self.assertAlmostEqual(float(limit["upper"]), expected_upper)
                self.assertLessEqual(float(limit["lower"]), 0.0)
                self.assertGreaterEqual(float(limit["upper"]), 0.0)

    def test_joint_axes_match_external_rigid_target_metrology(self) -> None:
        for name, expected in EXPECTED_PROJECT_AXES.items():
            with self.subTest(joint=name):
                actual = tuple(
                    float(value)
                    for value in self.joints[name].find("axis").attrib["xyz"].split()
                )
                self.assertEqual(actual, expected)

    def test_srdf_home_remains_all_zero(self) -> None:
        root = ET.parse(SRDF_PATH).getroot()
        home = next(state for state in root.findall("group_state") if state.attrib["name"] == "home")
        self.assertEqual(len(home.findall("joint")), 5)
        self.assertTrue(all(float(joint.attrib["value"]) == 0.0 for joint in home))

    def test_hardware_raw_2048_zero_remains_unchanged(self) -> None:
        calibration = json.loads(CALIBRATION_PATH.read_text())
        arm = calibration["joints"][:5]
        self.assertEqual(len(arm), 5)
        self.assertTrue(all(joint["zero_raw"] == 2048 for joint in arm))

    def test_isaac_arm_mapping_keeps_zero_offsets(self) -> None:
        text = MAPPING_PATH.read_text()
        arm_block = text.split("JOINT_SPECS = (", 1)[1].split('JointSpec(\n        "left_gripper_joint"', 1)[0]
        entries = re.findall(r'JointSpec\(\s*"left_[^"]+_joint",\s*"[^"]+",\s*-1\.0,\s*0\.0,', arm_block)
        self.assertEqual(len(entries), 5)


    def test_isaac_wrist_flex_q0_anchor_and_limits_match_urdf(self) -> None:
        joint = self.joints["wrist_flex_joint"]
        origin = joint.find("origin")
        expected_xyz = tuple(float(value) for value in origin.attrib["xyz"].split())
        expected_rotation = rotation_from_rpy(
            tuple(float(value) for value in origin.attrib["rpy"].split())
        )
        limit = joint.find("limit").attrib

        base_text = ISAAC_BASE_PATH.read_text()
        base_block = base_text.split("def Xform \"wrist_link\"", 1)[1].split(
            "def Xform \"sts3215_03a_no_horn_v1\"", 1
        )[0]
        physics_text = ISAAC_PHYSICS_PATH.read_text()
        physics_block = physics_text.split(
            "def PhysicsRevoluteJoint \"wrist_flex\"", 1
        )[1].split("def PhysicsRevoluteJoint \"elbow_flex\"", 1)[0]

        base_xyz = parse_usd_tuple(base_block, "double3 xformOp:translate")
        base_quaternion = parse_usd_tuple(base_block, "quatf xformOp:orient")
        physics_xyz = parse_usd_tuple(
            physics_block,
            "point3f physics:localPos0",
        )
        physics_quaternion = parse_usd_tuple(
            physics_block,
            "quatf physics:localRot0",
        )

        np.testing.assert_allclose(base_xyz, expected_xyz, atol=2e-9)
        np.testing.assert_allclose(physics_xyz, expected_xyz, atol=2e-9)
        np.testing.assert_allclose(
            rotation_from_quaternion_wxyz(base_quaternion),
            expected_rotation,
            atol=1e-7,
        )
        np.testing.assert_allclose(
            rotation_from_quaternion_wxyz(physics_quaternion),
            expected_rotation,
            atol=1e-7,
        )

        usd_lower = float(
            re.search(
                r"float physics:lowerLimit = ([^\s]+)",
                physics_block,
            ).group(1)
        )
        usd_upper = float(
            re.search(
                r"float physics:upperLimit = ([^\s]+)",
                physics_block,
            ).group(1)
        )
        self.assertAlmostEqual(
            usd_lower,
            -math.degrees(float(limit["upper"])),
            places=4,
        )
        self.assertAlmostEqual(
            usd_upper,
            -math.degrees(float(limit["lower"])),
            places=4,
        )


if __name__ == "__main__":
    unittest.main()
