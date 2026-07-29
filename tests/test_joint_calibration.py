import math
from pathlib import Path
import re
import unittest

import yaml
from tools.joint_calibration import (
    CalibrationError,
    calibration_hash,
    load_calibration,
    raw_to_urad,
    urad_to_raw,
)

class JointCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.calibration = load_calibration(
            Path("config/single_arm_calibration.json")
        )

    def test_hash_matches_verified_firmware(self) -> None:
        self.assertEqual(calibration_hash(self.calibration), 0x4D62F8D5)

    def test_operational_raw_ranges_match_observed_envelope_with_margin(self) -> None:
        expected = {
            "BASE": (2048, 2610),
            "SHOULDER": (1988, 3766),
            "ELBOW": (627, 2258),
            "WRIST_FLEX": (1194, 2048),
            "WRIST_ROLL": (1874, 2219),
            "GRIPPER": (1866, 2048),
        }
        self.assertEqual(
            {
                joint["name"]: (
                    joint["minimum_raw"],
                    joint["maximum_raw"],
                )
                for joint in self.calibration["joints"]
            },
            expected,
        )

    def test_host_firmware_and_moveit_ranges_stay_in_sync(self) -> None:
        bridge = load_calibration(
            Path("ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json")
        )
        authoritative_ranges = [
            (joint["id"], joint["name"], joint["zero_raw"],
             joint["minimum_raw"], joint["maximum_raw"],
             joint["p_gain"], joint["positive_raw_direction"])
            for joint in self.calibration["joints"]
        ]
        bridge_ranges = [
            (joint["id"], joint["name"], joint["zero_raw"],
             joint["minimum_raw"], joint["maximum_raw"],
             joint["p_gain"], joint["positive_raw_direction"])
            for joint in bridge["joints"]
        ]
        self.assertEqual(bridge_ranges, authoritative_ranges)

        firmware = Path(
            "firmware/stm32_g474_single_arm/Core/Src/servo_bus.c"
        ).read_text()
        firmware_ranges = [
            (int(servo_id), name, int(zero), int(minimum), int(maximum),
             int(p_gain), int(direction))
            for servo_id, name, zero, minimum, maximum, p_gain, direction in
            re.findall(
                r"\{(\d+)U,\s*\"([^\"]+)\",\s*1U,\s*"
                r"(\d+)U,\s*(\d+)U,\s*(\d+)U,\s*(\d+)U,\s*(-?1),",
                firmware,
            )
        ]
        self.assertEqual(firmware_ranges, authoritative_ranges)

        moveit = yaml.safe_load(
            Path("ros2_ws/src/so101_moveit_config/config/joint_limits.yaml")
            .read_text()
        )["joint_limits"]
        scale = 2.0 * math.pi / 4096.0
        for joint in self.calibration["joints"]:
            name = f"left_{joint['name'].lower()}_joint"
            endpoints = [
                (raw - joint["zero_raw"])
                * joint["positive_raw_direction"]
                * scale
                for raw in (joint["minimum_raw"], joint["maximum_raw"])
            ]
            self.assertTrue(moveit[name]["has_position_limits"])
            self.assertAlmostEqual(moveit[name]["min_position"], min(endpoints))
            self.assertAlmostEqual(moveit[name]["max_position"], max(endpoints))

    def test_every_joint_zero_maps_to_raw_2048(self) -> None:
        for joint_index in range(6):
            self.assertEqual(urad_to_raw(self.calibration, joint_index, 0), 2048)

    def test_verified_positive_directions_map_to_34_raw_steps(self) -> None:
        expected_raw = [2082, 2082, 2014, 2014, 2082, 2014]
        position_urad = raw_to_urad(self.calibration, 0, 2082)
        for joint_index, raw in enumerate(expected_raw):
            self.assertEqual(
                urad_to_raw(self.calibration, joint_index, position_urad),
                raw,
            )

    def test_raw_round_trip_covers_every_safe_endpoint(self) -> None:
        for joint_index, joint in enumerate(self.calibration["joints"]):
            for raw in (joint["minimum_raw"], joint["zero_raw"], joint["maximum_raw"]):
                urad = raw_to_urad(self.calibration, joint_index, raw)
                self.assertEqual(urad_to_raw(self.calibration, joint_index, urad), raw)

    def test_out_of_range_joint_target_is_rejected(self) -> None:
        with self.assertRaises(CalibrationError):
            urad_to_raw(self.calibration, 2, 3_000_000)

if __name__ == "__main__":
    unittest.main()
