import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/camera_calibration"
SPEC = importlib.util.spec_from_file_location(
    "capture_wrist_eye_in_hand_sample",
    TOOLS / "capture_wrist_eye_in_hand_sample.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CaptureWristEyeInHandSampleTest(unittest.TestCase):
    def test_rgb_image_decoding_respects_row_step(self):
        message = SimpleNamespace(
            width=2,
            height=1,
            encoding="rgb8",
            step=8,
            data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
        )
        image = MODULE.decode_image(message)
        self.assertEqual(image.shape, (1, 2, 3))
        self.assertEqual(image[0, 0].tolist(), [0, 0, 255])
        self.assertEqual(image[0, 1].tolist(), [0, 255, 0])

    def test_joint_positions_are_reordered_by_contract(self):
        message = SimpleNamespace(
            name=[
                "left_wrist_roll_joint",
                "left_elbow_joint",
                "left_base_joint",
                "left_wrist_flex_joint",
                "left_shoulder_joint",
            ],
            position=[5.0, 3.0, 1.0, 4.0, 2.0],
        )
        positions = MODULE.ordered_arm_positions(message)
        self.assertEqual(positions.tolist(), [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_right_joint_positions_use_the_right_arm_contract(self):
        message = SimpleNamespace(
            name=[
                "right_wrist_roll_joint",
                "right_elbow_joint",
                "right_base_joint",
                "right_wrist_flex_joint",
                "right_shoulder_joint",
            ],
            position=[5.0, 3.0, 1.0, 4.0, 2.0],
        )
        positions = MODULE.ordered_arm_positions(message, "right")
        self.assertEqual(positions.tolist(), [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_expected_marker_ids_are_the_planar_gridboard_ids_10_to_29(self):
        self.assertEqual(MODULE.EXPECTED_MARKER_IDS, tuple(range(10, 30)))

    def test_right_capture_is_fail_closed_on_resident_hold_proof(self):
        source = (
            TOOLS / "capture_wrist_eye_in_hand_sample.py"
        ).read_text()
        self.assertIn(
            "WRIST_EYE_IN_HAND_RESIDENT_TORQUE_HOLD_CAPTURE_PASS",
            source,
        )
        self.assertIn("--resident-required-owner", source)
        self.assertIn("--resident-required-epoch", source)
        self.assertIn("resident_terminal_measured_anchor", source)

    def test_generated_gridboard_is_recognized_only_when_complete(self):
        generator_spec = importlib.util.spec_from_file_location(
            "generate_planar_aruco_gridboard_for_capture_test",
            TOOLS / "generate_planar_aruco_gridboard.py",
        )
        generator = importlib.util.module_from_spec(generator_spec)
        assert generator_spec.loader is not None
        generator_spec.loader.exec_module(generator)
        board = generator.draw_board(1900)
        image = cv2.cvtColor(
            cv2.copyMakeBorder(
                board,
                150,
                150,
                150,
                150,
                cv2.BORDER_CONSTANT,
                value=255,
            ),
            cv2.COLOR_GRAY2BGR,
        )
        self.assertEqual(
            MODULE.detect_expected_gridboard(image),
            MODULE.EXPECTED_MARKER_IDS,
        )
        occluded = image.copy()
        half_height = occluded.shape[0] // 2
        occluded[:half_height, :] = 255
        self.assertNotEqual(
            MODULE.detect_expected_gridboard(occluded),
            MODULE.EXPECTED_MARKER_IDS,
        )


if __name__ == "__main__":
    unittest.main()
