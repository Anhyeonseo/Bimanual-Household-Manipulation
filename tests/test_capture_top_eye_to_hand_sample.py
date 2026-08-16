import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


TOOLS = Path(__file__).resolve().parents[1] / "tools"
SPEC = importlib.util.spec_from_file_location(
    "capture_top_eye_to_hand_sample",
    TOOLS / "capture_top_eye_to_hand_sample.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CaptureTopEyeToHandSampleTest(unittest.TestCase):
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

    def test_right_joint_positions_are_reordered_by_contract(self):
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

    def test_generated_gridboard_is_recognized_only_when_complete(self):
        generator_spec = importlib.util.spec_from_file_location(
            "generate_top_eye_to_hand_gridboard_for_capture_test",
            TOOLS / "generate_top_eye_to_hand_gridboard.py",
        )
        generator = importlib.util.module_from_spec(generator_spec)
        assert generator_spec.loader is not None
        generator_spec.loader.exec_module(generator)
        board = generator.draw_board(1200)
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
        occluded[:750, :750] = np.full((750, 750, 3), 255, dtype=np.uint8)
        self.assertNotEqual(
            MODULE.detect_expected_gridboard(occluded),
            MODULE.EXPECTED_MARKER_IDS,
        )

    def test_camera_model_and_visual_pose_span_are_metric(self):
        camera_info = (
            Path(__file__).resolve().parents[1]
            / "ros2_ws/src/manipulation_camera_manager/config/"
            "top_camera_info.yaml"
        )
        camera_matrix, distortion = MODULE.load_camera_model(camera_info)
        self.assertEqual(camera_matrix.shape, (3, 3))
        self.assertEqual(distortion.shape, (5,))

        half_degree = np.deg2rad(0.5)
        rotations = [
            np.eye(3),
            np.asarray(
                [
                    [np.cos(half_degree), -np.sin(half_degree), 0.0],
                    [np.sin(half_degree), np.cos(half_degree), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            ),
        ]
        translation_span, rotation_span = MODULE.maximum_target_pose_span(
            [np.zeros(3), np.asarray([0.002, 0.0, 0.0])],
            rotations,
        )
        self.assertAlmostEqual(translation_span, 0.002)
        self.assertAlmostEqual(np.rad2deg(rotation_span), 0.5)

    def test_visual_stability_defaults_are_fail_closed(self):
        source = (TOOLS / "capture_top_eye_to_hand_sample.py").read_text()
        self.assertIn('"--max-pnp-rms-px", type=float, default=1.5', source)
        self.assertIn('"--max-target-translation-span-mm"', source)
        self.assertIn('"--max-target-rotation-span-deg"', source)
        self.assertIn("calibration target moved during capture", source)

    def test_capture_window_spans_exposes_one_visual_outlier(self):
        joints = [np.zeros(5) for _ in range(5)]
        translations = [np.zeros(3) for _ in range(4)] + [
            np.asarray([0.003, 0.0, 0.0])
        ]
        rotations = [np.eye(3) for _ in range(5)]
        span, maximum, translation_mm, rotation_deg = (
            MODULE.capture_window_spans(joints, translations, rotations)
        )
        np.testing.assert_allclose(span, 0.0)
        self.assertEqual(maximum, 0.0)
        self.assertAlmostEqual(translation_mm, 3.0)
        self.assertAlmostEqual(rotation_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
