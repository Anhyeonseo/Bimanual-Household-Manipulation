import argparse
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2

import numpy as np

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "detect_top_object_pose.py"
SPEC = importlib.util.spec_from_file_location("detect_top_object_pose", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopObjectPoseTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.image_path = self.directory / "image.png"
        self.camera_path = self.directory / "camera.yaml"
        self.homography_path = self.directory / "homography.yaml"

        self.camera = {
            "image_width": 200,
            "image_height": 150,
            "camera_name": "test_top",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [
                    100.0,
                    0.0,
                    100.0,
                    0.0,
                    100.0,
                    75.0,
                    0.0,
                    0.0,
                    1.0,
                ],
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [
                    100.0,
                    0.0,
                    100.0,
                    0.0,
                    0.0,
                    100.0,
                    75.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                ],
            },
        }
        self._write_camera()
        self.homography = {
            "status": "BOARD_RELATIVE_VALID_BASE_REGISTRATION_REQUIRED",
            "motion_authorized": False,
            "camera": {
                "image_width": 200,
                "image_height": 150,
                "input_domain": "rectified_pixel_using_projection_matrix",
                "camera_info_sha256": MODULE.file_sha256(self.camera_path),
            },
            "homography": {
                "rectified_pixel_to_board_m": {
                    "rows": 3,
                    "cols": 3,
                    "data": [
                        0.001,
                        0.0,
                        0.0,
                        0.0,
                        0.001,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    ],
                },
            },
            "board": {
                "inner_corner_span_m": [0.18, 0.13],
            },
            "base_registration": {
                "status": "PROVISIONAL_RULER_MEASUREMENT",
                "motion_authorized": False,
            },
        }
        self._write_homography()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_camera(self) -> None:
        self.camera_path.write_text(
            yaml.safe_dump(self.camera, sort_keys=False),
            encoding="utf-8",
        )

    def _write_homography(self) -> None:
        self.homography_path.write_text(
            yaml.safe_dump(self.homography, sort_keys=False),
            encoding="utf-8",
        )

    def _arguments(self) -> argparse.Namespace:
        return argparse.Namespace(
            image=self.image_path,
            camera_info=self.camera_path,
            homography=self.homography_path,
            output=None,
            threshold=110,
            min_area_px=300.0,
            min_width_px=10,
            min_height_px=10,
            min_solidity=0.5,
        )

    def _write_image(self, rectangles: list[tuple[int, int, int, int]]) -> None:
        image = np.full((150, 200, 3), 240, dtype=np.uint8)
        for x0, y0, x1, y1 in rectangles:
            cv2.rectangle(image, (x0, y0), (x1, y1), (20, 20, 20), -1)
        self.assertTrue(cv2.imwrite(str(self.image_path), image))

    def test_exactly_one_object_reports_board_pose_fail_closed(self) -> None:
        self._write_image([(60, 50, 140, 90)])

        result = MODULE.detect(self._arguments())

        self.assertEqual(result["detected_count"], 1)
        self.assertEqual(result["frame_id"], "top_board")
        self.assertAlmostEqual(
            result["pose"]["board_position_m"][0],
            0.1,
            places=3,
        )
        self.assertAlmostEqual(
            result["pose"]["board_position_m"][1],
            0.07,
            places=3,
        )
        self.assertAlmostEqual(result["pose"]["size_m"][0], 0.08, places=3)
        self.assertAlmostEqual(result["pose"]["size_m"][1], 0.04, places=3)
        self.assertAlmostEqual(result["pose"]["yaw_rad"], 0.0, places=3)
        self.assertTrue(
            result["pose"]["calibration_region"]["footprint_inside"]
        )
        self.assertTrue(
            result["pose"]["calibration_region"]["image_fully_visible"]
        )
        self.assertFalse(result["pose"]["calibration_region"]["extrapolated"])
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["robot_target_available"])

    def test_zero_objects_is_rejected(self) -> None:
        self._write_image([])

        with self.assertRaisesRegex(RuntimeError, "detected 0"):
            MODULE.detect(self._arguments())

    def test_two_objects_are_rejected(self) -> None:
        self._write_image([(20, 20, 60, 60), (120, 80, 170, 130)])

        with self.assertRaisesRegex(RuntimeError, "detected 2"):
            MODULE.detect(self._arguments())

    def test_fully_outside_contour_does_not_pollute_object_count(self) -> None:
        self._write_image([(60, 50, 140, 90), (185, 10, 199, 50)])

        result = MODULE.detect(self._arguments())

        self.assertEqual(result["detected_count"], 1)
        self.assertEqual(
            result["pose"]["calibration_region"][
                "ignored_fully_outside_count"
            ],
            1,
        )

    def test_outside_center_intersecting_region_remains_blocking(self) -> None:
        self._write_image([(175, 50, 188, 90)])

        with self.assertRaisesRegex(RuntimeError, "center is outside"):
            MODULE.detect(self._arguments())

    def test_exclusion_rectangle_removes_fixed_robot_contour(self) -> None:
        self._write_image([(0, 110, 45, 149), (60, 50, 140, 90)])
        image = cv2.imread(str(self.image_path), cv2.IMREAD_COLOR)
        calibration = MODULE.shared_detector.load_calibration(
            self.camera_path,
            self.homography_path,
        )

        pose = MODULE.shared_detector.detect_one_object(
            image,
            calibration,
            MODULE.shared_detector.DetectorConfig(
                threshold=110,
                min_area_px=300.0,
                min_width_px=10,
                min_height_px=10,
                min_solidity=0.5,
                exclusion_rectangles_px=((0, 105, 50, 45),),
            ),
        )

        self.assertAlmostEqual(pose["board_position_m"][0], 0.1, places=3)
        self.assertAlmostEqual(pose["board_position_m"][1], 0.07, places=3)

    def test_camera_hash_mismatch_is_rejected(self) -> None:
        self._write_image([(60, 50, 140, 90)])
        self.homography["camera"]["camera_info_sha256"] = "wrong"
        self._write_homography()

        with self.assertRaisesRegex(RuntimeError, "hash"):
            MODULE.detect(self._arguments())

    def test_object_outside_calibrated_board_is_rejected(self) -> None:
        self._write_image([(150, 50, 185, 90)])

        with self.assertRaisesRegex(RuntimeError, "outside calibrated"):
            MODULE.detect(self._arguments())

    def test_partial_footprint_can_be_observed_but_not_authorized(self) -> None:
        self._write_image([(150, 50, 185, 90)])
        image = cv2.imread(str(self.image_path), cv2.IMREAD_COLOR)
        calibration = MODULE.shared_detector.load_calibration(
            self.camera_path,
            self.homography_path,
        )
        pose = MODULE.shared_detector.detect_one_object(
            image,
            calibration,
            MODULE.shared_detector.DetectorConfig(
                threshold=110,
                min_area_px=300.0,
                min_width_px=10,
                min_height_px=10,
                min_solidity=0.5,
            ),
            require_full_footprint=False,
        )

        self.assertTrue(pose["calibration_region"]["center_inside"])
        self.assertFalse(pose["calibration_region"]["footprint_inside"])
        self.assertTrue(pose["calibration_region"]["image_fully_visible"])
        self.assertTrue(pose["calibration_region"]["extrapolated"])

    def test_object_touching_image_margin_is_rejected(self) -> None:
        self._write_image([(2, 50, 45, 90)])
        image = cv2.imread(str(self.image_path), cv2.IMREAD_COLOR)
        calibration = MODULE.shared_detector.load_calibration(
            self.camera_path,
            self.homography_path,
        )
        with self.assertRaisesRegex(
            RuntimeError,
            "camera image safety margin",
        ):
            MODULE.shared_detector.detect_one_object(
                image,
                calibration,
                MODULE.shared_detector.DetectorConfig(
                    threshold=110,
                    min_area_px=300.0,
                    min_width_px=10,
                    min_height_px=10,
                    min_solidity=0.5,
                    image_edge_margin_px=8,
                ),
                require_full_footprint=False,
            )

    def test_json_result_is_serializable(self) -> None:
        self._write_image([(60, 50, 140, 90)])

        serialized = json.dumps(MODULE.detect(self._arguments()))

        self.assertIn("TOP_OBJECT_POSE_PASS", serialized)


if __name__ == "__main__":
    unittest.main()
