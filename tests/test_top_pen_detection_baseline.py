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
MODULE_PATH = ROOT / "tools" / "evaluate_top_pen_detection_baseline.py"
SPEC = importlib.util.spec_from_file_location(
    "evaluate_top_pen_detection_baseline",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopPenDetectionBaselineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.camera_path = self.directory / "camera.yaml"
        self.homography_path = self.directory / "homography.yaml"
        self.contract_path = self.directory / "contract.json"
        self.manifest_path = self.directory / "manifest.json"
        self._write_geometry()
        self._write_contract()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_geometry(self) -> None:
        camera = {
            "image_width": 200,
            "image_height": 150,
            "camera_name": "test_top",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [
                    100.0, 0.0, 100.0,
                    0.0, 100.0, 75.0,
                    0.0, 0.0, 1.0,
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
                    100.0, 0.0, 100.0, 0.0,
                    0.0, 100.0, 75.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                ],
            },
        }
        self.camera_path.write_text(
            yaml.safe_dump(camera, sort_keys=False),
            encoding="utf-8",
        )
        homography = {
            "status": "TEST_ONLY",
            "motion_authorized": False,
            "camera": {
                "image_width": 200,
                "image_height": 150,
                "input_domain": "rectified_pixel_using_projection_matrix",
                "camera_info_sha256": MODULE.shared_detector.file_sha256(
                    self.camera_path
                ),
            },
            "homography": {
                "rectified_pixel_to_board_m": {
                    "rows": 3,
                    "cols": 3,
                    "data": [
                        0.001, 0.0, 0.0,
                        0.0, 0.001, 0.0,
                        0.0, 0.0, 1.0,
                    ],
                },
            },
            "board": {"calibrated_span_m": [0.2, 0.15]},
            "base_registration": {
                "status": "TEST_ONLY",
                "motion_authorized": False,
            },
        }
        self.homography_path.write_text(
            yaml.safe_dump(homography, sort_keys=False),
            encoding="utf-8",
        )

    def _write_contract(self) -> None:
        contract = {
            "protocol_version": 1,
            "detector": {
                "threshold": 110,
                "min_area_px": 300.0,
                "min_width_px": 10,
                "min_height_px": 10,
                "min_solidity": 0.5,
                "image_edge_margin_px": 2,
                "exclusion_rectangles_px": [],
                "require_full_footprint": True,
            },
            "coverage": {
                "minimum_positive_cases": 2,
                "minimum_negative_cases": 1,
                "minimum_background_labels": 2,
                "minimum_lighting_labels": 2,
                "minimum_glare_labels": 2,
            },
            "acceptance": {
                "maximum_miss_rate": 0.0,
                "maximum_false_positive_rate": 0.0,
                "maximum_center_error_px_p95": 1.0,
                "maximum_yaw_error_deg_p95": 1.0,
            },
        }
        self.contract_path.write_text(
            json.dumps(contract),
            encoding="utf-8",
        )

    def _image(
        self,
        name: str,
        background: int,
        rectangle: tuple[int, int, int, int] | None,
    ) -> Path:
        path = self.directory / name
        image = np.full((150, 200, 3), background, dtype=np.uint8)
        if rectangle is not None:
            x0, y0, x1, y1 = rectangle
            cv2.rectangle(image, (x0, y0), (x1, y1), (20, 20, 20), -1)
        self.assertTrue(cv2.imwrite(str(path), image))
        return path

    def _case(
        self,
        identifier: str,
        image: Path,
        expected_present: bool,
        background: str,
        lighting: str,
        glare: str,
    ) -> dict:
        result = {
            "id": identifier,
            "image": image.name,
            "image_sha256": MODULE.shared_detector.file_sha256(image),
            "expected_present": expected_present,
            "condition": {
                "background": background,
                "lighting": lighting,
                "glare": glare,
            },
        }
        if expected_present:
            result["expected_center_px"] = [100.0, 70.0]
            result["expected_yaw_deg"] = 0.0
        return result

    def _write_manifest(self, cases: list[dict]) -> None:
        manifest = {
            "protocol_version": 1,
            "geometry": {
                "camera_info_sha256": MODULE.shared_detector.file_sha256(
                    self.camera_path
                ),
                "homography_sha256": MODULE.shared_detector.file_sha256(
                    self.homography_path
                ),
            },
            "cases": cases,
        }
        self.manifest_path.write_text(
            json.dumps(manifest),
            encoding="utf-8",
        )

    def _passing_cases(self) -> list[dict]:
        first = self._image("positive_a.png", 240, (60, 50, 140, 90))
        second = self._image("positive_b.png", 210, (60, 50, 140, 90))
        negative = self._image("negative.png", 225, None)
        return [
            self._case("positive-a", first, True, "marble", "normal", "none"),
            self._case("positive-b", second, True, "wood", "bright", "present"),
            self._case("negative", negative, False, "marble", "normal", "none"),
        ]

    def test_complete_dataset_passes_without_motion_authority(self) -> None:
        self._write_manifest(self._passing_cases())

        result = MODULE.evaluate(
            self.manifest_path,
            self.contract_path,
            self.camera_path,
            self.homography_path,
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "TOP_PEN_DETECTION_BASELINE_PASS")
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["robot_command_topics_created"], 0)
        self.assertEqual(result["metrics"]["misses"], 0)
        self.assertEqual(result["metrics"]["false_positives"], 0)

    def test_dark_shape_in_negative_case_fails_as_false_positive(self) -> None:
        cases = self._passing_cases()
        hard_negative = self._image(
            "hard_negative.png",
            240,
            (60, 50, 140, 90),
        )
        cases[2] = self._case(
            "negative",
            hard_negative,
            False,
            "marble",
            "normal",
            "none",
        )
        self._write_manifest(cases)

        result = MODULE.evaluate(
            self.manifest_path,
            self.contract_path,
            self.camera_path,
            self.homography_path,
        )

        self.assertFalse(result["passed"])
        self.assertEqual(result["metrics"]["false_positives"], 1)
        self.assertIn("false_positive_rate_exceeded", result["failures"])
    def test_negative_safety_rejection_is_not_processing_error(self) -> None:
        self._write_manifest(self._passing_cases())
        config = MODULE.detector_config(
            MODULE.load_json(self.contract_path)
        )

        def runner(image, calibration, require_full_footprint):
            if int(np.min(image)) < 100:
                return MODULE.shared_detector.detect_one_object(
                    image,
                    calibration,
                    config,
                    require_full_footprint=require_full_footprint,
                )
            raise MODULE.shared_detector.DetectionError(
                "IMAGE_FOOTPRINT_CLIPPED",
                "test candidate rejected at image margin",
            )

        result = MODULE.evaluate(
            self.manifest_path,
            self.contract_path,
            self.camera_path,
            self.homography_path,
            detector_runner=runner,
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["metrics"]["false_positives"], 0)
        self.assertEqual(result["metrics"]["processing_errors"], 0)


    def test_incomplete_environment_coverage_is_reported(self) -> None:
        cases = self._passing_cases()
        for case in cases:
            case["condition"] = {
                "background": "marble",
                "lighting": "normal",
                "glare": "none",
            }
        self._write_manifest(cases)

        result = MODULE.evaluate(
            self.manifest_path,
            self.contract_path,
            self.camera_path,
            self.homography_path,
        )

        self.assertFalse(result["passed"])
        self.assertIn("insufficient_background_coverage", result["failures"])
        self.assertIn("insufficient_lighting_coverage", result["failures"])
        self.assertIn("insufficient_glare_coverage", result["failures"])

    def test_image_hash_mismatch_is_rejected(self) -> None:
        cases = self._passing_cases()
        cases[0]["image_sha256"] = "wrong"
        self._write_manifest(cases)

        with self.assertRaisesRegex(ValueError, "image_sha256 mismatch"):
            MODULE.evaluate(
                self.manifest_path,
                self.contract_path,
                self.camera_path,
                self.homography_path,
            )

    def test_absolute_image_path_is_rejected(self) -> None:
        cases = self._passing_cases()
        cases[0]["image"] = str(
            (self.directory / cases[0]["image"]).resolve()
        )
        self._write_manifest(cases)

        with self.assertRaisesRegex(ValueError, "relative to the manifest"):
            MODULE.evaluate(
                self.manifest_path,
                self.contract_path,
                self.camera_path,
                self.homography_path,
            )

    def test_parent_traversal_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "inside the dataset"):
            MODULE.resolve_dataset_image(
                self.manifest_path,
                "../outside.png",
            )

    def test_long_axis_yaw_error_is_modulo_180_degrees(self) -> None:
        self.assertAlmostEqual(
            MODULE.undirected_yaw_error_deg(89.0, -89.0),
            2.0,
        )

    def test_image_yaw_is_transformed_into_board_frame(self) -> None:
        calibration = MODULE.shared_detector.load_calibration(
            self.camera_path,
            self.homography_path,
        )
        rotated = MODULE.shared_detector.Calibration(
            image_width=calibration.image_width,
            image_height=calibration.image_height,
            camera_matrix=calibration.camera_matrix,
            distortion=calibration.distortion,
            projection=calibration.projection,
            pixel_to_board=np.asarray(
                [
                    [0.0, -0.001, 0.0],
                    [0.001, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            board_span=calibration.board_span,
            camera_info_sha256=calibration.camera_info_sha256,
            homography_status=calibration.homography_status,
            base_registration_status=calibration.base_registration_status,
            motion_authorized=calibration.motion_authorized,
        )

        transformed = MODULE.image_axis_yaw_to_board_deg(
            [100.0, 70.0],
            0.0,
            rotated,
        )

        self.assertAlmostEqual(transformed, 90.0)

if __name__ == "__main__":
    unittest.main()
