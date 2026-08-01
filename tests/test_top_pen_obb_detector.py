import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws" / "src" / "so101_top_perception"
import sys

if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from so101_top_perception.detector import Calibration, DetectionError
from so101_top_perception.obb_detector import (
    BACKEND_NAME,
    OUTPUT_LAYOUT,
    YAW_SEMANTICS,
    decode_ultralytics_obb,
    letterbox,
    load_runtime_config,
    select_one_pose,
)


class TopPenObbDetectorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.model_path = self.directory / "pen.onnx"
        self.model_path.write_bytes(b"test-only-onnx-placeholder")
        self.holdout_hash = "a" * 64
        self.bundle_path = self.directory / "bundle.json"
        self._write_bundle()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _bundle(self) -> dict:
        return {
            "protocol_version": 1,
            "backend": BACKEND_NAME,
            "task": "obb",
            "model": {
                "path": self.model_path.name,
                "sha256": hashlib.sha256(
                    self.model_path.read_bytes()
                ).hexdigest(),
                "format": "onnx",
                "opset": 17,
            },
            "input": {
                "width": 320,
                "height": 320,
                "layout": "NCHW",
                "color_order": "RGB",
                "scale": 1.0 / 255.0,
                "letterbox_value": 114,
            },
            "output": {
                "layout": OUTPUT_LAYOUT,
                "class_names": ["pen"],
                "pen_class_id": 0,
                "yaw_semantics": YAW_SEMANTICS,
            },
            "thresholds": {
                "confidence": 0.4,
                "iou": 0.5,
                "maximum_detections": 10,
            },
            "training": {
                "holdout_used_for_training": False,
                "holdout_manifest_sha256": self.holdout_hash,
            },
        }

    def _write_bundle(self, document: dict | None = None) -> None:
        self.bundle_path.write_text(
            json.dumps(document or self._bundle()),
            encoding="utf-8",
        )

    @staticmethod
    def _calibration() -> Calibration:
        camera = np.asarray(
            [
                [100.0, 0.0, 320.0],
                [0.0, 100.0, 240.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        return Calibration(
            image_width=640,
            image_height=480,
            camera_matrix=camera,
            distortion=np.zeros(5, dtype=np.float64),
            projection=camera,
            pixel_to_board=np.asarray(
                [
                    [0.001, 0.0, 0.0],
                    [0.0, 0.001, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            board_span=np.asarray([0.64, 0.48], dtype=np.float64),
            camera_info_sha256="test",
            homography_status="TEST",
            base_registration_status="TEST",
            motion_authorized=False,
        )

    def test_bundle_contract_validates_hash_and_holdout_attestation(self) -> None:
        config = load_runtime_config(
            self.bundle_path,
            expected_holdout_manifest_sha256=self.holdout_hash,
        )

        self.assertEqual(config.input_width, 320)
        self.assertEqual(config.class_names, ("pen",))
        self.assertEqual(config.holdout_manifest_sha256, self.holdout_hash)

    def test_bundle_rejects_model_hash_mismatch(self) -> None:
        document = self._bundle()
        document["model"]["sha256"] = "0" * 64
        self._write_bundle(document)

        with self.assertRaisesRegex(ValueError, "SHA-256"):
            load_runtime_config(self.bundle_path)

    def test_bundle_rejects_holdout_used_for_training(self) -> None:
        document = self._bundle()
        document["training"]["holdout_used_for_training"] = True
        self._write_bundle(document)

        with self.assertRaisesRegex(ValueError, "holdout_used_for_training"):
            load_runtime_config(self.bundle_path)

    def test_letterbox_and_raw_output_decode_back_to_source_pixels(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        prepared, transform = letterbox(image, 320, 320)
        self.assertEqual(prepared.shape, (320, 320, 3))
        self.assertAlmostEqual(transform.scale, 0.5)
        self.assertEqual(transform.pad_y, 40)

        output = np.asarray(
            [
                [
                    [160.0, 20.0],
                    [160.0, 20.0],
                    [50.0, 5.0],
                    [10.0, 5.0],
                    [0.9, 0.1],
                    [math.radians(30.0), 0.0],
                ]
            ],
            dtype=np.float32,
        )
        detections = decode_ultralytics_obb(
            output,
            transform,
            class_count=1,
            pen_class_id=0,
            confidence_threshold=0.4,
            iou_threshold=0.5,
            maximum_detections=10,
        )

        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0]["raw_center_px"][0], 320.0)
        self.assertAlmostEqual(detections[0]["raw_center_px"][1], 240.0)
        self.assertAlmostEqual(detections[0]["raw_size_px"][0], 100.0)
        self.assertAlmostEqual(detections[0]["raw_size_px"][1], 20.0)

    def test_rotated_nms_removes_duplicate_predictions(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, transform = letterbox(image, 320, 320)
        output = np.asarray(
            [
                [
                    [160.0, 160.5],
                    [160.0, 160.5],
                    [50.0, 50.0],
                    [10.0, 10.0],
                    [0.9, 0.8],
                    [0.1, 0.1],
                ]
            ],
            dtype=np.float32,
        )

        detections = decode_ultralytics_obb(
            output,
            transform,
            class_count=1,
            pen_class_id=0,
            confidence_threshold=0.4,
            iou_threshold=0.5,
            maximum_detections=10,
        )

        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0]["confidence"], 0.9)

    def test_select_one_pose_preserves_undirected_yaw_contract(self) -> None:
        box = np.asarray(
            [
                [270.0, 230.0],
                [370.0, 230.0],
                [370.0, 250.0],
                [270.0, 250.0],
            ],
            dtype=np.float64,
        )
        pose = select_one_pose(
            [
                {
                    "class_id": 0,
                    "confidence": 0.91,
                    "raw_corners_px": box,
                }
            ],
            self._calibration(),
            image_edge_margin_px=8,
            require_full_footprint=True,
        )

        self.assertEqual(pose["yaw_semantics"], YAW_SEMANTICS)
        self.assertAlmostEqual(pose["raw_center_px"][0], 320.0)
        self.assertAlmostEqual(pose["raw_center_px"][1], 240.0)
        self.assertAlmostEqual(pose["yaw_deg"], 0.0)
        self.assertFalse(pose["calibration_region"]["extrapolated"])

    def test_select_one_pose_rejects_multiple_relevant_detections(self) -> None:
        first = np.asarray(
            [[100, 100], [180, 100], [180, 120], [100, 120]],
            dtype=np.float64,
        )
        second = first + np.asarray([200.0, 100.0])
        detections = [
            {"class_id": 0, "confidence": 0.9, "raw_corners_px": first},
            {"class_id": 0, "confidence": 0.8, "raw_corners_px": second},
        ]

        with self.assertRaisesRegex(DetectionError, "detected 2"):
            select_one_pose(
                detections,
                self._calibration(),
                image_edge_margin_px=8,
                require_full_footprint=True,
            )


if __name__ == "__main__":
    unittest.main()
