import importlib.util
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from tests import test_top_pen_detection_baseline as fixtures


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "evaluate_top_pen_yolo_obb.py"
SPEC = importlib.util.spec_from_file_location(
    "evaluate_top_pen_yolo_obb",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopPenYoloObbEvaluationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = fixtures.TopPenDetectionBaselineTest()
        self.fixture.setUp()
        self.fixture._write_manifest(self.fixture._passing_cases())
        contract = json.loads(
            self.fixture.contract_path.read_text(encoding="utf-8")
        )
        contract["detector"]["backend"] = MODULE.obb_detector.BACKEND_NAME
        contract["detector"]["image_edge_margin_px"] = 2
        self.fixture.contract_path.write_text(
            json.dumps(contract),
            encoding="utf-8",
        )
        self.bundle_path = self.fixture.directory / "bundle.json"
        self.bundle_path.write_text("{}\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.fixture.tearDown()

    def test_common_holdout_metrics_are_reused_for_obb_candidate(self) -> None:
        expected_holdout_hash = (
            MODULE.baseline.shared_detector.file_sha256(
                self.fixture.manifest_path
            )
        )

        class FakeDetector:
            def __init__(instance, bundle, expected_holdout_manifest_sha256):
                self.assertEqual(bundle, self.bundle_path.resolve())
                self.assertEqual(
                    expected_holdout_manifest_sha256,
                    expected_holdout_hash,
                )
                instance.config = SimpleNamespace(model_sha256="b" * 64)

            def detect(
                instance,
                image,
                calibration,
                image_edge_margin_px,
                require_full_footprint,
            ):
                return MODULE.baseline.shared_detector.detect_one_object(
                    image,
                    calibration,
                    MODULE.baseline.detector_config(
                        MODULE.baseline.load_json(
                            self.fixture.contract_path
                        )
                    ),
                    require_full_footprint=require_full_footprint,
                )

        with mock.patch.object(
            MODULE.obb_detector,
            "OpenCvYoloObbDetector",
            FakeDetector,
        ):
            result = MODULE.evaluate(
                self.fixture.manifest_path,
                self.fixture.contract_path,
                self.fixture.camera_path,
                self.fixture.homography_path,
                self.bundle_path,
            )

        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "TOP_PEN_YOLO_OBB_HOLDOUT_PASS")
        self.assertEqual(
            result["detector_backend"],
            MODULE.obb_detector.BACKEND_NAME,
        )
        self.assertFalse(result["holdout_used_for_training"])
        self.assertEqual(result["model_sha256"], "b" * 64)
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["robot_command_topics_created"], 0)

    def test_wrong_backend_contract_is_rejected_before_model_load(self) -> None:
        contract = json.loads(
            self.fixture.contract_path.read_text(encoding="utf-8")
        )
        contract["detector"]["backend"] = "legacy_dark_threshold"
        self.fixture.contract_path.write_text(
            json.dumps(contract),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "detector.backend"):
            MODULE.evaluate(
                self.fixture.manifest_path,
                self.fixture.contract_path,
                self.fixture.camera_path,
                self.fixture.homography_path,
                self.bundle_path,
            )


if __name__ == "__main__":
    unittest.main()
