import argparse
import importlib.util
import sys
import unittest
from pathlib import Path

from tests import test_top_pen_obb_training_dataset as fixtures


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "train_export_top_pen_yolo_obb.py"
SPEC = importlib.util.spec_from_file_location(
    "train_export_top_pen_yolo_obb",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopPenObbTrainExportTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = fixtures.TopPenObbTrainingDatasetTest()
        self.fixture.setUp()

    def tearDown(self) -> None:
        self.fixture.tearDown()

    def test_dry_run_validates_without_importing_training_framework(self) -> None:
        data_yaml = self.fixture.directory / "data.yaml"
        data_yaml.write_text(
            "path: .\ntrain: images\nval: images\nnames: [pen]\n",
            encoding="utf-8",
        )
        self.fixture.manifest["ultralytics_data_yaml"] = data_yaml.name
        self.fixture._write_manifest()
        args = argparse.Namespace(
            manifest=self.fixture.manifest_path,
            holdout_manifest=self.fixture.holdout_path,
            training_contract=self.fixture.contract_path,
            output_dir=self.fixture.directory / "output",
            base_model="yolo11n-obb.pt",
            image_size=320,
            epochs=1,
            batch=1,
            workers=0,
            seed=101,
            device="cpu",
            confidence=0.4,
            iou=0.5,
            dry_run=True,
            output=None,
        )

        result = MODULE.train_and_export(args)

        self.assertTrue(result["passed"])
        self.assertEqual(
            result["status"],
            "TOP_PEN_YOLO_OBB_TRAIN_EXPORT_DRY_RUN_PASS",
        )
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["robot_command_topics_created"], 0)
