import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "validate_top_pen_obb_training_dataset.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_top_pen_obb_training_dataset",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopPenObbTrainingDatasetTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.manifest_path = self.directory / "training_manifest.json"
        self.holdout_path = self.directory / "holdout_manifest.json"
        self.contract_path = self.directory / "contract.json"
        self.images = self.directory / "images"
        self.labels = self.directory / "labels"
        self.images.mkdir()
        self.labels.mkdir()
        self.contract = {
            "protocol_version": 1,
            "dataset_role": "training",
            "class_names": ["pen"],
            "yaw_semantics": MODULE.YAW_SEMANTICS,
            "coverage": {
                "minimum_train_positive": 1,
                "minimum_train_negative": 1,
                "minimum_validation_positive": 1,
                "minimum_validation_negative": 1,
                "minimum_background_labels": 1,
                "minimum_lighting_labels": 1,
                "minimum_glare_labels": 1,
            },
            "leakage": {
                "forbid_holdout_image_sha256": True,
                "require_disjoint_train_validation_sha256": True,
            },
            "annotation": {
                "format": MODULE.LABEL_FORMAT,
                "positive_objects_per_image": 1,
                "negative_label_must_be_empty": True,
            },
            "motion_authorized": False,
        }
        self.contract_path.write_text(
            json.dumps(self.contract),
            encoding="utf-8",
        )
        holdout_image = self._write_image("holdout.png", 50)
        self.holdout_path.write_text(
            json.dumps(
                {
                    "protocol_version": 1,
                    "cases": [
                        {
                            "image_sha256": MODULE.file_sha256(
                                holdout_image
                            )
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        self.manifest = self._passing_manifest()
        self._write_manifest()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_image(self, name: str, value: int) -> Path:
        path = self.images / name
        image = np.full((32, 64, 3), value, dtype=np.uint8)
        self.assertTrue(cv2.imwrite(str(path), image))
        return path

    def _case(
        self,
        name: str,
        value: int,
        expected_present: bool,
    ) -> dict:
        image = self._write_image(f"{name}.png", value)
        label = self.labels / f"{name}.txt"
        label.write_text(
            (
                "0 0.2 0.4 0.8 0.4 0.8 0.6 0.2 0.6\n"
                if expected_present
                else ""
            ),
            encoding="utf-8",
        )
        return {
            "id": name,
            "image": str(image.relative_to(self.directory)),
            "image_sha256": MODULE.file_sha256(image),
            "label": str(label.relative_to(self.directory)),
            "label_sha256": MODULE.file_sha256(label),
            "expected_present": expected_present,
            "condition": {
                "background": "test",
                "lighting": "normal",
                "glare": "none",
            },
        }

    def _passing_manifest(self) -> dict:
        return {
            "protocol_version": 1,
            "dataset_id": "test-training",
            "dataset_role": "training",
            "class_names": ["pen"],
            "yaw_semantics": MODULE.YAW_SEMANTICS,
            "splits": {
                "train": [
                    self._case("train-positive", 100, True),
                    self._case("train-negative", 110, False),
                ],
                "validation": [
                    self._case("validation-positive", 120, True),
                    self._case("validation-negative", 130, False),
                ],
            },
        }

    def _write_manifest(self) -> None:
        self.manifest_path.write_text(
            json.dumps(self.manifest),
            encoding="utf-8",
        )

    def _evaluate(self) -> dict:
        return MODULE.validate_training_dataset(
            self.manifest_path,
            self.holdout_path,
            self.contract_path,
        )

    def test_valid_training_dataset_passes_without_motion_authority(self) -> None:
        result = self._evaluate()

        self.assertTrue(result["passed"])
        self.assertEqual(
            result["status"],
            "TOP_PEN_OBB_TRAINING_DATASET_PASS",
        )
        self.assertEqual(result["holdout_overlap_count"], 0)
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["robot_command_topics_created"], 0)

    def test_holdout_image_reuse_is_rejected(self) -> None:
        holdout = json.loads(self.holdout_path.read_text(encoding="utf-8"))
        case = self.manifest["splits"]["train"][0]
        case["image_sha256"] = holdout["cases"][0]["image_sha256"]
        holdout_image = self.images / "holdout.png"
        target = self.directory / case["image"]
        target.write_bytes(holdout_image.read_bytes())
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "frozen holdout"):
            self._evaluate()

    def test_non_empty_negative_label_is_rejected(self) -> None:
        case = self.manifest["splits"]["train"][1]
        label = self.directory / case["label"]
        label.write_text(
            "0 0.2 0.4 0.8 0.4 0.8 0.6 0.2 0.6\n",
            encoding="utf-8",
        )
        case["label_sha256"] = MODULE.file_sha256(label)
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "negative label"):
            self._evaluate()

    def test_invalid_positive_polygon_is_rejected(self) -> None:
        case = self.manifest["splits"]["validation"][0]
        label = self.directory / case["label"]
        label.write_text(
            "0 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5\n",
            encoding="utf-8",
        )
        case["label_sha256"] = MODULE.file_sha256(label)
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "area is zero"):
            self._evaluate()

    def test_insufficient_coverage_returns_failed_gate(self) -> None:
        self.contract["coverage"]["minimum_train_positive"] = 2
        self.contract_path.write_text(
            json.dumps(self.contract),
            encoding="utf-8",
        )

        result = self._evaluate()

        self.assertFalse(result["passed"])
        self.assertIn("insufficient_train_positive", result["failures"])

    def test_weakened_leakage_contract_is_rejected(self) -> None:
        self.contract["leakage"][
            "forbid_holdout_image_sha256"
        ] = False
        self.contract_path.write_text(
            json.dumps(self.contract),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "frozen holdout"):
            self._evaluate()

    def test_duplicate_image_across_splits_is_rejected(self) -> None:
        train = self.manifest["splits"]["train"][0]
        validation = self.manifest["splits"]["validation"][0]
        source = self.directory / train["image"]
        target = self.directory / validation["image"]
        target.write_bytes(source.read_bytes())
        validation["image_sha256"] = MODULE.file_sha256(target)
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "duplicates"):
            self._evaluate()


if __name__ == "__main__":
    unittest.main()
