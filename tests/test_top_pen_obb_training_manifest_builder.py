import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "build_top_pen_obb_training_manifest.py"
SPEC = importlib.util.spec_from_file_location(
    "build_top_pen_obb_training_manifest",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopPenObbTrainingManifestBuilderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "images").mkdir()
        (self.root / "labels").mkdir()
        (self.root / "data.yaml").write_text("names: [pen]\n", encoding="utf-8")
        self.image = self.root / "images" / "positive.png"
        self.image.write_bytes(b"image")
        self.label = self.root / "labels" / "positive.txt"
        self.label.write_text(
            "0 0.2 0.4 0.8 0.4 0.8 0.6 0.2 0.6\n",
            encoding="utf-8",
        )
        self.metadata_path = self.root / "metadata.json"
        self.metadata = {
            "protocol_version": 1,
            "dataset_id": "test",
            "ultralytics_data_yaml": "data.yaml",
            "cases": [
                {
                    "id": "positive",
                    "split": "train",
                    "image": "images/positive.png",
                    "label": "labels/positive.txt",
                    "expected_present": True,
                    "condition": {
                        "background": "home",
                        "lighting": "normal",
                        "glare": "none",
                    },
                }
            ],
        }
        self._write_metadata()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_metadata(self) -> None:
        self.metadata_path.write_text(
            json.dumps(self.metadata),
            encoding="utf-8",
        )

    def test_builds_hash_pinned_manifest(self) -> None:
        manifest = MODULE.build_manifest(self.root, self.metadata_path)

        case = manifest["splits"]["train"][0]
        self.assertEqual(case["image_sha256"], MODULE.file_sha256(self.image))
        self.assertEqual(case["label_sha256"], MODULE.file_sha256(self.label))
        self.assertEqual(manifest["splits"]["validation"], [])
        self.assertEqual(
            manifest["yaw_semantics"],
            MODULE.YAW_SEMANTICS,
        )

    def test_rejects_path_escape(self) -> None:
        self.metadata["cases"][0]["image"] = "../outside.png"
        self._write_metadata()

        with self.assertRaisesRegex(ValueError, "inside the dataset"):
            MODULE.build_manifest(self.root, self.metadata_path)

    def test_rejects_duplicate_ids(self) -> None:
        self.metadata["cases"].append(dict(self.metadata["cases"][0]))
        self._write_metadata()

        with self.assertRaisesRegex(ValueError, "duplicate"):
            MODULE.build_manifest(self.root, self.metadata_path)


if __name__ == "__main__":
    unittest.main()
