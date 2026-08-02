import copy
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "validate_policy_deployment_bundle.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_policy_deployment_bundle",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class PolicyDeploymentBundleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.contract_path = self.directory / "deployment_contract.json"
        self.observation_path = self.directory / "observation.json"
        self.model_path = self.directory / "policy.onnx"
        self.manifest_path = self.directory / "bundle.json"
        self.contract = json.loads(
            (ROOT / "config" / "policy_deployment_contract.json").read_text(
                encoding="utf-8"
            )
        )
        self.contract_path.write_text(
            json.dumps(self.contract),
            encoding="utf-8",
        )
        self.model_path.write_bytes(b"test-only-policy-onnx")
        self.observation = {
            "schema_version": 1,
            "contract_kind": MODULE.OBSERVATION_KIND,
            "motion_authorized": False,
            "mode": "structured_state",
            "timestamp": {
                "max_age_ms": 100.0,
                "max_source_skew_ms": 30.0,
                "stale_action": "reject",
                "skew_action": "reject",
            },
            "camera_order": [],
            "inputs": [
                {
                    "name": "observation",
                    "dtype": "float32",
                    "shape": [1, 3],
                    "source": {
                        "kind": "structured_vector",
                        "ordered_features": [
                            "left_base_joint.position",
                            "left_shoulder_joint.position",
                            "target.x",
                        ],
                    },
                    "preprocessing": {
                        "kind": "affine",
                        "offset": [0.0, 0.0, 0.0],
                        "scale": [1.0, 1.0, 1.0],
                    },
                }
            ],
        }
        self._write_observation()
        self.manifest = self._passing_manifest()
        self._write_manifest()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_observation(self) -> None:
        self.observation_path.write_text(
            json.dumps(self.observation),
            encoding="utf-8",
        )

    def _passing_manifest(self) -> dict:
        return {
            "schema_version": 1,
            "bundle_kind": MODULE.BUNDLE_KIND,
            "bundle_id": "test-policy-v1",
            "motion_authorized": False,
            "deployment_contract_sha256": MODULE.file_sha256(
                self.contract_path
            ),
            "model": {
                "path": self.model_path.name,
                "sha256": MODULE.file_sha256(self.model_path),
                "format": "onnx",
                "opset": 17,
                "outputs": [
                    {
                        "name": "action",
                        "dtype": "float32",
                        "shape": [1, 2],
                    }
                ],
            },
            "observation_contract": {
                "path": self.observation_path.name,
                "sha256": MODULE.file_sha256(self.observation_path),
            },
            "runtime": {
                "mode": "SHADOW",
                "backend": "onnxruntime_cpu",
                "control_dt_s": 0.1,
                "target_inference_hz": 10.0,
                "deadline_ms": 80.0,
                "intra_op_threads": 1,
                "inter_op_threads": 1,
                "command_publications_allowed": False,
            },
            "safety": copy.deepcopy(MODULE.REQUIRED_SAFETY),
            "action": {
                "output_name": "action",
                "representation": "joint_position_residual_rad",
                "order": [
                    "left_base_joint",
                    "left_shoulder_joint",
                ],
                "scale": [0.02, 0.02],
                "lower": [-0.02, -0.02],
                "upper": [0.02, 0.02],
            },
            "provenance": {
                "training_framework": "isaac_lab",
                "checkpoint_sha256": "1" * 64,
                "training_config_sha256": "2" * 64,
                "export_config_sha256": "3" * 64,
            },
        }

    def _write_manifest(self) -> None:
        self.manifest_path.write_text(
            json.dumps(self.manifest),
            encoding="utf-8",
        )

    @staticmethod
    def _matching_model(_: Path) -> dict:
        return {
            "opset": 17,
            "inputs": [
                {
                    "name": "observation",
                    "dtype": "float32",
                    "shape": [1, 3],
                }
            ],
            "outputs": [
                {
                    "name": "action",
                    "dtype": "float32",
                    "shape": [1, 2],
                }
            ],
        }

    def _validate(self) -> dict:
        return MODULE.validate_policy_deployment_bundle(
            self.manifest_path,
            self.contract_path,
            inspect_model=self._matching_model,
        )

    @staticmethod
    def _rgb_observation() -> dict:
        preprocessing = {
            "layout": "NCHW",
            "resize_hw": [96, 128],
            "color_order": "RGB",
            "scale": 1.0 / 255.0,
            "mean": [0.485, 0.456, 0.406],
            "std": [0.229, 0.224, 0.225],
        }
        return {
            "schema_version": 1,
            "contract_kind": MODULE.OBSERVATION_KIND,
            "motion_authorized": False,
            "mode": "rgb_tensor",
            "timestamp": {
                "max_age_ms": 100.0,
                "max_source_skew_ms": 30.0,
                "stale_action": "reject",
                "skew_action": "reject",
            },
            "camera_order": ["top", "wrist_a"],
            "inputs": [
                {
                    "name": "top_rgb",
                    "dtype": "float32",
                    "shape": [1, 3, 96, 128],
                    "source": {
                        "kind": "rgb_image",
                        "camera": "top",
                        "encoding": "rgb8",
                    },
                    "preprocessing": copy.deepcopy(preprocessing),
                },
                {
                    "name": "wrist_a_rgb",
                    "dtype": "float32",
                    "shape": [1, 3, 96, 128],
                    "source": {
                        "kind": "rgb_image",
                        "camera": "wrist_a",
                        "encoding": "rgb8",
                    },
                    "preprocessing": copy.deepcopy(preprocessing),
                },
            ],
        }

    def test_valid_bundle_is_hash_pinned_and_shadow_only(self) -> None:
        result = self._validate()

        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "POLICY_DEPLOYMENT_BUNDLE_PASS")
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["command_publications_allowed"])
        self.assertEqual(result["robot_command_topics_created"], 0)
        self.assertEqual(result["model_sha256"], MODULE.file_sha256(self.model_path))
        self.assertEqual(
            result["observation_contract_sha256"],
            MODULE.file_sha256(self.observation_path),
        )

    def test_model_hash_mismatch_is_rejected(self) -> None:
        self.manifest["model"]["sha256"] = "0" * 64
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "model.sha256 mismatch"):
            self._validate()

    def test_observation_hash_mismatch_is_rejected(self) -> None:
        self.observation["timestamp"]["max_age_ms"] = 200.0
        self._write_observation()

        with self.assertRaisesRegex(
            ValueError,
            "observation_contract.sha256 mismatch",
        ):
            self._validate()

    def test_path_traversal_is_rejected(self) -> None:
        self.manifest["model"]["path"] = "../policy.onnx"
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "stay inside"):
            self._validate()

    def test_motion_authority_is_rejected(self) -> None:
        self.manifest["motion_authorized"] = True
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "motion_authorized"):
            self._validate()

    def test_command_publication_is_rejected(self) -> None:
        self.manifest["runtime"]["command_publications_allowed"] = True
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "command publications"):
            self._validate()

    def test_weakened_safety_policy_is_rejected(self) -> None:
        self.manifest["safety"]["deadline_miss"] = "reuse_last"
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "safety policy"):
            self._validate()

    def test_onnx_input_mismatch_is_rejected(self) -> None:
        def mismatched_model(_: Path) -> dict:
            model = self._matching_model(Path())
            model["inputs"][0]["shape"] = [1, 4]
            return model

        with self.assertRaisesRegex(ValueError, "ONNX inputs"):
            MODULE.validate_policy_deployment_bundle(
                self.manifest_path,
                self.contract_path,
                inspect_model=mismatched_model,
            )

    def test_onnx_output_mismatch_is_rejected(self) -> None:
        def mismatched_model(_: Path) -> dict:
            model = self._matching_model(Path())
            model["outputs"][0]["dtype"] = "float16"
            return model

        with self.assertRaisesRegex(ValueError, "ONNX outputs"):
            MODULE.validate_policy_deployment_bundle(
                self.manifest_path,
                self.contract_path,
                inspect_model=mismatched_model,
            )

    def test_rgb_camera_order_and_preprocessing_are_accepted(self) -> None:
        observation = self._rgb_observation()

        specs = MODULE.validate_observation_contract(
            observation,
            self.contract,
        )

        self.assertEqual(
            [value["name"] for value in specs],
            ["top_rgb", "wrist_a_rgb"],
        )

    def test_rgb_camera_order_mismatch_is_rejected(self) -> None:
        observation = self._rgb_observation()
        observation["camera_order"] = ["wrist_a", "top"]

        with self.assertRaisesRegex(ValueError, "camera_order"):
            MODULE.validate_observation_contract(
                observation,
                self.contract,
            )

    def test_rgb_shape_must_match_resize_and_layout(self) -> None:
        observation = self._rgb_observation()
        observation["inputs"][0]["shape"] = [1, 3, 128, 96]

        with self.assertRaisesRegex(ValueError, "resize/layout"):
            MODULE.validate_observation_contract(
                observation,
                self.contract,
            )

    def test_deadline_must_fit_control_period(self) -> None:
        self.manifest["runtime"]["deadline_ms"] = 100.1
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "must not exceed control_dt"):
            self._validate()

    def test_provenance_hashes_are_required(self) -> None:
        self.manifest["provenance"]["checkpoint_sha256"] = "missing"
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "checkpoint_sha256"):
            self._validate()

    def test_action_vector_length_mismatch_is_rejected(self) -> None:
        self.manifest["action"]["order"] = ["left_base_joint"]
        self.manifest["action"]["scale"] = [0.02]
        self.manifest["action"]["lower"] = [-0.02]
        self.manifest["action"]["upper"] = [0.02]
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "output shape"):
            self._validate()

    def test_contract_cannot_allow_motion(self) -> None:
        self.contract["motion_authorized"] = True
        self.contract_path.write_text(
            json.dumps(self.contract),
            encoding="utf-8",
        )
        self.manifest["deployment_contract_sha256"] = MODULE.file_sha256(
            self.contract_path
        )
        self._write_manifest()

        with self.assertRaisesRegex(ValueError, "motion_authorized"):
            self._validate()


if __name__ == "__main__":
    unittest.main()
