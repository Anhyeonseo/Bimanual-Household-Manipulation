import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/camera_calibration"
SPEC = importlib.util.spec_from_file_location(
    "assemble_top_eye_to_hand_session",
    TOOLS / "assemble_top_eye_to_hand_session.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class AssembleTopEyeToHandSessionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.output = self.root / "session" / "session.yaml"

    def tearDown(self):
        self.temporary.cleanup()

    def make_capture(
        self,
        capture_id: str,
        *,
        marker_ids=(0, 1, 2, 3),
        motion_authorized=False,
        arm="left",
    ) -> Path:
        directory = self.root / "captures" / capture_id
        directory.mkdir(parents=True)
        image_files = []
        for index in range(2):
            image = directory / f"frame_{index:03d}.png"
            image.write_bytes(b"test-image")
            image_files.append(image.name)
        document = {
            "schema_version": 1,
            "status": "STATIONARY_READ_ONLY_CAPTURE_PASS",
            "motion_authorized": motion_authorized,
            "robot_target_available": False,
            "capture": {
                "id": capture_id,
                "arm": arm,
                "measured_arm_rad": [0.1, 0.2, 0.3, 0.4, 0.5],
                "joint_span_rad": [0.0] * 5,
                "image_files": image_files,
                "image_source_stamp_first": 100.0,
                "image_source_stamp_last": 101.0,
                "detected_marker_ids": list(marker_ids),
            },
        }
        (directory / "capture.yaml").write_text(
            yaml.safe_dump(document, sort_keys=False),
            encoding="utf-8",
        )
        return directory

    def valid_inputs(self):
        training = [
            self.make_capture(f"train_{index:02d}")
            for index in range(1, 9)
        ]
        validation = [
            self.make_capture(f"validation_{index:02d}")
            for index in range(1, 3)
        ]
        return training, validation

    def test_assembles_fail_closed_session_with_relative_images(self):
        training, validation = self.valid_inputs()
        document = MODULE.assemble_document(
            "session_01",
            training,
            validation,
            self.output,
        )
        self.assertEqual(
            document["status"],
            "CAPTURE_SET_VALIDATED_READY_TO_SOLVE",
        )
        self.assertFalse(document["motion_authorized"])
        self.assertFalse(document["robot_target_available"])
        self.assertEqual(len(document["training_captures"]), 8)
        self.assertEqual(len(document["validation_captures"]), 2)
        capture = document["training_captures"][0]
        self.assertFalse(Path(capture["image_files"][0]).is_absolute())
        self.assertEqual(len(capture["source_capture_sha256"]), 64)

    def test_rejects_wrong_marker_ids(self):
        training, validation = self.valid_inputs()
        replacement = self.make_capture(
            "wrong_markers",
            marker_ids=(0, 1, 2),
        )
        training[0] = replacement
        with self.assertRaisesRegex(ValueError, "marker IDs"):
            MODULE.assemble_document(
                "session_01",
                training,
                validation,
                self.output,
            )

    def test_rejects_motion_authorized_capture(self):
        training, validation = self.valid_inputs()
        replacement = self.make_capture(
            "unsafe_capture",
            motion_authorized=True,
        )
        training[0] = replacement
        with self.assertRaisesRegex(ValueError, "authorizes motion"):
            MODULE.assemble_document(
                "session_01",
                training,
                validation,
                self.output,
            )

    def test_rejects_duplicate_capture_ids(self):
        training, validation = self.valid_inputs()
        validation[0] = training[0]
        with self.assertRaisesRegex(ValueError, "unique"):
            MODULE.assemble_document(
                "session_01",
                training,
                validation,
                self.output,
            )

    def test_rejects_insufficient_capture_counts(self):
        training, validation = self.valid_inputs()
        with self.assertRaisesRegex(ValueError, "training capture count"):
            MODULE.assemble_document(
                "session_01",
                training[:7],
                validation,
                self.output,
            )
        with self.assertRaisesRegex(ValueError, "validation capture count"):
            MODULE.assemble_document(
                "session_01",
                training,
                validation[:1],
                self.output,
            )

    def test_assembles_right_arm_frames(self):
        training = [
            self.make_capture(f"right_train_{index:02d}", arm="right")
            for index in range(1, 9)
        ]
        validation = [
            self.make_capture(f"right_validation_{index:02d}", arm="right")
            for index in range(1, 3)
        ]
        document = MODULE.assemble_document(
            "right_session_01",
            training,
            validation,
            self.output,
            "right",
        )
        self.assertEqual(document["arm"], "right")
        self.assertEqual(document["frames"]["robot"], "right_base_link")
        self.assertEqual(
            document["frames"]["gripper"],
            "right_gripper_frame_link",
        )

    def test_rejects_capture_from_other_arm(self):
        training, validation = self.valid_inputs()
        with self.assertRaisesRegex(ValueError, "session arm"):
            MODULE.assemble_document(
                "right_session_01",
                training,
                validation,
                self.output,
                "right",
            )


if __name__ == "__main__":
    unittest.main()
