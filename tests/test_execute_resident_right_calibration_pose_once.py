import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/resident_gate"
SPEC = importlib.util.spec_from_file_location(
    "execute_resident_right_calibration_pose_once",
    TOOLS / "execute_resident_right_calibration_pose_once.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ExecuteResidentRightCalibrationPoseOnceTest(unittest.TestCase):
    def _capture(self, path: Path, **updates) -> None:
        capture = {
            "id": "right_train_08",
            "arm": "right",
            "measured_arm_rad": [-0.17, 0.09, -0.72, 0.94, -0.24],
            "pnp_rms_px_max": 1.2,
            "pnp_rms_px_limit": 2.5,
            "detected_marker_ids": [0, 1, 2, 3],
        }
        capture.update(updates)
        path.write_text(
            yaml.safe_dump(
                {
                    "status": "STATIONARY_READ_ONLY_CAPTURE_PASS",
                    "motion_authorized": False,
                    "capture": capture,
                }
            )
        )

    def test_loads_only_a_complete_passed_right_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.yaml"
            self._capture(path)
            capture_id, target = MODULE.load_capture_target(path)
            self.assertEqual(capture_id, "right_train_08")
            self.assertEqual(len(target), 5)

            self._capture(path, detected_marker_ids=[0, 1, 2])
            with self.assertRaisesRegex(ValueError, "complete GridBoard"):
                MODULE.load_capture_target(path)

    def test_loads_legacy_wrist_capture_as_a_motion_target_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "status": "WRIST_EYE_IN_HAND_STATIONARY_CAPTURE_PASS",
                        "motion_authorized": False,
                        "arm": "right",
                        "capture": {
                            "id": "right_wrist_train_01",
                            "measured_arm_rad": [0.1] * 5,
                            "detected_marker_ids": list(range(10, 30)),
                        },
                    }
                )
            )
            capture_id, target = MODULE.load_capture_target(path)
            self.assertEqual(capture_id, "right_wrist_train_01")
            self.assertEqual(target, (0.1,) * 5)

            document = yaml.safe_load(path.read_text())
            document["capture"]["detected_marker_ids"] = list(range(10, 29))
            path.write_text(yaml.safe_dump(document))
            with self.assertRaisesRegex(ValueError, "wrist target capture"):
                MODULE.load_capture_target(path)

    def test_loads_explicit_unarmed_wrist_visibility_route_target(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.yaml"
            document = {
                "record_kind": "right_wrist_visibility_route_target",
                "status": "WRIST_ROUTE_TARGET_STATIONARY_CAPTURE_PASS",
                "motion_authorized": False,
                "robot_target_available": True,
                "purpose": "visibility_route_target_only",
                "arm": "right",
                "capture": {
                    "id": "right_tabletop_staged_route_01",
                    "arm": "right",
                    "measured_arm_rad": [0.2] * 5,
                    "detected_marker_ids": list(range(10, 30)),
                    "joint_state_source": "timestamp_synchronized_joint_state",
                },
            }
            path.write_text(yaml.safe_dump(document))
            capture_id, target = MODULE.load_capture_target(path)
            self.assertEqual(capture_id, "right_tabletop_staged_route_01")
            self.assertEqual(target, (0.2,) * 5)

            document["purpose"] = "eye_in_hand_calibration"
            path.write_text(yaml.safe_dump(document))
            with self.assertRaisesRegex(ValueError, "route target provenance"):
                MODULE.load_capture_target(path)

    def test_composition_holds_left_arm_and_both_grippers(self):
        anchor = tuple(float(index) for index in range(12))
        right = (-1.0, -2.0, -3.0, -4.0, -5.0)
        target = MODULE.compose_bimanual_target(anchor, right)
        self.assertEqual(target[:6], anchor[:6])
        self.assertEqual(target[6:11], right)
        self.assertEqual(target[11], anchor[11])

    def test_operator_target_offset_is_small_and_explicit(self):
        source = (0.0,) * 5
        actual = MODULE.offset_right_target(
            source,
            (0.12, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(actual, (0.12, 0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "approved bound"):
            MODULE.offset_right_target(
                source,
                (0.151, 0.0, 0.0, 0.0, 0.0),
            )

    def test_segmentation_bounds_every_subleg(self):
        start = (0.0,) * 12
        target = (0.0,) * 6 + (0.17, 0.0, 0.0, 0.0, 0.0, 0.0)
        segments = MODULE.segmented_targets(start, target)
        self.assertEqual(len(segments), 5)
        previous = start
        for segment in segments:
            maximum = max(abs(b - a) for a, b in zip(previous, segment))
            self.assertLessEqual(maximum, MODULE.MAXIMUM_SUBLEG_DELTA_RAD)
            previous = segment
        self.assertEqual(segments[-1], target)

    def test_terminal_residual_is_a_gross_sanity_bound(self):
        self.assertEqual(MODULE.FINAL_RESIDUAL_LIMIT_RAD, 0.05)
        self.assertGreater(MODULE.FINAL_RESIDUAL_LIMIT_RAD, 0.030679)

    def test_precheck_mode_is_explicitly_motionless(self):
        source = (
            TOOLS / "execute_resident_right_calibration_pose_once.py"
        ).read_text()
        self.assertIn('"--precheck-only"', source)
        self.assertIn("motion_request_sent=false torque_enabled=false", source)

    def test_capture_completion_gate_is_bounded_and_fail_closed(self):
        source = (
            TOOLS / "execute_resident_right_calibration_pose_once.py"
        ).read_text()
        self.assertIn('"--capture-completion-file"', source)
        self.assertIn("capture completion file already exists before motion", source)
        self.assertIn("bounded hold timeout", source)
        self.assertIn("RESIDENT_RIGHT_CALIBRATION_CAPTURE_FILE_CONFIRMED", source)

    def test_session_epoch_accepts_fresh_or_same_owner_held_state(self):
        fresh = {
            "state": "ready",
            "owner": None,
            "arbiter_epoch": 0,
            "motion_authorized": True,
            "torque_hold_active": False,
            "fault_diagnostic": None,
        }
        self.assertEqual(
            MODULE.initial_session_epoch(
                fresh,
                resume_held_session=False,
                precheck_only=False,
            ),
            0,
        )
        held = dict(
            fresh,
            owner=MODULE.OWNER,
            arbiter_epoch=12,
            torque_hold_active=True,
        )
        self.assertEqual(
            MODULE.initial_session_epoch(
                held,
                resume_held_session=True,
                precheck_only=False,
            ),
            12,
        )
        held["owner"] = "different_owner"
        with self.assertRaisesRegex(ValueError, "cannot be resumed"):
            MODULE.initial_session_epoch(
                held,
                resume_held_session=True,
                precheck_only=False,
            )

    def test_successful_hold_can_be_left_for_a_resumed_command(self):
        source = (
            TOOLS / "execute_resident_right_calibration_pose_once.py"
        ).read_text()
        self.assertIn('"--resume-held-session"', source)
        self.assertIn('"--leave-torque-hold-active"', source)
        self.assertIn("RESIDENT_RIGHT_CALIBRATION_POSE_HOLD_ACTIVE", source)

    def test_visibility_skip_preserves_only_an_explicit_held_session(self):
        source = (
            TOOLS / "execute_resident_right_calibration_pose_once.py"
        ).read_text()
        self.assertIn('"--capture-skip-file"', source)
        self.assertIn("capture skip file already exists before motion", source)
        self.assertIn("CAPTURE_VISIBILITY_SKIPPED", source)
        self.assertIn("VISIBILITY_SKIPPED_HOLD_ACTIVE", source)


if __name__ == "__main__":
    unittest.main()
