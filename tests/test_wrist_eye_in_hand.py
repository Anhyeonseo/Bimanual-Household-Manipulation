import importlib.util
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/camera_calibration"
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location(
    "solve_wrist_eye_in_hand",
    TOOLS / "solve_wrist_eye_in_hand.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def transform(rotation_vector, translation):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotation_vector).as_matrix()
    result[:3, 3] = translation
    return result


def transform_error(first, second):
    error = MODULE.invert_transform(first) @ second
    return (
        np.linalg.norm(error[:3, 3]),
        Rotation.from_matrix(error[:3, :3]).magnitude(),
    )


class WristEyeInHandTest(unittest.TestCase):
    def valid_right_registration(self):
        joint_names = MODULE.ARM_JOINT_NAMES_BY_SIDE["right"]
        return {
            "status": MODULE.VALIDATED_EYE_TO_HAND_STATUS,
            "motion_authorized": False,
            "arm": "right",
            "method": MODULE.RIGHT_REGISTRATION_METHOD,
            "right_kinematic_registration": {
                "training_only_fit": True,
                "validation_used_in_fit": False,
                "joint_zero_offsets_rad": {
                    name: index * 0.01
                    for index, name in enumerate(joint_names)
                },
            },
        }

    def valid_right_session(self):
        def capture(capture_id, epoch):
            return {
                "id": capture_id,
                "source_capture_mode": MODULE.RIGHT_CAPTURE_MODE,
                "source_motion_authorized": True,
                "resident_torque_hold": {
                    "status_service": MODULE.RESIDENT_STATUS_SERVICE,
                    "owner": "resident_right_calibration_operator",
                    "arbiter_epoch": epoch,
                    "torque_hold_active": True,
                    "terminal_anchor_stamp": 100.0 + epoch,
                    "required_owner": "resident_right_calibration_operator",
                    "required_epoch": epoch,
                },
            }

        return {
            "capture_mode": MODULE.RIGHT_CAPTURE_MODE,
            "source_motion_authorized": True,
            "training_captures": [capture("train", 1)],
            "validation_captures": [capture("validation", 2)],
        }

    def synthetic_observations(self):
        gripper_to_camera = transform(
            [0.05, -0.02, 1.55],
            [0.02, -0.01, 0.03],
        )
        base_to_target = transform(
            [0.0, 0.0, 0.10],
            [0.30, -0.12, 0.0],
        )
        random = np.random.default_rng(7)
        observations = []
        for index in range(12):
            base_to_gripper = transform(
                random.uniform(-0.5, 0.5, 3),
                random.uniform(
                    [0.10, -0.20, 0.05],
                    [0.35, 0.10, 0.30],
                ),
            )
            camera_to_target = (
                MODULE.invert_transform(gripper_to_camera)
                @ MODULE.invert_transform(base_to_gripper)
                @ base_to_target
            )
            observations.append(
                MODULE.PoseObservation(
                    capture_id=f"S{index:02d}",
                    base_to_gripper=base_to_gripper,
                    camera_to_target=camera_to_target,
                    pnp_rms_px=0.1,
                    image_border_px=50.0,
                    detected_marker_ids=tuple(range(10, 30)),
                )
            )
        return observations, gripper_to_camera, base_to_target

    def test_solver_recovers_the_unknown_transform(self):
        observations, expected_camera, _ = self.synthetic_observations()
        actual_camera = MODULE.solve_eye_in_hand(observations)
        translation, rotation = transform_error(expected_camera, actual_camera)
        self.assertLess(translation, 1e-9)
        self.assertLess(rotation, 1e-9)

    def test_clean_training_and_validation_pass_but_never_authorize_motion(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:10]
        validation = observations[10:]
        training_summary = MODULE.residual_summary(training, camera, target)
        validation_summary = MODULE.residual_summary(validation, camera, target)
        status, failures = MODULE.classify(
            training,
            validation,
            training_summary,
            validation_summary,
        )
        self.assertEqual(
            status,
            "EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        )
        self.assertEqual(failures, [])

    def test_right_resident_track_accepts_six_training_captures(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:6]
        validation = observations[10:]
        status, failures = MODULE.classify(
            training,
            validation,
            MODULE.residual_summary(training, camera, target),
            MODULE.residual_summary(validation, camera, target),
            MODULE.MIN_RIGHT_RESIDENT_TRAINING_CAPTURES,
        )
        self.assertEqual(
            status,
            "EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        )
        self.assertEqual(failures, [])

    def test_small_pose_distribution_is_rejected(self):
        observations, camera, target = self.synthetic_observations()
        collapsed = [
            replace(
                observation,
                base_to_gripper=transform(
                    [0.0, 0.0, math.radians(index)],
                    [0.20 + index * 0.001, 0.0, 0.20],
                ),
            )
            for index, observation in enumerate(observations)
        ]
        # base_to_gripper changed, so camera_to_target must be regenerated to
        # stay consistent with a target fixed in base -- otherwise the
        # residual check (not the span check) would reject first.
        consistent = [
            replace(
                observation,
                camera_to_target=(
                    MODULE.invert_transform(camera)
                    @ MODULE.invert_transform(observation.base_to_gripper)
                    @ target
                ),
            )
            for observation in collapsed
        ]
        summary = MODULE.residual_summary(consistent, camera, target)
        status, failures = MODULE.classify(
            consistent[:10],
            consistent[10:],
            summary,
            summary,
        )
        self.assertEqual(status, "REJECTED_EYE_IN_HAND_CALIBRATION")
        self.assertIn("training translation span is too small", failures)

    def test_pnp_reprojection_rms_at_limit_is_accepted(self):
        observations, camera, target = self.synthetic_observations()
        observations = [
            replace(observation, pnp_rms_px=MODULE.MAX_PNP_RMS_PX)
            for observation in observations
        ]
        training = observations[:10]
        validation = observations[10:]

        status, failures = MODULE.classify(
            training,
            validation,
            MODULE.residual_summary(training, camera, target),
            MODULE.residual_summary(validation, camera, target),
        )

        self.assertEqual(
            status,
            "EYE_IN_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        )
        self.assertEqual(failures, [])

    def test_pnp_reprojection_rms_above_limit_is_rejected(self):
        observations, camera, target = self.synthetic_observations()
        observations[0] = replace(
            observations[0],
            pnp_rms_px=MODULE.MAX_PNP_RMS_PX + 0.001,
        )
        training = observations[:10]
        validation = observations[10:]

        status, failures = MODULE.classify(
            training,
            validation,
            MODULE.residual_summary(training, camera, target),
            MODULE.residual_summary(validation, camera, target),
        )

        self.assertEqual(status, "REJECTED_EYE_IN_HAND_CALIBRATION")
        self.assertIn(
            "training PnP reprojection error exceeds threshold",
            failures,
        )

    def test_residual_is_zero_for_self_consistent_observations(self):
        observations, camera, target = self.synthetic_observations()
        for observation in observations:
            translation, rotation = MODULE.transform_residual(
                observation, camera, target
            )
            self.assertLess(translation, 1e-9)
            self.assertLess(rotation, 1e-9)

    def test_validated_right_registration_returns_ordered_offsets(self):
        candidate = self.valid_right_registration()
        offsets, registration = (
            MODULE.validated_right_joint_zero_offsets(candidate)
        )

        np.testing.assert_allclose(offsets, [0.0, 0.01, 0.02, 0.03, 0.04])
        self.assertTrue(registration["training_only_fit"])

    def test_right_registration_rejects_validation_leakage(self):
        candidate = self.valid_right_registration()
        candidate["right_kinematic_registration"][
            "validation_used_in_fit"
        ] = True

        with self.assertRaisesRegex(RuntimeError, "leaked validation"):
            MODULE.validated_right_joint_zero_offsets(candidate)

    def test_right_registration_rejects_incomplete_joint_set(self):
        candidate = self.valid_right_registration()
        del candidate["right_kinematic_registration"][
            "joint_zero_offsets_rad"
        ]["right_wrist_roll_joint"]

        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            MODULE.validated_right_joint_zero_offsets(candidate)

    def test_right_session_requires_resident_torque_hold_provenance(self):
        session = self.valid_right_session()
        MODULE.validate_session_capture_provenance(session, "right")

        session["training_captures"][0]["source_capture_mode"] = (
            "stationary_read_only"
        )
        with self.assertRaisesRegex(RuntimeError, "lacks consistent"):
            MODULE.validate_session_capture_provenance(session, "right")

    def test_legacy_right_session_is_rejected_before_solving(self):
        session = self.valid_right_session()
        session["capture_mode"] = "stationary_read_only"
        with self.assertRaisesRegex(RuntimeError, "resident torque hold"):
            MODULE.validate_session_capture_provenance(session, "right")


if __name__ == "__main__":
    unittest.main()
