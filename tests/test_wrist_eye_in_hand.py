import importlib.util
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


TOOLS = Path(__file__).resolve().parents[1] / "tools"
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
            replace(observation, pnp_rms_px=1.5)
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
        observations[0] = replace(observations[0], pnp_rms_px=1.501)
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


if __name__ == "__main__":
    unittest.main()
