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
    "solve_top_eye_to_hand",
    TOOLS / "solve_top_eye_to_hand.py",
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


class TopEyeToHandTest(unittest.TestCase):
    def synthetic_observations(self):
        base_to_camera = transform(
            [0.20, -0.10, 0.40],
            [0.10, 0.20, 0.50],
        )
        gripper_to_target = transform(
            [-0.10, 0.30, 0.20],
            [0.02, -0.03, 0.08],
        )
        random = np.random.default_rng(42)
        observations = []
        for index in range(12):
            base_to_gripper = transform(
                random.uniform(-0.8, 0.8, 3),
                random.uniform(
                    [0.05, -0.15, 0.10],
                    [0.30, 0.15, 0.35],
                ),
            )
            camera_to_target = (
                MODULE.invert_transform(base_to_camera)
                @ base_to_gripper
                @ gripper_to_target
            )
            observations.append(
                MODULE.PoseObservation(
                    capture_id=f"S{index:02d}",
                    base_to_gripper=base_to_gripper,
                    camera_to_target=camera_to_target,
                    pnp_rms_px=0.1,
                    image_border_px=50.0,
                    detected_marker_ids=(0, 1, 2, 3),
                )
            )
        return observations, base_to_camera, gripper_to_target

    def test_solver_recovers_both_unknown_transforms(self):
        observations, expected_camera, expected_target = (
            self.synthetic_observations()
        )
        actual_camera, actual_target = MODULE.solve_eye_to_hand(observations)
        camera_translation, camera_rotation = transform_error(
            expected_camera,
            actual_camera,
        )
        target_translation, target_rotation = transform_error(
            expected_target,
            actual_target,
        )
        self.assertLess(camera_translation, 1e-9)
        self.assertLess(camera_rotation, 1e-9)
        self.assertLess(target_translation, 1e-9)
        self.assertLess(target_rotation, 1e-9)

    def test_clean_training_and_validation_pass_but_never_authorize_motion(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:10]
        validation = observations[10:]
        training_summary = MODULE.residual_summary(
            training,
            camera,
            target,
        )
        validation_summary = MODULE.residual_summary(
            validation,
            camera,
            target,
        )
        status, failures = MODULE.classify(
            training,
            validation,
            training_summary,
            validation_summary,
        )
        self.assertEqual(
            status,
            "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        )
        self.assertEqual(failures, [])

    def test_small_pose_distribution_is_rejected(self):
        observations, camera, target = self.synthetic_observations()
        collapsed = [
            MODULE.PoseObservation(
                capture_id=observation.capture_id,
                base_to_gripper=transform(
                    [0.0, 0.0, math.radians(index)],
                    [0.20 + index * 0.001, 0.0, 0.20],
                ),
                camera_to_target=observation.camera_to_target,
                pnp_rms_px=0.1,
                image_border_px=50.0,
                detected_marker_ids=(0, 1, 2, 3),
            )
            for index, observation in enumerate(observations)
        ]
        summary = MODULE.residual_summary(
            collapsed,
            camera,
            target,
        )
        status, failures = MODULE.classify(
            collapsed[:10],
            collapsed[10:],
            summary,
            summary,
        )
        self.assertEqual(status, "REJECTED_EYE_TO_HAND_CALIBRATION")
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
            "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
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

        self.assertEqual(status, "REJECTED_EYE_TO_HAND_CALIBRATION")
        self.assertIn(
            "training PnP reprojection error exceeds threshold",
            failures,
        )


if __name__ == "__main__":
    unittest.main()
