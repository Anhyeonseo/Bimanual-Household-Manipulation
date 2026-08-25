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

    def test_resident_torque_hold_contract_allows_six_pose_training_gate(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:6]
        validation = observations[10:]
        session = {
            "capture_mode": MODULE.RESIDENT_TORQUE_HOLD_CAPTURE_MODE,
            "source_motion_authorized": True,
            "training_captures": [],
            "validation_captures": [],
        }
        for index in range(8):
            session[
                "training_captures" if index < 6 else "validation_captures"
            ].append(
                {
                    "source_capture_mode": (
                        MODULE.RESIDENT_TORQUE_HOLD_CAPTURE_MODE
                    ),
                    "source_motion_authorized": True,
                    "resident_torque_hold": {
                        "status_service": MODULE.RESIDENT_STATUS_SERVICE,
                        "owner": "resident_right_calibration_operator",
                        "arbiter_epoch": index + 1,
                        "torque_hold_active": True,
                        "terminal_anchor_stamp": 100.0 + index,
                        "required_owner": (
                            "resident_right_calibration_operator"
                        ),
                        "required_epoch": index + 1,
                    },
                }
            )

        mode = MODULE.session_capture_mode(session)
        minimum = MODULE.minimum_training_captures_for_mode(mode)
        status, failures = MODULE.classify(
            training,
            validation,
            MODULE.residual_summary(training, camera, target),
            MODULE.residual_summary(validation, camera, target),
            minimum,
        )

        self.assertEqual(minimum, 6)
        self.assertEqual(
            status,
            "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
        )
        self.assertEqual(failures, [])

    def test_resident_torque_hold_contract_rejects_epoch_mismatch(self):
        session = {
            "capture_mode": MODULE.RESIDENT_TORQUE_HOLD_CAPTURE_MODE,
            "source_motion_authorized": True,
            "training_captures": [
                {
                    "source_capture_mode": (
                        MODULE.RESIDENT_TORQUE_HOLD_CAPTURE_MODE
                    ),
                    "source_motion_authorized": True,
                    "resident_torque_hold": {
                        "status_service": MODULE.RESIDENT_STATUS_SERVICE,
                        "owner": "owner",
                        "arbiter_epoch": 4,
                        "torque_hold_active": True,
                        "terminal_anchor_stamp": 100.0,
                        "required_owner": "owner",
                        "required_epoch": 5,
                    },
                }
            ],
            "validation_captures": [],
        }

        with self.assertRaisesRegex(RuntimeError, "inconsistent"):
            MODULE.session_capture_mode(session)

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
            "EYE_TO_HAND_VALIDATED_MOTION_STILL_NOT_AUTHORIZED",
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

        self.assertEqual(status, "REJECTED_EYE_TO_HAND_CALIBRATION")
        self.assertIn(
            "training PnP reprojection error exceeds threshold",
            failures,
        )

    def test_task_based_residual_limits_are_inclusive(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:10]
        validation = observations[10:]
        training_summary = MODULE.residual_summary(training, camera, target)
        training_summary.update(
            translation_rms_mm=5.0,
            translation_max_mm=8.0,
            rotation_rms_deg=1.5,
            rotation_max_deg=3.0,
        )
        validation_summary = MODULE.residual_summary(validation, camera, target)
        validation_summary.update(
            translation_max_mm=8.0,
            rotation_max_deg=3.0,
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

    def test_centimetre_scale_training_outlier_is_rejected(self):
        observations, camera, target = self.synthetic_observations()
        training = observations[:10]
        validation = observations[10:]
        training_summary = MODULE.residual_summary(training, camera, target)
        training_summary.update(
            translation_rms_mm=5.6,
            translation_max_mm=13.0,
        )

        status, failures = MODULE.classify(
            training,
            validation,
            training_summary,
            MODULE.residual_summary(validation, camera, target),
        )

        self.assertEqual(status, "REJECTED_EYE_TO_HAND_CALIBRATION")
        self.assertIn("training translation RMS exceeds threshold", failures)
        self.assertIn("training translation max exceeds threshold", failures)

    def test_constrained_right_registration_recovers_internal_zero_offsets(self):
        urdf = """
<robot name="synthetic_right">
  <link name="right_base_link"/>
  <link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="l4"/><link name="right_gripper_frame_link"/>
  <joint name="right_base_joint" type="revolute">
    <parent link="right_base_link"/><child link="l1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0.03 0 0.04" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_elbow_joint" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0.12 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_wrist_flex_joint" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_wrist_roll_joint" type="revolute">
    <parent link="l4"/><child link="right_gripper_frame_link"/>
    <origin xyz="0.06 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  </joint>
</robot>
"""
        names = MODULE.ARM_JOINT_NAMES_BY_SIDE["right"]
        true_offsets = np.asarray([0.0, 0.04, -0.06, 0.05, 0.0])
        workcell_to_camera = transform(
            [0.1, -0.2, 0.3],
            [0.05, 0.10, 0.55],
        )
        workcell_to_right_base = MODULE.make_transform(
            np.eye(3),
            MODULE.RIGHT_MOUNT_PRIOR_XYZ_M,
        )
        gripper_to_target = transform(
            [0.2, -0.1, 0.15],
            [0.03, -0.02, 0.05],
        )
        random = np.random.default_rng(7)
        captures = []
        observations = []
        for index in range(16):
            measured = random.uniform(-0.8, 0.8, len(names))
            nominal_fk = MODULE.urdf_fk(
                urdf,
                "right_base_link",
                "right_gripper_frame_link",
                dict(zip(names, measured, strict=True)),
            )
            actual_fk = MODULE.urdf_fk(
                urdf,
                "right_base_link",
                "right_gripper_frame_link",
                dict(zip(names, measured + true_offsets, strict=True)),
            )
            camera_to_target = (
                MODULE.invert_transform(workcell_to_camera)
                @ workcell_to_right_base
                @ actual_fk
                @ gripper_to_target
            )
            captures.append(
                {
                    "id": f"R{index:02d}",
                    "measured_arm_rad": measured.tolist(),
                }
            )
            observations.append(
                MODULE.PoseObservation(
                    capture_id=f"R{index:02d}",
                    base_to_gripper=nominal_fk,
                    camera_to_target=camera_to_target,
                    pnp_rms_px=0.1,
                    image_border_px=100.0,
                    detected_marker_ids=(0, 1, 2, 3),
                )
            )

        mount, target, offsets, fit = (
            MODULE.fit_right_joint_zero_registration(
                captures,
                observations,
                urdf,
                "right_base_link",
                "right_gripper_frame_link",
                workcell_to_camera,
                workcell_to_right_base,
            )
        )

        self.assertTrue(fit.success)
        self.assertTrue(
            np.allclose(
                offsets[list(MODULE.RIGHT_IDENTIFIABLE_ZERO_INDICES)],
                true_offsets[list(MODULE.RIGHT_IDENTIFIABLE_ZERO_INDICES)],
                atol=1.2e-2,
            ),
            msg=f"actual offsets: {offsets}",
        )
        adjusted = MODULE.observations_with_joint_zero_offsets(
            captures,
            observations,
            urdf,
            "right_base_link",
            "right_gripper_frame_link",
            names,
            offsets,
        )
        right_to_camera = MODULE.invert_transform(mount) @ workcell_to_camera
        summary = MODULE.residual_summary(adjusted, right_to_camera, target)
        self.assertLess(summary["translation_max_mm"], 0.5)
        self.assertLess(summary["rotation_max_deg"], 0.1)


if __name__ == "__main__":
    unittest.main()
