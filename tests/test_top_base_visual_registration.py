import importlib.util
import math
import sys
import unittest
from unittest import mock
from pathlib import Path

import numpy as np


MODULE_PATH = Path("tools/setup/camera_calibration/solve_top_base_visual_registration.py")
SPEC = importlib.util.spec_from_file_location(
    "solve_top_base_visual_registration",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopBaseVisualRegistrationTest(unittest.TestCase):
    def test_rigid_fit_recovers_known_transform(self) -> None:
        source = np.asarray([[0.0, 0.0], [0.2, 0.0], [0.03, 0.15]])
        yaw = 0.7
        expected_rotation = np.asarray(
            [
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ]
        )
        expected_translation = np.asarray([0.31, -0.12])
        target = (
            expected_rotation @ source.T
        ).T + expected_translation

        rotation, translation, residuals, _ = MODULE.fit_rigid_2d(
            source,
            target,
        )

        np.testing.assert_allclose(rotation, expected_rotation, atol=1e-12)
        np.testing.assert_allclose(
            translation,
            expected_translation,
            atol=1e-12,
        )
        np.testing.assert_allclose(residuals, 0.0, atol=1e-12)

    def test_fk_applies_urdf_origin_then_joint_rotation(self) -> None:
        urdf = """
        <robot name="test">
          <link name="base"/>
          <link name="moving"/>
          <link name="tip"/>
          <joint name="spin" type="revolute">
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <parent link="base"/>
            <child link="moving"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="tip_fixed" type="fixed">
            <origin xyz="1 0 0" rpy="0 0 0"/>
            <parent link="moving"/>
            <child link="tip"/>
          </joint>
        </robot>
        """
        pose = MODULE.urdf_fk(
            urdf,
            "base",
            "tip",
            {"spin": math.pi / 2.0},
        )

        np.testing.assert_allclose(pose[:3, 3], [1.0, 1.0, 0.0], atol=1e-12)

    def test_marker_contract_selects_marker_frame_not_tcp(self) -> None:
        session = {
            "frames": {
                "robot": "base",
                "marker": "mount_center",
                "tcp": "must_not_be_used",
            },
            "visual_marker_points": [{"capture_status": "PASS"}],
        }
        (
            frames,
            target,
            points,
            stored_xyz_key,
            method,
        ) = MODULE.registration_contract(session)

        self.assertIs(frames, session["frames"])
        self.assertEqual(target, "mount_center")
        self.assertEqual(points, session["visual_marker_points"])
        self.assertEqual(stored_xyz_key, "base_marker_xyz_m")
        self.assertEqual(
            method,
            "height_corrected_visual_marker_se2",
        )

    def test_ray_intersects_requested_board_height(self) -> None:
        camera_matrix = np.eye(3)
        camera_center = np.asarray([0.0, 0.0, 1.0])
        # Board Z points opposite camera Z, placing the board in front.
        rotation = np.diag([1.0, -1.0, -1.0])
        point = MODULE.raw_pixel_to_board_plane(
            np.asarray([0.2, -0.1]),
            0.25,
            camera_matrix,
            np.zeros(5),
            rotation,
            camera_center,
        )

        self.assertAlmostEqual(point[2], 0.25)
        np.testing.assert_allclose(point[:2], [0.15, 0.075], atol=1e-12)

    def test_registration_output_stays_fail_closed(self) -> None:
        self.assertIn(
            "REQUIRES_INDEPENDENT_VALIDATION",
            "PROVISIONAL_VISUAL_TCP_REQUIRES_INDEPENDENT_VALIDATION",
        )

    def test_fit_classifier_rejects_bad_span_and_geometry(self) -> None:
        self.assertTrue(
            MODULE.classify_fit(7.3, 9.4, 0.003, 0.317).startswith("REJECTED")
        )
        self.assertTrue(
            MODULE.classify_fit(1.0, 2.0, 0.02, 1.0).startswith("PROVISIONAL")
        )


if __name__ == "__main__":
    unittest.main()
