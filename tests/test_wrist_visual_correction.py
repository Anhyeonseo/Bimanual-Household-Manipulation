import importlib.util
import math
import sys
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "tools"
SPEC = importlib.util.spec_from_file_location(
    "wrist_visual_correction",
    TOOLS / "wrist_visual_correction.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def translation(x: float, y: float, z: float):
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def yaw_rotation(angle_rad: float):
    cos, sin = math.cos(angle_rad), math.sin(angle_rad)
    return [
        [cos, -sin, 0.0, 0.0],
        [sin, cos, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def policy(**overrides):
    return MODULE.WristCorrectionPolicy(arm="left", **overrides)


def observation(**overrides):
    defaults = {
        "camera_to_object_xyz_m": (0.0, 0.0, 0.10),
        "frame_age_s": 0.05,
        "confidence": 0.9,
    }
    defaults.update(overrides)
    return MODULE.WristObservation(**defaults)


class WristVisualCorrectionTest(unittest.TestCase):
    def test_observed_position_composes_both_transforms(self):
        # gripper 0.30 m out along base x, camera another 0.02 m along y,
        # object 0.10 m in front of the camera along z.
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.0, 0.0, 0.10)),
            translation(0.30, 0.0, 0.20),
            translation(0.0, 0.02, 0.0),
            nominal_base_xy_m=(0.30, 0.02),
        )
        for actual, expected in zip(
            decision.observed_base_xyz_m, (0.30, 0.02, 0.30), strict=True
        ):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_correction_within_band_is_applied(self):
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.010, 0.0, 0.0)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.APPLY)
        self.assertTrue(decision.requires_replan)
        self.assertAlmostEqual(decision.correction_magnitude_m, 0.010)
        self.assertAlmostEqual(decision.corrected_base_xy_m[0], 0.010)
        self.assertAlmostEqual(decision.correction_mm(), 10.0)

    def test_correction_below_calibration_noise_floor_holds(self):
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.002, 0.0, 0.0)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.HOLD)
        self.assertFalse(decision.requires_replan)
        self.assertIn("chase", decision.reason)
        # The measurement is still reported; holding is not the same as
        # discarding what was seen.
        self.assertAlmostEqual(decision.correction_magnitude_m, 0.002)

    def test_oversized_correction_is_rejected_as_a_fault(self):
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.050, 0.0, 0.0)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("detection or calibration fault", decision.reason)

    def test_z_is_reported_but_never_corrected(self):
        # A large Z discrepancy must not change the correction, because the
        # wrist camera's depth estimate is the least trustworthy axis.
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.012, 0.0, 0.500)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.APPLY)
        self.assertAlmostEqual(decision.correction_magnitude_m, 0.012)
        self.assertEqual(len(decision.correction_xy_m), 2)
        self.assertAlmostEqual(decision.observed_base_xyz_m[2], 0.500)

    def test_multiple_or_zero_detections_are_rejected_before_any_maths(self):
        for count in (0, 2):
            decision = MODULE.evaluate(
                policy(),
                observation(detection_count=count),
                IDENTITY,
                IDENTITY,
                nominal_base_xy_m=(0.0, 0.0),
            )
            self.assertEqual(decision.action, MODULE.REJECT)
            self.assertIn("exactly one", decision.reason)
            # Fail-closed means no coordinates are published at all.
            self.assertIsNone(decision.observed_base_xyz_m)

    def test_stale_frame_is_rejected(self):
        decision = MODULE.evaluate(
            policy(),
            observation(frame_age_s=0.5),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("old", decision.reason)

    def test_low_confidence_is_rejected(self):
        decision = MODULE.evaluate(
            policy(),
            observation(confidence=0.5),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("confidence", decision.reason)

    def test_non_finite_observation_is_rejected(self):
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(float("nan"), 0.0, 0.0)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("non-finite", decision.reason)

    def test_corrected_target_outside_workspace_is_rejected(self):
        decision = MODULE.evaluate(
            policy(workspace_x_m=(0.20, 0.46), workspace_y_m=(-0.30, 0.08)),
            observation(camera_to_object_xyz_m=(0.020, 0.0, 0.0)),
            translation(0.45, 0.0, 0.0),
            IDENTITY,
            nominal_base_xy_m=(0.45, 0.0),
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("outside the validated workspace", decision.reason)
        # The rejected value is still reported so the operator can see it.
        self.assertAlmostEqual(decision.corrected_base_xy_m[0], 0.470)

    def test_corrected_target_inside_workspace_is_applied(self):
        decision = MODULE.evaluate(
            policy(workspace_x_m=(0.20, 0.46), workspace_y_m=(-0.30, 0.08)),
            observation(camera_to_object_xyz_m=(0.010, 0.0, 0.0)),
            translation(0.35, -0.10, 0.0),
            IDENTITY,
            nominal_base_xy_m=(0.35, -0.10),
        )
        self.assertEqual(decision.action, MODULE.APPLY)

    def test_gripper_rotation_is_respected_when_mapping_to_base(self):
        # With the gripper yawed 90 degrees, a camera-frame +x offset must
        # land on base +y. Getting this wrong would push corrections
        # perpendicular to the real error.
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.010, 0.0, 0.0)),
            yaw_rotation(math.pi / 2.0),
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.APPLY)
        self.assertAlmostEqual(decision.correction_xy_m[0], 0.0, places=9)
        self.assertAlmostEqual(decision.correction_xy_m[1], 0.010, places=9)

    def test_yaw_is_passed_through_untouched(self):
        decision = MODULE.evaluate(
            policy(),
            observation(
                camera_to_object_xyz_m=(0.010, 0.0, 0.0),
                yaw_rad=-0.358,
            ),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        # Solving wrist_roll from this yaw belongs to grasp_yaw_kinematics;
        # this module only carries the measurement across the boundary.
        self.assertAlmostEqual(decision.observed_yaw_rad, -0.358)

    def test_floor_suppresses_every_measured_spurious_correction(self):
        # Measured 2026-08-09: the W3 session's fixed ArUco board was run
        # through this very evaluator from all 10 captures. The board never
        # moved, so every non-zero correction below is calibration + arm-FK
        # error, not a moved object. The floor exists to suppress exactly
        # these, and the worst of them (6.42 mm) is what sets it.
        measured_spurious_mm = (
            0.95, 1.55, 1.66, 1.75, 2.08, 3.00, 4.67, 5.64, 5.99, 6.42,
        )
        for magnitude_mm in measured_spurious_mm:
            decision = MODULE.evaluate(
                policy(),
                observation(
                    camera_to_object_xyz_m=(magnitude_mm / 1000.0, 0.0, 0.0)
                ),
                IDENTITY,
                IDENTITY,
                nominal_base_xy_m=(0.0, 0.0),
            )
            self.assertEqual(
                decision.action,
                MODULE.HOLD,
                f"{magnitude_mm} mm spurious correction must not be applied",
            )
        # A real offset comfortably above that noise band still gets through,
        # otherwise the floor would have made the module useless.
        decision = MODULE.evaluate(
            policy(),
            observation(camera_to_object_xyz_m=(0.015, 0.0, 0.0)),
            IDENTITY,
            IDENTITY,
            nominal_base_xy_m=(0.0, 0.0),
        )
        self.assertEqual(decision.action, MODULE.APPLY)

    def test_policy_rejects_a_missing_arm_name(self):
        with self.assertRaisesRegex(ValueError, "arm name is required"):
            MODULE.WristCorrectionPolicy(arm="")

    def test_policy_rejects_an_inverted_correction_band(self):
        with self.assertRaisesRegex(ValueError, "must be tighter"):
            policy(minimum_correction_m=0.05, maximum_correction_m=0.03)

    def test_policy_rejects_an_inverted_workspace(self):
        with self.assertRaisesRegex(ValueError, "lower bound must be below"):
            policy(workspace_x_m=(0.46, 0.20))

    def test_policy_rejects_an_impossible_confidence(self):
        with self.assertRaisesRegex(ValueError, "minimum_confidence"):
            policy(minimum_confidence=1.5)


if __name__ == "__main__":
    unittest.main()
