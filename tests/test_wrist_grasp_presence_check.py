import importlib.util
import sys
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "tools"
SPEC = importlib.util.spec_from_file_location(
    "wrist_grasp_presence_check",
    TOOLS / "wrist_grasp_presence_check.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def policy(**overrides):
    defaults = {"arm": "left", "minimum_occupancy_score": 0.5}
    defaults.update(overrides)
    return MODULE.GraspCheckPolicy(**defaults)


def observation(**overrides):
    defaults = {
        "occupancy_score": 0.8,
        "frame_age_s": 0.05,
        "confidence": 0.9,
    }
    defaults.update(overrides)
    return MODULE.GraspCheckObservation(**defaults)


class WristGraspPresenceCheckTest(unittest.TestCase):
    def test_score_above_threshold_is_present(self):
        decision = MODULE.evaluate(
            policy(), observation(occupancy_score=0.8), MODULE.PRE_CLOSE
        )
        self.assertEqual(decision.action, MODULE.PRESENT)
        self.assertTrue(decision.confirmed_present)
        self.assertTrue(decision.trustworthy)

    def test_score_below_threshold_is_absent(self):
        decision = MODULE.evaluate(
            policy(), observation(occupancy_score=0.2), MODULE.PRE_CLOSE
        )
        self.assertEqual(decision.action, MODULE.ABSENT)
        self.assertFalse(decision.confirmed_present)
        self.assertTrue(decision.trustworthy)

    def test_score_exactly_at_threshold_is_present(self):
        decision = MODULE.evaluate(
            policy(minimum_occupancy_score=0.5),
            observation(occupancy_score=0.5),
            MODULE.POST_LIFT,
        )
        self.assertEqual(decision.action, MODULE.PRESENT)

    def test_unknown_checkpoint_is_rejected_before_any_maths(self):
        with self.assertRaisesRegex(ValueError, "checkpoint must be one of"):
            MODULE.evaluate(policy(), observation(), "mid_flight")

    def test_multiple_or_zero_detections_are_rejected(self):
        for count in (0, 2):
            decision = MODULE.evaluate(
                policy(), observation(detection_count=count), MODULE.PRE_CLOSE
            )
            self.assertEqual(decision.action, MODULE.REJECT)
            self.assertIn("exactly one", decision.reason)
            self.assertFalse(decision.trustworthy)
            self.assertIsNone(decision.occupancy_score)

    def test_stale_frame_is_rejected(self):
        decision = MODULE.evaluate(
            policy(), observation(frame_age_s=0.5), MODULE.PRE_CLOSE
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("old", decision.reason)

    def test_low_confidence_is_rejected(self):
        decision = MODULE.evaluate(
            policy(), observation(confidence=0.5), MODULE.PRE_CLOSE
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("confidence", decision.reason)

    def test_non_finite_score_is_rejected(self):
        decision = MODULE.evaluate(
            policy(),
            observation(occupancy_score=float("nan")),
            MODULE.PRE_CLOSE,
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("finite", decision.reason)

    def test_out_of_range_score_is_rejected(self):
        decision = MODULE.evaluate(
            policy(), observation(occupancy_score=1.5), MODULE.PRE_CLOSE
        )
        self.assertEqual(decision.action, MODULE.REJECT)
        self.assertIn("outside [0, 1]", decision.reason)

    def test_policy_requires_a_threshold_with_no_default(self):
        with self.assertRaises(TypeError):
            MODULE.GraspCheckPolicy(arm="left")

    def test_policy_rejects_a_missing_arm_name(self):
        with self.assertRaisesRegex(ValueError, "arm name is required"):
            policy(arm="")

    def test_policy_rejects_a_boundary_threshold(self):
        with self.assertRaisesRegex(ValueError, "within \\(0, 1\\)"):
            policy(minimum_occupancy_score=1.0)
        with self.assertRaisesRegex(ValueError, "within \\(0, 1\\)"):
            policy(minimum_occupancy_score=0.0)

    def test_policy_rejects_an_impossible_confidence(self):
        with self.assertRaisesRegex(ValueError, "minimum_confidence"):
            policy(minimum_confidence=1.5)


class FuseWithGapCheckTest(unittest.TestCase):
    def _decision(self, action, occupancy_score=0.8):
        return MODULE.GraspCheckDecision(
            arm="left",
            checkpoint=MODULE.PRE_CLOSE,
            action=action,
            reason="test",
            occupancy_score=occupancy_score,
        )

    def test_both_present_proceeds(self):
        fused = MODULE.fuse_with_gap_check(
            self._decision(MODULE.PRESENT), gap_confirmed=True
        )
        self.assertEqual(fused.action, MODULE.PROCEED)

    def test_both_absent_stops_empty(self):
        fused = MODULE.fuse_with_gap_check(
            self._decision(MODULE.ABSENT), gap_confirmed=False
        )
        self.assertEqual(fused.action, MODULE.STOP_EMPTY)

    def test_disagreement_stops_as_contradiction_not_a_guess(self):
        fused = MODULE.fuse_with_gap_check(
            self._decision(MODULE.PRESENT), gap_confirmed=False
        )
        self.assertEqual(fused.action, MODULE.STOP_CONTRADICTION)
        self.assertIn("do not guess", fused.reason)

        fused = MODULE.fuse_with_gap_check(
            self._decision(MODULE.ABSENT), gap_confirmed=True
        )
        self.assertEqual(fused.action, MODULE.STOP_CONTRADICTION)

    def test_untrustworthy_camera_falls_back_to_gap_alone(self):
        rejected = self._decision(MODULE.REJECT, occupancy_score=None)
        fused = MODULE.fuse_with_gap_check(rejected, gap_confirmed=True)
        self.assertEqual(fused.action, MODULE.DEGRADED_GAP_ONLY)
        self.assertIn("confirmed=True", fused.reason)


if __name__ == "__main__":
    unittest.main()
