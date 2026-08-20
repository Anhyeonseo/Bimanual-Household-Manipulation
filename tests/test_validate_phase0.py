import copy
import unittest

from tools.setup.firmware.validate_phase0 import EXPECTED_JOINTS, validate_baseline


def make_valid_baseline():
    joint_names = sorted(EXPECTED_JOINTS)

    def make_arm():
        return {
            "unexpected_motion_on_powerup": False,
            "adapter_reconnect_ok": True,
            "settings_persist_after_power_cycle": True,
            "wrong_id_silent": True,
            "servos": [
                {
                    "id": servo_id,
                    "joint_name": joint_name,
                    "ping_ok": True,
                    "raw_increases_with_positive_joint": True,
                    "raw_current": 2048,
                    "raw_safe_min": 1000,
                    "raw_safe_max": 3000,
                    "feedback": {"position": True, "speed": True, "load": True, "voltage": True},
                    "abnormal_heat_or_noise": False,
                    "communication_errors": False,
                }
                for servo_id, joint_name in enumerate(joint_names, start=1)
            ],
        }

    return {
        "schema_version": 1,
        "measured_at": "2026-07-12T12:00:00+09:00",
        "operator": "test",
        "power": {
            side: {"rated_voltage_v": 12.0, "rated_current_a": 10.0, "no_load_v": 12.1, "enabled_idle_v": 12.0, "moving_min_v": 11.9}
            for side in ("left", "right")
        },
        "arms": {side: make_arm() for side in ("left", "right")},
    }


class ValidatePhase0Tests(unittest.TestCase):
    def test_complete_baseline_passes(self):
        self.assertEqual(validate_baseline(make_valid_baseline()), [])

    def test_right_only_ignores_unmeasured_left(self):
        baseline = make_valid_baseline()
        baseline["arms"]["left"] = {}
        baseline["power"]["left"] = {}
        self.assertEqual(validate_baseline(baseline, "right"), [])

    def test_duplicate_joint_fails(self):
        baseline = make_valid_baseline()
        baseline["arms"]["right"]["servos"][1]["joint_name"] = baseline["arms"]["right"]["servos"][0]["joint_name"]
        errors = validate_baseline(baseline, "right")
        self.assertTrue(any("exactly once" in error for error in errors))

    def test_unexpected_motion_fails(self):
        baseline = make_valid_baseline()
        baseline["arms"]["right"]["unexpected_motion_on_powerup"] = True
        errors = validate_baseline(baseline, "right")
        self.assertTrue(any("unexpected_motion_on_powerup" in error for error in errors))

    def test_current_outside_safe_range_fails(self):
        baseline = copy.deepcopy(make_valid_baseline())
        baseline["arms"]["right"]["servos"][0]["raw_current"] = 4000
        errors = validate_baseline(baseline, "right")
        self.assertTrue(any("outside the recorded safe range" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
