import copy
import json
import unittest
from pathlib import Path

from tools.run.validate_camera_schedule import validate_schedule


SCHEDULE_PATH = Path(__file__).parents[1] / "config" / "camera_schedule.json"


def load_schedule():
    return json.loads(SCHEDULE_PATH.read_text(encoding="utf-8"))


class ValidateCameraScheduleTests(unittest.TestCase):
    def test_repository_schedule_passes(self):
        self.assertEqual(validate_schedule(load_schedule()), [])

    def test_total_inference_over_budget_fails(self):
        schedule = load_schedule()
        schedule["phases"]["SEARCH"]["wrist_a"] = {"decode_hz": 10, "inference_hz": 10}
        errors = validate_schedule(schedule)
        self.assertTrue(any("total inference" in error for error in errors))

    def test_inference_cannot_exceed_decode(self):
        schedule = load_schedule()
        schedule["phases"]["SEARCH"]["top"] = {"decode_hz": 5, "inference_hz": 8}
        errors = validate_schedule(schedule)
        self.assertTrue(any("exceeds decode_hz" in error for error in errors))

    def test_raw_image_policy_fails(self):
        schedule = copy.deepcopy(load_schedule())
        schedule["policy_runtime"]["raw_image_input"] = True
        errors = validate_schedule(schedule)
        self.assertTrue(any("raw_image_input" in error for error in errors))

    def test_unbounded_queue_fails(self):
        schedule = load_schedule()
        schedule["capture"]["queue_depth"] = 4
        errors = validate_schedule(schedule)
        self.assertTrue(any("queue_depth" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
