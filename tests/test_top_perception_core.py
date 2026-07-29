import unittest

from tests.test_top_object_pose import MODULE


class TopPerceptionFrameAgeTest(unittest.TestCase):
    def test_fresh_source_timestamp_returns_age(self) -> None:
        age = MODULE.shared_detector.frame_age_seconds(
            now_nanoseconds=10_100_000_000,
            stamp_seconds=10,
            stamp_nanoseconds=0,
            max_frame_age_s=0.2,
            future_tolerance_s=0.05,
        )

        self.assertAlmostEqual(age, 0.1)

    def test_stale_source_timestamp_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            MODULE.shared_detector.DetectionError,
            "exceeds",
        ) as context:
            MODULE.shared_detector.frame_age_seconds(
                now_nanoseconds=10_300_000_000,
                stamp_seconds=10,
                stamp_nanoseconds=0,
                max_frame_age_s=0.2,
                future_tolerance_s=0.05,
            )

        self.assertEqual(context.exception.code, "STALE_FRAME")

    def test_missing_source_timestamp_is_rejected(self) -> None:
        with self.assertRaises(
            MODULE.shared_detector.DetectionError,
        ) as context:
            MODULE.shared_detector.frame_age_seconds(
                now_nanoseconds=10_000_000_000,
                stamp_seconds=0,
                stamp_nanoseconds=0,
                max_frame_age_s=0.2,
                future_tolerance_s=0.05,
            )

        self.assertEqual(context.exception.code, "MISSING_TIMESTAMP")

    def test_excessively_future_timestamp_is_rejected(self) -> None:
        with self.assertRaises(
            MODULE.shared_detector.DetectionError,
        ) as context:
            MODULE.shared_detector.frame_age_seconds(
                now_nanoseconds=10_000_000_000,
                stamp_seconds=10,
                stamp_nanoseconds=100_000_000,
                max_frame_age_s=0.2,
                future_tolerance_s=0.05,
            )

        self.assertEqual(context.exception.code, "CLOCK_SKEW")


if __name__ == "__main__":
    unittest.main()
