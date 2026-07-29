import math

import pytest

from so101_top_perception.detector import (
    DetectionError,
    frame_age_seconds,
    normalize_axis_yaw,
)


def test_fresh_frame_age() -> None:
    age = frame_age_seconds(
        now_nanoseconds=10_100_000_000,
        stamp_seconds=10,
        stamp_nanoseconds=0,
        max_frame_age_s=0.2,
        future_tolerance_s=0.05,
    )

    assert age == pytest.approx(0.1)


def test_stale_frame_is_rejected() -> None:
    with pytest.raises(DetectionError) as context:
        frame_age_seconds(
            now_nanoseconds=10_300_000_000,
            stamp_seconds=10,
            stamp_nanoseconds=0,
            max_frame_age_s=0.2,
            future_tolerance_s=0.05,
        )

    assert context.value.code == "STALE_FRAME"


def test_missing_timestamp_is_rejected() -> None:
    with pytest.raises(DetectionError) as context:
        frame_age_seconds(
            now_nanoseconds=10_000_000_000,
            stamp_seconds=0,
            stamp_nanoseconds=0,
            max_frame_age_s=0.2,
            future_tolerance_s=0.05,
        )

    assert context.value.code == "MISSING_TIMESTAMP"


def test_undirected_yaw_is_normalized_modulo_pi() -> None:
    assert normalize_axis_yaw(math.radians(170.0)) == pytest.approx(
        math.radians(-10.0)
    )
