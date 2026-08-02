import pytest

from so101_top_perception.runtime_monitor import (
    InferenceMetrics,
    InferenceRateLimiter,
    pose_confidence,
)


def test_rate_limiter_selects_at_most_target_rate() -> None:
    limiter = InferenceRateLimiter(4.0)

    assert limiter.should_run(10.0)
    assert not limiter.should_run(10.1)
    assert limiter.should_run(10.25)
    assert not limiter.should_run(10.49)
    assert limiter.should_run(10.50)


def test_rate_limiter_rejects_nonpositive_rate() -> None:
    with pytest.raises(ValueError, match="positive"):
        InferenceRateLimiter(0.0)


def test_rate_limiter_averages_four_hz_from_six_hz_camera() -> None:
    limiter = InferenceRateLimiter(4.0)
    selected = [
        sampled_at
        for sampled_at in (index / 6.0 for index in range(60))
        if limiter.should_run(sampled_at)
    ]

    assert len(selected) == 40


def test_inference_metrics_separate_expected_rejection_from_error() -> None:
    metrics = InferenceMetrics()
    metrics.record(10.0, "success")
    metrics.record(20.0, "rejection")
    metrics.record(30.0, "error")
    metrics.record_skipped_frame()
    metrics.record_input_rejection()
    metrics.record_input_processing_error()

    assert metrics.inference_count == 3
    assert metrics.successful_observation_count == 1
    assert metrics.detection_rejection_count == 1
    assert metrics.processing_error_count == 1
    assert metrics.skipped_frame_count == 1
    assert metrics.input_rejection_count == 1
    assert metrics.input_processing_error_count == 1
    assert metrics.latency_summary()["p95_ms"] == pytest.approx(29.0)


def test_pose_confidence_accepts_both_detector_contracts() -> None:
    assert pose_confidence({"confidence": 0.9}) == pytest.approx(0.9)
    assert pose_confidence({"solidity": 0.8}) == pytest.approx(0.8)
    assert pose_confidence({"confidence": 2.0}) == 1.0
