"""Small runtime helpers shared by the ROS wrapper and unit tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


LEGACY_BACKEND = "legacy_dark_threshold"


class InferenceRateLimiter:
    """Select at most one camera frame per configured inference period."""

    def __init__(self, target_hz: float) -> None:
        if target_hz <= 0.0:
            raise ValueError("inference_hz must be positive")
        self.target_hz = float(target_hz)
        self._period_s = 1.0 / self.target_hz
        self._next_at: float | None = None

    def should_run(self, sampled_at: float) -> bool:
        if self._next_at is not None and sampled_at < self._next_at:
            return False
        if self._next_at is None:
            self._next_at = sampled_at + self._period_s
        else:
            self._next_at += self._period_s
            if self._next_at <= sampled_at:
                self._next_at = sampled_at + self._period_s
        return True


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = fraction * (len(ordered) - 1)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass
class InferenceMetrics:
    """Bounded inference telemetry without retaining camera images."""

    inference_count: int = 0
    successful_observation_count: int = 0
    detection_rejection_count: int = 0
    processing_error_count: int = 0
    input_rejection_count: int = 0
    input_processing_error_count: int = 0
    skipped_frame_count: int = 0
    last_inference_ms: float = 0.0
    _latencies_ms: deque[float] = field(
        default_factory=lambda: deque(maxlen=512),
        repr=False,
    )

    def record(self, latency_ms: float, outcome: str) -> None:
        if latency_ms < 0.0:
            raise ValueError("latency_ms must not be negative")
        if outcome not in {"success", "rejection", "error"}:
            raise ValueError(f"unsupported inference outcome: {outcome}")
        self.inference_count += 1
        self.last_inference_ms = float(latency_ms)
        self._latencies_ms.append(float(latency_ms))
        if outcome == "success":
            self.successful_observation_count += 1
        elif outcome == "rejection":
            self.detection_rejection_count += 1
        else:
            self.processing_error_count += 1

    def record_skipped_frame(self) -> None:
        self.skipped_frame_count += 1

    def record_input_rejection(self) -> None:
        self.input_rejection_count += 1

    def record_input_processing_error(self) -> None:
        self.input_processing_error_count += 1

    def latency_summary(self) -> dict[str, float]:
        values = list(self._latencies_ms)
        return {
            "p50_ms": percentile(values, 0.50),
            "p95_ms": percentile(values, 0.95),
            "max_ms": max(values, default=0.0),
        }


def pose_confidence(pose: dict) -> float:
    """Normalize confidence from either supported detector contract."""
    value = pose.get("confidence", pose.get("solidity"))
    if value is None:
        raise ValueError("detector pose does not contain confidence")
    return min(1.0, max(0.0, float(value)))
