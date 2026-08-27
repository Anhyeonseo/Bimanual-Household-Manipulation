"""Fail-closed observation lifecycle and episode evidence for R1.

This module never publishes a motion command.  It only decides whether a
window of towel observations is fresh and trustworthy enough to become the
before/after evidence for one bounded primitive attempt.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any, Mapping

from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    StateEstimate,
    TowelObservation,
    TowelTaskContractError,
    stabilize_observations,
)


class ObservationLifecyclePhase(str, Enum):
    OBSERVE_CLEAR = "OBSERVE_CLEAR"
    PRIMITIVE = "PRIMITIVE"
    RETREAT_AND_SETTLE = "RETREAT_AND_SETTLE"
    REOBSERVE_CLEAR = "REOBSERVE_CLEAR"
    FAILED = "FAILED"


class EpisodeOutcome(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    IMPROVED = "IMPROVED"
    NO_CHANGE = "NO_CHANGE"
    REGRESSED = "REGRESSED"
    UNKNOWN = "UNKNOWN"


def _positive_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TowelTaskContractError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise TowelTaskContractError(f"{label} must be finite and positive")
    return result


def _nonnegative_stamp(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise TowelTaskContractError(f"{label} must be a nonnegative integer")
    return value


@dataclass(frozen=True, slots=True)
class ObservationLifecycleLimits:
    maximum_frame_age_ns: int
    minimum_retreat_and_settle_ns: int
    minimum_consecutive_observations: int

    @classmethod
    def from_contract(
        cls, contract: Mapping[str, Any]
    ) -> "ObservationLifecycleLimits":
        values = contract.get("observation_lifecycle")
        if not isinstance(values, Mapping):
            raise TowelTaskContractError(
                "observation_lifecycle must be an object"
            )
        if values.get("status") != "R1_REAL_BURST_VALIDATED":
            raise TowelTaskContractError(
                "observation lifecycle must retain its R1 burst validation"
            )
        if values.get("motion_authorized") is not False:
            raise TowelTaskContractError(
                "observation lifecycle must keep motion_authorized=false"
            )
        maximum_age_ms = _positive_number(
            values.get("maximum_frame_age_ms"), "maximum_frame_age_ms"
        )
        minimum_settle_ms = _positive_number(
            values.get("minimum_retreat_and_settle_ms"),
            "minimum_retreat_and_settle_ms",
        )
        minimum_consecutive = values.get("minimum_consecutive_observations")
        if (
            isinstance(minimum_consecutive, bool)
            or not isinstance(minimum_consecutive, int)
            or minimum_consecutive < 2
        ):
            raise TowelTaskContractError(
                "minimum_consecutive_observations must be an integer >= 2"
            )
        if values.get("require_settled") is not True:
            raise TowelTaskContractError("settled observations must be required")
        if values.get("require_clear_pose_verified") is not True:
            raise TowelTaskContractError(
                "verified clear pose must be required"
            )
        if values.get("require_clear_view_valid") is not True:
            raise TowelTaskContractError(
                "a valid clear view must be required"
            )
        return cls(
            maximum_frame_age_ns=round(maximum_age_ms * 1_000_000),
            minimum_retreat_and_settle_ns=round(
                minimum_settle_ms * 1_000_000
            ),
            minimum_consecutive_observations=minimum_consecutive,
        )


@dataclass(frozen=True, slots=True)
class ApprovedObservationWindow:
    observation_ids: tuple[str, ...]
    source_sha256s: tuple[str, ...]
    capture_stamps_ns: tuple[int, ...]
    state: str
    state_reason: str

    @classmethod
    def from_window(
        cls,
        window: tuple[TowelObservation, ...],
        estimate: StateEstimate,
    ) -> "ApprovedObservationWindow":
        return cls(
            observation_ids=tuple(item.observation_id for item in window),
            source_sha256s=tuple(item.source_sha256 for item in window),
            capture_stamps_ns=tuple(
                int(item.capture_stamp_ns) for item in window
            ),
            state=estimate.state.value,
            state_reason=estimate.reason,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_ids": list(self.observation_ids),
            "source_sha256s": list(self.source_sha256s),
            "capture_stamps_ns": list(self.capture_stamps_ns),
            "state": self.state,
            "state_reason": self.state_reason,
        }


@dataclass(frozen=True, slots=True)
class TowelEpisodeRecord:
    episode_id: str
    action_id: str
    primitive: str
    before: ApprovedObservationWindow
    after: ApprovedObservationWindow
    outcome: EpisodeOutcome
    action_started_ns: int
    action_finished_ns: int
    episode_finished_ns: int
    calibration_sha256: str
    model_sha256: str
    robot_model_sha256: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "record_kind": "towel_observation_episode",
            "episode_id": self.episode_id,
            "motion_authorized": False,
            "motion_commands": 0,
            "execution_api_used": False,
            "identities": {
                "calibration_sha256": self.calibration_sha256,
                "model_sha256": self.model_sha256,
                "robot_model_sha256": self.robot_model_sha256,
            },
            "before": self.before.to_dict(),
            "action": {
                "action_id": self.action_id,
                "primitive": self.primitive,
                "started_ns": self.action_started_ns,
                "finished_ns": self.action_finished_ns,
            },
            "after": self.after.to_dict(),
            "outcome": self.outcome.value,
            "episode_finished_ns": self.episode_finished_ns,
        }


@dataclass(slots=True)
class TowelObservationLifecycle:
    lifecycle_limits: ObservationLifecycleLimits
    perception_limits: PerceptionLimits
    phase: ObservationLifecyclePhase = ObservationLifecyclePhase.OBSERVE_CLEAR
    phase_started_ns: int = 0
    rejected_observations: int = 0
    unstable_windows: int = 0
    failure_reason: str | None = None
    episodes: list[TowelEpisodeRecord] = field(default_factory=list)
    _pending: list[TowelObservation] = field(default_factory=list)
    _seen_observation_ids: set[str] = field(default_factory=set)
    _seen_source_sha256s: set[str] = field(default_factory=set)
    _last_capture_stamp_ns: int | None = None
    _calibration_sha256: str | None = None
    _model_sha256: str | None = None
    _robot_model_sha256: str | None = None
    _before: ApprovedObservationWindow | None = None
    _after: ApprovedObservationWindow | None = None
    _action_id: str | None = None
    _primitive: str | None = None
    _action_started_ns: int | None = None
    _action_finished_ns: int | None = None

    @classmethod
    def from_contract(
        cls,
        contract: Mapping[str, Any],
        *,
        started_ns: int = 0,
    ) -> "TowelObservationLifecycle":
        return cls(
            lifecycle_limits=ObservationLifecycleLimits.from_contract(contract),
            perception_limits=PerceptionLimits.from_contract(contract),
            phase_started_ns=_nonnegative_stamp(started_ns, "started_ns"),
        )

    def _reject(self, reason: str) -> None:
        self.rejected_observations += 1
        raise TowelTaskContractError(reason)

    def _require_phase(self, expected: ObservationLifecyclePhase) -> None:
        if self.phase == ObservationLifecyclePhase.FAILED:
            raise TowelTaskContractError("failed observation lifecycle cannot be reused")
        if self.phase != expected:
            raise TowelTaskContractError(
                f"expected lifecycle phase {expected.value}, got {self.phase.value}"
            )

    def _pin_or_check_identities(self, observation: TowelObservation) -> None:
        identities = (
            ("calibration", observation.calibration_sha256),
            ("model", observation.model_sha256),
            ("robot model", observation.robot_model_sha256),
        )
        current = (
            self._calibration_sha256,
            self._model_sha256,
            self._robot_model_sha256,
        )
        if any(value is None for _, value in identities):
            self._reject("observation identity metadata is incomplete")
        if all(value is None for value in current):
            self._calibration_sha256 = str(identities[0][1])
            self._model_sha256 = str(identities[1][1])
            self._robot_model_sha256 = str(identities[2][1])
            return
        for (label, observed), expected in zip(identities, current, strict=True):
            if observed != expected:
                self._reject(f"observation {label} identity changed within session")

    def observe(
        self,
        observation: TowelObservation,
        *,
        now_ns: int,
    ) -> StateEstimate | None:
        """Validate one clear-view frame and approve only a stable window."""
        if self.phase not in {
            ObservationLifecyclePhase.OBSERVE_CLEAR,
            ObservationLifecyclePhase.REOBSERVE_CLEAR,
        }:
            self._reject(
                f"observations are forbidden during {self.phase.value}"
            )
        now = _nonnegative_stamp(now_ns, "now_ns")
        expected_phase = self.phase.value
        if observation.lifecycle_phase != expected_phase:
            self._reject(
                f"observation phase must be {expected_phase}"
            )
        stamp = observation.capture_stamp_ns
        if stamp is None:
            self._reject("capture_stamp_ns is required")
        stamp = _nonnegative_stamp(stamp, "capture_stamp_ns")
        if stamp < self.phase_started_ns:
            self._reject("observation predates the active clear-view phase")
        if stamp > now:
            self._reject("observation timestamp is in the future")
        if now - stamp > self.lifecycle_limits.maximum_frame_age_ns:
            self._reject("observation exceeds the freshness limit")
        if self._last_capture_stamp_ns is not None and stamp <= self._last_capture_stamp_ns:
            self._reject("observation timestamps must increase monotonically")
        if observation.observation_id in self._seen_observation_ids:
            self._reject("observation_id was already consumed")
        if observation.source_sha256 in self._seen_source_sha256s:
            self._reject("source frame was already consumed")
        if observation.stale:
            self._reject("stale observation cannot enter a clear-view window")
        if observation.settled is not True:
            self._reject("observation must be explicitly settled")
        if observation.clear_pose_verified is not True:
            self._reject("clear pose must be explicitly verified")
        if observation.clear_view_valid is not True:
            self._reject("clear view must be explicitly valid")
        self._pin_or_check_identities(observation)

        self._last_capture_stamp_ns = stamp
        self._seen_observation_ids.add(observation.observation_id)
        self._seen_source_sha256s.add(observation.source_sha256)
        self._pending.append(observation)
        minimum = self.lifecycle_limits.minimum_consecutive_observations
        if len(self._pending) < minimum:
            return None
        window = tuple(self._pending[-minimum:])
        try:
            estimate = stabilize_observations(
                window,
                self.perception_limits,
                minimum_consecutive=minimum,
            )
        except TowelTaskContractError:
            self.unstable_windows += 1
            return None
        approved = ApprovedObservationWindow.from_window(window, estimate)
        if self.phase == ObservationLifecyclePhase.OBSERVE_CLEAR:
            self._before = approved
        else:
            self._after = approved
        return estimate

    def begin_primitive(
        self,
        *,
        action_id: str,
        primitive: str,
        now_ns: int,
    ) -> None:
        self._require_phase(ObservationLifecyclePhase.OBSERVE_CLEAR)
        if self._before is None:
            raise TowelTaskContractError(
                "primitive requires an approved before-observation window"
            )
        if not isinstance(action_id, str) or not action_id:
            raise TowelTaskContractError("action_id is required")
        if not isinstance(primitive, str) or not primitive:
            raise TowelTaskContractError("primitive is required")
        now = _nonnegative_stamp(now_ns, "now_ns")
        if now < self._before.capture_stamps_ns[-1]:
            raise TowelTaskContractError("primitive cannot predate its observation")
        self._action_id = action_id
        self._primitive = primitive
        self._action_started_ns = now
        self._action_finished_ns = None
        self._after = None
        self._pending.clear()
        self.phase = ObservationLifecyclePhase.PRIMITIVE
        self.phase_started_ns = now

    def finish_primitive(self, *, now_ns: int) -> None:
        self._require_phase(ObservationLifecyclePhase.PRIMITIVE)
        now = _nonnegative_stamp(now_ns, "now_ns")
        if self._action_started_ns is None or now < self._action_started_ns:
            raise TowelTaskContractError("primitive finish timestamp is invalid")
        self._action_finished_ns = now
        self._pending.clear()
        self.phase = ObservationLifecyclePhase.RETREAT_AND_SETTLE
        self.phase_started_ns = now

    def begin_reobserve_clear(self, *, now_ns: int) -> None:
        self._require_phase(ObservationLifecyclePhase.RETREAT_AND_SETTLE)
        now = _nonnegative_stamp(now_ns, "now_ns")
        if self._action_finished_ns is None:
            raise TowelTaskContractError("primitive finish evidence is missing")
        elapsed = now - self._action_finished_ns
        if elapsed < self.lifecycle_limits.minimum_retreat_and_settle_ns:
            raise TowelTaskContractError(
                "retreat-and-settle interval has not elapsed"
            )
        self._pending.clear()
        self.phase = ObservationLifecyclePhase.REOBSERVE_CLEAR
        self.phase_started_ns = now

    def complete_episode(
        self,
        *,
        episode_id: str,
        outcome: EpisodeOutcome | str,
        now_ns: int,
    ) -> TowelEpisodeRecord:
        self._require_phase(ObservationLifecyclePhase.REOBSERVE_CLEAR)
        if self._before is None or self._after is None:
            raise TowelTaskContractError(
                "episode requires approved before and after observation windows"
            )
        if not isinstance(episode_id, str) or not episode_id:
            raise TowelTaskContractError("episode_id is required")
        if any(record.episode_id == episode_id for record in self.episodes):
            raise TowelTaskContractError("episode_id must be unique")
        try:
            outcome_value = (
                outcome if isinstance(outcome, EpisodeOutcome) else EpisodeOutcome(outcome)
            )
        except (TypeError, ValueError) as exc:
            raise TowelTaskContractError("invalid episode outcome") from exc
        now = _nonnegative_stamp(now_ns, "now_ns")
        if now < self._after.capture_stamps_ns[-1]:
            raise TowelTaskContractError("episode cannot predate its after observation")
        if None in {
            self._action_id,
            self._primitive,
            self._action_started_ns,
            self._action_finished_ns,
            self._calibration_sha256,
            self._model_sha256,
            self._robot_model_sha256,
        }:
            raise TowelTaskContractError("episode evidence is incomplete")
        record = TowelEpisodeRecord(
            episode_id=episode_id,
            action_id=str(self._action_id),
            primitive=str(self._primitive),
            before=self._before,
            after=self._after,
            outcome=outcome_value,
            action_started_ns=int(self._action_started_ns),
            action_finished_ns=int(self._action_finished_ns),
            episode_finished_ns=now,
            calibration_sha256=str(self._calibration_sha256),
            model_sha256=str(self._model_sha256),
            robot_model_sha256=str(self._robot_model_sha256),
        )
        self.episodes.append(record)
        self._before = self._after
        self._after = None
        self._action_id = None
        self._primitive = None
        self._action_started_ns = None
        self._action_finished_ns = None
        self._pending.clear()
        self.phase = ObservationLifecyclePhase.OBSERVE_CLEAR
        self.phase_started_ns = self._before.capture_stamps_ns[-1]
        return record

    def abort(self, reason: str, *, now_ns: int) -> None:
        if self.phase == ObservationLifecyclePhase.FAILED:
            raise TowelTaskContractError("failed observation lifecycle cannot be reused")
        if not isinstance(reason, str) or not reason:
            raise TowelTaskContractError("abort reason is required")
        self.phase_started_ns = _nonnegative_stamp(now_ns, "now_ns")
        self.failure_reason = reason
        self.phase = ObservationLifecyclePhase.FAILED
        self._pending.clear()
