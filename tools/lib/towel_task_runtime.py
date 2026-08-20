"""Fail-closed towel state estimation and bounded task decisions."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from tools.lib.towel_geometry import (
    Point,
    QuadrilateralMetrics,
    TowelGeometryError,
    quadrilateral_metrics,
)


class TowelTaskContractError(ValueError):
    """A towel observation or contract is unsafe or incomplete."""


def _unit_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TowelTaskContractError(f"{label} must be a number within 0..1")
    normalized = float(value)
    if not math.isfinite(normalized) or not 0.0 <= normalized <= 1.0:
        raise TowelTaskContractError(f"{label} must be within 0..1")
    return normalized


def _boolean(value: Any, label: str, *, default: bool | None = None) -> bool:
    if value is None and default is not None:
        return default
    if not isinstance(value, bool):
        raise TowelTaskContractError(f"{label} must be boolean")
    return value


class TowelState(str, Enum):
    AMBIGUOUS = "AMBIGUOUS"
    CRUMPLED = "CRUMPLED"
    PARTIALLY_OPEN = "PARTIALLY_OPEN"
    TWO_CORNERS_VISIBLE = "TWO_CORNERS_VISIBLE"
    FOUR_CORNERS_VISIBLE = "FOUR_CORNERS_VISIBLE"
    FLAT_BUT_ROTATED = "FLAT_BUT_ROTATED"
    ALIGNED = "ALIGNED"
    FOLD_1_COMPLETE = "FOLD_1_COMPLETE"
    FOLD_2_COMPLETE = "FOLD_2_COMPLETE"


class TaskPhase(str, Enum):
    OBSERVE_INITIAL = "OBSERVE_INITIAL"
    COARSE_UNFOLD = "COARSE_UNFOLD"
    REOBSERVE = "REOBSERVE"
    CORNER_RECOVERY = "CORNER_RECOVERY"
    FLATTEN = "FLATTEN"
    ALIGN = "ALIGN"
    VERIFY_FLAT = "VERIFY_FLAT"
    FOLD_FIRST = "FOLD_FIRST"
    VERIFY_FIRST = "VERIFY_FIRST"
    FOLD_SECOND = "FOLD_SECOND"
    VERIFY_FINAL = "VERIFY_FINAL"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class RecoveryKind(str, Enum):
    CORNER_REDETECTION = "corner_redetection"
    LIFT_AND_UNFOLD = "lift_and_unfold"
    CORNER_DRAG = "corner_drag_per_corner"
    FOLD_PLACEMENT = "fold_placement_correction_per_fold"


HARDWARE_LIMIT_FIELDS = {
    "jaw_open_command_rad",
    "cloth_contact_command_rad",
    "cloth_contact_threshold_raw",
    "maximum_tension_proxy",
    "maximum_tcp_separation_m",
    "maximum_bimanual_speed_difference_m_s",
    "controlled_shake_amplitude_m",
    "controlled_shake_frequency_hz",
    "controlled_shake_cycles",
    "towel_table_friction",
}


@dataclass(frozen=True, slots=True)
class CornerCandidate:
    point_xy_m: Point
    confidence: float
    visible: bool = True
    graspable: bool = True

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "CornerCandidate":
        point = value.get("point_xy_m")
        if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) != 2:
            raise TowelTaskContractError("corner point_xy_m must contain x and y")
        if any(
            isinstance(coordinate, bool)
            or not isinstance(coordinate, (int, float))
            for coordinate in point
        ):
            raise TowelTaskContractError("corner coordinates must be numbers")
        confidence = _unit_number(value.get("confidence"), "corner confidence")
        candidate = cls(
            point_xy_m=(float(point[0]), float(point[1])),
            confidence=confidence,
            visible=_boolean(value.get("visible"), "corner visible", default=True),
            graspable=_boolean(
                value.get("graspable"), "corner graspable", default=True
            ),
        )
        if not all(math.isfinite(coordinate) for coordinate in candidate.point_xy_m):
            raise TowelTaskContractError("corner coordinates must be finite")
        return candidate


@dataclass(frozen=True, slots=True)
class TowelObservation:
    observation_id: str
    source_sha256: str
    calibration_sha256: str
    visible_area_ratio: float
    topology_confidence: float
    flatness_score: float
    fold_count: int
    outline_iou: float | None
    corners: tuple[CornerCandidate, ...]
    stale: bool = False

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> "TowelObservation":
        if document.get("schema_version") != 1:
            raise TowelTaskContractError("observation schema_version must be 1")
        if document.get("record_kind") != "towel_state_observation":
            raise TowelTaskContractError(
                "record_kind must be towel_state_observation"
            )
        corners_value = document.get("corners", [])
        if not isinstance(corners_value, list) or len(corners_value) > 4:
            raise TowelTaskContractError("corners must be a list of at most four")
        if any(not isinstance(value, Mapping) for value in corners_value):
            raise TowelTaskContractError("each corner must be an object")
        fold_count = document.get("fold_count", 0)
        if not isinstance(fold_count, int) or isinstance(fold_count, bool):
            raise TowelTaskContractError("fold_count must be an integer")
        outline_value = document.get("outline_iou")
        observation_id = document.get("observation_id")
        if not isinstance(observation_id, str) or not observation_id:
            raise TowelTaskContractError("observation_id is required")
        source_digest = document.get("source_sha256")
        calibration_digest = document.get("calibration_sha256")
        if not isinstance(source_digest, str) or not isinstance(
            calibration_digest, str
        ):
            raise TowelTaskContractError("observation digests must be strings")
        observation = cls(
            observation_id=observation_id,
            source_sha256=source_digest,
            calibration_sha256=calibration_digest,
            visible_area_ratio=_unit_number(
                document.get("visible_area_ratio"), "visible_area_ratio"
            ),
            topology_confidence=_unit_number(
                document.get("topology_confidence"), "topology_confidence"
            ),
            flatness_score=_unit_number(
                document.get("flatness_score"), "flatness_score"
            ),
            fold_count=fold_count,
            outline_iou=(
                None
                if outline_value is None
                else _unit_number(outline_value, "outline_iou")
            ),
            corners=tuple(CornerCandidate.from_dict(value) for value in corners_value),
            stale=_boolean(document.get("stale"), "stale", default=False),
        )
        observation.validate()
        return observation

    def validate(self) -> None:
        if not self.observation_id:
            raise TowelTaskContractError("observation_id is required")
        for label, digest in (
            ("source_sha256", self.source_sha256),
            ("calibration_sha256", self.calibration_sha256),
        ):
            if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
                raise TowelTaskContractError(f"{label} must be lowercase SHA-256")
        for label, value in (
            ("visible_area_ratio", self.visible_area_ratio),
            ("topology_confidence", self.topology_confidence),
            ("flatness_score", self.flatness_score),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise TowelTaskContractError(f"{label} must be within 0..1")
        if self.fold_count not in (0, 1, 2):
            raise TowelTaskContractError("fold_count must be 0, 1, or 2")
        if self.outline_iou is not None and (
            not math.isfinite(self.outline_iou)
            or not 0.0 <= self.outline_iou <= 1.0
        ):
            raise TowelTaskContractError("outline_iou must be within 0..1")

    @property
    def usable_corners(self) -> tuple[CornerCandidate, ...]:
        return tuple(
            corner for corner in self.corners
            if corner.visible and corner.graspable
        )


@dataclass(frozen=True, slots=True)
class PerceptionLimits:
    minimum_corner_confidence: float
    minimum_topology_confidence: float
    partially_open_visible_area_ratio: float
    flat_visible_area_ratio: float
    maximum_edge_relative_spread: float
    maximum_diagonal_relative_difference: float
    minimum_flatness_score: float
    maximum_axis_alignment_error_deg: float
    minimum_intermediate_outline_iou: float
    minimum_final_outline_iou: float

    @classmethod
    def from_contract(cls, contract: Mapping[str, Any]) -> "PerceptionLimits":
        values = contract.get("perception_candidate_limits")
        if not isinstance(values, Mapping):
            raise TowelTaskContractError(
                "perception_candidate_limits must be an object"
            )
        try:
            limits = cls(**{
                field_name: float(values[field_name])
                for field_name in cls.__dataclass_fields__
            })
        except (KeyError, TypeError, ValueError) as exc:
            raise TowelTaskContractError(
                f"invalid perception candidate limits: {exc}"
            ) from exc
        if not all(
            math.isfinite(getattr(limits, field_name))
            and getattr(limits, field_name) >= 0.0
            for field_name in cls.__dataclass_fields__
        ):
            raise TowelTaskContractError(
                "perception candidate limits must be finite and nonnegative"
            )
        return limits


@dataclass(frozen=True, slots=True)
class StateEstimate:
    state: TowelState
    reason: str
    corner_count: int
    minimum_corner_confidence: float | None
    geometry: QuadrilateralMetrics | None


def estimate_towel_state(
    observation: TowelObservation,
    limits: PerceptionLimits,
) -> StateEstimate:
    """Classify one stabilized observation without authorizing motion."""
    usable = tuple(
        corner for corner in observation.usable_corners
        if corner.confidence >= limits.minimum_corner_confidence
    )
    minimum_confidence = (
        min(corner.confidence for corner in usable) if usable else None
    )
    if observation.stale:
        return StateEstimate(
            TowelState.AMBIGUOUS, "observation is stale",
            len(usable), minimum_confidence, None,
        )
    if observation.topology_confidence < limits.minimum_topology_confidence:
        return StateEstimate(
            TowelState.AMBIGUOUS, "topology confidence is below threshold",
            len(usable), minimum_confidence, None,
        )
    if observation.fold_count == 2:
        if (
            observation.outline_iou is not None
            and observation.outline_iou >= limits.minimum_final_outline_iou
        ):
            return StateEstimate(
                TowelState.FOLD_2_COMPLETE, "final outline passes",
                len(usable), minimum_confidence, None,
            )
        return StateEstimate(
            TowelState.AMBIGUOUS, "second fold outline is not verified",
            len(usable), minimum_confidence, None,
        )
    if observation.fold_count == 1:
        if (
            observation.outline_iou is not None
            and observation.outline_iou >= limits.minimum_intermediate_outline_iou
        ):
            return StateEstimate(
                TowelState.FOLD_1_COMPLETE, "intermediate outline passes",
                len(usable), minimum_confidence, None,
            )
        return StateEstimate(
            TowelState.AMBIGUOUS, "first fold outline is not verified",
            len(usable), minimum_confidence, None,
        )

    geometry = None
    if len(usable) == 4:
        try:
            geometry = quadrilateral_metrics(
                corner.point_xy_m for corner in usable
            )
        except TowelGeometryError:
            return StateEstimate(
                TowelState.AMBIGUOUS, "four corners are geometrically invalid",
                len(usable), minimum_confidence, None,
            )
        square_like = (
            geometry.edge_relative_spread
            <= limits.maximum_edge_relative_spread
            and geometry.diagonal_relative_difference
            <= limits.maximum_diagonal_relative_difference
        )
        flat = (
            square_like
            and observation.visible_area_ratio >= limits.flat_visible_area_ratio
            and observation.flatness_score >= limits.minimum_flatness_score
        )
        if flat and (
            geometry.axis_alignment_error_deg
            <= limits.maximum_axis_alignment_error_deg
        ):
            return StateEstimate(
                TowelState.ALIGNED, "flat square is aligned",
                len(usable), minimum_confidence, geometry,
            )
        if flat:
            return StateEstimate(
                TowelState.FLAT_BUT_ROTATED, "flat square requires alignment",
                len(usable), minimum_confidence, geometry,
            )
        return StateEstimate(
            TowelState.FOUR_CORNERS_VISIBLE,
            "four corners visible but flatness contract is not met",
            len(usable), minimum_confidence, geometry,
        )
    if len(usable) >= 2:
        return StateEstimate(
            TowelState.TWO_CORNERS_VISIBLE, "two graspable corners visible",
            len(usable), minimum_confidence, None,
        )
    if (
        observation.visible_area_ratio
        >= limits.partially_open_visible_area_ratio
    ):
        return StateEstimate(
            TowelState.PARTIALLY_OPEN, "visible area supports corner recovery",
            len(usable), minimum_confidence, None,
        )
    return StateEstimate(
        TowelState.CRUMPLED, "visible area and corners are insufficient",
        len(usable), minimum_confidence, None,
    )


@dataclass(frozen=True, slots=True)
class TaskDecision:
    phase: TaskPhase
    primitive: str | None
    terminal: bool
    reason: str


def decision_for_state(state: TowelState) -> TaskDecision:
    decisions = {
        TowelState.AMBIGUOUS: TaskDecision(
            TaskPhase.CORNER_RECOVERY, "redetect_corners", False,
            "observation must be disambiguated",
        ),
        TowelState.CRUMPLED: TaskDecision(
            TaskPhase.COARSE_UNFOLD, "lift_and_observe", False,
            "coarse unfolding is required",
        ),
        TowelState.PARTIALLY_OPEN: TaskDecision(
            TaskPhase.COARSE_UNFOLD, "tension_spread", False,
            "increase visible area before corner recovery",
        ),
        TowelState.TWO_CORNERS_VISIBLE: TaskDecision(
            TaskPhase.CORNER_RECOVERY, "grasp_two_corners", False,
            "dual-corner spreading can proceed",
        ),
        TowelState.FOUR_CORNERS_VISIBLE: TaskDecision(
            TaskPhase.FLATTEN, "drag_corner", False,
            "flatten and recover square geometry",
        ),
        TowelState.FLAT_BUT_ROTATED: TaskDecision(
            TaskPhase.ALIGN, "align_square", False,
            "align the flat towel to a workcell axis",
        ),
        TowelState.ALIGNED: TaskDecision(
            TaskPhase.FOLD_FIRST, "fold_edge_pair", False,
            "first half-fold may be planned",
        ),
        TowelState.FOLD_1_COMPLETE: TaskDecision(
            TaskPhase.FOLD_SECOND, "fold_edge_pair", False,
            "second orthogonal half-fold may be planned",
        ),
        TowelState.FOLD_2_COMPLETE: TaskDecision(
            TaskPhase.COMPLETE, None, True, "final outline passes",
        ),
    }
    return decisions[state]


@dataclass(slots=True)
class RecoveryLedger:
    limits: dict[RecoveryKind, int]
    attempts: dict[str, int] = field(default_factory=dict)

    @classmethod
    def from_contract(cls, contract: Mapping[str, Any]) -> "RecoveryLedger":
        values = contract.get("recovery_limits")
        if not isinstance(values, Mapping):
            raise TowelTaskContractError("recovery_limits must be an object")
        limits = {}
        for kind in RecoveryKind:
            value = values.get(kind.value)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise TowelTaskContractError(
                    f"recovery limit {kind.value} must be a nonnegative integer"
                )
            limits[kind] = value
        return cls(limits=limits)

    def claim(self, kind: RecoveryKind, *, key: str = "global") -> int:
        attempt_key = f"{kind.value}:{key}"
        attempt = self.attempts.get(attempt_key, 0) + 1
        if attempt > self.limits[kind]:
            raise TowelTaskContractError(
                f"recovery budget exhausted for {attempt_key}"
            )
        self.attempts[attempt_key] = attempt
        return attempt

    def remaining(self, kind: RecoveryKind, *, key: str = "global") -> int:
        attempt_key = f"{kind.value}:{key}"
        return self.limits[kind] - self.attempts.get(attempt_key, 0)


@dataclass(slots=True)
class TowelTaskStateMachine:
    ledger: RecoveryLedger
    phase: TaskPhase = TaskPhase.OBSERVE_INITIAL
    decision_count: int = 0
    maximum_decisions: int = 32

    @classmethod
    def from_contract(
        cls, contract: Mapping[str, Any]
    ) -> "TowelTaskStateMachine":
        values = contract.get("recovery_limits")
        if not isinstance(values, Mapping):
            raise TowelTaskContractError("recovery_limits must be an object")
        maximum = values.get("maximum_task_decisions")
        if not isinstance(maximum, int) or isinstance(maximum, bool) or maximum <= 0:
            raise TowelTaskContractError(
                "maximum_task_decisions must be a positive integer"
            )
        return cls(
            ledger=RecoveryLedger.from_contract(contract),
            maximum_decisions=maximum,
        )

    def decide(
        self,
        estimate: StateEstimate,
        *,
        fault: bool = False,
        workspace_exit: bool = False,
    ) -> TaskDecision:
        if self.phase in (TaskPhase.COMPLETE, TaskPhase.FAILED):
            raise TowelTaskContractError("terminal task state cannot be reused")
        self.decision_count += 1
        if self.decision_count > self.maximum_decisions:
            self.phase = TaskPhase.FAILED
            return TaskDecision(
                TaskPhase.FAILED, None, True,
                "maximum task decisions exhausted",
            )
        if fault or workspace_exit:
            self.phase = TaskPhase.FAILED
            return TaskDecision(
                TaskPhase.FAILED, None, True,
                "fault or workspace exit is not retryable",
            )
        decision = decision_for_state(estimate.state)
        try:
            if estimate.state == TowelState.AMBIGUOUS:
                self.ledger.claim(RecoveryKind.CORNER_REDETECTION)
            elif estimate.state == TowelState.CRUMPLED:
                self.ledger.claim(RecoveryKind.LIFT_AND_UNFOLD)
            elif estimate.state == TowelState.FOUR_CORNERS_VISIBLE:
                self.ledger.claim(RecoveryKind.CORNER_DRAG, key="unresolved")
        except TowelTaskContractError as exc:
            self.phase = TaskPhase.FAILED
            return TaskDecision(TaskPhase.FAILED, None, True, str(exc))
        self.phase = decision.phase
        return decision


def load_towel_contract(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise TowelTaskContractError(f"could not load towel contract: {exc}") from exc
    if not isinstance(document, dict):
        raise TowelTaskContractError("towel contract must be a mapping")
    validate_towel_contract(document)
    return document


def validate_towel_contract(contract: Mapping[str, Any]) -> None:
    if contract.get("schema_version") != 1:
        raise TowelTaskContractError("contract schema_version must be 1")
    if contract.get("record_kind") != "towel_task_contract":
        raise TowelTaskContractError("record_kind must be towel_task_contract")
    if contract.get("status") != "SOFTWARE_CONTRACT_ONLY":
        raise TowelTaskContractError("candidate status must be SOFTWARE_CONTRACT_ONLY")
    if contract.get("motion_authorized") is not False:
        raise TowelTaskContractError(
            "candidate towel contract must keep motion_authorized=false"
        )
    towel = contract.get("towel")
    if not isinstance(towel, Mapping) or towel.get("shape") != "square":
        raise TowelTaskContractError("towel shape must be square")
    if towel.get("provenance") != "not_measured":
        raise TowelTaskContractError("towel provenance must be not_measured")
    physical_fields = {
        "nominal_side_mm", "side_tolerance_mm", "thickness_mm", "mass_g",
        "material", "measurement_artifact",
    }
    if any(towel.get(name) is not None for name in physical_fields):
        raise TowelTaskContractError(
            "unmeasured towel properties must remain null"
        )
    workspace = contract.get("workspace")
    if not isinstance(workspace, Mapping) or not isinstance(
        workspace.get("frame"), str
    ) or not workspace.get("frame"):
        raise TowelTaskContractError("workspace frame is required")
    expected_workspace_flags = {
        "start_fully_inside_workspace": True,
        "start_fully_visible_from_top": True,
        "other_objects_allowed": False,
        "knots_allowed": False,
        "multiple_towels_allowed": False,
    }
    if any(workspace.get(name) is not expected for name, expected in expected_workspace_flags.items()):
        raise TowelTaskContractError("workspace scope flags must remain fail-closed")
    fold_policy = contract.get("fold_policy")
    if not isinstance(fold_policy, Mapping) or (
        fold_policy.get("target_area_ratio_after_first_fold") != 0.5
        or fold_policy.get("target_area_ratio_after_second_fold") != 0.25
    ):
        raise TowelTaskContractError("fold policy must define half then quarter area")
    hardware = contract.get("hardware_limits")
    if not isinstance(hardware, Mapping):
        raise TowelTaskContractError("hardware_limits must be an object")
    if hardware.get("provenance") != "not_measured":
        raise TowelTaskContractError(
            "candidate hardware provenance must be not_measured"
        )
    if set(hardware) != HARDWARE_LIMIT_FIELDS | {"provenance"}:
        raise TowelTaskContractError(
            "hardware_limits must contain the complete candidate field set"
        )
    if any(
        value is not None
        for name, value in hardware.items()
        if name != "provenance"
    ):
        raise TowelTaskContractError(
            "unmeasured hardware limits must remain null in the candidate"
        )
    limits = PerceptionLimits.from_contract(contract)
    probability_fields = (
        "minimum_corner_confidence",
        "minimum_topology_confidence",
        "partially_open_visible_area_ratio",
        "flat_visible_area_ratio",
        "maximum_edge_relative_spread",
        "maximum_diagonal_relative_difference",
        "minimum_flatness_score",
        "minimum_intermediate_outline_iou",
        "minimum_final_outline_iou",
    )
    if any(not 0.0 <= getattr(limits, name) <= 1.0 for name in probability_fields):
        raise TowelTaskContractError(
            "probability, ratio, and relative-error limits must be within 0..1"
        )
    if not 0.0 <= limits.maximum_axis_alignment_error_deg <= 45.0:
        raise TowelTaskContractError(
            "maximum_axis_alignment_error_deg must be within 0..45"
        )
    RecoveryLedger.from_contract(contract)
    TowelTaskStateMachine.from_contract(contract)
    recovery = contract["recovery_limits"]
    if recovery.get("retry_after_fault") != 0 or recovery.get(
        "retry_after_workspace_exit"
    ) != 0:
        raise TowelTaskContractError("fault and workspace exit retries must be zero")
    acceptance = contract.get("acceptance")
    if not isinstance(acceptance, Mapping):
        raise TowelTaskContractError("acceptance must be an object")
    zero_incident_fields = (
        "maximum_collisions", "maximum_uncommanded_motions",
        "maximum_towel_drops", "maximum_workspace_exits",
    )
    if any(acceptance.get(name) != 0 for name in zero_incident_fields):
        raise TowelTaskContractError("acceptance incident limits must remain zero")
    if acceptance.get("every_run_requires_artifact") is not True:
        raise TowelTaskContractError("every run must require an artifact")


def stabilize_observations(
    observations: Sequence[TowelObservation],
    limits: PerceptionLimits,
    *,
    minimum_consecutive: int = 3,
) -> StateEstimate:
    """Require consecutive agreement before a state can drive task planning."""
    if minimum_consecutive < 1:
        raise TowelTaskContractError("minimum_consecutive must be positive")
    if len(observations) < minimum_consecutive:
        raise TowelTaskContractError(
            f"at least {minimum_consecutive} observations are required"
        )
    window = tuple(observations[-minimum_consecutive:])
    if len({observation.observation_id for observation in window}) != len(window):
        raise TowelTaskContractError("observation ids must be unique")
    if len({observation.calibration_sha256 for observation in window}) != 1:
        raise TowelTaskContractError(
            "stabilization window mixes calibration identities"
        )
    estimates = tuple(estimate_towel_state(observation, limits) for observation in window)
    states = {estimate.state for estimate in estimates}
    if len(states) != 1 or TowelState.AMBIGUOUS in states:
        raise TowelTaskContractError(
            "consecutive observations do not agree on a usable state"
        )
    return estimates[-1]
