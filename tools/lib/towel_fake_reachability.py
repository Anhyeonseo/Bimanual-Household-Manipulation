"""Deterministic fixture backend for fold candidate and arm-assignment tests."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Mapping

from tools.lib.towel_task_runtime import TowelTaskContractError


COST_FIELDS = (
    "joint_distance",
    "crossing_penalty",
    "workspace_margin_penalty",
)
ARM_ASSIGNMENTS = {"left_to_first_corner", "right_to_first_corner"}


@dataclass(frozen=True, slots=True)
class FakeReachabilityCandidate:
    candidate_id: str
    axis: str
    direction: str
    arm_assignment: str
    reachable: bool
    collision_free: bool
    costs: dict[str, float]
    total_cost: float

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "FakeReachabilityCandidate":
        candidate_id = value.get("candidate_id")
        if not isinstance(candidate_id, str) or not candidate_id:
            raise TowelTaskContractError("candidate_id is required")
        axis = value.get("axis")
        if axis not in {"x", "y"}:
            raise TowelTaskContractError(f"{candidate_id}: axis must be x or y")
        direction = value.get("direction")
        if direction not in {"positive_to_negative", "negative_to_positive"}:
            raise TowelTaskContractError(f"{candidate_id}: invalid direction")
        assignment = value.get("arm_assignment")
        if assignment not in ARM_ASSIGNMENTS:
            raise TowelTaskContractError(f"{candidate_id}: invalid arm assignment")
        for field_name in ("reachable", "collision_free"):
            if not isinstance(value.get(field_name), bool):
                raise TowelTaskContractError(
                    f"{candidate_id}: {field_name} must be boolean"
                )
        source_costs = value.get("costs")
        if not isinstance(source_costs, Mapping) or set(source_costs) != set(COST_FIELDS):
            raise TowelTaskContractError(
                f"{candidate_id}: costs must contain exactly {COST_FIELDS}"
            )
        costs = {}
        for name in COST_FIELDS:
            raw = source_costs[name]
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise TowelTaskContractError(f"{candidate_id}: invalid cost {name}")
            cost = float(raw)
            if not math.isfinite(cost) or cost < 0.0:
                raise TowelTaskContractError(
                    f"{candidate_id}: costs must be finite and nonnegative"
                )
            costs[name] = cost
        return cls(
            candidate_id=candidate_id,
            axis=axis,
            direction=direction,
            arm_assignment=assignment,
            reachable=value["reachable"],
            collision_free=value["collision_free"],
            costs=costs,
            total_cost=sum(costs.values()),
        )


def evaluate_fake_reachability(
    candidate_documents: Iterable[Mapping[str, Any]],
    *,
    fixture_sha256: str,
) -> dict[str, Any]:
    if len(fixture_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in fixture_sha256
    ):
        raise TowelTaskContractError("fixture_sha256 must be lowercase SHA-256")
    candidates = tuple(
        FakeReachabilityCandidate.from_dict(value) for value in candidate_documents
    )
    if not candidates:
        raise TowelTaskContractError("at least one candidate is required")
    ids = [candidate.candidate_id for candidate in candidates]
    if len(ids) != len(set(ids)):
        raise TowelTaskContractError("candidate_id values must be unique")
    ordered = sorted(candidates, key=lambda item: item.candidate_id)
    feasible = [
        candidate for candidate in ordered
        if candidate.reachable and candidate.collision_free
    ]
    selected = min(
        feasible,
        key=lambda item: (item.total_cost, item.candidate_id),
        default=None,
    )
    return {
        "schema_version": 1,
        "record_kind": "towel_fake_reachability_result",
        "status": (
            "FAKE_CANDIDATE_SELECTED" if selected else "NO_FAKE_CANDIDATE_FEASIBLE"
        ),
        "motion_authorized": False,
        "motion_commands": 0,
        "execution_api_used": False,
        "backend": "fixture_only_not_moveit",
        "fixture_sha256": fixture_sha256,
        "selected_candidate_id": selected.candidate_id if selected else None,
        "candidates": [asdict(candidate) for candidate in ordered],
    }
