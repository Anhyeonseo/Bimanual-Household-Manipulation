"""Build motion-free towel task plan artifacts from reviewed observations."""

from __future__ import annotations

from dataclasses import asdict
from hashlib import sha256
import json
import math
from pathlib import Path
from typing import Any, Mapping

from tools.lib.towel_fold_path import build_geometric_fold_arc
from tools.lib.towel_geometry import (
    TowelGeometryError,
    build_half_fold,
    choose_first_fold_axis,
)
from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    TowelObservation,
    TowelState,
    TowelTaskContractError,
    decision_for_state,
    estimate_towel_state,
    validate_towel_contract,
)


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _finite_costs(document: Mapping[str, Any], key: str) -> dict[str, float]:
    value = document.get(key)
    if not isinstance(value, Mapping):
        raise TowelTaskContractError(f"{key} must be an object")
    try:
        costs = {str(name): float(cost) for name, cost in value.items()}
    except (TypeError, ValueError) as exc:
        raise TowelTaskContractError(f"{key} contains an invalid cost") from exc
    if not all(math.isfinite(cost) and cost >= 0.0 for cost in costs.values()):
        raise TowelTaskContractError(
            f"{key} costs must be finite and nonnegative"
        )
    return costs


def _direction_for_axis(document: Mapping[str, Any], axis: str) -> str:
    costs = _finite_costs(document, "fold_direction_costs")
    names = (
        f"{axis}_positive_to_negative",
        f"{axis}_negative_to_positive",
    )
    if any(name not in costs for name in names):
        raise TowelTaskContractError(
            f"fold_direction_costs must contain {names}"
        )
    selected = min(names, key=lambda name: (costs[name], name))
    return selected.removeprefix(f"{axis}_")


def build_towel_plan(
    contract: Mapping[str, Any],
    observation_document: Mapping[str, Any],
    *,
    contract_sha256: str,
    observation_sha256: str,
) -> dict[str, Any]:
    validate_towel_contract(contract)
    for label, digest in (
        ("contract_sha256", contract_sha256),
        ("observation_sha256", observation_sha256),
    ):
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise TowelTaskContractError(f"{label} must be lowercase SHA-256")
    observation = TowelObservation.from_dict(observation_document)
    limits = PerceptionLimits.from_contract(contract)
    estimate = estimate_towel_state(observation, limits)
    decision = decision_for_state(estimate.state)
    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "towel_task_plan_only",
        "status": "TOWEL_TASK_PLAN_ONLY_PASS",
        "motion_authorized": False,
        "motion_commands": 0,
        "execution_api_used": False,
        "contract_sha256": contract_sha256,
        "observation_sha256": observation_sha256,
        "source_image_sha256": observation.source_sha256,
        "calibration_sha256": observation.calibration_sha256,
        "observation_id": observation.observation_id,
        "estimated_state": estimate.state.value,
        "state_reason": estimate.reason,
        "next_phase": decision.phase.value,
        "next_primitive": decision.primitive,
        "terminal": decision.terminal,
        "fold_sequence": [],
        "hardware_blockers": [
            name
            for name, value in contract["hardware_limits"].items()
            if name != "provenance" and value is None
        ],
    }
    if estimate.geometry is not None:
        document["geometry"] = {
            **asdict(estimate.geometry),
            "ordered_corner_labels": [
                "top_left", "top_right", "bottom_right", "bottom_left"
            ],
        }
    if estimate.state != TowelState.ALIGNED:
        return document

    axis_costs = _finite_costs(observation_document, "fold_axis_costs")
    first_axis = choose_first_fold_axis(axis_costs)
    second_axis = "y" if first_axis == "x" else "x"
    try:
        first = build_half_fold(
            estimate.geometry.ordered_corners,
            first_axis,
            _direction_for_axis(observation_document, first_axis),
        )
        second = build_half_fold(
            first.expected_footprint,
            second_axis,
            _direction_for_axis(observation_document, second_axis),
        )
    except TowelGeometryError as exc:
        raise TowelTaskContractError(f"could not build fold sequence: {exc}") from exc
    document["fold_sequence"] = [
        {
            "index": 1,
            **asdict(first),
            "geometric_arc": [
                asdict(waypoint) for waypoint in build_geometric_fold_arc(first)
            ],
            "kinematic_reachability_checked": False,
            "collision_checked": False,
        },
        {
            "index": 2,
            **asdict(second),
            "geometric_arc": [
                asdict(waypoint) for waypoint in build_geometric_fold_arc(second)
            ],
            "expected_final_area_ratio": 0.25,
            "kinematic_reachability_checked": False,
            "collision_checked": False,
        },
    ]
    document["selected_first_axis"] = first_axis
    document["selected_second_axis"] = second_axis
    return document


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TowelTaskContractError(f"could not load JSON {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise TowelTaskContractError(f"JSON root must be an object: {path}")
    return document
