from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from tools.lib.towel_geometry import polygon_area
from tools.lib.towel_task_planning import build_towel_plan, load_json_object
from tools.lib.towel_task_runtime import (
    TowelTaskContractError,
    load_towel_contract,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = load_towel_contract(
    ROOT / "config/towel_task_contract.candidate.yaml"
)
EXAMPLE = load_json_object(ROOT / "config/towel_observation.example.json")


def build(observation=EXAMPLE):
    return build_towel_plan(
        CONTRACT,
        observation,
        contract_sha256="c" * 64,
        observation_sha256="d" * 64,
    )


def test_aligned_observation_builds_two_orthogonal_folds():
    plan = build()
    assert plan["estimated_state"] == "ALIGNED"
    assert plan["selected_first_axis"] == "x"
    assert plan["selected_second_axis"] == "y"
    assert len(plan["fold_sequence"]) == 2
    first, second = plan["fold_sequence"]
    assert polygon_area(first["expected_footprint"]) == pytest.approx(0.045)
    assert polygon_area(second["expected_footprint"]) == pytest.approx(0.0225)
    assert len(first["geometric_arc"]) == 9
    assert first["geometric_arc"][0]["progress"] == 0.0
    assert first["geometric_arc"][-1]["progress"] == 1.0
    assert first["kinematic_reachability_checked"] is False
    assert first["collision_checked"] is False


def test_plan_artifact_is_permanently_motion_free():
    plan = build()
    assert plan["motion_authorized"] is False
    assert plan["motion_commands"] == 0
    assert plan["execution_api_used"] is False
    assert plan["hardware_blockers"]


def test_crumpled_observation_recommends_unfolding_without_fold_waypoints():
    value = deepcopy(EXAMPLE)
    value.update({
        "visible_area_ratio": 0.2,
        "corners": [],
        "flatness_score": 0.1,
    })
    plan = build(value)
    assert plan["estimated_state"] == "CRUMPLED"
    assert plan["next_primitive"] == "lift_and_observe"
    assert plan["fold_sequence"] == []


def test_fold_axis_requires_explicit_costs_for_aligned_observation():
    value = deepcopy(EXAMPLE)
    del value["fold_axis_costs"]
    with pytest.raises(TowelTaskContractError, match="fold_axis_costs"):
        build(value)


def test_negative_direction_cost_is_rejected():
    value = deepcopy(EXAMPLE)
    value["fold_direction_costs"]["x_positive_to_negative"] = -1.0
    with pytest.raises(TowelTaskContractError, match="nonnegative"):
        build(value)


def test_candidate_contract_cannot_be_silently_authorized():
    modified = deepcopy(CONTRACT)
    modified["motion_authorized"] = True
    with pytest.raises(TowelTaskContractError, match="motion_authorized=false"):
        build_towel_plan(
            modified,
            EXAMPLE,
            contract_sha256="c" * 64,
            observation_sha256="d" * 64,
        )
