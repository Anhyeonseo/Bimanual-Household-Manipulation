from __future__ import annotations

from copy import deepcopy

import pytest

from tools.lib.towel_fake_reachability import evaluate_fake_reachability
from tools.lib.towel_task_runtime import TowelTaskContractError


def candidate(candidate_id, cost, *, reachable=True, collision_free=True):
    return {
        "candidate_id": candidate_id,
        "axis": "x",
        "direction": "positive_to_negative",
        "arm_assignment": "left_to_first_corner",
        "reachable": reachable,
        "collision_free": collision_free,
        "costs": {
            "joint_distance": cost,
            "crossing_penalty": 0.0,
            "workspace_margin_penalty": 0.0,
        },
    }


def evaluate(values):
    return evaluate_fake_reachability(values, fixture_sha256="a" * 64)


def test_selection_is_feasibility_gated_order_independent_and_motion_free():
    values = [
        candidate("b", 1.0),
        candidate("a", 1.0),
        candidate("cheap-collision", 0.1, collision_free=False),
    ]
    forward = evaluate(values)
    reverse = evaluate(reversed(values))
    assert forward["selected_candidate_id"] == "a"
    assert reverse["selected_candidate_id"] == "a"
    assert forward["motion_authorized"] is False
    assert forward["motion_commands"] == 0


def test_all_rejected_is_a_valid_fail_closed_result():
    result = evaluate([candidate("blocked", 0.0, reachable=False)])
    assert result["status"] == "NO_FAKE_CANDIDATE_FEASIBLE"
    assert result["selected_candidate_id"] is None


def test_candidate_contract_rejects_duplicates_and_implicit_boolean():
    value = candidate("same", 1.0)
    with pytest.raises(TowelTaskContractError, match="unique"):
        evaluate([value, deepcopy(value)])
    value["reachable"] = 1
    with pytest.raises(TowelTaskContractError, match="boolean"):
        evaluate([value])
