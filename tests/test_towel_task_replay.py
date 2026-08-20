from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from tools.lib.towel_task_replay import replay_towel_task
from tools.lib.towel_task_runtime import (
    TowelTaskContractError,
    load_towel_contract,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = load_towel_contract(
    ROOT / "config/towel_task_contract.candidate.yaml"
)
BASE = {
    "schema_version": 1,
    "record_kind": "towel_state_observation",
    "observation_id": "base",
    "source_sha256": "a" * 64,
    "calibration_sha256": "b" * 64,
    "visible_area_ratio": 0.95,
    "topology_confidence": 0.95,
    "flatness_score": 0.95,
    "fold_count": 0,
    "outline_iou": None,
    "corners": [
        {"point_xy_m": [-0.2, 0.2], "confidence": 0.95},
        {"point_xy_m": [0.2, 0.2], "confidence": 0.95},
        {"point_xy_m": [0.2, -0.2], "confidence": 0.95},
        {"point_xy_m": [-0.2, -0.2], "confidence": 0.95},
    ],
}


def item(index, **changes):
    value = deepcopy(BASE)
    value["observation_id"] = f"step-{index}"
    value["source_sha256"] = f"{index:x}" * 64
    value.update(changes)
    return value


def test_nominal_replay_reaches_complete_without_motion():
    result = replay_towel_task(CONTRACT, [
        item(1, visible_area_ratio=0.2, corners=[], flatness_score=0.1),
        item(2, visible_area_ratio=0.7, corners=[], flatness_score=0.2),
        item(3),
        item(4, fold_count=1, outline_iou=0.9),
        item(5, fold_count=2, outline_iou=0.9),
    ])
    assert result["terminal_phase"] == "COMPLETE"
    assert result["motion_authorized"] is False
    assert result["motion_commands"] == 0
    assert result["steps"][-1]["estimated_state"] == "FOLD_2_COMPLETE"


def test_exhausted_sequence_fails_closed():
    result = replay_towel_task(CONTRACT, [item(1)])
    assert result["terminal_phase"] == "FAILED"
    assert result["steps"][-1]["decision_reason"].startswith(
        "observation sequence exhausted"
    )


def test_fault_is_terminal_and_does_not_consume_later_observations():
    result = replay_towel_task(CONTRACT, [
        item(1, fault=True),
        item(2, fold_count=2, outline_iou=0.9),
    ])
    assert result["terminal_phase"] == "FAILED"
    assert len(result["steps"]) == 1


def test_replay_rejects_duplicate_ids_and_mixed_calibration():
    duplicate = item(1)
    with pytest.raises(TowelTaskContractError, match="duplicate"):
        replay_towel_task(CONTRACT, [duplicate, duplicate])
    mixed = item(2)
    mixed["calibration_sha256"] = "c" * 64
    with pytest.raises(TowelTaskContractError, match="calibration"):
        replay_towel_task(CONTRACT, [item(1), mixed])


def test_replay_rejects_implicit_fault_boolean():
    with pytest.raises(TowelTaskContractError, match="fault must be boolean"):
        replay_towel_task(CONTRACT, [item(1, fault="false")])
