from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

import pytest

from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    RecoveryKind,
    RecoveryLedger,
    TaskPhase,
    TowelObservation,
    TowelState,
    TowelTaskContractError,
    TowelTaskStateMachine,
    estimate_towel_state,
    load_towel_contract,
    stabilize_observations,
    validate_towel_contract,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "config/towel_task_contract.candidate.yaml"


def contract():
    return load_towel_contract(CONTRACT_PATH)


def observation(**changes):
    value = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": "test-observation",
        "source_sha256": "a" * 64,
        "calibration_sha256": "b" * 64,
        "visible_area_ratio": 0.95,
        "topology_confidence": 0.95,
        "flatness_score": 0.95,
        "fold_count": 0,
        "outline_iou": None,
        "stale": False,
        "corners": [
            {"point_xy_m": [-0.2, 0.2], "confidence": 0.95},
            {"point_xy_m": [0.2, 0.2], "confidence": 0.95},
            {"point_xy_m": [0.2, -0.2], "confidence": 0.95},
            {"point_xy_m": [-0.2, -0.2], "confidence": 0.95},
        ],
    }
    value.update(changes)
    return TowelObservation.from_dict(value)


def estimate(value):
    return estimate_towel_state(
        value, PerceptionLimits.from_contract(contract())
    )


def test_candidate_contract_is_motion_locked_with_300_mm_nominal_towel():
    document = contract()
    assert document["motion_authorized"] is False
    assert document["towel"]["nominal_side_mm"] == pytest.approx(300.0)
    assert document["towel"]["provenance"] == "user_reported_nominal_side_only"
    assert all(
        value is None
        for key, value in document["hardware_limits"].items()
        if key != "provenance"
    )


def test_workcell_observation_candidate_remains_fail_closed():
    candidate = contract()["workcell_observation_candidate"]
    assert candidate["motion_authorized"] is False
    assert candidate["top_camera"]["device_path"].endswith(
        "platform-xhci-hcd.0-usb-0:1.1:1.0-video-index0"
    )
    assert candidate["top_camera"]["width"] == 1280
    assert candidate["top_camera"]["height"] == 960
    assert candidate["top_camera"]["metric_calibration_validated"] is False
    assert candidate["towel_envelope"][
        "required_perimeter_margin_mm"
    ] == pytest.approx(30.0)
    clear = candidate["observe_clear"]
    assert len(clear["joint_names"]) == len(clear["joint_positions_rad"]) == 12
    assert clear["present_mask"] == 0xFFF
    assert clear["torque_enabled"] is False
    assert clear["visual_towel_occlusion"] is False
    assert clear["motion_reproducibility_validated"] is False


def test_candidate_contract_rejects_missing_hardware_field_and_wrong_towel_size():
    document = contract()
    del document["hardware_limits"]["maximum_tension_proxy"]
    with pytest.raises(TowelTaskContractError, match="complete candidate"):
        validate_towel_contract(document)
    document = contract()
    document["towel"]["nominal_side_mm"] = 400
    with pytest.raises(TowelTaskContractError, match="300 mm"):
        validate_towel_contract(document)
    document = contract()
    document["towel"]["mass_g"] = 50
    with pytest.raises(TowelTaskContractError, match="other than nominal side"):
        validate_towel_contract(document)


def test_aligned_square_is_accepted_only_with_four_confident_corners():
    result = estimate(observation())
    assert result.state == TowelState.ALIGNED
    assert result.geometry is not None
    weak = deepcopy([
        {"point_xy_m": [-0.2, 0.2], "confidence": 0.2},
        {"point_xy_m": [0.2, 0.2], "confidence": 0.2},
        {"point_xy_m": [0.2, -0.2], "confidence": 0.2},
        {"point_xy_m": [-0.2, -0.2], "confidence": 0.2},
    ])
    assert estimate(observation(corners=weak)).state == TowelState.PARTIALLY_OPEN


def test_rotated_flat_square_requires_alignment():
    angle = math.radians(12.0)
    rotated = [
        {
            "point_xy_m": [
                math.cos(angle) * x - math.sin(angle) * y,
                math.sin(angle) * x + math.cos(angle) * y,
            ],
            "confidence": 0.95,
        }
        for x, y in ((-0.2, 0.2), (0.2, 0.2), (0.2, -0.2), (-0.2, -0.2))
    ]
    assert estimate(observation(corners=rotated)).state == TowelState.FLAT_BUT_ROTATED


@pytest.mark.parametrize(
    ("changes", "expected"),
    (
        ({"visible_area_ratio": 0.2, "corners": []}, TowelState.CRUMPLED),
        ({"visible_area_ratio": 0.7, "corners": []}, TowelState.PARTIALLY_OPEN),
        ({"corners": [
            {"point_xy_m": [-0.2, 0.2], "confidence": 0.9},
            {"point_xy_m": [0.2, 0.2], "confidence": 0.9},
        ]}, TowelState.TWO_CORNERS_VISIBLE),
        ({"fold_count": 1, "outline_iou": 0.9}, TowelState.FOLD_1_COMPLETE),
        ({"fold_count": 2, "outline_iou": 0.9}, TowelState.FOLD_2_COMPLETE),
        ({"stale": True}, TowelState.AMBIGUOUS),
    ),
)
def test_state_classification(changes, expected):
    assert estimate(observation(**changes)).state == expected


def test_recovery_budget_is_finite_and_keyed_per_corner():
    ledger = RecoveryLedger.from_contract(contract())
    assert ledger.claim(RecoveryKind.CORNER_DRAG, key="top_left") == 1
    assert ledger.claim(RecoveryKind.CORNER_DRAG, key="top_left") == 2
    with pytest.raises(TowelTaskContractError, match="exhausted"):
        ledger.claim(RecoveryKind.CORNER_DRAG, key="top_left")
    assert ledger.remaining(RecoveryKind.CORNER_DRAG, key="top_right") == 2


def test_state_machine_terminates_after_crumpled_recovery_budget():
    machine = TowelTaskStateMachine.from_contract(contract())
    crumpled = estimate(observation(visible_area_ratio=0.2, corners=[]))
    assert machine.decide(crumpled).phase == TaskPhase.COARSE_UNFOLD
    assert machine.decide(crumpled).phase == TaskPhase.COARSE_UNFOLD
    exhausted = machine.decide(crumpled)
    assert exhausted.phase == TaskPhase.FAILED
    assert exhausted.terminal is True
    with pytest.raises(TowelTaskContractError, match="terminal"):
        machine.decide(crumpled)


def test_fault_and_workspace_exit_are_never_retried():
    machine = TowelTaskStateMachine.from_contract(contract())
    failed = machine.decide(estimate(observation()), fault=True)
    assert failed.phase == TaskPhase.FAILED
    assert failed.terminal is True


def test_three_consecutive_observations_are_required_for_stabilization():
    limits = PerceptionLimits.from_contract(contract())
    values = []
    for index in range(3):
        item = observation()
        document = {
            "schema_version": 1,
            "record_kind": "towel_state_observation",
            "observation_id": f"stable-{index}",
            "source_sha256": item.source_sha256,
            "calibration_sha256": item.calibration_sha256,
            "visible_area_ratio": item.visible_area_ratio,
            "topology_confidence": item.topology_confidence,
            "flatness_score": item.flatness_score,
            "fold_count": item.fold_count,
            "outline_iou": item.outline_iou,
            "corners": [
                {
                    "point_xy_m": list(corner.point_xy_m),
                    "confidence": corner.confidence,
                }
                for corner in item.corners
            ],
        }
        values.append(TowelObservation.from_dict(document))
    assert stabilize_observations(values, limits).state == TowelState.ALIGNED
    with pytest.raises(TowelTaskContractError, match="at least 3"):
        stabilize_observations(values[:2], limits)


def test_stabilization_rejects_mixed_calibration_and_state_flicker():
    limits = PerceptionLimits.from_contract(contract())
    first = observation()
    second_document = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": "second",
        "source_sha256": "c" * 64,
        "calibration_sha256": "d" * 64,
        "visible_area_ratio": 0.95,
        "topology_confidence": 0.95,
        "flatness_score": 0.95,
        "fold_count": 0,
        "outline_iou": None,
        "corners": [],
    }
    second = TowelObservation.from_dict(second_document)
    with pytest.raises(TowelTaskContractError, match="calibration"):
        stabilize_observations([first, second], limits, minimum_consecutive=2)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("fold_count", 0.5, "integer"),
        ("stale", "false", "boolean"),
        ("visible_area_ratio", True, "number"),
        ("topology_confidence", "0.9", "number"),
    ),
)
def test_runtime_observation_rejects_implicit_type_coercion(
    field, value, message
):
    base = observation()
    document = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": base.observation_id,
        "source_sha256": base.source_sha256,
        "calibration_sha256": base.calibration_sha256,
        "visible_area_ratio": base.visible_area_ratio,
        "topology_confidence": base.topology_confidence,
        "flatness_score": base.flatness_score,
        "fold_count": base.fold_count,
        "outline_iou": base.outline_iou,
        "stale": base.stale,
        "corners": [
            {"point_xy_m": list(corner.point_xy_m), "confidence": corner.confidence}
            for corner in base.corners
        ],
    }
    document[field] = value
    with pytest.raises(TowelTaskContractError, match=message):
        TowelObservation.from_dict(document)
