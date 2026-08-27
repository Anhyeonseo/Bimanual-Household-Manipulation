from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from tools.lib.towel_observation_lifecycle import (
    EpisodeOutcome,
    ObservationLifecyclePhase,
    TowelObservationLifecycle,
)
from tools.lib.towel_task_runtime import (
    TowelObservation,
    TowelTaskContractError,
    load_towel_contract,
    validate_towel_contract,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "config/towel_task_contract.candidate.yaml"


def contract():
    return load_towel_contract(CONTRACT_PATH)


def observation(
    index: int,
    stamp_ns: int,
    *,
    phase: str = "OBSERVE_CLEAR",
    **changes,
) -> TowelObservation:
    value = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": f"lifecycle-{index}",
        "source_sha256": f"{index % 16:x}" * 64,
        "calibration_sha256": "a" * 64,
        "visible_area_ratio": 0.95,
        "topology_confidence": 0.95,
        "flatness_score": 0.95,
        "fold_count": 0,
        "outline_iou": None,
        "stale": False,
        "capture_stamp_ns": stamp_ns,
        "lifecycle_phase": phase,
        "model_sha256": "b" * 64,
        "robot_model_sha256": "c" * 64,
        "settled": True,
        "clear_pose_verified": True,
        "clear_view_valid": True,
        "corners": [
            {"point_xy_m": [-0.2, 0.2], "confidence": 0.95},
            {"point_xy_m": [0.2, 0.2], "confidence": 0.95},
            {"point_xy_m": [0.2, -0.2], "confidence": 0.95},
            {"point_xy_m": [-0.2, -0.2], "confidence": 0.95},
        ],
    }
    value.update(changes)
    return TowelObservation.from_dict(value)


def approve_initial_window(
    lifecycle: TowelObservationLifecycle,
) -> None:
    for index, stamp in enumerate(
        (1_000_000_000, 1_100_000_000, 1_200_000_000), start=1
    ):
        estimate = lifecycle.observe(
            observation(index, stamp), now_ns=stamp + 10_000_000
        )
    assert estimate is not None
    assert estimate.state.value == "ALIGNED"


def test_nominal_lifecycle_emits_motion_locked_before_after_episode():
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=900_000_000
    )
    approve_initial_window(lifecycle)
    lifecycle.begin_primitive(
        action_id="fold-01",
        primitive="bimanual_edge_pair",
        now_ns=1_300_000_000,
    )
    assert lifecycle.phase == ObservationLifecyclePhase.PRIMITIVE
    lifecycle.finish_primitive(now_ns=1_400_000_000)
    assert lifecycle.phase == ObservationLifecyclePhase.RETREAT_AND_SETTLE
    lifecycle.begin_reobserve_clear(now_ns=2_150_000_000)
    assert lifecycle.phase == ObservationLifecyclePhase.REOBSERVE_CLEAR

    for index, stamp in enumerate(
        (2_160_000_000, 2_260_000_000, 2_360_000_000), start=4
    ):
        estimate = lifecycle.observe(
            observation(
                index,
                stamp,
                phase="REOBSERVE_CLEAR",
                fold_count=1,
                outline_iou=0.9,
            ),
            now_ns=stamp + 10_000_000,
        )
    assert estimate is not None
    assert estimate.state.value == "FOLD_1_COMPLETE"
    record = lifecycle.complete_episode(
        episode_id="episode-01",
        outcome=EpisodeOutcome.SUCCEEDED,
        now_ns=2_400_000_000,
    )
    document = record.to_dict()
    assert lifecycle.phase == ObservationLifecyclePhase.OBSERVE_CLEAR
    assert document["before"]["state"] == "ALIGNED"
    assert document["after"]["state"] == "FOLD_1_COMPLETE"
    assert document["action"]["primitive"] == "bimanual_edge_pair"
    assert document["motion_authorized"] is False
    assert document["motion_commands"] == 0
    assert document["execution_api_used"] is False


def test_lifecycle_rejects_stale_future_prephase_and_nonmonotonic_frames():
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=1_000_000_000
    )
    with pytest.raises(TowelTaskContractError, match="predates"):
        lifecycle.observe(
            observation(1, 999_999_999), now_ns=1_000_000_000
        )
    with pytest.raises(TowelTaskContractError, match="future"):
        lifecycle.observe(
            observation(2, 1_100_000_000), now_ns=1_099_999_999
        )
    with pytest.raises(TowelTaskContractError, match="freshness"):
        lifecycle.observe(
            observation(3, 1_100_000_000), now_ns=1_600_000_001
        )
    lifecycle.observe(
        observation(4, 1_200_000_000), now_ns=1_210_000_000
    )
    with pytest.raises(TowelTaskContractError, match="monotonically"):
        lifecycle.observe(
            observation(5, 1_200_000_000), now_ns=1_210_000_000
        )
    assert lifecycle.rejected_observations == 4


@pytest.mark.parametrize(
    ("changes", "message"),
    (
        ({"settled": False}, "settled"),
        ({"clear_pose_verified": False}, "clear pose"),
        ({"clear_view_valid": False}, "clear view"),
        ({"model_sha256": None}, "identity metadata"),
    ),
)
def test_lifecycle_rejects_unsettled_unclear_or_unidentified_frames(
    changes, message
):
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=1_000_000_000
    )
    with pytest.raises(TowelTaskContractError, match=message):
        lifecycle.observe(
            observation(1, 1_100_000_000, **changes),
            now_ns=1_110_000_000,
        )


def test_lifecycle_pins_calibration_perception_and_robot_model_identities():
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=1_000_000_000
    )
    lifecycle.observe(
        observation(1, 1_100_000_000), now_ns=1_110_000_000
    )
    for index, changes, message in (
        (2, {"calibration_sha256": "d" * 64}, "calibration"),
        (3, {"model_sha256": "d" * 64}, "model identity"),
        (4, {"robot_model_sha256": "d" * 64}, "robot model"),
    ):
        with pytest.raises(TowelTaskContractError, match=message):
            lifecycle.observe(
                observation(index, 1_100_000_000 + index * 10_000_000, **changes),
                now_ns=1_110_000_000 + index * 10_000_000,
            )


def test_observation_is_forbidden_during_primitive_and_settle_is_enforced():
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=900_000_000
    )
    approve_initial_window(lifecycle)
    lifecycle.begin_primitive(
        action_id="fold-01",
        primitive="bimanual_edge_pair",
        now_ns=1_300_000_000,
    )
    with pytest.raises(TowelTaskContractError, match="forbidden"):
        lifecycle.observe(
            observation(4, 1_310_000_000), now_ns=1_320_000_000
        )
    lifecycle.finish_primitive(now_ns=1_400_000_000)
    with pytest.raises(TowelTaskContractError, match="has not elapsed"):
        lifecycle.begin_reobserve_clear(now_ns=2_149_999_999)
    lifecycle.begin_reobserve_clear(now_ns=2_150_000_000)


def test_state_flicker_never_approves_a_before_window():
    lifecycle = TowelObservationLifecycle.from_contract(
        contract(), started_ns=900_000_000
    )
    lifecycle.observe(
        observation(1, 1_000_000_000), now_ns=1_010_000_000
    )
    lifecycle.observe(
        observation(
            2,
            1_100_000_000,
            visible_area_ratio=0.2,
            flatness_score=0.1,
            corners=[],
        ),
        now_ns=1_110_000_000,
    )
    assert lifecycle.observe(
        observation(3, 1_200_000_000), now_ns=1_210_000_000
    ) is None
    assert lifecycle.unstable_windows == 1
    with pytest.raises(TowelTaskContractError, match="before-observation"):
        lifecycle.begin_primitive(
            action_id="unsafe",
            primitive="bimanual_edge_pair",
            now_ns=1_300_000_000,
        )


def test_contract_rejects_relaxed_or_motion_enabled_lifecycle():
    document = contract()
    document["observation_lifecycle"]["motion_authorized"] = True
    with pytest.raises(TowelTaskContractError, match="motion-locked"):
        validate_towel_contract(document)
    document = contract()
    document["observation_lifecycle"][
        "minimum_consecutive_observations"
    ] = 1
    with pytest.raises(TowelTaskContractError, match=">= 2"):
        validate_towel_contract(document)
