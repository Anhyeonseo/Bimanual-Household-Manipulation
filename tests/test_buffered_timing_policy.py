from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from single_arm_bridge.buffered_timing import (
    BufferedTimingError,
    derive_buffered_timing_policy,
)
from single_arm_bridge.buffered_validation_capture import build_capture_document
from single_arm_bridge.buffered_trajectory import (
    BufferedQueueState,
    BufferedSetpointQueueModel,
    ScheduledSetpoint,
    load_buffered_trajectory_contract,
)


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
CONTRACT_PATH = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
POLICY_PATH = (
    ROOT / "artifacts" / "motion" / "2026-08-03" /
    "buffered_timing_policy_reviewed.json"
)


def capture(lead_ms: int, *, outage_ms: float = 80.064074) -> dict:
    return build_capture_document(
        firmware_version=0x00021900,
        calibration_hash=0xB317C672,
        capabilities=0x000007FF,
        requested_samples=1_000,
        interval_ms=20,
        lead_ms=lead_ms,
        sample_spacing_ms=20,
        serial_round_trip_ms=[17.43] * 1_000,
        host_command_jitter_ms=[0.063] * 1_000,
        delivery_lateness_ms=[0.0] * 1_000,
        host_outage_ms=[20.06, 40.06, outage_ms],
        transport_error_count=0,
    )


def derive(captures=None):
    return derive_buffered_timing_policy(
        captures or [capture(100), capture(80), capture(60), capture(380)],
        rejected_first_lead_ms=40,
        rejected_status_code=1,
        rejected_detail=9,
    )


def test_reviewed_policy_derives_measured_queue_values_without_motion() -> None:
    policy = derive()

    assert policy["status"] == "REVIEWED_DEPLOYMENT_INPUT"
    assert policy["measurement_input_authorized"] is True
    assert policy["operational_values_authorized"] is True
    assert policy["motion_authorized"] is False
    assert policy["deployment_values"] == {
        "sample_period_ms": 20,
        "minimum_lead_ms": 60,
        "maximum_lead_ms": 400,
        "startup_prime_depth_samples": 16,
        "low_watermark_samples": 10,
        "refill_target_samples": 16,
    }
    assert policy["derivation"]["recovery_consumption_samples"] == 9
    assert policy["derivation"]["full_queue_horizon_ms"] == 360


def test_machine_contract_pins_reviewed_policy_artifact() -> None:
    contract = load_buffered_trajectory_contract(CONTRACT_PATH)
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    digest = hashlib.sha256(POLICY_PATH.read_bytes()).hexdigest()

    assert contract["motion_authorized"] is False
    assert contract["timing_analysis"]["motion_authorized"] is False
    assert contract["timing_analysis"]["policy_sha256"] == digest
    assert policy["motion_authorized"] is False
    assert policy["deployment_values"] == {
        "sample_period_ms": contract["timing_analysis"]["sample_period_ms"],
        "minimum_lead_ms": contract["timing_analysis"]["minimum_lead_ms"],
        "maximum_lead_ms": contract["timing_analysis"]["maximum_lead_ms"],
        "startup_prime_depth_samples": contract["timing_analysis"][
            "startup_prime_depth_samples"
        ],
        "low_watermark_samples": contract["timing_analysis"][
            "low_watermark_samples"
        ],
        "refill_target_samples": contract["timing_analysis"][
            "refill_target_samples"
        ],
    }


def sample(tick_ms: int) -> ScheduledSetpoint:
    return ScheduledSetpoint(tick_ms, (0.0,) * 6)


def test_reviewed_queue_survives_outage_and_refills_without_time_gap() -> None:
    queue = BufferedSetpointQueueModel(
        joint_count=6,
        capacity_samples=16,
        maximum_batch_samples=9,
        minimum_start_samples=16,
        minimum_lead_ms=60,
        maximum_lead_ms=400,
    )
    queue.push_batch(
        [sample(tick) for tick in range(60, 221, 20)],
        current_tick_ms=0,
    )
    queue.push_batch(
        [sample(tick) for tick in range(240, 361, 20)],
        current_tick_ms=18,
    )
    assert queue.snapshot().queued_samples == 16
    queue.start()

    for tick in range(60, 161, 20):
        assert queue.take_due(tick) == sample(tick)
    assert queue.snapshot().queued_samples == 10

    # 80 ms outage + RTT + scheduler guard recovers before the 280 ms point.
    for tick in range(180, 261, 20):
        assert queue.take_due(tick) == sample(tick)
    assert queue.snapshot().queued_samples == 5
    queue.push_batch(
        [sample(tick) for tick in range(380, 541, 20)],
        current_tick_ms=278,
    )
    queue.push_batch(
        [sample(560), sample(580)],
        current_tick_ms=296,
    )
    assert queue.snapshot().queued_samples == 16
    queue.mark_input_complete()

    for tick in range(280, 581, 20):
        assert queue.take_due(tick) == sample(tick)
    assert queue.state is BufferedQueueState.SUCCEEDED


def test_underwatermark_without_refill_still_fails_closed() -> None:
    queue = BufferedSetpointQueueModel(
        joint_count=6,
        capacity_samples=16,
        maximum_batch_samples=9,
        minimum_start_samples=16,
        minimum_lead_ms=60,
        maximum_lead_ms=400,
    )
    queue.push_batch(
        [sample(tick) for tick in range(60, 221, 20)],
        current_tick_ms=0,
    )
    queue.push_batch(
        [sample(tick) for tick in range(240, 361, 20)],
        current_tick_ms=18,
    )
    queue.start()
    for tick in range(60, 361, 20):
        queue.take_due(tick)

    assert queue.take_due(380) is None
    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.HOLD
    assert snapshot.safe_stop_required is True
    assert snapshot.reason == "queue_underflow"


def test_policy_requires_fail_closed_lower_boundary() -> None:
    with pytest.raises(BufferedTimingError, match="exactly one sample"):
        derive_buffered_timing_policy(
            [capture(100), capture(80), capture(60), capture(380)],
            rejected_first_lead_ms=20,
            rejected_status_code=1,
            rejected_detail=9,
        )
    with pytest.raises(BufferedTimingError, match="queue rejection"):
        derive_buffered_timing_policy(
            [capture(100), capture(80), capture(60), capture(380)],
            rejected_first_lead_ms=40,
            rejected_status_code=0,
            rejected_detail=0,
        )


def test_policy_rejects_non_hardware_or_mismatched_capture() -> None:
    invalid = capture(80)
    invalid["source"]["provenance"] = "synthetic"
    with pytest.raises(BufferedTimingError, match="hardware gates"):
        derive([capture(100), invalid, capture(60), capture(380)])

    wrong_period = deepcopy(capture(80))
    wrong_period["capture_parameters"]["interval_ms"] = 10
    with pytest.raises(BufferedTimingError, match="period and spacing"):
        derive([capture(100), wrong_period, capture(60), capture(380)])


def test_policy_rejects_missing_maximum_horizon_evidence() -> None:
    with pytest.raises(BufferedTimingError, match="maximum horizon"):
        derive([capture(100), capture(80), capture(60), capture(200)])
