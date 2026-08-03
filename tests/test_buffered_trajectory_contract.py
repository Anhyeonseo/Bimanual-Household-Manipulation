import json
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.action_validation import (  # noqa: E402
    GoalValidationError,
    TrajectoryPointData,
    interpolate_buffered_trajectory,
    validate_buffered_trajectory,
)
from single_arm_bridge.buffered_trajectory import (  # noqa: E402
    BufferedQueueError,
    BufferedQueueState,
    BufferedSetpointQueueModel,
    BufferedTrajectoryContractError,
    ScheduledSetpoint,
    load_buffered_trajectory_contract,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402


CONTRACT_PATH = (
    PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
)
CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


@pytest.fixture(scope="module")
def arm_contract():
    calibration = load_calibration(CALIBRATION_PATH)
    names = tuple(calibration.ros_joint_names[:5])
    return {
        "names": names,
        "limits": {
            name: calibration.ros_radian_limits[name] for name in names
        },
        "velocity": {name: 0.5 for name in names},
        "acceleration": {name: 1.0 for name in names},
    }


def point(value: float, time_ms: int, *, count: int = 5):
    return TrajectoryPointData(
        positions=tuple(value for _ in range(count)),
        time_from_start_ns=time_ms * 1_000_000,
    )


def validate(arm_contract, points, *, names=None, start=None):
    return validate_buffered_trajectory(
        names or arm_contract["names"],
        points,
        arm_contract["names"],
        arm_contract["limits"],
        start or (0.0,) * 5,
        arm_contract["velocity"],
        arm_contract["acceleration"],
        start_tolerance_rad=0.01,
    )


def sample(tick: int, value: float = 0.0) -> ScheduledSetpoint:
    return ScheduledSetpoint(tick, (value,) * 6)


def queue_model(**overrides) -> BufferedSetpointQueueModel:
    values = {
        "joint_count": 6,
        "capacity_samples": 4,
        "maximum_batch_samples": 3,
        "minimum_start_samples": 2,
        "minimum_lead_ms": 5,
        "maximum_lead_ms": 100,
    }
    values.update(overrides)
    return BufferedSetpointQueueModel(**values)


def test_machine_contract_is_mock_only_and_fail_closed() -> None:
    contract = load_buffered_trajectory_contract(CONTRACT_PATH)

    assert contract["status"] == "BOARD_VALIDATION_ONLY"
    assert contract["motion_authorized"] is False
    assert contract["current_runtime"] == {
        "firmware_supports_buffered_execution": False,
        "action_adapter_supports_buffered_execution": False,
        "maximum_accepted_sample_count": 1,
    }
    assert contract["firmware_candidate"] == {
        "core_executor_implemented": True,
        "buffered_command_route_candidate_implemented": True,
        "extended_terminal_status_candidate_implemented": True,
        "g474_cross_build_compiles_source": True,
        "binary_command_route_connected": True,
        "binary_command_route_mode": "validation_only",
        "validation_only_safety_state": "safe_disabled_read_only_allowed",
        "host_candidate_codec_implemented": True,
        "host_timing_analysis_implemented": True,
        "firmware_identity_changed": True,
        "capability_advertised": True,
        "host_fail_closed_capability_required": True,
        "timing_parameters_measured": True,
    }
    assert contract["timing_analysis"]["operational_values_authorized"] is True
    assert contract["timing_analysis"]["motion_authorized"] is False
    assert contract["timing_analysis"]["minimum_lead_ms"] == 60
    assert contract["timing_analysis"]["maximum_lead_ms"] == 400
    assert contract["timing_analysis"]["startup_prime_depth_samples"] == 16
    assert contract["timing_analysis"]["low_watermark_samples"] == 10
    assert contract["timing_analysis"]["refill_target_samples"] == 16
    assert contract["host_adapter_candidate"] == {
        "multi_point_validation_reused": True,
        "linear_resampling_period_ms": 20,
        "initial_first_sample_lead_ms": 100,
        "startup_prime_depth_samples": 16,
        "low_watermark_samples": 10,
        "refill_target_samples": 16,
        "maximum_samples_per_batch": 9,
        "gripper_position_preserved": True,
        "ack_accounting_fail_closed": True,
        "automatic_retransmission": False,
        "ros_action_server_connected": False,
        "transport_execution_connected": False,
        "motion_authorized": False,
    }


def test_machine_contract_cannot_enable_motion(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["motion_authorized"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="motion_authorized"):
        load_buffered_trajectory_contract(path)


def test_machine_contract_cannot_claim_runtime_support(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["current_runtime"]["firmware_supports_buffered_execution"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="runtime gate"):
        load_buffered_trajectory_contract(path)


def test_firmware_candidate_cannot_return_to_dormant_route(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["firmware_candidate"]["binary_command_route_connected"] = False
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="validation-only"):
        load_buffered_trajectory_contract(path)


def test_reviewed_timing_values_cannot_be_weakened(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["timing_analysis"]["minimum_lead_ms"] = 20
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(BufferedTrajectoryContractError, match="reviewed"):
        load_buffered_trajectory_contract(path)


def test_g474_runtime_identity_advertises_validation_only_candidate() -> None:
    config = (
        ROOT
        / "firmware"
        / "stm32_g474_single_arm"
        / "Core"
        / "Inc"
        / "single_arm_config.h"
    ).read_text(encoding="utf-8")
    binary_control = (
        ROOT
        / "firmware"
        / "stm32_g474_single_arm"
        / "Core"
        / "Src"
        / "binary_control.c"
    ).read_text(encoding="utf-8")

    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00021900)" in config
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x000007FF)" in config
    assert "HOST_BUFFERED_VALIDATION_CAPABILITY UINT32_C(0x00000400)" in config
    assert "if (sample_count == 1U)" in binary_control
    assert '#include "actuator_core/buffered_command_route.h"' in binary_control
    assert "Host_ValidateBufferedCandidate(request);" in binary_control


def test_valid_multi_point_path_reorders_joints_and_interpolates(
    arm_contract,
) -> None:
    names = tuple(reversed(arm_contract["names"]))
    first_by_name = {name: 0.0 for name in names}
    second_by_name = {
        name: 0.02 * (index + 1)
        for index, name in enumerate(arm_contract["names"])
    }
    third_by_name = {
        name: 0.04 * (index + 1)
        for index, name in enumerate(arm_contract["names"])
    }
    points = (
        TrajectoryPointData(
            tuple(first_by_name[name] for name in names),
            0,
        ),
        TrajectoryPointData(
            tuple(second_by_name[name] for name in names),
            1_000_000_000,
        ),
        TrajectoryPointData(
            tuple(third_by_name[name] for name in names),
            2_000_000_000,
        ),
    )

    trajectory = validate(arm_contract, points, names=names)

    assert trajectory.duration_ms == 2000
    assert trajectory.ordered_points[1] == tuple(
        second_by_name[name] for name in arm_contract["names"]
    )
    assert interpolate_buffered_trajectory(trajectory, 1_500_000_000) == pytest.approx(
        tuple(0.03 * (index + 1) for index in range(5))
    )
    with pytest.raises(GoalValidationError, match="non-negative"):
        interpolate_buffered_trajectory(trajectory, -1)
    assert interpolate_buffered_trajectory(trajectory, 3_000_000_000) == (
        trajectory.ordered_points[-1]
    )


def test_multi_point_requires_two_points_and_strict_millisecond_time(
    arm_contract,
) -> None:
    with pytest.raises(GoalValidationError, match="at least two"):
        validate(arm_contract, [point(0.0, 0)])

    non_monotonic = [point(0.0, 0), point(0.01, 500), point(0.02, 500)]
    with pytest.raises(GoalValidationError, match="strictly increasing"):
        validate(arm_contract, non_monotonic)

    non_millisecond = [
        point(0.0, 0),
        TrajectoryPointData((0.01,) * 5, 500_000_001),
    ]
    with pytest.raises(GoalValidationError, match="integer milliseconds"):
        validate(arm_contract, non_millisecond)


def test_zero_time_point_must_match_fresh_feedback(arm_contract) -> None:
    points = [point(0.02, 0), point(0.03, 1000)]

    with pytest.raises(GoalValidationError, match="fresh start tolerance"):
        validate(arm_contract, points)


def test_dynamic_fields_are_never_silently_discarded(arm_contract) -> None:
    points = [
        point(0.0, 0),
        TrajectoryPointData(
            (0.01,) * 5,
            1_000_000_000,
            velocities=(0.01,) * 5,
        ),
    ]

    with pytest.raises(GoalValidationError, match="does not accept velocity"):
        validate(arm_contract, points)


def test_segment_velocity_and_acceleration_limits_are_enforced(
    arm_contract,
) -> None:
    velocity_violation = [point(0.0, 0), point(0.2, 300)]
    with pytest.raises(GoalValidationError, match="velocity exceeds"):
        validate(arm_contract, velocity_violation)

    acceleration_violation = [
        point(0.0, 0),
        point(0.05, 500),
        point(0.10, 600),
    ]
    with pytest.raises(GoalValidationError, match="acceleration exceeds"):
        validate(arm_contract, acceleration_violation)


def test_queue_batch_admission_is_atomic() -> None:
    queue = queue_model(capacity_samples=3, maximum_batch_samples=3)

    with pytest.raises(BufferedQueueError, match="strictly increasing"):
        queue.push_batch([sample(10), sample(9)], current_tick_ms=0)
    assert queue.snapshot().queued_samples == 0

    queue.push_batch([sample(10), sample(20)], current_tick_ms=0)
    with pytest.raises(BufferedQueueError, match="capacity"):
        queue.push_batch([sample(30), sample(40)], current_tick_ms=0)
    assert queue.snapshot().queued_samples == 2


def test_queue_rejects_invalid_lead_and_nonfinite_positions() -> None:
    queue = queue_model()

    with pytest.raises(BufferedQueueError, match="stale or too short"):
        queue.push_batch([sample(4)], current_tick_ms=0)
    with pytest.raises(BufferedQueueError, match="exceeds maximum"):
        queue.push_batch([sample(101)], current_tick_ms=0)
    with pytest.raises(BufferedQueueError, match="positions are invalid"):
        queue.push_batch(
            [ScheduledSetpoint(10, (0.0, 0.0, math.nan, 0.0, 0.0, 0.0))],
            current_tick_ms=0,
        )
    with pytest.raises(BufferedQueueError, match="positions are invalid"):
        queue.push_batch(
            [ScheduledSetpoint(10, (0.0, 0.0, "0", 0.0, 0.0, 0.0))],
            current_tick_ms=0,
        )


def test_queue_preserves_uint32_tick_order_across_wrap() -> None:
    queue = queue_model(minimum_start_samples=1)
    current = 0xFFFFFFF5
    queue.push_batch([sample(5)], current_tick_ms=current)
    queue.mark_input_complete()
    queue.start()

    assert queue.take_due(0xFFFFFFFF) is None
    assert queue.take_due(5) == sample(5)
    assert queue.state is BufferedQueueState.SUCCEEDED


def test_queue_primes_and_finishes_without_intermediate_stop() -> None:
    queue = queue_model()
    queue.push_batch([sample(10, 0.1), sample(20, 0.2)], current_tick_ms=0)
    queue.mark_input_complete()
    queue.start()

    assert queue.take_due(9) is None
    assert queue.take_due(10) == sample(10, 0.1)
    assert queue.state is BufferedQueueState.RUNNING
    assert queue.take_due(20) == sample(20, 0.2)
    assert queue.state is BufferedQueueState.SUCCEEDED
    assert queue.snapshot().safe_stop_required is False


def test_queue_underflow_enters_hold_and_requires_safe_stop() -> None:
    queue = queue_model()
    queue.push_batch([sample(10), sample(20)], current_tick_ms=0)
    queue.start()
    queue.take_due(10)
    queue.take_due(20)

    assert queue.take_due(21) is None
    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.HOLD
    assert snapshot.queued_samples == 0
    assert snapshot.safe_stop_required is True
    assert snapshot.reason == "queue_underflow"
    with pytest.raises(BufferedQueueError, match="only start once"):
        queue.start()


def test_missed_apply_tick_enters_fail_closed_hold() -> None:
    queue = queue_model(minimum_start_samples=1)
    queue.push_batch([sample(10)], current_tick_ms=0)
    queue.start()

    assert queue.take_due(11) is None
    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.HOLD
    assert snapshot.safe_stop_required is True
    assert snapshot.reason == "missed_apply_tick"


def test_cancel_clears_queue_and_cannot_resume() -> None:
    queue = queue_model()
    queue.push_batch([sample(10), sample(20)], current_tick_ms=0)
    queue.start()
    queue.cancel()

    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.CANCELED
    assert snapshot.queued_samples == 0
    assert snapshot.safe_stop_required is True
    with pytest.raises(BufferedQueueError, match="cannot accept"):
        queue.push_batch([sample(30)], current_tick_ms=0)


def test_planned_hold_is_distinct_from_latched_cancel() -> None:
    queue = queue_model()
    queue.push_batch([sample(10), sample(20)], current_tick_ms=0)
    queue.start()
    queue.planned_hold()

    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.HOLD
    assert snapshot.queued_samples == 0
    assert snapshot.safe_stop_required is False
    assert snapshot.reason == "planned_hold"


def test_connection_loss_aborts_without_automatic_resume() -> None:
    queue = queue_model()
    queue.push_batch([sample(10), sample(20)], current_tick_ms=0)
    queue.start()
    queue.connection_loss()

    snapshot = queue.snapshot()
    assert snapshot.state is BufferedQueueState.ABORTED
    assert snapshot.safe_stop_required is True
    assert snapshot.reason == "connection_loss"
    with pytest.raises(BufferedQueueError, match="only start once"):
        queue.start()
