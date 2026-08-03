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

    assert contract["status"] == "LOCAL_ACTION_INTEGRATION_CANDIDATE"
    assert contract["motion_authorized"] is False
    assert contract["current_runtime"] == {
        "firmware_supports_buffered_execution": True,
        "action_adapter_supports_buffered_execution": True,
        "maximum_accepted_sample_count": None,
        "execution_mode": "streamed_20ms_batches",
    }
    assert contract["firmware_candidate"] == {
        "core_executor_implemented": True,
        "buffered_command_route_candidate_implemented": True,
        "extended_terminal_status_candidate_implemented": True,
        "g474_cross_build_compiles_source": True,
        "binary_command_route_connected": True,
        "binary_command_route_mode": "validation_and_physical_separated",
        "validation_only_safety_state": "safe_disabled_read_only_allowed",
        "host_candidate_codec_implemented": True,
        "host_timing_analysis_implemented": True,
        "firmware_identity_changed": True,
        "capability_advertised": True,
        "host_fail_closed_capability_required": True,
        "timing_parameters_measured": True,
        "physical_execution_route_implemented": True,
        "physical_execution_capability": "0x00000800",
        "physical_execution_deployed": False,
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
        "initial_first_sample_lead_ms": 140,
        "physical_uart_baud": 115200,
        "startup_prime_wire_lower_bound_ms": 87.674,
        "startup_anchor_wire_margin_ms": 32.326,
        "fresh_start_wire_sample_included": True,
        "firmware_anchor_lead_ms": 80,
        "firmware_anchor_source": "validated_t0_wire_sample",
        "startup_prime_depth_samples": 16,
        "low_watermark_samples": 10,
        "refill_target_samples": 16,
        "maximum_samples_per_batch": 9,
        "gripper_position_preserved": True,
        "ack_accounting_fail_closed": True,
        "extended_motion_result_mapping": True,
        "terminal_state_reason_gate": True,
        "transport_timeout_discards_pending": True,
        "outbound_frame_encoder_implemented": True,
        "mock_exchange_driver_implemented": True,
        "response_sequence_gate": True,
        "automatic_retransmission": False,
        "clock_progress_source": (
            "firmware_heartbeat_tick_after_5ms_lateness_margin"
        ),
        "clock_progress_cannot_create_success": True,
        "firmware_terminal_required": True,
        "ros_action_server_connected": True,
        "transport_execution_connected": True,
        "motion_authorized": False,
    }
    assert contract["physical_execution_candidate"] == {
        "firmware_version": "0x00022100",
        "capabilities": "0x00000FFF",
        "validation_route_preserved": True,
        "execution_route_separate": True,
        "fresh_t0_anchor_without_servo_read_sweep": True,
        "executor_step_period_ms": 1,
        "servo_sync_write_period_ms": 5,
        "validation_maximum_apply_lateness_ms": 0,
        "execution_maximum_apply_lateness_ms": 5,
        "lateness_exceeded_action": "missed_apply_tick_safe_stop",
        "success_terminal_detail": "maximum_apply_lateness_ms",
        "sample_period_ms": 20,
        "minimum_lead_ms": 60,
        "maximum_lead_ms": 400,
        "startup_prime_depth_samples": 16,
        "terminal_safe_stop_mapping": True,
        "firmware_terminal_scope": "setpoint_application_complete",
        "host_success_requires_post_settle": True,
        "post_settle_tolerance_raw": 30,
        "post_settle_consecutive_snapshots": 2,
        "commissioning_observable_motion_gate": True,
        "commissioning_minimum_command_delta_raw": 16,
        "commissioning_minimum_directional_progress_raw": 10,
        "commissioning_selected_joint_target_tolerance_raw": 8,
        "commissioning_other_axis_tolerance_raw": 30,
        "commissioning_tool_disables_after_attempt": True,
        "failed_attempt_disable_confirmation": "disable_ack_latch_preserved",
        "commissioning_motion_passed": True,
        "ros_action_server_connected": True,
        "deployed": False,
        "motion_authorized": False,
    }


def test_machine_contract_cannot_enable_motion(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["motion_authorized"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="motion_authorized"):
        load_buffered_trajectory_contract(path)


def test_machine_contract_cannot_disconnect_action_runtime(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["current_runtime"]["action_adapter_supports_buffered_execution"] = False
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="streamed Action"):
        load_buffered_trajectory_contract(path)


def test_machine_contract_cannot_equate_terminal_with_servo_settle(
    tmp_path: Path,
) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["physical_execution_candidate"][
        "host_success_requires_post_settle"
    ] = False
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(
        BufferedTrajectoryContractError,
        match="physical execution candidate",
    ):
        load_buffered_trajectory_contract(path)


def test_firmware_candidate_cannot_return_to_dormant_route(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["firmware_candidate"]["binary_command_route_connected"] = False
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BufferedTrajectoryContractError, match="remain separated"):
        load_buffered_trajectory_contract(path)


def test_reviewed_timing_values_cannot_be_weakened(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["timing_analysis"]["minimum_lead_ms"] = 20
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(BufferedTrajectoryContractError, match="reviewed"):
        load_buffered_trajectory_contract(path)


def test_g474_identity_advertises_separate_validation_and_execution_routes() -> None:
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

    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022100)" in config
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)" in config
    assert "HOST_BUFFERED_VALIDATION_CAPABILITY UINT32_C(0x00000400)" in config
    assert "HOST_BUFFERED_EXECUTION_CAPABILITY UINT32_C(0x00000800)" in config
    assert "if (sample_count == 1U)" in binary_control
    assert '#include "actuator_core/buffered_command_route.h"' in binary_control
    assert "Host_ValidateBufferedCandidate(request);" in binary_control
    assert "Host_ExecuteBufferedCandidate(request);" in binary_control


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
    validated = validate(arm_contract, non_millisecond)
    assert validated.duration_ms == 520


def test_zero_time_point_must_match_fresh_feedback(arm_contract) -> None:
    points = [point(0.02, 0), point(0.03, 1000)]

    with pytest.raises(GoalValidationError, match="fresh start tolerance"):
        validate(arm_contract, points)


def test_moveit_dynamic_fields_are_validated_before_position_resampling(
    arm_contract,
) -> None:
    points = [
        point(0.0, 0),
        TrajectoryPointData(
            (0.01,) * 5,
            1_000_000_000,
            velocities=(0.01,) * 5,
        ),
    ]

    assert validate(arm_contract, points).duration_ms == 1000

    invalid = [
        point(0.0, 0),
        TrajectoryPointData(
            (0.01,) * 5,
            1_000_000_000,
            velocities=(0.51,) * 5,
        ),
    ]
    with pytest.raises(GoalValidationError, match="declared velocity exceeds"):
        validate(arm_contract, invalid)

    effort = [
        point(0.0, 0),
        TrajectoryPointData(
            (0.01,) * 5,
            1_000_000_000,
            effort=(0.0,) * 5,
        ),
    ]
    with pytest.raises(GoalValidationError, match="effort"):
        validate(arm_contract, effort)


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
