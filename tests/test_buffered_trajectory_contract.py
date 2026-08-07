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


def test_machine_contract_is_physically_commissioned_and_fail_closed() -> None:
    contract = load_buffered_trajectory_contract(CONTRACT_PATH)

    assert contract["status"] == "PHYSICAL_ACTION_COMMISSIONED"
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
        "physical_execution_deployed": True,
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
        "initial_first_sample_lead_ms": 220,
        "physical_uart_baud": 115200,
        "startup_prime_wire_lower_bound_ms": 87.674,
        "startup_anchor_wire_margin_ms": 52.326,
        "startup_prime_elapsed_window_ms": [120, 140],
        "startup_prime_maximum_heartbeat_gates": 8,
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
        "post_settle_timeout_s": 2.5,
        "post_settle_maximum_timeout_s": 10.0,
        "post_settle_poll_interval_s": 0.1,
        "post_settle_tolerance_raw": 30,
        "post_settle_consecutive_snapshots": 2,
        "post_settle_position_source": "position_only_get_state",
        "post_settle_terminal_heartbeat_gate": True,
        "post_settle_position_heartbeat_before_each_snapshot": True,
        "post_settle_full_diagnostics_after_position_gate": 1,
        "post_settle_full_diagnostics_heartbeat_gate": True,
        "post_settle_failure_diagnostics": [
            "per_observation_axis_errors_raw",
            "per_axis_minimum_error_raw",
            "final_axis_errors_raw",
            "best_maximum_error_raw",
            "observation_count",
            "heartbeat_gate_count",
        ],
        "post_settle_automatic_retry": False,
        "commissioning_observable_motion_gate": True,
        "commissioning_minimum_command_delta_raw": 16,
        "commissioning_minimum_directional_progress_raw": 10,
        "commissioning_selected_joint_target_tolerance_raw": 8,
        "commissioning_other_axis_tolerance_raw": 30,
        "commissioning_tool_disables_after_attempt": True,
        "failed_attempt_disable_confirmation": "disable_ack_latch_preserved",
        "commissioning_motion_passed": True,
        "ros_action_server_connected": True,
        "deployed": True,
        "motion_authorized": False,
    }
    assert contract["motion9_physical_evidence"] == {
        "status": "PASS",
        "plan_sha256": (
            "d29086a5ad699ac9229113ee730d815429323a661bdf3b257222e9a9b1eadb0d"
        ),
        "sender_sha256": (
            "d66f26f7b3907fda1988895a01e657bafa902ea901396a2c38f8524f16e93671"
        ),
        "execution_log_sha256": (
            "80f14845bab532de3217fcee7a9c4c2b0b5cf4241b65023844d6ba7d615de087"
        ),
        "firmware_version": "0x00022100",
        "calibration_hash": "0xB317C672",
        "duration_ms": 1200,
        "sample_count": 61,
        "action_send_count": 1,
        "automatic_retry_count": 0,
        "maximum_apply_lateness_ms": 4,
        "firmware_post_settle_max_error_raw": 6,
        "independent_round_trip_max_error_raw": 6,
        "physical_disable_6axis_pass": True,
        "abnormal_noise_or_vibration": False,
        "motion_authorized": False,
    }


def test_joint_limit_margin_route_is_deployed_and_unauthorized() -> None:
    contract = load_buffered_trajectory_contract(CONTRACT_PATH)

    assert contract["servo_uart_receive_candidate"] == {
        "status": "LOCAL_JOINT_LIMIT_MARGIN_DEPLOYED",
        "firmware_version": "0x00022C00",
        "previous_candidate_firmware_version": "0x00022B00",
        "previous_deployed_firmware_version": "0x00022B00",
        "baud": 1_000_000,
        "rx_fifo_enabled": False,
        "receive_api": "HAL_UARTEx_ReceiveToIdle_DMA",
        "dma_mode": "circular",
        "dma_ring_capacity_bytes": 256,
        "armed_before_first_request": False,
        "rx_dma_lifecycle": "transaction_scoped_lazy_arm",
        "disarm_after_transaction": True,
        "rearm_on_idle_or_transfer_complete": False,
        "rx_idle_bias": "internal_pull_up",
        "idle_high_stable_ms": 2,
        "idle_high_timeout_ms": 20,
        "receiver_hard_resync": "usart_re_disable_enable",
        "hal_error_irq_abort_disabled": True,
        "uart_error_capture": "polled_flags_with_callback_fallback",
        "dma_active_gate": [
            "software_started",
            "uart_dmar",
            "dma_channel_enabled",
            "rx_state_busy",
        ],
        "pre_transaction_quarantine": True,
        "transaction_window_max_bytes": 64,
        "transaction_timeout_ms": 50,
        "parser_state_preserved_across_bursts": True,
        "resynchronizes_split_stale_corrupt_responses": True,
        "soft_error_policy": "PE_NE_checksum_resynchronize",
        "hard_error_policy": "FE_ORE_RTO_DMA_fail_closed_receiver_resync",
        "diagnosed_uart_errors": ["PE", "NE", "FE", "ORE", "RTO", "DMA"],
        "recovery_action": "preserve_snapshot_abort_toggle_re_leave_unarmed",
        "failure_snapshot_bytes": 16,
        "extended_health_schema_version": 2,
        "internal_read_retry_count": 3,
        "feedback_fail_closed_count": 3,
        "apply_lateness_histogram_buckets": 6,
        "apply_lateness_worst_sample_index_reported": True,
        "host_uart_baud": 115_200,
        "buffered_status_acknowledgement_payload_bytes": 32,
        "buffered_status_terminal_payload_bytes": 60,
        "buffered_status_acknowledgement_transmit_ms": 4.688,
        "apply_lateness_allowance_ms": 5,
        "host_frame_transmit_is_blocking": True,
        "buffered_execution_servo_reads": False,
        "motion_safety_polling_during_buffered_execution": False,
        "host_heartbeat_response_budget_ms": 400,
        "mcu_heartbeat_watchdog_ms": 500,
        "host_frame_tx_accounting": True,
        "diagnostics_payload_bytes": 146,
        "deployed": True,
        "motion_authorized": False,
    }



def test_continuous_pick_place_route_is_plan_only_and_split_at_the_gripper() -> None:
    """leg 경계가 gripper 동작 지점이라는 사실을 계약으로 고정한다.

    buffered 실행에는 load/current 감시가 없다. `Servo_MotionSafetyPoll` 은
    비버퍼드 경로에만 있다. 경계가 조용히 옮겨져 접촉이 buffered leg 안으로
    들어가면 그 구간이 통째로 무감시가 된다.
    """
    contract = load_buffered_trajectory_contract(CONTRACT_PATH)
    route = contract["continuous_pick_place_candidate"]

    assert route["deployed"] is False
    assert route["motion_authorized"] is False
    assert route["action_count"] == 3
    assert route["leg_boundaries_are_gripper_actions"] is True
    assert route["gripper_moves_inside_a_buffered_leg"] is False
    assert route["buffered_execution_has_load_current_monitoring"] is False
    assert route["gripper_command_path_has_load_current_monitoring"] is True
    # 2026-08-06 대조군 실측으로 확정됐다.
    assert route["gripper_contact_behavior_measured"] is True
    measurement = route["gripper_contact_measurement"]
    # reached_goal 은 네 회차 모두 True 였다. 파지 증거가 아니다.
    assert measurement["reached_goal_is_contact_evidence"] is False
    # 잔여 간격이 판별자다. 기준 5, 접촉 23, 임계는 그 사이 14.
    assert measurement["control_close_residual_raw"] < (
        measurement["minimum_contact_gap_raw"]
    ) < measurement["contact_close_residual_raw"]
    # 접촉 close 가 정상 종료하므로 leg 사이 gripper 가 다음 leg 를 막지 않는다.
    assert measurement["contact_close_terminates_normally"] is True
    assert measurement["contact_close_latches"] is False
    # 사이에 q0 복귀가 없어야 "연속" 이다.
    assert route["q0_return_between_legs"] is False
    assert route["admission_simulation_underflow"] == 0
    assert route["maximum_batch_samples"] <= 9

    chain = [route["legs"][0]["start_pose"]]
    for leg in route["legs"]:
        assert leg["start_pose"] == chain[-1]
        chain.extend(leg["waypoints"])
    assert chain == [
        "q0", "pick_pregrasp", "pick_grasp",
        "lift20", "place_pregrasp", "place",
        "retreat", "q0",
    ]
    assert [leg["gripper_action_after"] for leg in route["legs"]] == [
        "pick_close", "place_release", None
    ]


def test_continuous_pick_place_cannot_move_the_gripper_inside_a_leg(
    tmp_path: Path,
) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["continuous_pick_place_candidate"][
        "gripper_moves_inside_a_buffered_leg"
    ] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(
        BufferedTrajectoryContractError, match="split at"
    ):
        load_buffered_trajectory_contract(path)


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
        match="physical execution",
    ):
        load_buffered_trajectory_contract(path)


def test_motion9_physical_evidence_cannot_be_weakened(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["motion9_physical_evidence"][
        "maximum_apply_lateness_ms"
    ] = 5
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(
        BufferedTrajectoryContractError,
        match="Motion-9 physical evidence",
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

    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022C00)" in config
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


def test_fresh_segment_leg_route_refuses_endpoint_only_plans() -> None:
    """갓 계획한 경로를 실행하는 모드는 안전 논리가 다르다.

    `ros_moveit_plan_grasp.py` 는 궤적 점을 저장하지 않는다. 그 출력만으로
    실행하면 MoveIt 이 검사하지 않은 경로를 직선으로 이어 달리게 된다.
    경계된 segment 체인을 요구하는 것이 이 모드의 전부다.
    """
    route = load_buffered_trajectory_contract(CONTRACT_PATH)[
        "fresh_segment_leg_candidate"
    ]
    assert route["deployed"] is False
    assert route["motion_authorized"] is False
    assert route["route_source"] == "planned_in_this_session"
    assert route["accepts_endpoint_only_plans"] is False
    assert route["requires_bounded_segment_chain"] is True
    assert route["requires_straight_joint_space_chain"] is True
    assert route["requires_moveit_success_per_segment"] is True
    # 계획과 실행 사이에 경로 파일이 바뀌면 거부되어야 한다.
    assert route["segment_sha_rechecked_at_execution"] is True
    assert route["plan_recomputed_at_execution"] is True
    assert route["gripper_moves_inside_a_leg"] is False


def test_fresh_segment_leg_cannot_accept_endpoint_only_plans(
    tmp_path: Path,
) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["fresh_segment_leg_candidate"]["accepts_endpoint_only_plans"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(
        BufferedTrajectoryContractError, match="endpoint-only"
    ):
        load_buffered_trajectory_contract(path)


def test_pick_and_place_tcp_offsets_are_separated_and_honest() -> None:
    """한쪽 측정이 다른 쪽을 검증한 것처럼 보이면 안 된다.

    2026-08-06 A4: Pick 은 gripper 잔여 간격으로 실측했다(0.025 에서 3 raw
    = 놓침, 0.017 에서 20 raw = 파지). Place 는 Stage 7 이 -5 mm 보정 2회를
    필요로 했다는 정황만 있고 측정된 적이 없다.
    """
    offsets = load_buffered_trajectory_contract(CONTRACT_PATH)[
        "tcp_contact_offsets"
    ]
    assert offsets["pick_and_place_offsets_separated"] is True
    assert offsets["pick_offset_measured"] is True
    assert offsets["place_offset_measured"] is False
    assert offsets["deployed"] is False
    assert offsets["motion_authorized"] is False

    # 측정된 Pick 값은 공칭보다 낮아야 한다. 공칭으로는 놓쳤다.
    assert offsets["pick_grasp_offset_m"] < offsets["nominal_grasp_offset_m"]
    # Place 는 아직 공칭 그대로임을 드러낸다.
    assert offsets["place_grasp_offset_m"] == offsets["nominal_grasp_offset_m"]

    held = [e for e in offsets["sweep"] if e["held"] is True]
    missed = [e for e in offsets["sweep"] if e["held"] is False]
    assert held and missed
    # 잡힌 높이가 놓친 높이보다 낮아야 한다.
    assert max(e["grasp_offset_m"] for e in held) < min(
        e["grasp_offset_m"] for e in missed
    )
    # 임계가 대조군과 실측 파지 사이에 있어야 한다.
    assert offsets["control_close_residual_raw"] < offsets[
        "contact_threshold_raw"
    ] <= min(e["residual_gap_raw"] for e in held)


def test_contract_refuses_a_place_offset_claimed_as_measured(
    tmp_path: Path,
) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["tcp_contact_offsets"]["place_offset_measured"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(
        BufferedTrajectoryContractError, match="not measured yet"
    ):
        load_buffered_trajectory_contract(path)


def test_top_shadow_grasp_route_replaces_the_publisher_lock_with_its_own() -> None:
    """인식 결과를 파지 좌표로 쓰는 것은 잠금을 넘는 행위다.

    `ShadowObjectTarget` 은 "Never consume this as a motion goal" 로 시작한다.
    넘으려면 대신할 게이트가 있어야 하고, 그것이 무엇인지가 계약에 있어야 한다.
    """
    route = load_buffered_trajectory_contract(CONTRACT_PATH)[
        "top_shadow_grasp_candidate"
    ]
    assert route["deployed"] is False
    assert route["motion_authorized"] is False
    assert route["message_forbids_direct_motion_use"] is True
    assert route["publisher_must_not_claim_authority"] is True
    assert route["collision_checked_per_run"] is True
    assert route["operator_approves_each_descent"] is True
    # 흔들림 한계는 인식 정확도보다 작아야 게이트 구실을 한다.
    assert route["maximum_position_spread_m"] < route[
        "measured_perception_error_position_m"
    ]
    assert route["maximum_yaw_spread_rad"] < route[
        "measured_perception_error_yaw_rad"
    ]
    # 파지 높이는 A4 실측값을 참조해야 한다. 여기에 다시 적으면 갈라진다.
    assert route["grasp_offset_m_source"] == (
        "tcp_contact_offsets.pick_grasp_offset_m"
    )


def test_contract_refuses_shadow_spread_as_wide_as_the_perception_error(
    tmp_path: Path,
) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    shadow = contract["top_shadow_grasp_candidate"]
    shadow["maximum_position_spread_m"] = shadow[
        "measured_perception_error_position_m"
    ]
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(
        BufferedTrajectoryContractError, match="inside the measured perception"
    ):
        load_buffered_trajectory_contract(path)


def test_contact_boundary_is_recorded_as_unmapped() -> None:
    """0.017 은 대상 펜에서 확인됐지만 경계는 재지 않았다."""
    offsets = load_buffered_trajectory_contract(CONTRACT_PATH)[
        "tcp_contact_offsets"
    ]
    assert offsets["contact_boundary_mapped"] is False
    assert "물체 크기가 바뀌면" in offsets["boundary_note"]
