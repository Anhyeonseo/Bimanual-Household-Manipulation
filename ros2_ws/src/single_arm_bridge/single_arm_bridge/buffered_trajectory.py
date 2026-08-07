"""Host-only buffered trajectory queue model; never commands hardware."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any, Sequence

# 안전 허용치는 Action 이 소유한다. 여기서 값을 베껴 적으면 둘이 조용히
# 갈라지므로 정의 자체를 가져온다.
from .buffered_action_execution import (
    POST_SETTLE_TOLERANCE_RAW as POST_SETTLE_SAFETY_TOLERANCE_RAW,
)


CONTRACT_KIND = "single_arm_buffered_trajectory"
CONTRACT_STATUS = "PHYSICAL_ACTION_COMMISSIONED"
TOTAL_JOINT_COUNT = 6
ARM_JOINT_COUNT = 5
WIRE_BATCH_MAX_SAMPLES = 9
MCU_QUEUE_CAPACITY_SAMPLES = 16
UINT32_MAX = 0xFFFFFFFF
MAX_UNAMBIGUOUS_TICK_LEAD = 0x7FFFFFFF

REQUIRED_SAFETY = {
    "operator_cancel": "clear_queue_then_safe_stop_latched",
    "planned_hold": "clear_queue_then_hold_without_auto_resume",
    "queue_underflow": "hold_then_safe_stop_latched",
    "missed_apply_tick": "hold_then_safe_stop_latched",
    "connection_loss": "clear_queue_then_safe_stop_latched",
    "tracking_error": "abort_without_automatic_resend",
    "reconnect": "refuse_automatic_resume",
}

REQUIRED_MEASUREMENTS = [
    "sample_period_ms",
    "minimum_lead_ms",
    "maximum_lead_ms",
    "startup_prime_depth_samples",
    "low_watermark_samples",
    "refill_target_samples",
    "serial_round_trip_p95_ms",
    "serial_round_trip_p99_ms",
    "host_command_jitter_p95_ms",
    "delivery_lateness_p95_ms",
    "observed_max_host_outage_ms",
]


class BufferedTrajectoryContractError(ValueError):
    """Raised when a host-only buffered trajectory contract is weakened."""


class BufferedQueueError(RuntimeError):
    """Raised when the pure queue model is used outside its contract."""


class BufferedQueueState(Enum):
    PRIMING = "priming"
    RUNNING = "running"
    HOLD = "hold"
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class ScheduledSetpoint:
    apply_tick_ms: int
    positions_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class QueueSnapshot:
    state: BufferedQueueState
    queued_samples: int
    input_complete: bool
    last_applied_tick_ms: int | None
    safe_stop_required: bool
    reason: str | None


def _require_object(document: dict[str, Any], key: str) -> dict[str, Any]:
    value = document.get(key)
    if not isinstance(value, dict):
        raise BufferedTrajectoryContractError(f"{key} must be an object")
    return value


def _is_uint32(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= UINT32_MAX
    )


def _tick_delta(candidate: int, reference: int) -> int:
    return (candidate - reference) & UINT32_MAX


def _tick_is_after(candidate: int, reference: int) -> bool:
    delta = _tick_delta(candidate, reference)
    return 0 < delta <= MAX_UNAMBIGUOUS_TICK_LEAD


def _positions_are_valid(values: object, joint_count: int) -> bool:
    if not isinstance(values, (tuple, list)) or len(values) != joint_count:
        return False
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        if not math.isfinite(float(value)):
            return False
    return True


def validate_buffered_trajectory_contract(document: dict[str, Any]) -> None:
    if document.get("schema_version") != 1:
        raise BufferedTrajectoryContractError("schema_version must be 1")
    if document.get("contract_kind") != CONTRACT_KIND:
        raise BufferedTrajectoryContractError(
            f"contract_kind must be {CONTRACT_KIND}"
        )
    if document.get("status") != CONTRACT_STATUS:
        raise BufferedTrajectoryContractError(
            f"status must remain {CONTRACT_STATUS}"
        )
    if document.get("motion_authorized") is not False:
        raise BufferedTrajectoryContractError("motion_authorized must be false")

    runtime = _require_object(document, "current_runtime")
    if runtime != {
        "firmware_supports_buffered_execution": True,
        "action_adapter_supports_buffered_execution": True,
        "maximum_accepted_sample_count": None,
        "execution_mode": "streamed_20ms_batches",
    }:
        raise BufferedTrajectoryContractError(
            "local runtime must expose only the reviewed streamed Action route"
        )

    candidate = _require_object(document, "firmware_candidate")
    if candidate != {
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
    }:
        raise BufferedTrajectoryContractError(
            "firmware validation and physical routes must remain separated"
        )

    uart_candidate = _require_object(
        document,
        "servo_uart_receive_candidate",
    )
    if uart_candidate != {
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
    }:
        raise BufferedTrajectoryContractError(
            "joint limit margin route must stay deployed "
            "and unauthorized for motion"
        )

    # Motion-13. leg 경계는 gripper 동작 지점이다. buffered 실행에는
    # load/current 감시가 없으므로(`Servo_MotionSafetyPoll` 은 비버퍼드
    # 경로에만 있다) 접촉은 감시가 있는 gripper 명령에서만 일어나야 한다.
    # 이 dict 를 통째로 대조해 그 경계가 조용히 옮겨지지 못하게 한다.
    pick_place = _require_object(document, "continuous_pick_place_candidate")
    if pick_place != {
        "status": "LOCAL_CONTINUOUS_PICK_PLACE_PLAN_ONLY",
        "route_manifest_sha256": (
            "7c0d44a96dbd4ff214bf9858f1adff183f5fdc9079256ab4534c58d3a73e6d5c"
        ),
        "route_phase_count": 7,
        "route_arm_segment_count": 36,
        "route_is_piecewise_straight_joint_space": True,
        "route_chain_discontinuity_rad": 0.0,
        "key_pose_count": 7,
        "action_count": 3,
        "leg_boundaries_are_gripper_actions": True,
        "legs": [
            {
                "leg": "A",
                "start_pose": "q0",
                "waypoints": ["pick_pregrasp", "pick_grasp"],
                "duration_ms": 41000,
                "sample_count": 2051,
                "gripper_action_after": "pick_close",
            },
            {
                "leg": "B",
                "start_pose": "pick_grasp",
                "waypoints": ["lift20", "place_pregrasp", "place"],
                "duration_ms": 14000,
                "sample_count": 701,
                "gripper_action_after": "place_release",
            },
            {
                "leg": "C",
                "start_pose": "place",
                "waypoints": ["retreat", "q0"],
                "duration_ms": 39000,
                "sample_count": 1951,
                "gripper_action_after": None,
            },
        ],
        "q0_return_between_legs": False,
        "gripper_moves_inside_a_buffered_leg": False,
        "buffered_execution_has_load_current_monitoring": False,
        "gripper_command_path_has_load_current_monitoring": True,
        "gripper_contact_behavior_measured": True,
        "gripper_contact_measurement": {
            "measured_on": "2026-08-06",
            "close_command_raw": 1963,
            "control_close_residual_raw": 5,
            "contact_close_residual_raw": 23,
            "open_residual_raw": 6,
            "minimum_contact_gap_raw": 14,
            "reached_goal_is_contact_evidence": False,
            "contact_close_terminates_normally": True,
            "contact_close_latches": False,
            "arm_motion_during_contact_close_rad": 0.006136,
        },
        "anchor_deviation_limit_raw": 40,
        "tracking_error_carried_between_stages": True,
        "admission_simulation_underflow": 0,
        "maximum_batch_samples": 9,
        "deployed": False,
        "motion_authorized": False,
    }:
        raise BufferedTrajectoryContractError(
            "continuous Pick/Place route must stay plan-only and split at "
            "the gripper actions"
        )

    # A4 / A4.5. Motion-13 은 SHA 로 고정된 manifest 만 실행할 수 있다.
    # 갓 계획한 경로를 실행하는 모드는 안전 논리가 다르므로 따로 못 박는다.
    # 특히 endpoint 만 있는 계획을 받지 않는다 — `ros_moveit_plan_grasp.py` 는
    # 궤적 점을 저장하지 않아 MoveIt 이 검사한 경로를 재생할 수 없다.
    fresh_leg = _require_object(document, "fresh_segment_leg_candidate")
    if fresh_leg != {
        "status": "LOCAL_FRESH_SEGMENT_LEG_PLAN_ONLY",
        "purpose": "A4 grasp offset 재계측과 A4.5 Top 인식 기반 파지",
        "route_source": "planned_in_this_session",
        "accepts_endpoint_only_plans": False,
        "requires_bounded_segment_chain": True,
        "requires_straight_joint_space_chain": True,
        "requires_moveit_success_per_segment": True,
        "segment_sha_pinned_by_operator": True,
        "segment_sha_rechecked_at_execution": True,
        "plan_recomputed_at_execution": True,
        "anchor_deviation_limit_raw": 40,
        "gripper_moves_inside_a_leg": False,
        "maximum_batch_samples": 9,
        "deployed": False,
        "motion_authorized": False,
    }:
        raise BufferedTrajectoryContractError(
            "fresh segment leg route must stay plan-only and refuse "
            "endpoint-only plans"
        )

    # A4. Pick 과 Place 의 TCP-to-contact offset 은 서로 다른 값이며 각각
    # 따로 측정되어야 한다. Pick 은 2026-08-06 에 gripper 잔여 간격으로
    # 쟀고(공칭 0.025 에서 잔여 3 raw = 놓침, 0.017 에서 20 raw = 파지),
    # Place 는 아직 공칭값이다. 하나로 묶어두면 한쪽 측정이 다른 쪽을
    # 검증한 것처럼 보인다.
    offsets = _require_object(document, "tcp_contact_offsets")
    if offsets["pick_and_place_offsets_separated"] is not True:
        raise BufferedTrajectoryContractError(
            "pick and place TCP offsets must stay separated"
        )
    if offsets["pick_offset_measured"] is not True:
        raise BufferedTrajectoryContractError(
            "pick TCP offset must record its measurement"
        )
    if offsets["place_offset_measured"] is not False:
        raise BufferedTrajectoryContractError(
            "place TCP offset is not measured yet; it must say so"
        )
    if offsets["motion_authorized"] is not False:
        raise BufferedTrajectoryContractError(
            "TCP offset block must keep motion_authorized=false"
        )
    if not (
        offsets["control_close_residual_raw"]
        < offsets["contact_threshold_raw"]
        <= min(
            entry["residual_gap_raw"]
            for entry in offsets["sweep"]
            if entry["held"] is True
        )
    ):
        raise BufferedTrajectoryContractError(
            "contact threshold must separate the control gap from every "
            "measured grasp"
        )

    # C2. 수렴이 적용된 뒤 파지 offset 을 다시 쟀다. 종전 값은 처짐을
    # 흡수하도록 맞춰져 있었으므로, 처짐을 줄이면 반드시 다시 재야 한다.
    if offsets["measured_under_convergence"] is not True:
        raise BufferedTrajectoryContractError(
            "the pick offset must record that it was measured with the "
            "convergence layer active"
        )
    if not (
        offsets["pick_grasp_offset_m"]
        < offsets["superseded_pick_grasp_offset_m"]
    ):
        raise BufferedTrajectoryContractError(
            "removing sag can only require a shallower offset, never a deeper "
            "one; the recorded values disagree"
        )
    converged = offsets["converged_sweep"]
    held = [entry for entry in converged if entry["held"] is True]
    missed = [entry for entry in converged if entry["held"] is False]
    if not held or not missed:
        raise BufferedTrajectoryContractError(
            "the converged sweep must bracket the grasp with both a miss and "
            "a hold"
        )
    if offsets["pick_grasp_offset_m"] != min(
        entry["grasp_offset_m"] for entry in held
    ):
        raise BufferedTrajectoryContractError(
            "the deployed pick offset must be the shallowest offset that was "
            "measured to hold"
        )
    if not all(
        entry["residual_gap_raw"] >= offsets["contact_threshold_raw"]
        for entry in held
    ):
        raise BufferedTrajectoryContractError(
            "every held entry must clear the contact threshold"
        )
    if not all(
        entry["residual_gap_raw"] < offsets["contact_threshold_raw"]
        for entry in missed
    ):
        raise BufferedTrajectoryContractError(
            "every missed entry must sit below the contact threshold"
        )

    # A4.5. `ShadowObjectTarget` 은 "Never consume this as a motion goal" 로
    # 시작하고 발행자는 언제나 robot_target_available=false 를 낸다. 그 잠금을
    # 넘어 파지 좌표로 쓰려면 대신할 게이트가 있어야 하고, 그 게이트가 무엇을
    # 거부하는지가 곧 인식 기반 파지의 안전 논리다.
    shadow = _require_object(document, "top_shadow_grasp_candidate")
    if shadow["publisher_must_not_claim_authority"] is not True:
        raise BufferedTrajectoryContractError(
            "the perception publisher must never claim motion authority"
        )
    if shadow["motion_authorized"] is not False:
        raise BufferedTrajectoryContractError(
            "top shadow grasp route must keep motion_authorized=false"
        )
    if shadow["operator_approves_each_descent"] is not True:
        raise BufferedTrajectoryContractError(
            "each perception-driven descent needs separate approval"
        )
    if shadow["collision_checked_per_run"] is not True:
        raise BufferedTrajectoryContractError(
            "a perception-derived pose must be collision checked in the loop"
        )
    # 표본 흔들림 한계가 인식 정확도만큼 크면 게이트가 아무것도 걸러내지 못한다.
    if not (
        shadow["maximum_position_spread_m"]
        < shadow["measured_perception_error_position_m"]
        and shadow["maximum_yaw_spread_rad"]
        < shadow["measured_perception_error_yaw_rad"]
    ):
        raise BufferedTrajectoryContractError(
            "shadow spread limits must stay inside the measured perception "
            "error"
        )

    # C1. 도달 수렴 계층. 여기서 지켜지는 것은 **허용치의 소유자가 둘로
    # 갈라져 있다** 는 사실이다. 안전 허용치 30 raw 는 "동작이 잘못되지
    # 않았다" 에 답하고 Action 이 소유한다. 과제 허용치는 "이 파지는 성공할
    # 것이다" 에 답하고 과제 계층이 소유한다. 하나가 둘 다 답하면 Action 이
    # 안전하지만 정밀하지 않은 동작을 실패로 만들거나, 반대로 과제가 헐거운
    # 기준을 물려받는다.
    convergence = _require_object(document, "grasp_convergence")
    if convergence["tolerances_separated"] is not True:
        raise BufferedTrajectoryContractError(
            "the safety and task tolerances must stay separated"
        )
    if convergence["safety_tolerance_raw"] != POST_SETTLE_SAFETY_TOLERANCE_RAW:
        raise BufferedTrajectoryContractError(
            "the recorded safety tolerance must match the Action's constant"
        )
    if convergence["safety_tolerance_unchanged"] is not True:
        raise BufferedTrajectoryContractError(
            "the Action's safety tolerance must not be tightened by the "
            "task layer"
        )
    if not (
        0.0
        < convergence["task_tolerance_m"]
        < convergence["maximum_correction_m"]
    ):
        raise BufferedTrajectoryContractError(
            "the task tolerance must be positive and tighter than the "
            "bounded correction limit"
        )
    if convergence["motion_authorized"] is not False:
        raise BufferedTrajectoryContractError(
            "convergence block must keep motion_authorized=false"
        )

    # 예산이 실제로 닫히는지 계약 안에서 산술로 확인한다. A4 스윕이 위로
    # 8 mm 를 확실한 실패점으로 남겼다.
    if (
        convergence["task_tolerance_m"]
        + convergence["plan_residual_measured_m"]["grasp"]
        >= convergence["known_grasp_failure_offset_m"]
    ):
        raise BufferedTrajectoryContractError(
            "the plan residual and task tolerance together reach the measured "
            "grasp failure point"
        )
    if convergence["budget_closes"] is not True:
        raise BufferedTrajectoryContractError(
            "the convergence block must state that the budget closes"
        )

    # 계획 목표는 점이 아니라 상자다. 조인 값이 실제로 잔차를 줄였는지,
    # 그리고 그 잔차가 artifact 에 남는지를 계약이 요구한다.
    if not (
        convergence["plan_position_tolerance_m"]
        < convergence["previous_plan_position_tolerance_m"]
    ):
        raise BufferedTrajectoryContractError(
            "the plan position tolerance must be tighter than the value it "
            "replaced"
        )
    for pose in ("pregrasp", "grasp"):
        if not (
            convergence["plan_residual_measured_m"][pose]
            < convergence["previous_plan_residual_measured_m"][pose]
        ):
            raise BufferedTrajectoryContractError(
                f"the measured {pose} plan residual did not improve"
            )
    if convergence["plan_residual_recorded_in_artifact"] is not True:
        raise BufferedTrajectoryContractError(
            "a plan artifact must record the residual it was allowed to have"
        )

    # 넘겨명령은 그 회차의 실측 잔차만큼이다. 상한이 관측된 최대 잔차보다
    # 작으면 바로 그 상황에서 거부되어 아무 일도 하지 못한다.
    if not (
        convergence["largest_measured_residual_m"]
        < convergence["maximum_overshoot_m"]
        <= convergence["maximum_correction_m"]
    ):
        raise BufferedTrajectoryContractError(
            "the overshoot cap must cover the largest measured residual and "
            "stay inside the bounded correction limit"
        )
    # C2 실측: 같은 목표를 다시 보내는 것은 문턱 아래 명령이 되어 서보를
    # 움직이지 못한다. 넘겨명령만이 문턱을 넘는 명령을 만든다.
    if not (
        convergence["minimum_observable_command_raw"]
        <= convergence["ineffective_correction_raw"]
    ):
        raise BufferedTrajectoryContractError(
            "the ineffective-correction threshold must not sit below the "
            "measured minimum observable command"
        )
    for field in (
        "overshoot_is_the_only_supra_threshold_command",
        "overshoot_clamped_at_joint_limits",
        "clamped_joints_reported",
    ):
        if convergence[field] is not True:
            raise BufferedTrajectoryContractError(
                f"convergence must record what C2 measured: {field}"
            )
    # 와이어는 위치를 µrad 정수로 싣는다. 한계에 정확히 걸치는 명령은
    # 올림되어 거부되므로 애초에 만들지 않는다.
    if convergence["never_command_exactly_on_a_limit"] is not True:
        raise BufferedTrajectoryContractError(
            "a command must never sit exactly on a joint limit"
        )
    if not (
        convergence["joint_limit_margin_rad"]
        > convergence["wire_position_quantisation_rad"]
        > convergence["bridge_joint_limit_epsilon_rad"]
    ):
        raise BufferedTrajectoryContractError(
            "the joint-limit margin must exceed the wire quantisation, which "
            "in turn exceeds the bridge validation epsilon"
        )
    if convergence["overshoot_used_at_most_once"] is not True:
        raise BufferedTrajectoryContractError(
            "the measured overshoot must be bounded to a single use"
        )
    if convergence["error_measured_against_nominal_goal"] is not True:
        raise BufferedTrajectoryContractError(
            "the residual must be measured against the original goal, not "
            "against the corrected command"
        )
    if convergence["fail_closed_on_non_convergence"] is not True:
        raise BufferedTrajectoryContractError(
            "non-convergence must stop and report, not retry silently"
        )

    # 층을 섞지 않는다. bridge 는 계획하지 않는다.
    if convergence["bridge_action_unchanged"] is not True:
        raise BufferedTrajectoryContractError(
            "the physically validated buffered Action must not be changed by "
            "the convergence layer"
        )
    if convergence["convergence_lives_in_host_library"] is not True:
        raise BufferedTrajectoryContractError(
            "convergence is planning and must live outside the bridge"
        )

    # 양팔로 확장될 모양인지. 나중에 끼워 넣기 어려운 것들이다.
    for field in (
        "per_arm_policy",
        "arm_name_required",
        "send_and_evaluate_separated",
        "coordinated_stop_input",
        "per_joint_post_settle_recorded",
    ):
        if convergence[field] is not True:
            raise BufferedTrajectoryContractError(
                f"convergence must be bimanual-shaped now: {field}"
            )

    host_adapter = _require_object(document, "host_adapter_candidate")
    if host_adapter != {
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
    }:
        raise BufferedTrajectoryContractError(
            "host adapter must remain action-disconnected and fail-closed"
        )

    physical = _require_object(document, "physical_execution_candidate")
    if physical != {
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
    }:
        raise BufferedTrajectoryContractError(
            "physical execution must remain commissioned and motion-disabled"
        )

    evidence = _require_object(document, "motion9_physical_evidence")
    if evidence != {
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
    }:
        raise BufferedTrajectoryContractError(
            "Motion-9 physical evidence must match the commissioned run"
        )

    timing = _require_object(document, "timing_analysis")
    if timing != {
        "minimum_hardware_samples_per_series": 1000,
        "required_provenance": "pi_vcp_hardware",
        "required_clock_source": "monotonic_raw",
        "requires_buffered_capability": True,
        "synthetic_measurements_can_authorize_values": False,
        "operational_values_authorized": True,
        "motion_authorized": False,
        "policy_path": (
            "artifacts/motion/2026-08-03/"
            "buffered_timing_policy_reviewed.json"
        ),
        "policy_sha256": (
            "362e09c91e696ca587963c664f26cc49e06a44d205e2b5d6daf186c63e1fd8f2"
        ),
        "sample_period_ms": 20,
        "minimum_lead_ms": 60,
        "maximum_lead_ms": 400,
        "startup_prime_depth_samples": 16,
        "low_watermark_samples": 10,
        "refill_target_samples": 16,
        "serial_round_trip_p95_ms": 17.428593,
        "serial_round_trip_p99_ms": 17.533277,
        "host_command_jitter_p95_ms": 0.062925,
        "delivery_lateness_p95_ms": 0.0,
        "observed_max_host_outage_ms": 80.064074,
        "rejected_first_lead_ms": 40,
        "rejected_status_code": 1,
        "rejected_detail": 9,
    }:
        raise BufferedTrajectoryContractError(
            "timing values must match the reviewed no-motion hardware policy"
        )

    wire = _require_object(document, "existing_wire_limits")
    required_wire = {
        "protocol_version": 1,
        "total_joint_count": TOTAL_JOINT_COUNT,
        "arm_joint_count": ARM_JOINT_COUNT,
        "maximum_samples_per_batch": WIRE_BATCH_MAX_SAMPLES,
        "queue_capacity_samples": MCU_QUEUE_CAPACITY_SAMPLES,
        "tick_unit": "uint32_milliseconds",
    }
    if wire != required_wire:
        raise BufferedTrajectoryContractError(
            "existing wire limits do not match the implemented protocol"
        )

    trajectory = _require_object(document, "trajectory")
    required_trajectory = {
        "minimum_points": 2,
        "timestamps": (
            "strictly_increasing_nanoseconds_rounded_up_to_20ms_sample"
        ),
        "requires_fresh_start_feedback": True,
        "zero_time_point_must_match_fresh_start": True,
        "interpolation": "linear_position_between_validated_points",
        "dynamic_fields": (
            "validate_velocity_acceleration_limits_reject_effort"
        ),
        "velocity_limit_source": "so101_moveit_config/config/joint_limits.yaml",
        "acceleration_limit_source": "so101_moveit_config/config/joint_limits.yaml",
    }
    if trajectory != required_trajectory:
        raise BufferedTrajectoryContractError("trajectory contract was weakened")

    if _require_object(document, "safety") != REQUIRED_SAFETY:
        raise BufferedTrajectoryContractError("safety contract was weakened")
    deployment = _require_object(document, "deployment_gate")
    if deployment.get("required_measurements") != REQUIRED_MEASUREMENTS:
        raise BufferedTrajectoryContractError(
            "deployment gate measurements are incomplete"
        )
    if deployment.get("firmware_change_requires_separate_issue") is not True:
        raise BufferedTrajectoryContractError(
            "firmware change must require a separate issue"
        )
    if deployment.get("physical_motion_requires_explicit_approval") is not True:
        raise BufferedTrajectoryContractError(
            "physical motion must require explicit approval"
        )


def load_buffered_trajectory_contract(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise BufferedTrajectoryContractError("contract root must be an object")
    validate_buffered_trajectory_contract(document)
    return document


class BufferedSetpointQueueModel:
    """Deterministic mock of queue admission and fail-closed transitions."""

    def __init__(
        self,
        *,
        joint_count: int,
        capacity_samples: int,
        maximum_batch_samples: int,
        minimum_start_samples: int,
        minimum_lead_ms: int,
        maximum_lead_ms: int,
    ) -> None:
        integer_values = (
            joint_count,
            capacity_samples,
            maximum_batch_samples,
            minimum_start_samples,
            minimum_lead_ms,
            maximum_lead_ms,
        )
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value <= 0
            for value in integer_values
        ):
            raise ValueError("queue model limits must be positive integers")
        if maximum_batch_samples > capacity_samples:
            raise ValueError("maximum batch cannot exceed queue capacity")
        if minimum_start_samples > capacity_samples:
            raise ValueError("minimum start depth cannot exceed queue capacity")
        if minimum_lead_ms > maximum_lead_ms:
            raise ValueError("minimum lead cannot exceed maximum lead")
        if maximum_lead_ms > MAX_UNAMBIGUOUS_TICK_LEAD:
            raise ValueError("maximum lead is ambiguous in uint32 tick space")
        self._joint_count = joint_count
        self._capacity = capacity_samples
        self._maximum_batch = maximum_batch_samples
        self._minimum_start = minimum_start_samples
        self._minimum_lead = minimum_lead_ms
        self._maximum_lead = maximum_lead_ms
        self._queue: list[ScheduledSetpoint] = []
        self._state = BufferedQueueState.PRIMING
        self._input_complete = False
        self._last_applied_tick_ms: int | None = None
        self._safe_stop_required = False
        self._reason: str | None = None

    @property
    def state(self) -> BufferedQueueState:
        return self._state

    def snapshot(self) -> QueueSnapshot:
        return QueueSnapshot(
            state=self._state,
            queued_samples=len(self._queue),
            input_complete=self._input_complete,
            last_applied_tick_ms=self._last_applied_tick_ms,
            safe_stop_required=self._safe_stop_required,
            reason=self._reason,
        )

    def push_batch(
        self,
        samples: Sequence[ScheduledSetpoint],
        *,
        current_tick_ms: int,
    ) -> None:
        if self._state not in (
            BufferedQueueState.PRIMING,
            BufferedQueueState.RUNNING,
        ):
            raise BufferedQueueError("terminal or HOLD queue cannot accept samples")
        if self._input_complete:
            raise BufferedQueueError("completed input cannot accept more samples")
        if not _is_uint32(current_tick_ms):
            raise BufferedQueueError("current tick must be uint32 milliseconds")
        batch = tuple(samples)
        if not batch or len(batch) > self._maximum_batch:
            raise BufferedQueueError("batch size is outside the queue contract")
        if len(self._queue) + len(batch) > self._capacity:
            raise BufferedQueueError("batch would exceed queue capacity")

        previous_tick = (
            self._queue[-1].apply_tick_ms if self._queue else current_tick_ms
        )
        for sample in batch:
            tick = sample.apply_tick_ms
            if not _is_uint32(tick):
                raise BufferedQueueError("apply tick must be uint32 milliseconds")
            lead = _tick_delta(tick, current_tick_ms)
            if not _tick_is_after(tick, current_tick_ms) or lead < self._minimum_lead:
                raise BufferedQueueError("setpoint lead is stale or too short")
            if lead > self._maximum_lead:
                raise BufferedQueueError("setpoint lead exceeds maximum")
            if not _tick_is_after(tick, previous_tick):
                raise BufferedQueueError("apply ticks must be strictly increasing")
            if not _positions_are_valid(sample.positions_rad, self._joint_count):
                raise BufferedQueueError("setpoint positions are invalid")
            previous_tick = tick

        self._queue.extend(batch)

    def mark_input_complete(self) -> None:
        if self._state not in (
            BufferedQueueState.PRIMING,
            BufferedQueueState.RUNNING,
        ):
            raise BufferedQueueError("terminal or HOLD queue cannot complete input")
        self._input_complete = True

    def start(self) -> None:
        if self._state is not BufferedQueueState.PRIMING:
            raise BufferedQueueError("queue can only start once from PRIMING")
        if not self._queue:
            raise BufferedQueueError("queue cannot start empty")
        if len(self._queue) < self._minimum_start and not self._input_complete:
            raise BufferedQueueError("queue is not sufficiently primed")
        self._state = BufferedQueueState.RUNNING

    def take_due(self, current_tick_ms: int) -> ScheduledSetpoint | None:
        if self._state is not BufferedQueueState.RUNNING:
            raise BufferedQueueError("queue is not running")
        if not _is_uint32(current_tick_ms):
            raise BufferedQueueError("current tick must be uint32 milliseconds")
        if not self._queue:
            if self._input_complete:
                self._state = BufferedQueueState.SUCCEEDED
                return None
            self._fail_safe_hold("queue_underflow")
            return None

        sample = self._queue[0]
        if current_tick_ms != sample.apply_tick_ms:
            if _tick_is_after(sample.apply_tick_ms, current_tick_ms):
                return None
            self._fail_safe_hold("missed_apply_tick")
            return None

        self._queue.pop(0)
        self._last_applied_tick_ms = sample.apply_tick_ms
        if not self._queue and self._input_complete:
            self._state = BufferedQueueState.SUCCEEDED
        return sample

    def planned_hold(self) -> None:
        if self._state not in (
            BufferedQueueState.PRIMING,
            BufferedQueueState.RUNNING,
        ):
            raise BufferedQueueError("queue cannot enter HOLD from terminal state")
        self._queue.clear()
        self._state = BufferedQueueState.HOLD
        self._safe_stop_required = False
        self._reason = "planned_hold"

    def cancel(self) -> None:
        if self._state not in (
            BufferedQueueState.PRIMING,
            BufferedQueueState.RUNNING,
        ):
            raise BufferedQueueError("queue cannot cancel from terminal state")
        self._queue.clear()
        self._state = BufferedQueueState.CANCELED
        self._safe_stop_required = True
        self._reason = "operator_cancel"

    def connection_loss(self) -> None:
        if self._state not in (
            BufferedQueueState.PRIMING,
            BufferedQueueState.RUNNING,
        ):
            raise BufferedQueueError("queue cannot abort from terminal state")
        self._queue.clear()
        self._state = BufferedQueueState.ABORTED
        self._safe_stop_required = True
        self._reason = "connection_loss"

    def _fail_safe_hold(self, reason: str) -> None:
        self._queue.clear()
        self._state = BufferedQueueState.HOLD
        self._safe_stop_required = True
        self._reason = reason
