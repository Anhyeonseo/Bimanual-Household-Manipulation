"""Host-only buffered trajectory queue model; never commands hardware."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any, Sequence


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
        "status": "LOCAL_APPLY_LATENESS_PROFILE_CANDIDATE",
        "firmware_version": "0x00022600",
        "previous_candidate_firmware_version": "0x00022500",
        "previous_deployed_firmware_version": "0x00022500",
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
        "buffered_status_payload_bytes": 60,
        "deployed": False,
        "motion_authorized": False,
    }:
        raise BufferedTrajectoryContractError(
            "apply lateness profile candidate must remain bounded and undeployed"
        )

    host_adapter = _require_object(document, "host_adapter_candidate")
    if host_adapter != {
        "multi_point_validation_reused": True,
        "linear_resampling_period_ms": 20,
        "initial_first_sample_lead_ms": 160,
        "physical_uart_baud": 115200,
        "startup_prime_wire_lower_bound_ms": 87.674,
        "startup_anchor_wire_margin_ms": 52.326,
        "startup_prime_elapsed_window_ms": [60, 80],
        "startup_prime_maximum_heartbeat_gates": 3,
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
            "d5378b6c0eb5eb4069e79e609ee12efb14750d228b61b009d29555fb573f47f8"
        ),
        "sender_sha256": (
            "d66f26f7b3907fda1988895a01e657bafa902ea901396a2c38f8524f16e93671"
        ),
        "execution_log_sha256": (
            "80f14845bab532de3217fcee7a9c4c2b0b5cf4241b65023844d6ba7d615de087"
        ),
        "firmware_version": "0x00022100",
        "calibration_hash": "0x8AD27897",
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
