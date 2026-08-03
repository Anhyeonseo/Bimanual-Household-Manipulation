"""Host-only buffered trajectory queue model; never commands hardware."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any, Sequence


CONTRACT_KIND = "single_arm_buffered_trajectory"
CONTRACT_STATUS = "BOARD_VALIDATION_ONLY"
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
        "firmware_supports_buffered_execution": False,
        "action_adapter_supports_buffered_execution": False,
        "maximum_accepted_sample_count": 1,
    }:
        raise BufferedTrajectoryContractError(
            "current runtime gate must remain single-sample and disabled"
        )

    candidate = _require_object(document, "firmware_candidate")
    if candidate != {
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
    }:
        raise BufferedTrajectoryContractError(
            "firmware route must remain validation-only until timing gates pass"
        )

    host_adapter = _require_object(document, "host_adapter_candidate")
    if host_adapter != {
        "multi_point_validation_reused": True,
        "linear_resampling_period_ms": 20,
        "initial_first_sample_lead_ms": 100,
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
        "ros_action_server_connected": False,
        "transport_execution_connected": False,
        "motion_authorized": False,
    }:
        raise BufferedTrajectoryContractError(
            "host adapter must remain mock-only and fail-closed"
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
        "timestamps": "strictly_increasing_integer_milliseconds",
        "requires_fresh_start_feedback": True,
        "zero_time_point_must_match_fresh_start": True,
        "interpolation": "linear_position_between_validated_points",
        "dynamic_fields": "reject_nonempty",
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
