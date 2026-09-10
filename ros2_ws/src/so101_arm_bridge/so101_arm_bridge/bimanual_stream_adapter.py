"""Source-agnostic resident adapter for the protocol-v2 12-axis stream.

This module deliberately has no ROS dependency.  A MoveIt action server, an
FSM, or a learned-policy process can all translate their output into the same
``TimedJointPoint`` contract and use one serialized transport owner.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol, Sequence

from .stream_protocol_v2 import (
    ARM_MASK_BOTH,
    JOINT_COUNT,
    MAX_SAMPLES,
    StreamBatchV2,
    StreamContractResultV2,
    StreamExecutorStateV2,
    StreamPolicyV2,
    StreamSampleV2,
    StreamStatusCodeV2,
)
from .stream_transport_v2 import StreamResponseTimeoutError


CANONICAL_JOINT_NAMES = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
    "left_gripper_joint",
    "right_base_joint",
    "right_shoulder_joint",
    "right_elbow_joint",
    "right_wrist_flex_joint",
    "right_wrist_roll_joint",
    "right_gripper_joint",
)
F8_FIRMWARE_VERSION = 0x00024809
F8_CAPABILITIES = 0xEFFFFFFF
CALIBRATION_HASH = 0x2D90167E
PROTOCOL_VERSION = 2
CONTROL_TICK_MS = 5
DEFAULT_MINIMUM_LEAD_MS = 20
DEFAULT_MAXIMUM_LEAD_MS = 400
DEFAULT_OPEN_COMMAND_TIMEOUT_MS = 100
DEFAULT_FINITE_COMMAND_TIMEOUT_MS = 500
DEFAULT_MAXIMUM_APPLY_LATENESS_MS = 5
DEFAULT_TRACKING_ERROR_LIMIT_URAD = 90_000
DEFAULT_GRIPPER_TRACKING_ERROR_LIMIT_URAD = 150_000
DEFAULT_MAXIMUM_STEP_URAD_PER_TICK = 9_000
DEFAULT_TERMINAL_SETTLE_TOLERANCE_URAD = 46_020
DEFAULT_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD = 150_000
DEFAULT_TERMINAL_FEEDBACK_MAX_AGE_MS = 150
DEFAULT_HEARTBEAT_PERIOD_S = 0.1
DEFAULT_FEEDBACK_TRANSPORT_FAILURE_LIMIT = 3
EXECUTOR_QUEUE_CAPACITY = 16


class BimanualStreamError(RuntimeError):
    """Base error for host-side stream refusal or transport failure."""


class BimanualStreamContractError(BimanualStreamError, ValueError):
    """Raised before any motion when an upper-layer command is invalid."""


class BimanualStreamOwnershipError(BimanualStreamError):
    """Raised when a second command source attempts to take the session."""


class BimanualTransientFeedbackError(BimanualStreamError):
    """A bounded feedback transport delay that has not faulted the session."""


class AdapterState(Enum):
    NEW = "new"
    READY = "ready"
    ACTIVE = "active"
    STOPPED = "stopped"
    FAULTED = "faulted"


@dataclass(frozen=True, slots=True)
class TimedJointPoint:
    """One absolute 12-axis target at a time relative to command admission."""

    offset_ms: int
    positions_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class OperationalLimits:
    joint_names: tuple[str, ...]
    minimum_urad: tuple[int, ...]
    maximum_urad: tuple[int, ...]
    source_sha256: str | None = None


@dataclass(frozen=True, slots=True)
class PreparedBimanualState:
    positions_raw: tuple[int, ...]
    unwrapped_positions_raw: tuple[int, ...]
    positions_urad: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class ResidentStreamConfig:
    minimum_start_samples: int = 2
    minimum_lead_ms: int = DEFAULT_MINIMUM_LEAD_MS
    maximum_lead_ms: int = DEFAULT_MAXIMUM_LEAD_MS
    open_command_timeout_ms: int = DEFAULT_OPEN_COMMAND_TIMEOUT_MS
    finite_command_timeout_ms: int = DEFAULT_FINITE_COMMAND_TIMEOUT_MS
    maximum_apply_lateness_ms: int = DEFAULT_MAXIMUM_APPLY_LATENESS_MS
    tracking_error_limit_urad: int = DEFAULT_TRACKING_ERROR_LIMIT_URAD
    gripper_tracking_error_limit_urad: int = (
        DEFAULT_GRIPPER_TRACKING_ERROR_LIMIT_URAD
    )
    maximum_step_urad_per_tick: int = DEFAULT_MAXIMUM_STEP_URAD_PER_TICK
    terminal_settle_tolerance_urad: int = (
        DEFAULT_TERMINAL_SETTLE_TOLERANCE_URAD
    )
    terminal_settle_gripper_tolerance_urad: int = (
        DEFAULT_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD
    )
    terminal_feedback_max_age_ms: int = DEFAULT_TERMINAL_FEEDBACK_MAX_AGE_MS


class StreamTransportV2(Protocol):
    def enter_binary_mode(self): ...
    def heartbeat(self): ...
    def get_state(self): ...
    def prepare_shadow(self): ...
    def get_dispatch_diagnostics(self): ...
    def get_tracking_diagnostics(self): ...
    def get_feedback_snapshot(self): ...
    def get_executor_diagnostics(self): ...
    def arm(self, calibration_hash: int) -> None: ...
    def enable(self): ...
    def open_stream(self, policy: StreamPolicyV2): ...
    def append(self, batch: StreamBatchV2): ...
    def splice(self, batch: StreamBatchV2): ...
    def safe_stop(self): ...


def _flatten_limit_entries(document: dict) -> tuple[dict, ...]:
    entries = []
    for arm in ("left", "right"):
        arm_limits = document["arms"][arm]
        for joint in (
            "base",
            "shoulder",
            "elbow",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ):
            entries.append(arm_limits[joint])
    return tuple(entries)


def load_operational_limits(path: Path) -> OperationalLimits:
    """Load only the operator-approved full task envelope."""
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        document.get("record_kind") != "bimanual_operational_limits"
        or document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True
    ):
        raise BimanualStreamContractError(
            "operational-limit document is not operator-authorized"
        )
    joint_names = tuple(document.get("joint_order", ()))
    if joint_names != CANONICAL_JOINT_NAMES:
        raise BimanualStreamContractError(
            "operational-limit joint order does not match the 12-axis contract"
        )
    try:
        entries = _flatten_limit_entries(document)
        minimum = tuple(int(entry["minimum_urad"]) for entry in entries)
        maximum = tuple(int(entry["maximum_urad"]) for entry in entries)
    except (KeyError, TypeError, ValueError) as error:
        raise BimanualStreamContractError(
            "operational-limit document is incomplete"
        ) from error
    if len(minimum) != JOINT_COUNT or any(
        lower >= upper for lower, upper in zip(minimum, maximum, strict=True)
    ):
        raise BimanualStreamContractError("operational limits are invalid")
    source = document.get("source", {})
    return OperationalLimits(
        joint_names=joint_names,
        minimum_urad=minimum,
        maximum_urad=maximum,
        source_sha256=source.get("sha256") if isinstance(source, dict) else None,
    )


def normalize_joint_positions(
    joint_names: Sequence[str],
    positions_rad: Sequence[float],
) -> tuple[float, ...]:
    """Reorder an exact 12-axis vector into the firmware's canonical order."""
    names = tuple(joint_names)
    positions = tuple(positions_rad)
    if len(names) != JOINT_COUNT or len(positions) != JOINT_COUNT:
        raise BimanualStreamContractError(
            "joint names and positions must each contain exactly 12 values"
        )
    if len(set(names)) != JOINT_COUNT or set(names) != set(CANONICAL_JOINT_NAMES):
        raise BimanualStreamContractError(
            "joint names must contain each canonical bimanual joint exactly once"
        )
    by_name = dict(zip(names, positions, strict=True))
    return tuple(by_name[name] for name in CANONICAL_JOINT_NAMES)


def _to_urad(positions_rad: Sequence[float]) -> tuple[int, ...]:
    if len(positions_rad) != JOINT_COUNT:
        raise BimanualStreamContractError("each point must contain 12 positions")
    converted = []
    for value in positions_rad:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise BimanualStreamContractError("joint positions must be numeric")
        if not math.isfinite(value):
            raise BimanualStreamContractError("joint positions must be finite")
        urad = round(float(value) * 1_000_000.0)
        if not -(1 << 31) <= urad < (1 << 31):
            raise BimanualStreamContractError("joint position exceeds int32 urad")
        converted.append(urad)
    return tuple(converted)


def _tick(base: int, offset: int) -> int:
    return (base + offset) & 0xFFFFFFFF


def _tick_after(candidate: int, reference: int) -> bool:
    return 0 < ((candidate - reference) & 0xFFFFFFFF) < 0x80000000


class ResidentBimanualStreamAdapter:
    """Single-owner, fail-closed session around the synchronous v2 transport."""

    def __init__(
        self,
        transport: StreamTransportV2,
        limits: OperationalLimits,
        *,
        motion_authorized: bool = False,
        config: ResidentStreamConfig = ResidentStreamConfig(),
    ) -> None:
        self._transport = transport
        self._limits = limits
        self._motion_authorized = motion_authorized
        self._config = config
        self._state = AdapterState.NEW
        self._owner: str | None = None
        self._epoch = 0
        self._prepared: PreparedBimanualState | None = None
        self._tail_tick: int | None = None
        self._tail_positions_urad: tuple[int, ...] | None = None
        self._route_samples: tuple[StreamSampleV2, ...] = ()
        self._finite_horizon_tick: int | None = None
        self._pending_finite_samples: tuple[StreamSampleV2, ...] = ()
        self._armed = False
        self._last_heartbeat_monotonic: float | None = None
        self._fault_diagnostic: str | None = None
        self._feedback_transport_failure_streak = 0

    @property
    def state(self) -> AdapterState:
        return self._state

    @property
    def owner(self) -> str | None:
        return self._owner

    @property
    def epoch(self) -> int:
        return self._epoch

    @property
    def prepared_state(self) -> PreparedBimanualState | None:
        """Return the latest measured resident anchor without touching hardware."""
        return self._prepared

    @property
    def heartbeat_required(self) -> bool:
        """Return whether enabled torque currently depends on host keepalive."""
        return self._armed and self._state in (
            AdapterState.READY,
            AdapterState.ACTIVE,
        )

    @property
    def fault_diagnostic(self) -> str | None:
        return self._fault_diagnostic

    def _collect_post_stop_diagnostics(self) -> str:
        snapshots = []
        for label, read in (
            ("executor", self._transport.get_executor_diagnostics),
            ("dispatch", self._transport.get_dispatch_diagnostics),
            ("tracking", self._transport.get_tracking_diagnostics),
            ("feedback", self._transport.get_feedback_snapshot),
            ("state", self._transport.get_state),
        ):
            try:
                snapshots.append(f"{label}={read()!r}")
            except Exception as diagnostic_error:
                snapshots.append(
                    f"{label}_error={type(diagnostic_error).__name__}: "
                    f"{diagnostic_error}"
                )
        return "; ".join(snapshots)

    def _fault(self, error: Exception) -> None:
        self._state = AdapterState.FAULTED
        message = str(error)
        if self._armed:
            stop_error = None
            try:
                self._transport.safe_stop()
            except Exception as caught_stop_error:
                stop_error = caught_stop_error
            self._armed = False
            diagnostics = self._collect_post_stop_diagnostics()
            if stop_error is not None:
                message += (
                    f"; safe_stop_error={type(stop_error).__name__}: "
                    f"{stop_error}"
                )
            message += f"; post_stop_diagnostics: {diagnostics}"
        self._fault_diagnostic = message
        raise BimanualStreamError(message) from error

    def _advance_epoch(self) -> None:
        self._epoch = (self._epoch + 1) & 0xFFFFFFFF
        if self._epoch == 0:
            self._epoch = 1

    def _claim(self, owner: str) -> None:
        normalized = owner.strip()
        if not normalized:
            raise BimanualStreamOwnershipError("stream owner must not be empty")
        if self._owner is None:
            self._owner = normalized
            self._advance_epoch()
        elif self._owner != normalized:
            raise BimanualStreamOwnershipError(
                f"stream is owned by {self._owner!r}, not {normalized!r}"
            )

    def _prepared_from_snapshot(self, snapshot) -> PreparedBimanualState:
        shadow_failure_reasons = {
            1: "stream executor is active",
            2: "left-arm torque-disable command failed",
            3: "right-arm verified torque-disable failed",
            4: "left-arm position capture failed",
            5: "right-arm position capture failed",
            6: "raw-to-angle calibration conversion failed",
            7: "invalid shadow preparation request",
            8: "joint unwrap or operational-limit binding failed",
        }
        if snapshot.status_code != 0:
            reason = shadow_failure_reasons.get(
                snapshot.status_code,
                "unknown firmware shadow failure",
            )
            raise BimanualStreamError(
                "12-axis anchor failed: "
                f"status={snapshot.status_code} reason={reason}; "
                "firmware stop is latched and STM32 reset is required"
            )
        if (
            snapshot.joint_count != JOINT_COUNT
            or snapshot.left_present_mask != 0x3F
            or snapshot.right_present_mask != 0x3F
            or len(snapshot.positions_raw) != JOINT_COUNT
            or len(snapshot.unwrapped_positions_raw) != JOINT_COUNT
            or len(snapshot.anchor_positions_urad) != JOINT_COUNT
        ):
            raise BimanualStreamError(f"12-axis anchor failed: {snapshot}")
        for index, position in enumerate(snapshot.anchor_positions_urad):
            if not (
                self._limits.minimum_urad[index]
                <= position
                <= self._limits.maximum_urad[index]
            ):
                raise BimanualStreamError(
                    f"anchor outside operational limits: "
                    f"joint={CANONICAL_JOINT_NAMES[index]} value={position}"
                )
        return PreparedBimanualState(
            positions_raw=tuple(snapshot.positions_raw),
            unwrapped_positions_raw=tuple(snapshot.unwrapped_positions_raw),
            positions_urad=tuple(snapshot.anchor_positions_urad),
        )

    def prepare(self) -> PreparedBimanualState:
        """Verify exact F8 identity and capture both arms without enabling torque."""
        if self._state is not AdapterState.NEW:
            raise BimanualStreamError("adapter preparation is single-use")
        try:
            hello = self._transport.enter_binary_mode()
            if (
                hello.protocol_version != PROTOCOL_VERSION
                or hello.joint_count != JOINT_COUNT
                or hello.firmware_version != F8_FIRMWARE_VERSION
                or hello.left_calibration_hash != CALIBRATION_HASH
                or hello.right_calibration_hash != CALIBRATION_HASH
                or hello.capabilities != F8_CAPABILITIES
                or hello.stop_latched
            ):
                raise BimanualStreamError(f"unexpected F8 identity: {hello}")
            state = self._transport.heartbeat()
            dispatch = self._transport.get_dispatch_diagnostics()
            tracking = self._transport.get_tracking_diagnostics()
            snapshot = self._transport.prepare_shadow()
            if state.stop_latched or state.status_code != 0:
                raise BimanualStreamError(f"unhealthy preflight state: {state}")
            if (
                dispatch.status_code != 0
                or dispatch.active
                or dispatch.faulted
                or not dispatch.ready
            ):
                raise BimanualStreamError(f"dispatch is not ready: {dispatch}")
            if tracking.status_code != 0 or tracking.active or tracking.pending:
                raise BimanualStreamError(f"tracking is not idle: {tracking}")
            self._prepared = self._prepared_from_snapshot(snapshot)
            self._state = AdapterState.READY
            return self._prepared
        except Exception as error:
            self._fault(error)

    def refresh_unarmed_anchor(self) -> PreparedBimanualState:
        """Recapture the torque-off pose immediately before first motion.

        The initial prepare anchor is intentionally not assigned an unlimited
        lifetime: an unpowered arm can sag while a policy or operator waits.
        Refresh is allowed only before this resident session has armed, so it
        cannot introduce a torque gap between finite legs.
        """
        if (
            self._state is not AdapterState.READY
            or self._armed
            or self._owner is not None
        ):
            raise BimanualStreamContractError(
                "unarmed anchor refresh requires an unowned READY session"
            )
        try:
            state = self._transport.heartbeat()
            if state.stop_latched or state.status_code != 0:
                raise BimanualStreamError(
                    f"anchor refresh heartbeat failed: {state}"
                )
            snapshot = self._transport.prepare_shadow()
            self._prepared = self._prepared_from_snapshot(snapshot)
            return self._prepared
        except Exception as error:
            self._fault(error)

    def _healthy_heartbeat(self):
        state = self._transport.heartbeat()
        if state.stop_latched or state.status_code != 0:
            raise BimanualStreamError(f"heartbeat failed: {state}")
        self._last_heartbeat_monotonic = time.monotonic()
        return state

    def keepalive(self, *, force: bool = False):
        """Refresh the MCU watchdog whenever this resident session is armed.

        Finite completion intentionally leaves both arms enabled so the next
        resident leg can start without a torque cycle. READY therefore needs
        the same heartbeat ownership as ACTIVE. The host-time gate prevents
        feedback, poll, and status callbacks from producing a heartbeat storm.
        """
        if not self.heartbeat_required:
            return None
        now = time.monotonic()
        if (
            not force
            and self._last_heartbeat_monotonic is not None
            and now - self._last_heartbeat_monotonic
            < DEFAULT_HEARTBEAT_PERIOD_S
        ):
            return None
        try:
            return self._healthy_heartbeat()
        except Exception as error:
            self._fault(error)

    def _validated_feedback_snapshot(self):
        snapshot = self._transport.get_feedback_snapshot()
        if (
            snapshot.status_code != 0
            or snapshot.joint_count != JOINT_COUNT
            or snapshot.present_mask != 0x0FFF
            or len(snapshot.positions_urad) != JOINT_COUNT
            or len(snapshot.sample_age_ms) != JOINT_COUNT
        ):
            raise BimanualStreamError(
                f"incomplete 12-axis feedback snapshot: {snapshot}"
            )
        return snapshot

    def feedback_snapshot(self):
        """Return one atomic measured 12-axis vector with per-joint age."""
        if self._state in (AdapterState.NEW, AdapterState.FAULTED):
            raise BimanualStreamError("feedback is unavailable")
        try:
            self.keepalive()
            snapshot = self._validated_feedback_snapshot()
            self._feedback_transport_failure_streak = 0
            return snapshot
        except StreamResponseTimeoutError as error:
            self._feedback_transport_failure_streak += 1
            streak = self._feedback_transport_failure_streak
            if streak < DEFAULT_FEEDBACK_TRANSPORT_FAILURE_LIMIT:
                raise BimanualTransientFeedbackError(
                    "transient feedback transport delay "
                    f"({streak}/{DEFAULT_FEEDBACK_TRANSPORT_FAILURE_LIMIT}): "
                    f"{error}"
                ) from error
            self._fault(
                BimanualStreamError(
                    "feedback transport failed consecutively "
                    f"({streak}/{DEFAULT_FEEDBACK_TRANSPORT_FAILURE_LIMIT}): "
                    f"{error}"
                )
            )
        except Exception as error:
            self._fault(error)

    def _validate_points(
        self,
        points: Sequence[TimedJointPoint],
        *,
        first_reference_urad: tuple[int, ...] | None,
        first_span_ms: int,
    ) -> tuple[tuple[int, tuple[int, ...]], ...]:
        if not 1 <= len(points) <= MAX_SAMPLES:
            raise BimanualStreamContractError("batch must contain 1..9 points")
        converted: list[tuple[int, tuple[int, ...]]] = []
        previous_offset: int | None = None
        previous_positions = first_reference_urad
        span_ms = first_span_ms
        for point in points:
            if (
                isinstance(point.offset_ms, bool)
                or not isinstance(point.offset_ms, int)
                or point.offset_ms <= 0
            ):
                raise BimanualStreamContractError(
                    "point offsets must be positive integer milliseconds"
                )
            if point.offset_ms > self._config.maximum_lead_ms:
                raise BimanualStreamContractError(
                    "point exceeds the configured maximum lead"
                )
            if previous_offset is not None:
                span_ms = point.offset_ms - previous_offset
            if span_ms <= 0 or span_ms % CONTROL_TICK_MS != 0:
                raise BimanualStreamContractError(
                    "point intervals must be positive multiples of 5 ms"
                )
            positions = _to_urad(point.positions_rad)
            for index, position in enumerate(positions):
                if not (
                    self._limits.minimum_urad[index]
                    <= position
                    <= self._limits.maximum_urad[index]
                ):
                    raise BimanualStreamContractError(
                        f"joint target outside approved range: "
                        f"joint={CANONICAL_JOINT_NAMES[index]} value={position}"
                    )
                if previous_positions is not None:
                    allowed = (
                        self._config.maximum_step_urad_per_tick
                        * (span_ms // CONTROL_TICK_MS)
                    )
                    if abs(position - previous_positions[index]) > allowed:
                        raise BimanualStreamContractError(
                            f"joint step exceeds host policy: "
                            f"joint={CANONICAL_JOINT_NAMES[index]}"
                        )
            converted.append((point.offset_ms, positions))
            previous_offset = point.offset_ms
            previous_positions = positions
        return tuple(converted)

    def _validate_finite_plan_points(
        self,
        points: Sequence[TimedJointPoint],
    ) -> tuple[tuple[int, tuple[int, ...]], ...]:
        """Validate a complete finite route before arming.

        Wire batches remain limited to nine samples and 400 ms lead. The
        resident adapter feeds a longer, prevalidated finite route in time.
        """
        if len(points) < self._config.minimum_start_samples:
            raise BimanualStreamContractError(
                "finite plan has fewer than minimum_start_samples"
            )
        if points[0].offset_ms < self._config.minimum_lead_ms:
            raise BimanualStreamContractError(
                "finite plan initial point lead is too short"
            )
        converted: list[tuple[int, tuple[int, ...]]] = []
        previous_offset = 0
        previous_positions = (
            None if self._prepared is None else self._prepared.positions_urad
        )
        for point in points:
            if (
                isinstance(point.offset_ms, bool)
                or not isinstance(point.offset_ms, int)
                or point.offset_ms <= previous_offset
                or point.offset_ms % CONTROL_TICK_MS != 0
            ):
                raise BimanualStreamContractError(
                    "finite plan offsets must be increasing positive "
                    "multiples of 5 ms"
                )
            positions = _to_urad(point.positions_rad)
            span_ms = point.offset_ms - previous_offset
            for index, position in enumerate(positions):
                if not (
                    self._limits.minimum_urad[index]
                    <= position
                    <= self._limits.maximum_urad[index]
                ):
                    raise BimanualStreamContractError(
                        "joint target outside approved range: "
                        f"joint={CANONICAL_JOINT_NAMES[index]} value={position}"
                    )
                if previous_positions is not None:
                    allowed = (
                        self._config.maximum_step_urad_per_tick
                        * (span_ms // CONTROL_TICK_MS)
                    )
                    if abs(position - previous_positions[index]) > allowed:
                        raise BimanualStreamContractError(
                            "joint step exceeds host policy: "
                            f"joint={CANONICAL_JOINT_NAMES[index]}"
                        )
            converted.append((point.offset_ms, positions))
            previous_offset = point.offset_ms
            previous_positions = positions
        return tuple(converted)

    def _pump_finite_plan(self) -> None:
        """Keep a prevalidated finite route ahead of the MCU cursor."""
        if not self._pending_finite_samples:
            return
        if self._finite_horizon_tick is None or self._tail_tick is None:
            raise BimanualStreamError("finite plan feeder has no horizon")
        base = self._healthy_heartbeat().last_heartbeat_ms
        future_route = self._future_route(self._route_samples, base)
        first_tick = self._pending_finite_samples[0].apply_tick
        if not _tick_after(first_tick, base):
            raise BimanualStreamError("finite plan feeder underrun")
        first_lead = (first_tick - base) & 0xFFFFFFFF
        if first_lead < self._config.minimum_lead_ms:
            raise BimanualStreamError(
                f"finite plan feeder underrun: first_lead_ms={first_lead}"
            )
        capacity = EXECUTOR_QUEUE_CAPACITY - len(future_route)
        count_limit = min(MAX_SAMPLES, capacity)
        ready: list[StreamSampleV2] = []
        for sample in self._pending_finite_samples:
            lead = (sample.apply_tick - base) & 0xFFFFFFFF
            if lead > self._config.maximum_lead_ms or len(ready) >= count_limit:
                break
            ready.append(sample)
        if not ready:
            return
        status = self._transport.append(
            StreamBatchV2(
                horizon_end_tick=self._finite_horizon_tick,
                arbiter_epoch=self._epoch,
                samples=tuple(ready),
                arm_mask=ARM_MASK_BOTH,
            )
        )
        self._require_accepted(status, "finite plan append")
        self._route_samples = future_route + tuple(ready)
        self._tail_tick = ready[-1].apply_tick
        self._tail_positions_urad = ready[-1].positions_urad
        self._pending_finite_samples = (
            self._pending_finite_samples[len(ready):]
        )

    def _policy(self, *, horizon_end_tick: int) -> StreamPolicyV2:
        timeout = (
            self._config.open_command_timeout_ms
            if horizon_end_tick == 0
            else self._config.finite_command_timeout_ms
        )
        return StreamPolicyV2(
            minimum_start_samples=self._config.minimum_start_samples,
            minimum_lead_ms=self._config.minimum_lead_ms,
            horizon_end_tick=horizon_end_tick,
            maximum_lead_ms=self._config.maximum_lead_ms,
            command_timeout_ms=timeout,
            maximum_apply_lateness_ms=self._config.maximum_apply_lateness_ms,
            tracking_error_limit_urad=tuple(
                self._config.gripper_tracking_error_limit_urad
                if index in (5, 11)
                else self._config.tracking_error_limit_urad
                for index in range(JOINT_COUNT)
            ),
            maximum_step_urad_per_tick=(
                self._config.maximum_step_urad_per_tick,
            )
            * JOINT_COUNT,
            arm_mask=ARM_MASK_BOTH,
        )

    @staticmethod
    def _require_accepted(status, label: str) -> None:
        if (
            status.status_code != StreamStatusCodeV2.OK
            or status.contract_result != StreamContractResultV2.OK
            or status.arm_mask != ARM_MASK_BOTH
        ):
            raise BimanualStreamError(f"{label} rejected: {status}")

    @staticmethod
    def _future_route(
        route: Sequence[StreamSampleV2],
        base_tick: int,
    ) -> tuple[StreamSampleV2, ...]:
        return tuple(
            sample for sample in route
            if _tick_after(sample.apply_tick, base_tick)
        )

    @staticmethod
    def _interpolate_route(
        route: Sequence[StreamSampleV2],
        apply_tick: int,
    ) -> tuple[int, ...]:
        for sample in route:
            if sample.apply_tick == apply_tick:
                return tuple(sample.positions_urad)
        for left, right in zip(route, route[1:]):
            if not (
                _tick_after(apply_tick, left.apply_tick)
                and _tick_after(right.apply_tick, apply_tick)
            ):
                continue
            span = (right.apply_tick - left.apply_tick) & 0xFFFFFFFF
            elapsed = (apply_tick - left.apply_tick) & 0xFFFFFFFF
            positions = []
            for left_position, right_position in zip(
                left.positions_urad,
                right.positions_urad,
                strict=True,
            ):
                numerator = (right_position - left_position) * elapsed
                delta = (
                    numerator // span
                    if numerator >= 0
                    else -((-numerator) // span)
                )
                positions.append(left_position + delta)
            return tuple(positions)
        raise BimanualStreamContractError(
            "splice point is not inside the currently admitted future route"
        )

    def start(
        self,
        owner: str,
        points: Sequence[TimedJointPoint],
        *,
        finite: bool,
    ):
        """Arm and admit the first finite or rolling-horizon batch."""
        if not self._motion_authorized:
            raise BimanualStreamError("motion_authorized is false")
        if self._state is not AdapterState.READY or self._prepared is None:
            raise BimanualStreamError("adapter is not ready")
        # A torque-off prepare anchor can become stale while the upper layer
        # plans or waits for an operator. Re-read both buses before validating
        # the first route. Any route based on an old pose is then rejected
        # before ARM/ENABLE rather than producing a catch-up command.
        if not self._armed:
            self.refresh_unarmed_anchor()
        if finite:
            converted_all = self._validate_finite_plan_points(points)
            initial_count = min(
                MAX_SAMPLES,
                sum(
                    offset <= self._config.maximum_lead_ms
                    for offset, _positions in converted_all
                ),
            )
            if initial_count < self._config.minimum_start_samples:
                raise BimanualStreamContractError(
                    "finite plan does not provide enough samples inside "
                    "the initial lead window"
                )
            converted = converted_all[:initial_count]
        else:
            if len(points) < self._config.minimum_start_samples:
                raise BimanualStreamContractError(
                    "initial batch has fewer than minimum_start_samples"
                )
            if points[0].offset_ms < self._config.minimum_lead_ms:
                raise BimanualStreamContractError(
                    "initial point lead is too short"
                )
            converted_all = self._validate_points(
                points,
                first_reference_urad=self._prepared.positions_urad,
                first_span_ms=points[0].offset_ms,
            )
            converted = converted_all
        already_owned = self._owner is not None
        self._claim(owner)
        if already_owned:
            self._advance_epoch()
        try:
            if not self._armed:
                self._armed = True
                self._transport.arm(CALIBRATION_HASH)
                post_arm = self._healthy_heartbeat()
                if post_arm.stop_latched or post_arm.status_code != 0:
                    raise BimanualStreamError(
                        f"post-arm heartbeat failed: {post_arm}"
                    )
                enabled = self._transport.enable()
                if enabled.stop_latched or enabled.status_code != 0:
                    raise BimanualStreamError(f"enable failed: {enabled}")
            base = self._healthy_heartbeat().last_heartbeat_ms
            all_sample_ticks = tuple(
                _tick(base, offset) for offset, _ in converted_all
            )
            sample_ticks = all_sample_ticks[:len(converted)]
            horizon = all_sample_ticks[-1] if finite else 0
            opened = self._transport.open_stream(
                self._policy(horizon_end_tick=horizon)
            )
            self._require_accepted(opened, "stream open")
            appended = self._transport.append(
                StreamBatchV2(
                    horizon_end_tick=horizon,
                    arbiter_epoch=self._epoch,
                    samples=tuple(
                        StreamSampleV2(tick, positions)
                        for tick, (_, positions) in zip(
                            sample_ticks, converted, strict=True
                        )
                    ),
                    arm_mask=ARM_MASK_BOTH,
                )
            )
            self._require_accepted(appended, "initial append")
            self._route_samples = tuple(
                StreamSampleV2(tick, positions)
                for tick, (_, positions) in zip(
                    sample_ticks, converted, strict=True
                )
            )
            self._tail_tick = sample_ticks[-1]
            self._tail_positions_urad = converted[-1][1]
            self._finite_horizon_tick = horizon if finite else None
            self._pending_finite_samples = tuple(
                StreamSampleV2(tick, positions)
                for tick, (_offset, positions) in zip(
                    all_sample_ticks[len(converted):],
                    converted_all[len(converted):],
                    strict=True,
                )
            )
            self._state = AdapterState.ACTIVE
            return appended
        except Exception as error:
            self._fault(error)

    def append(self, owner: str, points: Sequence[TimedJointPoint]):
        """Append points whose offsets are relative to a fresh firmware tick."""
        self._claim(owner)
        if (
            self._state is not AdapterState.ACTIVE
            or self._tail_tick is None
            or self._tail_positions_urad is None
        ):
            raise BimanualStreamError("stream is not active")
        try:
            base = self._healthy_heartbeat().last_heartbeat_ms
            future_route = self._future_route(self._route_samples, base)
            first_tick = _tick(base, points[0].offset_ms)
            if not _tick_after(first_tick, self._tail_tick):
                raise BimanualStreamContractError(
                    "append begins at or before the admitted tail"
                )
            gap = (first_tick - self._tail_tick) & 0xFFFFFFFF
            alignment_ms = (-gap) % CONTROL_TICK_MS
            if (
                points[-1].offset_ms + alignment_ms
                > self._config.maximum_lead_ms
            ):
                raise BimanualStreamContractError(
                    "append grid alignment exceeds maximum lead"
                )
            gap += alignment_ms
            converted = self._validate_points(
                points,
                first_reference_urad=self._tail_positions_urad,
                first_span_ms=gap,
            )
            sample_ticks = tuple(
                _tick(base, offset + alignment_ms)
                for offset, _ in converted
            )
            horizon = self._finite_horizon_tick or 0
            status = self._transport.append(
                StreamBatchV2(
                    horizon_end_tick=horizon,
                    arbiter_epoch=self._epoch,
                    samples=tuple(
                        StreamSampleV2(tick, positions)
                        for tick, (_, positions) in zip(
                            sample_ticks, converted, strict=True
                        )
                    ),
                    arm_mask=ARM_MASK_BOTH,
                )
            )
            self._require_accepted(status, "append")
            self._route_samples = future_route + tuple(
                StreamSampleV2(tick, positions)
                for tick, (_, positions) in zip(
                    sample_ticks, converted, strict=True
                )
            )
            self._tail_tick = sample_ticks[-1]
            self._tail_positions_urad = converted[-1][1]
            return status
        except Exception as error:
            self._fault(error)

    def splice(
        self,
        owner: str,
        points: Sequence[TimedJointPoint],
        *,
        splice_offset_ms: int,
    ):
        """Replace future targets while synthesizing the continuity point."""
        self._claim(owner)
        if self._state is not AdapterState.ACTIVE or self._tail_tick is None:
            raise BimanualStreamError("stream is not active")
        if (
            isinstance(splice_offset_ms, bool)
            or not isinstance(splice_offset_ms, int)
            or splice_offset_ms % CONTROL_TICK_MS != 0
        ):
            raise BimanualStreamContractError(
                "splice_offset_ms must be an integer multiple of 5 ms"
            )
        if splice_offset_ms < self._config.minimum_lead_ms:
            raise BimanualStreamContractError("splice lead is too short")
        if not 1 <= len(points) <= MAX_SAMPLES - 1:
            raise BimanualStreamContractError(
                "splice replacement must contain 1..8 target points"
            )
        if points[0].offset_ms <= splice_offset_ms:
            raise BimanualStreamContractError(
                "splice replacement points must occur after splice_offset_ms"
            )
        converted = self._validate_points(
            points,
            first_reference_urad=None,
            first_span_ms=points[0].offset_ms - splice_offset_ms,
        )
        try:
            base = self._healthy_heartbeat().last_heartbeat_ms
            future_route = self._future_route(self._route_samples, base)
            raw_splice_tick = _tick(base, splice_offset_ms)
            distance_to_tail = (self._tail_tick - raw_splice_tick) & 0xFFFFFFFF
            alignment_ms = distance_to_tail % CONTROL_TICK_MS
            if points[-1].offset_ms + alignment_ms > self._config.maximum_lead_ms:
                raise BimanualStreamContractError(
                    "splice grid alignment exceeds maximum lead"
                )
            splice_tick = _tick(base, splice_offset_ms + alignment_ms)
            if _tick_after(splice_tick, self._tail_tick):
                raise BimanualStreamContractError(
                    "splice begins after the admitted tail"
                )
            continuity_positions = self._interpolate_route(
                future_route,
                splice_tick,
            )
            converted = self._validate_points(
                points,
                first_reference_urad=continuity_positions,
                first_span_ms=points[0].offset_ms - splice_offset_ms,
            )
            target_ticks = tuple(
                _tick(base, offset + alignment_ms)
                for offset, _ in converted
            )
            self._advance_epoch()
            replacement_samples = (
                StreamSampleV2(splice_tick, continuity_positions),
            ) + tuple(
                StreamSampleV2(tick, positions)
                for tick, (_, positions) in zip(
                    target_ticks, converted, strict=True
                )
            )
            status = self._transport.splice(
                StreamBatchV2(
                    horizon_end_tick=0,
                    arbiter_epoch=self._epoch,
                    splice_at_tick=splice_tick,
                    samples=replacement_samples,
                    arm_mask=ARM_MASK_BOTH,
                )
            )
            self._require_accepted(status, "splice")
            retained = tuple(
                sample for sample in future_route
                if _tick_after(splice_tick, sample.apply_tick)
            )
            self._route_samples = retained + replacement_samples
            self._tail_tick = target_ticks[-1]
            self._tail_positions_urad = converted[-1][1]
            return status
        except Exception as error:
            self._fault(error)

    def poll(self, owner: str):
        """Check all runtime safety domains; fail closed on any inconsistency."""
        self._claim(owner)
        if self._state is not AdapterState.ACTIVE:
            raise BimanualStreamError("stream is not active")
        try:
            self._pump_finite_plan()
            state = self._healthy_heartbeat()
            executor = self._transport.get_executor_diagnostics()
            terminal_feedback = None
            if executor.state is StreamExecutorStateV2.SUCCEEDED:
                # Preserve freshness before later diagnostics transactions.
                terminal_feedback = self._validated_feedback_snapshot()
            state = self._healthy_heartbeat()
            dispatch = self._transport.get_dispatch_diagnostics()
            tracking = self._transport.get_tracking_diagnostics()
            state = self._healthy_heartbeat()
            if (
                dispatch.faulted
                or dispatch.failure_count
                or (not dispatch.active and not dispatch.ready)
            ):
                raise BimanualStreamError(f"dispatch fault: {dispatch}")
            # failed_pairs is cumulative degraded-telemetry evidence. F8.6
            # latches in firmware only when three read pairs fail
            # consecutively; a later successful pair clears that streak.
            # Measured tracking-limit violations and dispatch/DMA faults stay
            # immediately fail-closed in firmware and surface through the
            # heartbeat/dispatch/executor checks around this snapshot.
            if executor.state in (
                StreamExecutorStateV2.ABORTED,
                StreamExecutorStateV2.HOLD,
            ) or executor.safe_stop_required:
                raise BimanualStreamError(f"executor requires stop: {executor}")
            if executor.state is StreamExecutorStateV2.SUCCEEDED:
                if dispatch.active or tracking.active or tracking.pending:
                    return state, executor, dispatch, tracking
                if self._prepared is None or self._tail_positions_urad is None:
                    raise BimanualStreamError(
                        "finite completion has no resident anchor"
                    )
                if terminal_feedback is None:
                    raise BimanualStreamError(
                        "finite completion has no terminal feedback"
                    )
                feedback = terminal_feedback
                maximum_age = max(int(age) for age in feedback.sample_age_ms)
                if maximum_age > self._config.terminal_feedback_max_age_ms:
                    raise BimanualStreamError(
                        f"terminal feedback is stale: maximum_age_ms={maximum_age}"
                    )
                measured = tuple(int(value) for value in feedback.positions_urad)
                errors = tuple(
                    abs(actual - target)
                    for actual, target in zip(
                        measured, self._tail_positions_urad, strict=True
                    )
                )
                maximum_error = max(errors)
                tolerances = tuple(
                    self._config.terminal_settle_gripper_tolerance_urad
                    if index in (5, 11)
                    else self._config.terminal_settle_tolerance_urad
                    for index in range(JOINT_COUNT)
                )
                violations = tuple(
                    index
                    for index, (error, tolerance) in enumerate(
                        zip(errors, tolerances, strict=True)
                    )
                    if error > tolerance
                )
                if violations:
                    raise BimanualStreamError(
                        "terminal settle error: "
                        f"maximum_error_urad={maximum_error} "
                        f"violating_joints={violations}"
                    )
                self._prepared = PreparedBimanualState(
                    positions_raw=self._prepared.positions_raw,
                    unwrapped_positions_raw=(
                        self._prepared.unwrapped_positions_raw
                    ),
                    positions_urad=measured,
                )
                self._tail_tick = self._tail_positions_urad = None
                self._route_samples = ()
                self._finite_horizon_tick = None
                self._pending_finite_samples = ()
                self._state = AdapterState.READY
            return state, executor, dispatch, tracking
        except Exception as error:
            self._fault(error)

    def stop(self, owner: str) -> None:
        """Latch the coordinated dual-arm stop and release no new owner."""
        self._claim(owner)
        if self._state is AdapterState.STOPPED:
            return
        try:
            stopped = self._transport.safe_stop()
            if not stopped.stop_latched:
                raise BimanualStreamError("SAFE_STOP was not latched")
            self._armed = False
            self._route_samples = ()
            self._finite_horizon_tick = None
            self._pending_finite_samples = ()
            self._state = AdapterState.STOPPED
        except Exception as error:
            self._fault(error)
