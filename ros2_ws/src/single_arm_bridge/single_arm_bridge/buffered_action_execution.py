"""Runtime execution core for continuous buffered arm trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from .action_execution import ExecutionError, ExecutionOutcome, TerminalState
from .action_validation import ValidatedBufferedTrajectory
from .buffered_action_adapter import (
    INITIAL_FIRST_SAMPLE_LEAD_MS,
    MAXIMUM_APPLY_LATENESS_MS,
    STARTUP_PRIME_MINIMUM_ELAPSED_MS,
    STARTUP_PRIME_SAMPLES,
    UINT32_HALF_RANGE,
    UINT32_MAX,
    BufferedAdapterState,
    BufferedBatchScheduler,
    BufferedExecutionPlan,
    prepare_buffered_execution_plan,
    reanchor_buffered_execution_plan,
)
from .buffered_transport_driver import BufferedExchangeResponse, BufferedTransportDriver
from .calibration import ArmCalibration
from .hardware_identity import validate_hardware_identity
from .protocol import Hello


# **안전 허용치다. 과제 허용치가 아니다.**
#
# 이 값이 답하는 질문은 "동작이 잘못되지 않았다" 이지 "이 파지는 성공할
# 것이다" 가 아니다. 반경 0.4 m 에서 30 raw 는 약 19 mm 이고, 펜 파지가
# 요구하는 정밀도보다 한참 헐겁다. 정밀도 판정은 과제 계층이 하며 그 값은
# `buffered_trajectory_contract.json` 의 `grasp_convergence` 에 있다.
#
# Action 이 안전하지만 정밀하지 않은 동작을 실패로 만들면 안 된다. 그래서
# 이 값은 낮추지 않는다.
POST_SETTLE_TOLERANCE_RAW = 30
POST_SETTLE_CONSECUTIVE_SNAPSHOTS = 2
POST_SETTLE_TIMEOUT_S = 2.5
POST_SETTLE_POLL_INTERVAL_S = 0.1

# **속도를 올리면 관절이 정착에 더 오래 걸릴 수 있다 — 그 자체는 실패가
# 아니다.** 2026-08-07 speed-ramp 실기에서 120 raw/s 로 올렸더니 한 관절이
# 목표에서 283 raw 벗어난 채 시작해 매 관측마다 거의 일정한 속도로
# 줄어들다가(283→...→120) `POST_SETTLE_TIMEOUT_S`(2.5s)에 걸려 실패
# 처리됐다 — 추세선대로면 완주까지 ~4.5s 가 걸렸을 값이다.
#
# 그런데 2026-08-06 에는 같은 신호(정착 못 함)가 다른 모양으로 나온 적이
# 있다 — SHOULDER 가 32 raw 에 **멈춰서 14회 관측이 전부 동일**했다. 그건
# 아무리 기다려도 안 온다. 두 경우를 구분하는 것은 시간이 아니라 **매
# 관측마다 개선되고 있는가**다.
#
# 그래서 마감을 고정하지 않는다 — 매 관측이 직전보다 엄격히 좋아질 때만
# `POST_SETTLE_TIMEOUT_S` 단위로 연장하고, 이 상한에서 멈춘다. 정체·악화는
# 즉시 연장을 멈추므로 평형 사례는 원래와 똑같이 2.5s 에서 실패한다.
POST_SETTLE_MAXIMUM_TIMEOUT_S = 10.0
STARTUP_FIRST_SAMPLE_LEAD_GATE_MS = 80
STARTUP_MAXIMUM_HEARTBEAT_GATES = 8


@dataclass(frozen=True)
class PostSettleMeasurement:
    """post-settle 에서 실제로 관측된 것 전부.

    종전에는 `max()` 한 개만 남기고 목표·실측·관절별 오차를 전부 버렸다.
    그래서 2026-08-06 A4 파지 성공 회차와 A4.5 실패 회차의 **도달 자세를
    되살릴 수 없다.** 어느 관절이 얼마나 못 갔는지 모르면 그 오차가 TCP
    에서 몇 mm 였는지도 계산할 수 없고, 수렴 계층은 그 값 없이는 아무것도
    보정할 수 없다.

    `max_error_raw` 는 종전 반환값과 정확히 같은 수다. 나머지는 덧붙인 것이며
    Action 의 판정 논리는 바뀌지 않는다.
    """

    max_error_raw: int
    target_raw: tuple[int, ...]
    measured_raw: tuple[int, ...]
    error_raw: tuple[int, ...]

    def terminal_summary(self) -> str:
        """종단 문자열에 실을 형태. sender 가 이것을 증거로 보존한다."""
        def joined(values: tuple[int, ...]) -> str:
            return ",".join(str(value) for value in values)

        return (
            f"post_settle_target_raw={joined(self.target_raw)} "
            f"post_settle_measured_raw={joined(self.measured_raw)} "
            f"post_settle_error_raw={joined(self.error_raw)}"
        )


def format_apply_lateness_profile(result: Any) -> str:
    """
    Render the firmware's apply-lateness distribution for the terminal string.

    A single maximum cannot separate a rare spike from systemic drift.
    Motion-11 reached the top of its 0..5 ms allowance with only that number,
    which is why 0x00022800 reports per-bucket counts and the applied-sample
    index where the maximum was last raised. From 0x00022A00 the block
    rides on terminal frames only: every status frame is transmitted by a
    blocking call on the loop that steps the executor, so carrying it on
    refill acknowledgements charged its own length to the lateness it was
    measuring.

    Firmware older than 0x00022800 omits the block, so this degrades to a
    marker rather than failing.
    """
    histogram = getattr(result, "apply_lateness_histogram", None)
    if not histogram:
        return "lateness_profile=unavailable"
    buckets = ",".join(str(int(count)) for count in histogram)
    worst = getattr(result, "maximum_apply_lateness_sample_index", None)
    worst_text = "none" if not worst else str(int(worst))
    return f"lateness_buckets={buckets} lateness_worst_sample={worst_text}"


def format_f0_metrics(result: Any) -> str:
    """Render the F0 terminal-only timing snapshot without making it a gate."""
    values = (
        getattr(result, "f0_loop_period_max_us", None),
        getattr(result, "f0_loop_work_max_us", None),
        getattr(result, "f0_servo_sync_write_max_us", None),
        getattr(result, "f0_host_tx_max_us", None),
    )
    if any(value is None for value in values):
        return "f0_metrics=unavailable"
    loop_period, loop_work, servo_write, host_tx = (int(value) for value in values)
    return (
        f"f0_loop_period_max_us={loop_period} "
        f"f0_loop_work_max_us={loop_work} "
        f"f0_servo_sync_write_max_us={servo_write} "
        f"f0_host_tx_max_us={host_tx}"
    )


def format_h2_telemetry(result: Any) -> str:
    """Render H2.0 position-only in-motion telemetry when firmware supplies it."""
    maximum = getattr(result, "h2_tracking_error_max_raw", None)
    requested = getattr(result, "h2_telemetry_requested_samples", None)
    completed = getattr(result, "h2_telemetry_completed_samples", None)
    failed = getattr(result, "h2_telemetry_failed_samples", None)
    latency = getattr(result, "h2_telemetry_maximum_reply_latency_ms", None)
    if maximum is None or any(value is None for value in (requested, completed, failed, latency)):
        return "h2_telemetry=unavailable"
    maximum_text = ",".join(str(int(value)) for value in maximum)
    return (
        f"h2_tracking_error_max_raw={maximum_text} "
        f"h2_telemetry_requested={int(requested)} "
        f"h2_telemetry_completed={int(completed)} "
        f"h2_telemetry_failed={int(failed)} "
        f"h2_telemetry_reply_latency_max_ms={int(latency)}"
    )


def format_f3_control_tick_metrics(result: Any) -> str:
    """Render F3.0's observation-only TIM6 timing snapshot."""
    values = (
        getattr(result, "f3_control_tick_period_max_us", None),
        getattr(result, "f3_control_tick_jitter_max_us", None),
        getattr(result, "f3_control_tick_work_max_us", None),
        getattr(result, "f3_control_tick_count", None),
    )
    if any(value is None for value in values):
        return "f3_control_tick=unavailable"
    period, jitter, work, count = (int(value) for value in values)
    return (
        f"f3_control_tick_period_max_us={period} "
        f"f3_control_tick_jitter_max_us={jitter} "
        f"f3_control_tick_work_max_us={work} "
        f"f3_control_tick_count={count}"
    )


def _tick_has_reached(current_tick_ms: int, apply_tick_ms: int, margin_ms: int) -> bool:
    elapsed = (current_tick_ms - apply_tick_ms) & UINT32_MAX
    return margin_ms <= elapsed <= UINT32_HALF_RANGE


def conservative_applied_count(
    plan: BufferedExecutionPlan,
    *,
    accepted_samples: int,
    current_tick_ms: int,
) -> int:
    """Count only accepted samples whose apply deadline is safely in the past."""

    count = 0
    for sample in plan.samples[:accepted_samples]:
        if not _tick_has_reached(
            current_tick_ms,
            sample.apply_tick_ms,
            MAXIMUM_APPLY_LATENESS_MS,
        ):
            break
        count += 1
    return count


class BufferedActionExecutionCore:
    """Own one buffered goal and keep all transport failures fail-closed."""

    def __init__(
        self,
        transport: Any,
        hello: Hello,
        calibration: ArmCalibration,
        *,
        post_settle_timeout_s: float = POST_SETTLE_TIMEOUT_S,
        post_settle_poll_interval_s: float = POST_SETTLE_POLL_INTERVAL_S,
        post_settle_maximum_timeout_s: float = POST_SETTLE_MAXIMUM_TIMEOUT_S,
    ) -> None:
        validate_hardware_identity(hello, calibration.calibration_hash)
        if post_settle_timeout_s <= 0.0 or post_settle_poll_interval_s < 0.0:
            raise ValueError("post-settle timing values are invalid")
        if post_settle_maximum_timeout_s < post_settle_timeout_s:
            raise ValueError(
                "post-settle maximum timeout must not be below the base "
                "timeout"
            )
        self._transport = transport
        self._calibration = calibration
        self._blocked = hello.stop_latched
        self._post_settle_timeout_s = float(post_settle_timeout_s)
        self._post_settle_poll_interval_s = float(
            post_settle_poll_interval_s
        )
        self._post_settle_maximum_timeout_s = float(
            post_settle_maximum_timeout_s
        )
        self._plan: BufferedExecutionPlan | None = None
        self._scheduler: BufferedBatchScheduler | None = None
        self._driver: BufferedTransportDriver | None = None
        self._startup_diagnostics: str | None = None
        self._lock = threading.RLock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._scheduler is not None

    @property
    def blocked(self) -> bool:
        with self._lock:
            return self._blocked

    def start_goal(
        self,
        trajectory: ValidatedBufferedTrajectory,
        *,
        preserved_gripper_rad: float,
    ) -> BufferedExecutionPlan:
        with self._lock:
            if self._blocked:
                raise ExecutionError("buffered execution is blocked pending recovery")
            if self._scheduler is not None:
                raise ExecutionError("another buffered motion goal is active")
            self._startup_diagnostics = None
            stage = "precompute"
            precompute_started = time.monotonic()
            # Kept outside the try so the failure path can report whichever
            # phases had already been timed. The lead gate fails before
            # _startup_diagnostics exists, so without these the rejection
            # arrives with no breakdown -- which is what happened on
            # 2026-08-06 run 7 and left that stop unexplained.
            precompute_ms: float | None = None
            reanchor_ms: float | None = None
            prime_frame_1_ms: float | None = None
            try:
                plan = prepare_buffered_execution_plan(
                    trajectory,
                    self._calibration,
                    preserved_gripper_rad=preserved_gripper_rad,
                    # Provisional only: no physical apply tick is retained
                    # across potentially expensive position resampling.
                    current_tick_ms=0,
                )
                precompute_ms = round(
                    (time.monotonic() - precompute_started) * 1000.0,
                    3,
                )
                stage = "fresh_reanchor_heartbeat"
                heartbeat = self._transport.heartbeat()
                fresh_heartbeat_host_time = time.monotonic()
                # Everything from here until the prime-2 heartbeat is spent
                # out of the 220 ms lead that this heartbeat just bought.
                # precompute_ms sits *before* the heartbeat and therefore
                # cannot consume it -- so when the lead collapses the cause
                # has to be in this window, and the window has never been
                # broken down. Time its two parts separately.
                reanchor_started = time.monotonic()
                plan = reanchor_buffered_execution_plan(
                    plan,
                    current_tick_ms=heartbeat.last_heartbeat_ms,
                )
                scheduler = BufferedBatchScheduler(plan)
                driver = BufferedTransportDriver(scheduler, self._transport)
                self._plan = plan
                self._scheduler = scheduler
                self._driver = driver
                reanchor_ms = round(
                    (time.monotonic() - reanchor_started) * 1000.0, 3
                )
                stage = "prime_frame_1"
                prime_started = time.monotonic()
                first = driver.service_once(
                    current_tick_ms=heartbeat.last_heartbeat_ms
                )
                prime_frame_1_ms = round(
                    (time.monotonic() - prime_started) * 1000.0, 3
                )
                if first is None:
                    raise ExecutionError("startup prime frame 1 was not produced")

                second = None
                prime_heartbeat = None
                first_sample_lead_ms = None
                prime_heartbeat_gates = 0
                # A tight heartbeat-only retry loop does not reliably reach
                # STARTUP_PRIME_MINIMUM_ELAPSED_MS: on real hardware each
                # heartbeat round trip is only a few ms, so gates alone can
                # exhaust STARTUP_MAXIMUM_HEARTBEAT_GATES while still short of
                # the elapsed time the second batch needs to fall under the
                # firmware's maximum lead (2026-08-07 physical run: 8 gates,
                # still short). Sleep host-side toward that target first; the
                # gate loop below then only has to absorb clock jitter.
                stage = "prime_frame_2_wait"
                elapsed_since_reanchor_s = (
                    time.monotonic() - fresh_heartbeat_host_time
                )
                remaining_wait_s = (
                    STARTUP_PRIME_MINIMUM_ELAPSED_MS / 1000.0
                    - elapsed_since_reanchor_s
                )
                if remaining_wait_s > 0.0:
                    time.sleep(remaining_wait_s)
                for gate_index in range(1, STARTUP_MAXIMUM_HEARTBEAT_GATES + 1):
                    stage = f"prime_frame_2_heartbeat_{gate_index}"
                    prime_heartbeat = self._transport.heartbeat()
                    prime_heartbeat_gates += 1
                    first_sample_lead_ms = (
                        plan.samples[0].apply_tick_ms
                        - prime_heartbeat.last_heartbeat_ms
                    ) & UINT32_MAX
                    if not (
                        STARTUP_FIRST_SAMPLE_LEAD_GATE_MS
                        <= first_sample_lead_ms
                        <= INITIAL_FIRST_SAMPLE_LEAD_MS
                    ):
                        raise ExecutionError(
                            "startup first-sample lead gate failed: "
                            f"lead_ms={first_sample_lead_ms} "
                            f"required={STARTUP_FIRST_SAMPLE_LEAD_GATE_MS}.."
                            f"{INITIAL_FIRST_SAMPLE_LEAD_MS} "
                            f"heartbeat_gates={prime_heartbeat_gates}"
                        )
                    stage = f"prime_frame_2_attempt_{gate_index}"
                    second = driver.service_once(
                        current_tick_ms=prime_heartbeat.last_heartbeat_ms
                    )
                    if second is not None:
                        break
                if second is None:
                    raise ExecutionError(
                        "startup prime frame 2 did not enter the reviewed "
                        f"horizon after {prime_heartbeat_gates} heartbeat gates"
                    )
                assert prime_heartbeat is not None
                assert first_sample_lead_ms is not None
                snapshot = scheduler.snapshot()
                if (
                    snapshot.accepted_samples != STARTUP_PRIME_SAMPLES
                    or snapshot.applied_samples != 0
                    or snapshot.queued_samples != STARTUP_PRIME_SAMPLES
                    or snapshot.pending_batch
                ):
                    raise ExecutionError(
                        "startup prime accounting gate failed: "
                        f"accepted={snapshot.accepted_samples} "
                        f"applied={snapshot.applied_samples} "
                        f"queued={snapshot.queued_samples} "
                        f"pending={int(snapshot.pending_batch)}"
                    )
                self._startup_diagnostics = (
                    f"precompute_ms={precompute_ms:.3f} "
                    f"reanchor_ms={reanchor_ms:.3f} "
                    f"prime_frame_1_ms={prime_frame_1_ms:.3f} "
                    f"fresh_tick={heartbeat.last_heartbeat_ms} "
                    f"prime_tick={prime_heartbeat.last_heartbeat_ms} "
                    f"first_sample_lead_ms={first_sample_lead_ms} "
                    f"prime_heartbeat_gates={prime_heartbeat_gates} "
                    f"prime_frames={driver.commands_sent} "
                    f"accepted={snapshot.accepted_samples} "
                    f"applied={snapshot.applied_samples} "
                    f"queued={snapshot.queued_samples}"
                )
                return plan
            except Exception as error:
                diagnostics = self._startup_diagnostics or " ".join(
                    f"{name}={value:.3f}"
                    for name, value in (
                        (
                            "precompute_ms",
                            precompute_ms
                            if precompute_ms is not None
                            else (time.monotonic() - precompute_started)
                            * 1000.0,
                        ),
                        ("reanchor_ms", reanchor_ms),
                        ("prime_frame_1_ms", prime_frame_1_ms),
                    )
                    if value is not None
                )
                self._clear_active()
                self._fail_closed()
                raise ExecutionError(
                    "buffered start failed: "
                    f"stage={stage} {diagnostics} error={error}"
                ) from error

    def poll(self) -> ExecutionOutcome | None:
        with self._lock:
            scheduler, driver, plan = self._require_active()
            try:
                terminal = self._take_terminal(driver)
                if terminal is not None:
                    return self._finish_terminal(terminal)
                heartbeat = self._transport.heartbeat()
                snapshot = scheduler.snapshot()
                if snapshot.state in (
                    BufferedAdapterState.RUNNING,
                    BufferedAdapterState.INPUT_COMPLETE,
                ):
                    applied = conservative_applied_count(
                        plan,
                        accepted_samples=snapshot.accepted_samples,
                        current_tick_ms=heartbeat.last_heartbeat_ms,
                    )
                    if applied > snapshot.applied_samples:
                        scheduler.record_clock_progress(applied)
                driver.service_once(current_tick_ms=heartbeat.last_heartbeat_ms)
                terminal = self._take_terminal(driver)
                if terminal is not None:
                    return self._finish_terminal(terminal)
                return None
            except Exception as error:
                return self._abort_active(
                    f"buffered runtime failed: {error}"
                )

    def cancel_active_goal(self) -> ExecutionOutcome:
        with self._lock:
            scheduler, _, _ = self._require_active()
            scheduler.cancel()
            self._clear_active()
            self._blocked = True
            try:
                self._transport.safe_stop()
                return ExecutionOutcome(
                    TerminalState.CANCELED,
                    0,
                    8,
                    None,
                    "buffered goal canceled and SAFE_STOP latched",
                )
            except Exception as error:
                return ExecutionOutcome(
                    TerminalState.ABORTED,
                    0,
                    None,
                    None,
                    f"buffered cancel SAFE_STOP failed: {error}",
                )

    def handle_connection_loss(self, reason: str) -> ExecutionOutcome | None:
        with self._lock:
            if self._scheduler is None:
                self._blocked = True
                return None
            return self._abort_active(f"connection lost: {reason}")

    def replace_transport_after_explicit_recovery(
        self,
        transport: Any,
        hello: Hello,
    ) -> None:
        with self._lock:
            if self._scheduler is not None:
                raise ExecutionError("cannot recover while a buffered goal is active")
            validate_hardware_identity(hello, self._calibration.calibration_hash)
            if hello.stop_latched:
                self._blocked = True
                raise ExecutionError("cannot recover while stop is latched")
            self._transport = transport
            self._blocked = False

    def _take_terminal(
        self,
        driver: BufferedTransportDriver,
    ) -> BufferedExchangeResponse | None:
        responses = self._transport.drain_buffered_motion_results()
        if not responses:
            return None
        if len(responses) > 1:
            raise ExecutionError("multiple buffered terminals received")
        driver.observe_terminal(responses[0])
        return responses[0]

    def _finish_terminal(
        self,
        response: BufferedExchangeResponse,
    ) -> ExecutionOutcome:
        scheduler, _, plan = self._require_active()
        snapshot = scheduler.snapshot()
        result = response.result
        if snapshot.state is not BufferedAdapterState.SUCCEEDED:
            reason = snapshot.reason or snapshot.state.value
            return self._abort_active(
                "buffered terminal did not succeed: "
                f"state={snapshot.state.value} reason={reason}",
                request_sequence=result.request_sequence,
                status_code=result.status_code,
                detail=result.detail,
            )
        try:
            settle = self._verify_post_settle(plan)
        except Exception as error:
            return self._abort_active(
                f"buffered post-settle diagnostics failed: {error}",
                request_sequence=result.request_sequence,
                status_code=result.status_code,
                detail=result.detail,
            )
        startup = self._startup_diagnostics or "startup=unavailable"
        lateness = format_apply_lateness_profile(result)
        f0_metrics = format_f0_metrics(result)
        h2_telemetry = format_h2_telemetry(result)
        f3_tick = format_f3_control_tick_metrics(result)
        self._clear_active()
        return ExecutionOutcome(
            TerminalState.SUCCEEDED,
            result.request_sequence,
            result.status_code,
            result.detail,
            "buffered trajectory completed; "
            f"maximum_apply_lateness_ms={result.detail} "
            f"post_settle_max_error_raw={settle.max_error_raw}; "
            f"{startup}; {lateness}; {f0_metrics}; {h2_telemetry}; {f3_tick}; "
            f"{settle.terminal_summary()}",
        )

    def _verify_post_settle(
        self, plan: BufferedExecutionPlan
    ) -> PostSettleMeasurement:
        final = (*plan.final_arm_positions_rad, plan.preserved_gripper_rad)
        targets = tuple(
            round(
                joint.zero_raw
                + joint.direction * position * 4096.0 / (2.0 * math.pi)
            )
            for joint, position in zip(
                self._calibration.joints,
                final,
                strict=True,
            )
        )
        started_at = time.monotonic()
        deadline = started_at + self._post_settle_timeout_s
        absolute_deadline = started_at + self._post_settle_maximum_timeout_s
        consecutive = 0
        maximum = 0
        last_error = 0
        previous_last_error: int | None = None
        observations = 0
        heartbeat_gates = 0
        error_trace: list[tuple[int, ...]] = []
        per_axis_minimum: tuple[int, ...] | None = None
        final_errors: tuple[int, ...] | None = None
        best_maximum_error: int | None = None

        def elapsed_ms() -> int:
            return max(0, round((time.monotonic() - started_at) * 1000.0))

        def format_errors(values: tuple[int, ...] | None) -> str:
            if values is None:
                return "none"
            return "[" + ",".join(str(value) for value in values) + "]"

        def diagnostic_context() -> str:
            trace = "|".join(
                f"{index}:{format_errors(values)}"
                for index, values in enumerate(error_trace, start=1)
            )
            return (
                f"observations={observations} consecutive={consecutive} "
                f"heartbeat_gates={heartbeat_gates} "
                f"elapsed_ms={elapsed_ms()} "
                f"error_trace_raw={trace or 'none'} "
                "per_axis_minimum_error_raw="
                f"{format_errors(per_axis_minimum)} "
                f"final_errors_raw={format_errors(final_errors)} "
                "best_maximum_error_raw="
                f"{best_maximum_error if best_maximum_error is not None else 'none'}"
            )

        def heartbeat_gate(stage: str) -> None:
            nonlocal heartbeat_gates
            try:
                self._transport.heartbeat()
            except Exception as error:
                raise ExecutionError(
                    "post-settle heartbeat gate failed "
                    f"stage={stage} {diagnostic_context()}: {error}"
                ) from error
            heartbeat_gates += 1

        # A successful firmware terminal ends setpoint application, but does
        # not relax the 500 ms MCU heartbeat watchdog.  Re-establish the host
        # lease before any potentially long servo-bus position sweep.
        heartbeat_gate("terminal_received")
        while time.monotonic() < deadline:
            stage = f"position_snapshot_{observations + 1}"
            heartbeat_gate(stage)
            try:
                state = self._transport.get_state(include_positions=True)
            except Exception as error:
                raise ExecutionError(
                    "post-settle position feedback failed "
                    f"stage={stage} {diagnostic_context()}: {error}"
                ) from error
            if state.raw_positions is None:
                raise ExecutionError(
                    "position-only post-settle feedback is missing "
                    f"stage={stage} {diagnostic_context()}"
                )
            errors = tuple(
                abs(position - target)
                for position, target in zip(
                    state.raw_positions,
                    targets,
                    strict=True,
                )
            )
            observations += 1
            error_trace.append(errors)
            final_errors = errors
            if per_axis_minimum is None:
                per_axis_minimum = errors
            else:
                per_axis_minimum = tuple(
                    min(previous, current)
                    for previous, current in zip(
                        per_axis_minimum,
                        errors,
                        strict=True,
                    )
                )
            last_error = max(errors)
            best_maximum_error = (
                last_error
                if best_maximum_error is None
                else min(best_maximum_error, last_error)
            )
            if last_error <= POST_SETTLE_TOLERANCE_RAW:
                consecutive += 1
                maximum = max(maximum, last_error)
                if consecutive >= POST_SETTLE_CONSECUTIVE_SNAPSHOTS:
                    break
            else:
                consecutive = 0
                maximum = 0
                # Still outside tolerance. A strictly improving worst-axis
                # error means the arm is genuinely catching up, not stuck --
                # buy it one more base timeout window, capped at the
                # absolute ceiling. Anything else (stalled or worsening)
                # gets no extension, so a true plateau still fails at the
                # original POST_SETTLE_TIMEOUT_S boundary.
                if (
                    previous_last_error is not None
                    and last_error < previous_last_error
                    and deadline < absolute_deadline
                ):
                    deadline = min(
                        deadline + self._post_settle_timeout_s,
                        absolute_deadline,
                    )
            previous_last_error = last_error
            time.sleep(self._post_settle_poll_interval_s)
        else:
            raise ExecutionError(
                f"last maximum error {last_error} did not provide "
                f"{POST_SETTLE_CONSECUTIVE_SNAPSHOTS} consecutive "
                f"position-only snapshots within "
                f"{POST_SETTLE_TOLERANCE_RAW} raw; "
                f"{diagnostic_context()}"
            )

        heartbeat_gate("full_diagnostics")
        try:
            diagnostics = self._transport.get_diagnostics()
        except Exception as error:
            raise ExecutionError(
                "post-settle full diagnostics failed "
                f"stage=full_diagnostics {diagnostic_context()}: {error}"
            ) from error
        if any(not sample.torque_enabled for sample in diagnostics.joints):
            raise ExecutionError(
                "torque disabled before final full diagnostics verification; "
                f"{diagnostic_context()}"
            )
        diagnostic_errors = tuple(
            abs(sample.position_raw - target)
            for sample, target in zip(
                diagnostics.joints,
                targets,
                strict=True,
            )
        )
        diagnostic_error = max(diagnostic_errors)
        if diagnostic_error > POST_SETTLE_TOLERANCE_RAW:
            raise ExecutionError(
                f"final full diagnostics maximum error {diagnostic_error} "
                f"exceeds {POST_SETTLE_TOLERANCE_RAW} raw; "
                "diagnostic_errors_raw="
                f"{format_errors(diagnostic_errors)} "
                f"{diagnostic_context()}"
            )
        return PostSettleMeasurement(
            max_error_raw=max(maximum, diagnostic_error),
            target_raw=targets,
            # 최종 full diagnostics 는 torque 확인까지 마친 완전한 읽기다.
            # 실측 자세로 남길 값은 이것이다.
            measured_raw=tuple(
                int(sample.position_raw) for sample in diagnostics.joints
            ),
            error_raw=diagnostic_errors,
        )

    def _abort_active(
        self,
        reason: str,
        *,
        request_sequence: int = 0,
        status_code: int | None = None,
        detail: int | None = None,
    ) -> ExecutionOutcome:
        startup = self._startup_diagnostics or "startup=unavailable"
        self._clear_active()
        self._fail_closed()
        return ExecutionOutcome(
            TerminalState.ABORTED,
            request_sequence,
            status_code,
            detail,
            f"{reason}; {startup}",
        )

    def _fail_closed(self) -> None:
        self._blocked = True
        try:
            self._transport.safe_stop()
        except Exception:
            pass

    def _require_active(self):
        if self._scheduler is None or self._driver is None or self._plan is None:
            raise ExecutionError("there is no active buffered goal")
        return self._scheduler, self._driver, self._plan

    def _clear_active(self) -> None:
        self._scheduler = None
        self._driver = None
        self._plan = None
        self._startup_diagnostics = None
