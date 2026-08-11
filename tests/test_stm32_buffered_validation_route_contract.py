from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.hardware_identity import (  # noqa: E402
    HardwareIdentityError,
    validate_hardware_identity,
)
from single_arm_bridge.protocol import Hello  # noqa: E402


CONFIG = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def hello(
    *,
    firmware_version: int = 0x00022F00,
    capabilities: int = 0x00000FFF,
) -> Hello:
    return Hello(
        protocol_version=1,
        joint_count=6,
        stop_latched=False,
        firmware_version=firmware_version,
        calibration_hash=0x2D90167E,
        capabilities=capabilities,
        rejected_frame_count=0,
    )


def test_identity_and_capability_are_fail_closed() -> None:
    validate_hardware_identity(hello(), 0x2D90167E)

    with pytest.raises(HardwareIdentityError, match="firmware version mismatch"):
        validate_hardware_identity(
            hello(firmware_version=0x00021800),
            0x2D90167E,
        )
    with pytest.raises(
        HardwareIdentityError,
        match="buffered validation route capability is missing",
    ):
        validate_hardware_identity(
            hello(capabilities=0x000003FF),
            0x2D90167E,
        )
    with pytest.raises(
        HardwareIdentityError,
        match="buffered execution route capability is missing",
    ):
        validate_hardware_identity(
            hello(capabilities=0x000007FF),
            0x2D90167E,
        )
    validate_hardware_identity(
        hello(firmware_version=0x00023200, capabilities=0x00007FFF),
        0x2D90167E,
    )
    validate_hardware_identity(
        hello(firmware_version=0x00023400, capabilities=0x0000FFFF),
        0x2D90167E,
    )
    with pytest.raises(
        HardwareIdentityError,
        match="in-motion telemetry capability is missing",
    ):
        validate_hardware_identity(
            hello(firmware_version=0x00023400, capabilities=0x00007FFF),
            0x2D90167E,
        )
    with pytest.raises(
        HardwareIdentityError,
        match="heartbeat RX timestamp capability is missing",
    ):
        validate_hardware_identity(
            hello(firmware_version=0x00023200, capabilities=0x00003FFF),
            0x2D90167E,
        )


def test_capability_is_removed_when_route_initialization_fails() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG
    assert "HOST_BUFFERED_VALIDATION_CAPABILITY UINT32_C(0x00000400)" in CONFIG
    assert "HOST_BUFFERED_EXECUTION_CAPABILITY UINT32_C(0x00000800)" in CONFIG

    body = function_body(BINARY, "static uint32_t Host_BinaryCapabilities(void)")
    assert "host_buffered_validation_route_ready == 0U" in body
    assert "capabilities &= ~HOST_BUFFERED_VALIDATION_CAPABILITY" in body
    assert "capabilities &= ~HOST_BUFFERED_EXECUTION_CAPABILITY" in body


def test_candidate_route_is_validation_only_and_never_writes_servos() -> None:
    body = function_body(
        BINARY,
        "static void Host_ValidateBufferedCandidate(",
    )
    assert "ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY" in body
    assert "actuator_buffered_command_decode(" in body
    assert "actuator_buffered_command_route_admit(" in body
    assert "Host_SendBinaryBufferedSetpointStatus(" in body
    assert "Host_StartBinaryMotion(" not in body
    assert "actuator_buffered_command_route_start(" not in body
    assert "actuator_buffered_command_route_step(" not in body
    assert "Servo_SyncWritePositions(" not in body


def test_candidate_validation_is_available_while_physically_disabled() -> None:
    body = function_body(
        BINARY,
        "static void Host_ValidateBufferedCandidate(",
    )
    assert "actuator_safety_accepts_setpoint(" not in body
    assert "host_stop_latched != 0U" in body
    assert "ACTUATOR_STATE_FAULT" in body
    assert "ACTUATOR_STATE_ESTOPPED" in body
    assert "host_binary_motion.active != 0U" in body


def test_dispatch_separates_candidate_from_legacy_motion() -> None:
    handler = function_body(
        BINARY,
        "static void Host_HandleBinaryFrame(",
    )
    start = handler.index("case ACTUATOR_MSG_SETPOINT_BATCH:")
    end = handler.index("case ACTUATOR_MSG_SAFE_STOP:", start)
    setpoint_case = handler[start:end]
    assert "ACTUATOR_BUFFERED_FLAG_CANDIDATE" in setpoint_case
    assert "Host_ValidateBufferedCandidate(request);" in setpoint_case
    assert "Host_ExecuteBufferedCandidate(request);" in setpoint_case
    assert "Host_ValidateLegacyBinarySetpointBatch(request);" in setpoint_case

    legacy = function_body(
        BINARY,
        "static void Host_ValidateLegacyBinarySetpointBatch(",
    )
    assert "if (sample_count == 1U)" in legacy
    assert "Host_StartBinaryMotion(" in legacy


def test_candidate_status_is_extended_but_legacy_status_remains_16_bytes() -> None:
    extended = function_body(
        BINARY,
        "static void Host_SendBinaryBufferedSetpointStatus(",
    )
    legacy = function_body(
        BINARY,
        "static void Host_SendBinarySetpointStatus(",
    )
    assert "actuator_buffered_status_encode(" in extended
    assert "response.payload_length = 16U;" in legacy


def test_physical_candidate_uses_reviewed_timing_and_no_start_read_sweep() -> None:
    execute = function_body(BINARY, "static void Host_ExecuteBufferedCandidate(")
    service = function_body(BINARY, "static void Host_ServiceBufferedExecution(")

    assert "HOST_BUFFERED_EXECUTION_MINIMUM_LEAD_MS" in execute
    assert "HOST_BUFFERED_EXECUTION_MAXIMUM_LEAD_MS" in execute
    assert "HOST_BUFFERED_EXECUTION_ANCHOR_OFFSET_MS" in execute
    assert "command.samples[0].position_urad" in execute
    assert "Servo_PositionSweep" not in execute
    assert "actuator_buffered_command_route_start(" in execute
    assert "actuator_buffered_command_route_step(" in service
    assert "HOST_BUFFERED_EXECUTION_OUTPUT_PERIOD_MS" in service
    assert "Servo_SyncWritePositions(" in service
    # buffered 실행 경로는 servo read 를 전혀 하지 않는다. read 1회 최악 비용이
    # 79 ms 로 16 ms 폴링 slot 을 넘겨 host UART 처리를 굶겼기 때문이다.
    # 자세한 근거는 tests/test_stm32_main_loop_blocking_budget.py 에 있다.
    assert "Servo_MotionSafetyPoll()" not in service


def test_physical_terminal_paths_are_fail_closed_and_no_retry_exists() -> None:
    abort = function_body(BINARY, "static void Host_AbortBufferedExecution(")
    finalize = function_body(BINARY, "static void Host_FinalizeBufferedExecution(")
    execute = function_body(BINARY, "static void Host_ExecuteBufferedCandidate(")

    assert "ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS" in abort
    assert "actuator_buffered_command_route_tracking_error(" in abort
    assert "diagnostics->safe_stop_required" in finalize
    assert "host_stop_latched = 1U" in finalize
    assert "status_code = 2U" in execute
    assert "retry" not in execute.lower()


def test_lost_start_frame_cannot_leave_priming_active_forever() -> None:
    service = function_body(BINARY, "static void Host_ServiceBufferedExecution(")
    not_started = service.index("if (!host_buffered_execution_route.started)")
    pre_anchor = service.index(
        "host_binary_buffered_motion.anchor_tick) < 0",
        not_started,
    )
    priming_guard = service[not_started:pre_anchor]

    assert "HAL_GetTick()" in service[:not_started]
    assert "anchor_tick) >= 0" in priming_guard
    assert "Host_AbortBufferedExecution(" in priming_guard
    assert "ACTUATOR_BUFFERED_REASON_TRACKING_ERROR" in priming_guard
    assert "ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK" in priming_guard
