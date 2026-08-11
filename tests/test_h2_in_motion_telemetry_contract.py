"""H2.0 must observe buffered motion without stealing a servo output slot."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")
SERVO_HEADER = (FIRMWARE / "Core/Inc/servo_bus.h").read_text(encoding="utf-8")
SERVO = (FIRMWARE / "Core/Src/servo_bus.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
HOST_TX = (FIRMWARE / "Core/Src/host_uart_tx.c").read_text(encoding="utf-8")
PROTOCOL = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/protocol.py"
).read_text(encoding="utf-8")
IDENTITY = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
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


def test_h2_identity_and_terminal_wire_schema_are_explicit() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG
    assert "HOST_H2_IN_MOTION_TELEMETRY_CAPABILITY UINT32_C(0x00008000)" in CONFIG
    assert "H2_TELEMETRY_TIMING_CANDIDATE_FIRMWARE_VERSION = 0x00023400" in IDENTITY
    assert "in-motion telemetry capability is missing" in IDENTITY
    assert "SETPOINT_STATUS_H2_TELEMETRY = struct.Struct(\"<6H4I\")" in PROTOCOL
    assert "HOST_H2_TERMINAL_TELEMETRY_SIZE 28U" in BINARY


def test_h2_read_request_is_interrupt_driven_and_reply_is_incremental() -> None:
    start = function_body(SERVO, "HAL_StatusTypeDef Servo_InMotionTelemetryStart(")
    poll = function_body(SERVO, "HAL_StatusTypeDef Servo_InMotionTelemetryPoll(")
    assert "HAL_UART_Transmit_IT(" in start
    assert "HAL_UART_Transmit(" not in start
    assert "ServoBus_PrepareTransaction" in start
    assert "ServoRxWindow_Consume(" in poll
    assert "SERVO_IN_MOTION_TELEMETRY_TIMEOUT_MS" in poll
    assert "SERVO_IN_MOTION_TELEMETRY_TIMEOUT_MS UINT32_C(4)" in SERVO
    assert "servo_in_motion_snapshot.completed_samples++" in poll
    assert "servo_in_motion_snapshot.failed_samples++" in SERVO
    assert "Servo_InMotionTelemetryOnTxComplete" in HOST_TX
    assert "Servo_InMotionTelemetryOnTxComplete(host_uart);" in HOST_TX
    assert "ServoInMotionTelemetrySnapshot" in SERVO_HEADER


def test_h2_never_delays_a_sync_write_for_telemetry() -> None:
    service = function_body(BINARY, "static void Host_ServiceBufferedExecution(void)")
    poll = service.index("Servo_InMotionTelemetryPoll(")
    pending = service.index("Servo_InMotionTelemetryPending()")
    write = service.index("Servo_SyncWritePositions(output_positions_raw)")
    start = service.index("Servo_InMotionTelemetryStart(")
    assert poll < pending < write < start
    assert "Host_AbortBufferedExecution(" in service[pending:write]
    assert "diagnostics->state != ACTUATOR_BUFFERED_SUCCEEDED" in service[write:start]


def test_h2_snapshot_is_terminal_only_and_survives_until_status_is_encoded() -> None:
    status = function_body(BINARY, "static void Host_SendBinaryBufferedSetpointStatus(")
    final = function_body(BINARY, "static void Host_FinalizeBufferedExecution(uint8_t detail)")
    assert "if (status_code == HOST_BUFFERED_STATUS_TERMINAL)" in status
    assert "Servo_InMotionTelemetryGetSnapshot()" in status
    assert "payload_length += HOST_H2_TERMINAL_TELEMETRY_SIZE;" in status
    assert final.index("Host_SendBinaryBufferedSetpointStatus(") < final.index(
        "Servo_InMotionTelemetryEnd();"
    )
