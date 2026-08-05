import struct
from pathlib import Path

import pytest

from single_arm_bridge.protocol import ProtocolError, parse_state
from single_arm_bridge.transport import PositionReadError


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text(encoding="utf-8")
IDENTITY = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
).read_text(encoding="utf-8")
TRANSPORT = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/transport.py"
).read_text(encoding="utf-8")
BRIDGE = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bridge_node.py"
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


def test_identity_requires_position_read_failure_diagnostics() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022A00)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)" in CONFIG
    assert "POSITION_READ_FAILURE_DIAGNOSTICS_CAPABILITY = 0x00000100" in IDENTITY
    assert "SERVO_BUS_RECOVERY_DIAGNOSTICS_CAPABILITY = 0x00000200" in IDENTITY
    assert "position read failure diagnostics capability is missing" in IDENTITY
    assert "servo bus recovery diagnostics capability is missing" in IDENTITY


def test_background_get_state_requires_three_failed_sweeps() -> None:
    body = function_body(BINARY, "static void Host_SendBinaryStateWithPositions(")
    failure = body[body.index("if (Servo_ReadAllPositions") : body.index(
        "actuator_frame_t response;"
    )]
    assert "host_position_read_failed_servo_id" in failure
    assert "servo_last_all_read_failed_id" in failure
    assert "host_position_read_failure_streak++" in failure
    assert "HOST_POSITION_READ_FAILURE_LIMIT UINT8_C(3)" in CONFIG
    assert (
        "host_position_read_failure_streak >=\n"
        "            HOST_POSITION_READ_FAILURE_LIMIT"
    ) in failure
    assert failure.index("HOST_POSITION_READ_FAILURE_LIMIT") < failure.index(
        "host_stop_latched = 1U"
    )
    assert "Host_SendBinaryPositionReadFailure(request_sequence);" in failure
    assert "Host_ResetPositionReadFailure();" in body


def test_motion_start_and_final_verification_remain_fail_closed() -> None:
    body = function_body(BINARY, "static void Host_ServiceBinaryMotion(void)")
    start_failure = body[
        body.index("if (start_status != HAL_OK)") : body.index(
            "memcpy(", body.index("if (start_status != HAL_OK)")
        )
    ]
    verify_failure = body[
        body.index("if (verify_status != HAL_OK)") : body.index(
            "host_binary_motion.verify_sweep_active = 0U;",
            body.index("if (verify_status != HAL_OK)"),
        )
    ]
    assert "host_stop_latched = 1U;" in start_failure
    assert "host_stop_latched = 1U;" in verify_failure
    assert "servo_last_all_read_failed_id" in start_failure
    assert "servo_last_all_read_failed_id" in verify_failure


def test_position_read_failure_wire_payload_reports_axis_and_streak() -> None:
    base = struct.pack(
        "<BBBBIIII",
        0,
        2,
        6,
        1,
        100,
        4,
        0xB317C672,
        5000,
    )
    state = parse_state(base + struct.pack("<BBBB", 3, 2, 3, 0))
    assert state.status_code == 2
    assert not state.stop_latched
    assert state.raw_positions is None
    assert state.position_read_failed_servo_id == 3
    assert state.position_read_failure_streak == 2
    assert state.position_read_failure_limit == 3

    latched = parse_state(
        bytes((1,)) + (base + struct.pack("<BBBB", 3, 3, 3, 0))[1:]
    )
    assert latched.stop_latched
    assert latched.position_read_failure_streak == 3

    extended = parse_state(
        base
        + struct.pack(
            "<BBBBBBHH2xII",
            1,
            2,
            3,
            8,
            1,
            0,
            17,
            9,
            0x00000004,
            0x00A000E0,
        )
    )
    assert extended.position_read_failed_servo_id == 1
    assert extended.position_read_failure_reason == 8
    assert extended.position_read_hal_status == 1
    assert extended.position_read_servo_status == 0
    assert extended.position_read_recovery_count == 17
    assert extended.position_read_discarded_bytes == 9
    assert extended.position_read_uart_error_code == 0x00000004
    assert extended.position_read_uart_isr == 0x00A000E0

    snapshot_extended = parse_state(
        base
        + struct.pack(
            "<BBBBBBHH2xIIBB16s",
            1, 2, 3, 4, 3, 0, 18, 4,
            0, 0x006010C0, 4, 0,
            bytes.fromhex("ffff0104") + bytes(12),
        )
    )
    assert snapshot_extended.position_read_snapshot == bytes.fromhex("ffff0104")
    assert not snapshot_extended.position_read_receiver_armed

    with pytest.raises(ProtocolError):
        parse_state(base + b"\x03\x02")


def test_transport_error_preserves_failed_axis_and_latch_state() -> None:
    error = PositionReadError(servo_id=4, streak=2, limit=3, stop_latched=False)
    assert error.servo_id == 4
    assert error.streak == 2
    assert error.limit == 3
    assert not error.stop_latched
    assert "servo_id=4 streak=2/3 latched=0" in str(error)


def test_transport_error_surfaces_bus_failure_cause() -> None:
    error = PositionReadError(
        servo_id=1,
        streak=3,
        limit=3,
        stop_latched=True,
        reason=3,
        hal_status=1,
        servo_status=0,
        recovery_count=12,
        discarded_bytes=7,
        uart_error_code=4,
        uart_isr=0xE0,
    )
    message = str(error)
    assert "cause=uart" in message
    assert "recoveries=12 discarded=7" in message
    assert "uart_error=0x00000004" in message
    assert "uart_isr=0x000000E0" in message

    snapshot_error = PositionReadError(
        servo_id=1, streak=1, limit=3, stop_latched=False,
        reason=4, snapshot=bytes.fromhex("ffff0104"), receiver_armed=False,
    )
    assert "snapshot=ffff0104" in str(snapshot_error)
    assert "receiver_armed=0" in str(snapshot_error)


def test_host_keeps_heartbeat_and_feedback_failure_streaks_separate() -> None:
    assert "self._feedback_errors = 0" in BRIDGE
    assert "self._heartbeat_errors = 0" in BRIDGE
    handler = BRIDGE[
        BRIDGE.index("def _handle_transport_error(") : BRIDGE.index(
            "def destroy_node(", BRIDGE.index("def _handle_transport_error(")
        )
    ]
    assert 'stage == "heartbeat"' in handler
    assert "self._heartbeat_errors += 1" in handler
    assert 'stage == "feedback"' in handler
    assert "self._feedback_errors += 1" in handler
    assert "except StopLatchedError as error:" in BRIDGE
    assert 'self._handle_transport_error("heartbeat", error, immediate=True)' in BRIDGE
    assert "immediate=error.stop_latched" in BRIDGE
    assert "class PositionReadError(TransportError):" in TRANSPORT
