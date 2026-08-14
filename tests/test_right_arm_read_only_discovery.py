"""UART4 right-arm discovery must remain read-only and isolated."""

from __future__ import annotations

from pathlib import Path
import struct
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws/src/single_arm_bridge"
sys.path.insert(0, str(PACKAGE))

from single_arm_bridge.protocol import (  # noqa: E402
    ProtocolError,
    RIGHT_ARM_DISCOVERY,
    parse_right_arm_discovery,
)


FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")
MAIN = (FIRMWARE / "Core/Src/main.c").read_text(encoding="utf-8")
MSP = (FIRMWARE / "Core/Src/stm32g4xx_hal_msp.c").read_text(encoding="utf-8")
RIGHT_BUS = (FIRMWARE / "Core/Src/right_servo_bus.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
TRANSPORT = (PACKAGE / "single_arm_bridge/transport.py").read_text(encoding="utf-8")
BRIDGE = (PACKAGE / "single_arm_bridge/bridge_node.py").read_text(encoding="utf-8")


def test_discovery_wire_schema_parses_complete_and_partial_buses() -> None:
    payload = struct.pack(
        "<BBBB6H6B2xII",
        0, 6, 0x3F, 0,
        2048, 2049, 2050, 2051, 2052, 2053,
        0, 0, 0, 0, 0, 0,
        6, 0,
    )
    assert len(payload) == RIGHT_ARM_DISCOVERY.size == 32
    snapshot = parse_right_arm_discovery(payload)
    assert snapshot.present_mask == 0x3F
    assert snapshot.positions_raw == (2048, 2049, 2050, 2051, 2052, 2053)
    assert snapshot.read_statuses == (0, 0, 0, 0, 0, 0)
    assert snapshot.transaction_count == 6
    assert snapshot.failure_count == 0

    partial = bytearray(payload)
    partial[0] = 2
    partial[2] = 0x03
    partial[16:22] = bytes((0, 0, 3, 3, 3, 3))
    assert parse_right_arm_discovery(bytes(partial)).present_mask == 0x03


@pytest.mark.parametrize("payload", (b"", bytes(31), bytes((0, 5)) + bytes(30)))
def test_discovery_parser_rejects_invalid_identity(payload: bytes) -> None:
    with pytest.raises(ProtocolError):
        parse_right_arm_discovery(payload)


def test_uart4_is_wired_for_read_only_discovery_and_not_left_motion() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_RIGHT_ARM_READ_ONLY_DISCOVERY_CAPABILITY UINT32_C(0x00020000)" in CONFIG
    assert "MX_UART4_Init();" in MAIN
    assert "RightServoBus_Init(&huart4);" in MAIN
    assert "huart4.Instance = UART4;" in MAIN
    assert "huart4.Init.BaudRate = 1000000;" in MAIN
    assert "PC10 is UART4_TX; PC11 is UART4_RX" in MSP
    assert "GPIO_PIN_10" in MSP and "GPIO_PIN_11" in MSP
    assert "GPIO_AF5_UART4" in MSP
    assert "HAL_UART_Transmit(" in RIGHT_BUS
    assert "HAL_UART_Receive(" in RIGHT_BUS
    assert "RIGHT_SERVO_READ_ADDRESS UINT8_C(56)" in RIGHT_BUS
    assert "0x02U" in RIGHT_BUS
    discovery = RIGHT_BUS[
        RIGHT_BUS.index("const RightServoDiscoverySnapshot *RightServoBus_Discover"):
        RIGHT_BUS.index("RightServoJogSnapshot RightServoBus_JogOnce")
    ]
    assert "RightServo_ReadPosition" in discovery
    assert "RightServo_WriteData" not in discovery


def test_discovery_is_capability_gated_and_blocked_during_left_motion() -> None:
    assert "ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_REQUEST" in BINARY
    assert "ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_RESPONSE" in BINARY
    discovery = BINARY[BINARY.index("static void Host_SendRightArmDiscovery"):]
    assert "Host_BufferedExecutionIsActive()" in discovery
    assert "host_binary_motion.active" in discovery
    assert "RIGHT_ARM_READ_ONLY_DISCOVERY_CAPABILITY" in TRANSPORT
    assert "discover_right_arm_read_only" in TRANSPORT
    assert '"discover_right_arm_read_only"' in BRIDGE
    assert '"commanded_operations": ["READ present_position"]' in BRIDGE
