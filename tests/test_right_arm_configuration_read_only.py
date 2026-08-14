"""R2 configuration snapshots must remain bounded and read-only."""

from pathlib import Path
import struct
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws/src/single_arm_bridge"
sys.path.insert(0, str(PACKAGE))

from single_arm_bridge.protocol import (  # noqa: E402
    ProtocolError,
    RIGHT_ARM_CONFIGURE_ONCE,
    RIGHT_ARM_CONFIGURATION,
    parse_right_arm_configuration,
    parse_right_arm_configure_once,
)

FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
BUS = (FIRMWARE / "Core/Src/right_servo_bus.c").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()
TRANSPORT = (PACKAGE / "single_arm_bridge/transport.py").read_text()
TOOL = (ROOT / "tools/capture_right_arm_configuration_read_only.py").read_text()


def test_configuration_wire_schema_round_trips_all_fields() -> None:
    values = (
        0, 3, 0, 0x1F, 123456,
        0, 56, 64, 0, 121, 32,
        2048, 0, 0, 0, 500, 2048, 777,
        1, 2, 1000, 20, 1, 1, 511,
        0, 20, 200, 80,
    )
    payload = RIGHT_ARM_CONFIGURATION.pack(*values)
    assert len(payload) == RIGHT_ARM_CONFIGURATION.size == 48
    snapshot = parse_right_arm_configuration(payload)
    assert snapshot.servo_id == 3
    assert snapshot.successful_block_mask == 0x1F
    assert snapshot.p_gain == 56
    assert snapshot.model_number == 777


@pytest.mark.parametrize("payload", (b"", bytes(47), bytes((0, 0)) + bytes(46)))
def test_configuration_parser_rejects_bad_wire_identity(payload: bytes) -> None:
    with pytest.raises(ProtocolError):
        parse_right_arm_configuration(payload)


def test_configure_once_wire_schema_and_torque_off_contract() -> None:
    payload = RIGHT_ARM_CONFIGURE_ONCE.pack(
        0, 2, 0, 64, 64, 0, 0, 0, 2048, 2048, 800, 900
    )
    snapshot = parse_right_arm_configure_once(payload)
    assert snapshot.status_code == 0
    assert snapshot.torque_enabled == 0
    assert snapshot.present_position_raw == snapshot.goal_position_raw == 2048
    assert snapshot.p_gain == snapshot.d_gain == 64
    assert snapshot.goal_speed_raw == 800
    assert snapshot.torque_limit_raw == 900


def test_firmware_reads_five_bounded_blocks_without_writes() -> None:
    start = BUS.index("RightServoConfigurationSnapshot RightServoBus_ReadConfiguration")
    end = BUS.index("RightServoConfigureSnapshot RightServoBus_ConfigureAtPresentPositionOnce", start)
    function = BUS[start:end]
    assert "RIGHT_SERVO_CONFIGURATION_ALL_BLOCKS_MASK" in function
    assert "uint8_t identity[5]" in function
    assert "uint8_t limits_and_gains[14]" in function
    assert "uint8_t protection[4]" in function
    assert "uint8_t command_state[10]" in function
    assert "uint8_t feedback[15]" in function
    assert "RightServo_ReadData" in function
    assert "RightServo_WriteData" not in function


def test_r2_is_versioned_gated_and_fail_closed() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x007FFFFF)" in CONFIG
    assert "UINT32_C(0x00100000)" in CONFIG
    assert "ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_REQUEST" in BINARY
    assert "ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_RESPONSE" in BINARY
    assert "read_right_arm_configuration" in TRANSPORT
    assert "host_binary_motion.active" in BINARY
    assert "Host_BufferedExecutionIsActive()" in BINARY
    assert '"write_operations": []' in TOOL
    assert '"motion_authorized": False' in TOOL
    assert 'record["successful_block_mask"] == 0x1F' in TOOL
    assert 'record["model_number"] == EXPECTED_MODEL_NUMBER' in TOOL
    assert 'record["p_gain"] == int(joint["p_gain"])' in TOOL
    assert 'record["d_gain"] == int(joint["d_gain"])' in TOOL
    assert '"torque_disabled": record["torque_enabled"] == 0' in TOOL
    assert 'if __name__ == "__main__":' in TOOL
    assert "raise SystemExit(main())" in TOOL


def test_r2_1_configuration_is_bounded_and_never_enables_torque() -> None:
    start = BUS.index(
        "RightServoConfigureSnapshot RightServoBus_ConfigureAtPresentPositionOnce"
    )
    end = BUS.index("HAL_StatusTypeDef RightServoBus_DisableTorqueAll", start)
    function = BUS[start:end]
    assert "RIGHT_SERVO_CONFIGURE_TORQUE_NOT_DISABLED" in function
    assert "snapshot.torque_enabled != 0U" in function
    assert "RightServo_WriteData(servo_id, UINT8_C(40)" in function
    assert "RightServo_WriteData(servo_id, UINT8_C(46)" in function
    assert "RightServo_WriteData(servo_id, UINT8_C(42)" not in function
    assert "torque_on" not in function
    assert "host_right_arm_output_active = 1U" in BINARY
    assert "ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_REQUEST" in BINARY
    assert "Host_RightArmOneShotPermitted" in BINARY
    bridge = (PACKAGE / "single_arm_bridge/bridge_node.py").read_text()
    assert "RIGHT_ARM_CONFIGURE_CONFIRMATION" in bridge
