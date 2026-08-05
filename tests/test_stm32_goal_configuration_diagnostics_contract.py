from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Inc"
    / "single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Src"
    / "binary_control.c"
).read_text(encoding="utf-8")
PROTOCOL = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "protocol.py"
).read_text(encoding="utf-8")
IDENTITY = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "hardware_identity.py"
).read_text(encoding="utf-8")
BRIDGE = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "bridge_node.py"
).read_text(encoding="utf-8")


def test_goal_and_configuration_diagnostics_have_distinct_identity() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022A00)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x00000FFF)" in CONFIG
    assert "EXPECTED_FIRMWARE_VERSION = 0x00022A00" in IDENTITY
    assert "SERVO_COMMAND_CONFIGURATION_DIAGNOSTICS_CAPABILITY = 0x00000080" in IDENTITY
    assert "servo command/configuration diagnostics capability is missing" in IDENTITY


def test_diagnostics_read_goal_model_and_eeprom_protection_without_motion() -> None:
    assert "response.payload_length = 146U;" in BINARY
    assert "DIAGNOSTICS_BUS_HEALTH" in PROTOCOL
    assert "sizeof(identity)" in BINARY
    assert "13U,\n                 16U,\n                 &protection[0]" in BINARY
    assert "29U,\n                 11U,\n                 &protection[16]" in BINARY
    assert "13U,\n                sizeof(protection)" not in BINARY
    assert "(uint16_t)runtime[2]" in BINARY
    assert "((uint16_t)runtime[3] << 8U)" in BINARY
    assert "(uint16_t)identity[3]" in BINARY
    assert "((uint16_t)identity[4] << 8U)" in BINARY
    assert "response.payload[44] = protection[20];" in BINARY
    assert "read_status |= UINT8_C(0x08);" in BINARY
    assert "read_status |= UINT8_C(0x10);" in BINARY


def test_host_exposes_fields_needed_to_separate_command_and_load_faults() -> None:
    assert 'DIAGNOSTICS_JOINT = struct.Struct("<8B7H2B2H2BH4B")' in PROTOCOL
    for field in (
        "goal_position_raw",
        "model_number",
        "maximum_torque_limit_raw",
        "minimum_startup_force_raw",
        "protection_current_raw",
        "operating_mode",
    ):
        assert f"{field}: int" in PROTOCOL
        assert f'"{field}"' in BRIDGE
