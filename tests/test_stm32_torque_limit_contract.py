from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Inc"
    / "single_arm_config.h"
)
SERVO_BUS = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Src"
    / "servo_bus.c"
)


def test_payload_torque_caps_and_watchdogs_are_fixed() -> None:
    config = CONFIG.read_text(encoding="utf-8")
    assert "SERVO_SHOULDER_TORQUE_LIMIT_RAW UINT16_C(780)" in config
    assert "SERVO_ELBOW_TORQUE_LIMIT_RAW UINT16_C(650)" in config
    assert "SERVO_MOTION_LOAD_LIMIT_RAW UINT16_C(800)" in config
    assert "SERVO_MOTION_CURRENT_LIMIT_RAW UINT16_C(320)" in config
    assert "SERVO_MOTION_LIMIT_CONSECUTIVE UINT8_C(2)" in config
    assert (
        "SERVO_SHOULDER_TORQUE_LIMIT_RAW >= "
        "SERVO_MOTION_LOAD_LIMIT_RAW"
    ) in config
    assert (
        "SERVO_ELBOW_TORQUE_LIMIT_RAW >= "
        "SERVO_MOTION_LOAD_LIMIT_RAW"
    ) in config


def test_servo_table_uses_named_payload_torque_caps() -> None:
    source = SERVO_BUS.read_text(encoding="utf-8")
    assert source.count("SERVO_SHOULDER_TORQUE_LIMIT_RAW") == 1
    assert source.count("SERVO_ELBOW_TORQUE_LIMIT_RAW") == 1
    assert "\"SHOULDER\"" in source
    assert "\"ELBOW\"" in source

def test_trajectory_configuration_reads_back_torque_limit_fail_closed() -> None:
    source = SERVO_BUS.read_text(encoding="utf-8")
    configure_start = source.index(
        "HAL_StatusTypeDef Servo_ConfigureForTrajectory("
    )
    configure_end = source.index(
        "HAL_StatusTypeDef Servo_ReadAllPositions(",
        configure_start,
    )
    configure = source[configure_start:configure_end]

    assert "uint8_t torque_limit_readback[2] = {0U};" in configure
    assert "servo_id,\n            48U," in configure
    assert (
        "torque_limit_readback[0] !=\n"
        "            (uint8_t)(torque_limit & 0xFFU)"
    ) in configure
    assert (
        "torque_limit_readback[1] !=\n"
        "            (uint8_t)((torque_limit >> 8) & 0xFFU)"
    ) in configure
    assert configure.index("torque_limit_readback") > configure.index(
        "Servo_WriteData(\n            servo_id, 41U, runtime_data, 9U"
    )
    mismatch_index = configure.index("torque_limit_readback[0] !=")
    assert configure.index("return HAL_ERROR;", mismatch_index) > mismatch_index

