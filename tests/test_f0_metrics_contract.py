"""F0 timing instrumentation must remain observation-only and terminal-only."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")
TIMEBASE = (FIRMWARE / "Core/Src/timebase.c").read_text(encoding="utf-8")
METRICS = (FIRMWARE / "Core/Src/f0_metrics.c").read_text(encoding="utf-8")
MAIN = (FIRMWARE / "Core/Src/main.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
SERVO = (FIRMWARE / "Core/Src/servo_bus.c").read_text(encoding="utf-8")
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text(encoding="utf-8")
PROTOCOL = (ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/protocol.py").read_text(encoding="utf-8")


def test_f0_has_a_distinct_candidate_identity_and_capability() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_F0_METRICS_CAPABILITY UINT32_C(0x00001000)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG


def test_f0_uses_a_free_running_observation_only_microsecond_timebase() -> None:
    assert "__HAL_RCC_TIM2_CLK_ENABLE();" in TIMEBASE
    assert "TIM2->PSC = 169U;" in TIMEBASE
    assert "TIM2->ARR = UINT32_MAX;" in TIMEBASE
    assert "TIM2->CR1 = TIM_CR1_CEN;" in TIMEBASE
    assert "Timebase_Init();" in MAIN
    assert "F0Metrics_LoopBegin();" in MAIN
    assert "F0Metrics_LoopEnd();" in MAIN


def test_f0_records_only_timing_maxima_and_does_not_read_or_command_servos() -> None:
    for field in ("loop_period_max_us", "loop_work_max_us", "servo_sync_write_max_us", "host_tx_max_us"):
        assert field in METRICS
    assert "Servo_Read" not in METRICS
    assert "HAL_UART_Transmit" not in METRICS
    assert "F0Metrics_ObserveServoSyncWrite" in SERVO
    assert "F0Metrics_ObserveHostTx" in BINARY


def test_f0_metrics_remain_terminal_only_while_f2_restores_refill_lateness() -> None:
    assert "#define HOST_F0_TERMINAL_METRICS_SIZE 16U" in BINARY
    assert "if (status_code == HOST_BUFFERED_STATUS_TERMINAL)" in BINARY
    assert "payload_length += HOST_F0_TERMINAL_METRICS_SIZE;" in BINARY
    assert "(status_code == HOST_BUFFERED_STATUS_TERMINAL)" in BINARY
    assert "SETPOINT_STATUS_F0_METRICS = struct.Struct(\"<4I\")" in PROTOCOL


def test_f0_sources_are_in_the_cross_firmware_build() -> None:
    assert '"${CORE_DIR}/Src/f0_metrics.c"' in CMAKE
    assert '"${CORE_DIR}/Src/timebase.c"' in CMAKE
