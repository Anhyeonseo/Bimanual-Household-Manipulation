"""F3.0 must measure a TIM6 clock without changing servo output ownership."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")
TICK = (FIRMWARE / "Core/Src/control_tick.c").read_text(encoding="utf-8")
IRQ = (FIRMWARE / "Core/Src/stm32g4xx_it.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text(encoding="utf-8")
PROTOCOL = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/protocol.py"
).read_text(encoding="utf-8")
IDENTITY = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/hardware_identity.py"
).read_text(encoding="utf-8")


def test_f3_has_a_distinct_identity_capability_and_terminal_schema() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_F3_CONTROL_TICK_METRICS_CAPABILITY UINT32_C(0x00010000)" in CONFIG
    assert "#define HOST_F3_CONTROL_TICK_METRICS_SIZE 16U" in BINARY
    assert "SETPOINT_STATUS_F3_CONTROL_TICK_METRICS = struct.Struct(\"<4I\")" in PROTOCOL
    assert "F3 control-tick metrics capability is missing" in IDENTITY


def test_f3_reserves_tim6_for_an_exact_observation_only_five_ms_clock() -> None:
    assert "__HAL_RCC_TIM6_CLK_ENABLE();" in TICK
    assert "TIM6->PSC = 169U;" in TICK
    assert "TIM6->ARR = (CONTROL_TICK_PERIOD_US - 1U);" in TICK
    assert "TIM6->DIER = TIM_DIER_UIE;" in TICK
    assert "HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);" in TICK
    assert "void TIM6_DAC_IRQHandler(void)" in IRQ
    assert "ControlTick_OnInterrupt();" in IRQ
    assert '"${CORE_DIR}/Src/control_tick.c"' in CMAKE


def test_f3_tick_is_measurement_only_and_cannot_write_or_poll_servos() -> None:
    body = TICK[TICK.index("void ControlTick_OnInterrupt(void)"):]
    for forbidden in (
        "Servo_",
        "HAL_UART_",
        "BinaryControl_",
        "HAL_Delay",
    ):
        assert forbidden not in body
    assert "Timebase_NowUs()" in body
    assert "Timebase_ElapsedUs(started_us)" in body


def test_f3_snapshot_resets_with_each_physical_buffered_begin() -> None:
    begin = BINARY[BINARY.index("if ((begin != 0U)"):]
    assert "F0Metrics_Reset();" in begin
    assert "ControlTick_Reset();" in begin
