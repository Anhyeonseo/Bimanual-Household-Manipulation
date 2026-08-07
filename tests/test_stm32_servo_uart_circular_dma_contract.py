from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
MAIN = (FIRMWARE / "Core/Src/main.c").read_text(encoding="utf-8")
MSP = (FIRMWARE / "Core/Src/stm32g4xx_hal_msp.c").read_text(encoding="utf-8")
IRQS = (FIRMWARE / "Core/Src/stm32g4xx_it.c").read_text(encoding="utf-8")
SERVO_BUS = (FIRMWARE / "Core/Src/servo_bus.c").read_text(encoding="utf-8")
APP = (FIRMWARE / "Core/Src/single_arm_app.c").read_text(encoding="utf-8")
BINARY_CONTROL = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.rindex(signature)
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


def test_dma_clock_and_channel_exist_before_uart_and_app_init() -> None:
    main_start = MAIN.index("int main(void)")
    body = MAIN[main_start : MAIN.index("void SystemClock_Config", main_start)]
    assert body.index("MX_DMA_Init();") < body.index("MX_USART1_UART_Init();")
    assert body.index("MX_USART1_UART_Init();") < body.index("SingleArmApp_Init(")
    assert "__HAL_RCC_DMA1_CLK_ENABLE();" in MAIN
    assert "__HAL_RCC_DMAMUX1_CLK_ENABLE();" in MAIN


def test_usart1_rx_uses_dedicated_circular_dma_and_idle_pullup() -> None:
    for statement in (
        "hdma_usart1_rx.Instance = DMA1_Channel1;",
        "hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;",
        "hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;",
        "hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;",
        "__HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);",
    ):
        assert statement in MSP
    rx_pin = MSP.index("GPIO_InitStruct.Pin = GPIO_PIN_5;")
    dma_init = MSP.index("hdma_usart1_rx.Instance = DMA1_Channel1;")
    assert rx_pin < MSP.index("GPIO_InitStruct.Pull = GPIO_PULLUP;", rx_pin) < dma_init
    assert "HAL_DMA_IRQHandler(&hdma_usart1_rx);" in IRQS
    assert "HAL_UART_IRQHandler(&huart1);" in IRQS


def test_host_heartbeat_irq_remains_higher_priority_than_servo_dma() -> None:
    assert "HAL_NVIC_SetPriority(LPUART1_IRQn, 1U, 0U);" in MSP
    assert "HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);" in MSP
    assert "HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2U, 0U);" in MAIN


def test_boot_leaves_servo_rx_dma_unarmed() -> None:
    init = function_body(SERVO_BUS, "void ServoBus_Init(")
    assert "ServoBus_StartCircularDma()" not in init
    assert "servo_bus_health.dma_started = 0U;" in init
    assert "servo_transaction_active = 0U;" in init
    assert "unarmed until the first transaction" in init
    app_init = APP[APP.index("void SingleArmApp_Init(") :]
    assert app_init.index("ServoBus_Init(") < app_init.index("Servo_ReadData(")


def test_transaction_lazy_arm_requires_idle_high_stability() -> None:
    idle = function_body(
        SERVO_BUS, "static uint8_t ServoBus_WaitForIdleHighStable(void)"
    )
    assert "HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)" in idle
    assert "SERVO_BUS_IDLE_HIGH_STABLE_MS" in idle
    assert "SERVO_BUS_IDLE_HIGH_TIMEOUT_MS" in idle
    assert "(uint32_t)(now - high_started)" in idle
    arm = function_body(
        SERVO_BUS, "static HAL_StatusTypeDef ServoBus_ArmReceiver(void)"
    )
    assert arm.index("ServoBus_WaitForIdleHighStable()") < arm.index(
        "ServoBus_StartCircularDma()"
    )
    prepare = function_body(
        SERVO_BUS, "static HAL_StatusTypeDef ServoBus_PrepareTransaction("
    )
    assert prepare.index("ServoBus_ArmReceiver()") < prepare.index(
        "*start_absolute = ServoBus_ProducerAbsolute();"
    )


def test_dma_is_transaction_scoped_and_disarmed_after_success() -> None:
    read_start = SERVO_BUS.rindex("HAL_StatusTypeDef Servo_ReadData(")
    write_start = SERVO_BUS.index("HAL_StatusTypeDef Servo_WriteData(", read_start)
    position_start = SERVO_BUS.index("HAL_StatusTypeDef Servo_ReadPosition(", write_start)
    read_path = SERVO_BUS[read_start:write_start]
    write_path = SERVO_BUS[write_start:position_start]
    for path in (read_path, write_path):
        assert path.index("ServoBus_PrepareTransaction(") < path.index(
            "HAL_UART_Transmit("
        )
        assert "ServoBus_DisarmReceiver()" in path
        assert "HAL_UARTEx_ReceiveToIdle_DMA(" not in path
    disarm = function_body(
        SERVO_BUS, "static HAL_StatusTypeDef ServoBus_DisarmReceiver(void)"
    )
    assert "HAL_UART_AbortReceive(servo_uart_handle)" in disarm
    assert "servo_bus_health.dma_started = 0U;" in disarm


def test_hard_resync_toggles_receiver_and_leaves_dma_unarmed() -> None:
    hard = function_body(
        SERVO_BUS, "static HAL_StatusTypeDef ServoBus_HardResyncReceiver(void)"
    )
    assert "HAL_UART_AbortReceive(servo_uart_handle)" in hard
    assert "ATOMIC_CLEAR_BIT(servo_uart_handle->Instance->CR1, USART_CR1_RE)" in hard
    assert "USART_ISR_REACK" in hard
    assert "ATOMIC_SET_BIT(servo_uart_handle->Instance->CR1, USART_CR1_RE)" in hard
    assert "ServoBus_StartCircularDma()" not in hard
    recovery = function_body(SERVO_BUS, "static HAL_StatusTypeDef ServoBus_Recover(")
    assert recovery.index("ServoBus_CaptureFailureSnapshot();") < recovery.index(
        "ServoBus_HardResyncReceiver();"
    )


def test_failure_snapshot_is_captured_before_ring_clear() -> None:
    snapshot = function_body(
        SERVO_BUS, "static void ServoBus_CaptureFailureSnapshot(void)"
    )
    assert "SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES" in snapshot
    assert "servo_transaction_start_absolute" in snapshot
    assert "ServoRxWindow_CaptureRecent" in snapshot
    assert "servo_bus_diagnostics.snapshot" in snapshot
    recovery = function_body(SERVO_BUS, "static HAL_StatusTypeDef ServoBus_Recover(")
    assert recovery.index("ServoBus_CaptureFailureSnapshot();") < recovery.index(
        "ServoBus_HardResyncReceiver();"
    )


def test_hal_error_abort_is_disabled_and_dma_gate_checks_hardware() -> None:
    policy = function_body(
        SERVO_BUS, "static void ServoBus_DisableHalErrorAbort(void)"
    )
    assert "USART_CR1_PEIE | USART_CR1_RTOIE" in policy
    assert "USART_CR3_EIE" in policy
    gate = function_body(
        SERVO_BUS, "static uint8_t ServoBus_DmaHardwareActive(void)"
    )
    for condition in (
        "servo_bus_health.dma_started == 0U",
        "servo_uart_handle->RxState == HAL_UART_STATE_BUSY_RX",
        "USART_CR3_DMAR",
        "DMA_CCR_EN",
    ):
        assert condition in gate


def test_error_policy_hard_resyncs_fe_ore_rto_and_dma() -> None:
    read_start = SERVO_BUS.rindex("HAL_StatusTypeDef Servo_ReadData(")
    write_start = SERVO_BUS.index("HAL_StatusTypeDef Servo_WriteData(", read_start)
    read_path = SERVO_BUS[read_start:write_start]
    assert "NE/PE bytes remain checksum-gated" in read_path
    assert "ServoRxWindow_HardResyncRequired(" in read_path
    assert "HAL_UART_ERROR_FE" in read_path
    assert "HAL_UART_ERROR_ORE" in read_path
    assert "HAL_UART_ERROR_RTO" in read_path
    assert "SERVO_BUS_FAILURE_DMA" in read_path
    assert "(void)ServoBus_Recover();" in read_path
    assert "ServoRxWindow_Consume(" in read_path


def test_receive_to_idle_dma_has_one_controlled_start_site() -> None:
    assert SERVO_BUS.count("HAL_UARTEx_ReceiveToIdle_DMA(") == 1
    start = function_body(
        SERVO_BUS, "static HAL_StatusTypeDef ServoBus_StartCircularDma(void)"
    )
    assert "HAL_UARTEx_ReceiveToIdle_DMA(" in start
    assert "ServoBus_DisableHalErrorAbort();" in start


def test_buffered_setpoint_hot_path_never_opens_an_rx_transaction() -> None:
    """
    The 5 ms buffered output path must stay TX-only.

    ServoBus_PrepareTransaction can block up to SERVO_BUS_IDLE_HIGH_TIMEOUT_MS
    waiting for idle-high, and ServoBus_Recover calls HAL_Delay while it hard
    resyncs the receiver.  Either inside Servo_SyncWritePositions would stall
    the 1 ms executor and trip the 5 ms apply-lateness gate, so the sync-write
    packet is transmitted without arming, recovering, or delaying.
    """
    sync_write = function_body(
        SERVO_BUS, "HAL_StatusTypeDef Servo_SyncWritePositions("
    )
    for blocking_call in (
        "ServoBus_PrepareTransaction(",
        "ServoBus_Recover(",
        "ServoBus_ArmReceiver(",
        "ServoBus_DisarmReceiver(",
        "ServoBus_HardResyncReceiver(",
        "ServoBus_WaitForIdleHighStable(",
        "HAL_Delay(",
    ):
        assert blocking_call not in sync_write
    assert "HAL_UART_Transmit(" in sync_write
    executor = function_body(
        BINARY_CONTROL, "static void Host_ServiceBufferedExecution(void)"
    )
    assert "Servo_SyncWritePositions(" in executor
    assert "Servo_ReadData(" not in executor
    assert "Servo_ReadPosition(" not in executor


def test_servo_reads_are_refused_while_buffered_execution_is_active() -> None:
    """
    Every Servo_ReadData call site sits behind the buffered-execution gate, so
    a failing read can never invoke ServoBus_Recover during a buffered Action.
    """
    diagnostics = function_body(
        BINARY_CONTROL, "static void Host_SendBinaryDiagnostics("
    )
    assert diagnostics.index(
        "Host_BufferedExecutionIsActive() != 0U"
    ) < diagnostics.index("Servo_ReadData(")
    # Host_SendBinaryDiagnostics is the sole owner of the servo read path.
    assert BINARY_CONTROL.count("Servo_ReadData(") == diagnostics.count(
        "Servo_ReadData("
    )
