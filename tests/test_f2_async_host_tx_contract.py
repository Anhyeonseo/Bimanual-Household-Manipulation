"""F2 must remove UART wire time from the cooperative control loop."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text(encoding="utf-8")
TX_HEADER = (FIRMWARE / "Core/Inc/host_uart_tx.h").read_text(encoding="utf-8")
TX = (FIRMWARE / "Core/Src/host_uart_tx.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
MSP = (FIRMWARE / "Core/Src/stm32g4xx_hal_msp.c").read_text(encoding="utf-8")
MAIN = (FIRMWARE / "Core/Src/main.c").read_text(encoding="utf-8")
IRQ = (FIRMWARE / "Core/Src/stm32g4xx_it.c").read_text(encoding="utf-8")
CMAKE = (FIRMWARE / "CMakeLists.txt").read_text(encoding="utf-8")


def test_f2_has_a_distinct_candidate_identity_and_dma_capability() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG
    assert "HOST_F2_ASYNC_HOST_TX_CAPABILITY UINT32_C(0x00002000)" in CONFIG


def test_f2_uses_bounded_dma_tx_and_latches_queue_or_dma_faults() -> None:
    assert "HOST_UART_TX_QUEUE_DEPTH UINT8_C(4)" in TX_HEADER
    assert "HAL_UART_Transmit_DMA(" in TX
    assert "HAL_UART_Transmit(" not in TX
    assert "host_tx_count >= HOST_UART_TX_QUEUE_DEPTH" in TX
    assert "host_tx_fault = 1U;" in TX
    assert "void HAL_UART_TxCpltCallback" in TX
    assert "HostUartTx_TakeFault()" in BINARY
    assert "ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS" in BINARY


def test_f2_links_lpuart1_tx_to_a_low_priority_dma_channel() -> None:
    assert "DMA_REQUEST_LPUART1_TX" in MSP
    assert "DMA1_Channel2" in MSP
    assert "__HAL_LINKDMA(huart, hdmatx, hdma_lpuart1_tx);" in MSP
    assert "DMA1_Channel2_IRQn, 5U" in MAIN
    assert "void DMA1_Channel2_IRQHandler(void)" in IRQ
    assert '"${CORE_DIR}/Src/host_uart_tx.c"' in CMAKE


def test_f2_restores_lateness_histogram_to_refill_acknowledgements() -> None:
    assert "diagnostics,\n            true" in BINARY
    assert "F2 validates the original 60-byte refill response" in BINARY
