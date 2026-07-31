#include "host_uart_rx.h"

#include <stddef.h>

#if (HOST_UART_RX_RING_CAPACITY == 0U) || \
    ((HOST_UART_RX_RING_CAPACITY & (HOST_UART_RX_RING_CAPACITY - 1U)) != 0U)
#error "Host UART RX ring capacity must be a non-zero power of two"
#endif

static UART_HandleTypeDef *host_rx_uart = NULL;
static uint8_t host_rx_interrupt_byte = 0U;
static uint8_t host_rx_ring[HOST_UART_RX_RING_CAPACITY] = {0U};
static volatile uint16_t host_rx_head = 0U;
static volatile uint16_t host_rx_tail = 0U;
static volatile uint8_t host_rx_fault = 0U;
static volatile uint8_t host_rx_started = 0U;

static uint16_t HostUartRx_Next(uint16_t index)
{
    return (uint16_t)(
        (index + 1U) & (HOST_UART_RX_RING_CAPACITY - 1U)
    );
}

static HAL_StatusTypeDef HostUartRx_Rearm(void)
{
    if (host_rx_uart == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive_IT(
        host_rx_uart,
        &host_rx_interrupt_byte,
        1U
    );
    if (status != HAL_OK)
    {
        host_rx_fault = 1U;
    }
    return status;
}

void HostUartRx_Init(UART_HandleTypeDef *host_uart)
{
    host_rx_uart = host_uart;
    host_rx_interrupt_byte = 0U;
    host_rx_head = 0U;
    host_rx_tail = 0U;
    host_rx_fault = 0U;
    host_rx_started = 0U;
}

HAL_StatusTypeDef HostUartRx_Start(void)
{
    if ((host_rx_uart == NULL) || (host_rx_started != 0U))
    {
        return HAL_ERROR;
    }

    host_rx_head = 0U;
    host_rx_tail = 0U;
    host_rx_fault = 0U;
    host_rx_started = 1U;

    if (HostUartRx_Rearm() != HAL_OK)
    {
        host_rx_started = 0U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t HostUartRx_Pop(uint8_t *byte)
{
    if ((byte == NULL) || (host_rx_tail == host_rx_head))
    {
        return 0U;
    }

    *byte = host_rx_ring[host_rx_tail];
    host_rx_tail = HostUartRx_Next(host_rx_tail);
    return 1U;
}

uint8_t HostUartRx_TakeFault(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    uint8_t fault = host_rx_fault;
    host_rx_fault = 0U;
    if (fault != 0U)
    {
        /*
         * A capacity/UART/rearm fault means the byte stream can contain a gap.
         * Discard every queued byte before the parser is allowed to run again.
         */
        host_rx_tail = host_rx_head;
    }
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return fault;
}

uint16_t HostUartRx_Count(void)
{
    return (uint16_t)(
        (host_rx_head - host_rx_tail) &
        (HOST_UART_RX_RING_CAPACITY - 1U)
    );
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != host_rx_uart) || (host_rx_started == 0U))
    {
        return;
    }

    uint16_t next = HostUartRx_Next(host_rx_head);
    if (next == host_rx_tail)
    {
        host_rx_fault = 1U;
    }
    else
    {
        host_rx_ring[host_rx_head] = host_rx_interrupt_byte;
        host_rx_head = next;
    }

    (void)HostUartRx_Rearm();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != host_rx_uart) || (host_rx_started == 0U))
    {
        return;
    }

    host_rx_fault = 1U;
    __HAL_UART_CLEAR_OREFLAG(host_rx_uart);
    __HAL_UART_SEND_REQ(host_rx_uart, UART_RXDATA_FLUSH_REQUEST);
    (void)HostUartRx_Rearm();
}
