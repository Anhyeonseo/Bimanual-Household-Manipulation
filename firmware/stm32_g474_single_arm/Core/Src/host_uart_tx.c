#include "host_uart_tx.h"

#include "actuator_core/protocol.h"
#include "servo_bus.h"

#include <string.h>

#if (HOST_UART_TX_QUEUE_DEPTH < 2U)
#error "Host UART TX queue needs an active and one pending frame"
#endif

typedef struct
{
    uint16_t length;
    uint8_t data[ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE];
} HostUartTxSlot;

static UART_HandleTypeDef *host_tx_uart = NULL;
static HostUartTxSlot host_tx_queue[HOST_UART_TX_QUEUE_DEPTH];
static volatile uint8_t host_tx_head = 0U;
static volatile uint8_t host_tx_tail = 0U;
static volatile uint8_t host_tx_count = 0U;
static volatile uint8_t host_tx_active = 0U;
static volatile uint8_t host_tx_fault = 0U;

static uint8_t HostUartTx_Next(uint8_t index)
{
    return (uint8_t)((index + 1U) % HOST_UART_TX_QUEUE_DEPTH);
}

static HAL_StatusTypeDef HostUartTx_StartLocked(void)
{
    if ((host_tx_uart == NULL) || (host_tx_count == 0U))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        host_tx_uart,
        host_tx_queue[host_tx_head].data,
        host_tx_queue[host_tx_head].length
    );
    if (status == HAL_OK)
    {
        host_tx_active = 1U;
    }
    else
    {
        host_tx_fault = 1U;
    }
    return status;
}

void HostUartTx_Init(UART_HandleTypeDef *host_uart)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    host_tx_uart = host_uart;
    host_tx_head = 0U;
    host_tx_tail = 0U;
    host_tx_count = 0U;
    host_tx_active = 0U;
    host_tx_fault = 0U;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
}

HAL_StatusTypeDef HostUartTx_Enqueue(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) ||
        (length > ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE))
    {
        return HAL_ERROR;
    }

    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if ((host_tx_uart == NULL) ||
        (host_tx_fault != 0U) ||
        (host_tx_count >= HOST_UART_TX_QUEUE_DEPTH))
    {
        host_tx_fault = 1U;
        if (interrupt_mask == 0U)
        {
            __enable_irq();
        }
        return HAL_ERROR;
    }

    HostUartTxSlot *slot = &host_tx_queue[host_tx_tail];
    memcpy(slot->data, data, length);
    slot->length = length;
    host_tx_tail = HostUartTx_Next(host_tx_tail);
    host_tx_count++;

    HAL_StatusTypeDef status = HAL_OK;
    if (host_tx_active == 0U)
    {
        status = HostUartTx_StartLocked();
    }
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return status;
}

uint8_t HostUartTx_TakeFault(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    uint8_t fault = host_tx_fault;
    host_tx_fault = 0U;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return fault;
}

void HostUartTx_OnError(UART_HandleTypeDef *host_uart)
{
    if (host_uart != host_tx_uart)
    {
        return;
    }
    host_tx_fault = 1U;
    host_tx_head = 0U;
    host_tx_tail = 0U;
    host_tx_count = 0U;
    host_tx_active = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *host_uart)
{
    /* H2.0 position reads use USART1 TX interrupt, distinct from host DMA. */
    Servo_InMotionTelemetryOnTxComplete(host_uart);

    if ((host_uart != host_tx_uart) || (host_tx_active == 0U))
    {
        return;
    }

    host_tx_head = HostUartTx_Next(host_tx_head);
    host_tx_count--;
    host_tx_active = 0U;
    if (host_tx_count != 0U)
    {
        (void)HostUartTx_StartLocked();
    }
}
