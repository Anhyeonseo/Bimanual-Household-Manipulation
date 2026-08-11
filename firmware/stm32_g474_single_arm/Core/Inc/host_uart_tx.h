#ifndef HOST_UART_TX_H
#define HOST_UART_TX_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

/* Four complete protocol frames fit in 2.2 KiB of MCU RAM. */
#define HOST_UART_TX_QUEUE_DEPTH UINT8_C(4)

void HostUartTx_Init(UART_HandleTypeDef *host_uart);
HAL_StatusTypeDef HostUartTx_Enqueue(const uint8_t *data, uint16_t length);
uint8_t HostUartTx_TakeFault(void);
void HostUartTx_OnError(UART_HandleTypeDef *host_uart);

#endif /* HOST_UART_TX_H */
