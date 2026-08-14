#ifndef HOST_UART_RX_H
#define HOST_UART_RX_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

/*
 * The host link must continue receiving while the main loop is blocked in a
 * servo-bus transaction. At 115200 baud, polling a one-byte hardware register
 * cannot preserve a complete heartbeat frame during even a short servo read.
 */
#define HOST_UART_RX_RING_CAPACITY UINT16_C(1024)

void HostUartRx_Init(UART_HandleTypeDef *host_uart);
HAL_StatusTypeDef HostUartRx_Start(void);
/*
 * Pop one byte and the ISR tick at which it reached the host UART. The
 * timestamp is paired with the byte in the same ring slot so parser backlog
 * cannot turn an old frame into a fresh heartbeat.
 */
uint8_t HostUartRx_Pop(uint8_t *byte, uint32_t *received_at_ms);
uint8_t HostUartRx_TakeFault(void);
uint16_t HostUartRx_Count(void);

#endif /* HOST_UART_RX_H */
