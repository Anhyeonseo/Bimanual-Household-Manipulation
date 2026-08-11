#ifndef BINARY_CONTROL_H
#define BINARY_CONTROL_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

void BinaryControl_Init(UART_HandleTypeDef *host_uart);
void BinaryControl_Service(void);
void BinaryControl_EnterMode(void);
uint8_t BinaryControl_IsBinaryMode(void);
void BinaryControl_ProcessByte(uint8_t byte, uint32_t received_at_ms);
void BinaryControl_HandleHostUartError(void);

uint8_t BinaryControl_StopIsLatched(void);
void BinaryControl_LatchStop(void);
void BinaryControl_ClearStopLatch(void);

#endif /* BINARY_CONTROL_H */
