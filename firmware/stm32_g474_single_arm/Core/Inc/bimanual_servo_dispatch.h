#ifndef BIMANUAL_SERVO_DISPATCH_H
#define BIMANUAL_SERVO_DISPATCH_H

#include "actuator_core/bimanual_dispatch.h"
#include "stm32g4xx_hal.h"

#include <stdint.h>

void BimanualServoDispatch_Init(
    UART_HandleTypeDef *left_uart,
    UART_HandleTypeDef *right_uart);

HAL_StatusTypeDef BimanualServoDispatch_Launch(
    const uint16_t left_positions[6],
    const uint16_t right_positions[6],
    uint32_t control_tick_ms,
    uint32_t control_tick_started_us);

void BimanualServoDispatch_OnTxComplete(UART_HandleTypeDef *uart);
void BimanualServoDispatch_OnUartError(UART_HandleTypeDef *uart);
void BimanualServoDispatch_Stop(void);
void BimanualServoDispatch_LatchFault(void);
uint8_t BimanualServoDispatch_Faulted(void);
uint8_t BimanualServoDispatch_Ready(void);

const actuator_bimanual_dispatch_snapshot_t *
BimanualServoDispatch_GetSnapshot(void);

#endif
