#include "bimanual_servo_dispatch.h"

#include "actuator_core/sts3215_packet.h"
#include "timebase.h"

#include <stddef.h>

static UART_HandleTypeDef *left_bus_uart;
static UART_HandleTypeDef *right_bus_uart;
static actuator_bimanual_dispatch_t dispatch_state;
static uint8_t left_packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
static uint8_t right_packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE];
static volatile uint8_t launch_in_progress;
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
static uint8_t right_dma_fault_injection_consumed;
#endif
static const uint8_t servo_ids[6] = {1U, 2U, 3U, 4U, 5U, 6U};

void BimanualServoDispatch_Init(
    UART_HandleTypeDef *left_uart,
    UART_HandleTypeDef *right_uart)
{
    left_bus_uart = left_uart;
    right_bus_uart = right_uart;
    actuator_bimanual_dispatch_init(&dispatch_state);
    launch_in_progress = 0U;
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
    right_dma_fault_injection_consumed = 0U;
#endif
}

HAL_StatusTypeDef BimanualServoDispatch_Launch(
    const uint16_t left_positions[6],
    const uint16_t right_positions[6],
    uint32_t control_tick_ms,
    uint32_t control_tick_started_us)
{
    size_t left_length = 0U;
    size_t right_length = 0U;
    uint32_t left_start_us;
    uint32_t right_start_us;

    if ((left_bus_uart == NULL) || (right_bus_uart == NULL) ||
        (left_positions == NULL) || (right_positions == NULL) ||
        !actuator_bimanual_dispatch_can_launch(&dispatch_state))
    {
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
    if ((actuator_sts3215_build_sync_write_positions(
             servo_ids, left_positions, 6U, left_packet, &left_length) !=
         ACTUATOR_STS3215_PACKET_OK) ||
        (actuator_sts3215_build_sync_write_positions(
             servo_ids, right_positions, 6U, right_packet, &right_length) !=
         ACTUATOR_STS3215_PACKET_OK))
    {
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }

    launch_in_progress = 1U;
    left_start_us = Timebase_NowUs();
    if (HAL_UART_Transmit_DMA(
            left_bus_uart, left_packet, (uint16_t)left_length) != HAL_OK)
    {
        launch_in_progress = 0U;
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD
    {
        const actuator_bimanual_dispatch_snapshot_t *snapshot =
            actuator_bimanual_dispatch_snapshot(&dispatch_state);
        if ((right_dma_fault_injection_consumed == 0U) &&
            (snapshot != NULL) && (snapshot->completed_count >= 8U))
        {
            right_dma_fault_injection_consumed = 1U;
            launch_in_progress = 0U;
            (void)HAL_UART_AbortTransmit(left_bus_uart);
            actuator_bimanual_dispatch_fail(&dispatch_state);
            return HAL_ERROR;
        }
    }
#endif
    right_start_us = Timebase_NowUs();
    if (HAL_UART_Transmit_DMA(
            right_bus_uart, right_packet, (uint16_t)right_length) != HAL_OK)
    {
        launch_in_progress = 0U;
        (void)HAL_UART_AbortTransmit(left_bus_uart);
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
    if (actuator_bimanual_dispatch_begin(
            &dispatch_state,
            control_tick_ms,
            control_tick_started_us,
            left_start_us,
            right_start_us) != ACTUATOR_BIMANUAL_DISPATCH_OK)
    {
        launch_in_progress = 0U;
        (void)HAL_UART_AbortTransmit(left_bus_uart);
        (void)HAL_UART_AbortTransmit(right_bus_uart);
        actuator_bimanual_dispatch_fail(&dispatch_state);
        return HAL_ERROR;
    }
    launch_in_progress = 0U;
    return HAL_OK;
}

void BimanualServoDispatch_OnTxComplete(UART_HandleTypeDef *uart)
{
    const actuator_bimanual_dispatch_snapshot_t *snapshot =
        actuator_bimanual_dispatch_snapshot(&dispatch_state);
    actuator_bimanual_dispatch_result_t result;

    if ((snapshot == NULL) || !snapshot->active)
    {
        return;
    }
    if (uart == left_bus_uart)
    {
        result = actuator_bimanual_dispatch_complete_left(&dispatch_state);
    }
    else if (uart == right_bus_uart)
    {
        result = actuator_bimanual_dispatch_complete_right(&dispatch_state);
    }
    else
    {
        return;
    }
    if (result != ACTUATOR_BIMANUAL_DISPATCH_OK)
    {
        actuator_bimanual_dispatch_fail(&dispatch_state);
    }
}

void BimanualServoDispatch_OnUartError(UART_HandleTypeDef *uart)
{
    const actuator_bimanual_dispatch_snapshot_t *snapshot =
        actuator_bimanual_dispatch_snapshot(&dispatch_state);
    if (((uart == left_bus_uart) || (uart == right_bus_uart)) &&
        ((launch_in_progress != 0U) ||
         ((snapshot != NULL) && snapshot->active)))
    {
        actuator_bimanual_dispatch_fail(&dispatch_state);
    }
}

void BimanualServoDispatch_Stop(void)
{
    const actuator_bimanual_dispatch_snapshot_t *snapshot =
        actuator_bimanual_dispatch_snapshot(&dispatch_state);
    launch_in_progress = 0U;
    if (left_bus_uart != NULL)
    {
        (void)HAL_UART_AbortTransmit(left_bus_uart);
    }
    if (right_bus_uart != NULL)
    {
        (void)HAL_UART_AbortTransmit(right_bus_uart);
    }
    if ((snapshot != NULL) && snapshot->active)
    {
        actuator_bimanual_dispatch_fail(&dispatch_state);
    }
}

void BimanualServoDispatch_LatchFault(void)
{
    launch_in_progress = 0U;
    if (left_bus_uart != NULL)
    {
        (void)HAL_UART_AbortTransmit(left_bus_uart);
    }
    if (right_bus_uart != NULL)
    {
        (void)HAL_UART_AbortTransmit(right_bus_uart);
    }
    actuator_bimanual_dispatch_fail(&dispatch_state);
}

uint8_t BimanualServoDispatch_Faulted(void)
{
    const actuator_bimanual_dispatch_snapshot_t *snapshot =
        actuator_bimanual_dispatch_snapshot(&dispatch_state);
    return ((snapshot != NULL) && snapshot->faulted) ? 1U : 0U;
}

uint8_t BimanualServoDispatch_Ready(void)
{
    return actuator_bimanual_dispatch_can_launch(&dispatch_state) ? 1U : 0U;
}

const actuator_bimanual_dispatch_snapshot_t *
BimanualServoDispatch_GetSnapshot(void)
{
    return actuator_bimanual_dispatch_snapshot(&dispatch_state);
}
