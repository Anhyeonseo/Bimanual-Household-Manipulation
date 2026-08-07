#include "servo_rx_window.h"

#include <stddef.h>
#include <string.h>

void ServoRxWindow_Init(
    ServoRxWindow *window,
    uint8_t expected_id,
    uint8_t expected_data_length,
    uint32_t start_absolute
)
{
    if (window == NULL)
    {
        return;
    }

    memset(window, 0, sizeof(*window));
    ServoResponseParser_Init(
        &window->parser,
        expected_id,
        expected_data_length
    );
    window->next_absolute = start_absolute;
}

ServoRxWindowResult ServoRxWindow_Consume(
    ServoRxWindow *window,
    const volatile uint8_t *ring,
    uint16_t ring_capacity,
    uint32_t producer_absolute,
    uint8_t *output,
    uint8_t output_capacity
)
{
    if ((window == NULL) ||
        (ring == NULL) ||
        (ring_capacity == 0U) ||
        (output == NULL) ||
        (output_capacity < window->parser.expected_data_length))
    {
        return SERVO_RX_WINDOW_OVERFLOW;
    }

    uint32_t available = producer_absolute - window->next_absolute;
    if ((available > ring_capacity) ||
        ((uint32_t)window->consumed_bytes + available >
            SERVO_RX_WINDOW_MAX_BYTES))
    {
        return SERVO_RX_WINDOW_OVERFLOW;
    }

    while (available > 0U)
    {
        uint16_t index = (uint16_t)(
            window->next_absolute % ring_capacity
        );
        ServoResponseParseResult parsed = ServoResponseParser_Push(
            &window->parser,
            ring[index]
        );
        window->next_absolute++;
        window->consumed_bytes++;
        available--;

        if (parsed == SERVO_RESPONSE_FRAME_READY)
        {
            memcpy(
                output,
                ServoResponseParser_Data(&window->parser),
                window->parser.expected_data_length
            );
            return SERVO_RX_WINDOW_FRAME_READY;
        }
        if (parsed == SERVO_RESPONSE_STATUS_ERROR)
        {
            return SERVO_RX_WINDOW_STATUS_ERROR;
        }
    }

    if (window->consumed_bytes >= SERVO_RX_WINDOW_MAX_BYTES)
    {
        return SERVO_RX_WINDOW_OVERFLOW;
    }
    return SERVO_RX_WINDOW_NEED_MORE;
}

uint8_t ServoRxWindow_ArmPermitted(
    uint8_t line_high,
    uint8_t uart_idle,
    uint32_t stable_elapsed_ms,
    uint32_t required_stable_ms,
    uint8_t hardware_error_present
)
{
    return ((line_high != 0U) &&
            (uart_idle != 0U) &&
            (hardware_error_present == 0U) &&
            (stable_elapsed_ms >= required_stable_ms)) ? 1U : 0U;
}

uint8_t ServoRxWindow_HardResyncRequired(
    uint8_t framing_error,
    uint8_t overrun_error,
    uint8_t receiver_timeout,
    uint8_t dma_error
)
{
    return ((framing_error != 0U) ||
            (overrun_error != 0U) ||
            (receiver_timeout != 0U) ||
            (dma_error != 0U)) ? 1U : 0U;
}

uint8_t ServoRxWindow_CaptureRecent(
    const volatile uint8_t *ring,
    uint16_t ring_capacity,
    uint32_t transaction_start_absolute,
    uint32_t producer_absolute,
    uint8_t *snapshot,
    uint8_t snapshot_capacity
)
{
    if ((ring == NULL) ||
        (ring_capacity == 0U) ||
        (snapshot == NULL) ||
        (snapshot_capacity == 0U))
    {
        return 0U;
    }

    uint32_t available = producer_absolute - transaction_start_absolute;
    if (available > ring_capacity)
    {
        available = ring_capacity;
    }
    uint32_t count = available;
    if (count > snapshot_capacity)
    {
        count = snapshot_capacity;
    }
    uint32_t start = producer_absolute - count;
    for (uint32_t index = 0U; index < count; index++)
    {
        snapshot[index] = ring[(start + index) % ring_capacity];
    }
    return (uint8_t)count;
}

uint8_t ServoRxWindow_DeadlineExpired(
    uint32_t start_tick,
    uint32_t current_tick,
    uint32_t timeout_ms
)
{
    return ((uint32_t)(current_tick - start_tick) >= timeout_ms)
        ? 1U
        : 0U;
}
