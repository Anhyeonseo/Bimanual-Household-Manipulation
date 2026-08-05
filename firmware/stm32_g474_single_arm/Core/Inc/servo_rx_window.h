#ifndef SERVO_RX_WINDOW_H
#define SERVO_RX_WINDOW_H

#include "servo_response_parser.h"

#include <stdint.h>

#define SERVO_RX_WINDOW_MAX_BYTES UINT16_C(64)

typedef enum
{
    SERVO_RX_WINDOW_NEED_MORE = 0,
    SERVO_RX_WINDOW_FRAME_READY = 1,
    SERVO_RX_WINDOW_STATUS_ERROR = 2,
    SERVO_RX_WINDOW_OVERFLOW = 3
} ServoRxWindowResult;

typedef struct
{
    ServoResponseParser parser;
    uint32_t next_absolute;
    uint16_t consumed_bytes;
} ServoRxWindow;

void ServoRxWindow_Init(
    ServoRxWindow *window,
    uint8_t expected_id,
    uint8_t expected_data_length,
    uint32_t start_absolute
);

ServoRxWindowResult ServoRxWindow_Consume(
    ServoRxWindow *window,
    const volatile uint8_t *ring,
    uint16_t ring_capacity,
    uint32_t producer_absolute,
    uint8_t *output,
    uint8_t output_capacity
);

uint8_t ServoRxWindow_DeadlineExpired(
    uint32_t start_tick,
    uint32_t current_tick,
    uint32_t timeout_ms
);

uint8_t ServoRxWindow_ArmPermitted(
    uint8_t line_high,
    uint8_t uart_idle,
    uint32_t stable_elapsed_ms,
    uint32_t required_stable_ms,
    uint8_t hardware_error_present
);

uint8_t ServoRxWindow_HardResyncRequired(
    uint8_t framing_error,
    uint8_t overrun_error,
    uint8_t receiver_timeout,
    uint8_t dma_error
);

uint8_t ServoRxWindow_CaptureRecent(
    const volatile uint8_t *ring,
    uint16_t ring_capacity,
    uint32_t transaction_start_absolute,
    uint32_t producer_absolute,
    uint8_t *snapshot,
    uint8_t snapshot_capacity
);

#endif /* SERVO_RX_WINDOW_H */
