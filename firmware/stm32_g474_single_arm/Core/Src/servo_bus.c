#include "servo_bus.h"
#include "actuator_core/sts3215_packet.h"
#include "f0_metrics.h"
#include "timebase.h"
#include "servo_response_parser.h"
#include "servo_rx_window.h"
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
#include "right_servo_bus.h"
#endif

#include <stddef.h>
#include <string.h>

#define SERVO_BUS_READ_TIMEOUT_MS UINT32_C(50)
#define SERVO_BUS_READ_TX_TIMEOUT_MS UINT32_C(5)
#define SERVO_BUS_DMA_RING_CAPACITY UINT16_C(256)
#define SERVO_BUS_RECOVERY_QUIET_MS UINT32_C(2)
#define SERVO_BUS_PREFLIGHT_IDLE_TIMEOUT_MS UINT32_C(2)
#define SERVO_BUS_IDLE_HIGH_STABLE_MS UINT32_C(2)
#define SERVO_BUS_IDLE_HIGH_TIMEOUT_MS UINT32_C(20)
#define SERVO_BUS_RECEIVER_ACK_TIMEOUT_MS UINT32_C(2)
#define SERVO_BUS_WRITE_REPLY_SETTLE_MS UINT32_C(2)
/*
 * 2026-08-12 H2.0 physical evidence recorded one clean-recovery telemetry
 * timeout at 2 ms, with no UART/DMA fault.  Four millisecond ticks leave one
 * complete tick before the fixed next 5 ms sync-write slot.  The scheduler
 * still aborts rather than delaying that slot if the reply is pending.
 */
#define SERVO_IN_MOTION_TELEMETRY_TIMEOUT_MS UINT32_C(4)

static UART_HandleTypeDef *servo_uart_handle = NULL;
static ServoStopRequestedFn servo_stop_requested = NULL;
static ServoReadFailureFn servo_read_failure = NULL;
static volatile uint8_t servo_dma_rx_ring[SERVO_BUS_DMA_RING_CAPACITY]
    __attribute__((aligned(4))) = {0U};
static volatile uint32_t servo_dma_wrap_count = 0U;
static volatile uint32_t servo_uart_async_errors = 0U;
static volatile uint32_t servo_dma_async_error = 0U;
static uint32_t servo_transaction_start_absolute = 0U;
static uint8_t servo_transaction_active = 0U;
static ServoBusDiagnostics servo_bus_diagnostics = {0};
static ServoBusHealth servo_bus_health = {0};

typedef enum
{
    SERVO_IN_MOTION_TELEMETRY_IDLE = 0,
    SERVO_IN_MOTION_TELEMETRY_TX_PENDING = 1,
    SERVO_IN_MOTION_TELEMETRY_WAIT_REPLY = 2
} ServoInMotionTelemetryState;

typedef struct
{
    ServoInMotionTelemetryState state;
    uint8_t enabled;
    uint8_t joint_index;
    uint8_t servo_id;
    uint8_t tx_completed;
    uint32_t started_at_ms;
    uint8_t request[8];
    ServoRxWindow window;
} ServoInMotionTelemetry;

static ServoInMotionTelemetry servo_in_motion_telemetry = {0};
static ServoInMotionTelemetrySnapshot servo_in_motion_snapshot = {0};

static uint32_t ServoBus_CurrentUartErrorCode(void);
static void ServoBus_ClearHardwareRxState(void);
static HAL_StatusTypeDef ServoBus_HardResyncReceiver(void);
static HAL_StatusTypeDef ServoBus_ArmReceiver(void);
static HAL_StatusTypeDef ServoBus_DisarmReceiver(void);
static void ServoBus_CaptureFailureSnapshot(void);
static HAL_StatusTypeDef ServoBus_Recover(void);
static void ServoBus_RecordFailure(
    ServoBusFailureReason reason,
    uint8_t servo_id,
    HAL_StatusTypeDef hal_status,
    uint8_t servo_status,
    uint16_t discarded_bytes
);
static HAL_StatusTypeDef ServoInMotionTelemetry_Fail(
    ServoBusFailureReason reason,
    HAL_StatusTypeDef status,
    uint8_t servo_status,
    uint16_t discarded_bytes
);

/*
 * HAL_UART_IRQHandler treats every UART error as blocking while DMAR is set
 * and aborts even a circular receive. The servo bus polls and clears error
 * flags at transaction boundaries so a power-domain edge cannot silently
 * kill an active transaction ring.
 */
static void ServoBus_DisableHalErrorAbort(void)
{
    if (servo_uart_handle == NULL)
    {
        return;
    }

    ATOMIC_CLEAR_BIT(
        servo_uart_handle->Instance->CR1,
        USART_CR1_PEIE | USART_CR1_RTOIE
    );
    ATOMIC_CLEAR_BIT(
        servo_uart_handle->Instance->CR3,
        USART_CR3_EIE
    );
}

static HAL_StatusTypeDef ServoBus_StartCircularDma(void)
{
    if ((servo_uart_handle == NULL) ||
        (servo_uart_handle->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        servo_uart_handle,
        (uint8_t *)servo_dma_rx_ring,
        SERVO_BUS_DMA_RING_CAPACITY
    );
    if (status != HAL_OK)
    {
        servo_bus_health.dma_started = 0U;
        return status;
    }

    ServoBus_DisableHalErrorAbort();
    servo_bus_health.dma_started = 1U;
    return HAL_OK;
}

static uint8_t ServoBus_DmaHardwareActive(void)
{
    if ((servo_uart_handle == NULL) ||
        (servo_uart_handle->hdmarx == NULL) ||
        (servo_bus_health.dma_started == 0U))
    {
        return 0U;
    }

    return ((servo_uart_handle->RxState == HAL_UART_STATE_BUSY_RX) &&
            (HAL_IS_BIT_SET(
                servo_uart_handle->Instance->CR3,
                USART_CR3_DMAR
            )) &&
            (HAL_IS_BIT_SET(
                servo_uart_handle->hdmarx->Instance->CCR,
                DMA_CCR_EN
            ))) ? 1U : 0U;
}

static void ServoBus_IncrementU16(uint16_t *value)
{
    if ((value != NULL) && (*value < UINT16_MAX))
    {
        (*value)++;
    }
}

static uint8_t servo_motion_safety_mask = 0U;
static uint32_t servo_motion_safety_last_sample_ms = 0U;
static uint8_t servo_motion_safety_next_joint = 0U;
static uint8_t servo_load_limit_counts[SINGLE_ARM_JOINT_COUNT] = {0U};
static uint8_t servo_current_limit_counts[SINGLE_ARM_JOINT_COUNT] = {0U};
static ServoMotionSafetyDiagnostics servo_motion_safety_diagnostics = {
    SERVO_MOTION_SAFETY_NONE, 0U, 0U, 0U, 0U, 0U
};

const ServoJointConfig servo_joints[SINGLE_ARM_JOINT_COUNT] = {
    {1U, "BASE",        1U, 2048U, 1988U, 2610U, 16U, 32U,  1,  34U,  600U, 400U},
    {2U, "SHOULDER",    1U, 2048U, 1988U, 3766U, 64U, 64U,  1,  34U, 1200U,
        SERVO_SHOULDER_TORQUE_LIMIT_RAW},
    {3U, "ELBOW",       1U, 2048U, 627U, 2258U, 56U, 64U, -1,  34U, 1000U,
        SERVO_ELBOW_TORQUE_LIMIT_RAW},
    {4U, "WRIST_FLEX",  1U, 2048U, 1194U, 2108U, 16U, 32U, -1,  34U,  800U, 400U},
    {5U, "WRIST_ROLL",  1U, 2048U, 1874U, 2219U, 16U, 32U,  1,  34U,  500U, 250U},
    {6U, "GRIPPER",     1U, 2048U, 1866U, 2048U, 16U, 32U, -1,  34U,  800U, 150U}
};

const uint8_t servo_joint_count = SINGLE_ARM_JOINT_COUNT;
uint8_t servo_last_all_read_failed_id = 0U;

void ServoBus_Init(
    UART_HandleTypeDef *servo_uart,
    ServoStopRequestedFn stop_requested,
    ServoReadFailureFn read_failure
)
{
    servo_uart_handle = servo_uart;
    servo_stop_requested = stop_requested;
    servo_read_failure = read_failure;
    servo_last_all_read_failed_id = 0U;
    memset(&servo_bus_diagnostics, 0, sizeof(servo_bus_diagnostics));
    memset(&servo_bus_health, 0, sizeof(servo_bus_health));
    memset((void *)servo_dma_rx_ring, 0, sizeof(servo_dma_rx_ring));
    servo_dma_wrap_count = 0U;
    servo_uart_async_errors = 0U;
    servo_dma_async_error = 0U;

    /*
     * The external servo adapter lives in the switched 12 V domain. Leaving
     * DMA armed while that domain is off captures the power edge as a partial
     * response and can poison the USART receiver until an MCU reset. Keep RX
     * unarmed until the first transaction observes a stable idle-high line.
     */
    ServoBus_ClearHardwareRxState();
    ServoBus_DisableHalErrorAbort();
    servo_bus_health.dma_started = 0U;
    servo_transaction_start_absolute = 0U;
    servo_transaction_active = 0U;
    Servo_MotionSafetyEnd();
    memset(
        &servo_motion_safety_diagnostics,
        0,
        sizeof(servo_motion_safety_diagnostics)
    );
    memset(&servo_in_motion_telemetry, 0, sizeof(servo_in_motion_telemetry));
    memset(&servo_in_motion_snapshot, 0, sizeof(servo_in_motion_snapshot));
}

const ServoBusDiagnostics *ServoBus_GetDiagnostics(void)
{
    return &servo_bus_diagnostics;
}

const ServoBusHealth *ServoBus_GetHealth(void)
{
    return &servo_bus_health;
}

static void ServoBus_CountUartErrors(uint32_t errors)
{
    if ((errors & HAL_UART_ERROR_PE) != 0U)
    {
        ServoBus_IncrementU16(&servo_bus_health.pe_count);
    }
    if ((errors & HAL_UART_ERROR_NE) != 0U)
    {
        ServoBus_IncrementU16(&servo_bus_health.ne_count);
    }
    if ((errors & HAL_UART_ERROR_FE) != 0U)
    {
        ServoBus_IncrementU16(&servo_bus_health.fe_count);
    }
    if ((errors & HAL_UART_ERROR_ORE) != 0U)
    {
        ServoBus_IncrementU16(&servo_bus_health.ore_count);
    }
    if ((errors & HAL_UART_ERROR_RTO) != 0U)
    {
        ServoBus_IncrementU16(&servo_bus_health.rto_count);
    }
}

void ServoBus_HandleUartError(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart != servo_uart_handle))
    {
        return;
    }

    uint32_t errors = ServoBus_CurrentUartErrorCode();
    servo_uart_async_errors |= errors;
    ServoBus_CountUartErrors(errors);
    /* HAL delivers this callback after aborting DMA-mode reception. */
    servo_bus_health.dma_started = 0U;
    if (uart->hdmarx != NULL)
    {
        uint32_t dma_error = uart->hdmarx->ErrorCode;
        servo_dma_async_error |= dma_error;
        if (dma_error != HAL_DMA_ERROR_NONE)
        {
            ServoBus_IncrementU16(&servo_bus_health.dma_error_count);
        }
    }
}

void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *uart,
    uint16_t received
)
{
    (void)received;
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    RightServoBus_InMotionTelemetryOnRxEvent(uart, received);
#endif
    if ((uart == NULL) || (uart != servo_uart_handle))
    {
        return;
    }

    HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(uart);
    servo_bus_health.last_rx_event = (uint8_t)event;
    servo_bus_health.rx_event_count++;
    if (event == HAL_UART_RXEVENT_TC)
    {
        servo_dma_wrap_count++;
    }
}

static void ServoBus_ClearHardwareRxState(void)
{
    if (servo_uart_handle == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_FLAG(
        servo_uart_handle,
        UART_CLEAR_OREF |
        UART_CLEAR_NEF |
        UART_CLEAR_PEF |
        UART_CLEAR_FEF |
        UART_CLEAR_RTOF
    );
    __HAL_UART_SEND_REQ(
        servo_uart_handle,
        UART_RXDATA_FLUSH_REQUEST
    );
}

static uint32_t ServoBus_ProducerAbsolute(void)
{
    if ((servo_uart_handle == NULL) ||
        (servo_uart_handle->hdmarx == NULL) ||
        (ServoBus_DmaHardwareActive() == 0U))
    {
        return 0U;
    }

    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    uint32_t wraps = servo_dma_wrap_count;
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(
        servo_uart_handle->hdmarx
    );
    uint32_t pending_tc = __HAL_DMA_GET_FLAG(
        servo_uart_handle->hdmarx,
        __HAL_DMA_GET_TC_FLAG_INDEX(servo_uart_handle->hdmarx)
    );
    if ((pending_tc != 0U) && (remaining != 0U))
    {
        wraps++;
    }
    uint32_t absolute =
        (wraps * SERVO_BUS_DMA_RING_CAPACITY) +
        (SERVO_BUS_DMA_RING_CAPACITY - remaining);
    __DMB();
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    servo_bus_health.producer_index = (uint16_t)(
        absolute % SERVO_BUS_DMA_RING_CAPACITY
    );
    return absolute;
}

static uint32_t ServoBus_CurrentUartErrorCode(void)
{
    if (servo_uart_handle == NULL)
    {
        return HAL_UART_ERROR_NONE;
    }

    uint32_t errors = servo_uart_handle->ErrorCode;
    uint32_t isr = servo_uart_handle->Instance->ISR;

    if ((isr & UART_FLAG_PE) != 0U)
    {
        errors |= HAL_UART_ERROR_PE;
    }
    if ((isr & UART_FLAG_NE) != 0U)
    {
        errors |= HAL_UART_ERROR_NE;
    }
    if ((isr & UART_FLAG_FE) != 0U)
    {
        errors |= HAL_UART_ERROR_FE;
    }
    if ((isr & UART_FLAG_ORE) != 0U)
    {
        errors |= HAL_UART_ERROR_ORE;
    }
    if ((isr & UART_FLAG_RTOF) != 0U)
    {
        errors |= HAL_UART_ERROR_RTO;
    }

    return errors;
}

static uint8_t ServoBus_WaitForIdleHighStable(void)
{
    if (servo_uart_handle == NULL)
    {
        return 0U;
    }

    uint32_t wait_started = HAL_GetTick();
    uint32_t high_started = wait_started;
    uint8_t high_active = 0U;

    while ((uint32_t)(HAL_GetTick() - wait_started) <
           SERVO_BUS_IDLE_HIGH_TIMEOUT_MS)
    {
        uint32_t now = HAL_GetTick();
        uint8_t line_high = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) ==
                             GPIO_PIN_SET) ? 1U : 0U;
        uint8_t uart_idle = (__HAL_UART_GET_FLAG(
            servo_uart_handle, UART_FLAG_BUSY
        ) == RESET) ? 1U : 0U;

        uint8_t hardware_error_present = (
            (ServoBus_CurrentUartErrorCode() != HAL_UART_ERROR_NONE) ||
            ((servo_uart_handle->hdmarx != NULL) &&
             (servo_uart_handle->hdmarx->ErrorCode != HAL_DMA_ERROR_NONE))
        ) ? 1U : 0U;
        if ((line_high != 0U) &&
            (uart_idle != 0U) &&
            (hardware_error_present == 0U))
        {
            if (high_active == 0U)
            {
                high_active = 1U;
                high_started = now;
            }
            if (ServoRxWindow_ArmPermitted(
                    line_high,
                    uart_idle,
                    (uint32_t)(now - high_started),
                    SERVO_BUS_IDLE_HIGH_STABLE_MS,
                    hardware_error_present
                ) != 0U)
            {
                return 1U;
            }
        }
        else
        {
            high_active = 0U;
            high_started = now;
        }
    }

    return 0U;
}

static HAL_StatusTypeDef ServoBus_HardResyncReceiver(void)
{
    if ((servo_uart_handle == NULL) ||
        (servo_uart_handle->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_AbortReceive(servo_uart_handle);
    servo_bus_health.dma_started = 0U;
    if (status != HAL_OK)
    {
        return status;
    }

    ATOMIC_CLEAR_BIT(servo_uart_handle->Instance->CR1, USART_CR1_RE);
    uint32_t ack_started = HAL_GetTick();
    while (HAL_IS_BIT_SET(
        servo_uart_handle->Instance->ISR, USART_ISR_REACK
    ))
    {
        if ((uint32_t)(HAL_GetTick() - ack_started) >=
            SERVO_BUS_RECEIVER_ACK_TIMEOUT_MS)
        {
            return HAL_TIMEOUT;
        }
    }

    ServoBus_ClearHardwareRxState();
    HAL_Delay(SERVO_BUS_RECOVERY_QUIET_MS);
    ServoBus_ClearHardwareRxState();

    ATOMIC_SET_BIT(servo_uart_handle->Instance->CR1, USART_CR1_RE);
    ack_started = HAL_GetTick();
    while (!HAL_IS_BIT_SET(
        servo_uart_handle->Instance->ISR, USART_ISR_REACK
    ))
    {
        if ((uint32_t)(HAL_GetTick() - ack_started) >=
            SERVO_BUS_RECEIVER_ACK_TIMEOUT_MS)
        {
            return HAL_TIMEOUT;
        }
    }

    ServoBus_ClearHardwareRxState();
    servo_uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
    servo_uart_handle->hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
    servo_uart_async_errors = 0U;
    servo_dma_async_error = 0U;
    servo_bus_health.receiver_resync_count++;
    return HAL_OK;
}

static HAL_StatusTypeDef ServoBus_ArmReceiver(void)
{
    if ((servo_uart_handle == NULL) ||
        (servo_uart_handle->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_AbortReceive(servo_uart_handle);
    servo_bus_health.dma_started = 0U;
    if (status != HAL_OK)
    {
        return status;
    }
    ServoBus_ClearHardwareRxState();
    if (ServoBus_WaitForIdleHighStable() == 0U)
    {
        return HAL_TIMEOUT;
    }

    memset((void *)servo_dma_rx_ring, 0, sizeof(servo_dma_rx_ring));
    servo_dma_wrap_count = 0U;
    servo_uart_async_errors = 0U;
    servo_dma_async_error = 0U;
    servo_uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
    servo_uart_handle->hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;

    status = ServoBus_StartCircularDma();
    if (status == HAL_OK)
    {
        servo_bus_health.lazy_arm_count++;
    }
    return status;
}

static HAL_StatusTypeDef ServoBus_DisarmReceiver(void)
{
    if (servo_uart_handle == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_AbortReceive(servo_uart_handle);
    servo_bus_health.dma_started = 0U;
    servo_transaction_active = 0U;
    ServoBus_ClearHardwareRxState();
    servo_uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
    servo_uart_async_errors = 0U;
    servo_dma_async_error = 0U;
    return status;
}

static void ServoBus_CaptureFailureSnapshot(void)
{
    servo_bus_diagnostics.snapshot_length = 0U;
    memset(
        servo_bus_diagnostics.snapshot,
        0,
        sizeof(servo_bus_diagnostics.snapshot)
    );

    if ((servo_transaction_active == 0U) ||
        (ServoBus_DmaHardwareActive() == 0U))
    {
        return;
    }

    servo_bus_diagnostics.snapshot_length = ServoRxWindow_CaptureRecent(
        servo_dma_rx_ring,
        SERVO_BUS_DMA_RING_CAPACITY,
        servo_transaction_start_absolute,
        ServoBus_ProducerAbsolute(),
        servo_bus_diagnostics.snapshot,
        SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES
    );
}

static HAL_StatusTypeDef ServoBus_PrepareTransaction(
    uint8_t servo_id,
    uint32_t *start_absolute
)
{
    if ((servo_uart_handle == NULL) || (start_absolute == NULL))
    {
        return HAL_ERROR;
    }

    memset(&servo_bus_diagnostics, 0, sizeof(servo_bus_diagnostics));
    servo_bus_diagnostics.servo_id = servo_id;
    servo_bus_diagnostics.recovery_count = servo_bus_health.recovery_count;
    servo_bus_health.transaction_count++;
    servo_transaction_active = 0U;

    uint32_t callback_errors = servo_uart_async_errors;
    uint32_t polled_errors = ServoBus_CurrentUartErrorCode();
    uint32_t uart_errors = callback_errors | polled_errors;
    uint32_t dma_error = servo_dma_async_error;
    if (servo_uart_handle->hdmarx != NULL)
    {
        dma_error |= servo_uart_handle->hdmarx->ErrorCode;
    }

    uint8_t receiver_active = ServoBus_DmaHardwareActive();
    if ((uart_errors != HAL_UART_ERROR_NONE) ||
        (dma_error != HAL_DMA_ERROR_NONE) ||
        ((receiver_active != 0U) && (__HAL_UART_GET_FLAG(
            servo_uart_handle, UART_FLAG_BUSY
        ) != RESET)))
    {
        ServoBus_CountUartErrors(polled_errors & ~callback_errors);
        if (dma_error != HAL_DMA_ERROR_NONE)
        {
            ServoBus_IncrementU16(&servo_bus_health.dma_error_count);
        }
        if (ServoBus_Recover() != HAL_OK)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_RECOVERY, servo_id, HAL_ERROR, 0U, 0U
            );
            return HAL_ERROR;
        }
    }
    if (ServoBus_DmaHardwareActive() == 0U)
    {
        if (ServoBus_ArmReceiver() != HAL_OK)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_RECOVERY, servo_id, HAL_TIMEOUT, 0U, 0U
            );
            return HAL_ERROR;
        }
    }

    uint32_t idle_started = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(
        servo_uart_handle, UART_FLAG_BUSY
    ) != RESET)
    {
        if ((uint32_t)(HAL_GetTick() - idle_started) >=
            SERVO_BUS_PREFLIGHT_IDLE_TIMEOUT_MS)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_UART, servo_id, HAL_BUSY, 0U, 0U
            );
            (void)ServoBus_Recover();
            return HAL_BUSY;
        }
    }

    if (ServoBus_DmaHardwareActive() == 0U)
    {
        ServoBus_RecordFailure(
            SERVO_BUS_FAILURE_DMA, servo_id, HAL_ERROR, 0U, 0U
        );
        (void)ServoBus_Recover();
        return HAL_ERROR;
    }

    *start_absolute = ServoBus_ProducerAbsolute();
    servo_transaction_start_absolute = *start_absolute;
    servo_transaction_active = 1U;
    return HAL_OK;
}

static void ServoBus_RecordFailure(
    ServoBusFailureReason reason,
    uint8_t servo_id,
    HAL_StatusTypeDef hal_status,
    uint8_t servo_status,
    uint16_t discarded_bytes
)
{
    servo_bus_diagnostics.reason = reason;
    servo_bus_diagnostics.servo_id = servo_id;
    servo_bus_diagnostics.hal_status = (uint8_t)hal_status;
    servo_bus_diagnostics.servo_status = servo_status;
    servo_bus_diagnostics.discarded_bytes = discarded_bytes;
    servo_bus_health.failure_count++;
    servo_bus_health.discarded_bytes += discarded_bytes;

    if (servo_uart_handle != NULL)
    {
        servo_bus_diagnostics.uart_error_code =
            servo_uart_handle->ErrorCode;
        servo_bus_diagnostics.uart_isr =
            servo_uart_handle->Instance->ISR;
        if (servo_uart_handle->hdmarx != NULL)
        {
            servo_bus_diagnostics.dma_error_code =
                servo_uart_handle->hdmarx->ErrorCode;
        }
    }
}

static HAL_StatusTypeDef ServoBus_Recover(void)
{
    if (servo_uart_handle == NULL)
    {
        servo_bus_diagnostics.reason = SERVO_BUS_FAILURE_RECOVERY;
        return HAL_ERROR;
    }

    /* Preserve the last transaction bytes before abort/reset clears the ring. */
    ServoBus_CaptureFailureSnapshot();
    servo_transaction_active = 0U;
    servo_bus_health.recovery_count++;
    servo_bus_diagnostics.recovery_count = servo_bus_health.recovery_count;

    HAL_StatusTypeDef status = ServoBus_HardResyncReceiver();
    if (status != HAL_OK)
    {
        servo_bus_diagnostics.reason = SERVO_BUS_FAILURE_RECOVERY;
        servo_bus_diagnostics.hal_status = (uint8_t)status;
        if (servo_uart_handle->hdmarx != NULL)
        {
            servo_bus_diagnostics.dma_error_code =
                servo_uart_handle->hdmarx->ErrorCode;
        }
        return status;
    }
    return HAL_OK;
}

static ServoBusFailureReason ServoBus_MapRejectReason(
    ServoResponseRejectReason reason
)
{
    switch (reason)
    {
        case SERVO_RESPONSE_REJECT_HEADER:
            return SERVO_BUS_FAILURE_HEADER;
        case SERVO_RESPONSE_REJECT_ID:
            return SERVO_BUS_FAILURE_ID;
        case SERVO_RESPONSE_REJECT_LENGTH:
            return SERVO_BUS_FAILURE_LENGTH;
        case SERVO_RESPONSE_REJECT_CHECKSUM:
            return SERVO_BUS_FAILURE_CHECKSUM;
        case SERVO_RESPONSE_REJECT_NONE:
        default:
            return SERVO_BUS_FAILURE_RX_TIMEOUT;
    }
}

static uint8_t Servo_Checksum(
    const uint8_t *packet,
    uint8_t last_index
)
{
    uint8_t sum = 0U;

    for (uint8_t i = 2U; i <= last_index; i++)
    {
        sum = (uint8_t)(sum + packet[i]);
    }

    return (uint8_t)(~sum);
}

void Servo_InMotionTelemetryBegin(void)
{
    memset(&servo_in_motion_telemetry, 0, sizeof(servo_in_motion_telemetry));
    memset(&servo_in_motion_snapshot, 0, sizeof(servo_in_motion_snapshot));
    servo_in_motion_telemetry.enabled = 1U;
}

void Servo_InMotionTelemetryEnd(void)
{
    if (servo_in_motion_telemetry.enabled != 0U)
    {
        if (servo_uart_handle != NULL)
        {
            (void)HAL_UART_AbortTransmit(servo_uart_handle);
        }
        if (ServoBus_DmaHardwareActive() != 0U)
        {
            (void)ServoBus_DisarmReceiver();
        }
    }
    memset(&servo_in_motion_telemetry, 0, sizeof(servo_in_motion_telemetry));
}

uint8_t Servo_InMotionTelemetryPending(void)
{
    return (servo_in_motion_telemetry.state !=
            SERVO_IN_MOTION_TELEMETRY_IDLE) ? 1U : 0U;
}

const ServoInMotionTelemetrySnapshot *
Servo_InMotionTelemetryGetSnapshot(void)
{
    return &servo_in_motion_snapshot;
}

HAL_StatusTypeDef Servo_InMotionTelemetryStart(
    uint8_t joint_index,
    uint32_t started_at_ms
)
{
    if ((servo_in_motion_telemetry.enabled == 0U) ||
        (servo_in_motion_telemetry.state !=
            SERVO_IN_MOTION_TELEMETRY_IDLE) ||
        (joint_index >= servo_joint_count))
    {
        return HAL_BUSY;
    }

    const uint8_t servo_id = servo_joints[joint_index].id;
    uint32_t transaction_start_absolute = 0U;
    if (ServoBus_PrepareTransaction(servo_id, &transaction_start_absolute) !=
        HAL_OK)
    {
        servo_in_motion_snapshot.failed_samples++;
        return HAL_ERROR;
    }

    ServoInMotionTelemetry *telemetry = &servo_in_motion_telemetry;
    telemetry->joint_index = joint_index;
    telemetry->servo_id = servo_id;
    telemetry->started_at_ms = started_at_ms;
    telemetry->tx_completed = 0U;
    telemetry->request[0] = 0xFFU;
    telemetry->request[1] = 0xFFU;
    telemetry->request[2] = servo_id;
    telemetry->request[3] = 0x04U;
    telemetry->request[4] = 0x02U;
    telemetry->request[5] = 56U;
    telemetry->request[6] = 2U;
    telemetry->request[7] = Servo_Checksum(telemetry->request, 6U);
    ServoRxWindow_Init(
        &telemetry->window,
        servo_id,
        2U,
        transaction_start_absolute
    );

    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(
        servo_uart_handle,
        telemetry->request,
        sizeof(telemetry->request)
    );
    if (status != HAL_OK)
    {
        return ServoInMotionTelemetry_Fail(
            SERVO_BUS_FAILURE_TX, status, 0U, 0U
        );
    }

    telemetry->state = SERVO_IN_MOTION_TELEMETRY_TX_PENDING;
    servo_in_motion_snapshot.requested_samples++;
    return HAL_OK;
}

void Servo_InMotionTelemetryOnTxComplete(UART_HandleTypeDef *uart)
{
    if ((uart == servo_uart_handle) &&
        (servo_in_motion_telemetry.state ==
            SERVO_IN_MOTION_TELEMETRY_TX_PENDING))
    {
        servo_in_motion_telemetry.tx_completed = 1U;
    }
}

static HAL_StatusTypeDef ServoInMotionTelemetry_Fail(
    ServoBusFailureReason reason,
    HAL_StatusTypeDef status,
    uint8_t servo_status,
    uint16_t discarded_bytes
)
{
    ServoInMotionTelemetry *telemetry = &servo_in_motion_telemetry;
    ServoBus_RecordFailure(
        reason,
        telemetry->servo_id,
        status,
        servo_status,
        discarded_bytes
    );
    servo_in_motion_snapshot.failed_samples++;
    telemetry->state = SERVO_IN_MOTION_TELEMETRY_IDLE;
    (void)ServoBus_Recover();
    return status;
}

HAL_StatusTypeDef Servo_InMotionTelemetryPoll(
    uint32_t now_ms,
    const uint16_t commanded_positions[SINGLE_ARM_JOINT_COUNT]
)
{
    ServoInMotionTelemetry *telemetry = &servo_in_motion_telemetry;
    if ((telemetry->enabled == 0U) ||
        (telemetry->state == SERVO_IN_MOTION_TELEMETRY_IDLE))
    {
        return HAL_OK;
    }
    if (commanded_positions == NULL)
    {
        return ServoInMotionTelemetry_Fail(
            SERVO_BUS_FAILURE_RECOVERY, HAL_ERROR, 0U, 0U
        );
    }

    uint32_t callback_errors = servo_uart_async_errors;
    uint32_t polled_errors = ServoBus_CurrentUartErrorCode();
    uint32_t uart_errors = callback_errors | polled_errors;
    uint32_t dma_error = servo_dma_async_error;
    if (servo_uart_handle->hdmarx != NULL)
    {
        dma_error |= servo_uart_handle->hdmarx->ErrorCode;
    }
    if (uart_errors != HAL_UART_ERROR_NONE)
    {
        ServoBus_CountUartErrors(polled_errors & ~callback_errors);
        servo_uart_async_errors = 0U;
        servo_uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
        return ServoInMotionTelemetry_Fail(
            SERVO_BUS_FAILURE_UART,
            HAL_ERROR,
            0U,
            telemetry->window.parser.discarded_bytes
        );
    }
    if (dma_error != HAL_DMA_ERROR_NONE)
    {
        return ServoInMotionTelemetry_Fail(
            SERVO_BUS_FAILURE_DMA,
            HAL_ERROR,
            0U,
            telemetry->window.parser.discarded_bytes
        );
    }

    if ((telemetry->state == SERVO_IN_MOTION_TELEMETRY_TX_PENDING) &&
        (telemetry->tx_completed != 0U))
    {
        telemetry->state = SERVO_IN_MOTION_TELEMETRY_WAIT_REPLY;
    }

    if (telemetry->state == SERVO_IN_MOTION_TELEMETRY_WAIT_REPLY)
    {
        uint8_t position_data[2] = {0U};
        ServoRxWindowResult result = ServoRxWindow_Consume(
            &telemetry->window,
            servo_dma_rx_ring,
            SERVO_BUS_DMA_RING_CAPACITY,
            ServoBus_ProducerAbsolute(),
            position_data,
            sizeof(position_data)
        );
        if (result == SERVO_RX_WINDOW_FRAME_READY)
        {
            uint16_t actual = (uint16_t)(
                (uint16_t)position_data[0] |
                ((uint16_t)position_data[1] << 8U)
            );
            uint16_t commanded = commanded_positions[telemetry->joint_index];
            uint16_t error = (actual > commanded) ?
                (uint16_t)(actual - commanded) :
                (uint16_t)(commanded - actual);
            if (error > servo_in_motion_snapshot.maximum_error_raw[
                    telemetry->joint_index])
            {
                servo_in_motion_snapshot.maximum_error_raw[
                    telemetry->joint_index] = error;
            }
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
            servo_in_motion_snapshot.last_joint_index =
                telemetry->joint_index;
            servo_in_motion_snapshot.last_position_raw = actual;
            servo_in_motion_snapshot.last_commanded_raw = commanded;
#endif
            uint32_t latency_ms = now_ms - telemetry->started_at_ms;
            if (latency_ms >
                servo_in_motion_snapshot.maximum_reply_latency_ms)
            {
                servo_in_motion_snapshot.maximum_reply_latency_ms = latency_ms;
            }
            servo_in_motion_snapshot.completed_samples++;
            servo_bus_diagnostics.discarded_bytes =
                telemetry->window.parser.discarded_bytes;
            servo_bus_diagnostics.received_bytes =
                telemetry->window.consumed_bytes;
            servo_bus_health.success_count++;
            servo_bus_health.discarded_bytes +=
                telemetry->window.parser.discarded_bytes;
            servo_transaction_active = 0U;
            telemetry->state = SERVO_IN_MOTION_TELEMETRY_IDLE;
            return HAL_OK;
        }
        if (result == SERVO_RX_WINDOW_STATUS_ERROR)
        {
            return ServoInMotionTelemetry_Fail(
                SERVO_BUS_FAILURE_STATUS,
                HAL_ERROR,
                telemetry->window.parser.servo_status,
                telemetry->window.parser.discarded_bytes
            );
        }
        if (result == SERVO_RX_WINDOW_OVERFLOW)
        {
            servo_bus_health.overflow_count++;
            return ServoInMotionTelemetry_Fail(
                SERVO_BUS_FAILURE_RX_OVERFLOW,
                HAL_ERROR,
                0U,
                telemetry->window.parser.discarded_bytes
            );
        }
    }

    if (ServoRxWindow_DeadlineExpired(
            telemetry->started_at_ms,
            now_ms,
            SERVO_IN_MOTION_TELEMETRY_TIMEOUT_MS
        ) != 0U)
    {
        servo_bus_health.timeout_count++;
        ServoBusFailureReason reason = SERVO_BUS_FAILURE_RX_TIMEOUT;
        if (telemetry->window.parser.last_reject !=
            SERVO_RESPONSE_REJECT_NONE)
        {
            reason = ServoBus_MapRejectReason(
                telemetry->window.parser.last_reject
            );
        }
        return ServoInMotionTelemetry_Fail(
            reason,
            HAL_TIMEOUT,
            0U,
            telemetry->window.parser.discarded_bytes
        );
    }
    return HAL_BUSY;
}

HAL_StatusTypeDef Servo_ReadPosition(
    uint8_t servo_id,
    uint16_t *position
);

HAL_StatusTypeDef Servo_ReadData(
    uint8_t servo_id,
    uint8_t address,
    uint8_t data_length,
    uint8_t *data
);

HAL_StatusTypeDef Servo_WriteData(
    uint8_t servo_id,
    uint8_t address,
    const uint8_t *data,
    uint8_t data_length
);

HAL_StatusTypeDef Servo_DisableTorqueAll(void)
{
    uint8_t torque_off[1] = {0U};
    HAL_StatusTypeDef result = HAL_OK;

#if HOST_SERVO_DISABLE_READBACK_RECOVERY_BUILD
    /*
     * Torque removal is proven by register-40 readback, not by the write
     * response alone.  STS3215 may apply a write even when its status reply is
     * lost during a power-domain/UART recovery edge, so do not convert that
     * transient into a permanent stop before physical state is verified.
     */
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        (void)Servo_WriteData(
            servo_joints[i].id,
            40U,
            torque_off,
            sizeof(torque_off)
        );
    }

    HAL_Delay(5U);

    /*
     * A failed or non-zero readback receives one bounded per-ID rewrite and
     * one final readback.  The function remains fail-closed: every servo must
     * report Torque Enable == 0 or the caller latches the existing fault.
     */
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        uint8_t verified_disabled = 0U;

        for (uint8_t attempt = 0U; attempt < 2U; attempt++)
        {
            uint8_t torque_readback[1] = {1U};

            if ((Servo_ReadData(
                    servo_joints[i].id,
                    40U,
                    sizeof(torque_readback),
                    torque_readback
                ) == HAL_OK) &&
                (torque_readback[0] == 0U))
            {
                verified_disabled = 1U;
                break;
            }

            if (attempt == 0U)
            {
                (void)Servo_WriteData(
                    servo_joints[i].id,
                    40U,
                    torque_off,
                    sizeof(torque_off)
                );
                HAL_Delay(SERVO_BUS_RECOVERY_QUIET_MS);
            }
        }

        if (verified_disabled == 0U)
        {
            result = HAL_ERROR;
        }
    }
#else
    /*
     * Continue through all six IDs even after one failure.  A partial bus
     * failure must not prevent the remaining joints from receiving the
     * safest command available.
     */
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        if (Servo_WriteData(
                servo_joints[i].id,
                40U,
                torque_off,
                sizeof(torque_off)
            ) != HAL_OK)
        {
            result = HAL_ERROR;
        }
    }

    HAL_Delay(5U);

    /*
     * DISABLE may be acknowledged only after every servo reports Torque
     * Enable register 40 as zero.  A write-only ACK previously allowed the
     * host state machine to claim torque=DISABLED while the arm stayed stiff.
     */
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        uint8_t torque_readback[1] = {1U};

        if ((Servo_ReadData(
                servo_joints[i].id,
                40U,
                sizeof(torque_readback),
                torque_readback
            ) != HAL_OK) ||
            (torque_readback[0] != 0U))
        {
            result = HAL_ERROR;
        }
    }
#endif

    return result;
}

int32_t Servo_PositionError(
    uint16_t actual_position,
    uint16_t target_position
)
{
    int32_t error =
        (int32_t)actual_position -
        (int32_t)target_position;

    /* 0/4095 경계를 고려한 최단 위치 오차 */
    if (error > 2048)
    {
        error -= 4096;
    }
    else if (error < -2048)
    {
        error += 4096;
    }

    return error;
}

#if ENABLE_SERVO_CENTERING_COMMAND
HAL_StatusTypeDef Servo_CenterAtCurrentPosition(
    uint8_t servo_id,
    uint16_t *position_before,
    int16_t *offset_before
)
{
    if ((position_before == NULL) ||
        (offset_before == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t offset_data[2] = {0U};
    uint8_t torque_off[1] = {0U};
    uint8_t center_command[1] = {128U};

    if (Servo_ReadPosition(
            servo_id,
            position_before
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (Servo_ReadData(
            servo_id,
            31U,
            2U,
            offset_data
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *offset_before = (int16_t)(
        (uint16_t)offset_data[0] |
        ((uint16_t)offset_data[1] << 8)
    );

    if (Servo_WriteData(
            servo_id,
            40U,
            torque_off,
            sizeof(torque_off)
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(50U);

    /*
     * 공식 one-key centering:
     * 현재 물리 위치를 내부 위치 2048로 보정
     */
    if (Servo_WriteData(
            servo_id,
            40U,
            center_command,
            sizeof(center_command)
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(500U);

    /* 새 현재 위치를 목표로 넣은 뒤 토크 재활성화 */
    return HAL_OK;
}
#endif

HAL_StatusTypeDef Servo_WaitForPosition(
    uint8_t servo_id,
    uint16_t target_position,
    uint16_t tolerance,
    uint32_t timeout_ms,
    uint16_t *actual_position
)
{
    if (actual_position == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t wait_start = HAL_GetTick();
    *actual_position = 0U;

    while ((HAL_GetTick() - wait_start) < timeout_ms)
    {
        uint16_t current_position = 0U;

        if (Servo_ReadPosition(
                servo_id,
                &current_position
            ) == HAL_OK)
        {
            *actual_position = current_position;

            int32_t error = Servo_PositionError(
                current_position,
                target_position
            );

            if ((error >= -(int32_t)tolerance) &&
                (error <= (int32_t)tolerance))
            {
                return HAL_OK;
            }
        }

        HAL_Delay(20U);
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef Servo_ReadData(
    uint8_t servo_id,
    uint8_t start_address,
    uint8_t data_length,
    uint8_t *data
)
{
    if ((servo_uart_handle == NULL) ||
        (data == NULL) ||
        (data_length == 0U) ||
        (data_length > 16U))
    {
        return HAL_ERROR;
    }

    uint8_t request[8] = {
        0xFFU, 0xFFU, 0x00U, 0x04U,
        0x02U, 0x00U, 0x00U, 0x00U
    };

    request[2] = servo_id;
    request[5] = start_address;
    request[6] = data_length;
    request[7] = Servo_Checksum(request, 6U);

    uint32_t transaction_start_absolute = 0U;
    if (ServoBus_PrepareTransaction(
            servo_id,
            &transaction_start_absolute
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }
    uint32_t transaction_start_tick = HAL_GetTick();

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        servo_uart_handle,
        request,
        sizeof(request),
        SERVO_BUS_READ_TX_TIMEOUT_MS
    );

    if (status != HAL_OK)
    {
        ServoBus_RecordFailure(
            SERVO_BUS_FAILURE_TX,
            servo_id,
            status,
            0U,
            0U
        );
        (void)ServoBus_Recover();
        return status;
    }

    ServoRxWindow window;
    ServoRxWindow_Init(
        &window,
        servo_id,
        data_length,
        transaction_start_absolute
    );

    while (ServoRxWindow_DeadlineExpired(
        transaction_start_tick,
        HAL_GetTick(),
        SERVO_BUS_READ_TIMEOUT_MS
    ) == 0U)
    {
        uint32_t callback_errors = servo_uart_async_errors;
        uint32_t polled_errors = ServoBus_CurrentUartErrorCode();
        uint32_t uart_errors = callback_errors | polled_errors;
        uint32_t dma_error = servo_dma_async_error;
        if (servo_uart_handle->hdmarx != NULL)
        {
            dma_error |= servo_uart_handle->hdmarx->ErrorCode;
        }
        if (uart_errors != HAL_UART_ERROR_NONE)
        {
            ServoBus_CountUartErrors(polled_errors & ~callback_errors);
            servo_uart_async_errors = 0U;
            servo_uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
            __HAL_UART_CLEAR_FLAG(
                servo_uart_handle,
                UART_CLEAR_OREF |
                UART_CLEAR_NEF |
                UART_CLEAR_PEF |
                UART_CLEAR_FEF |
                UART_CLEAR_RTOF
            );

            /*
             * NE/PE bytes remain checksum-gated. FE indicates receiver framing
             * contamination at the power boundary; FE/ORE/RTO all hard-
             * resynchronize the receiver and fail this transaction closed.
             */
            if (ServoRxWindow_HardResyncRequired(
                    (uart_errors & HAL_UART_ERROR_FE) != 0U,
                    (uart_errors & HAL_UART_ERROR_ORE) != 0U,
                    (uart_errors & HAL_UART_ERROR_RTO) != 0U,
                    0U
                ) != 0U)
            {
                ServoBus_RecordFailure(
                    SERVO_BUS_FAILURE_UART,
                    servo_id,
                    HAL_ERROR,
                    0U,
                    window.parser.discarded_bytes
                );
                servo_bus_diagnostics.uart_error_code = uart_errors;
                servo_bus_diagnostics.received_bytes =
                    window.consumed_bytes;
                (void)ServoBus_Recover();
                return HAL_ERROR;
            }
        }
        if (dma_error != HAL_DMA_ERROR_NONE)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_DMA,
                servo_id,
                HAL_ERROR,
                0U,
                window.parser.discarded_bytes
            );
            servo_bus_diagnostics.dma_error_code = dma_error;
            servo_bus_diagnostics.received_bytes = window.consumed_bytes;
            (void)ServoBus_Recover();
            return HAL_ERROR;
        }

        ServoRxWindowResult result = ServoRxWindow_Consume(
            &window,
            servo_dma_rx_ring,
            SERVO_BUS_DMA_RING_CAPACITY,
            ServoBus_ProducerAbsolute(),
            data,
            data_length
        );
        if (result == SERVO_RX_WINDOW_FRAME_READY)
        {
            servo_bus_diagnostics.discarded_bytes =
                window.parser.discarded_bytes;
            servo_bus_diagnostics.received_bytes =
                window.consumed_bytes;
            if (ServoBus_DisarmReceiver() != HAL_OK)
            {
                ServoBus_RecordFailure(
                    SERVO_BUS_FAILURE_RECOVERY,
                    servo_id,
                    HAL_ERROR,
                    0U,
                    window.parser.discarded_bytes
                );
                return HAL_ERROR;
            }
            servo_bus_health.success_count++;
            servo_bus_health.discarded_bytes +=
                window.parser.discarded_bytes;
            return HAL_OK;
        }
        if (result == SERVO_RX_WINDOW_STATUS_ERROR)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_STATUS,
                servo_id,
                HAL_ERROR,
                window.parser.servo_status,
                window.parser.discarded_bytes
            );
            servo_bus_diagnostics.received_bytes = window.consumed_bytes;
            (void)ServoBus_Recover();
            return HAL_ERROR;
        }
        if (result == SERVO_RX_WINDOW_OVERFLOW)
        {
            servo_bus_health.overflow_count++;
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_RX_OVERFLOW,
                servo_id,
                HAL_ERROR,
                0U,
                window.parser.discarded_bytes
            );
            servo_bus_diagnostics.received_bytes = window.consumed_bytes;
            (void)ServoBus_Recover();
            return HAL_ERROR;
        }
    }

    servo_bus_health.timeout_count++;
    ServoBusFailureReason reason = SERVO_BUS_FAILURE_RX_TIMEOUT;
    if (window.parser.last_reject != SERVO_RESPONSE_REJECT_NONE)
    {
        reason = ServoBus_MapRejectReason(window.parser.last_reject);
    }
    ServoBus_RecordFailure(
        reason,
        servo_id,
        HAL_TIMEOUT,
        0U,
        window.parser.discarded_bytes
    );
    servo_bus_diagnostics.received_bytes = window.consumed_bytes;
    (void)ServoBus_Recover();
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef Servo_WriteData(
    uint8_t servo_id,
    uint8_t start_address,
    const uint8_t *data,
    uint8_t data_length
)
{
    if ((servo_uart_handle == NULL) ||
        (data == NULL) ||
        (data_length == 0U) ||
        (data_length > 16U))
    {
        return HAL_ERROR;
    }

    uint8_t packet[23] = {0U};

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = servo_id;
    packet[3] = (uint8_t)(data_length + 3U);
    packet[4] = 0x03U;
    packet[5] = start_address;

    for (uint8_t i = 0U; i < data_length; i++)
    {
        packet[6U + i] = data[i];
    }

    uint8_t checksum_index = (uint8_t)(6U + data_length);

    packet[checksum_index] = Servo_Checksum(
        packet,
        (uint8_t)(checksum_index - 1U)
    );

    uint32_t transaction_start_absolute = 0U;
    if (ServoBus_PrepareTransaction(
            servo_id,
            &transaction_start_absolute
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }
    (void)transaction_start_absolute;

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        servo_uart_handle,
        packet,
        (uint16_t)data_length + 7U,
        100U
    );

    if (status != HAL_OK)
    {
        ServoBus_RecordFailure(
            SERVO_BUS_FAILURE_TX,
            servo_id,
            status,
            0U,
            0U
        );
        (void)ServoBus_Recover();
        return status;
    }

    /*
     * Safety-critical callers verify runtime writes by register readback. Do
     * not start a fixed six-byte receive that can time out halfway through a
     * late optional status frame and poison the next READ. Let an optional
     * reply finish on the wire, then discard it atomically.
     */
    HAL_Delay(SERVO_BUS_WRITE_REPLY_SETTLE_MS);
    if (ServoBus_DisarmReceiver() != HAL_OK)
    {
        ServoBus_RecordFailure(
            SERVO_BUS_FAILURE_RECOVERY, servo_id, HAL_ERROR, 0U, 0U
        );
        return HAL_ERROR;
    }
    servo_bus_health.success_count++;
    return HAL_OK;
}

HAL_StatusTypeDef Servo_ReadPosition(
    uint8_t servo_id,
    uint16_t *position
)
{
    uint8_t position_data[2] = {0U};

    HAL_StatusTypeDef status = Servo_ReadData(
        servo_id,
        56U,
        sizeof(position_data),
        position_data
    );

    if (status == HAL_OK)
    {
        *position = (uint16_t)(
            (uint16_t)position_data[0] |
            ((uint16_t)position_data[1] << 8)
        );
    }

    return status;
}

HAL_StatusTypeDef Servo_ReadTelemetry(
    uint8_t servo_id,
    uint16_t *position,
    uint16_t *speed_raw,
    uint16_t *load_raw,
    uint8_t *voltage_raw,
    uint8_t *temperature_c,
    uint16_t *current_raw
)
{
    if ((position == NULL) ||
        (speed_raw == NULL) ||
        (load_raw == NULL) ||
        (voltage_raw == NULL) ||
        (temperature_c == NULL) ||
        (current_raw == NULL))
    {
        return HAL_ERROR;
    }

    /* Address 56 through 70: position, speed, load, voltage,
       temperature, asynchronous-write flag/status/moving, current. */
    uint8_t telemetry[15] = {0U};

    HAL_StatusTypeDef status = Servo_ReadData(
        servo_id,
        56U,
        sizeof(telemetry),
        telemetry
    );

    if (status == HAL_OK)
    {
        *position = (uint16_t)(
            (uint16_t)telemetry[0] |
            ((uint16_t)telemetry[1] << 8)
        );

        *speed_raw = (uint16_t)(
            (uint16_t)telemetry[2] |
            ((uint16_t)telemetry[3] << 8)
        );

        *load_raw = (uint16_t)(
            (uint16_t)telemetry[4] |
            ((uint16_t)telemetry[5] << 8)
        );

        *voltage_raw = telemetry[6];
        *temperature_c = telemetry[7];

        *current_raw = (uint16_t)(
            (uint16_t)telemetry[13] |
            ((uint16_t)telemetry[14] << 8)
        );
    }

    return status;
}

static uint16_t Servo_CurrentMagnitude(uint16_t current_raw)
{
    int32_t signed_current = (int32_t)(int16_t)current_raw;

    if (signed_current < 0)
    {
        signed_current = -signed_current;
    }

    return (uint16_t)signed_current;
}

void Servo_MotionSafetyBegin(uint8_t joint_mask)
{
    servo_motion_safety_mask = joint_mask;
    servo_motion_safety_next_joint = 0U;
    servo_motion_safety_last_sample_ms =
        HAL_GetTick() - SERVO_MOTION_SAFETY_SLOT_MS;

    memset(
        servo_load_limit_counts,
        0,
        sizeof(servo_load_limit_counts)
    );
    memset(
        servo_current_limit_counts,
        0,
        sizeof(servo_current_limit_counts)
    );
    memset(
        &servo_motion_safety_diagnostics,
        0,
        sizeof(servo_motion_safety_diagnostics)
    );
}

void Servo_MotionSafetyEnd(void)
{
    servo_motion_safety_mask = 0U;
    servo_motion_safety_next_joint = 0U;
}

const ServoMotionSafetyDiagnostics *Servo_MotionSafetyGetDiagnostics(void)
{
    return &servo_motion_safety_diagnostics;
}

HAL_StatusTypeDef Servo_MotionSafetyPoll(void)
{
    uint32_t now = HAL_GetTick();

    if ((servo_motion_safety_mask == 0U) ||
        ((now - servo_motion_safety_last_sample_ms) <
            SERVO_MOTION_SAFETY_SLOT_MS))
    {
        return HAL_OK;
    }

    servo_motion_safety_last_sample_ms = now;

    /*
     * Sample exactly one enabled joint per call. A six-joint synchronous sweep
     * can occupy the main loop long enough to starve host traffic. Round-robin
     * slots retain approximately the original 100 ms per-joint cadence while
     * bounding one service call to one servo transaction.
     */
    for (uint8_t checked = 0U; checked < servo_joint_count; checked++)
    {
        uint8_t i = servo_motion_safety_next_joint;
        servo_motion_safety_next_joint = (uint8_t)(
            (servo_motion_safety_next_joint + 1U) % servo_joint_count
        );

        if ((servo_motion_safety_mask & (uint8_t)(1U << i)) == 0U)
        {
            continue;
        }

        uint16_t position = 0U;
        uint16_t speed_raw = 0U;
        uint16_t load_raw = 0U;
        uint8_t voltage_raw = 0U;
        uint8_t temperature_c = 0U;
        uint16_t current_raw = 0U;

        if (Servo_ReadTelemetry(
                servo_joints[i].id,
                &position,
                &speed_raw,
                &load_raw,
                &voltage_raw,
                &temperature_c,
                &current_raw
            ) != HAL_OK)
        {
            servo_motion_safety_diagnostics.reason =
                SERVO_MOTION_SAFETY_READ_FAILURE;
            servo_motion_safety_diagnostics.servo_id =
                servo_joints[i].id;
            return HAL_ERROR;
        }

        uint16_t load_magnitude = (uint16_t)(load_raw & 0x03FFU);
        uint16_t current_magnitude = Servo_CurrentMagnitude(current_raw);

        servo_motion_safety_diagnostics.servo_id = servo_joints[i].id;
        servo_motion_safety_diagnostics.last_load_raw = load_magnitude;
        servo_motion_safety_diagnostics.last_current_raw = current_magnitude;

        if (load_magnitude >
            servo_motion_safety_diagnostics.maximum_load_raw)
        {
            servo_motion_safety_diagnostics.maximum_load_raw = load_magnitude;
        }
        if (current_magnitude >
            servo_motion_safety_diagnostics.maximum_current_raw)
        {
            servo_motion_safety_diagnostics.maximum_current_raw =
                current_magnitude;
        }

        if (load_magnitude >= SERVO_MOTION_LOAD_LIMIT_RAW)
        {
            if (servo_load_limit_counts[i] < UINT8_MAX)
            {
                servo_load_limit_counts[i]++;
            }
        }
        else
        {
            servo_load_limit_counts[i] = 0U;
        }

        if (current_magnitude >= SERVO_MOTION_CURRENT_LIMIT_RAW)
        {
            if (servo_current_limit_counts[i] < UINT8_MAX)
            {
                servo_current_limit_counts[i]++;
            }
        }
        else
        {
            servo_current_limit_counts[i] = 0U;
        }

        if (servo_load_limit_counts[i] >=
            SERVO_MOTION_LIMIT_CONSECUTIVE)
        {
            servo_motion_safety_diagnostics.reason =
                SERVO_MOTION_SAFETY_LOAD_LIMIT;
            return HAL_BUSY;
        }
        if (servo_current_limit_counts[i] >=
            SERVO_MOTION_LIMIT_CONSECUTIVE)
        {
            servo_motion_safety_diagnostics.reason =
                SERVO_MOTION_SAFETY_CURRENT_LIMIT;
            return HAL_BUSY;
        }
        return HAL_OK;
    }

    return HAL_OK;
}

static uint8_t Host_StopRequestedDuringMotion(void)
{
    if (servo_stop_requested == NULL)
    {
        return 0U;
    }

    return servo_stop_requested();
}

HAL_StatusTypeDef Servo_RunSmoothstep(
    uint8_t servo_id,
    uint16_t start_position,
    uint16_t target_position,
    uint32_t duration_ms
)
{
    const uint32_t control_period_ms = 20U;
    uint8_t joint_mask = 0U;

    if ((duration_ms < control_period_ms) ||
        ((duration_ms % control_period_ms) != 0U))
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        if (servo_joints[i].id == servo_id)
        {
            joint_mask = (uint8_t)(1U << i);
            break;
        }
    }

    if (joint_mask == 0U)
    {
        return HAL_ERROR;
    }

    Servo_MotionSafetyBegin(joint_mask);

    uint32_t trajectory_steps =
        duration_ms / control_period_ms;

    uint32_t denominator =
        trajectory_steps *
        trajectory_steps *
        trajectory_steps;

    int32_t signed_delta =
        (int32_t)target_position -
        (int32_t)start_position;

    for (uint32_t step = 1U;
         step <= trajectory_steps;
         step++)
    {
        if (Host_StopRequestedDuringMotion() != 0U)
        {
            Servo_MotionSafetyEnd();
            return HAL_BUSY;
        }

        uint32_t cycle_start = HAL_GetTick();
        uint32_t step_squared = step * step;

        /* Smoothstep: s(t) = 3t^2 - 2t^3 */
        uint32_t smooth_numerator =
            (3U * step_squared * trajectory_steps) -
            (2U * step_squared * step);

        int32_t position_offset =
            (signed_delta *
                (int32_t)smooth_numerator) /
            (int32_t)denominator;

        uint16_t setpoint = (uint16_t)(
            (int32_t)start_position +
            position_offset
        );

        uint8_t goal_data[2] = {
            (uint8_t)(setpoint & 0xFFU),
            (uint8_t)((setpoint >> 8) & 0xFFU)
        };

        if (Servo_WriteData(
                servo_id,
                42U,
                goal_data,
                sizeof(goal_data)
            ) != HAL_OK)
        {
            Servo_MotionSafetyEnd();
            return HAL_ERROR;
        }

        HAL_StatusTypeDef safety_status =
            Servo_MotionSafetyPoll();

        if (safety_status != HAL_OK)
        {
            Servo_MotionSafetyEnd();
            return safety_status;
        }

        uint32_t elapsed =
            HAL_GetTick() - cycle_start;

        if (elapsed < control_period_ms)
        {
            HAL_Delay(control_period_ms - elapsed);
        }
    }

    Servo_MotionSafetyEnd();
    return HAL_OK;
}

HAL_StatusTypeDef Servo_ConfigureForTrajectory(
    uint8_t servo_id,
    uint16_t torque_limit,
    uint8_t p_gain,
    uint8_t d_gain,
    uint16_t *initial_position
)
{
    if (Servo_ReadPosition(
            servo_id,
            initial_position
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }

    uint8_t torque_off[1] = {0U};
    uint8_t torque_on[1] = {1U};
    uint8_t lock_volatile[1] = {1U};
    uint8_t position_mode[1] = {0U};

    /* 주소 순서: P, D, I */
    uint8_t pid_data[3] = {
        p_gain,
        d_gain,
        0U
    };

    uint8_t runtime_data[9] = {
        0U,
        (uint8_t)(*initial_position & 0xFFU),
        (uint8_t)((*initial_position >> 8) & 0xFFU),
        0U, 0U,
        (uint8_t)(SERVO_GOAL_SPEED_RAW & 0xFFU),
        (uint8_t)((SERVO_GOAL_SPEED_RAW >> 8) & 0xFFU),
        (uint8_t)(torque_limit & 0xFFU),
        (uint8_t)((torque_limit >> 8) & 0xFFU)
    };

    if ((Servo_WriteData(
            servo_id, 40U, torque_off, 1U
        ) != HAL_OK) ||
        (Servo_WriteData(
            servo_id, 55U, lock_volatile, 1U
        ) != HAL_OK) ||
        (Servo_WriteData(
            servo_id, 33U, position_mode, 1U
        ) != HAL_OK) ||
        (Servo_WriteData(
            servo_id, 21U, pid_data, 3U
        ) != HAL_OK) ||
        (Servo_WriteData(
            servo_id, 41U, runtime_data, 9U
        ) != HAL_OK) ||
        (Servo_WriteData(
            servo_id, 40U, torque_on, 1U
        ) != HAL_OK))
    {
        return HAL_ERROR;
    }

    HAL_Delay(20U);

    uint8_t pid_readback[3] = {0U};
    uint8_t torque_limit_readback[2] = {0U};

    if ((Servo_ReadData(
            servo_id,
            21U,
            sizeof(pid_readback),
            pid_readback
        ) != HAL_OK) ||
        (pid_readback[0] != p_gain) ||
        (pid_readback[1] != d_gain) ||
        (pid_readback[2] != 0U) ||
        (Servo_ReadData(
            servo_id,
            48U,
            sizeof(torque_limit_readback),
            torque_limit_readback
        ) != HAL_OK) ||
        (torque_limit_readback[0] !=
            (uint8_t)(torque_limit & 0xFFU)) ||
        (torque_limit_readback[1] !=
            (uint8_t)((torque_limit >> 8) & 0xFFU)))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Servo_PositionSweepBegin(ServoPositionSweep *sweep)
{
    if (sweep == NULL)
    {
        return;
    }

    memset(sweep, 0, sizeof(*sweep));
    servo_last_all_read_failed_id = 0U;
}

HAL_StatusTypeDef Servo_PositionSweepStep(ServoPositionSweep *sweep)
{
    if (sweep == NULL)
    {
        return HAL_ERROR;
    }
    if (sweep->next_joint >= servo_joint_count)
    {
        return HAL_OK;
    }

    uint8_t joint = sweep->next_joint;
    uint16_t position = 0U;

    HAL_Delay(10U);
    HAL_StatusTypeDef read_status = Servo_ReadPosition(
        servo_joints[joint].id,
        &position
    );

    if (read_status == HAL_OK)
    {
        sweep->positions[joint] = position;
        sweep->next_joint++;
        sweep->attempt = 0U;
        return (sweep->next_joint >= servo_joint_count) ?
            HAL_OK : HAL_BUSY;
    }

    sweep->attempt++;
    if (sweep->attempt < 3U)
    {
        return HAL_BUSY;
    }

    servo_last_all_read_failed_id = servo_joints[joint].id;
    if (servo_read_failure != NULL)
    {
        servo_read_failure(servo_last_all_read_failed_id);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef Servo_ReadAllPositions(
    uint16_t positions[6]
)
{
    if (positions == NULL)
    {
        return HAL_ERROR;
    }

    ServoPositionSweep sweep;
    Servo_PositionSweepBegin(&sweep);

    HAL_StatusTypeDef status = HAL_BUSY;
    while (status == HAL_BUSY)
    {
        status = Servo_PositionSweepStep(&sweep);
    }
    if (status == HAL_OK)
    {
        memcpy(positions, sweep.positions, sizeof(sweep.positions));
    }
    return status;
}

#if HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD
HAL_StatusTypeDef Servo_SyncWritePositions(
    const uint16_t positions[6]
)
{
    uint8_t ids[SINGLE_ARM_JOINT_COUNT] = {0U};
    uint8_t packet[ACTUATOR_STS3215_SYNC_WRITE_POSITION_PACKET_SIZE] = {0U};
    size_t packet_length = 0U;

    if (positions == NULL)
    {
        return HAL_ERROR;
    }
    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        ids[i] = servo_joints[i].id;
    }
    if (actuator_sts3215_build_sync_write_positions(
            ids,
            positions,
            servo_joint_count,
            packet,
            &packet_length
        ) != ACTUATOR_STS3215_PACKET_OK)
    {
        return HAL_ERROR;
    }

    uint32_t started_us = Timebase_NowUs();
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        servo_uart_handle,
        packet,
        (uint16_t)packet_length,
        100U
    );
    F0Metrics_ObserveServoSyncWrite(Timebase_ElapsedUs(started_us));
    return status;
}
#else
HAL_StatusTypeDef Servo_SyncWritePositions(
    const uint16_t positions[6]
)
{
    if (positions == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * FEETECH protocol-0 SYNC WRITE:
     * FF FF FE LEN 83 START_ADDR DATA_LEN [ID DATA...] CHECKSUM
     */
    uint8_t packet[26] = {0U};
    uint8_t packet_index = 0U;

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = 0xFEU;
    packet[3] = (uint8_t)(
        4U + (servo_joint_count * 3U)
    );
    packet[4] = 0x83U;
    packet[5] = 42U;
    packet[6] = 2U;
    packet_index = 7U;

    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        packet[packet_index++] = servo_joints[i].id;
        packet[packet_index++] =
            (uint8_t)(positions[i] & 0xFFU);
        packet[packet_index++] =
            (uint8_t)((positions[i] >> 8) & 0xFFU);
    }

    packet[packet_index] = Servo_Checksum(
        packet,
        (uint8_t)(packet_index - 1U)
    );
    packet_index++;

    uint32_t started_us = Timebase_NowUs();
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        servo_uart_handle,
        packet,
        packet_index,
        100U
    );
    F0Metrics_ObserveServoSyncWrite(Timebase_ElapsedUs(started_us));
    return status;
}
#endif

HAL_StatusTypeDef Servo_ConfigureAllForTrajectory(
    uint16_t initial_positions[6]
)
{
    if (initial_positions == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        if (Servo_ConfigureForTrajectory(
                servo_joints[i].id,
                servo_joints[i].torque_limit,
                servo_joints[i].p_gain,
                servo_joints[i].d_gain,
                &initial_positions[i]
            ) != HAL_OK)
        {
            uint8_t torque_off[1] = {0U};

            for (uint8_t rollback = 0U;
                 rollback <= i;
                 rollback++)
            {
                (void)Servo_WriteData(
                    servo_joints[rollback].id,
                    40U,
                    torque_off,
                    sizeof(torque_off)
                );
            }

            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Servo_RunSynchronizedSmoothstep(
    const uint16_t start_positions[6],
    const uint16_t target_positions[6],
    uint32_t duration_ms
)
{
    const uint32_t control_period_ms = 20U;

    if ((start_positions == NULL) ||
        (target_positions == NULL) ||
        (duration_ms < control_period_ms) ||
        ((duration_ms % control_period_ms) != 0U))
    {
        return HAL_ERROR;
    }

    uint32_t trajectory_steps =
        duration_ms / control_period_ms;

    if (trajectory_steps > 100U)
    {
        return HAL_ERROR;
    }

    Servo_MotionSafetyBegin(
        (uint8_t)((1U << SINGLE_ARM_JOINT_COUNT) - 1U)
    );

    uint32_t denominator =
        trajectory_steps *
        trajectory_steps *
        trajectory_steps;

    for (uint32_t step = 1U;
         step <= trajectory_steps;
         step++)
    {
        if (Host_StopRequestedDuringMotion() != 0U)
        {
            Servo_MotionSafetyEnd();
            return HAL_BUSY;
        }

        uint32_t cycle_start = HAL_GetTick();
        uint32_t step_squared = step * step;
        uint32_t smooth_numerator =
            (3U * step_squared * trajectory_steps) -
            (2U * step_squared * step);
        uint16_t setpoints[6] = {0U};

        for (uint8_t i = 0U; i < servo_joint_count; i++)
        {
            int32_t signed_delta =
                (int32_t)target_positions[i] -
                (int32_t)start_positions[i];

            int32_t position_offset = (int32_t)(
                ((int64_t)signed_delta *
                    (int64_t)smooth_numerator) /
                (int64_t)denominator
            );

            int32_t setpoint =
                (int32_t)start_positions[i] +
                position_offset;

            if ((setpoint < 0) || (setpoint > 4095))
            {
                Servo_MotionSafetyEnd();
                return HAL_ERROR;
            }

            setpoints[i] = (uint16_t)setpoint;
        }

        if (Servo_SyncWritePositions(setpoints) != HAL_OK)
        {
            Servo_MotionSafetyEnd();
            return HAL_ERROR;
        }

        HAL_StatusTypeDef safety_status =
            Servo_MotionSafetyPoll();

        if (safety_status != HAL_OK)
        {
            Servo_MotionSafetyEnd();
            return safety_status;
        }

        uint32_t elapsed = HAL_GetTick() - cycle_start;

        if (elapsed < control_period_ms)
        {
            HAL_Delay(control_period_ms - elapsed);
        }
    }

    Servo_MotionSafetyEnd();
    return HAL_OK;
}
