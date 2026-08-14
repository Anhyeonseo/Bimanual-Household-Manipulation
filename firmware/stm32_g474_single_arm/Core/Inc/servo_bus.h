#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include "stm32g4xx_hal.h"
#include "single_arm_config.h"

#include <stdint.h>

#define SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES UINT8_C(16)

typedef struct
{
    uint8_t id;
    const char *name;
    uint8_t motion_enabled;
    uint16_t home_position;
    uint16_t min_position;
    uint16_t max_position;
    uint8_t p_gain;
    uint8_t d_gain;
    int8_t test_direction;
    uint16_t test_delta;
    uint32_t duration_ms;
    uint16_t torque_limit;
} ServoJointConfig;

typedef uint8_t (*ServoStopRequestedFn)(void);
typedef void (*ServoReadFailureFn)(uint8_t servo_id);

typedef enum
{
    SERVO_MOTION_SAFETY_NONE = 0,
    SERVO_MOTION_SAFETY_LOAD_LIMIT = 1,
    SERVO_MOTION_SAFETY_CURRENT_LIMIT = 2,
    SERVO_MOTION_SAFETY_READ_FAILURE = 3
} ServoMotionSafetyReason;

typedef struct
{
    ServoMotionSafetyReason reason;
    uint8_t servo_id;
    uint16_t last_load_raw;
    uint16_t last_current_raw;
    uint16_t maximum_load_raw;
    uint16_t maximum_current_raw;
} ServoMotionSafetyDiagnostics;

typedef enum
{
    SERVO_BUS_FAILURE_NONE = 0,
    SERVO_BUS_FAILURE_TX = 1,
    SERVO_BUS_FAILURE_RX_TIMEOUT = 2,
    SERVO_BUS_FAILURE_UART = 3,
    SERVO_BUS_FAILURE_HEADER = 4,
    SERVO_BUS_FAILURE_ID = 5,
    SERVO_BUS_FAILURE_LENGTH = 6,
    SERVO_BUS_FAILURE_STATUS = 7,
    SERVO_BUS_FAILURE_CHECKSUM = 8,
    SERVO_BUS_FAILURE_RECOVERY = 9,
    SERVO_BUS_FAILURE_RX_OVERFLOW = 10,
    SERVO_BUS_FAILURE_DMA = 11
} ServoBusFailureReason;

typedef struct
{
    ServoBusFailureReason reason;
    uint8_t servo_id;
    uint8_t hal_status;
    uint8_t servo_status;
    uint32_t uart_error_code;
    uint32_t uart_isr;
    uint32_t dma_error_code;
    uint32_t recovery_count;
    uint16_t discarded_bytes;
    uint16_t received_bytes;
    uint8_t snapshot_length;
    uint8_t snapshot[SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES];
} ServoBusDiagnostics;

typedef struct
{
    uint32_t transaction_count;
    uint32_t success_count;
    uint32_t failure_count;
    uint32_t recovery_count;
    uint32_t discarded_bytes;
    uint32_t timeout_count;
    uint32_t overflow_count;
    uint32_t rx_event_count;
    uint32_t lazy_arm_count;
    uint32_t receiver_resync_count;
    uint16_t pe_count;
    uint16_t ne_count;
    uint16_t fe_count;
    uint16_t ore_count;
    uint16_t rto_count;
    uint16_t dma_error_count;
    uint16_t producer_index;
    uint8_t dma_started;
    uint8_t last_rx_event;
} ServoBusHealth;

typedef struct
{
    uint16_t positions[SINGLE_ARM_JOINT_COUNT];
    uint8_t next_joint;
    uint8_t attempt;
} ServoPositionSweep;

/* Position-only H2.0 telemetry. Load/current stays in the later F4 work. */
typedef struct
{
    uint16_t maximum_error_raw[SINGLE_ARM_JOINT_COUNT];
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    uint16_t last_position_raw;
    uint16_t last_commanded_raw;
    uint8_t last_joint_index;
#endif
    uint32_t requested_samples;
    uint32_t completed_samples;
    uint32_t failed_samples;
    uint32_t maximum_reply_latency_ms;
} ServoInMotionTelemetrySnapshot;

extern const ServoJointConfig servo_joints[SINGLE_ARM_JOINT_COUNT];
extern const uint8_t servo_joint_count;
extern uint8_t servo_last_all_read_failed_id;

void ServoBus_Init(
    UART_HandleTypeDef *servo_uart,
    ServoStopRequestedFn stop_requested,
    ServoReadFailureFn read_failure
);

const ServoBusDiagnostics *ServoBus_GetDiagnostics(void);
const ServoBusHealth *ServoBus_GetHealth(void);
void ServoBus_HandleUartError(UART_HandleTypeDef *uart);
HAL_StatusTypeDef Servo_ReadPosition(
    uint8_t servo_id,
    uint16_t *position
);
HAL_StatusTypeDef Servo_ReadData(
    uint8_t servo_id,
    uint8_t start_address,
    uint8_t data_length,
    uint8_t *data
);
HAL_StatusTypeDef Servo_WriteData(
    uint8_t servo_id,
    uint8_t start_address,
    const uint8_t *data,
    uint8_t data_length
);
HAL_StatusTypeDef Servo_DisableTorqueAll(void);
int32_t Servo_PositionError(
    uint16_t actual_position,
    uint16_t target_position
);
HAL_StatusTypeDef Servo_CenterAtCurrentPosition(
    uint8_t servo_id,
    uint16_t *position_before,
    int16_t *offset_before
);
HAL_StatusTypeDef Servo_WaitForPosition(
    uint8_t servo_id,
    uint16_t target_position,
    uint16_t tolerance,
    uint32_t timeout_ms,
    uint16_t *actual_position
);
HAL_StatusTypeDef Servo_ReadTelemetry(
    uint8_t servo_id,
    uint16_t *position,
    uint16_t *speed_raw,
    uint16_t *load_raw,
    uint8_t *voltage_raw,
    uint8_t *temperature_c,
    uint16_t *current_raw
);
void Servo_MotionSafetyBegin(uint8_t joint_mask);
void Servo_MotionSafetyEnd(void);
HAL_StatusTypeDef Servo_MotionSafetyPoll(void);
const ServoMotionSafetyDiagnostics *Servo_MotionSafetyGetDiagnostics(void);
HAL_StatusTypeDef Servo_RunSmoothstep(
    uint8_t servo_id,
    uint16_t start_position,
    uint16_t target_position,
    uint32_t duration_ms
);
HAL_StatusTypeDef Servo_ConfigureForTrajectory(
    uint8_t servo_id,
    uint16_t torque_limit,
    uint8_t p_gain,
    uint8_t d_gain,
    uint16_t *initial_position
);
void Servo_PositionSweepBegin(ServoPositionSweep *sweep);
HAL_StatusTypeDef Servo_PositionSweepStep(ServoPositionSweep *sweep);
HAL_StatusTypeDef Servo_ReadAllPositions(
    uint16_t positions[SINGLE_ARM_JOINT_COUNT]
);
HAL_StatusTypeDef Servo_SyncWritePositions(
    const uint16_t positions[SINGLE_ARM_JOINT_COUNT]
);
void Servo_InMotionTelemetryBegin(void);
void Servo_InMotionTelemetryEnd(void);
uint8_t Servo_InMotionTelemetryPending(void);
HAL_StatusTypeDef Servo_InMotionTelemetryStart(
    uint8_t joint_index,
    uint32_t started_at_ms
);
HAL_StatusTypeDef Servo_InMotionTelemetryPoll(
    uint32_t now_ms,
    const uint16_t commanded_positions[SINGLE_ARM_JOINT_COUNT]
);
void Servo_InMotionTelemetryOnTxComplete(UART_HandleTypeDef *uart);
const ServoInMotionTelemetrySnapshot *
Servo_InMotionTelemetryGetSnapshot(void);
HAL_StatusTypeDef Servo_ConfigureAllForTrajectory(
    uint16_t initial_positions[SINGLE_ARM_JOINT_COUNT]
);
HAL_StatusTypeDef Servo_RunSynchronizedSmoothstep(
    const uint16_t start_positions[SINGLE_ARM_JOINT_COUNT],
    const uint16_t target_positions[SINGLE_ARM_JOINT_COUNT],
    uint32_t duration_ms
);

#endif /* SERVO_BUS_H */
