#ifndef RIGHT_SERVO_BUS_H
#define RIGHT_SERVO_BUS_H

#include "stm32g4xx_hal.h"

#include <stdint.h>

/*
 * R0 is observation-only.  R1 adds one tightly bounded migration primitive:
 * one already-torque-enabled servo may receive one Goal_Position (address 42)
 * within +/-20 raw of its freshly read position.  It cannot enable torque,
 * configure PID/speed, change limits, sync-write, broadcast, or command more
 * than one ID.  An active R1 session exposes a physical torque-disable path.
 */
#define RIGHT_SERVO_BUS_JOINT_COUNT UINT8_C(6)
#define RIGHT_SERVO_JOG_MINIMUM_ABSOLUTE_DELTA_RAW INT16_C(8)
#define RIGHT_SERVO_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW INT16_C(20)

typedef enum
{
    RIGHT_SERVO_READ_OK = 0,
    RIGHT_SERVO_READ_UNAVAILABLE = 1,
    RIGHT_SERVO_READ_TX = 2,
    RIGHT_SERVO_READ_RX_TIMEOUT = 3,
    RIGHT_SERVO_READ_HEADER = 4,
    RIGHT_SERVO_READ_ID = 5,
    RIGHT_SERVO_READ_LENGTH = 6,
    RIGHT_SERVO_READ_STATUS = 7,
    RIGHT_SERVO_READ_CHECKSUM = 8
} RightServoReadStatus;

typedef struct
{
    uint8_t present_mask;
    uint8_t statuses[RIGHT_SERVO_BUS_JOINT_COUNT];
    uint16_t positions[RIGHT_SERVO_BUS_JOINT_COUNT];
    uint32_t transaction_count;
    uint32_t failure_count;
} RightServoDiscoverySnapshot;

typedef enum
{
    RIGHT_SERVO_JOG_OK = 0,
    RIGHT_SERVO_JOG_UNAVAILABLE = 1,
    RIGHT_SERVO_JOG_INVALID_REQUEST = 2,
    RIGHT_SERVO_JOG_READ_TORQUE_FAILED = 3,
    RIGHT_SERVO_JOG_TORQUE_DISABLED = 4,
    RIGHT_SERVO_JOG_READ_POSITION_FAILED = 5,
    RIGHT_SERVO_JOG_TARGET_OUT_OF_RANGE = 6,
    RIGHT_SERVO_JOG_WRITE_FAILED = 7,
    RIGHT_SERVO_JOG_POST_READ_FAILED = 8
} RightServoJogStatus;

typedef struct
{
    RightServoJogStatus status;
    uint8_t servo_id;
    int8_t delta_raw;
    uint8_t torque_enabled;
    uint16_t start_position;
    uint16_t target_position;
    uint16_t observed_position;
} RightServoJogSnapshot;

typedef enum
{
    RIGHT_SERVO_TORQUE_ENABLE_OK = 0,
    RIGHT_SERVO_TORQUE_ENABLE_UNAVAILABLE = 1,
    RIGHT_SERVO_TORQUE_ENABLE_INVALID_REQUEST = 2,
    RIGHT_SERVO_TORQUE_ENABLE_READ_TORQUE_FAILED = 3,
    RIGHT_SERVO_TORQUE_ENABLE_READ_POSITION_FAILED = 5,
    RIGHT_SERVO_TORQUE_ENABLE_HOLD_WRITE_FAILED = 7,
    RIGHT_SERVO_TORQUE_ENABLE_WRITE_FAILED = 9,
    RIGHT_SERVO_TORQUE_ENABLE_READBACK_FAILED = 10,
    RIGHT_SERVO_TORQUE_ALREADY_ENABLED = 11,
    RIGHT_SERVO_TORQUE_ENABLE_POSITION_OUT_OF_RANGE = 12
} RightServoTorqueEnableStatus;

typedef struct
{
    RightServoTorqueEnableStatus status;
    uint8_t servo_id;
    uint8_t torque_enabled;
    uint16_t present_position;
    uint16_t held_goal_position;
    uint16_t observed_position;
} RightServoTorqueEnableSnapshot;

#define RIGHT_SERVO_CONFIGURATION_BLOCK_COUNT UINT8_C(5)

typedef struct
{
    uint8_t status;
    uint8_t servo_id;
    uint8_t read_status;
    uint8_t successful_block_mask;
    uint32_t sample_time_ms;
    uint8_t torque_enabled;
    uint8_t p_gain;
    uint8_t d_gain;
    uint8_t i_gain;
    uint8_t voltage_raw;
    uint8_t temperature_c;
    uint16_t position_raw;
    uint16_t speed_raw;
    uint16_t load_raw;
    uint16_t current_raw;
    uint16_t runtime_torque_limit_raw;
    uint16_t goal_position_raw;
    uint16_t model_number;
    uint8_t firmware_major_version;
    uint8_t firmware_minor_version;
    uint16_t maximum_torque_limit_raw;
    uint16_t minimum_startup_force_raw;
    uint8_t cw_dead_zone_raw;
    uint8_t ccw_dead_zone_raw;
    uint16_t protection_current_raw;
    uint8_t operating_mode;
    uint8_t protective_torque_raw;
    uint8_t protection_time_raw;
    uint8_t overload_torque_raw;
} RightServoConfigurationSnapshot;

typedef enum
{
    RIGHT_SERVO_CONFIGURE_OK = 0,
    RIGHT_SERVO_CONFIGURE_UNAVAILABLE = 1,
    RIGHT_SERVO_CONFIGURE_INVALID_REQUEST = 2,
    RIGHT_SERVO_CONFIGURE_READ_TORQUE_FAILED = 3,
    RIGHT_SERVO_CONFIGURE_TORQUE_NOT_DISABLED = 4,
    RIGHT_SERVO_CONFIGURE_READ_POSITION_FAILED = 5,
    RIGHT_SERVO_CONFIGURE_WRITE_FAILED = 6,
    RIGHT_SERVO_CONFIGURE_READBACK_FAILED = 7
} RightServoConfigureStatus;

typedef struct
{
    RightServoConfigureStatus status;
    uint8_t servo_id;
    uint8_t torque_enabled;
    uint8_t p_gain;
    uint8_t d_gain;
    uint8_t i_gain;
    uint8_t operating_mode;
    uint16_t present_position;
    uint16_t goal_position;
    uint16_t goal_speed;
    uint16_t torque_limit;
} RightServoConfigureSnapshot;

typedef enum
{
    RIGHT_SERVO_DISABLE_OK = 0,
    RIGHT_SERVO_DISABLE_UNAVAILABLE = 1,
    RIGHT_SERVO_DISABLE_WRITE_FAILED = 2,
    RIGHT_SERVO_DISABLE_READBACK_FAILED = 3,
    RIGHT_SERVO_DISABLE_TORQUE_REMAINS_ENABLED = 4
} RightServoDisableStatus;

typedef struct
{
    RightServoDisableStatus status;
    uint8_t joint_count;
    uint8_t torque_enabled_mask;
    uint8_t failure_count;
} RightServoDisableSnapshot;

typedef struct
{
    uint16_t maximum_error_raw[RIGHT_SERVO_BUS_JOINT_COUNT];
    uint16_t last_position_raw;
    uint16_t last_commanded_raw;
    uint8_t last_joint_index;
    uint32_t requested_samples;
    uint32_t completed_samples;
    uint32_t failed_samples;
    uint32_t maximum_reply_latency_ms;
} RightServoInMotionTelemetrySnapshot;

void RightServoBus_Init(UART_HandleTypeDef *uart);
const RightServoDiscoverySnapshot *RightServoBus_Discover(void);
RightServoJogSnapshot RightServoBus_JogOnce(uint8_t servo_id, int8_t delta_raw);
RightServoTorqueEnableSnapshot RightServoBus_EnableTorqueAtPresentPositionOnce(
    uint8_t servo_id
);
RightServoConfigurationSnapshot RightServoBus_ReadConfiguration(
    uint8_t servo_id
);
RightServoConfigureSnapshot RightServoBus_ConfigureAtPresentPositionOnce(
    uint8_t servo_id,
    uint8_t p_gain,
    uint8_t d_gain,
    uint16_t goal_speed,
    uint16_t torque_limit
);
RightServoDisableSnapshot RightServoBus_DisableTorqueAllVerified(void);
HAL_StatusTypeDef RightServoBus_DisableTorqueAll(void);

#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
HAL_StatusTypeDef RightServoBus_InMotionTelemetryBegin(void);
void RightServoBus_InMotionTelemetryEnd(void);
uint8_t RightServoBus_InMotionTelemetryPending(void);
HAL_StatusTypeDef RightServoBus_InMotionTelemetryStart(
    uint8_t joint_index, uint32_t started_at_ms);
HAL_StatusTypeDef RightServoBus_InMotionTelemetryPoll(
    uint32_t now_ms,
    const uint16_t commanded_positions[RIGHT_SERVO_BUS_JOINT_COUNT]);
void RightServoBus_InMotionTelemetryOnTxComplete(UART_HandleTypeDef *uart);
void RightServoBus_InMotionTelemetryOnRxEvent(
    UART_HandleTypeDef *uart, uint16_t received);
void RightServoBus_InMotionTelemetryOnUartError(UART_HandleTypeDef *uart);
const RightServoInMotionTelemetrySnapshot *
RightServoBus_InMotionTelemetryGetSnapshot(void);
#endif

#endif /* RIGHT_SERVO_BUS_H */
