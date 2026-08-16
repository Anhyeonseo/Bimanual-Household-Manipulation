#include "binary_control.h"

#include "f0_metrics.h"
#include "control_tick.h"
#include "host_uart_tx.h"
#include "single_arm_config.h"
#include "right_servo_bus.h"
#include "servo_bus.h"
#include "timebase.h"
#include "actuator_core/buffered_command_route.h"
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
#include "bimanual_servo_dispatch.h"
#include "bimanual_operational_limits.h"
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
#include "bimanual_tracking_feedback.h"
#endif
#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
#include "bimanual_feedback_snapshot.h"
#endif
#endif
#include "actuator_core/calibration.h"
#include "actuator_core/joint_unwrap.h"
#include "actuator_core/crc32c.h"
#include "actuator_core/protocol.h"
#include "actuator_core/safety.h"
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
#include "actuator_core/stream_executor_v2.h"
#elif HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
#include "actuator_core/stream_session_v2.h"
#endif
#if HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD && !HOST_BIMANUAL_DMA_DISPATCH_BUILD
#include "bimanual_operational_limits.h"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* F0 metrics stay terminal-only; F2 restores the 60-byte lateness profile to
 * refill acknowledgements without charging its wire time to the control loop. */
#define HOST_F0_TERMINAL_METRICS_SIZE 16U
#define HOST_H2_TERMINAL_TELEMETRY_SIZE 28U
#define HOST_F3_CONTROL_TICK_METRICS_SIZE 16U

#if (ACTUATOR_BUFFERED_STATUS_LATENESS_SIZE + \
     HOST_F0_TERMINAL_METRICS_SIZE + \
     HOST_H2_TERMINAL_TELEMETRY_SIZE + \
     HOST_F3_CONTROL_TICK_METRICS_SIZE) > ACTUATOR_PROTOCOL_MAX_PAYLOAD
#error "Terminal telemetry exceeds the actuator protocol payload"
#endif

typedef struct
{
    uint8_t active;
    uint8_t starting;
    uint8_t verifying;
    uint8_t verify_consecutive;
    uint8_t verify_sweep_active;
    uint32_t request_sequence;
    uint32_t start_tick;
    uint32_t duration_ms;
    uint32_t last_control_tick;
    uint32_t verify_start_tick;
    uint16_t start_positions[SINGLE_ARM_JOINT_COUNT];
    uint16_t target_positions[SINGLE_ARM_JOINT_COUNT];
    ServoPositionSweep start_sweep;
    ServoPositionSweep verify_sweep;
} HostBinaryMotion;

typedef struct
{
    uint8_t active;
    uint8_t last_step_valid;
    uint8_t next_telemetry_joint;
    uint32_t request_sequence;
    uint32_t anchor_tick;
    uint32_t last_step_tick;
    int32_t anchor_positions_urad[SINGLE_ARM_JOINT_COUNT];
} HostBinaryBufferedMotion;

static UART_HandleTypeDef *binary_host_uart = NULL;
static volatile uint8_t host_stop_latched = 0U;
static actuator_stream_parser_t host_binary_parser;
static uint32_t host_binary_heartbeat_count = 0U;
static uint32_t host_binary_rejected_frame_count = 0U;
static uint32_t host_binary_last_heartbeat_ms = 0U;
static uint8_t host_binary_mode = 0U;
static actuator_safety_t host_binary_safety;
static HostBinaryMotion host_binary_motion;
static uint8_t host_binary_servos_configured = 0U;
/*
 * Host frame transmit accounting.
 *
 * F2 copies every encoded response into a bounded DMA queue. The wire still
 * preserves frame order, but the cooperative control loop is no longer held
 * for the full UART duration. Queue/DMA failures are latched and fail closed
 * on the next service iteration.
 */
static uint16_t host_tx_failure_count = 0U;
static uint16_t host_tx_timeout_count = 0U;
static uint16_t host_tx_maximum_ms = 0U;
static uint8_t host_tx_last_status = 0U;
static uint8_t host_position_read_failure_streak = 0U;
static uint8_t host_position_read_failed_servo_id = 0U;
static actuator_buffered_command_route_t host_buffered_validation_route;
static uint8_t host_buffered_validation_route_ready = 0U;
static actuator_buffered_command_route_t host_buffered_execution_route;
static uint8_t host_buffered_execution_route_ready = 0U;
static HostBinaryBufferedMotion host_binary_buffered_motion;
static uint8_t host_right_arm_output_active = 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
static uint32_t host_bimanual_arm_watchdog_grace_started_ms = 0U;
#endif
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
static actuator_v2_stream_executor_t host_v2_stream_executor;
static uint8_t host_v2_executor_ready = 0U;
static uint8_t host_v2_executor_clock_active = 0U;
static uint32_t host_v2_executor_next_tick = 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
static int32_t host_v2_output_urad[ACTUATOR_V2_JOINT_COUNT];
static uint8_t host_v2_coordinated_stop_pending = 0U;
static uint8_t host_v2_executor_start_pending = 0U;
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
static uint8_t host_v2_tracking_next_joint = 0U;
static uint32_t host_v2_tracking_last_dispatch_completed = 0U;
static uint16_t host_v2_last_left_raw[SINGLE_ARM_JOINT_COUNT];
static uint16_t host_v2_last_right_raw[SINGLE_ARM_JOINT_COUNT];
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
static uint8_t host_v2_terminal_settle_active = 0U;
static uint32_t host_v2_terminal_settle_baseline_completed_pairs = 0U;
static uint32_t host_v2_terminal_settle_started_ms = 0U;
#endif
#if HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
static uint8_t host_v2_tracking_fault_injection_consumed = 0U;
#endif
#endif
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD || \
    HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
static uint8_t host_v2_last_coordinated_stop_status = 2U;
#endif
#else
static int32_t host_v2_discarded_output_urad[ACTUATOR_V2_JOINT_COUNT];
#endif
#if HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
static uint8_t host_v2_shadow_anchor_ready = 0U;
static uint16_t host_v2_shadow_raw[ACTUATOR_V2_JOINT_COUNT];
static int32_t host_v2_shadow_anchor_urad[ACTUATOR_V2_JOINT_COUNT];
static int32_t host_v2_shadow_unwrapped_raw[ACTUATOR_V2_JOINT_COUNT];
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
static actuator_joint_unwrapper_t
    host_v2_shadow_unwrappers[ACTUATOR_V2_JOINT_COUNT];
#endif
static int32_t
    host_v2_shadow_executor_anchor_urad[ACTUATOR_V2_JOINT_COUNT];
#endif
#elif HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
static actuator_v2_stream_session_t host_v2_stream_session;
#endif

static void Host_WriteU32Le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void Host_WriteU16Le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static uint16_t Host_ReadU16Le(const uint8_t *source)
{
    return (uint16_t)(
        (uint16_t)source[0] |
        ((uint16_t)source[1] << 8U)
    );
}

static uint32_t Host_ReadU32Le(const uint8_t *source)
{
    return (uint32_t)source[0] |
        ((uint32_t)source[1] << 8U) |
        ((uint32_t)source[2] << 16U) |
        ((uint32_t)source[3] << 24U);
}

static int32_t Host_ReadI32Le(const uint8_t *source)
{
    return (int32_t)Host_ReadU32Le(source);
}

static actuator_joint_calibration_t Host_JointCalibration(
    uint8_t joint_index
)
{
    actuator_joint_calibration_t calibration = {
        servo_joints[joint_index].home_position,
        servo_joints[joint_index].min_position,
        servo_joints[joint_index].max_position,
        servo_joints[joint_index].test_direction
    };

    return calibration;
}

static uint32_t Host_CalibrationHash(void)
{
    uint8_t calibration_bytes[60] = {0U};
    uint16_t offset = 0U;

    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        const ServoJointConfig *joint = &servo_joints[i];

        calibration_bytes[offset++] = joint->id;
        calibration_bytes[offset++] =
            (uint8_t)(joint->home_position & 0xFFU);
        calibration_bytes[offset++] =
            (uint8_t)((joint->home_position >> 8U) & 0xFFU);
        calibration_bytes[offset++] =
            (uint8_t)(joint->min_position & 0xFFU);
        calibration_bytes[offset++] =
            (uint8_t)((joint->min_position >> 8U) & 0xFFU);
        calibration_bytes[offset++] =
            (uint8_t)(joint->max_position & 0xFFU);
        calibration_bytes[offset++] =
            (uint8_t)((joint->max_position >> 8U) & 0xFFU);
        calibration_bytes[offset++] = (uint8_t)joint->test_direction;
        calibration_bytes[offset++] = joint->p_gain;
        calibration_bytes[offset++] = joint->d_gain;
    }

    return actuator_crc32c(calibration_bytes, offset);
}

static uint8_t Host_InitBufferedRoute(
    actuator_buffered_command_route_t *route,
    uint8_t minimum_start_samples,
    uint32_t maximum_apply_lateness_ms
)
{
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];

    if (route == NULL)
    {
        return 0U;
    }

    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        const actuator_joint_calibration_t calibration =
            Host_JointCalibration(joint);
        int32_t first_limit = 0;
        int32_t second_limit = 0;

        if ((actuator_raw_to_urad(
                 &calibration,
                 servo_joints[joint].min_position,
                 &first_limit
             ) != ACTUATOR_CALIBRATION_OK) ||
            (actuator_raw_to_urad(
                 &calibration,
                 servo_joints[joint].max_position,
                 &second_limit
             ) != ACTUATOR_CALIBRATION_OK))
        {
            return 0U;
        }

        if (first_limit <= second_limit)
        {
            limits[joint].minimum_urad = first_limit;
            limits[joint].maximum_urad = second_limit;
        }
        else
        {
            limits[joint].minimum_urad = second_limit;
            limits[joint].maximum_urad = first_limit;
        }
    }

    return (actuator_buffered_command_route_init(
                route,
                minimum_start_samples,
                maximum_apply_lateness_ms,
                limits
            ) == ACTUATOR_BUFFERED_OK) ? 1U : 0U;
}

static uint8_t Host_InitBufferedValidationRoute(void)
{
    return Host_InitBufferedRoute(
        &host_buffered_validation_route,
        HOST_BUFFERED_VALIDATION_MINIMUM_START_SAMPLES,
        HOST_BUFFERED_VALIDATION_MAXIMUM_APPLY_LATENESS_MS
    );
}

static uint8_t Host_InitBufferedExecutionRoute(void)
{
    return Host_InitBufferedRoute(
        &host_buffered_execution_route,
        HOST_BUFFERED_EXECUTION_MINIMUM_START_SAMPLES,
        HOST_BUFFERED_EXECUTION_MAXIMUM_APPLY_LATENESS_MS
    );
}

static uint8_t Host_BufferedExecutionIsActive(void)
{
    return host_binary_buffered_motion.active;
}

static uint32_t Host_BinaryCapabilities(void)
{
    uint32_t capabilities = HOST_BINARY_CAPABILITIES;

    if (host_buffered_validation_route_ready == 0U)
    {
        capabilities &= ~HOST_BUFFERED_VALIDATION_CAPABILITY;
    }
    if (host_buffered_execution_route_ready == 0U)
    {
        capabilities &= ~HOST_BUFFERED_EXECUTION_CAPABILITY;
    }
    return capabilities;
}

static HAL_StatusTypeDef Host_SendBinaryFrame(
    const actuator_frame_t *frame
)
{
    uint8_t encoded[ACTUATOR_PROTOCOL_MAX_ENCODED_SIZE] = {0U};
    size_t encoded_length = 0U;

    if (actuator_frame_encode(
            frame,
            encoded,
            sizeof(encoded),
            &encoded_length
        ) != ACTUATOR_PROTOCOL_OK)
    {
        return HAL_ERROR;
    }

    uint32_t started = HAL_GetTick();
    uint32_t started_us = Timebase_NowUs();
    HAL_StatusTypeDef status = HostUartTx_Enqueue(
        encoded, (uint16_t)encoded_length
    );
    uint32_t elapsed = (uint32_t)(HAL_GetTick() - started);
    F0Metrics_ObserveHostTx(Timebase_ElapsedUs(started_us));

    if (elapsed > (uint32_t)host_tx_maximum_ms)
    {
        host_tx_maximum_ms = (elapsed > UINT16_MAX) ?
            UINT16_MAX : (uint16_t)elapsed;
    }
    host_tx_last_status = (uint8_t)status;
    if (status != HAL_OK)
    {
        if (host_tx_failure_count < UINT16_MAX)
        {
            host_tx_failure_count++;
        }
        if ((status == HAL_TIMEOUT) &&
            (host_tx_timeout_count < UINT16_MAX))
        {
            /*
             * A timeout means the frame was cut mid-transmission: the host
             * sees a partial packet and the rest arrives as an undecodable
             * tail. This is the counter that proves it from the MCU side.
             */
            host_tx_timeout_count++;
        }
    }
    return status;
}

static void Host_SendBinaryState(
    uint32_t request_sequence,
    uint8_t status_code
)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_STATE_FEEDBACK;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length =
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        ACTUATOR_V2_STATE_WIRE_SIZE;
#else
        20U;
#endif
    response.payload[0] = (host_stop_latched != 0U) ? 1U : 0U;
    response.payload[1] = status_code;
    response.payload[2] =
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        HOST_BINARY_JOINT_COUNT;
#else
        servo_joint_count;
#endif
    response.payload[3] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    Host_WriteU32Le(&response.payload[4], host_binary_heartbeat_count);
    Host_WriteU32Le(&response.payload[8], host_binary_rejected_frame_count);
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
    /* The reviewed right-arm candidate currently has the same packed values,
     * but v2 binds and reports both arm identities independently. */
    Host_WriteU32Le(&response.payload[16], Host_CalibrationHash());
    Host_WriteU32Le(
        &response.payload[20], host_binary_last_heartbeat_ms);
#else
    Host_WriteU32Le(
        &response.payload[16], host_binary_last_heartbeat_ms);
#endif

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryPositionReadFailure(
    uint32_t request_sequence
)
{
    actuator_frame_t response;
    const ServoBusDiagnostics *bus = ServoBus_GetDiagnostics();
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_STATE_FEEDBACK;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 58U;
    response.payload[0] = (host_stop_latched != 0U) ? 1U : 0U;
    response.payload[1] = 2U;
    response.payload[2] = servo_joint_count;
    response.payload[3] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    Host_WriteU32Le(&response.payload[4], host_binary_heartbeat_count);
    Host_WriteU32Le(&response.payload[8], host_binary_rejected_frame_count);
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());
    Host_WriteU32Le(
        &response.payload[16],
        host_binary_last_heartbeat_ms
    );
    response.payload[20] = host_position_read_failed_servo_id;
    response.payload[21] = host_position_read_failure_streak;
    response.payload[22] = HOST_POSITION_READ_FAILURE_LIMIT;
    response.payload[23] = (uint8_t)bus->reason;
    response.payload[24] = bus->hal_status;
    response.payload[25] = bus->servo_status;
    Host_WriteU16Le(
        &response.payload[26],
        (uint16_t)bus->recovery_count
    );
    Host_WriteU16Le(&response.payload[28], bus->discarded_bytes);
    response.payload[30] = 0U;
    response.payload[31] = 0U;
    Host_WriteU32Le(
        &response.payload[32],
        bus->uart_error_code
    );
    Host_WriteU32Le(
        &response.payload[36],
        bus->uart_isr
    );
    response.payload[40] = bus->snapshot_length;
    response.payload[41] = ServoBus_GetHealth()->dma_started;
    memcpy(
        &response.payload[42],
        bus->snapshot,
        SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES
    );

    (void)Host_SendBinaryFrame(&response);
}

static void Host_ResetPositionReadFailure(void)
{
    host_position_read_failure_streak = 0U;
    host_position_read_failed_servo_id = 0U;
}

static void Host_SendBinaryStateWithPositions(
    uint32_t request_sequence
)
{
    uint16_t positions[SINGLE_ARM_JOINT_COUNT] = {0U};

    if (Servo_ReadAllPositions(positions) != HAL_OK)
    {
        host_position_read_failed_servo_id =
            servo_last_all_read_failed_id;
        if (host_position_read_failure_streak < UINT8_MAX)
        {
            host_position_read_failure_streak++;
        }
        if (host_position_read_failure_streak >=
            HOST_POSITION_READ_FAILURE_LIMIT)
        {
            host_stop_latched = 1U;
            if (actuator_safety_accepts_setpoint(&host_binary_safety))
            {
                (void)actuator_safety_request_hold(&host_binary_safety);
            }
        }
        Host_SendBinaryPositionReadFailure(request_sequence);
        return;
    }

    Host_ResetPositionReadFailure();

    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_STATE_FEEDBACK;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length =
        20U + (2U * SINGLE_ARM_JOINT_COUNT);
    response.payload[0] = (host_stop_latched != 0U) ? 1U : 0U;
    response.payload[1] = 0U;
    response.payload[2] = servo_joint_count;
    response.payload[3] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    Host_WriteU32Le(&response.payload[4], host_binary_heartbeat_count);
    Host_WriteU32Le(&response.payload[8], host_binary_rejected_frame_count);
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());
    Host_WriteU32Le(
        &response.payload[16],
        host_binary_last_heartbeat_ms
    );

    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        Host_WriteU16Le(
            &response.payload[20U + ((uint16_t)joint * 2U)],
            positions[joint]
        );
    }

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryDiagnostics(
    uint32_t request_sequence,
    uint8_t joint_index
)
{
    actuator_frame_t response;
    uint8_t pid[3] = {0U};
    uint8_t runtime[10] = {0U};
    uint8_t identity[5] = {0U};
    uint8_t protection[27] = {0U};
    uint16_t position = 0U;
    uint16_t speed_raw = 0U;
    uint16_t load_raw = 0U;
    uint8_t voltage_raw = 0U;
    uint8_t temperature_c = 0U;
    uint16_t current_raw = 0U;
    uint8_t read_status = 0U;
    uint8_t failure_captured = 0U;
    ServoBusDiagnostics first_failure = {0};
    const ServoJointConfig *joint = &servo_joints[joint_index];

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_DIAGNOSTICS;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 146U;

    /*
     * Keep each request bounded to one servo. The host refreshes the heartbeat
     * between joints, so a complete six-joint snapshot cannot starve the
     * 500 ms host watchdog even when one bus read reaches its timeout.
     */
    if ((host_binary_motion.active != 0U) ||
        (Host_BufferedExecutionIsActive() != 0U))
    {
        read_status = UINT8_C(0x80);
    }
    else
    {
        if (Servo_ReadData(
                joint->id,
                21U,
                sizeof(pid),
                pid
            ) != HAL_OK)
        {
            read_status |= UINT8_C(0x01);
            first_failure = *ServoBus_GetDiagnostics();
            failure_captured = 1U;
        }

        if (Servo_ReadData(
                joint->id,
                40U,
                sizeof(runtime),
                runtime
            ) != HAL_OK)
        {
            read_status |= UINT8_C(0x02);
            if (failure_captured == 0U)
            {
                first_failure = *ServoBus_GetDiagnostics();
                failure_captured = 1U;
            }
        }

        if (Servo_ReadTelemetry(
                joint->id,
                &position,
                &speed_raw,
                &load_raw,
                &voltage_raw,
                &temperature_c,
                &current_raw
            ) != HAL_OK)
        {
            read_status |= UINT8_C(0x04);
            if (failure_captured == 0U)
            {
                first_failure = *ServoBus_GetDiagnostics();
                failure_captured = 1U;
            }
        }

        /*
         * Model identity and the non-volatile protection block distinguish
         * an SO-101 follower motor/configuration mismatch from a genuine
         * payload problem. Goal_Position is already present in runtime[2:3],
         * so exposing it adds no extra servo transaction.
         */
        if (Servo_ReadData(
                joint->id,
                0U,
                sizeof(identity),
                identity
            ) != HAL_OK)
        {
            read_status |= UINT8_C(0x08);
            if (failure_captured == 0U)
            {
                first_failure = *ServoBus_GetDiagnostics();
                failure_captured = 1U;
            }
        }

        /*
         * Servo_ReadData deliberately caps one transaction at 16 bytes.
         * Split EEPROM 13..39 into 13..28 and 29..39; a 27-byte request is
         * rejected locally before it ever reaches the STS3215 bus.
         */
        if ((Servo_ReadData(
                 joint->id,
                 13U,
                 16U,
                 &protection[0]
             ) != HAL_OK) ||
            (Servo_ReadData(
                 joint->id,
                 29U,
                 11U,
                 &protection[16]
             ) != HAL_OK))
        {
            read_status |= UINT8_C(0x10);
            if (failure_captured == 0U)
            {
                first_failure = *ServoBus_GetDiagnostics();
                failure_captured = 1U;
            }
        }
    }

    response.payload[0] = (read_status == 0U) ? 0U : 2U;
    response.payload[1] = joint_index;
    response.payload[2] = servo_joint_count;
    response.payload[3] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    Host_WriteU32Le(&response.payload[4], Host_CalibrationHash());
    Host_WriteU32Le(&response.payload[8], HAL_GetTick());
    response.payload[12] = joint->id;
    response.payload[13] = read_status;
    response.payload[14] = runtime[0];
    response.payload[15] = pid[0];
    response.payload[16] = pid[1];
    response.payload[17] = pid[2];
    response.payload[18] = voltage_raw;
    response.payload[19] = temperature_c;
    Host_WriteU16Le(&response.payload[20], position);
    Host_WriteU16Le(&response.payload[22], speed_raw);
    Host_WriteU16Le(&response.payload[24], load_raw);
    Host_WriteU16Le(&response.payload[26], current_raw);
    Host_WriteU16Le(
        &response.payload[28],
        (uint16_t)(
            (uint16_t)runtime[8] |
            ((uint16_t)runtime[9] << 8U)
        )
    );
    Host_WriteU16Le(
        &response.payload[30],
        (uint16_t)(
            (uint16_t)runtime[2] |
            ((uint16_t)runtime[3] << 8U)
        )
    );
    Host_WriteU16Le(
        &response.payload[32],
        (uint16_t)(
            (uint16_t)identity[3] |
            ((uint16_t)identity[4] << 8U)
        )
    );
    response.payload[34] = identity[0];
    response.payload[35] = identity[1];
    Host_WriteU16Le(
        &response.payload[36],
        (uint16_t)(
            (uint16_t)protection[3] |
            ((uint16_t)protection[4] << 8U)
        )
    );
    Host_WriteU16Le(
        &response.payload[38],
        (uint16_t)(
            (uint16_t)protection[11] |
            ((uint16_t)protection[12] << 8U)
        )
    );
    response.payload[40] = protection[13];
    response.payload[41] = protection[14];
    Host_WriteU16Le(
        &response.payload[42],
        (uint16_t)(
            (uint16_t)protection[15] |
            ((uint16_t)protection[16] << 8U)
        )
    );
    response.payload[44] = protection[20];
    response.payload[45] = protection[21];
    response.payload[46] = protection[22];
    response.payload[47] = protection[23];

    const ServoBusDiagnostics *bus = (failure_captured != 0U)
        ? &first_failure
        : ServoBus_GetDiagnostics();
    const ServoBusHealth *health = ServoBus_GetHealth();
    response.payload[48] = 2U;
    response.payload[49] = (uint8_t)bus->reason;
    response.payload[50] = bus->hal_status;
    response.payload[51] = bus->servo_status;
    response.payload[52] = health->dma_started;
    response.payload[53] = health->last_rx_event;
    response.payload[54] = (bus->received_bytes > UINT8_MAX)
        ? UINT8_MAX
        : (uint8_t)bus->received_bytes;
    response.payload[55] = (uint8_t)health->producer_index;
    Host_WriteU32Le(&response.payload[56], bus->uart_error_code);
    Host_WriteU32Le(&response.payload[60], bus->uart_isr);
    Host_WriteU32Le(&response.payload[64], bus->dma_error_code);
    Host_WriteU32Le(&response.payload[68], health->transaction_count);
    Host_WriteU32Le(&response.payload[72], health->success_count);
    Host_WriteU32Le(&response.payload[76], health->failure_count);
    Host_WriteU32Le(&response.payload[80], health->recovery_count);
    Host_WriteU32Le(&response.payload[84], health->discarded_bytes);
    Host_WriteU32Le(&response.payload[88], health->timeout_count);
    Host_WriteU32Le(&response.payload[92], health->overflow_count);
    Host_WriteU32Le(&response.payload[96], health->rx_event_count);
    Host_WriteU16Le(&response.payload[100], health->pe_count);
    Host_WriteU16Le(&response.payload[102], health->ne_count);
    Host_WriteU16Le(&response.payload[104], health->fe_count);
    Host_WriteU16Le(&response.payload[106], health->ore_count);
    Host_WriteU16Le(&response.payload[108], health->rto_count);
    Host_WriteU16Le(&response.payload[110], health->dma_error_count);
    Host_WriteU32Le(&response.payload[112], health->lazy_arm_count);
    Host_WriteU32Le(&response.payload[116], health->receiver_resync_count);
    Host_WriteU16Le(&response.payload[138], host_tx_failure_count);
    Host_WriteU16Le(&response.payload[140], host_tx_timeout_count);
    Host_WriteU16Le(&response.payload[142], host_tx_maximum_ms);
    response.payload[144] = host_tx_last_status;
    response.payload[145] = 0U;
    response.payload[120] = bus->snapshot_length;
    response.payload[121] = health->dma_started;
    memcpy(
        &response.payload[122],
        bus->snapshot,
        SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES
    );

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendRightArmDiscovery(uint32_t request_sequence)
{
    actuator_frame_t response;
    const RightServoDiscoverySnapshot *snapshot = NULL;
    uint8_t status_code = 0U;

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 32U;
    response.payload[1] = RIGHT_SERVO_BUS_JOINT_COUNT;

    /* A polling scan can wait 20 ms per bus ID, so it must never run while
     * either left-arm executor owns the cooperative control loop. */
    if ((host_binary_motion.active != 0U) ||
        (Host_BufferedExecutionIsActive() != 0U))
    {
        status_code = 1U;
    }
    else
    {
        snapshot = RightServoBus_Discover();
        if (snapshot == NULL)
        {
            status_code = 1U;
        }
        else
        {
            response.payload[2] = snapshot->present_mask;
            memcpy(&response.payload[4], snapshot->positions, 12U);
            memcpy(&response.payload[16], snapshot->statuses, 6U);
            Host_WriteU32Le(&response.payload[24], snapshot->transaction_count);
            Host_WriteU32Le(&response.payload[28], snapshot->failure_count);
            status_code = (snapshot->present_mask == UINT8_C(0x3F)) ? 0U : 2U;
        }
    }
    response.payload[0] = status_code;
    (void)Host_SendBinaryFrame(&response);
}

static uint8_t Host_RightArmOneShotPermitted(void)
{
    return ((host_binary_safety.state == ACTUATOR_STATE_SAFE_DISABLED) &&
            (host_stop_latched == 0U) &&
            (host_binary_heartbeat_count != 0U) &&
            ((uint32_t)(HAL_GetTick() - host_binary_last_heartbeat_ms) <=
                HOST_BINARY_HEARTBEAT_TIMEOUT_MS) &&
            (host_binary_motion.active == 0U) &&
            (Host_BufferedExecutionIsActive() == 0U)) ? 1U : 0U;
}

static void Host_SendRightArmJogOnce(
    uint32_t request_sequence, uint8_t servo_id, int8_t delta_raw)
{
    actuator_frame_t response;
    RightServoJogSnapshot snapshot = {0};
    uint8_t status_code = 0U;

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 12U;

    if (Host_RightArmOneShotPermitted() == 0U)
    {
        status_code = 1U;
    }
    else
    {
        snapshot = RightServoBus_JogOnce(servo_id, delta_raw);
        status_code = (uint8_t)snapshot.status;
        if ((snapshot.status == RIGHT_SERVO_JOG_OK) ||
            (snapshot.status == RIGHT_SERVO_JOG_POST_READ_FAILED))
        {
            host_right_arm_output_active = 1U;
        }
    }
    response.payload[0] = status_code;
    response.payload[1] = servo_id;
    response.payload[2] = (uint8_t)delta_raw;
    response.payload[3] = snapshot.torque_enabled;
    Host_WriteU16Le(&response.payload[4], snapshot.start_position);
    Host_WriteU16Le(&response.payload[6], snapshot.target_position);
    Host_WriteU16Le(&response.payload[8], snapshot.observed_position);
    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendRightArmTorqueEnableOnce(
    uint32_t request_sequence, uint8_t servo_id)
{
    actuator_frame_t response;
    RightServoTorqueEnableSnapshot snapshot = {0};
    uint8_t status_code = 0U;

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 12U;

    if (Host_RightArmOneShotPermitted() == 0U)
    {
        status_code = RIGHT_SERVO_TORQUE_ENABLE_UNAVAILABLE;
    }
    else
    {
        snapshot = RightServoBus_EnableTorqueAtPresentPositionOnce(servo_id);
        status_code = (uint8_t)snapshot.status;
        if ((snapshot.status == RIGHT_SERVO_TORQUE_ENABLE_OK) ||
            (snapshot.status == RIGHT_SERVO_TORQUE_ALREADY_ENABLED))
        {
            host_right_arm_output_active = 1U;
        }
    }
    response.payload[0] = status_code;
    response.payload[1] = servo_id;
    response.payload[2] = snapshot.torque_enabled;
    response.payload[3] = 0U;
    Host_WriteU16Le(&response.payload[4], snapshot.present_position);
    Host_WriteU16Le(&response.payload[6], snapshot.held_goal_position);
    Host_WriteU16Le(&response.payload[8], snapshot.observed_position);
    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendRightArmConfigureOnce(
    uint32_t request_sequence, uint8_t servo_id)
{
    actuator_frame_t response;
    RightServoConfigureSnapshot snapshot = {0};

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 16U;

    if ((Host_RightArmOneShotPermitted() == 0U) ||
        (servo_id == 0U) || (servo_id > servo_joint_count))
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_UNAVAILABLE;
        snapshot.servo_id = servo_id;
    }
    else
    {
        const ServoJointConfig *joint = &servo_joints[servo_id - 1U];
        host_right_arm_output_active = 1U;
        snapshot = RightServoBus_ConfigureAtPresentPositionOnce(
            servo_id,
            joint->p_gain,
            joint->d_gain,
            SERVO_GOAL_SPEED_RAW,
            joint->torque_limit
        );
    }

    response.payload[0] = (uint8_t)snapshot.status;
    response.payload[1] = snapshot.servo_id;
    response.payload[2] = snapshot.torque_enabled;
    response.payload[3] = snapshot.p_gain;
    response.payload[4] = snapshot.d_gain;
    response.payload[5] = snapshot.i_gain;
    response.payload[6] = snapshot.operating_mode;
    response.payload[7] = 0U;
    Host_WriteU16Le(&response.payload[8], snapshot.present_position);
    Host_WriteU16Le(&response.payload[10], snapshot.goal_position);
    Host_WriteU16Le(&response.payload[12], snapshot.goal_speed);
    Host_WriteU16Le(&response.payload[14], snapshot.torque_limit);
    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendRightArmConfiguration(
    uint32_t request_sequence, uint8_t servo_id)
{
    actuator_frame_t response;
    RightServoConfigurationSnapshot snapshot = {0};

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 48U;

    if ((host_binary_motion.active != 0U) ||
        (Host_BufferedExecutionIsActive() != 0U))
    {
        snapshot.status = 1U;
        snapshot.servo_id = servo_id;
    }
    else
    {
        snapshot = RightServoBus_ReadConfiguration(servo_id);
    }

    response.payload[0] = snapshot.status;
    response.payload[1] = snapshot.servo_id;
    response.payload[2] = snapshot.read_status;
    response.payload[3] = snapshot.successful_block_mask;
    Host_WriteU32Le(&response.payload[4], snapshot.sample_time_ms);
    response.payload[8] = snapshot.torque_enabled;
    response.payload[9] = snapshot.p_gain;
    response.payload[10] = snapshot.d_gain;
    response.payload[11] = snapshot.i_gain;
    response.payload[12] = snapshot.voltage_raw;
    response.payload[13] = snapshot.temperature_c;
    Host_WriteU16Le(&response.payload[14], snapshot.position_raw);
    Host_WriteU16Le(&response.payload[16], snapshot.speed_raw);
    Host_WriteU16Le(&response.payload[18], snapshot.load_raw);
    Host_WriteU16Le(&response.payload[20], snapshot.current_raw);
    Host_WriteU16Le(
        &response.payload[22], snapshot.runtime_torque_limit_raw);
    Host_WriteU16Le(&response.payload[24], snapshot.goal_position_raw);
    Host_WriteU16Le(&response.payload[26], snapshot.model_number);
    response.payload[28] = snapshot.firmware_major_version;
    response.payload[29] = snapshot.firmware_minor_version;
    Host_WriteU16Le(
        &response.payload[30], snapshot.maximum_torque_limit_raw);
    Host_WriteU16Le(
        &response.payload[32], snapshot.minimum_startup_force_raw);
    response.payload[34] = snapshot.cw_dead_zone_raw;
    response.payload[35] = snapshot.ccw_dead_zone_raw;
    Host_WriteU16Le(
        &response.payload[36], snapshot.protection_current_raw);
    response.payload[38] = snapshot.operating_mode;
    response.payload[39] = snapshot.protective_torque_raw;
    response.payload[40] = snapshot.protection_time_raw;
    response.payload[41] = snapshot.overload_torque_raw;
    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendRightArmDisableVerified(uint32_t request_sequence)
{
    actuator_frame_t response;
    RightServoDisableSnapshot snapshot = {
        .status = RIGHT_SERVO_DISABLE_UNAVAILABLE,
        .joint_count = RIGHT_SERVO_BUS_JOINT_COUNT,
        .torque_enabled_mask = 0U,
        .failure_count = RIGHT_SERVO_BUS_JOINT_COUNT,
    };

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_MSG_RIGHT_ARM_DISABLE_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 4U;

    if ((host_binary_motion.active == 0U) &&
        (Host_BufferedExecutionIsActive() == 0U))
    {
        snapshot = RightServoBus_DisableTorqueAllVerified();
    }

    if (snapshot.status == RIGHT_SERVO_DISABLE_OK)
    {
        host_right_arm_output_active = 0U;
    }
    else
    {
        host_stop_latched = 1U;
        actuator_safety_report_fault(
            &host_binary_safety, UINT16_C(0xFF04));
    }

    response.payload[0] = (uint8_t)snapshot.status;
    response.payload[1] = snapshot.joint_count;
    response.payload[2] = snapshot.torque_enabled_mask;
    response.payload[3] = snapshot.failure_count;
    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryHello(uint32_t request_sequence)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_HELLO_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length =
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        ACTUATOR_V2_HELLO_WIRE_SIZE;
#else
        20U;
#endif
    response.payload[0] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    response.payload[1] =
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        HOST_BINARY_JOINT_COUNT;
#else
        servo_joint_count;
#endif
    response.payload[2] = (host_stop_latched != 0U) ? 1U : 0U;
    response.payload[3] = 0U;
    Host_WriteU32Le(
        &response.payload[4],
        HOST_BINARY_FIRMWARE_VERSION
    );
    Host_WriteU32Le(
        &response.payload[8],
        Host_CalibrationHash()
    );
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());
    Host_WriteU32Le(&response.payload[16], Host_BinaryCapabilities());
    Host_WriteU32Le(
        &response.payload[20], host_binary_rejected_frame_count);
#else
    Host_WriteU32Le(&response.payload[12], Host_BinaryCapabilities());
    Host_WriteU32Le(
        &response.payload[16], host_binary_rejected_frame_count);
#endif

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryArmResponse(
    uint32_t request_sequence,
    actuator_safety_result_t result
)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_ARM_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 8U;
    response.payload[0] = (uint8_t)result;
    response.payload[1] = (uint8_t)host_binary_safety.state;
    response.payload[2] = 0U;
    response.payload[3] = 0U;
    Host_WriteU32Le(&response.payload[4], Host_CalibrationHash());

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinarySetpointStatus(
    uint32_t request_sequence,
    uint8_t status_code,
    uint8_t sample_count,
    uint32_t first_apply_tick,
    uint8_t detail
)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_SETPOINT_STATUS;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 16U;
    response.payload[0] = status_code;
    response.payload[1] = sample_count;
    response.payload[2] = (uint8_t)host_binary_safety.state;
    response.payload[3] = detail;
    Host_WriteU32Le(&response.payload[4], request_sequence);
    Host_WriteU32Le(&response.payload[8], first_apply_tick);
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryBufferedSetpointStatus(
    const actuator_buffered_command_route_t *route,
    uint32_t request_sequence,
    uint8_t status_code,
    uint8_t sample_count,
    uint32_t first_apply_tick,
    uint8_t detail
)
{
    actuator_frame_t response;
    size_t payload_length = 0U;
    const actuator_buffered_diagnostics_t *diagnostics = NULL;
    memset(&response, 0, sizeof(response));

    if (route != NULL)
    {
        diagnostics = actuator_buffered_executor_diagnostics(
            &route->executor
        );
    }

    response.message_type = ACTUATOR_MSG_SETPOINT_STATUS;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    /*
     * F2 validates the original 60-byte refill response under the DMA queue.
     * It used to be terminal-only because a blocking UART transmit consumed
     * the entire apply-lateness budget. The queue preserves wire order while
     * letting the cooperative loop return immediately after enqueue.
     */
    if (!actuator_buffered_status_encode(
            response.payload,
            sizeof(response.payload),
            &payload_length,
            status_code,
            sample_count,
            (uint8_t)host_binary_safety.state,
            detail,
            request_sequence,
            first_apply_tick,
            Host_CalibrationHash(),
            diagnostics,
            true
        ))
    {
        Host_SendBinarySetpointStatus(
            request_sequence,
            7U,
            sample_count,
            first_apply_tick,
            detail
        );
        return;
    }
    if (status_code == HOST_BUFFERED_STATUS_TERMINAL)
    {
        F0MetricsSnapshot snapshot = F0Metrics_Snapshot();
        Host_WriteU32Le(
            &response.payload[payload_length], snapshot.loop_period_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 4U], snapshot.loop_work_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 8U],
            snapshot.servo_sync_write_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 12U], snapshot.host_tx_max_us
        );
        payload_length += HOST_F0_TERMINAL_METRICS_SIZE;

        const ServoInMotionTelemetrySnapshot *telemetry =
            Servo_InMotionTelemetryGetSnapshot();
        for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
        {
            Host_WriteU16Le(
                &response.payload[payload_length + (joint * 2U)],
                telemetry->maximum_error_raw[joint]
            );
        }
        Host_WriteU32Le(
            &response.payload[payload_length + 12U],
            telemetry->requested_samples
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 16U],
            telemetry->completed_samples
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 20U],
            telemetry->failed_samples
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 24U],
            telemetry->maximum_reply_latency_ms
        );
        payload_length += HOST_H2_TERMINAL_TELEMETRY_SIZE;

        const ControlTickSnapshot tick = ControlTick_Snapshot();
        Host_WriteU32Le(
            &response.payload[payload_length], tick.period_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 4U], tick.jitter_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 8U], tick.work_max_us
        );
        Host_WriteU32Le(
            &response.payload[payload_length + 12U], tick.count
        );
        payload_length += HOST_F3_CONTROL_TICK_METRICS_SIZE;
    }
    response.payload_length = (uint16_t)payload_length;
    (void)Host_SendBinaryFrame(&response);
}

static void Host_StartBinaryMotion(
    const actuator_frame_t *request,
    uint32_t first_apply_tick,
    const uint16_t target_positions[6]
)
{
    uint32_t now = HAL_GetTick();
    uint32_t duration_ms = first_apply_tick - now;

    if ((host_binary_motion.active != 0U) ||
        (duration_ms < 20U) ||
        (duration_ms > 2000U))
    {
        Host_SendBinarySetpointStatus(
            request->sequence,
            2U,
            1U,
            first_apply_tick,
            0U
        );
        return;
    }

    if (host_binary_servos_configured == 0U)
    {
        host_stop_latched = 1U;
        Host_SendBinarySetpointStatus(
            request->sequence,
            7U,
            1U,
            first_apply_tick,
            0U
        );
        return;
    }

    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        host_binary_motion.target_positions[joint] = target_positions[joint];
    }

    /*
     * Accept and reserve the goal before reading six start positions. The
     * cooperative sweep performs only one bounded servo transaction per main
     * loop, so acknowledged heartbeats can be serviced between every attempt.
     */
    host_binary_motion.request_sequence = request->sequence;
    host_binary_motion.start_tick = now;
    host_binary_motion.duration_ms = duration_ms;
    host_binary_motion.last_control_tick = now;
    host_binary_motion.verify_start_tick = 0U;
    host_binary_motion.verify_consecutive = 0U;
    host_binary_motion.verify_sweep_active = 0U;
    host_binary_motion.verifying = 0U;
    host_binary_motion.starting = 1U;
    host_binary_motion.active = 1U;
    Servo_PositionSweepBegin(&host_binary_motion.start_sweep);

    Host_SendBinarySetpointStatus(
        request->sequence,
        0U,
        1U,
        first_apply_tick,
        0U
    );
}

static void Host_ServiceBinaryMotion(void)
{
    const uint32_t control_period_ms = 20U;
    uint32_t now;
    uint32_t elapsed;
    uint16_t setpoints[6] = {0U};

    if (host_binary_motion.active == 0U)
    {
        return;
    }

    if ((host_stop_latched != 0U) ||
        !actuator_safety_accepts_setpoint(&host_binary_safety))
    {
        host_binary_motion.active = 0U;
        Servo_MotionSafetyEnd();
        Host_SendBinarySetpointStatus(
            host_binary_motion.request_sequence,
            8U,
            1U,
            host_binary_motion.start_tick + host_binary_motion.duration_ms,
            (uint8_t)host_binary_safety.state
        );
        return;
    }

    if (host_binary_motion.starting != 0U)
    {
        HAL_StatusTypeDef start_status = Servo_PositionSweepStep(
            &host_binary_motion.start_sweep
        );
        if (start_status == HAL_BUSY)
        {
            return;
        }
        if (start_status != HAL_OK)
        {
            host_binary_motion.active = 0U;
            host_stop_latched = 1U;
            Host_SendBinarySetpointStatus(
                host_binary_motion.request_sequence,
                7U,
                1U,
                host_binary_motion.start_tick +
                    host_binary_motion.duration_ms,
                servo_last_all_read_failed_id
            );
            return;
        }

        memcpy(
            host_binary_motion.start_positions,
            host_binary_motion.start_sweep.positions,
            sizeof(host_binary_motion.start_positions)
        );
        host_binary_motion.start_tick = HAL_GetTick();
        host_binary_motion.last_control_tick = host_binary_motion.start_tick;
        host_binary_motion.starting = 0U;
        Servo_MotionSafetyBegin(
            (uint8_t)((1U << SINGLE_ARM_JOINT_COUNT) - 1U)
        );
        return;
    }

    now = HAL_GetTick();
    if (host_binary_motion.verifying != 0U)
    {
        HAL_StatusTypeDef safety_status = Servo_MotionSafetyPoll();
        if (safety_status != HAL_OK)
        {
            const ServoMotionSafetyDiagnostics *diagnostics =
                Servo_MotionSafetyGetDiagnostics();
            host_binary_motion.active = 0U;
            host_stop_latched = 1U;
            Servo_MotionSafetyEnd();
            Host_SendBinarySetpointStatus(
                host_binary_motion.request_sequence,
                9U,
                1U,
                host_binary_motion.start_tick +
                    host_binary_motion.duration_ms,
                diagnostics->servo_id
            );
            return;
        }

        now = HAL_GetTick();
        if (host_binary_motion.verify_sweep_active == 0U)
        {
            if ((now - host_binary_motion.last_control_tick) <
                SERVO_FINAL_SETTLE_SAMPLE_MS)
            {
                return;
            }
            Servo_PositionSweepBegin(&host_binary_motion.verify_sweep);
            host_binary_motion.verify_sweep_active = 1U;
        }

        HAL_StatusTypeDef verify_status = Servo_PositionSweepStep(
            &host_binary_motion.verify_sweep
        );
        if (verify_status == HAL_BUSY)
        {
            return;
        }
        if (verify_status != HAL_OK)
        {
            host_binary_motion.active = 0U;
            host_stop_latched = 1U;
            Servo_MotionSafetyEnd();
            Host_SendBinarySetpointStatus(
                host_binary_motion.request_sequence,
                7U,
                1U,
                host_binary_motion.start_tick +
                    host_binary_motion.duration_ms,
                servo_last_all_read_failed_id
            );
            return;
        }

        host_binary_motion.verify_sweep_active = 0U;
        now = HAL_GetTick();
        host_binary_motion.last_control_tick = now;

        uint16_t maximum_error = 0U;
        for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
        {
            int32_t error = Servo_PositionError(
                host_binary_motion.verify_sweep.positions[joint],
                host_binary_motion.target_positions[joint]
            );
            if (error < 0)
            {
                error = -error;
            }
            if ((uint16_t)error > maximum_error)
            {
                maximum_error = (uint16_t)error;
            }
        }

        if (maximum_error <= SERVO_FINAL_ERROR_TOLERANCE_RAW)
        {
            if (host_binary_motion.verify_consecutive < UINT8_MAX)
            {
                host_binary_motion.verify_consecutive++;
            }
        }
        else
        {
            host_binary_motion.verify_consecutive = 0U;
        }

        if ((host_binary_motion.verify_consecutive >=
                SERVO_FINAL_SETTLE_CONSECUTIVE) ||
            ((now - host_binary_motion.verify_start_tick) >=
                SERVO_FINAL_SETTLE_MAX_MS))
        {
            uint8_t reported_error = (maximum_error > UINT8_MAX) ?
                UINT8_MAX : (uint8_t)maximum_error;
            host_binary_motion.active = 0U;
            Servo_MotionSafetyEnd();
            Host_SendBinarySetpointStatus(
                host_binary_motion.request_sequence,
                6U,
                1U,
                host_binary_motion.start_tick +
                    host_binary_motion.duration_ms,
                reported_error
            );
        }
        return;
    }

    HAL_StatusTypeDef safety_status = Servo_MotionSafetyPoll();
    if (safety_status != HAL_OK)
    {
        const ServoMotionSafetyDiagnostics *diagnostics =
            Servo_MotionSafetyGetDiagnostics();
        host_binary_motion.active = 0U;
        host_stop_latched = 1U;
        Servo_MotionSafetyEnd();
        Host_SendBinarySetpointStatus(
            host_binary_motion.request_sequence,
            9U,
            1U,
            host_binary_motion.start_tick + host_binary_motion.duration_ms,
            diagnostics->servo_id
        );
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - host_binary_motion.last_control_tick) <
        control_period_ms)
    {
        return;
    }

    elapsed = now - host_binary_motion.start_tick;
    if (elapsed > host_binary_motion.duration_ms)
    {
        elapsed = host_binary_motion.duration_ms;
    }

    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        if (elapsed >= host_binary_motion.duration_ms)
        {
            setpoints[joint] = host_binary_motion.target_positions[joint];
        }
        else
        {
            int32_t signed_delta =
                (int32_t)host_binary_motion.target_positions[joint] -
                (int32_t)host_binary_motion.start_positions[joint];
            int64_t elapsed_squared = (int64_t)elapsed * elapsed;
            int64_t smooth_numerator =
                (3LL * elapsed_squared * host_binary_motion.duration_ms) -
                (2LL * elapsed_squared * elapsed);
            int64_t denominator =
                (int64_t)host_binary_motion.duration_ms *
                host_binary_motion.duration_ms *
                host_binary_motion.duration_ms;
            int32_t raw_position =
                (int32_t)host_binary_motion.start_positions[joint] +
                (int32_t)(
                    ((int64_t)signed_delta * smooth_numerator) / denominator
                );

            if ((raw_position < 0) || (raw_position > 4095))
            {
                host_binary_motion.active = 0U;
                host_stop_latched = 1U;
                Servo_MotionSafetyEnd();
                Host_SendBinarySetpointStatus(
                    host_binary_motion.request_sequence,
                    7U,
                    1U,
                    host_binary_motion.start_tick +
                        host_binary_motion.duration_ms,
                    0U
                );
                return;
            }
            setpoints[joint] = (uint16_t)raw_position;
        }
    }

    if (Servo_SyncWritePositions(setpoints) != HAL_OK)
    {
        host_binary_motion.active = 0U;
        host_stop_latched = 1U;
        Servo_MotionSafetyEnd();
        Host_SendBinarySetpointStatus(
            host_binary_motion.request_sequence,
            7U,
            1U,
            host_binary_motion.start_tick + host_binary_motion.duration_ms,
            0U
        );
        return;
    }

    host_binary_motion.last_control_tick = now;
    if (elapsed >= host_binary_motion.duration_ms)
    {
        host_binary_motion.verifying = 1U;
        host_binary_motion.verify_consecutive = 0U;
        host_binary_motion.verify_sweep_active = 0U;
        host_binary_motion.verify_start_tick = HAL_GetTick();
        host_binary_motion.last_control_tick =
            host_binary_motion.verify_start_tick;
    }
}

static void Host_ValidateLegacyBinarySetpointBatch(
    const actuator_frame_t *request
)
{
    const uint16_t header_size = 8U;
    const uint16_t sample_size = 52U;
    uint8_t sample_count = 0U;
    uint32_t first_apply_tick = 0U;
    uint8_t status_code = 1U;
    uint16_t target_positions[6] = {0U};

    if (request->payload_length >= header_size)
    {
        first_apply_tick = Host_ReadU32Le(&request->payload[0]);
        sample_count = request->payload[4];
    }

    if (!actuator_safety_accepts_setpoint(&host_binary_safety) ||
        (host_stop_latched != 0U) ||
        (host_binary_motion.active != 0U) ||
        (Host_BufferedExecutionIsActive() != 0U))
    {
        status_code = 2U;
    }
    else if ((request->payload_length < header_size) ||
             (sample_count == 0U) ||
             (sample_count > 9U) ||
             ((request->flags & (uint16_t)(~1U)) != 0U) ||
             (request->payload[5] != 1U) ||
             (Host_ReadU16Le(&request->payload[6]) != 0U) ||
             (request->payload_length !=
                 (uint16_t)(header_size +
                     ((uint16_t)sample_count * sample_size))))
    {
        status_code = 1U;
    }
    else
    {
        uint32_t previous_tick = 0U;
        uint32_t now = HAL_GetTick();
        status_code = 5U;

        for (uint8_t sample = 0U;
             sample < sample_count;
             sample++)
        {
            uint16_t sample_offset = (uint16_t)(
                header_size +
                ((uint16_t)sample * sample_size)
            );
            uint32_t tick_offset =
                Host_ReadU32Le(&request->payload[sample_offset]);
            uint32_t apply_tick = first_apply_tick + tick_offset;
            int32_t lead_ms = (int32_t)(apply_tick - now);

            if ((lead_ms < 20) || (lead_ms > 2000) ||
                ((sample > 0U) &&
                 ((int32_t)(apply_tick - previous_tick) <= 0)))
            {
                status_code = 1U;
                break;
            }
            previous_tick = apply_tick;

            for (uint8_t joint = 0U;
                 joint < servo_joint_count;
                 joint++)
            {
                int32_t position_urad = Host_ReadI32Le(
                    &request->payload[
                        sample_offset + 4U +
                        ((uint16_t)joint * 4U)
                    ]
                );
                actuator_joint_calibration_t calibration =
                    Host_JointCalibration(joint);

                if (actuator_urad_to_raw(
                        &calibration,
                        position_urad,
                        &target_positions[joint]
                    ) != ACTUATOR_CALIBRATION_OK)
                {
                    status_code = 3U;
                    break;
                }

                if (Host_ReadI32Le(
                        &request->payload[
                            sample_offset + 28U +
                            ((uint16_t)joint * 4U)
                        ]
                    ) != 0)
                {
                    status_code = 4U;
                    break;
                }
            }

            if (status_code != 5U)
            {
                break;
            }
        }
    }

    if ((status_code == 5U) &&
        ((request->flags & 1U) == 0U))
    {
        if (sample_count == 1U)
        {
            Host_StartBinaryMotion(
                request,
                first_apply_tick,
                target_positions
            );
            return;
        }
        status_code = 1U;
    }

    Host_SendBinarySetpointStatus(
        request->sequence,
        status_code,
        sample_count,
        first_apply_tick,
        0U
    );
}

static void Host_ValidateBufferedCandidate(
    const actuator_frame_t *request
)
{
    actuator_buffered_command_t command;
    actuator_buffered_command_result_t command_result =
        ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH;
    uint8_t sample_count = 0U;
    uint32_t first_apply_tick = 0U;
    uint8_t status_code = 1U;

    if (request->payload_length >= ACTUATOR_BUFFERED_WIRE_HEADER_SIZE)
    {
        first_apply_tick = Host_ReadU32Le(&request->payload[0]);
        sample_count = request->payload[4];
    }

    if ((request->flags & ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY) == 0U)
    {
        command_result = ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS;
    }
    else if (host_buffered_validation_route_ready == 0U)
    {
        status_code = 7U;
        command_result = ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }
    /*
     * Validation-only frames never enter the executor or write a servo.
     * Allow them while physically disabled so Pi-VCP timing can be measured
     * under the READ_ONLY contract.  Faulted, latched, and active-motion
     * states remain fail-closed.
     */
    else if ((host_stop_latched != 0U) ||
             (host_binary_safety.state == ACTUATOR_STATE_FAULT) ||
             (host_binary_safety.state == ACTUATOR_STATE_ESTOPPED) ||
             (host_binary_motion.active != 0U) ||
             (Host_BufferedExecutionIsActive() != 0U))
    {
        status_code = 2U;
        command_result = ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }
    else
    {
        command_result = actuator_buffered_command_decode(
            request->payload,
            request->payload_length,
            request->flags,
            &command
        );
        if (command_result == ACTUATOR_BUFFERED_COMMAND_OK)
        {
            command_result = actuator_buffered_command_route_admit(
                &host_buffered_validation_route,
                &command,
                request->sequence,
                HAL_GetTick(),
                HOST_BUFFERED_VALIDATION_MINIMUM_LEAD_MS,
                HOST_BUFFERED_VALIDATION_MAXIMUM_LEAD_MS
            );
            if (command_result == ACTUATOR_BUFFERED_COMMAND_OK)
            {
                status_code = 5U;
            }
        }
    }

    Host_SendBinaryBufferedSetpointStatus(
        &host_buffered_validation_route,
        request->sequence,
        status_code,
        sample_count,
        first_apply_tick,
        (uint8_t)command_result
    );
}

static void Host_ResetBufferedExecution(void)
{
    memset(
        &host_binary_buffered_motion,
        0,
        sizeof(host_binary_buffered_motion)
    );
    host_buffered_execution_route_ready =
        Host_InitBufferedExecutionRoute();
}

static void Host_FinalizeBufferedExecution(uint8_t detail)
{
    const actuator_buffered_diagnostics_t *diagnostics =
        actuator_buffered_executor_diagnostics(
            &host_buffered_execution_route.executor
        );
    uint32_t apply_tick = HAL_GetTick();
    uint32_t sequence = host_binary_buffered_motion.request_sequence;

    if (diagnostics != NULL)
    {
        apply_tick = (diagnostics->last_applied_tick != 0U) ?
            diagnostics->last_applied_tick : diagnostics->terminal_tick;
        if (diagnostics->safe_stop_required)
        {
            host_stop_latched = 1U;
            if (actuator_safety_accepts_setpoint(&host_binary_safety))
            {
                (void)actuator_safety_request_hold(&host_binary_safety);
            }
        }
    }

    Servo_MotionSafetyEnd();
    Host_SendBinaryBufferedSetpointStatus(
        &host_buffered_execution_route,
        sequence,
        HOST_BUFFERED_STATUS_TERMINAL,
        0U,
        apply_tick,
        detail
    );
    Servo_InMotionTelemetryEnd();
    Host_ResetBufferedExecution();
}

static void Host_AbortBufferedExecution(
    actuator_buffered_reason_t reason,
    uint8_t detail
)
{
    actuator_buffered_result_t result = ACTUATOR_BUFFERED_BAD_STATE;
    uint32_t now;

    if (Host_BufferedExecutionIsActive() == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if (reason == ACTUATOR_BUFFERED_REASON_PLANNED_HOLD)
    {
        result = actuator_buffered_command_route_planned_hold(
            &host_buffered_execution_route,
            now
        );
    }
    else if (reason == ACTUATOR_BUFFERED_REASON_OPERATOR_CANCEL)
    {
        result = actuator_buffered_command_route_cancel(
            &host_buffered_execution_route,
            now
        );
    }
    else if (reason == ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS)
    {
        result = actuator_buffered_command_route_connection_loss(
            &host_buffered_execution_route,
            now
        );
    }
    else
    {
        result = actuator_buffered_command_route_tracking_error(
            &host_buffered_execution_route,
            now
        );
    }

    if (result == ACTUATOR_BUFFERED_TERMINAL)
    {
        Host_FinalizeBufferedExecution(detail);
    }
    else
    {
        host_stop_latched = 1U;
        Servo_MotionSafetyEnd();
        Host_ResetBufferedExecution();
    }
}

static void Host_ExecuteBufferedCandidate(
    const actuator_frame_t *request
)
{
    actuator_buffered_command_t command;
    actuator_buffered_command_result_t command_result =
        ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH;
    uint8_t sample_count = 0U;
    uint32_t first_apply_tick = 0U;
    uint8_t status_code = 1U;
    uint8_t reset_after_response = 0U;
    const uint8_t begin =
        ((request->flags & ACTUATOR_BUFFERED_FLAG_BEGIN) != 0U) ? 1U : 0U;
    const uint8_t start =
        ((request->flags & ACTUATOR_BUFFERED_FLAG_START) != 0U) ? 1U : 0U;

    if ((begin != 0U) &&
        ((request->flags & ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY) == 0U))
    {
        F0Metrics_Reset();
        ControlTick_Reset();
    }

    if (request->payload_length >= ACTUATOR_BUFFERED_WIRE_HEADER_SIZE)
    {
        first_apply_tick = Host_ReadU32Le(&request->payload[0]);
        sample_count = request->payload[4];
    }

    if ((request->flags & ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY) != 0U)
    {
        command_result = ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS;
    }
    else if (host_buffered_execution_route_ready == 0U)
    {
        status_code = 7U;
        command_result = ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }
    else if (!actuator_safety_accepts_setpoint(&host_binary_safety) ||
             (host_stop_latched != 0U) ||
             (host_binary_servos_configured == 0U) ||
             (host_binary_motion.active != 0U) ||
             ((begin != 0U) &&
              (Host_BufferedExecutionIsActive() != 0U)) ||
             ((begin == 0U) &&
              (Host_BufferedExecutionIsActive() == 0U)))
    {
        status_code = 2U;
        command_result = ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }
    else
    {
        command_result = actuator_buffered_command_decode(
            request->payload,
            request->payload_length,
            request->flags,
            &command
        );
        if (command_result == ACTUATOR_BUFFERED_COMMAND_OK)
        {
            if (begin != 0U)
            {
                host_binary_buffered_motion.request_sequence =
                    request->sequence;
                host_binary_buffered_motion.anchor_tick =
                    command.samples[0].apply_tick -
                    HOST_BUFFERED_EXECUTION_ANCHOR_OFFSET_MS;
                memcpy(
                    host_binary_buffered_motion.anchor_positions_urad,
                    command.samples[0].position_urad,
                    sizeof(
                        host_binary_buffered_motion.anchor_positions_urad
                    )
                );
            }

            command_result = actuator_buffered_command_route_admit(
                &host_buffered_execution_route,
                &command,
                request->sequence,
                HAL_GetTick(),
                HOST_BUFFERED_EXECUTION_MINIMUM_LEAD_MS,
                HOST_BUFFERED_EXECUTION_MAXIMUM_LEAD_MS
            );
            if (command_result == ACTUATOR_BUFFERED_COMMAND_OK)
            {
                if (begin != 0U)
                {
                    host_binary_buffered_motion.active = 1U;
                }
                if (start != 0U)
                {
                    actuator_buffered_result_t start_result =
                        actuator_buffered_command_route_start(
                            &host_buffered_execution_route,
                            host_binary_buffered_motion.anchor_tick,
                            host_binary_buffered_motion.
                                anchor_positions_urad
                        );
                    if (start_result != ACTUATOR_BUFFERED_OK)
                    {
                        (void)actuator_buffered_command_route_tracking_error(
                            &host_buffered_execution_route,
                            HAL_GetTick()
                        );
                        host_stop_latched = 1U;
                        status_code = 2U;
                        command_result =
                            ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
                        reset_after_response = 1U;
                    }
                    else
                    {
                        /*
                         * H2.0 uses one nonblocking position query after a
                         * sync-write, never the legacy blocking read/poll
                         * path. A pending query at the next output slot is a
                         * fail-closed tracking error, not a delayed write.
                         */
                        Servo_InMotionTelemetryBegin();
                    }
                }
                if (reset_after_response == 0U)
                {
                    status_code = 0U;
                }
            }
        }
    }

    Host_SendBinaryBufferedSetpointStatus(
        &host_buffered_execution_route,
        request->sequence,
        status_code,
        sample_count,
        first_apply_tick,
        (uint8_t)command_result
    );

    if (reset_after_response != 0U)
    {
        Servo_MotionSafetyEnd();
        Host_ResetBufferedExecution();
    }
}

static void Host_ServiceBufferedExecution(void)
{
    uint32_t now;
    int32_t output_positions_urad[SINGLE_ARM_JOINT_COUNT] = {0};
    uint16_t output_positions_raw[SINGLE_ARM_JOINT_COUNT] = {0U};
    actuator_buffered_result_t result;
    const actuator_buffered_diagnostics_t *diagnostics;

    if (Host_BufferedExecutionIsActive() == 0U)
    {
        return;
    }

    if ((host_stop_latched != 0U) ||
        !actuator_safety_accepts_setpoint(&host_binary_safety))
    {
        Host_AbortBufferedExecution(
            ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS,
            (uint8_t)host_binary_safety.state
        );
        return;
    }

    now = HAL_GetTick();
    if (!host_buffered_execution_route.started)
    {
        /*
         * BEGIN and START are deliberately split across the 9+7 startup
         * prime frames.  A lost START must not leave a live trajectory in
         * PRIMING forever while heartbeats continue.  The anchor is the last
         * safe deadline because no setpoint has been applied before it.
         */
        if ((int32_t)(now - host_binary_buffered_motion.anchor_tick) >= 0)
        {
            Host_AbortBufferedExecution(
                ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                (uint8_t)ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK
            );
        }
        return;
    }

    if ((int32_t)(now - host_binary_buffered_motion.anchor_tick) < 0)
    {
        return;
    }

    if ((host_binary_buffered_motion.last_step_valid != 0U) &&
        (host_binary_buffered_motion.last_step_tick == now))
    {
        return;
    }
    host_binary_buffered_motion.last_step_tick = now;
    host_binary_buffered_motion.last_step_valid = 1U;

    result = actuator_buffered_command_route_step(
        &host_buffered_execution_route,
        now,
        output_positions_urad
    );
    diagnostics = actuator_buffered_executor_diagnostics(
        &host_buffered_execution_route.executor
    );

    if (result == ACTUATOR_BUFFERED_OUTPUT)
    {
        uint8_t write_due =
            (((now - host_binary_buffered_motion.anchor_tick) %
              HOST_BUFFERED_EXECUTION_OUTPUT_PERIOD_MS) == 0U) ? 1U : 0U;

        if ((diagnostics != NULL) &&
            (diagnostics->state == ACTUATOR_BUFFERED_SUCCEEDED))
        {
            write_due = 1U;
        }

        for (uint8_t joint = 0U;
             joint < servo_joint_count;
             joint++)
        {
            const actuator_joint_calibration_t calibration =
                Host_JointCalibration(joint);
            if (actuator_urad_to_raw(
                    &calibration,
                    output_positions_urad[joint],
                    &output_positions_raw[joint]
                ) != ACTUATOR_CALIBRATION_OK)
            {
                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                    servo_joints[joint].id
                );
                return;
            }
        }

        HAL_StatusTypeDef telemetry_status = Servo_InMotionTelemetryPoll(
            now, output_positions_raw
        );
        if (telemetry_status == HAL_ERROR || telemetry_status == HAL_TIMEOUT)
        {
            Host_AbortBufferedExecution(
                ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                0U
            );
            return;
        }
        if ((write_due != 0U) &&
            (Servo_InMotionTelemetryPending() != 0U))
        {
            Host_AbortBufferedExecution(
                ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                0U
            );
            return;
        }

        if (write_due != 0U)
        {
            if (Servo_SyncWritePositions(output_positions_raw) != HAL_OK)
            {
                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                    0U
                );
                return;
            }

            if ((diagnostics == NULL) ||
                (diagnostics->state != ACTUATOR_BUFFERED_SUCCEEDED))
            {
                if (Servo_InMotionTelemetryStart(
                        host_binary_buffered_motion.next_telemetry_joint,
                        now
                    ) != HAL_OK)
                {
                    Host_AbortBufferedExecution(
                        ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                        0U
                    );
                    return;
                }
                host_binary_buffered_motion.next_telemetry_joint =
                    (uint8_t)((host_binary_buffered_motion.
                        next_telemetry_joint + 1U) % servo_joint_count);
            }
        }

        if ((diagnostics != NULL) &&
            (diagnostics->state == ACTUATOR_BUFFERED_SUCCEEDED))
        {
            uint32_t maximum_lateness =
                diagnostics->maximum_apply_lateness_ticks;
            Host_FinalizeBufferedExecution(
                (maximum_lateness > UINT8_MAX) ?
                    UINT8_MAX : (uint8_t)maximum_lateness
            );
            return;
        }
    }
    else if (result == ACTUATOR_BUFFERED_TERMINAL)
    {
        Host_FinalizeBufferedExecution(
            (diagnostics == NULL) ? 0U : (uint8_t)diagnostics->reason
        );
        return;
    }
    else if (result != ACTUATOR_BUFFERED_WAITING)
    {
        Host_AbortBufferedExecution(
            ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
            (uint8_t)result
        );
        return;
    }

}

static uint8_t Host_BinaryClearStopIsSafe(void)
{
    uint16_t current_positions[6] = {0U};

    if (Servo_ReadAllPositions(current_positions) != HAL_OK)
    {
        return 2U;
    }

    for (uint8_t i = 0U; i < servo_joint_count; i++)
    {
        int32_t minimum_allowed =
            (int32_t)servo_joints[i].min_position - 40;
        int32_t maximum_allowed =
            (int32_t)servo_joints[i].max_position + 40;

        if (((int32_t)current_positions[i] < minimum_allowed) ||
            ((int32_t)current_positions[i] > maximum_allowed))
        {
            return 3U;
        }
    }

    return 0U;
}

#if HOST_F25_VALIDATION_ONLY_BUILD
static uint8_t Host_F25RejectsRequest(const actuator_frame_t *request)
{
    if (request->message_type == ACTUATOR_MSG_SETPOINT_BATCH)
    {
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        return 0U;
#else
        uint16_t required_flags =
            ACTUATOR_BUFFERED_FLAG_CANDIDATE |
            ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY;
        return ((request->flags & required_flags) == required_flags) ?
            0U : 1U;
#endif
    }

    switch (request->message_type)
    {
        case ACTUATOR_MSG_ARM_REQUEST:
        case ACTUATOR_MSG_ENABLE:
        case ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST:
        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_REQUEST:
            return 1U;

        default:
            return 0U;
    }
}
#endif

#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
static actuator_v2_stream_hard_caps_t Host_V2HardCaps(void)
{
    actuator_v2_stream_hard_caps_t caps;
    memset(&caps, 0, sizeof(caps));
    caps.minimum_lead_ms = 20U;
    caps.maximum_lead_ms = 400U;
    caps.maximum_command_timeout_ms = 500U;
    caps.maximum_open_command_timeout_ms = 100U;
    caps.maximum_apply_lateness_ms = 5U;
    for (uint8_t joint = 0U; joint < ACTUATOR_V2_JOINT_COUNT; joint++)
    {
        /* Validation-only values: these cannot authorize servo output. */
        caps.tracking_error_limit_urad[joint] =
#if HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD
            ((joint == (SINGLE_ARM_JOINT_COUNT - 1U)) ||
             (joint == (ACTUATOR_V2_JOINT_COUNT - 1U)))
                ? HOST_BIMANUAL_GRIPPER_TRACKING_HARD_CAP_URAD
                : INT32_C(100000);
#else
            INT32_C(100000);
#endif
        caps.maximum_step_urad_per_tick[joint] = 10000;
    }
    return caps;
}

#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
static void Host_V2JointLimits(
    actuator_v2_joint_limit_t limits[ACTUATOR_V2_JOINT_COUNT]
)
{
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    BimanualOperationalLimits_LoadExecutorLimits(limits);
#elif HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD
    BimanualOperationalLimits_LoadJ1LShadow(limits);
#else
    for (uint8_t joint = 0U; joint < ACTUATOR_V2_JOINT_COUNT; joint++)
    {
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
        /* Validation-only coordinates: no value reaches either servo bus. */
        limits[joint].minimum_urad = -6400000;
        limits[joint].maximum_urad = 6400000;
#elif HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
        const actuator_joint_calibration_t calibration =
            Host_JointCalibration(
                (uint8_t)(joint % SINGLE_ARM_JOINT_COUNT));
        int32_t first_limit = 0;
        int32_t second_limit = 0;
        if ((actuator_raw_to_urad(
                 &calibration, calibration.minimum_raw, &first_limit) !=
             ACTUATOR_CALIBRATION_OK) ||
            (actuator_raw_to_urad(
                 &calibration, calibration.maximum_raw, &second_limit) !=
             ACTUATOR_CALIBRATION_OK))
        {
            limits[joint].minimum_urad = 0;
            limits[joint].maximum_urad = 0;
        }
        else if (first_limit <= second_limit)
        {
            limits[joint].minimum_urad = first_limit;
            limits[joint].maximum_urad = second_limit;
        }
        else
        {
            limits[joint].minimum_urad = second_limit;
            limits[joint].maximum_urad = first_limit;
        }
#else
        /* Synthetic executor limits only. No value reaches a servo bus. */
        limits[joint].minimum_urad = -3200000;
        limits[joint].maximum_urad = 3200000;
#endif
    }
#endif
}

static actuator_v2_stream_session_result_t Host_V2ExecutorSessionResult(
    actuator_v2_stream_status_code_t status_code,
    actuator_v2_contract_result_t contract_result
)
{
    actuator_v2_stream_session_result_t result;
    const actuator_v2_stream_session_t *session =
        &host_v2_stream_executor.session;
    memset(&result, 0, sizeof(result));
    result.status_code = status_code;
    result.contract_result = contract_result;
    if (session->open)
    {
        result.arm_mask = session->policy.arm_mask;
        result.arbiter_epoch = session->arbiter_epoch;
        result.horizon_end_tick = session->horizon_end_tick;
        result.validated_sample_count = session->validated_sample_count;
        if (session->validated_sample_count > 0U)
        {
            result.validated_tail_tick = session->validated_samples[
                session->validated_sample_count - 1U].apply_tick;
        }
    }
    return result;
}
#endif

static void Host_SendV2StreamStatus(
    const actuator_frame_t *request,
    actuator_v2_stream_session_result_t result
)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_STREAM_STATUS;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = ACTUATOR_V2_STREAM_STATUS_WIRE_SIZE;
    response.payload[0] = (uint8_t)result.status_code;
    response.payload[1] = (uint8_t)result.contract_result;
    response.payload[2] = (uint8_t)host_binary_safety.state;
    response.payload[3] = result.arm_mask;
    Host_WriteU32Le(&response.payload[4], request->sequence);
    Host_WriteU32Le(&response.payload[8], request->sender_time_ms);
    Host_WriteU32Le(&response.payload[12], result.arbiter_epoch);
    Host_WriteU32Le(&response.payload[16], result.horizon_end_tick);
    Host_WriteU32Le(&response.payload[20], result.validated_tail_tick);
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
    const actuator_v2_executor_diagnostics_t *diagnostics =
        actuator_v2_stream_executor_diagnostics(&host_v2_stream_executor);
    Host_WriteU32Le(
        &response.payload[24],
        (diagnostics == NULL) ? 0U : diagnostics->queued_samples);
    Host_WriteU32Le(
        &response.payload[28],
        (diagnostics == NULL) ? 0U : diagnostics->accepted_samples);
    Host_WriteU32Le(
        &response.payload[32],
        (diagnostics == NULL) ? 0U : diagnostics->applied_samples);
#else
    /* The semantic validator is isolated from every executable/output path. */
    Host_WriteU32Le(&response.payload[24], 0U);
    Host_WriteU32Le(&response.payload[28], 0U);
    Host_WriteU32Le(&response.payload[32], 0U);
#endif
    (void)Host_SendBinaryFrame(&response);
}

static void Host_ValidateV2StreamOpen(const actuator_frame_t *request)
{
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
    actuator_v2_stream_session_result_t result;
    actuator_v2_executor_result_t executor_result;

    if (host_v2_executor_ready == 0U)
    {
        result = Host_V2ExecutorSessionResult(
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            ACTUATOR_V2_CONTRACT_NULL_ARGUMENT
        );
        host_binary_rejected_frame_count++;
        Host_SendV2StreamStatus(request, result);
        return;
    }
    executor_result = actuator_v2_stream_executor_open(
        &host_v2_stream_executor,
        request->payload,
        request->payload_length,
        HAL_GetTick()
    );
    if (executor_result == ACTUATOR_V2_EXECUTOR_OK)
    {
        result = Host_V2ExecutorSessionResult(
            ACTUATOR_V2_STREAM_STATUS_OK,
            ACTUATOR_V2_CONTRACT_OK
        );
        host_v2_executor_clock_active = 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
        host_v2_executor_start_pending = 0U;
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
        host_v2_terminal_settle_active = 0U;
        host_v2_terminal_settle_baseline_completed_pairs = 0U;
#endif
#endif
    }
    else
    {
        const actuator_v2_executor_diagnostics_t *diagnostics =
            actuator_v2_stream_executor_diagnostics(
                &host_v2_stream_executor);
        result = Host_V2ExecutorSessionResult(
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            (diagnostics == NULL) ?
                ACTUATOR_V2_CONTRACT_NULL_ARGUMENT :
                diagnostics->last_contract_result
        );
        host_binary_rejected_frame_count++;
    }
    Host_SendV2StreamStatus(request, result);
#else
    const actuator_v2_stream_hard_caps_t caps = Host_V2HardCaps();
    const actuator_v2_stream_session_result_t result =
        actuator_v2_stream_session_open(
            &host_v2_stream_session,
            request->payload,
            request->payload_length,
            &caps,
            HAL_GetTick()
        );
    if (result.status_code != ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY)
    {
        host_binary_rejected_frame_count++;
    }
    Host_SendV2StreamStatus(request, result);
#endif
}

static void Host_ValidateV2Batch(
    const actuator_frame_t *request,
    actuator_v2_batch_kind_t kind
)
{
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
    actuator_v2_stream_session_result_t result;
    actuator_v2_executor_result_t executor_result =
        actuator_v2_stream_executor_admit(
            &host_v2_stream_executor,
            request->payload,
            request->payload_length,
            kind,
            HAL_GetTick()
        );

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    if ((executor_result == ACTUATOR_V2_EXECUTOR_OK) &&
        ((host_binary_servos_configured == 0U) ||
         (host_right_arm_output_active == 0U) ||
         !actuator_safety_accepts_setpoint(&host_binary_safety)))
    {
        executor_result = ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }
#endif

    if ((executor_result == ACTUATOR_V2_EXECUTOR_OK) &&
        (host_v2_stream_executor.diagnostics.state ==
            ACTUATOR_V2_EXECUTOR_PRIMING) &&
        (host_v2_stream_executor.session.validated_sample_count >=
            host_v2_stream_executor.session.policy.minimum_start_samples))
    {
#if HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
        if (host_v2_shadow_anchor_ready == 0U)
        {
            executor_result = ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR;
        }
        else
        {
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
            if (BimanualTrackingFeedback_Begin() != HAL_OK)
            {
                executor_result = ACTUATOR_V2_EXECUTOR_BAD_STATE;
            }
            else
            {
                const actuator_bimanual_dispatch_snapshot_t *dispatch =
                    BimanualServoDispatch_GetSnapshot();
                host_v2_tracking_next_joint = 0U;
                host_v2_tracking_last_dispatch_completed =
                    (dispatch == NULL) ? 0U : dispatch->completed_count;
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
                host_v2_terminal_settle_active = 0U;
                host_v2_terminal_settle_baseline_completed_pairs = 0U;
#endif
#endif
            /* HAL_GetTick() is not phase-aligned with the free-running TIM6
             * control interrupt. Starting here made the executor 5 ms grid
             * differ from the ticks which call executor_step(). Defer start
             * until the first real TIM6 event supplies the control epoch. */
            ControlTick_ClearPending();
            host_v2_executor_start_pending = 1U;
            host_v2_executor_clock_active = 1U;
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
            }
#endif
#else
            const uint32_t start_tick = HAL_GetTick();
            executor_result = actuator_v2_stream_executor_start(
                &host_v2_stream_executor,
                start_tick,
                host_v2_shadow_executor_anchor_urad
            );
            if (executor_result == ACTUATOR_V2_EXECUTOR_OK)
            {
                host_v2_shadow_anchor_ready = 0U;
                host_v2_executor_next_tick = start_tick;
                host_v2_executor_clock_active = 1U;
            }
#endif
        }
#else
        int32_t synthetic_anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
        const uint32_t start_tick = HAL_GetTick();
        executor_result = actuator_v2_stream_executor_start(
            &host_v2_stream_executor,
            start_tick,
            synthetic_anchor
        );
        if (executor_result == ACTUATOR_V2_EXECUTOR_OK)
        {
            host_v2_executor_next_tick = start_tick;
            host_v2_executor_clock_active = 1U;
        }
#endif
    }

    if (executor_result == ACTUATOR_V2_EXECUTOR_OK)
    {
        result = Host_V2ExecutorSessionResult(
            ACTUATOR_V2_STREAM_STATUS_OK,
            ACTUATOR_V2_CONTRACT_OK
        );
    }
    else
    {
        const actuator_v2_executor_diagnostics_t *diagnostics =
            actuator_v2_stream_executor_diagnostics(
                &host_v2_stream_executor);
        actuator_v2_contract_result_t contract_result =
            (diagnostics == NULL) ?
                ACTUATOR_V2_CONTRACT_NULL_ARGUMENT :
                diagnostics->last_contract_result;
        if (contract_result == ACTUATOR_V2_CONTRACT_OK)
        {
            contract_result = ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY;
        }
        result = Host_V2ExecutorSessionResult(
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            contract_result
        );
        host_binary_rejected_frame_count++;
    }
    Host_SendV2StreamStatus(request, result);
#else
    const actuator_v2_stream_session_result_t result =
        actuator_v2_stream_session_batch(
            &host_v2_stream_session,
            request->payload,
            request->payload_length,
            kind,
            HAL_GetTick()
        );
    if (result.status_code != ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY)
    {
        host_binary_rejected_frame_count++;
    }
    Host_SendV2StreamStatus(request, result);
#endif
}

#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
#if !HOST_BIMANUAL_DMA_DISPATCH_BUILD
static uint8_t Host_V2TickReached(uint32_t now, uint32_t target)
{
    return ((int32_t)(now - target) >= 0) ? 1U : 0U;
}
#endif

static void Host_RequestV2CoordinatedStop(void)
{
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    BimanualServoDispatch_LatchFault();
    host_v2_coordinated_stop_pending = 1U;
    host_v2_executor_start_pending = 0U;
#endif
    host_v2_executor_clock_active = 0U;
    host_stop_latched = 1U;
}

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
static uint8_t Host_PerformV2CoordinatedStop(uint8_t dispatch_fault)
{
    uint8_t status = 0U;

    if (dispatch_fault != 0U)
    {
        BimanualServoDispatch_LatchFault();
    }
    else
    {
        BimanualServoDispatch_Stop();
    }
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    BimanualTrackingFeedback_End();
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
    host_v2_terminal_settle_active = 0U;
    host_v2_terminal_settle_baseline_completed_pairs = 0U;
#endif
#endif
    if (Servo_DisableTorqueAll() != HAL_OK)
    {
        status = 1U;
    }
    if (RightServoBus_DisableTorqueAll() != HAL_OK)
    {
        status = 1U;
    }
    host_binary_servos_configured = 0U;
    host_right_arm_output_active = 0U;
    host_bimanual_arm_watchdog_grace_started_ms = 0U;
    host_v2_coordinated_stop_pending = 0U;
    host_v2_executor_start_pending = 0U;
    host_stop_latched = 1U;
    if (status != 0U)
    {
        actuator_safety_report_fault(&host_binary_safety, UINT16_C(0xFF06));
    }
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD || \
    HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
    host_v2_last_coordinated_stop_status = status;
#endif
    return status;
}

static uint8_t Host_ConfigureBimanualForTrajectory(void)
{
    uint16_t left_positions[6] = {0U};

    if (Servo_ConfigureAllForTrajectory(left_positions) != HAL_OK)
    {
        (void)Servo_DisableTorqueAll();
        (void)RightServoBus_DisableTorqueAll();
        return 0U;
    }
    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        const RightServoConfigureSnapshot configured =
            RightServoBus_ConfigureAtPresentPositionOnce(
                servo_joints[joint].id,
                servo_joints[joint].p_gain,
                servo_joints[joint].d_gain,
                SERVO_GOAL_SPEED_RAW,
                servo_joints[joint].torque_limit);
        if (configured.status != RIGHT_SERVO_CONFIGURE_OK)
        {
            (void)Servo_DisableTorqueAll();
            (void)RightServoBus_DisableTorqueAll();
            return 0U;
        }
    }
    for (uint8_t joint = 0U; joint < servo_joint_count; joint++)
    {
        const RightServoTorqueEnableSnapshot enabled =
            RightServoBus_EnableTorqueAtPresentPositionOnce(
                servo_joints[joint].id);
        if (enabled.status != RIGHT_SERVO_TORQUE_ENABLE_OK)
        {
            (void)Servo_DisableTorqueAll();
            (void)RightServoBus_DisableTorqueAll();
            return 0U;
        }
    }
    host_bimanual_arm_watchdog_grace_started_ms = HAL_GetTick();
    host_right_arm_output_active = 1U;
    return 1U;
}
#endif

#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
static uint8_t Host_V2TrackingRawToUrad(
    uint8_t global_joint,
    uint16_t raw,
    int32_t *position_urad)
{
    const uint8_t arm_joint =
        (uint8_t)(global_joint % SINGLE_ARM_JOINT_COUNT);
    const BimanualArm arm = (global_joint < SINGLE_ARM_JOINT_COUNT) ?
        BIMANUAL_ARM_LEFT : BIMANUAL_ARM_RIGHT;
    const BimanualOperationalLimit *limit =
        BimanualOperationalLimits_Get(arm, arm_joint);
    int32_t unwrapped_raw = 0;

    if ((position_urad == NULL) || (limit == NULL) ||
        (actuator_joint_unwrapper_update(
             &host_v2_shadow_unwrappers[global_joint],
             raw,
             &unwrapped_raw) != ACTUATOR_UNWRAP_OK) ||
        (actuator_unwrapped_raw_to_urad(
             limit->zero_raw,
             limit->positive_raw_direction,
             unwrapped_raw,
             position_urad) != ACTUATOR_UNWRAP_OK))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t Host_V2TrackingCompletedPairs(void)
{
    const BimanualTrackingFeedbackSnapshot *tracking =
        BimanualTrackingFeedback_GetSnapshot();
    return (tracking == NULL) ? 0U : tracking->completed_pairs;
}

static void Host_ServiceV2TrackingFeedback(void)
{
    BimanualTrackingFeedbackSample sample;
    BimanualTrackingFeedbackResult feedback_result;
    const actuator_bimanual_dispatch_snapshot_t *dispatch;

    if (BimanualTrackingFeedback_Active() == 0U)
    {
        return;
    }
    feedback_result = BimanualTrackingFeedback_Poll(HAL_GetTick(), &sample);
    if (feedback_result == BIMANUAL_TRACKING_FAULT)
    {
        Host_RequestV2CoordinatedStop();
        return;
    }
    if (feedback_result == BIMANUAL_TRACKING_SAMPLE_READY)
    {
        int32_t left_measured_urad = 0;
        int32_t right_measured_urad = 0;
        const uint8_t right_joint =
            (uint8_t)(sample.joint_index + SINGLE_ARM_JOINT_COUNT);
        actuator_v2_executor_result_t result;

        if ((Host_V2TrackingRawToUrad(
                 sample.joint_index,
                 sample.left_position_raw,
                 &left_measured_urad) == 0U) ||
            (Host_V2TrackingRawToUrad(
                 right_joint,
                 sample.right_position_raw,
                 &right_measured_urad) == 0U))
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
        BimanualFeedbackSnapshot_UpdatePair(
            sample.joint_index,
            left_measured_urad,
            right_measured_urad,
            HAL_GetTick()
        );
#endif
#if HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
        {
            const BimanualTrackingFeedbackSnapshot *tracking =
                BimanualTrackingFeedback_GetSnapshot();
            if ((host_v2_tracking_fault_injection_consumed == 0U) &&
                (tracking != NULL) && (tracking->completed_pairs >= 8U))
            {
                host_v2_tracking_fault_injection_consumed = 1U;
                right_measured_urad =
                    sample.right_commanded_urad + INT32_C(100000);
            }
        }
#endif
        result = actuator_v2_stream_executor_check_joint_feedback(
            &host_v2_stream_executor,
            HAL_GetTick(),
            sample.joint_index,
            sample.left_commanded_urad,
            left_measured_urad);
        if (result != ACTUATOR_V2_EXECUTOR_OK)
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
        result = actuator_v2_stream_executor_check_joint_feedback(
            &host_v2_stream_executor,
            HAL_GetTick(),
            right_joint,
            sample.right_commanded_urad,
            right_measured_urad);
        if (result != ACTUATOR_V2_EXECUTOR_OK)
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
        if (host_v2_terminal_settle_active != 0U)
        {
            const int64_t terminal_tolerance =
                (sample.joint_index == (SINGLE_ARM_JOINT_COUNT - 1U))
                    ? HOST_BIMANUAL_TERMINAL_SETTLE_GRIPPER_TOLERANCE_URAD
                    : HOST_BIMANUAL_TERMINAL_SETTLE_ARM_TOLERANCE_URAD;
            int64_t left_error =
                (int64_t)left_measured_urad -
                (int64_t)sample.left_commanded_urad;
            int64_t right_error =
                (int64_t)right_measured_urad -
                (int64_t)sample.right_commanded_urad;
            if (left_error < 0)
            {
                left_error = -left_error;
            }
            if (right_error < 0)
            {
                right_error = -right_error;
            }
            if ((left_error > terminal_tolerance) ||
                (right_error > terminal_tolerance))
            {
                host_v2_terminal_settle_baseline_completed_pairs =
                    Host_V2TrackingCompletedPairs();
            }
        }
#endif
    }

    dispatch = BimanualServoDispatch_GetSnapshot();
    if ((BimanualTrackingFeedback_Pending() == 0U) &&
        (dispatch != NULL) && !dispatch->active && !dispatch->faulted &&
        host_v2_stream_executor.output_valid &&
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
        ((((host_v2_terminal_settle_active != 0U) &&
           ((Host_V2TrackingCompletedPairs() -
              host_v2_terminal_settle_baseline_completed_pairs) <
            HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS)) ||
          ((host_v2_terminal_settle_active == 0U) &&
           (dispatch->completed_count >
            host_v2_tracking_last_dispatch_completed))) &&
#else
        ((dispatch->completed_count >
          host_v2_tracking_last_dispatch_completed) &&
#endif
         ((host_v2_stream_executor.diagnostics.state ==
           ACTUATOR_V2_EXECUTOR_RUNNING) ||
          (host_v2_stream_executor.diagnostics.state ==
           ACTUATOR_V2_EXECUTOR_SUCCEEDED))))
    {
        const uint8_t joint = host_v2_tracking_next_joint;
        if (BimanualTrackingFeedback_Start(
                joint,
                HAL_GetTick(),
                host_v2_last_left_raw,
                host_v2_last_right_raw,
                host_v2_output_urad[joint],
                host_v2_output_urad[
                    joint + SINGLE_ARM_JOINT_COUNT]) != HAL_OK)
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
        if (host_v2_terminal_settle_active == 0U)
#endif
        {
            host_v2_tracking_last_dispatch_completed =
                dispatch->completed_count;
        }
        host_v2_tracking_next_joint = (uint8_t)(
            (joint + 1U) % SINGLE_ARM_JOINT_COUNT);
    }

#if HOST_BIMANUAL_RESIDENT_FINITE_BUILD
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
    if (host_v2_terminal_settle_active != 0U)
    {
        if ((HAL_GetTick() - host_v2_terminal_settle_started_ms) >
            HOST_BIMANUAL_TERMINAL_SETTLE_MAX_MS)
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
        if (((Host_V2TrackingCompletedPairs() -
              host_v2_terminal_settle_baseline_completed_pairs) >=
             HOST_BIMANUAL_TERMINAL_SETTLE_CONSECUTIVE_PAIRS) &&
            (BimanualTrackingFeedback_Pending() == 0U) &&
            (dispatch != NULL) && !dispatch->active && !dispatch->faulted &&
            (dispatch->launch_count == dispatch->completed_count))
        {
            BimanualFeedbackSnapshot snapshot;
            BimanualFeedbackSnapshot_Copy(HAL_GetTick(), &snapshot);
            if (snapshot.present_mask != BIMANUAL_FEEDBACK_COMPLETE_MASK)
            {
                Host_RequestV2CoordinatedStop();
                return;
            }
            memcpy(
                host_v2_shadow_executor_anchor_urad,
                snapshot.positions_urad,
                sizeof(host_v2_shadow_executor_anchor_urad)
            );
            host_v2_shadow_anchor_ready = 1U;
            host_v2_terminal_settle_active = 0U;
            BimanualTrackingFeedback_End();
        }
    }
#else
    {
        const BimanualTrackingFeedbackSnapshot *tracking =
            BimanualTrackingFeedback_GetSnapshot();
        if ((tracking != NULL) &&
            (host_v2_executor_clock_active == 0U) &&
            (host_v2_stream_executor.diagnostics.state ==
             ACTUATOR_V2_EXECUTOR_SUCCEEDED) &&
            (tracking->pending == 0U) &&
            (tracking->requested_pairs == tracking->completed_pairs) &&
            (dispatch != NULL) && !dispatch->active && !dispatch->faulted &&
            (dispatch->launch_count == dispatch->completed_count) &&
            (dispatch->completed_count ==
             host_v2_tracking_last_dispatch_completed))
        {
            memcpy(
                host_v2_shadow_executor_anchor_urad,
                host_v2_output_urad,
                sizeof(host_v2_shadow_executor_anchor_urad)
            );
            host_v2_shadow_anchor_ready = 1U;
            BimanualTrackingFeedback_End();
        }
    }
#endif
#endif
}
#endif

static void Host_ServiceV2Executor(void)
{
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    ControlTickEvent event;
    actuator_v2_executor_result_t result;
    uint16_t left_raw[6] = {0U};
    uint16_t right_raw[6] = {0U};
    uint8_t failed_joint = 0U;

    if ((host_v2_executor_clock_active == 0U) ||
        (ControlTick_TakeEvent(&event) == 0U))
    {
        return;
    }
    if ((event.missed_count != 0U) ||
        (BimanualServoDispatch_Ready() == 0U)
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
        || (BimanualTrackingFeedback_Pending() != 0U)
#endif
       )
    {
        Host_RequestV2CoordinatedStop();
        return;
    }
    if (host_v2_executor_start_pending != 0U)
    {
        result = actuator_v2_stream_executor_start(
            &host_v2_stream_executor,
            event.tick_ms,
            host_v2_shadow_executor_anchor_urad);
        if (result != ACTUATOR_V2_EXECUTOR_OK)
        {
            Host_RequestV2CoordinatedStop();
            return;
        }
        host_v2_executor_start_pending = 0U;
        host_v2_shadow_anchor_ready = 0U;
    }
    result = actuator_v2_stream_executor_step(
        &host_v2_stream_executor, event.tick_ms, host_v2_output_urad);
    if ((result != ACTUATOR_V2_EXECUTOR_OUTPUT_READY) &&
        !((result == ACTUATOR_V2_EXECUTOR_TERMINAL) &&
          (host_v2_stream_executor.diagnostics.state ==
           ACTUATOR_V2_EXECUTOR_SUCCEEDED)))
    {
        Host_RequestV2CoordinatedStop();
        return;
    }
    if (BimanualOperationalLimits_MapExecutorOutput(
            host_v2_output_urad, left_raw, right_raw, &failed_joint) !=
        ACTUATOR_BIMANUAL_GOAL_MAP_OK)
    {
        host_v2_stream_executor.diagnostics.tracking_error_joint =
            failed_joint;
        Host_RequestV2CoordinatedStop();
        return;
    }
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    memcpy(host_v2_last_left_raw, left_raw, sizeof(host_v2_last_left_raw));
    memcpy(host_v2_last_right_raw, right_raw, sizeof(host_v2_last_right_raw));
#endif
    if (BimanualServoDispatch_Launch(
            left_raw, right_raw, event.tick_ms, event.started_us) != HAL_OK)
    {
        Host_RequestV2CoordinatedStop();
        return;
    }
    if (result == ACTUATOR_V2_EXECUTOR_TERMINAL)
    {
        host_v2_executor_clock_active = 0U;
#if HOST_BIMANUAL_TERMINAL_SETTLE_BUILD
        host_v2_terminal_settle_active = 1U;
        host_v2_terminal_settle_baseline_completed_pairs =
            Host_V2TrackingCompletedPairs();
        host_v2_terminal_settle_started_ms = HAL_GetTick();
        host_v2_tracking_next_joint = 0U;
#endif
    }
#else
    uint8_t steps = 0U;
    const uint32_t now = HAL_GetTick();

    if (host_v2_executor_clock_active == 0U)
    {
        return;
    }
    while ((steps < 8U) &&
           (Host_V2TickReached(now, host_v2_executor_next_tick) != 0U))
    {
        const actuator_v2_executor_result_t result =
            actuator_v2_stream_executor_step(
                &host_v2_stream_executor,
                host_v2_executor_next_tick,
                host_v2_discarded_output_urad
            );
        host_v2_executor_next_tick += ACTUATOR_V2_CONTROL_TICK_MS;
        steps++;
        if (result == ACTUATOR_V2_EXECUTOR_TERMINAL)
        {
            host_v2_executor_clock_active = 0U;
            return;
        }
        if (result != ACTUATOR_V2_EXECUTOR_OUTPUT_READY)
        {
            host_v2_executor_clock_active = 0U;
            return;
        }
    }
#endif
}

static void Host_SendV2ExecutorDiagnostics(const actuator_frame_t *request)
{
    actuator_frame_t response;
    const actuator_v2_executor_diagnostics_t *diagnostics =
        actuator_v2_stream_executor_diagnostics(&host_v2_stream_executor);
    const actuator_v2_stream_session_t *session =
        &host_v2_stream_executor.session;
    uint32_t tail_tick = 0U;

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_EXECUTOR_DIAGNOSTICS;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = ACTUATOR_V2_EXECUTOR_DIAGNOSTICS_WIRE_SIZE;
    if (session->validated_sample_count > 0U)
    {
        tail_tick = session->validated_samples[
            session->validated_sample_count - 1U].apply_tick;
    }
    response.payload[0] = (diagnostics == NULL) ?
        (uint8_t)ACTUATOR_V2_EXECUTOR_CLOSED :
        (uint8_t)diagnostics->state;
    response.payload[1] = (diagnostics == NULL) ?
        (uint8_t)ACTUATOR_V2_TERMINAL_NONE :
        (uint8_t)diagnostics->terminal_reason;
    response.payload[2] = ((diagnostics != NULL) &&
        diagnostics->safe_stop_required) ? 1U : 0U;
    response.payload[3] = (diagnostics == NULL) ?
        0U : diagnostics->tracking_error_joint;
    Host_WriteU32Le(&response.payload[4], request->sequence);
    Host_WriteU32Le(&response.payload[8], request->sender_time_ms);
    Host_WriteU32Le(&response.payload[12], session->arbiter_epoch);
    Host_WriteU32Le(&response.payload[16], session->horizon_end_tick);
    Host_WriteU32Le(&response.payload[20], tail_tick);
    Host_WriteU32Le(
        &response.payload[24],
        (diagnostics == NULL) ? 0U : diagnostics->queued_samples);
    Host_WriteU32Le(
        &response.payload[28],
        (diagnostics == NULL) ? 0U : diagnostics->peak_queued_samples);
    Host_WriteU32Le(
        &response.payload[32],
        (diagnostics == NULL) ? 0U : diagnostics->accepted_samples);
    Host_WriteU32Le(
        &response.payload[36],
        (diagnostics == NULL) ? 0U : diagnostics->applied_samples);
    Host_WriteU32Le(
        &response.payload[40],
        (diagnostics == NULL) ? 0U : diagnostics->control_outputs);
    Host_WriteU32Le(
        &response.payload[44],
        (diagnostics == NULL) ? 0U : diagnostics->splice_count);
    Host_WriteU32Le(
        &response.payload[48],
        (diagnostics == NULL) ?
            0U : diagnostics->maximum_apply_lateness_ms);
    Host_WriteU32Le(
        &response.payload[52],
        (diagnostics == NULL) ? 0U : diagnostics->last_command_tick);
    Host_WriteU32Le(
        &response.payload[56],
        (diagnostics == NULL) ? 0U : diagnostics->terminal_tick);
    (void)Host_SendBinaryFrame(&response);
}

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
static void Host_SendV2DispatchDiagnostics(const actuator_frame_t *request)
{
    actuator_frame_t response;
    const actuator_bimanual_dispatch_snapshot_t *snapshot =
        BimanualServoDispatch_GetSnapshot();

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_DISPATCH_DIAGNOSTICS;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = ACTUATOR_V2_DISPATCH_DIAGNOSTICS_WIRE_SIZE;
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD || \
    HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
    response.payload[0] = (snapshot == NULL) ? 1U :
        host_v2_last_coordinated_stop_status;
#else
    response.payload[0] = (snapshot == NULL) ? 1U : 0U;
#endif
    response.payload[1] = ((snapshot != NULL) && snapshot->active) ? 1U : 0U;
    response.payload[2] = ((snapshot != NULL) && snapshot->faulted) ? 1U : 0U;
    response.payload[3] = BimanualServoDispatch_Ready();
    Host_WriteU32Le(&response.payload[4], request->sequence);
    Host_WriteU32Le(&response.payload[8], request->sender_time_ms);
    Host_WriteU32Le(&response.payload[12],
        (snapshot == NULL) ? 0U : snapshot->launch_count);
    Host_WriteU32Le(&response.payload[16],
        (snapshot == NULL) ? 0U : snapshot->completed_count);
    Host_WriteU32Le(&response.payload[20],
        (snapshot == NULL) ? 0U : snapshot->failure_count);
    Host_WriteU32Le(&response.payload[24],
        (snapshot == NULL) ? 0U : snapshot->maximum_start_skew_us);
    Host_WriteU32Le(&response.payload[28],
        (snapshot == NULL) ? 0U : snapshot->maximum_launch_lateness_us);
    Host_WriteU32Le(&response.payload[32],
        (snapshot == NULL) ? 0U : snapshot->last_control_tick_ms);
    Host_WriteU32Le(&response.payload[36],
        (snapshot == NULL) ? 0U : snapshot->last_left_start_us);
    Host_WriteU32Le(&response.payload[40],
        (snapshot == NULL) ? 0U : snapshot->last_right_start_us);
    (void)Host_SendBinaryFrame(&response);
}

#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
static void Host_SendV2TrackingDiagnostics(const actuator_frame_t *request)
{
    actuator_frame_t response;
    const BimanualTrackingFeedbackSnapshot *tracking =
        BimanualTrackingFeedback_GetSnapshot();
    const actuator_v2_executor_diagnostics_t *executor =
        actuator_v2_stream_executor_diagnostics(&host_v2_stream_executor);

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_TRACKING_DIAGNOSTICS;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = ACTUATOR_V2_TRACKING_DIAGNOSTICS_WIRE_SIZE;
    response.payload[0] = (tracking == NULL) ? 1U : 0U;
    response.payload[1] = ((tracking != NULL) &&
        (tracking->active != 0U)) ? 1U : 0U;
    response.payload[2] = ((tracking != NULL) &&
        (tracking->pending != 0U)) ? 1U : 0U;
    response.payload[3] = host_v2_tracking_next_joint;
    Host_WriteU32Le(&response.payload[4], request->sequence);
    Host_WriteU32Le(&response.payload[8], request->sender_time_ms);
    Host_WriteU32Le(&response.payload[12],
        (tracking == NULL) ? 0U : tracking->requested_pairs);
    Host_WriteU32Le(&response.payload[16],
        (tracking == NULL) ? 0U : tracking->completed_pairs);
    Host_WriteU32Le(&response.payload[20],
        (tracking == NULL) ? 0U : tracking->failed_pairs);
    Host_WriteU32Le(&response.payload[24],
        (tracking == NULL) ? 0U : tracking->maximum_reply_latency_ms);
    for (uint8_t joint = 0U;
         joint < ACTUATOR_V2_JOINT_COUNT;
         joint++)
    {
        Host_WriteU32Le(
            &response.payload[28U + ((uint16_t)joint * 4U)],
            (executor == NULL) ? 0U :
                executor->maximum_tracking_error_urad[joint]);
    }
    (void)Host_SendBinaryFrame(&response);
}
#endif

#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
static void Host_SendV2FeedbackSnapshot(const actuator_frame_t *request)
{
    actuator_frame_t response;
    BimanualFeedbackSnapshot snapshot;

    BimanualFeedbackSnapshot_Copy(HAL_GetTick(), &snapshot);
    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_FEEDBACK_SNAPSHOT;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = ACTUATOR_V2_FEEDBACK_SNAPSHOT_WIRE_SIZE;
    response.payload[0] =
        (snapshot.present_mask == BIMANUAL_FEEDBACK_COMPLETE_MASK) ? 0U : 1U;
    response.payload[1] = ACTUATOR_V2_JOINT_COUNT;
    Host_WriteU16Le(&response.payload[2], snapshot.present_mask);
    Host_WriteU32Le(&response.payload[4], request->sequence);
    Host_WriteU32Le(&response.payload[8], request->sender_time_ms);
    Host_WriteU32Le(&response.payload[12], snapshot.firmware_tick_ms);
    Host_WriteU32Le(&response.payload[16], snapshot.completed_pairs);
    for (uint8_t joint = 0U;
         joint < ACTUATOR_V2_JOINT_COUNT;
         joint++)
    {
        Host_WriteU32Le(
            &response.payload[20U + ((uint16_t)joint * 4U)],
            (uint32_t)snapshot.positions_urad[joint]
        );
        Host_WriteU32Le(
            &response.payload[68U + ((uint16_t)joint * 4U)],
            snapshot.sample_age_ms[joint]
        );
    }
    (void)Host_SendBinaryFrame(&response);
}
#endif
#endif

#if HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
#if !HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
static actuator_calibration_result_t Host_ShadowFeedbackRawToUrad(
    const actuator_joint_calibration_t *calibration,
    uint16_t raw,
    int32_t *position_urad
)
{
    actuator_joint_calibration_t feedback_calibration;

    if (calibration == NULL)
    {
        return ACTUATOR_CALIBRATION_NULL_ARGUMENT;
    }
    feedback_calibration = *calibration;
    feedback_calibration.minimum_raw =
        (calibration->minimum_raw > HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW) ?
        (uint16_t)(calibration->minimum_raw -
                   HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW) :
        0U;
    feedback_calibration.maximum_raw =
        (calibration->maximum_raw <
         (uint16_t)(4095U - HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW)) ?
        (uint16_t)(calibration->maximum_raw +
                   HOST_SHADOW_FEEDBACK_LIMIT_MARGIN_RAW) :
        4095U;
    return actuator_raw_to_urad(
        &feedback_calibration,
        raw,
        position_urad
    );
}
#endif

static void Host_SendV2ShadowSnapshot(const actuator_frame_t *request)
{
    actuator_frame_t response;
    RightServoDisableSnapshot right_disable;
    const RightServoDiscoverySnapshot *right_snapshot = NULL;
    uint16_t left_raw[SINGLE_ARM_JOINT_COUNT] = {0U};
    uint8_t status = 0U;
    uint8_t left_mask = 0U;
    uint8_t right_mask = 0U;
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    uint16_t maximum_reference_delta_raw = 0U;
    int32_t reference_unwrapped_raw[ACTUATOR_V2_JOINT_COUNT] = {0};
    actuator_joint_unwrapper_t
        candidate_unwrappers[ACTUATOR_V2_JOINT_COUNT];
    const uint8_t reference_bind_request =
        (request->payload_length == 52U) ? 1U : 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    const uint8_t automatic_bind_request =
        (request->payload_length == 0U) ? 1U : 0U;
#else
    const uint8_t automatic_bind_request = 0U;
#endif
    const uint8_t bind_request =
        (uint8_t)(reference_bind_request | automatic_bind_request);
#endif

    memset(&response, 0, sizeof(response));
    response.message_type = ACTUATOR_V2_MSG_SHADOW_SNAPSHOT;
    response.sequence = request->sequence;
    response.sender_time_ms = HAL_GetTick();
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    response.payload_length = 124U;
#else
    response.payload_length = ACTUATOR_V2_SHADOW_SNAPSHOT_WIRE_SIZE;
#endif
    response.payload[1] = ACTUATOR_V2_JOINT_COUNT;
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    if (bind_request != 0U)
    {
#endif
    host_v2_shadow_anchor_ready = 0U;
    memset(host_v2_shadow_raw, 0, sizeof(host_v2_shadow_raw));
    memset(
        host_v2_shadow_anchor_urad,
        0,
        sizeof(host_v2_shadow_anchor_urad)
    );
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    memset(
        host_v2_shadow_unwrapped_raw,
        0,
        sizeof(host_v2_shadow_unwrapped_raw)
    );
#endif
    memset(
        host_v2_shadow_executor_anchor_urad,
        0,
        sizeof(host_v2_shadow_executor_anchor_urad)
    );
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    }
#endif

#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
    if ((request->payload_length != 0U) &&
        (request->payload_length != 52U))
    {
        status = 7U;
    }
    else if (reference_bind_request != 0U)
    {
        maximum_reference_delta_raw = Host_ReadU16Le(&request->payload[0]);
        if ((Host_ReadU16Le(&request->payload[2]) != 0U) ||
            (maximum_reference_delta_raw == 0U) ||
            (maximum_reference_delta_raw >=
             ACTUATOR_UNWRAP_HALF_TURN_RAW))
        {
            status = 7U;
        }
        else
        {
            for (uint8_t joint = 0U;
                 joint < ACTUATOR_V2_JOINT_COUNT;
                 joint++)
            {
                reference_unwrapped_raw[joint] = Host_ReadI32Le(
                    &request->payload[4U + (uint16_t)joint * 4U]
                );
            }
        }
    }
    else if ((automatic_bind_request == 0U) &&
             (host_v2_shadow_anchor_ready == 0U))
    {
        status = 7U;
    }

    if (bind_request != 0U)
    {
        for (uint8_t joint = 0U;
             joint < ACTUATOR_V2_JOINT_COUNT;
             joint++)
        {
            actuator_joint_unwrapper_reset(&candidate_unwrappers[joint]);
        }
    }
    else
    {
        memcpy(
            candidate_unwrappers,
            host_v2_shadow_unwrappers,
            sizeof(candidate_unwrappers)
        );
    }
#endif

    if ((status == 0U) &&
        ((host_v2_stream_executor.diagnostics.state ==
         ACTUATOR_V2_EXECUTOR_RUNNING) ||
         (host_v2_executor_clock_active != 0U)))
    {
        status = 1U;
    }

    if ((status == 0U) && (Servo_DisableTorqueAll() != HAL_OK))
    {
        status = 2U;
    }
    right_disable = RightServoBus_DisableTorqueAllVerified();
    if ((right_disable.status != RIGHT_SERVO_DISABLE_OK) && (status == 0U))
    {
        status = 3U;
    }
    host_binary_servos_configured = 0U;
    host_right_arm_output_active = 0U;

    if ((status == 0U) && (Servo_ReadAllPositions(left_raw) != HAL_OK))
    {
        status = 4U;
    }
    if (status == 0U)
    {
        left_mask = UINT8_C(0x3F);
        right_snapshot = RightServoBus_Discover();
        if ((right_snapshot == NULL) ||
            (right_snapshot->present_mask != UINT8_C(0x3F)) ||
            (right_snapshot->failure_count != 0U))
        {
            status = 5U;
        }
        else
        {
            right_mask = right_snapshot->present_mask;
        }
    }

    if (status == 0U)
    {
        for (uint8_t joint = 0U; joint < ACTUATOR_V2_JOINT_COUNT; joint++)
        {
            const uint8_t arm_joint =
                (uint8_t)(joint % SINGLE_ARM_JOINT_COUNT);
            const actuator_joint_calibration_t calibration =
                Host_JointCalibration(arm_joint);
            const uint16_t raw = (joint < SINGLE_ARM_JOINT_COUNT) ?
                left_raw[joint] :
                right_snapshot->positions[arm_joint];
            host_v2_shadow_raw[joint] = raw;
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
            actuator_unwrap_result_t unwrap_result;
            int32_t unwrapped_raw = 0;
            if (reference_bind_request != 0U)
            {
                unwrap_result =
                    actuator_joint_unwrapper_bind(
                        &candidate_unwrappers[joint],
                        raw,
                        reference_unwrapped_raw[joint],
                        maximum_reference_delta_raw
                    );
                unwrapped_raw =
                    candidate_unwrappers[joint].unwrapped_raw;
            }
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
            else if (automatic_bind_request != 0U)
            {
                const BimanualArm arm =
                    (joint < SINGLE_ARM_JOINT_COUNT) ?
                    BIMANUAL_ARM_LEFT : BIMANUAL_ARM_RIGHT;
                if (!BimanualOperationalLimits_UnwrapModuloRaw(
                        arm, arm_joint, raw, &unwrapped_raw))
                {
                    status = 8U;
                    break;
                }
                unwrap_result = actuator_joint_unwrapper_bind(
                    &candidate_unwrappers[joint],
                    raw,
                    unwrapped_raw,
                    1U
                );
            }
#endif
            else
            {
                unwrap_result =
                    actuator_joint_unwrapper_update(
                        &candidate_unwrappers[joint],
                        raw,
                        &unwrapped_raw
                    );
            }
            if ((unwrap_result != ACTUATOR_UNWRAP_OK) ||
                (actuator_unwrapped_raw_to_urad(
                     calibration.zero_raw,
                     calibration.positive_raw_direction,
                     unwrapped_raw,
                     &host_v2_shadow_anchor_urad[joint]
                 ) != ACTUATOR_UNWRAP_OK))
            {
                status = 8U;
                break;
            }
            host_v2_shadow_unwrapped_raw[joint] = unwrapped_raw;
            host_v2_shadow_executor_anchor_urad[joint] =
                host_v2_shadow_anchor_urad[joint];
#else
            uint16_t executor_raw = raw;
            if (Host_ShadowFeedbackRawToUrad(
                    &calibration,
                    raw,
                    &host_v2_shadow_anchor_urad[joint]
                ) != ACTUATOR_CALIBRATION_OK)
            {
                status = 6U;
                break;
            }
            if (executor_raw < calibration.minimum_raw)
            {
                executor_raw = calibration.minimum_raw;
            }
            else if (executor_raw > calibration.maximum_raw)
            {
                executor_raw = calibration.maximum_raw;
            }
            if (actuator_raw_to_urad(
                    &calibration,
                    executor_raw,
                    &host_v2_shadow_executor_anchor_urad[joint]
                ) != ACTUATOR_CALIBRATION_OK)
            {
                status = 6U;
                break;
            }
#endif
        }
    }

    if (status == 0U)
    {
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
        memcpy(
            host_v2_shadow_unwrappers,
            candidate_unwrappers,
            sizeof(host_v2_shadow_unwrappers)
        );
#endif
        host_v2_shadow_anchor_ready = 1U;
#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
        BimanualFeedbackSnapshot_Seed(
            host_v2_shadow_anchor_urad,
            HAL_GetTick()
        );
#endif
        Host_ResetPositionReadFailure();
    }
    else
    {
        host_stop_latched = 1U;
        actuator_safety_report_fault(
            &host_binary_safety,
            UINT16_C(0xFF05)
        );
    }

    response.payload[0] = status;
    response.payload[2] = left_mask;
    response.payload[3] = right_mask;
    for (uint8_t joint = 0U; joint < ACTUATOR_V2_JOINT_COUNT; joint++)
    {
        Host_WriteU16Le(
            &response.payload[4U + (uint16_t)joint * 2U],
            host_v2_shadow_raw[joint]
        );
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
        Host_WriteU32Le(
            &response.payload[28U + (uint16_t)joint * 4U],
            (uint32_t)host_v2_shadow_unwrapped_raw[joint]
        );
        Host_WriteU32Le(
            &response.payload[76U + (uint16_t)joint * 4U],
            (uint32_t)host_v2_shadow_anchor_urad[joint]
        );
#else
        Host_WriteU32Le(
            &response.payload[28U + (uint16_t)joint * 4U],
            (uint32_t)host_v2_shadow_anchor_urad[joint]
        );
#endif
    }
    (void)Host_SendBinaryFrame(&response);
}
#endif
#endif
#endif

static void Host_HandleBinaryFrame(
    const actuator_frame_t *request,
    uint32_t received_at_ms
)
{
    if (request == NULL)
    {
        return;
    }

#if HOST_F25_VALIDATION_ONLY_BUILD
    if (Host_F25RejectsRequest(request) != 0U)
    {
        host_binary_rejected_frame_count++;
        Host_SendBinaryState(request->sequence, 8U);
        return;
    }
#endif

    switch (request->message_type)
    {
        case ACTUATOR_MSG_HELLO_REQUEST:
            if (request->payload_length == 0U)
            {
                Host_SendBinaryHello(request->sequence);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_HEARTBEAT:
            if (request->payload_length == 0U)
            {
                host_binary_last_heartbeat_ms = received_at_ms;
                host_binary_heartbeat_count++;
                actuator_safety_on_heartbeat(
                    &host_binary_safety,
                    host_binary_last_heartbeat_ms
                );
                Host_SendBinaryState(request->sequence, 0U);
            }
            break;

        case ACTUATOR_MSG_GET_STATE:
            if (request->payload_length == 0U)
            {
                Host_SendBinaryState(request->sequence, 0U);
            }
#if !HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
            else if ((request->payload_length == 1U) &&
                     (request->payload[0] == 1U))
            {
                Host_SendBinaryStateWithPositions(request->sequence);
            }
            else if ((request->payload_length == 2U) &&
                     (request->payload[0] == 2U) &&
                     (request->payload[1] < servo_joint_count))
            {
                Host_SendBinaryDiagnostics(
                    request->sequence,
                    request->payload[1]
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
#else
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
#endif
            break;

#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
        case ACTUATOR_V2_MSG_STREAM_OPEN:
            Host_ValidateV2StreamOpen(request);
            break;

        case ACTUATOR_V2_MSG_SPLICE:
            Host_ValidateV2Batch(request, ACTUATOR_V2_BATCH_SPLICE);
            break;

#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
        case ACTUATOR_V2_MSG_GET_EXECUTOR_DIAGNOSTICS:
            if (request->payload_length == 0U)
            {
                Host_SendV2ExecutorDiagnostics(request);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
        case ACTUATOR_V2_MSG_GET_DISPATCH_DIAGNOSTICS:
            if (request->payload_length == 0U)
            {
                Host_SendV2DispatchDiagnostics(request);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
        case ACTUATOR_V2_MSG_GET_TRACKING_DIAGNOSTICS:
            if (request->payload_length == 0U)
            {
                Host_SendV2TrackingDiagnostics(request);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;
#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
        case ACTUATOR_V2_MSG_GET_FEEDBACK_SNAPSHOT:
            if (request->payload_length == 0U)
            {
                Host_SendV2FeedbackSnapshot(request);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;
#endif
#endif
#endif
#if HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
        case ACTUATOR_V2_MSG_PREPARE_SHADOW:
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
            if ((request->payload_length == 0U) ||
                (request->payload_length == 52U))
#else
            if (request->payload_length == 0U)
#endif
            {
                Host_SendV2ShadowSnapshot(request);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;
#endif
#endif
#endif

        case ACTUATOR_MSG_RIGHT_ARM_DISCOVERY_REQUEST:
            if (request->payload_length == 0U)
            {
                Host_SendRightArmDiscovery(request->sequence);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_REQUEST:
            if (request->payload_length == 2U)
            {
                Host_SendRightArmJogOnce(
                    request->sequence,
                    request->payload[0],
                    (int8_t)request->payload[1]
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST:
            if (request->payload_length == 1U)
            {
                Host_SendRightArmTorqueEnableOnce(
                    request->sequence,
                    request->payload[0]
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURE_ONCE_REQUEST:
            if (request->payload_length == 1U)
            {
                Host_SendRightArmConfigureOnce(
                    request->sequence,
                    request->payload[0]
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_RIGHT_ARM_CONFIGURATION_REQUEST:
            if (request->payload_length == 1U)
            {
                Host_SendRightArmConfiguration(
                    request->sequence,
                    request->payload[0]
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_RIGHT_ARM_DISABLE_REQUEST:
            if (request->payload_length == 0U)
            {
                Host_SendRightArmDisableVerified(request->sequence);
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_ARM_REQUEST:
            if (request->payload_length == 4U)
            {
                uint32_t expected_hash =
                    Host_ReadU32Le(&request->payload[0]);
                uint8_t health_ok = 1U;

                if ((expected_hash == Host_CalibrationHash()) &&
                    (host_binary_servos_configured == 0U))
                {
                    uint16_t configured_positions[6] = {0U};
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
                    (void)configured_positions;
                    if (Host_ConfigureBimanualForTrajectory() != 0U)
#else
                    if (Servo_ConfigureAllForTrajectory(
                            configured_positions
                        ) == HAL_OK)
#endif
                    {
                        host_binary_servos_configured = 1U;
                    }
                    else
                    {
                        health_ok = 0U;
                    }
                }

                actuator_safety_result_t arm_result =
                    actuator_safety_request_arm(
                        &host_binary_safety,
                        health_ok != 0U,
                        expected_hash == Host_CalibrationHash()
                    );
                if (arm_result == ACTUATOR_SAFETY_OK)
                {
                    Host_ResetPositionReadFailure();
                }
                Host_SendBinaryArmResponse(
                    request->sequence,
                    arm_result
                );
            }
            else
            {
                Host_SendBinaryArmResponse(
                    request->sequence,
                    ACTUATOR_SAFETY_CONFIG_MISMATCH
                );
            }
            break;

        case ACTUATOR_MSG_ENABLE:
            if (request->payload_length == 0U)
            {
                actuator_safety_result_t enable_result =
                    actuator_safety_request_enable(
                        &host_binary_safety,
                        HAL_GetTick()
                    );
                Host_SendBinaryState(
                    request->sequence,
                    (uint8_t)enable_result
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_SETPOINT_BATCH:
#if HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
            Host_ValidateV2Batch(request, ACTUATOR_V2_BATCH_APPEND);
#else
            if ((request->flags & ACTUATOR_BUFFERED_FLAG_CANDIDATE) != 0U)
            {
                if ((request->flags &
                     ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY) != 0U)
                {
                    Host_ValidateBufferedCandidate(request);
                }
                else
                {
                    Host_ExecuteBufferedCandidate(request);
                }
            }
            else
            {
                Host_ValidateLegacyBinarySetpointBatch(request);
            }
#endif
            break;

        case ACTUATOR_MSG_SAFE_STOP:
            if (request->payload_length == 0U)
            {
                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_OPERATOR_CANCEL,
                    0U
                );
                if (actuator_safety_accepts_setpoint(
                        &host_binary_safety))
                {
                    (void)actuator_safety_request_hold(
                        &host_binary_safety
                    );
                }
                host_stop_latched = 1U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
                Host_SendBinaryState(
                    request->sequence,
                    Host_PerformV2CoordinatedStop(0U));
#else
                if ((host_right_arm_output_active != 0U) &&
                    (RightServoBus_DisableTorqueAll() != HAL_OK))
                {
                    actuator_safety_report_fault(
                        &host_binary_safety, UINT16_C(0xFF03));
                    Host_SendBinaryState(request->sequence, 2U);
                }
                else
                {
                    host_right_arm_output_active = 0U;
                    Host_SendBinaryState(request->sequence, 0U);
                }
#endif
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_CLEAR_FAULT:
        {
            uint8_t clear_status = 0U;

            if (host_stop_latched != 0U)
            {
                clear_status = Host_BinaryClearStopIsSafe();
                if (clear_status == 0U)
                {
                    if ((host_binary_safety.state ==
                         ACTUATOR_STATE_FAULT) ||
                        (host_binary_safety.state ==
                         ACTUATOR_STATE_ESTOPPED))
                    {
                        actuator_safety_result_t recovery_result =
                            actuator_safety_clear_latched_stop(
                                &host_binary_safety,
                                true
                            );
                        if (recovery_result != ACTUATOR_SAFETY_OK)
                        {
                            clear_status = (uint8_t)recovery_result;
                        }
                    }
                    else if (host_binary_safety.state !=
                             ACTUATOR_STATE_SAFE_DISABLED)
                    {
                        actuator_safety_result_t disable_result =
                            actuator_safety_request_disable(
                                &host_binary_safety
                            );
                        if (disable_result != ACTUATOR_SAFETY_OK)
                        {
                            clear_status = (uint8_t)disable_result;
                        }
                    }

                    if (clear_status == 0U)
                    {
                        host_stop_latched = 0U;
                        Host_ResetPositionReadFailure();
                    }
                }
            }

            Host_SendBinaryState(request->sequence, clear_status);
            break;
        }

        case ACTUATOR_MSG_HOLD:
            if ((request->payload_length == 0U) &&
                actuator_safety_accepts_setpoint(
                    &host_binary_safety))
            {
                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_PLANNED_HOLD,
                    0U
                );
                actuator_safety_result_t hold_result =
                    actuator_safety_request_hold(
                        &host_binary_safety
                    );
                host_stop_latched = 1U;
                Host_SendBinaryState(
                    request->sequence,
                    (uint8_t)hold_result
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        case ACTUATOR_MSG_DISABLE:
            if (request->payload_length == 0U)
            {
                actuator_safety_result_t disable_result =
                    ACTUATOR_SAFETY_OK;

                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_OPERATOR_CANCEL,
                    0U
                );

                /*
                 * DISABLE is an idempotent physical safety operation.  A
                 * logical FAULT/ESTOP latch must never block six-axis torque
                 * removal, and a successful physical readback must not be
                 * reported as BAD_STATE.  Preserve the latched logical state;
                 * only non-faulted states transition to SAFE_DISABLED.
                 */
                if ((host_binary_safety.state != ACTUATOR_STATE_FAULT) &&
                    (host_binary_safety.state != ACTUATOR_STATE_ESTOPPED))
                {
                    disable_result = actuator_safety_request_disable(
                        &host_binary_safety
                    );
                }

                /*
                 * Do not report physical success until all six Torque Enable
                 * registers have been written and read back as zero.  Mark the
                 * trajectory configuration stale so the next ARM request must
                 * explicitly configure and re-enable servos.
                 */
                host_binary_servos_configured = 0U;
                if (Servo_DisableTorqueAll() != HAL_OK)
                {
                    host_stop_latched = 1U;
                    actuator_safety_report_fault(
                        &host_binary_safety,
                        UINT16_C(0xFF02)
                    );
                    disable_result = ACTUATOR_SAFETY_HEALTH_FAILED;
                }
                else
                {
                    Host_ResetPositionReadFailure();
                }

                if ((host_right_arm_output_active != 0U) &&
                    (RightServoBus_DisableTorqueAll() != HAL_OK))
                {
                    host_stop_latched = 1U;
                    actuator_safety_report_fault(
                        &host_binary_safety,
                        UINT16_C(0xFF03)
                    );
                    disable_result = ACTUATOR_SAFETY_HEALTH_FAILED;
                }
                else
                {
                    host_right_arm_output_active = 0U;
                }

                Host_SendBinaryState(
                    request->sequence,
                    (uint8_t)disable_result
                );
            }
            else
            {
                Host_SendBinaryState(request->sequence, 1U);
            }
            break;

        default:
            Host_SendBinaryState(request->sequence, 4U);
            break;
    }
}

static void Host_ProcessBinaryByte(uint8_t byte, uint32_t received_at_ms)
{
    actuator_frame_t request;
    actuator_protocol_result_t result =
        actuator_stream_parser_push(
            &host_binary_parser,
            byte,
            &request
        );

    if (result == ACTUATOR_PROTOCOL_OK)
    {
        Host_HandleBinaryFrame(&request, received_at_ms);
    }
    else if (result != ACTUATOR_PROTOCOL_NO_FRAME)
    {
        host_binary_rejected_frame_count++;
    }
}



void BinaryControl_Init(UART_HandleTypeDef *host_uart)
{
    binary_host_uart = host_uart;
    HostUartTx_Init(host_uart);
    host_stop_latched = 0U;
    host_binary_heartbeat_count = 0U;
    host_binary_rejected_frame_count = 0U;
    host_binary_last_heartbeat_ms = 0U;
    host_binary_mode = 0U;
    host_binary_servos_configured = 0U;
    host_right_arm_output_active = 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    host_bimanual_arm_watchdog_grace_started_ms = 0U;
#endif
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
    {
        const actuator_v2_stream_hard_caps_t caps = Host_V2HardCaps();
        actuator_v2_joint_limit_t limits[ACTUATOR_V2_JOINT_COUNT];
        Host_V2JointLimits(limits);
        host_v2_executor_ready =
            (actuator_v2_stream_executor_init(
                &host_v2_stream_executor,
                &caps,
                limits
            ) == ACTUATOR_V2_EXECUTOR_OK) ? 1U : 0U;
        host_v2_executor_clock_active = 0U;
        host_v2_executor_next_tick = 0U;
        #if HOST_BIMANUAL_DMA_DISPATCH_BUILD
        memset(host_v2_output_urad, 0, sizeof(host_v2_output_urad));
        host_v2_coordinated_stop_pending = 0U;
        host_v2_executor_start_pending = 0U;
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
        host_v2_tracking_next_joint = 0U;
        host_v2_tracking_last_dispatch_completed = 0U;
#if HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD
        BimanualFeedbackSnapshot_Reset();
#endif
        memset(host_v2_last_left_raw, 0, sizeof(host_v2_last_left_raw));
        memset(host_v2_last_right_raw, 0, sizeof(host_v2_last_right_raw));
#if HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
        host_v2_tracking_fault_injection_consumed = 0U;
#endif
#endif
#if HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD || \
    HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD
        host_v2_last_coordinated_stop_status = 2U;
#endif
        ControlTick_ClearPending();
#else
        memset(
            host_v2_discarded_output_urad,
            0,
            sizeof(host_v2_discarded_output_urad)
        );
#endif
#if HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD
        host_v2_shadow_anchor_ready = 0U;
        memset(host_v2_shadow_raw, 0, sizeof(host_v2_shadow_raw));
        memset(
            host_v2_shadow_anchor_urad,
            0,
            sizeof(host_v2_shadow_anchor_urad)
        );
#if HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD
        memset(
            host_v2_shadow_unwrapped_raw,
            0,
            sizeof(host_v2_shadow_unwrapped_raw)
        );
        for (uint8_t joint = 0U;
             joint < ACTUATOR_V2_JOINT_COUNT;
             joint++)
        {
            actuator_joint_unwrapper_reset(
                &host_v2_shadow_unwrappers[joint]
            );
        }
#endif
        memset(
            host_v2_shadow_executor_anchor_urad,
            0,
            sizeof(host_v2_shadow_executor_anchor_urad)
        );
#endif
    }
#elif HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD
    actuator_v2_stream_session_init(&host_v2_stream_session);
#endif
    host_position_read_failure_streak = 0U;
    host_position_read_failed_servo_id = 0U;
    host_buffered_validation_route_ready =
        Host_InitBufferedValidationRoute();
    host_buffered_execution_route_ready =
        Host_InitBufferedExecutionRoute();
    memset(&host_binary_motion, 0, sizeof(host_binary_motion));
    memset(
        &host_binary_buffered_motion,
        0,
        sizeof(host_binary_buffered_motion)
    );
    Servo_MotionSafetyEnd();

    actuator_stream_parser_init(&host_binary_parser);
    actuator_safety_init(
        &host_binary_safety,
        HOST_BINARY_HEARTBEAT_TIMEOUT_MS
    );
    (void)actuator_safety_complete_boot(
        &host_binary_safety,
        true
    );
}

void BinaryControl_Service(void)
{
    uint32_t now_ms = HAL_GetTick();

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    if ((host_v2_coordinated_stop_pending != 0U) ||
        (BimanualServoDispatch_Faulted() != 0U))
    {
        (void)Host_PerformV2CoordinatedStop(1U);
    }
#endif

    if (HostUartTx_TakeFault() != 0U)
    {
        /* No response is trustworthy after a DMA/queue fault. Latch HOLD
         * before parsing or applying another host command. */
        host_stop_latched = 1U;
        Host_AbortBufferedExecution(
            ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS,
            0U
        );
        if (actuator_safety_accepts_setpoint(&host_binary_safety))
        {
            (void)actuator_safety_request_hold(&host_binary_safety);
        }
    }

    if ((host_binary_mode != 0U) &&
        (host_binary_heartbeat_count != 0U) &&
        (host_binary_safety.state == ACTUATOR_STATE_ACTIVE) &&
        (host_stop_latched == 0U) &&
        ((uint32_t)(now_ms - host_binary_last_heartbeat_ms) >
            HOST_BINARY_HEARTBEAT_TIMEOUT_MS))
    {
        host_stop_latched = 1U;
    }

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    if ((host_right_arm_output_active != 0U) &&
        ((host_binary_heartbeat_count == 0U) ||
         (((uint32_t)(now_ms -
              host_bimanual_arm_watchdog_grace_started_ms) >
            HOST_BINARY_HEARTBEAT_TIMEOUT_MS) &&
          ((uint32_t)(now_ms - host_binary_last_heartbeat_ms) >
            HOST_BINARY_HEARTBEAT_TIMEOUT_MS))))
#else
    if ((host_right_arm_output_active != 0U) &&
        ((host_binary_heartbeat_count == 0U) ||
         ((uint32_t)(now_ms - host_binary_last_heartbeat_ms) >
             HOST_BINARY_HEARTBEAT_TIMEOUT_MS)))
#endif
    {
        host_stop_latched = 1U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
        (void)Host_PerformV2CoordinatedStop(1U);
#else
        if (RightServoBus_DisableTorqueAll() != HAL_OK)
        {
            actuator_safety_report_fault(
                &host_binary_safety, UINT16_C(0xFF03));
        }
        host_right_arm_output_active = 0U;
#endif
    }

    actuator_safety_tick(&host_binary_safety, now_ms);
    if (host_binary_safety.state == ACTUATOR_STATE_HOLD)
    {
        host_stop_latched = 1U;
    }
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    if ((host_stop_latched != 0U) &&
        (host_right_arm_output_active != 0U))
    {
        (void)Host_PerformV2CoordinatedStop(1U);
    }
#endif

    Host_ServiceBufferedExecution();
    Host_ServiceBinaryMotion();
#if HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    Host_ServiceV2TrackingFeedback();
#endif
    Host_ServiceV2Executor();
#endif
}

void BinaryControl_EnterMode(void)
{
    actuator_stream_parser_init(&host_binary_parser);
    host_binary_mode = 1U;
}

uint8_t BinaryControl_IsBinaryMode(void)
{
    return host_binary_mode;
}

void BinaryControl_ProcessByte(uint8_t byte, uint32_t received_at_ms)
{
    Host_ProcessBinaryByte(byte, received_at_ms);
}

void BinaryControl_HandleHostUartError(void)
{
    if ((binary_host_uart != NULL) &&
        (__HAL_UART_GET_FLAG(binary_host_uart, UART_FLAG_ORE) != RESET))
    {
        __HAL_UART_CLEAR_OREFLAG(binary_host_uart);
        __HAL_UART_SEND_REQ(binary_host_uart, UART_RXDATA_FLUSH_REQUEST);
    }

    /*
     * The ISR ring reports overrun, framing, noise, rearm, and capacity faults
     * after HAL may already have cleared the peripheral flag. Every reported RX
     * fault invalidates the parser and must fail closed regardless of ORE state.
     */
    Host_AbortBufferedExecution(
        ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS,
        0U
    );
    actuator_stream_parser_init(&host_binary_parser);
    host_binary_rejected_frame_count++;
    if (actuator_safety_accepts_setpoint(&host_binary_safety))
    {
        (void)actuator_safety_request_hold(&host_binary_safety);
    }
    host_stop_latched = 1U;
}

uint8_t BinaryControl_StopIsLatched(void)
{
    return host_stop_latched;
}

void BinaryControl_LatchStop(void)
{
    host_stop_latched = 1U;
}

void BinaryControl_ClearStopLatch(void)
{
    host_stop_latched = 0U;
    Host_ResetPositionReadFailure();
}
