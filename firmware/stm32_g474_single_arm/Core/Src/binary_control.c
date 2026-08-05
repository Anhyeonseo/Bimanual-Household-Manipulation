#include "binary_control.h"

#include "single_arm_config.h"
#include "servo_bus.h"
#include "actuator_core/buffered_command_route.h"
#include "actuator_core/calibration.h"
#include "actuator_core/crc32c.h"
#include "actuator_core/protocol.h"
#include "actuator_core/safety.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

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
static uint8_t host_position_read_failure_streak = 0U;
static uint8_t host_position_read_failed_servo_id = 0U;
static actuator_buffered_command_route_t host_buffered_validation_route;
static uint8_t host_buffered_validation_route_ready = 0U;
static actuator_buffered_command_route_t host_buffered_execution_route;
static uint8_t host_buffered_execution_route_ready = 0U;
static HostBinaryBufferedMotion host_binary_buffered_motion;

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
    uint8_t calibration_bytes[54] = {0U};
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

    return HAL_UART_Transmit(
        binary_host_uart,
        encoded,
        (uint16_t)encoded_length,
        100U
    );
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
    response.payload_length = 20U;
    response.payload[0] = (host_stop_latched != 0U) ? 1U : 0U;
    response.payload[1] = status_code;
    response.payload[2] = servo_joint_count;
    response.payload[3] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    Host_WriteU32Le(&response.payload[4], host_binary_heartbeat_count);
    Host_WriteU32Le(&response.payload[8], host_binary_rejected_frame_count);
    Host_WriteU32Le(&response.payload[12], Host_CalibrationHash());
    Host_WriteU32Le(
        &response.payload[16],
        host_binary_last_heartbeat_ms
    );

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
    response.payload_length = 138U;

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
    response.payload[120] = bus->snapshot_length;
    response.payload[121] = health->dma_started;
    memcpy(
        &response.payload[122],
        bus->snapshot,
        SERVO_BUS_FAILURE_SNAPSHOT_MAX_BYTES
    );

    (void)Host_SendBinaryFrame(&response);
}

static void Host_SendBinaryHello(uint32_t request_sequence)
{
    actuator_frame_t response;
    memset(&response, 0, sizeof(response));

    response.message_type = ACTUATOR_MSG_HELLO_RESPONSE;
    response.sequence = request_sequence;
    response.sender_time_ms = HAL_GetTick();
    response.payload_length = 20U;
    response.payload[0] = (uint8_t)ACTUATOR_PROTOCOL_VERSION;
    response.payload[1] = servo_joint_count;
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
    Host_WriteU32Le(
        &response.payload[12],
        Host_BinaryCapabilities()
    );
    Host_WriteU32Le(
        &response.payload[16],
        host_binary_rejected_frame_count
    );

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
            diagnostics
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
        6U,
        0U,
        apply_tick,
        detail
    );
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
                        Servo_MotionSafetyBegin(
                            (uint8_t)(
                                (1U << SINGLE_ARM_JOINT_COUNT) - 1U
                            )
                        );
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
        if (Servo_MotionSafetyPoll() != HAL_OK)
        {
            const ServoMotionSafetyDiagnostics *safety =
                Servo_MotionSafetyGetDiagnostics();
            Host_AbortBufferedExecution(
                ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                safety->servo_id
            );
        }
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

        if (write_due != 0U)
        {
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

            if (Servo_SyncWritePositions(output_positions_raw) != HAL_OK)
            {
                Host_AbortBufferedExecution(
                    ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
                    0U
                );
                return;
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

    if (Servo_MotionSafetyPoll() != HAL_OK)
    {
        const ServoMotionSafetyDiagnostics *safety =
            Servo_MotionSafetyGetDiagnostics();
        Host_AbortBufferedExecution(
            ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
            safety->servo_id
        );
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

static void Host_HandleBinaryFrame(const actuator_frame_t *request)
{
    if (request == NULL)
    {
        return;
    }

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
                host_binary_last_heartbeat_ms = HAL_GetTick();
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
                    if (Servo_ConfigureAllForTrajectory(
                            configured_positions
                        ) == HAL_OK)
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
                Host_SendBinaryState(request->sequence, 0U);
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
                    host_stop_latched = 0U;
                    Host_ResetPositionReadFailure();
                    if (host_binary_safety.state !=
                        ACTUATOR_STATE_SAFE_DISABLED)
                    {
                        (void)actuator_safety_request_disable(
                            &host_binary_safety
                        );
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

static void Host_ProcessBinaryByte(uint8_t byte)
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
        Host_HandleBinaryFrame(&request);
    }
    else if (result != ACTUATOR_PROTOCOL_NO_FRAME)
    {
        host_binary_rejected_frame_count++;
    }
}



void BinaryControl_Init(UART_HandleTypeDef *host_uart)
{
    binary_host_uart = host_uart;
    host_stop_latched = 0U;
    host_binary_heartbeat_count = 0U;
    host_binary_rejected_frame_count = 0U;
    host_binary_last_heartbeat_ms = 0U;
    host_binary_mode = 0U;
    host_binary_servos_configured = 0U;
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
    if ((host_binary_mode != 0U) &&
        (host_binary_heartbeat_count != 0U) &&
        (host_binary_safety.state == ACTUATOR_STATE_ACTIVE) &&
        (host_stop_latched == 0U) &&
        ((uint32_t)(HAL_GetTick() - host_binary_last_heartbeat_ms) >
            HOST_BINARY_HEARTBEAT_TIMEOUT_MS))
    {
        host_stop_latched = 1U;
    }

    actuator_safety_tick(&host_binary_safety, HAL_GetTick());
    if (host_binary_safety.state == ACTUATOR_STATE_HOLD)
    {
        host_stop_latched = 1U;
    }

    Host_ServiceBufferedExecution();
    Host_ServiceBinaryMotion();
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

void BinaryControl_ProcessByte(uint8_t byte)
{
    Host_ProcessBinaryByte(byte);
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
