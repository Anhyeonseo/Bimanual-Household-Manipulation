#include "binary_control.h"

#include "single_arm_config.h"
#include "servo_bus.h"
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
    uint8_t verifying;
    uint32_t request_sequence;
    uint32_t start_tick;
    uint32_t duration_ms;
    uint32_t last_control_tick;
    uint16_t start_positions[SINGLE_ARM_JOINT_COUNT];
    uint16_t target_positions[SINGLE_ARM_JOINT_COUNT];
} HostBinaryMotion;

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

static void Host_SendBinaryStateWithPositions(
    uint32_t request_sequence
)
{
    uint16_t positions[SINGLE_ARM_JOINT_COUNT] = {0U};

    if (Servo_ReadAllPositions(positions) != HAL_OK)
    {
        host_stop_latched = 1U;
        Host_SendBinaryState(request_sequence, 2U);
        return;
    }

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
        HOST_BINARY_CAPABILITIES
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

    if ((host_binary_servos_configured == 0U) ||
        (Servo_ReadAllPositions(
            host_binary_motion.start_positions
        ) != HAL_OK))
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

    for (uint8_t joint = 0U;
         joint < servo_joint_count;
         joint++)
    {
        host_binary_motion.target_positions[joint] =
            target_positions[joint];
    }

    host_binary_motion.start_tick = HAL_GetTick();
    if ((int32_t)(first_apply_tick -
            host_binary_motion.start_tick) < 20)
    {
        Host_SendBinarySetpointStatus(
            request->sequence,
            1U,
            1U,
            first_apply_tick,
            0U
        );
        return;
    }

    host_binary_motion.request_sequence = request->sequence;
    host_binary_motion.duration_ms =
        first_apply_tick - host_binary_motion.start_tick;
    host_binary_motion.last_control_tick =
        host_binary_motion.start_tick;
    host_binary_motion.verifying = 0U;
    host_binary_motion.active = 1U;
    Servo_MotionSafetyBegin(
        (uint8_t)((1U << SINGLE_ARM_JOINT_COUNT) - 1U)
    );

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
        !actuator_safety_accepts_setpoint(
            &host_binary_safety))
    {
        host_binary_motion.active = 0U;
        Servo_MotionSafetyEnd();
        Host_SendBinarySetpointStatus(
            host_binary_motion.request_sequence,
            8U,
            1U,
            host_binary_motion.start_tick +
                host_binary_motion.duration_ms,
            0U
        );
        return;
    }

    now = HAL_GetTick();
    if (host_binary_motion.verifying != 0U)
    {
        uint16_t final_positions[6] = {0U};
        uint32_t verify_elapsed = now -
            host_binary_motion.last_control_tick;

        if (verify_elapsed < 100U)
        {
            return;
        }

        if (Servo_ReadAllPositions(final_positions) != HAL_OK)
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

        uint8_t maximum_error = 0U;
        for (uint8_t joint = 0U;
             joint < servo_joint_count;
             joint++)
        {
            int32_t error = Servo_PositionError(
                final_positions[joint],
                host_binary_motion.target_positions[joint]
            );
            if (error < 0)
            {
                error = -error;
            }
            if (error > 255)
            {
                error = 255;
            }
            if ((uint8_t)error > maximum_error)
            {
                maximum_error = (uint8_t)error;
            }
        }

        host_binary_motion.active = 0U;
        Servo_MotionSafetyEnd();
        Host_SendBinarySetpointStatus(
            host_binary_motion.request_sequence,
            6U,
            1U,
            host_binary_motion.start_tick +
                host_binary_motion.duration_ms,
            maximum_error
        );
        return;
    }

    if ((uint32_t)(now -
            host_binary_motion.last_control_tick) <
        control_period_ms)
    {
        return;
    }

    elapsed = now - host_binary_motion.start_tick;
    if (elapsed > host_binary_motion.duration_ms)
    {
        elapsed = host_binary_motion.duration_ms;
    }

    for (uint8_t joint = 0U;
         joint < servo_joint_count;
         joint++)
    {
        if (elapsed >= host_binary_motion.duration_ms)
        {
            setpoints[joint] =
                host_binary_motion.target_positions[joint];
        }
        else
        {
            int32_t signed_delta =
                (int32_t)host_binary_motion.target_positions[joint] -
                (int32_t)host_binary_motion.start_positions[joint];
            int64_t elapsed_squared =
                (int64_t)elapsed * elapsed;
            int64_t smooth_numerator =
                (3LL * elapsed_squared *
                    host_binary_motion.duration_ms) -
                (2LL * elapsed_squared * elapsed);
            int64_t denominator =
                (int64_t)host_binary_motion.duration_ms *
                host_binary_motion.duration_ms *
                host_binary_motion.duration_ms;
            int32_t raw_position =
                (int32_t)host_binary_motion.start_positions[joint] +
                (int32_t)(
                    ((int64_t)signed_delta * smooth_numerator) /
                    denominator
                );

            if ((raw_position < 0) ||
                (raw_position > 4095))
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
            host_binary_motion.start_tick +
                host_binary_motion.duration_ms,
            0U
        );
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
            host_binary_motion.start_tick +
                host_binary_motion.duration_ms,
            diagnostics->servo_id
        );
        return;
    }

    host_binary_motion.last_control_tick = now;
    if (elapsed >= host_binary_motion.duration_ms)
    {
        host_binary_motion.verifying = 1U;
        host_binary_motion.last_control_tick = now;
    }
}

static void Host_ValidateBinarySetpointBatch(
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
        (host_binary_motion.active != 0U))
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
            Host_ValidateBinarySetpointBatch(request);
            break;

        case ACTUATOR_MSG_SAFE_STOP:
            if (request->payload_length == 0U)
            {
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
                    actuator_safety_request_disable(
                        &host_binary_safety
                    );

                /*
                 * The safety state and physical STS3215 torque state are one
                 * transaction.  Do not report success until all six Torque
                 * Enable registers have been written and read back as zero.
                 * Mark the trajectory configuration stale so the next ARM
                 * request must explicitly configure and re-enable servos.
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
    memset(&host_binary_motion, 0, sizeof(host_binary_motion));
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
        actuator_stream_parser_init(&host_binary_parser);
        host_binary_rejected_frame_count++;
    }
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
}
