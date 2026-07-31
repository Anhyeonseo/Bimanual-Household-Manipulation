#include "servo_bus.h"
#include "servo_response_parser.h"

#include <stddef.h>
#include <string.h>

#define SERVO_BUS_READ_TIMEOUT_MS UINT32_C(50)
#define SERVO_BUS_MAX_RX_BYTES UINT16_C(64)
#define SERVO_BUS_RECOVERY_QUIET_MS UINT32_C(2)
#define SERVO_BUS_WRITE_REPLY_SETTLE_MS UINT32_C(2)

static UART_HandleTypeDef *servo_uart_handle = NULL;
static ServoStopRequestedFn servo_stop_requested = NULL;
static ServoReadFailureFn servo_read_failure = NULL;
static uint16_t servo_bus_recovery_count = 0U;
static ServoBusDiagnostics servo_bus_diagnostics = {
    SERVO_BUS_FAILURE_NONE, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};

static uint8_t servo_motion_safety_mask = 0U;
static uint32_t servo_motion_safety_last_sample_ms = 0U;
static uint8_t servo_motion_safety_next_joint = 0U;
static uint8_t servo_load_limit_counts[SINGLE_ARM_JOINT_COUNT] = {0U};
static uint8_t servo_current_limit_counts[SINGLE_ARM_JOINT_COUNT] = {0U};
static ServoMotionSafetyDiagnostics servo_motion_safety_diagnostics = {
    SERVO_MOTION_SAFETY_NONE, 0U, 0U, 0U, 0U, 0U
};

const ServoJointConfig servo_joints[SINGLE_ARM_JOINT_COUNT] = {
    {1U, "BASE",        1U, 2048U, 2048U, 2610U, 16U,  1,  34U,  600U, 400U},
    {2U, "SHOULDER",    1U, 2048U, 1988U, 3766U, 32U,  1,  34U, 1200U,
        SERVO_SHOULDER_TORQUE_LIMIT_RAW},
    {3U, "ELBOW",       1U, 2048U, 627U, 2258U, 28U, -1,  34U, 1000U,
        SERVO_ELBOW_TORQUE_LIMIT_RAW},
    {4U, "WRIST_FLEX",  1U, 2048U, 1194U, 2048U, 16U, -1,  34U,  800U, 400U},
    {5U, "WRIST_ROLL",  1U, 2048U, 1874U, 2219U, 16U,  1,  34U,  500U, 250U},
    {6U, "GRIPPER",     1U, 2048U, 1866U, 2048U, 16U, -1,  34U,  800U, 150U}
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
    servo_bus_recovery_count = 0U;
    memset(&servo_bus_diagnostics, 0, sizeof(servo_bus_diagnostics));
    Servo_MotionSafetyEnd();
    memset(
        &servo_motion_safety_diagnostics,
        0,
        sizeof(servo_motion_safety_diagnostics)
    );
}

const ServoBusDiagnostics *ServoBus_GetDiagnostics(void)
{
    return &servo_bus_diagnostics;
}

static void ServoBus_ClearRxState(void)
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

static void ServoBus_BeginTransaction(uint8_t servo_id)
{
    memset(&servo_bus_diagnostics, 0, sizeof(servo_bus_diagnostics));
    servo_bus_diagnostics.servo_id = servo_id;
    servo_bus_diagnostics.recovery_count = servo_bus_recovery_count;
    ServoBus_ClearRxState();
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

    if (servo_uart_handle != NULL)
    {
        servo_bus_diagnostics.uart_error_code =
            servo_uart_handle->ErrorCode;
        servo_bus_diagnostics.uart_isr =
            servo_uart_handle->Instance->ISR;
    }
}

static HAL_StatusTypeDef ServoBus_Recover(void)
{
    if (servo_uart_handle == NULL)
    {
        servo_bus_diagnostics.reason = SERVO_BUS_FAILURE_RECOVERY;
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Abort(servo_uart_handle);
    if (servo_bus_recovery_count < UINT16_MAX)
    {
        servo_bus_recovery_count++;
    }
    servo_bus_diagnostics.recovery_count = servo_bus_recovery_count;

    ServoBus_ClearRxState();
    HAL_Delay(SERVO_BUS_RECOVERY_QUIET_MS);
    ServoBus_ClearRxState();

    if (status != HAL_OK)
    {
        servo_bus_diagnostics.reason = SERVO_BUS_FAILURE_RECOVERY;
        servo_bus_diagnostics.hal_status = (uint8_t)status;
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

    ServoBus_BeginTransaction(servo_id);

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        servo_uart_handle,
        request,
        sizeof(request),
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

    ServoResponseParser parser;
    ServoResponseParser_Init(&parser, servo_id, data_length);
    uint32_t start_tick = HAL_GetTick();
    HAL_StatusTypeDef receive_status = HAL_TIMEOUT;

    for (uint16_t received = 0U;
         received < SERVO_BUS_MAX_RX_BYTES;
         received++)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= SERVO_BUS_READ_TIMEOUT_MS)
        {
            break;
        }

        uint8_t byte = 0U;
        receive_status = HAL_UART_Receive(
            servo_uart_handle,
            &byte,
            1U,
            SERVO_BUS_READ_TIMEOUT_MS - elapsed
        );
        if (receive_status != HAL_OK)
        {
            uint32_t uart_errors = servo_uart_handle->ErrorCode;
            uint32_t uart_flags = servo_uart_handle->Instance->ISR &
                (UART_FLAG_PE |
                 UART_FLAG_FE |
                 UART_FLAG_NE |
                 UART_FLAG_ORE |
                 UART_FLAG_RTOF);
            ServoBusFailureReason failure_reason =
                SERVO_BUS_FAILURE_RX_TIMEOUT;
            if ((uart_errors != HAL_UART_ERROR_NONE) ||
                (uart_flags != 0U))
            {
                failure_reason = SERVO_BUS_FAILURE_UART;
            }
            else if (parser.last_reject != SERVO_RESPONSE_REJECT_NONE)
            {
                /*
                 * Preserve the parser's concrete rejection cause when a
                 * corrupt or stale frame is followed by silence. Reporting
                 * only the final byte timeout would hide the evidence needed
                 * to distinguish desynchronization from a disconnected bus.
                 */
                failure_reason = ServoBus_MapRejectReason(
                    parser.last_reject
                );
            }
            ServoBus_RecordFailure(
                failure_reason,
                servo_id,
                receive_status,
                0U,
                parser.discarded_bytes
            );
            break;
        }

        ServoResponseParseResult parse_result =
            ServoResponseParser_Push(&parser, byte);
        if (parse_result == SERVO_RESPONSE_FRAME_READY)
        {
            memcpy(data, ServoResponseParser_Data(&parser), data_length);
            servo_bus_diagnostics.discarded_bytes =
                parser.discarded_bytes;
            return HAL_OK;
        }
        if (parse_result == SERVO_RESPONSE_STATUS_ERROR)
        {
            ServoBus_RecordFailure(
                SERVO_BUS_FAILURE_STATUS,
                servo_id,
                HAL_ERROR,
                parser.servo_status,
                parser.discarded_bytes
            );
            (void)ServoBus_Recover();
            return HAL_ERROR;
        }
    }

    HAL_StatusTypeDef terminal_status =
        (receive_status == HAL_OK) ? HAL_ERROR : receive_status;
    if (servo_bus_diagnostics.reason == SERVO_BUS_FAILURE_NONE)
    {
        ServoBus_RecordFailure(
            ServoBus_MapRejectReason(parser.last_reject),
            servo_id,
            terminal_status,
            0U,
            parser.discarded_bytes
        );
    }
    (void)ServoBus_Recover();
    return terminal_status;
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

    ServoBus_BeginTransaction(servo_id);

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
    ServoBus_ClearRxState();
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
        32U,
        0U
    };

    uint8_t runtime_data[9] = {
        0U,
        (uint8_t)(*initial_position & 0xFFU),
        (uint8_t)((*initial_position >> 8) & 0xFFU),
        0U, 0U,
        65U, 0U,
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
        (pid_readback[1] != 32U) ||
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

    return HAL_UART_Transmit(
        servo_uart_handle,
        packet,
        packet_index,
        100U
    );
}

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
