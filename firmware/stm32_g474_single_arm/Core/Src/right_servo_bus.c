#include "right_servo_bus.h"

#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
#include "servo_rx_window.h"
#endif

#if HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD
#include "bimanual_operational_limits.h"
#endif

#include <string.h>

#define RIGHT_SERVO_READ_ADDRESS UINT8_C(56)
#define RIGHT_SERVO_READ_LENGTH UINT8_C(2)
#define RIGHT_SERVO_TORQUE_ENABLE_ADDRESS UINT8_C(40)
#define RIGHT_SERVO_GOAL_POSITION_ADDRESS UINT8_C(42)
#define RIGHT_SERVO_REQUEST_SIZE UINT8_C(8)
#define RIGHT_SERVO_TX_TIMEOUT_MS UINT32_C(5)
#define RIGHT_SERVO_RX_TIMEOUT_MS UINT32_C(20)
#define RIGHT_SERVO_WRITE_SETTLE_MS UINT32_C(4)
#define RIGHT_SERVO_CONFIGURATION_ALL_BLOCKS_MASK UINT8_C(0x1F)

static uint16_t RightServo_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static UART_HandleTypeDef *right_servo_uart = NULL;
static RightServoDiscoverySnapshot right_servo_snapshot = {0};


#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
#define RIGHT_SERVO_TELEMETRY_RING_CAPACITY UINT16_C(256)
#define RIGHT_SERVO_TELEMETRY_TIMEOUT_MS UINT32_C(4)

typedef enum
{
    RIGHT_SERVO_TELEMETRY_IDLE = 0,
    RIGHT_SERVO_TELEMETRY_TX_PENDING = 1,
    RIGHT_SERVO_TELEMETRY_WAIT_REPLY = 2
} RightServoTelemetryState;

typedef struct
{
    RightServoTelemetryState state;
    uint8_t enabled;
    uint8_t joint_index;
    uint8_t servo_id;
    uint8_t tx_completed;
    uint32_t started_at_ms;
    uint8_t request[RIGHT_SERVO_REQUEST_SIZE];
    ServoRxWindow window;
} RightServoTelemetry;

static volatile uint8_t right_servo_rx_ring[
    RIGHT_SERVO_TELEMETRY_RING_CAPACITY] __attribute__((aligned(4))) = {0U};
static volatile uint32_t right_servo_rx_wrap_count = 0U;
static volatile uint32_t right_servo_uart_async_errors = 0U;
static volatile uint32_t right_servo_dma_async_error = 0U;
static RightServoTelemetry right_servo_telemetry = {0};
static RightServoInMotionTelemetrySnapshot
    right_servo_telemetry_snapshot = {0};

static uint8_t RightServo_TelemetryDmaActive(void)
{
    return ((right_servo_uart != NULL) &&
            (right_servo_uart->hdmarx != NULL) &&
            (right_servo_uart->RxState == HAL_UART_STATE_BUSY_RX) &&
            HAL_IS_BIT_SET(right_servo_uart->Instance->CR3, USART_CR3_DMAR) &&
            HAL_IS_BIT_SET(
                right_servo_uart->hdmarx->Instance->CCR, DMA_CCR_EN)) ? 1U : 0U;
}

static uint32_t RightServo_TelemetryProducerAbsolute(void)
{
    uint32_t interrupt_mask;
    uint32_t wraps;
    uint16_t remaining;
    uint32_t pending_tc;
    uint32_t absolute;

    if (RightServo_TelemetryDmaActive() == 0U)
    {
        return 0U;
    }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    wraps = right_servo_rx_wrap_count;
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(right_servo_uart->hdmarx);
    pending_tc = __HAL_DMA_GET_FLAG(
        right_servo_uart->hdmarx,
        __HAL_DMA_GET_TC_FLAG_INDEX(right_servo_uart->hdmarx));
    if ((pending_tc != 0U) && (remaining != 0U))
    {
        wraps++;
    }
    absolute = (wraps * RIGHT_SERVO_TELEMETRY_RING_CAPACITY) +
        (RIGHT_SERVO_TELEMETRY_RING_CAPACITY - remaining);
    __DMB();
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }
    return absolute;
}

static HAL_StatusTypeDef RightServo_TelemetryFail(HAL_StatusTypeDef status)
{
    right_servo_telemetry_snapshot.failed_samples++;
    right_servo_telemetry.state = RIGHT_SERVO_TELEMETRY_IDLE;
    return status;
}
#endif

static uint8_t RightServo_Checksum(const uint8_t *packet, uint8_t last_index)
{
    uint8_t sum = 0U;
    for (uint8_t index = 2U; index <= last_index; index++)
    {
        sum = (uint8_t)(sum + packet[index]);
    }
    return (uint8_t)(~sum);
}

static void RightServo_ClearReceiveState(void)
{
    if (right_servo_uart == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_FLAG(
        right_servo_uart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
        UART_CLEAR_FEF | UART_CLEAR_RTOF
    );
    __HAL_UART_SEND_REQ(right_servo_uart, UART_RXDATA_FLUSH_REQUEST);
    right_servo_uart->ErrorCode = HAL_UART_ERROR_NONE;
}

static RightServoReadStatus RightServo_ReadData(
    uint8_t servo_id, uint8_t address, uint8_t length, uint8_t *data)
{
    uint8_t request[RIGHT_SERVO_REQUEST_SIZE] = {
        0xFFU, 0xFFU, servo_id, 0x04U, 0x02U, address, length, 0U};
    uint8_t reply[22] = {0U};
    uint8_t reply_size = (uint8_t)(length + 6U);
    request[7] = RightServo_Checksum(request, 6U);

    if ((right_servo_uart == NULL) || (data == NULL) ||
        (servo_id == 0U) || (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT) ||
        (length == 0U) || (length > 16U))
    {
        return RIGHT_SERVO_READ_UNAVAILABLE;
    }
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    if (right_servo_telemetry.enabled != 0U)
    {
        return RIGHT_SERVO_READ_UNAVAILABLE;
    }
#endif

    RightServo_ClearReceiveState();
    if (HAL_UART_Transmit(
            right_servo_uart,
            request,
            sizeof(request),
            RIGHT_SERVO_TX_TIMEOUT_MS
        ) != HAL_OK)
    {
        RightServo_ClearReceiveState();
        return RIGHT_SERVO_READ_TX;
    }
    if (HAL_UART_Receive(
            right_servo_uart,
            reply,
            reply_size,
            RIGHT_SERVO_RX_TIMEOUT_MS
        ) != HAL_OK)
    {
        RightServo_ClearReceiveState();
        return RIGHT_SERVO_READ_RX_TIMEOUT;
    }
    RightServo_ClearReceiveState();

    if ((reply[0] != 0xFFU) || (reply[1] != 0xFFU))
    {
        return RIGHT_SERVO_READ_HEADER;
    }
    if (reply[2] != servo_id)
    {
        return RIGHT_SERVO_READ_ID;
    }
    if (reply[3] != (uint8_t)(length + 2U))
    {
        return RIGHT_SERVO_READ_LENGTH;
    }
    if (reply[4] != 0U)
    {
        return RIGHT_SERVO_READ_STATUS;
    }
    if (reply[reply_size - 1U] !=
        RightServo_Checksum(reply, (uint8_t)(reply_size - 2U)))
    {
        return RIGHT_SERVO_READ_CHECKSUM;
    }

    memcpy(data, &reply[5], length);
    return RIGHT_SERVO_READ_OK;
}

static RightServoReadStatus RightServo_ReadPosition(
    uint8_t servo_id, uint16_t *position)
{
    uint8_t data[2] = {0U};
    RightServoReadStatus status = RightServo_ReadData(
        servo_id, RIGHT_SERVO_READ_ADDRESS, RIGHT_SERVO_READ_LENGTH, data);
    if ((status == RIGHT_SERVO_READ_OK) && (position != NULL))
    {
        *position = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
    }
    return status;
}

static HAL_StatusTypeDef RightServo_WriteData(
    uint8_t servo_id, uint8_t address, const uint8_t *data, uint8_t length)
{
    uint8_t packet[23] = {0U};
    uint8_t packet_size;

    if ((right_servo_uart == NULL) || (data == NULL) ||
        (servo_id == 0U) || (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT) ||
        (length == 0U) || (length > 16U))
    {
        return HAL_ERROR;
    }
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    if (right_servo_telemetry.enabled != 0U)
    {
        return HAL_BUSY;
    }
#endif
    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = servo_id;
    packet[3] = (uint8_t)(length + 3U);
    packet[4] = 0x03U;
    packet[5] = address;
    memcpy(&packet[6], data, length);
    packet_size = (uint8_t)(length + 7U);
    packet[packet_size - 1U] = RightServo_Checksum(
        packet, (uint8_t)(packet_size - 2U));

    RightServo_ClearReceiveState();
    if (HAL_UART_Transmit(right_servo_uart, packet, packet_size,
                          RIGHT_SERVO_TX_TIMEOUT_MS) != HAL_OK)
    {
        RightServo_ClearReceiveState();
        return HAL_ERROR;
    }
    HAL_Delay(RIGHT_SERVO_WRITE_SETTLE_MS);
    RightServo_ClearReceiveState();
    return HAL_OK;
}

void RightServoBus_Init(UART_HandleTypeDef *uart)
{
    right_servo_uart = uart;
    memset(&right_servo_snapshot, 0, sizeof(right_servo_snapshot));
#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
    memset(&right_servo_telemetry, 0, sizeof(right_servo_telemetry));
    memset(&right_servo_telemetry_snapshot, 0,
           sizeof(right_servo_telemetry_snapshot));
    memset((void *)right_servo_rx_ring, 0, sizeof(right_servo_rx_ring));
    right_servo_rx_wrap_count = 0U;
    right_servo_uart_async_errors = 0U;
    right_servo_dma_async_error = 0U;
#endif
    RightServo_ClearReceiveState();
}

const RightServoDiscoverySnapshot *RightServoBus_Discover(void)
{
    memset(&right_servo_snapshot, 0, sizeof(right_servo_snapshot));

    for (uint8_t index = 0U; index < RIGHT_SERVO_BUS_JOINT_COUNT; index++)
    {
        const uint8_t servo_id = (uint8_t)(index + 1U);
        uint16_t position = 0U;
        RightServoReadStatus status = RightServo_ReadPosition(
            servo_id,
            &position
        );
        right_servo_snapshot.transaction_count++;
        right_servo_snapshot.statuses[index] = (uint8_t)status;
        if (status == RIGHT_SERVO_READ_OK)
        {
            right_servo_snapshot.present_mask |= (uint8_t)(1U << index);
            right_servo_snapshot.positions[index] = position;
        }
        else
        {
            right_servo_snapshot.failure_count++;
        }
    }

    return &right_servo_snapshot;
}

RightServoJogSnapshot RightServoBus_JogOnce(uint8_t servo_id, int8_t delta_raw)
{
    RightServoJogSnapshot snapshot = {0};
    uint8_t torque[1] = {0U};
    uint8_t goal[2] = {0U};
#if !HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD
    int32_t target;
#endif
    RightServoReadStatus read_status;

    snapshot.status = RIGHT_SERVO_JOG_INVALID_REQUEST;
    snapshot.servo_id = servo_id;
    snapshot.delta_raw = delta_raw;
    if ((right_servo_uart == NULL) || (servo_id == 0U) ||
        (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT) ||
        ((delta_raw > -RIGHT_SERVO_JOG_MINIMUM_ABSOLUTE_DELTA_RAW) &&
         (delta_raw < RIGHT_SERVO_JOG_MINIMUM_ABSOLUTE_DELTA_RAW)) ||
        (delta_raw < -RIGHT_SERVO_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW) ||
        (delta_raw > RIGHT_SERVO_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW))
    {
        return snapshot;
    }

    read_status = RightServo_ReadData(servo_id,
                                      RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
                                      sizeof(torque), torque);
    if (read_status != RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_JOG_READ_TORQUE_FAILED;
        return snapshot;
    }
    snapshot.torque_enabled = torque[0];
    if (torque[0] != 1U)
    {
        snapshot.status = RIGHT_SERVO_JOG_TORQUE_DISABLED;
        return snapshot;
    }
    read_status = RightServo_ReadPosition(servo_id, &snapshot.start_position);
    if (read_status != RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_JOG_READ_POSITION_FAILED;
        return snapshot;
    }
#if HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD
    if (!BimanualOperationalLimits_StepModuloRaw(
            BIMANUAL_ARM_RIGHT,
            (uint8_t)(servo_id - 1U),
            snapshot.start_position,
            delta_raw,
            &snapshot.target_position))
    {
        snapshot.status = RIGHT_SERVO_JOG_TARGET_OUT_OF_RANGE;
        return snapshot;
    }
#else
    target = (int32_t)snapshot.start_position + (int32_t)delta_raw;
    if ((target < 0) || (target > 4095))
    {
        snapshot.status = RIGHT_SERVO_JOG_TARGET_OUT_OF_RANGE;
        return snapshot;
    }
    snapshot.target_position = (uint16_t)target;
#endif
    goal[0] = (uint8_t)(snapshot.target_position & 0xFFU);
    goal[1] = (uint8_t)(snapshot.target_position >> 8U);
    if (RightServo_WriteData(servo_id, RIGHT_SERVO_GOAL_POSITION_ADDRESS,
                             goal, sizeof(goal)) != HAL_OK)
    {
        snapshot.status = RIGHT_SERVO_JOG_WRITE_FAILED;
        return snapshot;
    }
    read_status = RightServo_ReadPosition(servo_id, &snapshot.observed_position);
    snapshot.status = (read_status == RIGHT_SERVO_READ_OK) ?
        RIGHT_SERVO_JOG_OK : RIGHT_SERVO_JOG_POST_READ_FAILED;
    return snapshot;
}

RightServoTorqueEnableSnapshot RightServoBus_EnableTorqueAtPresentPositionOnce(
    uint8_t servo_id
)
{
    RightServoTorqueEnableSnapshot snapshot = {0};
    uint8_t torque[1] = {0U};
    uint8_t goal[2] = {0U};
    const uint8_t torque_on[1] = {1U};
    RightServoReadStatus read_status;

    snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_INVALID_REQUEST;
    snapshot.servo_id = servo_id;
    if ((right_servo_uart == NULL) || (servo_id == 0U) ||
        (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT))
    {
        return snapshot;
    }

    read_status = RightServo_ReadData(
        servo_id,
        RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
        sizeof(torque),
        torque
    );
    if (read_status != RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_READ_TORQUE_FAILED;
        return snapshot;
    }
    snapshot.torque_enabled = torque[0];
    if (torque[0] == 1U)
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ALREADY_ENABLED;
        return snapshot;
    }

    read_status = RightServo_ReadPosition(servo_id, &snapshot.present_position);
    if (read_status != RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_READ_POSITION_FAILED;
        return snapshot;
    }
#if HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD
    {
        int32_t present_unwrapped_raw = 0;
        if (!BimanualOperationalLimits_UnwrapModuloRaw(
                BIMANUAL_ARM_RIGHT,
                (uint8_t)(servo_id - 1U),
                snapshot.present_position,
                &present_unwrapped_raw))
        {
            snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_POSITION_OUT_OF_RANGE;
            return snapshot;
        }
    }
#endif
    snapshot.held_goal_position = snapshot.present_position;
    goal[0] = (uint8_t)(snapshot.held_goal_position & 0xFFU);
    goal[1] = (uint8_t)(snapshot.held_goal_position >> 8U);
    if (RightServo_WriteData(
            servo_id,
            RIGHT_SERVO_GOAL_POSITION_ADDRESS,
            goal,
            sizeof(goal)
        ) != HAL_OK)
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_HOLD_WRITE_FAILED;
        return snapshot;
    }
    if (RightServo_WriteData(
            servo_id,
            RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
            torque_on,
            sizeof(torque_on)
        ) != HAL_OK)
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_WRITE_FAILED;
        return snapshot;
    }
    read_status = RightServo_ReadData(
        servo_id,
        RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
        sizeof(torque),
        torque
    );
    snapshot.torque_enabled = torque[0];
    if ((read_status != RIGHT_SERVO_READ_OK) || (torque[0] != 1U))
    {
        snapshot.status = RIGHT_SERVO_TORQUE_ENABLE_READBACK_FAILED;
        return snapshot;
    }
    read_status = RightServo_ReadPosition(servo_id, &snapshot.observed_position);
    snapshot.status = (read_status == RIGHT_SERVO_READ_OK) ?
        RIGHT_SERVO_TORQUE_ENABLE_OK :
        RIGHT_SERVO_TORQUE_ENABLE_READBACK_FAILED;
    return snapshot;
}

RightServoConfigurationSnapshot RightServoBus_ReadConfiguration(
    uint8_t servo_id
)
{
    RightServoConfigurationSnapshot snapshot = {0};
    uint8_t identity[5] = {0U};
    uint8_t limits_and_gains[14] = {0U};
    uint8_t protection[4] = {0U};
    uint8_t command_state[10] = {0U};
    uint8_t feedback[15] = {0U};
    RightServoReadStatus read_status;

    snapshot.status = 1U;
    snapshot.servo_id = servo_id;
    snapshot.read_status = (uint8_t)RIGHT_SERVO_READ_UNAVAILABLE;
    snapshot.sample_time_ms = HAL_GetTick();
    if ((right_servo_uart == NULL) || (servo_id == 0U) ||
        (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT))
    {
        return snapshot;
    }

#define RIGHT_SERVO_READ_CONFIGURATION_BLOCK(address_, buffer_, bit_)       \
    do                                                                       \
    {                                                                        \
        read_status = RightServo_ReadData(                                   \
            servo_id, (address_), sizeof(buffer_), (buffer_)                 \
        );                                                                   \
        if (read_status == RIGHT_SERVO_READ_OK)                              \
        {                                                                    \
            snapshot.successful_block_mask |= (bit_);                        \
        }                                                                    \
        else if (snapshot.read_status == RIGHT_SERVO_READ_UNAVAILABLE)       \
        {                                                                    \
            snapshot.read_status = (uint8_t)read_status;                     \
        }                                                                    \
    } while (0)

    RIGHT_SERVO_READ_CONFIGURATION_BLOCK(UINT8_C(0), identity, UINT8_C(0x01));
    RIGHT_SERVO_READ_CONFIGURATION_BLOCK(
        UINT8_C(16), limits_and_gains, UINT8_C(0x02));
    RIGHT_SERVO_READ_CONFIGURATION_BLOCK(
        UINT8_C(33), protection, UINT8_C(0x04));
    RIGHT_SERVO_READ_CONFIGURATION_BLOCK(
        UINT8_C(40), command_state, UINT8_C(0x08));
    RIGHT_SERVO_READ_CONFIGURATION_BLOCK(
        UINT8_C(56), feedback, UINT8_C(0x10));

#undef RIGHT_SERVO_READ_CONFIGURATION_BLOCK

    snapshot.firmware_major_version = identity[0];
    snapshot.firmware_minor_version = identity[1];
    snapshot.model_number = RightServo_ReadU16Le(&identity[3]);
    snapshot.maximum_torque_limit_raw =
        RightServo_ReadU16Le(&limits_and_gains[0]);
    snapshot.p_gain = limits_and_gains[5];
    snapshot.d_gain = limits_and_gains[6];
    snapshot.i_gain = limits_and_gains[7];
    snapshot.minimum_startup_force_raw =
        RightServo_ReadU16Le(&limits_and_gains[8]);
    snapshot.cw_dead_zone_raw = limits_and_gains[10];
    snapshot.ccw_dead_zone_raw = limits_and_gains[11];
    snapshot.protection_current_raw =
        RightServo_ReadU16Le(&limits_and_gains[12]);
    snapshot.operating_mode = protection[0];
    snapshot.protective_torque_raw = protection[1];
    snapshot.protection_time_raw = protection[2];
    snapshot.overload_torque_raw = protection[3];
    snapshot.torque_enabled = command_state[0];
    snapshot.goal_position_raw = RightServo_ReadU16Le(&command_state[2]);
    snapshot.runtime_torque_limit_raw =
        RightServo_ReadU16Le(&command_state[8]);
    snapshot.position_raw = RightServo_ReadU16Le(&feedback[0]);
    snapshot.speed_raw = RightServo_ReadU16Le(&feedback[2]);
    snapshot.load_raw = RightServo_ReadU16Le(&feedback[4]);
    snapshot.voltage_raw = feedback[6];
    snapshot.temperature_c = feedback[7];
    snapshot.current_raw = RightServo_ReadU16Le(&feedback[13]);

    if (snapshot.successful_block_mask ==
        RIGHT_SERVO_CONFIGURATION_ALL_BLOCKS_MASK)
    {
        snapshot.status = 0U;
        snapshot.read_status = (uint8_t)RIGHT_SERVO_READ_OK;
    }
    else
    {
        snapshot.status = 2U;
    }
    return snapshot;
}

RightServoConfigureSnapshot RightServoBus_ConfigureAtPresentPositionOnce(
    uint8_t servo_id,
    uint8_t p_gain,
    uint8_t d_gain,
    uint16_t goal_speed,
    uint16_t torque_limit
)
{
    RightServoConfigureSnapshot snapshot = {0};
    uint8_t torque[1] = {0U};
    const uint8_t torque_off[1] = {0U};
    const uint8_t lock_volatile[1] = {1U};
    const uint8_t position_mode[1] = {0U};
    uint8_t pid_data[3] = {p_gain, d_gain, 0U};
    uint8_t speed_and_torque[4] = {
        (uint8_t)(goal_speed & 0xFFU),
        (uint8_t)(goal_speed >> 8U),
        (uint8_t)(torque_limit & 0xFFU),
        (uint8_t)(torque_limit >> 8U)
    };
    uint8_t pid_readback[3] = {0U};
    uint8_t mode_readback[1] = {0U};
    uint8_t runtime_readback[10] = {0U};
    uint8_t configuration_write_failed = 0U;

    snapshot.status = RIGHT_SERVO_CONFIGURE_INVALID_REQUEST;
    snapshot.servo_id = servo_id;
    if ((right_servo_uart == NULL) || (servo_id == 0U) ||
        (servo_id > RIGHT_SERVO_BUS_JOINT_COUNT))
    {
        return snapshot;
    }
    if (RightServo_ReadData(servo_id, UINT8_C(40), sizeof(torque), torque) !=
        RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_READ_TORQUE_FAILED;
        return snapshot;
    }
    snapshot.torque_enabled = torque[0];
    if (snapshot.torque_enabled != 0U)
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_TORQUE_NOT_DISABLED;
        return snapshot;
    }
    if (RightServo_ReadPosition(servo_id, &snapshot.present_position) !=
        RIGHT_SERVO_READ_OK)
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_READ_POSITION_FAILED;
        return snapshot;
    }

    if ((RightServo_WriteData(servo_id, UINT8_C(55), lock_volatile,
                              sizeof(lock_volatile)) != HAL_OK) ||
        (RightServo_WriteData(servo_id, UINT8_C(33), position_mode,
                              sizeof(position_mode)) != HAL_OK) ||
        (RightServo_WriteData(servo_id, UINT8_C(21), pid_data,
                              sizeof(pid_data)) != HAL_OK) ||
        (RightServo_WriteData(servo_id, UINT8_C(46), speed_and_torque,
                              sizeof(speed_and_torque)) != HAL_OK))
    {
        configuration_write_failed = 1U;
    }

    /* STS3215 may re-enable torque as a side effect of a runtime block write.
     * Never write Goal_Position in this configuration-only primitive, and
     * force torque off before any success or failure response. */
    if (RightServo_WriteData(servo_id, UINT8_C(40), torque_off,
                             sizeof(torque_off)) != HAL_OK)
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_WRITE_FAILED;
        return snapshot;
    }

    if ((RightServo_ReadData(servo_id, UINT8_C(21), sizeof(pid_readback),
                             pid_readback) != RIGHT_SERVO_READ_OK) ||
        (RightServo_ReadData(servo_id, UINT8_C(33), sizeof(mode_readback),
                             mode_readback) != RIGHT_SERVO_READ_OK) ||
        (RightServo_ReadData(servo_id, UINT8_C(40), sizeof(runtime_readback),
                             runtime_readback) != RIGHT_SERVO_READ_OK))
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_READBACK_FAILED;
        return snapshot;
    }

    snapshot.torque_enabled = runtime_readback[0];
    snapshot.p_gain = pid_readback[0];
    snapshot.d_gain = pid_readback[1];
    snapshot.i_gain = pid_readback[2];
    snapshot.operating_mode = mode_readback[0];
    snapshot.goal_position = RightServo_ReadU16Le(&runtime_readback[2]);
    snapshot.goal_speed = RightServo_ReadU16Le(&runtime_readback[6]);
    snapshot.torque_limit = RightServo_ReadU16Le(&runtime_readback[8]);
    if (configuration_write_failed != 0U)
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_WRITE_FAILED;
        return snapshot;
    }
    if ((snapshot.torque_enabled != 0U) ||
        (snapshot.p_gain != p_gain) ||
        (snapshot.d_gain != d_gain) ||
        (snapshot.i_gain != 0U) ||
        (snapshot.operating_mode != 0U) ||
        (snapshot.goal_speed != goal_speed) ||
        (snapshot.torque_limit != torque_limit))
    {
        snapshot.status = RIGHT_SERVO_CONFIGURE_READBACK_FAILED;
        return snapshot;
    }

    snapshot.status = RIGHT_SERVO_CONFIGURE_OK;
    return snapshot;
}

RightServoDisableSnapshot RightServoBus_DisableTorqueAllVerified(void)
{
    const uint8_t torque_off[1] = {0U};
    RightServoDisableSnapshot snapshot = {
        .status = RIGHT_SERVO_DISABLE_OK,
        .joint_count = RIGHT_SERVO_BUS_JOINT_COUNT,
        .torque_enabled_mask = 0U,
        .failure_count = 0U,
    };

    if (right_servo_uart == NULL)
    {
        snapshot.status = RIGHT_SERVO_DISABLE_UNAVAILABLE;
        snapshot.failure_count = RIGHT_SERVO_BUS_JOINT_COUNT;
        return snapshot;
    }

    for (uint8_t servo_id = 1U;
         servo_id <= RIGHT_SERVO_BUS_JOINT_COUNT;
         servo_id++)
    {
        if (RightServo_WriteData(servo_id, RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
                                 torque_off, sizeof(torque_off)) != HAL_OK)
        {
            snapshot.status = RIGHT_SERVO_DISABLE_WRITE_FAILED;
            snapshot.failure_count++;
            continue;
        }

        uint8_t torque_readback[1] = {UINT8_C(0xFF)};
        if (RightServo_ReadData(
                servo_id,
                RIGHT_SERVO_TORQUE_ENABLE_ADDRESS,
                sizeof(torque_readback),
                torque_readback
            ) != RIGHT_SERVO_READ_OK)
        {
            snapshot.status = RIGHT_SERVO_DISABLE_READBACK_FAILED;
            snapshot.failure_count++;
            continue;
        }
        if (torque_readback[0] != 0U)
        {
            snapshot.status = RIGHT_SERVO_DISABLE_TORQUE_REMAINS_ENABLED;
            snapshot.torque_enabled_mask |= (uint8_t)(1U << (servo_id - 1U));
            snapshot.failure_count++;
        }
    }

    return snapshot;
}

HAL_StatusTypeDef RightServoBus_DisableTorqueAll(void)
{
    const RightServoDisableSnapshot snapshot =
        RightServoBus_DisableTorqueAllVerified();
    return (snapshot.status == RIGHT_SERVO_DISABLE_OK) ? HAL_OK : HAL_ERROR;
}


#if HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD
HAL_StatusTypeDef RightServoBus_InMotionTelemetryBegin(void)
{
    HAL_StatusTypeDef status;

    if ((right_servo_uart == NULL) || (right_servo_uart->hdmarx == NULL) ||
        (right_servo_telemetry.enabled != 0U))
    {
        return HAL_ERROR;
    }
    (void)HAL_UART_AbortTransmit(right_servo_uart);
    (void)HAL_UART_AbortReceive(right_servo_uart);
    RightServo_ClearReceiveState();
    memset((void *)right_servo_rx_ring, 0, sizeof(right_servo_rx_ring));
    memset(&right_servo_telemetry, 0, sizeof(right_servo_telemetry));
    memset(&right_servo_telemetry_snapshot, 0,
           sizeof(right_servo_telemetry_snapshot));
    right_servo_rx_wrap_count = 0U;
    right_servo_uart_async_errors = 0U;
    right_servo_dma_async_error = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        right_servo_uart,
        (uint8_t *)right_servo_rx_ring,
        RIGHT_SERVO_TELEMETRY_RING_CAPACITY);
    if (status != HAL_OK)
    {
        return status;
    }
    ATOMIC_CLEAR_BIT(
        right_servo_uart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RTOIE);
    ATOMIC_CLEAR_BIT(right_servo_uart->Instance->CR3, USART_CR3_EIE);
    right_servo_telemetry.enabled = 1U;
    return HAL_OK;
}

void RightServoBus_InMotionTelemetryEnd(void)
{
    if (right_servo_uart != NULL)
    {
        (void)HAL_UART_AbortTransmit(right_servo_uart);
        (void)HAL_UART_AbortReceive(right_servo_uart);
        RightServo_ClearReceiveState();
    }
    memset(&right_servo_telemetry, 0, sizeof(right_servo_telemetry));
}

uint8_t RightServoBus_InMotionTelemetryPending(void)
{
    return (right_servo_telemetry.state != RIGHT_SERVO_TELEMETRY_IDLE) ?
        1U : 0U;
}

const RightServoInMotionTelemetrySnapshot *
RightServoBus_InMotionTelemetryGetSnapshot(void)
{
    return &right_servo_telemetry_snapshot;
}

HAL_StatusTypeDef RightServoBus_InMotionTelemetryStart(
    uint8_t joint_index,
    uint32_t started_at_ms)
{
    RightServoTelemetry *telemetry = &right_servo_telemetry;
    uint32_t transaction_start;

    if ((telemetry->enabled == 0U) ||
        (telemetry->state != RIGHT_SERVO_TELEMETRY_IDLE) ||
        (joint_index >= RIGHT_SERVO_BUS_JOINT_COUNT) ||
        (RightServo_TelemetryDmaActive() == 0U))
    {
        return HAL_BUSY;
    }
    transaction_start = RightServo_TelemetryProducerAbsolute();
    telemetry->joint_index = joint_index;
    telemetry->servo_id = (uint8_t)(joint_index + 1U);
    telemetry->started_at_ms = started_at_ms;
    telemetry->tx_completed = 0U;
    telemetry->request[0] = 0xFFU;
    telemetry->request[1] = 0xFFU;
    telemetry->request[2] = telemetry->servo_id;
    telemetry->request[3] = 0x04U;
    telemetry->request[4] = 0x02U;
    telemetry->request[5] = RIGHT_SERVO_READ_ADDRESS;
    telemetry->request[6] = RIGHT_SERVO_READ_LENGTH;
    telemetry->request[7] = RightServo_Checksum(telemetry->request, 6U);
    ServoRxWindow_Init(
        &telemetry->window, telemetry->servo_id, 2U, transaction_start);
    if (HAL_UART_Transmit_IT(
            right_servo_uart,
            telemetry->request,
            sizeof(telemetry->request)) != HAL_OK)
    {
        return RightServo_TelemetryFail(HAL_ERROR);
    }
    telemetry->state = RIGHT_SERVO_TELEMETRY_TX_PENDING;
    right_servo_telemetry_snapshot.requested_samples++;
    return HAL_OK;
}

void RightServoBus_InMotionTelemetryOnTxComplete(UART_HandleTypeDef *uart)
{
    if ((uart == right_servo_uart) &&
        (right_servo_telemetry.state == RIGHT_SERVO_TELEMETRY_TX_PENDING))
    {
        right_servo_telemetry.tx_completed = 1U;
    }
}

void RightServoBus_InMotionTelemetryOnRxEvent(
    UART_HandleTypeDef *uart,
    uint16_t received)
{
    (void)received;
    if ((uart == right_servo_uart) &&
        (HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_TC))
    {
        right_servo_rx_wrap_count++;
    }
}

void RightServoBus_InMotionTelemetryOnUartError(UART_HandleTypeDef *uart)
{
    if (uart != right_servo_uart)
    {
        return;
    }
    right_servo_uart_async_errors |= uart->ErrorCode;
    if (uart->hdmarx != NULL)
    {
        right_servo_dma_async_error |= uart->hdmarx->ErrorCode;
    }
}

HAL_StatusTypeDef RightServoBus_InMotionTelemetryPoll(
    uint32_t now_ms,
    const uint16_t commanded_positions[RIGHT_SERVO_BUS_JOINT_COUNT])
{
    RightServoTelemetry *telemetry = &right_servo_telemetry;

    if ((telemetry->enabled == 0U) ||
        (telemetry->state == RIGHT_SERVO_TELEMETRY_IDLE))
    {
        return HAL_OK;
    }
    if ((commanded_positions == NULL) ||
        (RightServo_TelemetryDmaActive() == 0U) ||
        (right_servo_uart_async_errors != HAL_UART_ERROR_NONE) ||
        (right_servo_dma_async_error != HAL_DMA_ERROR_NONE) ||
        ((right_servo_uart->hdmarx != NULL) &&
         (right_servo_uart->hdmarx->ErrorCode != HAL_DMA_ERROR_NONE)))
    {
        return RightServo_TelemetryFail(HAL_ERROR);
    }
    if ((telemetry->state == RIGHT_SERVO_TELEMETRY_TX_PENDING) &&
        (telemetry->tx_completed != 0U))
    {
        telemetry->state = RIGHT_SERVO_TELEMETRY_WAIT_REPLY;
    }
    if (telemetry->state == RIGHT_SERVO_TELEMETRY_WAIT_REPLY)
    {
        uint8_t data[2] = {0U};
        ServoRxWindowResult result = ServoRxWindow_Consume(
            &telemetry->window,
            right_servo_rx_ring,
            RIGHT_SERVO_TELEMETRY_RING_CAPACITY,
            RightServo_TelemetryProducerAbsolute(),
            data,
            sizeof(data));
        if (result == SERVO_RX_WINDOW_FRAME_READY)
        {
            const uint16_t actual = RightServo_ReadU16Le(data);
            const uint16_t commanded =
                commanded_positions[telemetry->joint_index];
            const uint16_t error = (actual > commanded) ?
                (uint16_t)(actual - commanded) :
                (uint16_t)(commanded - actual);
            const uint32_t latency = now_ms - telemetry->started_at_ms;
            if (error > right_servo_telemetry_snapshot.maximum_error_raw[
                    telemetry->joint_index])
            {
                right_servo_telemetry_snapshot.maximum_error_raw[
                    telemetry->joint_index] = error;
            }
            if (latency >
                right_servo_telemetry_snapshot.maximum_reply_latency_ms)
            {
                right_servo_telemetry_snapshot.maximum_reply_latency_ms =
                    latency;
            }
            right_servo_telemetry_snapshot.last_joint_index =
                telemetry->joint_index;
            right_servo_telemetry_snapshot.last_position_raw = actual;
            right_servo_telemetry_snapshot.last_commanded_raw = commanded;
            right_servo_telemetry_snapshot.completed_samples++;
            telemetry->state = RIGHT_SERVO_TELEMETRY_IDLE;
            return HAL_OK;
        }
        if ((result == SERVO_RX_WINDOW_STATUS_ERROR) ||
            (result == SERVO_RX_WINDOW_OVERFLOW))
        {
            return RightServo_TelemetryFail(HAL_ERROR);
        }
    }
    if (ServoRxWindow_DeadlineExpired(
            telemetry->started_at_ms,
            now_ms,
            RIGHT_SERVO_TELEMETRY_TIMEOUT_MS) != 0U)
    {
        return RightServo_TelemetryFail(HAL_TIMEOUT);
    }
    return HAL_BUSY;
}
#endif
