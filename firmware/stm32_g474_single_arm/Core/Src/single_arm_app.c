#include "single_arm_app.h"

#include "binary_control.h"
#include "host_uart_rx.h"
#include "servo_bus.h"
#include "single_arm_config.h"

#include <stdio.h>
#include <stdlib.h>

static UART_HandleTypeDef *app_host_uart = NULL;
static uint8_t rx_byte = 0U;
static uint8_t system_ready = 0U;
static uint8_t move_already_done = 0U;
static uint8_t return_available = 0U;
static uint8_t synchronized_pose_active = 0U;
static uint8_t absolute_pose_active = 0U;
static uint16_t return_position = 0U;
static char status_line[192];
static uint8_t active_servo_id = 1U;
static const char *active_joint_name = "BASE";
static uint16_t active_test_delta = 34U;
static uint32_t active_duration_ms = 600U;
static uint16_t active_torque_limit = 400U;
static uint16_t active_home_position = 2048U;
static int8_t active_test_direction = 1;
#if ENABLE_SERVO_CENTERING_COMMAND
static uint8_t centering_done = 0U;
#endif

static uint8_t SingleArmApp_StopRequested(void)
{
    if (BinaryControl_StopIsLatched() != 0U)
    {
        return 1U;
    }

    if ((app_host_uart != NULL) &&
        (__HAL_UART_GET_FLAG(app_host_uart, UART_FLAG_RXNE) != RESET))
    {
        uint8_t byte = (uint8_t)(app_host_uart->Instance->RDR & 0xFFU);
        if ((byte == (uint8_t)'X') || (byte == (uint8_t)'x'))
        {
            BinaryControl_LatchStop();
            return 1U;
        }
    }

    return 0U;
}

static void SingleArmApp_ReportServoReadFailure(uint8_t servo_id)
{
    if (BinaryControl_IsBinaryMode() == 0U)
    {
        char read_fail_line[48];
        int line_length = snprintf(
            read_fail_line,
            sizeof(read_fail_line),
            "ALL_POS_READ_FAIL ID=%u RETRIES=3\r\n",
            (unsigned int)servo_id
        );

        if ((line_length > 0) &&
            (line_length < (int)sizeof(read_fail_line)))
        {
            (void)HAL_UART_Transmit(
                app_host_uart,
                (uint8_t *)read_fail_line,
                (uint16_t)line_length,
                100U
            );
        }
    }
}

static void SingleArmApp_ReportMotionSafety(void)
{
    const ServoMotionSafetyDiagnostics *diagnostics =
        Servo_MotionSafetyGetDiagnostics();

    if ((diagnostics == NULL) ||
        (diagnostics->reason == SERVO_MOTION_SAFETY_NONE))
    {
        return;
    }

    BinaryControl_LatchStop();

    if (BinaryControl_IsBinaryMode() != 0U)
    {
        return;
    }

    const char *reason = "READ";

    if (diagnostics->reason == SERVO_MOTION_SAFETY_LOAD_LIMIT)
    {
        reason = "LOAD";
    }
    else if (diagnostics->reason ==
        SERVO_MOTION_SAFETY_CURRENT_LIMIT)
    {
        reason = "CURRENT";
    }

    int line_length = snprintf(
        status_line,
        sizeof(status_line),
        "MOTION_SAFETY_ABORT REASON=%s ID=%u LOAD_RAW=%u CURRENT_RAW=%u MAX_LOAD_RAW=%u MAX_CURRENT_RAW=%u\r\n",
        reason,
        (unsigned int)diagnostics->servo_id,
        (unsigned int)diagnostics->last_load_raw,
        (unsigned int)diagnostics->last_current_raw,
        (unsigned int)diagnostics->maximum_load_raw,
        (unsigned int)diagnostics->maximum_current_raw
    );

    if ((line_length > 0) &&
        (line_length < (int)sizeof(status_line)))
    {
        (void)HAL_UART_Transmit(
            app_host_uart,
            (uint8_t *)status_line,
            (uint16_t)line_length,
            100U
        );
    }
}

static HAL_StatusTypeDef Host_ReadLineAfterCommand(
    char *buffer,
    uint16_t capacity,
    uint32_t timeout_ms
)
{
    if ((buffer == NULL) || (capacity < 2U))
    {
        return HAL_ERROR;
    }

    uint16_t received_length = 0U;
    buffer[0] = '\0';

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle(
        app_host_uart,
        (uint8_t *)buffer,
        (uint16_t)(capacity - 1U),
        &received_length,
        timeout_ms
    );

    if ((status != HAL_OK) || (received_length == 0U))
    {
        return status;
    }

    while ((received_length > 0U) &&
           ((buffer[received_length - 1U] == '\r') ||
            (buffer[received_length - 1U] == '\n')))
    {
        received_length--;
    }

    buffer[received_length] = '\0';
    return (received_length > 0U) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef Host_ParseAbsoluteJointCommand(
    char *line,
    uint16_t target_positions[6],
    uint32_t *duration_ms
)
{
    if ((line == NULL) ||
        (target_positions == NULL) ||
        (duration_ms == NULL))
    {
        return HAL_ERROR;
    }

    char *cursor = line;

    for (uint8_t field = 0U; field < 7U; field++)
    {
        while ((*cursor == ' ') || (*cursor == '\t'))
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            return HAL_ERROR;
        }

        char *end_pointer = NULL;
        unsigned long parsed = strtoul(
            cursor,
            &end_pointer,
            10
        );

        if (end_pointer == cursor)
        {
            return HAL_ERROR;
        }

        if (field < 6U)
        {
            if (parsed > 4095UL)
            {
                return HAL_ERROR;
            }

            target_positions[field] = (uint16_t)parsed;
        }
        else
        {
            if ((parsed < 200UL) ||
                (parsed > 2000UL) ||
                ((parsed % 20UL) != 0UL))
            {
                return HAL_ERROR;
            }

            *duration_ms = (uint32_t)parsed;
        }

        cursor = end_pointer;
    }

    while ((*cursor == ' ') || (*cursor == '\t'))
    {
        cursor++;
    }

    return (*cursor == '\0') ? HAL_OK : HAL_ERROR;
}


void SingleArmApp_Init(
    UART_HandleTypeDef *host_uart,
    UART_HandleTypeDef *servo_uart
)
{
    app_host_uart = host_uart;
    HostUartRx_Init(host_uart);
    ServoBus_Init(
        servo_uart,
        SingleArmApp_StopRequested,
        SingleArmApp_ReportServoReadFailure
    );
    BinaryControl_Init(host_uart);

    rx_byte = 0U;
    system_ready = 0U;
    move_already_done = 0U;
    return_available = 0U;
    synchronized_pose_active = 0U;
    absolute_pose_active = 0U;
    return_position = 0U;
    active_servo_id = 1U;
    active_joint_name = "BASE";
    active_test_delta = 34U;
    active_duration_ms = 600U;
    active_torque_limit = 400U;
    active_home_position = 2048U;
    active_test_direction = 1;
#if ENABLE_SERVO_CENTERING_COMMAND
    centering_done = 0U;
#endif

  static const uint8_t startup_message[] =
      "FW_HOME2048_BINARY_V2\r\n"
      "SAFE_IDLE_ASCII_OR_SEND_P_FOR_BINARY\r\n";

  (void)HAL_UART_Transmit(
      app_host_uart,
      (uint8_t *)startup_message,
      sizeof(startup_message) - 1U,
      100U
  );

  HAL_Delay(500U);

#if ENABLE_BOOT_ID1_AUTOCONFIG
  uint8_t position_data[2] = {0U};
  uint8_t torque_off[1] = {0U};
  uint8_t torque_on[1] = {1U};
  uint8_t lock_volatile[1] = {1U};
  uint8_t position_mode[1] = {0U};

  /* 주소 순서: P, D, I */
  uint8_t pid_data[3] = {
      16U,
      32U,
      0U
  };

  if (Servo_ReadData(
          1U,
          56U,
          sizeof(position_data),
          position_data
      ) == HAL_OK)
  {
      uint16_t initial_position = (uint16_t)(
          (uint16_t)position_data[0] |
          ((uint16_t)position_data[1] << 8)
      );

      /*
       * 주소 41~49:
       * Acceleration, Goal Position, Goal Time,
       * Goal Velocity, Torque Limit
       */
      uint8_t runtime_data[9] = {
          0U,				/* STM32 trajectory owns acceleration */
          (uint8_t)(initial_position & 0xFFU),
          (uint8_t)((initial_position >> 8) & 0xFFU),
          0U, 0U,
          65U, 0U,
          0x90U, 0x01U       /* Torque limit 400 */
      };

      if ((Servo_WriteData(1U, 40U, torque_off, 1U) == HAL_OK) &&
          (Servo_WriteData(1U, 55U, lock_volatile, 1U) == HAL_OK) &&
          (Servo_WriteData(1U, 33U, position_mode, 1U) == HAL_OK) &&
          (Servo_WriteData(1U, 21U, pid_data, 3U) == HAL_OK) &&
          (Servo_WriteData(1U, 41U, runtime_data, 9U) == HAL_OK) &&
          (Servo_WriteData(1U, 40U, torque_on, 1U) == HAL_OK))
      {
          HAL_Delay(20U);

          uint8_t pid_readback[3] = {0U};

          if ((Servo_ReadData(
                  1U,
                  21U,
                  sizeof(pid_readback),
                  pid_readback
              ) == HAL_OK) &&
              (pid_readback[0] == 16U) &&
              (pid_readback[1] == 32U) &&
              (pid_readback[2] == 0U))
          {
              system_ready = 1U;

              int line_length = snprintf(
                  status_line,
                  sizeof(status_line),
				  "TRAJ_READY POS=%u P=16 ACC=0 SPEED=65 TORQUE=400 SEND_M_THEN_R\r\n",
                  (unsigned int)initial_position
              );

              if (line_length > 0)
              {
                  (void)HAL_UART_Transmit(
                      app_host_uart,
                      (uint8_t *)status_line,
                      (uint16_t)line_length,
                      100U
                  );
              }
          }
      }
  }

  if (system_ready == 0U)
  {
      static const uint8_t fail_message[] =
          "TRAJ_CONFIG_FAIL_TORQUE_MAY_BE_OFF\r\n";

      (void)HAL_UART_Transmit(
          app_host_uart,
          (uint8_t *)fail_message,
          sizeof(fail_message) - 1U,
          100U
      );
  }
#endif

}

void SingleArmApp_Process(void)
{
    if (BinaryControl_IsBinaryMode() != 0U)
    {
        /*
         * LPUART1 ISR captures host bytes while synchronous servo-bus calls are
         * in progress. A pre-existing RX fault is consumed before any buffered
         * byte is parsed; HostUartRx_TakeFault() also discards the invalidated
         * stream. Only an intact bounded burst reaches the protocol parser.
         */
        if (HostUartRx_TakeFault() != 0U)
        {
            BinaryControl_HandleHostUartError();
            BinaryControl_Service();
            return;
        }

        for (uint8_t processed = 0U;
             processed < HOST_BINARY_RX_BURST_MAX_BYTES;
             processed++)
        {
            if (HostUartRx_Pop(&rx_byte) == 0U)
            {
                break;
            }
            BinaryControl_ProcessByte(rx_byte);
        }

        if (HostUartRx_TakeFault() != 0U)
        {
            BinaryControl_HandleHostUartError();
        }

        BinaryControl_Service();
        return;
    }

    BinaryControl_Service();

      HAL_StatusTypeDef host_receive_status = HAL_UART_Receive(
              app_host_uart,
              &rx_byte,
              1U,
              10U
          );

      if (host_receive_status == HAL_OK)
      {
	  if (BinaryControl_IsBinaryMode() != 0U)
	  {
	      BinaryControl_ProcessByte(rx_byte);
	      return;
	  }
	  else if ((rx_byte == 'P') || (rx_byte == 'p'))
	  {
	      static const uint8_t binary_ready[] =
	          "BINARY_PROTOCOL_READY_RESET_TO_EXIT\r\n";

	      BinaryControl_EnterMode();

	      if (HostUartRx_Start() == HAL_OK)
	      {
	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)binary_ready,
	              sizeof(binary_ready) - 1U,
	              100U
	          );
	      }
	      else
	      {
	          static const uint8_t binary_rx_fail[] =
	              "BINARY_RX_INTERRUPT_START_FAIL_RESET_REQUIRED\r\n";
	          BinaryControl_HandleHostUartError();
	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)binary_rx_fail,
	              sizeof(binary_rx_fail) - 1U,
	              100U
	          );
	      }
	  }
	  else if ((rx_byte == 'X') || (rx_byte == 'x'))
	  {
	      BinaryControl_LatchStop();

	      static const uint8_t stop_message[] =
	          "STOP_LATCHED_HOLDING_POSITION\r\n";

	      (void)HAL_UART_Transmit(
	          app_host_uart,
	          (uint8_t *)stop_message,
	          sizeof(stop_message) - 1U,
	          100U
	      );
	  }
	  else if ((rx_byte == 'C') || (rx_byte == 'c'))
	  {
	      if (BinaryControl_StopIsLatched() == 0U)
	      {
	          static const uint8_t not_latched[] =
	              "STOP_NOT_LATCHED\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)not_latched,
	              sizeof(not_latched) - 1U,
	              100U
	          );
	      }
	      else
	      {
	          uint16_t current_positions[6] = {0U};
	          uint8_t unsafe_id = 0U;

	          if (Servo_ReadAllPositions(
	                  current_positions
	              ) != HAL_OK)
	          {
	              static const uint8_t clear_read_fail[] =
	                  "STOP_CLEAR_READ_FAIL_STILL_LATCHED\r\n";

	              (void)HAL_UART_Transmit(
	                  app_host_uart,
	                  (uint8_t *)clear_read_fail,
	                  sizeof(clear_read_fail) - 1U,
	                  100U
	              );
	          }
	          else
	          {
	              for (uint8_t i = 0U;
	                   i < servo_joint_count;
	                   i++)
	              {
	                  int32_t minimum_allowed =
	                      (int32_t)servo_joints[i].min_position - 40;
	                  int32_t maximum_allowed =
	                      (int32_t)servo_joints[i].max_position + 40;

	                  if (((int32_t)current_positions[i] <
	                          minimum_allowed) ||
	                      ((int32_t)current_positions[i] >
	                          maximum_allowed))
	                  {
	                      unsafe_id = servo_joints[i].id;
	                      break;
	                  }
	              }

	              if (unsafe_id != 0U)
	              {
	                  int line_length = snprintf(
	                      status_line,
	                      sizeof(status_line),
	                      "STOP_CLEAR_REJECTED ID=%u POS=%u\r\n",
	                      (unsigned int)unsafe_id,
	                      (unsigned int)current_positions[unsafe_id - 1U]
	                  );

	                  if ((line_length > 0) &&
	                      (line_length < (int)sizeof(status_line)))
	                  {
	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)status_line,
	                          (uint16_t)line_length,
	                          100U
	                      );
	                  }
	              }
	              else
	              {
	                  BinaryControl_ClearStopLatch();

	                  static const uint8_t cleared[] =
	                      "STOP_CLEARED_COMMANDS_ENABLED\r\n";

	                  (void)HAL_UART_Transmit(
	                      app_host_uart,
	                      (uint8_t *)cleared,
	                      sizeof(cleared) - 1U,
	                      100U
	                  );
	              }
	          }
	      }
	  }
	  else if ((BinaryControl_StopIsLatched() != 0U) &&
	           (rx_byte != 'S') && (rx_byte != 's'))
	  {
	      static const uint8_t stop_block[] =
	          "STOP_LATCHED_SEND_C_AFTER_SAFETY_CHECK\r\n";

	      (void)HAL_UART_Transmit(
	          app_host_uart,
	          (uint8_t *)stop_block,
	          sizeof(stop_block) - 1U,
	          100U
	      );
	  }
	  else if ((rx_byte == '1') || (rx_byte == '2') || (rx_byte == '3') || (rx_byte == '4') || (rx_byte == '5') || (rx_byte == '6'))
    	  {
	      if (synchronized_pose_active != 0U)
	      {
	          static const uint8_t sync_return_first[] =
	              "SYNC_RETURN_B_REQUIRED\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)sync_return_first,
	              sizeof(sync_return_first) - 1U,
	              100U
	          );
	      }
	      else if (absolute_pose_active != 0U)
	      {
	          static const uint8_t absolute_home_first[] =
	              "ABSOLUTE_MODE_SEND_J_HOME_FIRST\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)absolute_home_first,
	              sizeof(absolute_home_first) - 1U,
	              100U
	          );
	      }
	      else if (return_available != 0U)
    	      {
    	          static const uint8_t return_first[] =
    	              "RETURN_REQUIRED_BEFORE_SELECTION\r\n";

    	          (void)HAL_UART_Transmit(
    	              app_host_uart,
    	              (uint8_t *)return_first,
    	              sizeof(return_first) - 1U,
    	              100U
    	          );
    	      }
    	      else
    	      {
    	          uint8_t requested_id =
    	              (uint8_t)(rx_byte - (uint8_t)'0');

    	          uint8_t selection_found = 0U;

    	          for (uint8_t joint_index = 0U;
    	               joint_index < servo_joint_count;
    	               joint_index++)
    	          {
    	              if ((servo_joints[joint_index].id ==
    	                      requested_id) &&
    	                  (servo_joints[joint_index].motion_enabled != 0U))
    	              {
    	                  selection_found = 1U;
    	                  system_ready = 0U;

    	                  uint16_t configured_position = 0U;

	                  if (Servo_ConfigureForTrajectory(
	                          servo_joints[joint_index].id,
	                          servo_joints[joint_index].torque_limit,
	                          servo_joints[joint_index].p_gain,
	                          servo_joints[joint_index].d_gain,
	                          &configured_position
    	                      ) == HAL_OK)
    	                  {
    	                      active_servo_id =
    	                          servo_joints[joint_index].id;

    	                      active_joint_name =
    	                          servo_joints[joint_index].name;

    	                      active_test_delta =
    	                          servo_joints[joint_index].test_delta;

    	                      active_duration_ms =
    	                          servo_joints[joint_index].duration_ms;

    	                      active_torque_limit =
    	                          servo_joints[joint_index].torque_limit;

    	                      active_home_position =
    	                          servo_joints[joint_index].home_position;

    	                      active_test_direction =
    	                          servo_joints[joint_index].test_direction;

    	                      move_already_done = 0U;
    	                      return_available = 0U;
    	                      system_ready = 1U;

    	                      int line_length = snprintf(
    	                          status_line,
    	                          sizeof(status_line),
	                          "JOINT_SELECTED ID=%u NAME=%s POS=%u HOME=%u DELTA=%u DURATION=%luMS TORQUE=%u\r\n",
	                          (unsigned int)active_servo_id,
	                          active_joint_name,
	                          (unsigned int)configured_position,
	                          (unsigned int)active_home_position,
	                          (unsigned int)active_test_delta,
    	                          (unsigned long)active_duration_ms,
    	                          (unsigned int)active_torque_limit
    	                      );

    	                      if ((line_length > 0) &&
    	                          (line_length <
    	                              (int)sizeof(status_line)))
    	                      {
    	                          (void)HAL_UART_Transmit(
    	                              app_host_uart,
    	                              (uint8_t *)status_line,
    	                              (uint16_t)line_length,
    	                              100U
    	                          );
    	                      }
    	                  }
    	                  else
    	                  {
    	                      static const uint8_t select_fail[] =
    	                          "JOINT_CONFIG_FAIL\r\n";

    	                      (void)HAL_UART_Transmit(
    	                          app_host_uart,
    	                          (uint8_t *)select_fail,
    	                          sizeof(select_fail) - 1U,
    	                          100U
    	                      );
    	                  }

    	                  break;
    	              }
    	          }

    	          if (selection_found == 0U)
    	          {
    	              static const uint8_t locked_message[] =
    	                  "JOINT_SELECTION_LOCKED\r\n";

    	              (void)HAL_UART_Transmit(
    	                  app_host_uart,
    	                  (uint8_t *)locked_message,
    	                  sizeof(locked_message) - 1U,
    	                  100U
    	              );
    	          }
    	      }
    	  }
	  else if ((rx_byte == 'J') || (rx_byte == 'j'))
	  {
	      if (synchronized_pose_active != 0U)
	      {
	          static const uint8_t sync_return_first[] =
	              "J_REJECTED_SEND_B_FIRST\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)sync_return_first,
	              sizeof(sync_return_first) - 1U,
	              100U
	          );
	      }
	      else if (return_available != 0U)
	      {
	          static const uint8_t single_return_first[] =
	              "J_REJECTED_SEND_SINGLE_R_FIRST\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)single_return_first,
	              sizeof(single_return_first) - 1U,
	              100U
	          );
	      }
	      else
	      {
	          char command_line[96];
	          uint16_t target_positions[6] = {0U};
	          uint32_t duration_ms = 0U;

	          if ((Host_ReadLineAfterCommand(
	                  command_line,
	                  sizeof(command_line),
	                  2000U
	              ) != HAL_OK) ||
	              (Host_ParseAbsoluteJointCommand(
	                  command_line,
	                  target_positions,
	                  &duration_ms
	              ) != HAL_OK))
	          {
	              static const uint8_t format_fail[] =
	                  "J_REJECTED_FORMAT USE_J_P1_P2_P3_P4_P5_P6_MS\r\n";

	              (void)HAL_UART_Transmit(
	                  app_host_uart,
	                  (uint8_t *)format_fail,
	                  sizeof(format_fail) - 1U,
	                  100U
	              );
	          }
	          else
	          {
	              uint8_t limit_failed_index = 0xFFU;

	              for (uint8_t i = 0U;
	                   i < servo_joint_count;
	                   i++)
	              {
	                  if ((target_positions[i] <
	                          servo_joints[i].min_position) ||
	                      (target_positions[i] >
	                          servo_joints[i].max_position))
	                  {
	                      limit_failed_index = i;
	                      break;
	                  }
	              }

	              if (limit_failed_index != 0xFFU)
	              {
	                  int line_length = snprintf(
	                      status_line,
	                      sizeof(status_line),
	                      "J_REJECTED_LIMIT ID=%u TARGET=%u MIN=%u MAX=%u\r\n",
	                      (unsigned int)servo_joints[limit_failed_index].id,
	                      (unsigned int)target_positions[limit_failed_index],
	                      (unsigned int)servo_joints[limit_failed_index].min_position,
	                      (unsigned int)servo_joints[limit_failed_index].max_position
	                  );

	                  if ((line_length > 0) &&
	                      (line_length < (int)sizeof(status_line)))
	                  {
	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)status_line,
	                          (uint16_t)line_length,
	                          100U
	                      );
	                  }
	              }
	              else
	              {
	                  uint16_t start_positions[6] = {0U};
	                  uint8_t current_failed_index = 0xFFU;

	                  if (Servo_ReadAllPositions(
	                          start_positions
	                      ) != HAL_OK)
	                  {
	                      static const uint8_t read_fail[] =
	                          "J_READ_FAIL_POWER_OFF\r\n";

	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)read_fail,
	                          sizeof(read_fail) - 1U,
	                          100U
	                      );
	                  }
	                  else
	                  {
	                      for (uint8_t i = 0U;
	                           i < servo_joint_count;
	                           i++)
	                      {
	                          int32_t minimum_allowed =
	                              (int32_t)servo_joints[i].min_position - 40;
	                          int32_t maximum_allowed =
	                              (int32_t)servo_joints[i].max_position + 40;

	                          if (((int32_t)start_positions[i] <
	                                  minimum_allowed) ||
	                              ((int32_t)start_positions[i] >
	                                  maximum_allowed))
	                          {
	                              current_failed_index = i;
	                              break;
	                          }
	                      }

	                      if (current_failed_index != 0xFFU)
	                      {
	                          int line_length = snprintf(
	                              status_line,
	                              sizeof(status_line),
	                              "J_REJECTED_CURRENT_LIMIT ID=%u POS=%u\r\n",
	                              (unsigned int)servo_joints[current_failed_index].id,
	                              (unsigned int)start_positions[current_failed_index]
	                          );

	                          if ((line_length > 0) &&
	                              (line_length < (int)sizeof(status_line)))
	                          {
	                              (void)HAL_UART_Transmit(
	                                  app_host_uart,
	                                  (uint8_t *)status_line,
	                                  (uint16_t)line_length,
	                                  100U
	                              );
	                          }
	                      }
	                      else
	                      {
	                          int line_length = snprintf(
	                              status_line,
	                              sizeof(status_line),
	                              "J_ACCEPTED DURATION=%luMS CONFIGURING_6_AXES\r\n",
	                              (unsigned long)duration_ms
	                          );

	                          if ((line_length > 0) &&
	                              (line_length < (int)sizeof(status_line)))
	                          {
	                              (void)HAL_UART_Transmit(
	                                  app_host_uart,
	                                  (uint8_t *)status_line,
	                                  (uint16_t)line_length,
	                                  100U
	                              );
	                          }

	                          if (Servo_ConfigureAllForTrajectory(
	                                  start_positions
	                              ) != HAL_OK)
	                          {
	                              static const uint8_t config_fail[] =
	                                  "J_CONFIG_FAIL_TORQUE_OFF\r\n";

	                              (void)HAL_UART_Transmit(
	                                  app_host_uart,
	                                  (uint8_t *)config_fail,
	                                  sizeof(config_fail) - 1U,
	                                  100U
	                              );
	                          }
	                          else
	                          {
	                              static const uint8_t start_message[] =
	                                  "J_START\r\n";

	                              (void)HAL_UART_Transmit(
	                                  app_host_uart,
	                                  (uint8_t *)start_message,
	                                  sizeof(start_message) - 1U,
	                                  100U
	                              );

	                              HAL_StatusTypeDef run_status =
	                                  Servo_RunSynchronizedSmoothstep(
	                                      start_positions,
	                                      target_positions,
	                                      duration_ms
	                                  );

	                              if (run_status != HAL_OK)
	                              {
	                                  absolute_pose_active = 1U;

	                                  static const uint8_t stopped[] =
	                                      "J_STOPPED_HOLDING_POSITION SEND_C_AFTER_CHECK\r\n";
	                                  static const uint8_t run_fail[] =
	                                      "J_RUN_FAIL_POWER_OFF\r\n";
	                                  const uint8_t *message =
	                                      (run_status == HAL_BUSY)
	                                          ? stopped
	                                          : run_fail;
	                                  uint16_t message_length =
	                                      (run_status == HAL_BUSY)
	                                          ? (uint16_t)(sizeof(stopped) - 1U)
	                                          : (uint16_t)(sizeof(run_fail) - 1U);

	                                  (void)HAL_UART_Transmit(
	                                      app_host_uart,
	                                      (uint8_t *)message,
	                                      message_length,
	                                      100U
	                                  );
	                                  SingleArmApp_ReportMotionSafety();
	                              }
	                              else
	                              {
	                                  HAL_Delay(300U);
	                                  uint16_t end_positions[6] = {0U};

	                                  if (Servo_ReadAllPositions(
	                                          end_positions
	                                      ) != HAL_OK)
	                                  {
	                                      absolute_pose_active = 1U;

	                                      static const uint8_t verify_fail[] =
	                                          "J_VERIFY_READ_FAIL_SEND_J_HOME\r\n";

	                                      (void)HAL_UART_Transmit(
	                                          app_host_uart,
	                                          (uint8_t *)verify_fail,
	                                          sizeof(verify_fail) - 1U,
	                                          100U
	                                      );
	                                  }
	                                  else
	                                  {
	                                      int32_t worst_error = 0;
	                                      uint8_t worst_id = 1U;
	                                      uint8_t target_is_home = 1U;

	                                      for (uint8_t i = 0U;
	                                           i < servo_joint_count;
	                                           i++)
	                                      {
	                                          int32_t error = Servo_PositionError(
	                                              end_positions[i],
	                                              target_positions[i]
	                                          );
	                                          int32_t error_magnitude =
	                                              (error < 0) ? -error : error;

	                                          if (error_magnitude > worst_error)
	                                          {
	                                              worst_error = error_magnitude;
	                                              worst_id = servo_joints[i].id;
	                                          }

	                                          if (target_positions[i] !=
	                                              servo_joints[i].home_position)
	                                          {
	                                              target_is_home = 0U;
	                                          }
	                                      }

	                                      absolute_pose_active =
	                                          ((target_is_home != 0U) &&
	                                           (worst_error <= 20))
	                                              ? 0U
	                                              : 1U;
	                                      system_ready = 0U;
	                                      move_already_done = 0U;
	                                      return_available = 0U;

	                                      line_length = snprintf(
	                                          status_line,
	                                          sizeof(status_line),
	                                          "J_END WORST_ID=%u MAX_ERROR=%ld STATUS=%s ACTIVE=%u\r\n",
	                                          (unsigned int)worst_id,
	                                          (long)worst_error,
	                                          (worst_error <= 20) ? "OK" : "CHECK",
	                                          (unsigned int)absolute_pose_active
	                                      );

	                                      if ((line_length > 0) &&
	                                          (line_length < (int)sizeof(status_line)))
	                                      {
	                                          (void)HAL_UART_Transmit(
	                                              app_host_uart,
	                                              (uint8_t *)status_line,
	                                              (uint16_t)line_length,
	                                              100U
	                                          );
	                                      }
	                                  }
	                              }
	                          }
	                      }
	                  }
	              }
	          }
	      }
	  }
	  else if ((rx_byte == 'A') || (rx_byte == 'a'))
	  {
	      if (synchronized_pose_active != 0U)
	      {
	          static const uint8_t already_out[] =
	              "SYNC_A_ALREADY_ACTIVE_SEND_B\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)already_out,
	              sizeof(already_out) - 1U,
	              100U
	          );
	      }
	      else if (absolute_pose_active != 0U)
	      {
	          static const uint8_t absolute_home_first[] =
	              "SYNC_A_REJECTED_SEND_J_HOME_FIRST\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)absolute_home_first,
	              sizeof(absolute_home_first) - 1U,
	              100U
	          );
	      }
	      else if (return_available != 0U)
	      {
	          static const uint8_t single_return_first[] =
	              "SYNC_A_REJECTED_SEND_SINGLE_R_FIRST\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)single_return_first,
	              sizeof(single_return_first) - 1U,
	              100U
	          );
	      }
	      else
	      {
	          uint16_t start_positions[6] = {0U};
	          uint16_t target_positions[6] = {0U};
	          uint8_t rejected_id = 0U;

	          if (Servo_ReadAllPositions(start_positions) != HAL_OK)
	          {
	              static const uint8_t read_fail[] =
	                  "SYNC_A_READ_FAIL_POWER_OFF\r\n";

	              (void)HAL_UART_Transmit(
	                  app_host_uart,
	                  (uint8_t *)read_fail,
	                  sizeof(read_fail) - 1U,
	                  100U
	              );
	          }
	          else
	          {
	              for (uint8_t i = 0U;
	                   i < servo_joint_count;
	                   i++)
	              {
	                  int32_t home_error = Servo_PositionError(
	                      start_positions[i],
	                      servo_joints[i].home_position
	                  );

	                  if ((home_error < -80) ||
	                      (home_error > 80))
	                  {
	                      rejected_id = servo_joints[i].id;
	                      break;
	                  }
	              }

	              if (rejected_id != 0U)
	              {
	                  int line_length = snprintf(
	                      status_line,
	                      sizeof(status_line),
	                      "SYNC_A_REJECTED_NOT_HOME ID=%u POS=%u HOME=2048\r\n",
	                      (unsigned int)rejected_id,
	                      (unsigned int)start_positions[rejected_id - 1U]
	                  );

	                  if ((line_length > 0) &&
	                      (line_length < (int)sizeof(status_line)))
	                  {
	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)status_line,
	                          (uint16_t)line_length,
	                          100U
	                      );
	                  }
	              }
	              else
	              {
	                  static const uint8_t configuring[] =
	                      "SYNC_A_CONFIGURING_6_AXES\r\n";

	                  (void)HAL_UART_Transmit(
	                      app_host_uart,
	                      (uint8_t *)configuring,
	                      sizeof(configuring) - 1U,
	                      100U
	                  );

	                  if (Servo_ConfigureAllForTrajectory(
	                          start_positions
	                      ) != HAL_OK)
	                  {
	                      static const uint8_t config_fail[] =
	                          "SYNC_A_CONFIG_FAIL_TORQUE_OFF\r\n";

	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)config_fail,
	                          sizeof(config_fail) - 1U,
	                          100U
	                      );
	                  }
	                  else
	                  {
	                      for (uint8_t i = 0U;
	                           i < servo_joint_count;
	                           i++)
	                      {
	                          int32_t target =
	                              (int32_t)servo_joints[i].home_position +
	                              ((int32_t)servo_joints[i].test_direction *
	                               (int32_t)servo_joints[i].test_delta);

	                          target_positions[i] = (uint16_t)target;
	                      }

	                      static const uint8_t sync_start[] =
	                          "SYNC_A_START DURATION=1500MS DELTA=34\r\n";

	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)sync_start,
	                          sizeof(sync_start) - 1U,
	                          100U
	                      );

	                      if (Servo_RunSynchronizedSmoothstep(
	                              start_positions,
	                              target_positions,
	                              1500U
	                          ) != HAL_OK)
	                      {
	                          static const uint8_t run_fail[] =
	                              "SYNC_A_RUN_FAIL_POWER_OFF\r\n";

	                          (void)HAL_UART_Transmit(
	                              app_host_uart,
	                              (uint8_t *)run_fail,
	                              sizeof(run_fail) - 1U,
	                              100U
	                          );
	                          SingleArmApp_ReportMotionSafety();
	                      }
	                      else
	                      {
	                          HAL_Delay(200U);
	                          uint16_t end_positions[6] = {0U};

	                          if (Servo_ReadAllPositions(
	                                  end_positions
	                              ) != HAL_OK)
	                          {
	                              static const uint8_t verify_read_fail[] =
	                                  "SYNC_A_VERIFY_READ_FAIL_SEND_B\r\n";

	                              (void)HAL_UART_Transmit(
	                                  app_host_uart,
	                                  (uint8_t *)verify_read_fail,
	                                  sizeof(verify_read_fail) - 1U,
	                                  100U
	                              );
	                              synchronized_pose_active = 1U;
	                          }
	                          else
	                          {
	                              int32_t worst_error = 0;
	                              uint8_t worst_id = 1U;

	                              for (uint8_t i = 0U;
	                                   i < servo_joint_count;
	                                   i++)
	                              {
	                                  int32_t error = Servo_PositionError(
	                                      end_positions[i],
	                                      target_positions[i]
	                                  );

	                                  int32_t error_magnitude =
	                                      (error < 0) ? -error : error;

	                                  if (error_magnitude > worst_error)
	                                  {
	                                      worst_error = error_magnitude;
	                                      worst_id = servo_joints[i].id;
	                                  }
	                              }

	                              int line_length = snprintf(
	                                  status_line,
	                                  sizeof(status_line),
	                                  "SYNC_A_END WORST_ID=%u MAX_ERROR=%ld STATUS=%s SEND_B\r\n",
	                                  (unsigned int)worst_id,
	                                  (long)worst_error,
	                                  (worst_error <= 20) ? "OK" : "CHECK"
	                              );

	                              if ((line_length > 0) &&
	                                  (line_length < (int)sizeof(status_line)))
	                              {
	                                  (void)HAL_UART_Transmit(
	                                      app_host_uart,
	                                      (uint8_t *)status_line,
	                                      (uint16_t)line_length,
	                                      100U
	                                  );
	                              }

	                              synchronized_pose_active = 1U;
	                              system_ready = 0U;
	                              move_already_done = 0U;
	                              return_available = 0U;
	                          }
	                      }
	                  }
	              }
	          }
	      }
	  }
	  else if ((rx_byte == 'B') || (rx_byte == 'b'))
	  {
	      if (synchronized_pose_active == 0U)
	      {
	          static const uint8_t no_pose[] =
	              "SYNC_B_REJECTED_NO_ACTIVE_POSE\r\n";

	          (void)HAL_UART_Transmit(
	              app_host_uart,
	              (uint8_t *)no_pose,
	              sizeof(no_pose) - 1U,
	              100U
	          );
	      }
	      else
	      {
	          uint16_t start_positions[6] = {0U};
	          uint16_t home_positions[6] = {0U};

	          if (Servo_ReadAllPositions(start_positions) != HAL_OK)
	          {
	              static const uint8_t read_fail[] =
	                  "SYNC_B_READ_FAIL_POWER_OFF\r\n";

	              (void)HAL_UART_Transmit(
	                  app_host_uart,
	                  (uint8_t *)read_fail,
	                  sizeof(read_fail) - 1U,
	                  100U
	              );
	          }
	          else
	          {
	              for (uint8_t i = 0U;
	                   i < servo_joint_count;
	                   i++)
	              {
	                  home_positions[i] =
	                      servo_joints[i].home_position;
	              }

	              static const uint8_t sync_start[] =
	                  "SYNC_B_HOME_START DURATION=1500MS\r\n";

	              (void)HAL_UART_Transmit(
	                  app_host_uart,
	                  (uint8_t *)sync_start,
	                  sizeof(sync_start) - 1U,
	                  100U
	              );

	              if (Servo_RunSynchronizedSmoothstep(
	                      start_positions,
	                      home_positions,
	                      1500U
	                  ) != HAL_OK)
	              {
	                  static const uint8_t run_fail[] =
	                      "SYNC_B_RUN_FAIL_POWER_OFF\r\n";

	                  (void)HAL_UART_Transmit(
	                      app_host_uart,
	                      (uint8_t *)run_fail,
	                      sizeof(run_fail) - 1U,
	                      100U
	                  );
	                  SingleArmApp_ReportMotionSafety();
	              }
	              else
	              {
	                  HAL_Delay(200U);
	                  uint16_t end_positions[6] = {0U};

	                  if (Servo_ReadAllPositions(
	                          end_positions
	                      ) != HAL_OK)
	                  {
	                      static const uint8_t verify_fail[] =
	                          "SYNC_B_VERIFY_READ_FAIL_POWER_OFF\r\n";

	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)verify_fail,
	                          sizeof(verify_fail) - 1U,
	                          100U
	                      );
	                  }
	                  else
	                  {
	                      int32_t worst_error = 0;
	                      uint8_t worst_id = 1U;

	                      for (uint8_t i = 0U;
	                           i < servo_joint_count;
	                           i++)
	                      {
	                          int32_t error = Servo_PositionError(
	                              end_positions[i],
	                              home_positions[i]
	                          );

	                          int32_t error_magnitude =
	                              (error < 0) ? -error : error;

	                          if (error_magnitude > worst_error)
	                          {
	                              worst_error = error_magnitude;
	                              worst_id = servo_joints[i].id;
	                          }
	                      }

	                      int line_length = snprintf(
	                          status_line,
	                          sizeof(status_line),
	                          "SYNC_B_HOME_END WORST_ID=%u MAX_ERROR=%ld STATUS=%s\r\n",
	                          (unsigned int)worst_id,
	                          (long)worst_error,
	                          (worst_error <= 20) ? "OK" : "CHECK"
	                      );

	                      if ((line_length > 0) &&
	                          (line_length < (int)sizeof(status_line)))
	                      {
	                          (void)HAL_UART_Transmit(
	                              app_host_uart,
	                              (uint8_t *)status_line,
	                              (uint16_t)line_length,
	                              100U
	                          );
	                      }

	                      if (worst_error <= 20)
	                      {
	                          synchronized_pose_active = 0U;
	                      }
	                  }
	              }
	          }
	      }
	  }
	  else if ((rx_byte == 'S') || (rx_byte == 's'))
    	  {
    	      static const uint8_t scan_start[] =
    	          "ALL_AXIS_STATUS\r\n";

    	      (void)HAL_UART_Transmit(
    	          app_host_uart,
    	          (uint8_t *)scan_start,
    	          sizeof(scan_start) - 1U,
    	          100U
    	      );

    	      for (uint8_t joint_index = 0U;
    	           joint_index < servo_joint_count;
    	           joint_index++)
    	      {
    	          uint16_t position = 0U;
    	          uint16_t speed_raw = 0U;
	          uint16_t load_raw = 0U;
	          uint8_t voltage_raw = 0U;
	          uint8_t temperature_c = 0U;
	          uint16_t current_raw = 0U;

    	          HAL_StatusTypeDef telemetry_status =
    	              Servo_ReadTelemetry(
    	                  servo_joints[joint_index].id,
    	                  &position,
    	                  &speed_raw,
	                  &load_raw,
	                  &voltage_raw,
	                  &temperature_c,
	                  &current_raw
	              );

    	          int line_length;

	          if (telemetry_status == HAL_OK)
	          {
	              uint16_t load_magnitude =
	                  (uint16_t)(load_raw & 0x03FFU);
	              int16_t load_signed =
	                  ((load_raw & 0x0400U) != 0U)
	                      ? -(int16_t)load_magnitude
	                      : (int16_t)load_magnitude;
	              int32_t current_signed =
	                  (int32_t)(int16_t)current_raw;
	              if (current_signed < 0)
	              {
	                  current_signed = -current_signed;
	              }
	              uint32_t current_ma =
	                  ((uint32_t)current_signed * 65U) / 10U;

	              line_length = snprintf(
	                  status_line,
	                  sizeof(status_line),
	                  "AXIS ID=%u NAME=%s POS=%u SPEED_RAW=%u LOAD=%d LOAD_RAW=%u CURRENT_RAW=%lu CURRENT_MA=%lu VOLT=%u.%uV TEMP=%uC MOTION=%s\r\n",
	                  (unsigned int)servo_joints[joint_index].id,
	                  servo_joints[joint_index].name,
	                  (unsigned int)position,
	                  (unsigned int)speed_raw,
	                  (int)load_signed,
	                  (unsigned int)load_raw,
	                  (unsigned long)current_signed,
	                  (unsigned long)current_ma,
	                  (unsigned int)(voltage_raw / 10U),
    	                  (unsigned int)(voltage_raw % 10U),
    	                  (unsigned int)temperature_c,
    	                  (servo_joints[joint_index].motion_enabled != 0U)
    	                      ? "ENABLED"
    	                      : "LOCKED"
    	              );
    	          }
    	          else
    	          {
    	              line_length = snprintf(
    	                  status_line,
    	                  sizeof(status_line),
    	                  "AXIS ID=%u NAME=%s READ_FAIL\r\n",
    	                  (unsigned int)servo_joints[joint_index].id,
    	                  servo_joints[joint_index].name
    	              );
    	          }

    	          if ((line_length > 0) &&
    	              (line_length < (int)sizeof(status_line)))
    	          {
    	              (void)HAL_UART_Transmit(
    	                  app_host_uart,
    	                  (uint8_t *)status_line,
    	                  (uint16_t)line_length,
    	                  100U
    	              );
    	          }

    	          HAL_Delay(10U);
    	      }

    	      static const uint8_t scan_end[] =
    	          "ALL_AXIS_STATUS_END\r\n";

    	      (void)HAL_UART_Transmit(
    	          app_host_uart,
    	          (uint8_t *)scan_end,
    	          sizeof(scan_end) - 1U,
    	          100U
    	      );
    	  }
    	  else if ((rx_byte == 'M') || (rx_byte == 'm'))
          {
              if (system_ready == 0U)
              {
                  static const uint8_t not_ready[] =
                      "TRAJ_NOT_READY\r\n";

                  (void)HAL_UART_Transmit(
                      app_host_uart,
                      (uint8_t *)not_ready,
                      sizeof(not_ready) - 1U,
                      100U
                  );
              }
              else if (move_already_done != 0U)
              {
                  static const uint8_t send_return[] =
                      "SEND_R_FIRST\r\n";

                  (void)HAL_UART_Transmit(
                      app_host_uart,
                      (uint8_t *)send_return,
                      sizeof(send_return) - 1U,
                      100U
                  );
              }
              else
              {
                  uint16_t start_position = 0U;

                  if (Servo_ReadPosition(
                		  active_servo_id,
                          &start_position
                      ) == HAL_OK)
                  {
                	  int32_t calculated_target =
                	      (int32_t)active_home_position +
                	      ((int32_t)active_test_direction *
                	       (int32_t)active_test_delta);

                	  if ((calculated_target < 0) ||
                	      (calculated_target > 4095))
                	  {
                	      static const uint8_t target_invalid[] =
                	          "TARGET_OUT_OF_RANGE\r\n";

                	      (void)HAL_UART_Transmit(
                	          app_host_uart,
                	          (uint8_t *)target_invalid,
                	          sizeof(target_invalid) - 1U,
                	          100U
                	      );

                	      return;
                	  }

                	  uint16_t target_position =
                	      (uint16_t)calculated_target;

                	  return_position = active_home_position;
                      return_available = 1U;
                      move_already_done = 1U;

                      int line_length = snprintf(
                          status_line,
                          sizeof(status_line),
						  "OUT_START ID=%u NAME=%s POS=%u TARGET=%u DURATION=%luMS\r\n",
						  (unsigned int)active_servo_id,
						  active_joint_name,
						  (unsigned int)start_position,
						  (unsigned int)target_position,
						  (unsigned long)active_duration_ms
                      );

                      if (line_length > 0)
                      {
                          (void)HAL_UART_Transmit(
                              app_host_uart,
                              (uint8_t *)status_line,
                              (uint16_t)line_length,
                              100U
                          );
                      }

                      if (Servo_RunSmoothstep(
                    		  active_servo_id,
                              start_position,
                              target_position,
							  active_duration_ms
                          ) == HAL_OK)
                      {
                          HAL_Delay(500U);

                          uint16_t end_position = 0U;

                          if (Servo_ReadPosition(
                        		  active_servo_id,
                                  &end_position
                              ) == HAL_OK)
                          {
                              int32_t target_error =
                                  (int32_t)end_position -
                                  (int32_t)target_position;

                              line_length = snprintf(
                                  status_line,
                                  sizeof(status_line),
                                  "OUT_END=%u ERROR=%ld SEND_R\r\n",
                                  (unsigned int)end_position,
                                  (long)target_error
                              );

                              if (line_length > 0)
                              {
                                  (void)HAL_UART_Transmit(
                                      app_host_uart,
                                      (uint8_t *)status_line,
                                      (uint16_t)line_length,
                                      100U
                                  );
                              }
                          }
                          else
                          {
                              static const uint8_t end_read_fail[] =
                                  "OUT_END_READ_FAIL_SEND_R\r\n";

                              (void)HAL_UART_Transmit(
                                  app_host_uart,
                                  (uint8_t *)end_read_fail,
                                  sizeof(end_read_fail) - 1U,
                                  100U
                              );
                          }
                      }
                      else
                      {
                          static const uint8_t out_write_fail[] =
                              "OUT_WRITE_FAIL_SEND_R\r\n";

                          (void)HAL_UART_Transmit(
                              app_host_uart,
                              (uint8_t *)out_write_fail,
                              sizeof(out_write_fail) - 1U,
                              100U
                          );
                          SingleArmApp_ReportMotionSafety();
                      }
                  }
                  else
                  {
                      static const uint8_t start_read_fail[] =
                          "OUT_START_READ_FAIL\r\n";

                      (void)HAL_UART_Transmit(
                          app_host_uart,
                          (uint8_t *)start_read_fail,
                          sizeof(start_read_fail) - 1U,
                          100U
                      );
                  }
              }
          }
#if ENABLE_SERVO_CENTERING_COMMAND
	  else if (rx_byte == 'Z')
    	  {
    	      if (centering_done != 0U)
    	      {
    	          static const uint8_t already_centered[] =
    	              "CENTER_ALREADY_DONE_REFLASH_REQUIRED\r\n";

    	          (void)HAL_UART_Transmit(
    	              app_host_uart,
    	              (uint8_t *)already_centered,
    	              sizeof(already_centered) - 1U,
    	              100U
    	          );
    	      }
    	      else
    	      {
	          static const uint8_t center_start[] =
	              "CENTER_ONE_START_DO_NOT_TOUCH\r\n";

    	          (void)HAL_UART_Transmit(
    	              app_host_uart,
    	              (uint8_t *)center_start,
    	              sizeof(center_start) - 1U,
    	              100U
    	          );

    	          /* 말단부터 베이스 방향 */
	          const uint8_t center_order[1] = {
	              active_servo_id
	          };

    	          uint8_t centered_count = 0U;

	          for (uint8_t i = 0U; i < 1U; i++)
	          {
	              uint16_t position_before = 0U;
	              int16_t offset_before = 0;

    	              HAL_StatusTypeDef center_status =
	                  Servo_CenterAtCurrentPosition(
	                      center_order[i],
	                      &position_before,
	                      &offset_before
	                  );

    	              int line_length = snprintf(
    	                  status_line,
    	                  sizeof(status_line),
	                  "CENTER_COMMAND ID=%u POS_BEFORE=%u OFFSET_BEFORE=%d STATUS=%s\r\n",
	                  (unsigned int)center_order[i],
	                  (unsigned int)position_before,
	                  (int)offset_before,
	                  (center_status == HAL_OK) ? "OK" : "FAIL"
	              );

    	              if ((line_length > 0) &&
    	                  (line_length < (int)sizeof(status_line)))
    	              {
    	                  (void)HAL_UART_Transmit(
    	                      app_host_uart,
    	                      (uint8_t *)status_line,
    	                      (uint16_t)line_length,
    	                      100U
    	                  );
    	              }

    	              if (center_status != HAL_OK)
    	              {
    	                  break;
    	              }

    	              centered_count++;
    	          }

	          if (centered_count == 1U)
    	          {
    	              centering_done = 1U;

	              static const uint8_t center_ok[] =
	                  "CENTER_COMMAND_SENT_POWER_CYCLE_REQUIRED\r\n";

    	              (void)HAL_UART_Transmit(
    	                  app_host_uart,
    	                  (uint8_t *)center_ok,
    	                  sizeof(center_ok) - 1U,
    	                  100U
    	              );
    	          }
    	          else
    	          {
	              static const uint8_t center_fail[] =
	                  "CENTER_COMMAND_FAILED_POWER_OFF\r\n";

    	              (void)HAL_UART_Transmit(
    	                  app_host_uart,
    	                  (uint8_t *)center_fail,
    	                  sizeof(center_fail) - 1U,
    	                  100U
    	              );
    	          }
    	      }
	  }
#endif
	  else if ((rx_byte == 'H') || (rx_byte == 'h'))
    	  {
    	      if (system_ready == 0U)
    	      {
    	          static const uint8_t home_not_ready[] =
    	              "HOME_NOT_READY_SELECT_JOINT\r\n";

    	          (void)HAL_UART_Transmit(
    	              app_host_uart,
    	              (uint8_t *)home_not_ready,
    	              sizeof(home_not_ready) - 1U,
    	              100U
    	          );
    	      }
    	      else
    	      {
    	          uint16_t home_start = 0U;

    	          if (Servo_ReadPosition(
    	                  active_servo_id,
    	                  &home_start
    	              ) == HAL_OK)
    	          {
    	              uint32_t home_distance =
    	                  (home_start >= active_home_position) ?
    	                  (uint32_t)(home_start - active_home_position) :
    	                  (uint32_t)(active_home_position - home_start);

    	              /* 현재 단계에서는 홈에서 약 45도 이상이면 거부 */
    	              if (home_distance > 512U)
    	              {
    	                  int line_length = snprintf(
    	                      status_line,
    	                      sizeof(status_line),
    	                      "HOME_REJECTED POS=%u HOME=%u DIST=%lu\r\n",
    	                      (unsigned int)home_start,
    	                      (unsigned int)active_home_position,
    	                      (unsigned long)home_distance
    	                  );

    	                  if (line_length > 0)
    	                  {
    	                      (void)HAL_UART_Transmit(
    	                          app_host_uart,
    	                          (uint8_t *)status_line,
    	                          (uint16_t)line_length,
    	                          100U
    	                      );
    	                  }
    	              }
    	              else if (home_distance <= 3U)
    	              {
    	                  static const uint8_t already_home[] =
    	                      "HOME_ALREADY_REACHED\r\n";

    	                  (void)HAL_UART_Transmit(
    	                      app_host_uart,
    	                      (uint8_t *)already_home,
    	                      sizeof(already_home) - 1U,
    	                      100U
    	                  );

    	                  return_available = 0U;
    	                  move_already_done = 0U;
    	              }
    	              else
    	              {
    	                  uint32_t home_duration =
    	                      ((home_distance * active_duration_ms) +
    	                       active_test_delta - 1U) /
    	                      active_test_delta;

    	                  if (home_duration < 200U)
    	                  {
    	                      home_duration = 200U;
    	                  }

    	                  /* Smoothstep 제어 주기인 20ms 배수로 정렬 */
    	                  home_duration =
    	                      ((home_duration + 19U) / 20U) * 20U;

    	                  int line_length = snprintf(
    	                      status_line,
    	                      sizeof(status_line),
    	                      "HOME_START ID=%u NAME=%s POS=%u TARGET=%u DURATION=%luMS\r\n",
    	                      (unsigned int)active_servo_id,
    	                      active_joint_name,
    	                      (unsigned int)home_start,
    	                      (unsigned int)active_home_position,
    	                      (unsigned long)home_duration
    	                  );

    	                  if (line_length > 0)
    	                  {
    	                      (void)HAL_UART_Transmit(
    	                          app_host_uart,
    	                          (uint8_t *)status_line,
    	                          (uint16_t)line_length,
    	                          100U
    	                      );
    	                  }

    	                  if (Servo_RunSmoothstep(
    	                          active_servo_id,
    	                          home_start,
    	                          active_home_position,
    	                          home_duration
    	                      ) == HAL_OK)
    	                  {
    	                	  uint16_t home_end = 0U;

    	                	  HAL_StatusTypeDef home_wait_status =
    	                	      Servo_WaitForPosition(
    	                	          active_servo_id,
    	                	          active_home_position,
    	                	          20U,
    	                	          3000U,
    	                	          &home_end
    	                	      );

    	                	  int32_t home_error =
    	                	      Servo_PositionError(
    	                	          home_end,
    	                	          active_home_position
    	                	      );

    	                	  const char *home_result =
    	                	      (home_wait_status == HAL_OK) ?
    	                	      "HOME_READY" :
    	                	      "HOME_TIMEOUT";

    	                	  line_length = snprintf(
    	                	      status_line,
    	                	      sizeof(status_line),
    	                	      "HOME_END=%u ERROR=%ld %s\r\n",
    	                	      (unsigned int)home_end,
    	                	      (long)home_error,
    	                	      home_result
    	                	  );

    	                	  if (line_length > 0)
    	                	  {
    	                	      (void)HAL_UART_Transmit(
    	                	          app_host_uart,
    	                	          (uint8_t *)status_line,
    	                	          (uint16_t)line_length,
    	                	          100U
    	                	      );
    	                	  }

    	                	  if (home_wait_status == HAL_OK)
    	                	  {
    	                	      return_available = 0U;
    	                	      move_already_done = 0U;
    	                	  }
    	                  }
    	                  else
    	                  {
    	                      static const uint8_t home_move_fail[] =
    	                          "HOME_MOVE_FAIL\r\n";

	                      (void)HAL_UART_Transmit(
	                          app_host_uart,
	                          (uint8_t *)home_move_fail,
	                          sizeof(home_move_fail) - 1U,
	                          100U
	                      );
	                      SingleArmApp_ReportMotionSafety();
    	                  }
    	              }
    	          }
    	          else
    	          {
    	              static const uint8_t home_read_fail[] =
    	                  "HOME_POSITION_READ_FAIL\r\n";

    	              (void)HAL_UART_Transmit(
    	                  app_host_uart,
    	                  (uint8_t *)home_read_fail,
    	                  sizeof(home_read_fail) - 1U,
    	                  100U
    	              );
    	          }
    	      }
    	  }
          else if ((rx_byte == 'R') || (rx_byte == 'r'))
          {
              if (system_ready == 0U)
              {
                  static const uint8_t not_ready[] =
                      "TRAJ_NOT_READY\r\n";

                  (void)HAL_UART_Transmit(
                      app_host_uart,
                      (uint8_t *)not_ready,
                      sizeof(not_ready) - 1U,
                      100U
                  );
              }
              else if (return_available == 0U)
              {
                  static const uint8_t no_return[] =
                      "RETURN_NOT_AVAILABLE_SEND_M\r\n";

                  (void)HAL_UART_Transmit(
                      app_host_uart,
                      (uint8_t *)no_return,
                      sizeof(no_return) - 1U,
                      100U
                  );
              }
              else
              {
                  uint16_t return_start = 0U;

                  if (Servo_ReadPosition(
                		  active_servo_id,
                          &return_start
                      ) == HAL_OK)
                  {
                      int line_length = snprintf(
                          status_line,
                          sizeof(status_line),
						  "RETURN_START ID=%u NAME=%s POS=%u TARGET=%u DURATION=%luMS\r\n",
						  (unsigned int)active_servo_id,
						  active_joint_name,
						  (unsigned int)return_start,
						  (unsigned int)return_position,
						  (unsigned long)active_duration_ms
                      );

                      if (line_length > 0)
                      {
                          (void)HAL_UART_Transmit(
                              app_host_uart,
                              (uint8_t *)status_line,
                              (uint16_t)line_length,
                              100U
                          );
                      }

                      if (Servo_RunSmoothstep(
                    		  active_servo_id,
                              return_start,
                              return_position,
							  active_duration_ms
                          ) == HAL_OK)
                      {
                          HAL_Delay(500U);

                          uint16_t return_end = 0U;

                          if (Servo_ReadPosition(
                        		  active_servo_id,
                                  &return_end
                              ) == HAL_OK)
                          {
                              int32_t return_error =
                                  (int32_t)return_end -
                                  (int32_t)return_position;

                              line_length = snprintf(
                                  status_line,
                                  sizeof(status_line),
                                  "RETURN_END=%u ERROR=%ld CYCLE_READY\r\n",
                                  (unsigned int)return_end,
                                  (long)return_error
                              );

                              if (line_length > 0)
                              {
                                  (void)HAL_UART_Transmit(
                                      app_host_uart,
                                      (uint8_t *)status_line,
                                      (uint16_t)line_length,
                                      100U
                                  );
                              }

                              return_available = 0U;
                              move_already_done = 0U;
                          }
                          else
                          {
                              static const uint8_t return_read_fail[] =
                                  "RETURN_END_READ_FAIL\r\n";

                              (void)HAL_UART_Transmit(
                                  app_host_uart,
                                  (uint8_t *)return_read_fail,
                                  sizeof(return_read_fail) - 1U,
                                  100U
                              );
                          }
                      }
                      else
                      {
                          static const uint8_t return_write_fail[] =
                              "RETURN_WRITE_FAIL\r\n";

                          (void)HAL_UART_Transmit(
                              app_host_uart,
                              (uint8_t *)return_write_fail,
                              sizeof(return_write_fail) - 1U,
                              100U
                          );
                          SingleArmApp_ReportMotionSafety();
                      }
                  }
                  else
                  {
                      static const uint8_t return_start_fail[] =
                          "RETURN_START_READ_FAIL\r\n";

                      (void)HAL_UART_Transmit(
                          app_host_uart,
                          (uint8_t *)return_start_fail,
                          sizeof(return_start_fail) - 1U,
                          100U
                      );
                  }
              }
          }
      }
    else if ((host_receive_status == HAL_ERROR) &&
             (BinaryControl_IsBinaryMode() != 0U))
    {
        BinaryControl_HandleHostUartError();
    }
}
