cmake_minimum_required(VERSION 3.20)

project(stm32_g474_single_arm LANGUAGES C ASM)

option(
    F25_HOST_LINK_CANDIDATE
    "Build the 921600-baud, validation-only F2.5 entry-gate candidate"
    OFF
)
option(
    PROTOCOL_V2_VALIDATION_CANDIDATE
    "Build protocol v2 framing/session validation with all motion disabled"
    OFF
)
option(
    PROTOCOL_V2_EXECUTOR_CANDIDATE
    "Build protocol v2 executor state validation with all servo output disabled"
    OFF
)
option(
    PROTOCOL_V2_SHADOW_CANDIDATE
    "Build real-feedback protocol v2 shadow validation with no goal output"
    OFF
)
option(
    PROTOCOL_V2_UNWRAP_SHADOW_CANDIDATE
    "Build explicit-branch wrap-aware v2 shadow validation with no output"
    OFF
)
option(
    PROTOCOL_V2_J1_LIMITS_SHADOW_CANDIDATE
    "Build approved J1-L arm-limit parity validation with no output"
    OFF
)
option(
    BIMANUAL_OPERATIONAL_LIMITS_CANDIDATE
    "Build R4 services with the operator-verified full bimanual limits"
    OFF
)
option(
    BIMANUAL_DISPATCH_REFACTOR_CANDIDATE
    "Build the source-agnostic bimanual goal-map and shared packet refactor"
    OFF
)
option(
    BIMANUAL_DMA_DISPATCH_CANDIDATE
    "Build protocol-v2 12-axis dual-UART DMA dispatch"
    OFF
)
option(
    BIMANUAL_DMA_FAULT_INJECTION_CANDIDATE
    "Build one-shot right-DMA rejection with verified coordinated stop"
    OFF
)
option(
    BIMANUAL_TRACKING_FEEDBACK_CANDIDATE
    "Build asynchronous 12-axis route-time tracking feedback"
    OFF
)
option(
    BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE
    "Build one-shot route-time tracking-error with verified coordinated stop"
    OFF
)
option(
    BIMANUAL_RESIDENT_FINITE_CANDIDATE
    "Build resident finite-leg tracking finalization and re-entry"
    OFF
)
option(
    BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE
    "Build resident 12-axis measured feedback snapshots with sample age"
    OFF
)
option(
    BIMANUAL_TERMINAL_SETTLE_CANDIDATE
    "Build two-sweep measured terminal settling for resident finite legs"
    OFF
)
option(
    BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE
    "Build per-axis terminal settling with contact-tolerant grippers"
    OFF
)

set(VALIDATION_CANDIDATE_COUNT 0)
foreach(CANDIDATE_OPTION
    F25_HOST_LINK_CANDIDATE
    PROTOCOL_V2_VALIDATION_CANDIDATE
    PROTOCOL_V2_EXECUTOR_CANDIDATE
    PROTOCOL_V2_SHADOW_CANDIDATE
    PROTOCOL_V2_UNWRAP_SHADOW_CANDIDATE
    PROTOCOL_V2_J1_LIMITS_SHADOW_CANDIDATE
    BIMANUAL_OPERATIONAL_LIMITS_CANDIDATE
    BIMANUAL_DISPATCH_REFACTOR_CANDIDATE
    BIMANUAL_DMA_DISPATCH_CANDIDATE
    BIMANUAL_DMA_FAULT_INJECTION_CANDIDATE
    BIMANUAL_TRACKING_FEEDBACK_CANDIDATE
    BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE
    BIMANUAL_RESIDENT_FINITE_CANDIDATE
    BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE
    BIMANUAL_TERMINAL_SETTLE_CANDIDATE
    BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE
)
    if(${CANDIDATE_OPTION})
        math(EXPR VALIDATION_CANDIDATE_COUNT
            "${VALIDATION_CANDIDATE_COUNT}+1"
        )
    endif()
endforeach()
if(VALIDATION_CANDIDATE_COUNT GREATER 1)
    message(FATAL_ERROR "Select only one validation candidate")
endif()

set(TARGET_NAME stm32_g474_single_arm)
set(CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Core")
set(DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers")
set(ACTUATOR_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../stm32_actuator")

set(CORE_SOURCES
    "${CORE_DIR}/Src/binary_control.c"
    "${CORE_DIR}/Src/control_tick.c"
    "${CORE_DIR}/Src/f0_metrics.c"
    "${CORE_DIR}/Src/host_uart_rx.c"
    "${CORE_DIR}/Src/host_uart_tx.c"
    "${CORE_DIR}/Src/main.c"
    "${CORE_DIR}/Src/servo_bus.c"
    "${CORE_DIR}/Src/right_servo_bus.c"
    "${CORE_DIR}/Src/servo_response_parser.c"
    "${CORE_DIR}/Src/servo_rx_window.c"
    "${CORE_DIR}/Src/single_arm_app.c"
    "${CORE_DIR}/Src/stm32g4xx_hal_msp.c"
    "${CORE_DIR}/Src/stm32g4xx_it.c"
    "${CORE_DIR}/Src/syscalls.c"
    "${CORE_DIR}/Src/sysmem.c"
    "${CORE_DIR}/Src/system_stm32g4xx.c"
    "${CORE_DIR}/Src/timebase.c"
    "${CORE_DIR}/Startup/startup_stm32g474retx.s"
)

set(HAL_SOURCES
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c"
    "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c"
)

set(ACTUATOR_SOURCES
    "${ACTUATOR_CORE_DIR}/src/buffered_command_route.c"
    "${ACTUATOR_CORE_DIR}/src/buffered_executor.c"
    "${ACTUATOR_CORE_DIR}/src/calibration.c"
    "${ACTUATOR_CORE_DIR}/src/cobs.c"
    "${ACTUATOR_CORE_DIR}/src/crc32c.c"
    "${ACTUATOR_CORE_DIR}/src/joint_unwrap.c"
    "${ACTUATOR_CORE_DIR}/src/protocol.c"
    "${ACTUATOR_CORE_DIR}/src/safety.c"
    "${ACTUATOR_CORE_DIR}/src/setpoint_queue.c"
)

add_executable(
    "${TARGET_NAME}"
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${ACTUATOR_SOURCES}
)

target_include_directories(
    "${TARGET_NAME}"
    PRIVATE
        "${CORE_DIR}/Inc"
        "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Inc"
        "${DRIVER_DIR}/STM32G4xx_HAL_Driver/Inc/Legacy"
        "${DRIVER_DIR}/BSP/STM32G4xx_Nucleo"
        "${DRIVER_DIR}/CMSIS/Device/ST/STM32G4xx/Include"
        "${DRIVER_DIR}/CMSIS/Include"
        "${ACTUATOR_CORE_DIR}/include"
)

target_compile_definitions(
    "${TARGET_NAME}"
    PRIVATE
        USE_NUCLEO_64
        USE_HAL_DRIVER
        STM32G474xx
)

if(BIMANUAL_OPERATIONAL_LIMITS_CANDIDATE)
    # The existing one-servo +/-20 raw service remains bounded, but target
    # admission uses the single operator-verified table shared with v2.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${CORE_DIR}/Src/bimanual_operational_limits.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            HOST_BINARY_FIRMWARE_VERSION=0x00024400UL
            HOST_BINARY_CAPABILITIES=0x207FFFFFUL
            HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD=1U
    )
endif()

if(BIMANUAL_DISPATCH_REFACTOR_CANDIDATE)
    # F7-A is a behavior-preserving packet/goal-map refactor. The general
    # 12-axis executor output remains disconnected; existing v1 motion APIs
    # are retained only to perform the left-arm regression probe.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${CORE_DIR}/Src/bimanual_operational_limits.c"
            "${ACTUATOR_CORE_DIR}/src/bimanual_goal_map.c"
            "${ACTUATOR_CORE_DIR}/src/sts3215_packet.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            HOST_BINARY_FIRMWARE_VERSION=0x00024500UL
            HOST_BINARY_CAPABILITIES=0x607FFFFFUL
            HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD=1U
            HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U
    )
endif()

if(BIMANUAL_DMA_DISPATCH_CANDIDATE OR
   BIMANUAL_DMA_FAULT_INJECTION_CANDIDATE OR
   BIMANUAL_TRACKING_FEEDBACK_CANDIDATE OR
   BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE OR
   BIMANUAL_RESIDENT_FINITE_CANDIDATE OR
   BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE OR
   BIMANUAL_TERMINAL_SETTLE_CANDIDATE OR
   BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE)
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${CORE_DIR}/Src/bimanual_operational_limits.c"
            "${CORE_DIR}/Src/bimanual_servo_dispatch.c"
            "${ACTUATOR_CORE_DIR}/src/bimanual_dispatch.c"
            "${ACTUATOR_CORE_DIR}/src/bimanual_goal_map.c"
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_executor_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
            "${ACTUATOR_CORE_DIR}/src/sts3215_packet.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES=1U
            ACTUATOR_ENABLE_BIMANUAL_DISPATCH_MESSAGES=1U
            HOST_BINARY_CAPABILITIES=0xEFFFFFFFUL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U
            HOST_BIMANUAL_OPERATIONAL_LIMITS_BUILD=1U
            HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U
            HOST_BIMANUAL_DMA_DISPATCH_BUILD=1U
    )
    if(BIMANUAL_DMA_FAULT_INJECTION_CANDIDATE)
        target_compile_definitions(
            "${TARGET_NAME}"
            PRIVATE
                HOST_BINARY_FIRMWARE_VERSION=0x00024605UL
                HOST_BIMANUAL_DMA_FAULT_INJECTION_BUILD=1U
        )
    elseif(BIMANUAL_TRACKING_FEEDBACK_CANDIDATE OR
           BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE OR
           BIMANUAL_RESIDENT_FINITE_CANDIDATE OR
           BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE OR
           BIMANUAL_TERMINAL_SETTLE_CANDIDATE OR
           BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE)
        target_sources(
            "${TARGET_NAME}"
            PRIVATE
                "${CORE_DIR}/Src/bimanual_tracking_feedback.c"
        )
        target_compile_definitions(
            "${TARGET_NAME}"
            PRIVATE
                HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U
                ACTUATOR_ENABLE_BIMANUAL_TRACKING_MESSAGES=1U
        )
        if(BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE OR
           BIMANUAL_TERMINAL_SETTLE_CANDIDATE OR
           BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE)
            target_sources(
                "${TARGET_NAME}"
                PRIVATE
                    "${CORE_DIR}/Src/bimanual_feedback_snapshot.c"
            )
            if(BIMANUAL_GRIPPER_TERMINAL_SETTLE_CANDIDATE)
                target_compile_definitions(
                    "${TARGET_NAME}"
                    PRIVATE
                        HOST_BINARY_FIRMWARE_VERSION=0x00024809UL
                        HOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U
                        HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U
                        HOST_BIMANUAL_TERMINAL_SETTLE_BUILD=1U
                        HOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD=1U
                        HOST_SERVO_DISABLE_READBACK_RECOVERY_BUILD=1U
                        ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES=1U
                )
            elseif(BIMANUAL_TERMINAL_SETTLE_CANDIDATE)
                target_compile_definitions(
                    "${TARGET_NAME}"
                    PRIVATE
                        HOST_BINARY_FIRMWARE_VERSION=0x00024806UL
                        HOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U
                        HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U
                        HOST_BIMANUAL_TERMINAL_SETTLE_BUILD=1U
                        ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES=1U
                )
            else()
                target_compile_definitions(
                    "${TARGET_NAME}"
                    PRIVATE
                        HOST_BINARY_FIRMWARE_VERSION=0x00024800UL
                        HOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U
                        HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U
                        ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES=1U
                )
            endif()
        elseif(BIMANUAL_TRACKING_FAULT_INJECTION_CANDIDATE)
            target_compile_definitions(
                "${TARGET_NAME}"
                PRIVATE
                    HOST_BINARY_FIRMWARE_VERSION=0x00024701UL
                    HOST_BIMANUAL_TRACKING_FAULT_INJECTION_BUILD=1U
            )
        elseif(BIMANUAL_RESIDENT_FINITE_CANDIDATE)
            target_compile_definitions(
                "${TARGET_NAME}"
                PRIVATE
                    HOST_BINARY_FIRMWARE_VERSION=0x00024703UL
                    HOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U
            )
        else()
            target_compile_definitions(
                "${TARGET_NAME}"
                PRIVATE
                    HOST_BINARY_FIRMWARE_VERSION=0x00024700UL
            )
        endif()
    else()
        target_compile_definitions(
            "${TARGET_NAME}"
            PRIVATE
                HOST_BINARY_FIRMWARE_VERSION=0x00024604UL
        )
    endif()
endif()

if(F25_HOST_LINK_CANDIDATE)
    # Keep protocol v1 for an isolated host-link measurement, but advertise
    # only validation and asynchronous host-TX support.  The unique identity
    # is not accepted by the normal ROS bridge, and the firmware router also
    # rejects every torque/motion-producing request.
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            HOST_BINARY_FIRMWARE_VERSION=0x00023C01UL
            HOST_BINARY_CAPABILITIES=0x00802408UL
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
    )
endif()

if(PROTOCOL_V2_J1_LIMITS_SHADOW_CANDIDATE)
    # J1-W branch binding remains active, but executor admission now uses the
    # operator-approved 64-raw contracted arm limits. Gripper slots stay
    # synthetic and every output remains discarded.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${CORE_DIR}/Src/bimanual_operational_limits.c"
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_executor_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES=1U
            HOST_BINARY_FIRMWARE_VERSION=0x00024100UL
            HOST_BINARY_CAPABILITIES=0x0F802000UL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD=1U
    )
endif()

if(PROTOCOL_V2_UNWRAP_SHADOW_CANDIDATE)
    # The host must bind every 12-bit raw sample to an explicit unwrapped
    # branch reference before a real-feedback anchor is accepted. Both buses
    # remain verified torque-off and every executor output is discarded.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_executor_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES=1U
            HOST_BINARY_FIRMWARE_VERSION=0x00024000UL
            HOST_BINARY_CAPABILITIES=0x0F802000UL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U
    )
endif()

if(PROTOCOL_V2_EXECUTOR_CANDIDATE)
    # The executor consumes real v2 UART frames and advances its queue on a
    # 5 ms logical clock, but its 12-joint output is discarded. ARM, torque,
    # and both servo-bus write paths remain compile-time rejected.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_executor_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U
            HOST_BINARY_FIRMWARE_VERSION=0x00023E00UL
            HOST_BINARY_CAPABILITIES=0x03802000UL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U
    )
endif()

if(PROTOCOL_V2_SHADOW_CANDIDATE)
    # Both buses are verified torque-off and sampled to form a real 12-joint
    # anchor.  Executor outputs remain discarded; no goal-position path is
    # compiled into this candidate route.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_executor_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_EXECUTOR_MESSAGES=1U
            ACTUATOR_ENABLE_STREAM_V2_SHADOW_MESSAGES=1U
            HOST_BINARY_FIRMWARE_VERSION=0x00023F01UL
            HOST_BINARY_CAPABILITIES=0x07802000UL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_EXECUTOR_VALIDATION_BUILD=1U
            HOST_PROTOCOL_V2_SHADOW_VALIDATION_BUILD=1U
    )
endif()

if(PROTOCOL_V2_VALIDATION_CANDIDATE)
    # This candidate proves protocol-v2 framing and stream-state semantics
    # with both servo 12 V domains physically off. It cannot arm, enable,
    # enqueue an executable setpoint, or write either servo bus.
    target_sources(
        "${TARGET_NAME}"
        PRIVATE
            "${ACTUATOR_CORE_DIR}/src/stream_contract_v2.c"
            "${ACTUATOR_CORE_DIR}/src/stream_session_v2.c"
    )
    target_compile_definitions(
        "${TARGET_NAME}"
        PRIVATE
            ACTUATOR_PROTOCOL_VERSION=2U
            ACTUATOR_ENABLE_STREAM_V2_MESSAGES=1U
            HOST_BINARY_FIRMWARE_VERSION=0x00023D00UL
            HOST_BINARY_CAPABILITIES=0x01802000UL
            HOST_BINARY_JOINT_COUNT=12U
            HOST_BINARY_UART_BAUD=921600UL
            HOST_F25_VALIDATION_ONLY_BUILD=1U
            HOST_PROTOCOL_V2_VALIDATION_ONLY_BUILD=1U
    )
endif()

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(
    "${TARGET_NAME}"
    PRIVATE
        ${MCU_FLAGS}
        -ffunction-sections
        -fdata-sections
        -fstack-usage
        -Wall
        -Wextra
        -Wpedantic
        $<$<CONFIG:Debug>:-Og>
        $<$<CONFIG:Debug>:-g3>
        $<$<CONFIG:Release>:-Os>
)

target_link_options(
    "${TARGET_NAME}"
    PRIVATE
        ${MCU_FLAGS}
        "-T${CMAKE_CURRENT_SOURCE_DIR}/STM32G474RETX_FLASH.ld"
        --specs=nano.specs
        --specs=nosys.specs
        -Wl,--gc-sections
        "-Wl,-Map=${TARGET_NAME}.map"
        -Wl,--start-group
        -lc
        -lm
        -Wl,--end-group
)

set_target_properties(
    "${TARGET_NAME}"
    PROPERTIES
        SUFFIX ".elf"
)

add_custom_command(
    TARGET "${TARGET_NAME}"
    POST_BUILD
    COMMAND
        "${CMAKE_OBJCOPY}" -O ihex
        "$<TARGET_FILE:${TARGET_NAME}>"
        "${TARGET_NAME}.hex"
    COMMAND
        "${CMAKE_OBJCOPY}" -O binary
        "$<TARGET_FILE:${TARGET_NAME}>"
        "${TARGET_NAME}.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:${TARGET_NAME}>"
    VERBATIM
)
