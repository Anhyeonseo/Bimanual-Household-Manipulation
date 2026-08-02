cmake_minimum_required(VERSION 3.20)

project(stm32_g474_single_arm LANGUAGES C ASM)

set(TARGET_NAME stm32_g474_single_arm)
set(CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Core")
set(DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers")
set(ACTUATOR_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../stm32_actuator")

set(CORE_SOURCES
    "${CORE_DIR}/Src/binary_control.c"
    "${CORE_DIR}/Src/host_uart_rx.c"
    "${CORE_DIR}/Src/main.c"
    "${CORE_DIR}/Src/servo_bus.c"
    "${CORE_DIR}/Src/servo_response_parser.c"
    "${CORE_DIR}/Src/single_arm_app.c"
    "${CORE_DIR}/Src/stm32g4xx_hal_msp.c"
    "${CORE_DIR}/Src/stm32g4xx_it.c"
    "${CORE_DIR}/Src/syscalls.c"
    "${CORE_DIR}/Src/sysmem.c"
    "${CORE_DIR}/Src/system_stm32g4xx.c"
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
    "${ACTUATOR_CORE_DIR}/src/buffered_executor.c"
    "${ACTUATOR_CORE_DIR}/src/calibration.c"
    "${ACTUATOR_CORE_DIR}/src/cobs.c"
    "${ACTUATOR_CORE_DIR}/src/crc32c.c"
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
