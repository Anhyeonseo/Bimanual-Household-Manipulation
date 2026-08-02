cmake_minimum_required(VERSION 3.20)
project(stm32_actuator_core LANGUAGES C)

add_library(actuator_core STATIC
    src/buffered_executor.c
    src/calibration.c
    src/cobs.c
    src/crc32c.c
    src/protocol.c
    src/safety.c
    src/setpoint_queue.c
)
target_include_directories(actuator_core PUBLIC include)
target_compile_features(actuator_core PUBLIC c_std_11)

if(MSVC)
    target_compile_options(actuator_core PRIVATE /W4 /WX)
else()
    target_compile_options(actuator_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(actuator_core_tests tests/test_main.c)
    target_link_libraries(actuator_core_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_core_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_core_tests COMMAND actuator_core_tests)
endif()
