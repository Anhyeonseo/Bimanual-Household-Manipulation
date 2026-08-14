cmake_minimum_required(VERSION 3.20)
project(stm32_actuator_core LANGUAGES C)

add_library(actuator_core STATIC
    src/bimanual_dispatch.c
    src/bimanual_goal_map.c
    src/buffered_command_route.c
    src/buffered_executor.c
    src/calibration.c
    src/cobs.c
    src/crc32c.c
    src/joint_unwrap.c
    src/protocol.c
    src/safety.c
    src/stream_contract_v2.c
    src/stream_executor_v2.c
    src/stream_session_v2.c
    src/sts3215_packet.c
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
    add_executable(
        actuator_buffered_command_route_tests
        tests/test_buffered_command_route.c
    )
    target_link_libraries(actuator_buffered_command_route_tests PRIVATE actuator_core)
    add_executable(
        actuator_joint_unwrap_tests
        tests/test_joint_unwrap.c
    )
    target_link_libraries(actuator_joint_unwrap_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_contract_v2_tests
        tests/test_stream_contract_v2.c
    )
    target_link_libraries(actuator_stream_contract_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_executor_v2_tests
        tests/test_stream_executor_v2.c
    )
    target_link_libraries(actuator_stream_executor_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_session_v2_tests
        tests/test_stream_session_v2.c
    )
    target_link_libraries(actuator_stream_session_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_sts3215_packet_tests
        tests/test_sts3215_packet.c
    )
    target_link_libraries(actuator_sts3215_packet_tests PRIVATE actuator_core)
    add_executable(
        actuator_bimanual_dispatch_tests
        tests/test_bimanual_dispatch.c
    )
    target_link_libraries(actuator_bimanual_dispatch_tests PRIVATE actuator_core)
    add_executable(
        actuator_bimanual_goal_map_tests
        tests/test_bimanual_goal_map.c
    )
    target_link_libraries(actuator_bimanual_goal_map_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_core_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_buffered_command_route_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_joint_unwrap_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_contract_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_executor_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_session_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_sts3215_packet_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_bimanual_dispatch_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_bimanual_goal_map_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_buffered_command_route_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_joint_unwrap_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_contract_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_executor_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_session_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_sts3215_packet_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_bimanual_dispatch_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_bimanual_goal_map_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_core_tests COMMAND actuator_core_tests)
    add_test(NAME actuator_buffered_command_route_tests COMMAND actuator_buffered_command_route_tests)
    add_test(NAME actuator_joint_unwrap_tests COMMAND actuator_joint_unwrap_tests)
    add_test(NAME actuator_stream_contract_v2_tests COMMAND actuator_stream_contract_v2_tests)
    add_test(NAME actuator_stream_executor_v2_tests COMMAND actuator_stream_executor_v2_tests)
    add_test(NAME actuator_stream_session_v2_tests COMMAND actuator_stream_session_v2_tests)
    add_test(NAME actuator_sts3215_packet_tests COMMAND actuator_sts3215_packet_tests)
    add_test(NAME actuator_bimanual_dispatch_tests COMMAND actuator_bimanual_dispatch_tests)
    add_test(NAME actuator_bimanual_goal_map_tests COMMAND actuator_bimanual_goal_map_tests)
endif()
