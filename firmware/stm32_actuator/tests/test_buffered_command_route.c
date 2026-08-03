#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/buffered_command_route.h"

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void write_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8u);
}
static void write_u32_le(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8u);
    p[2] = (uint8_t)(value >> 16u); p[3] = (uint8_t)(value >> 24u);
}
static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}
static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static size_t make_payload(uint8_t *payload, uint32_t first_tick,
                           const uint32_t *offsets, const int32_t *positions,
                           uint8_t sample_count) {
    size_t sample;
    const size_t length = ACTUATOR_BUFFERED_WIRE_HEADER_SIZE +
        ((size_t)sample_count * ACTUATOR_BUFFERED_WIRE_SAMPLE_SIZE);
    memset(payload, 0, length);
    write_u32_le(payload, first_tick);
    payload[4] = sample_count; payload[5] = 1u; write_u16_le(&payload[6], 0u);
    for (sample = 0u; sample < sample_count; ++sample) {
        const size_t base = ACTUATOR_BUFFERED_WIRE_HEADER_SIZE +
            sample * ACTUATOR_BUFFERED_WIRE_SAMPLE_SIZE;
        size_t joint;
        write_u32_le(&payload[base], offsets[sample]);
        for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
            write_u32_le(&payload[base + 4u + joint * 4u],
                         (uint32_t)positions[sample]);
        }
    }
    return length;
}

static void fill_limits(actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    size_t joint;
    for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
        limits[joint].minimum_urad = -1000;
        limits[joint].maximum_urad = 1000;
    }
}

static void test_decode_rejects_wire_faults(void) {
    uint8_t payload[8u + 2u * 52u];
    const uint32_t offsets[2] = {0u, 10u};
    const int32_t positions[2] = {100, 200};
    actuator_buffered_command_t command;
    const size_t length = make_payload(payload, 10u, offsets, positions, 2u);
    const uint16_t flags = ACTUATOR_BUFFERED_FLAG_CANDIDATE |
        ACTUATOR_BUFFERED_FLAG_BEGIN | ACTUATOR_BUFFERED_FLAG_START |
        ACTUATOR_BUFFERED_FLAG_END;
    CHECK(actuator_buffered_command_decode(payload, length, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(command.samples[1].apply_tick == 20u);
    CHECK(actuator_buffered_command_decode(payload, length, 0u, &command) ==
          ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS);
    CHECK(actuator_buffered_command_decode(payload, length - 1u, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH);
    payload[5] = 2u;
    CHECK(actuator_buffered_command_decode(payload, length, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_INVALID_ARM_MASK);
    payload[5] = 1u; payload[6] = 1u;
    CHECK(actuator_buffered_command_decode(payload, length, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_INVALID_RESERVED);
    payload[6] = 0u;
    write_u32_le(&payload[8u + 52u], 0u);
    CHECK(actuator_buffered_command_decode(payload, length, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_NON_MONOTONIC_TICK);
    write_u32_le(&payload[8u + 52u], 10u);
    write_u32_le(&payload[8u + 28u], 1u);
    CHECK(actuator_buffered_command_decode(payload, length, flags, &command) ==
          ACTUATOR_BUFFERED_COMMAND_UNSUPPORTED_RIGHT_SLOT);
}

static void test_route_runs_and_encodes_status(void) {
    uint8_t payload[8u + 2u * 52u];
    uint8_t status[32];
    const uint32_t offsets[2] = {0u, 10u};
    const int32_t positions[2] = {100, 200};
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];
    actuator_buffered_command_t command;
    actuator_buffered_command_route_t route;
    int32_t anchor[ACTUATOR_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_JOINT_COUNT];
    size_t status_length = 0u;
    const uint16_t flags = ACTUATOR_BUFFERED_FLAG_CANDIDATE |
        ACTUATOR_BUFFERED_FLAG_BEGIN | ACTUATOR_BUFFERED_FLAG_START |
        ACTUATOR_BUFFERED_FLAG_END;
    fill_limits(limits);
    CHECK(actuator_buffered_command_decode(
        payload, make_payload(payload, 10u, offsets, positions, 2u),
        flags, &command) == ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_init(&route, 2u, limits) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_admit(&route, &command, 42u, 0u, 5u, 100u) ==
          ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_start(&route, 0u, anchor) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_step(&route, 5u, output) ==
          ACTUATOR_BUFFERED_OUTPUT && output[0] == 50);
    CHECK(actuator_buffered_command_route_step(&route, 10u, output) ==
          ACTUATOR_BUFFERED_OUTPUT && output[0] == 100);
    CHECK(actuator_buffered_command_route_step(&route, 15u, output) ==
          ACTUATOR_BUFFERED_OUTPUT && output[0] == 150);
    CHECK(actuator_buffered_command_route_step(&route, 20u, output) ==
          ACTUATOR_BUFFERED_OUTPUT);
    CHECK(route.executor.diagnostics.state == ACTUATOR_BUFFERED_SUCCEEDED);
    CHECK(actuator_buffered_status_encode(
        status, sizeof(status), &status_length, 6u, 2u, 3u, 4u, 42u, 20u,
        UINT32_C(0x8AD27897), &route.executor.diagnostics));
    CHECK(status_length == 32u && status[16] == ACTUATOR_BUFFERED_SUCCEEDED);
    CHECK(read_u16_le(&status[22]) == 2u);
    CHECK(read_u32_le(&status[24]) == 2u && read_u32_le(&status[28]) == 2u);
}

static void test_validation_refill_and_cancel_are_terminal(void) {
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];
    actuator_buffered_command_route_t route;
    actuator_buffered_command_t first;
    actuator_buffered_command_t validation;
    actuator_setpoint_queue_t queue_before_validation;
    uint8_t payload[8u + 2u * 52u];
    const uint32_t offsets[2] = {0u, 10u};
    const int32_t positions[2] = {100, 200};
    fill_limits(limits);
    CHECK(actuator_buffered_command_decode(
        payload, make_payload(payload, 10u, offsets, positions, 2u),
        ACTUATOR_BUFFERED_FLAG_CANDIDATE | ACTUATOR_BUFFERED_FLAG_BEGIN,
        &first) == ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_init(&route, 1u, limits) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_admit(&route, &first, 1u, 0u, 1u, 100u) ==
          ACTUATOR_BUFFERED_COMMAND_OK);
    validation = first;
    validation.flags = ACTUATOR_BUFFERED_FLAG_CANDIDATE |
        ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY;
    validation.sample_count = 1u;
    validation.samples[0].apply_tick = 30u;
    queue_before_validation = route.executor.queue;
    CHECK(actuator_buffered_command_route_admit(
        &route, &validation, 2u, 0u, 1u, 100u) == ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(memcmp(
        &route.executor.queue,
        &queue_before_validation,
        sizeof(queue_before_validation)) == 0);
    CHECK(route.executor.diagnostics.accepted_samples == 2u);
    CHECK(actuator_buffered_command_route_cancel(&route, 3u) ==
          ACTUATOR_BUFFERED_TERMINAL);
    CHECK(route.executor.diagnostics.state == ACTUATOR_BUFFERED_CANCELED);
    CHECK(route.executor.diagnostics.safe_stop_required);
    CHECK(actuator_buffered_command_route_admit(&route, &first, 4u, 0u, 1u, 100u) ==
          ACTUATOR_BUFFERED_COMMAND_BAD_STATE);
}

static void test_queue_underflow_requires_safe_stop(void) {
    uint8_t payload[8u + 2u * 52u];
    const uint32_t offsets[2] = {0u, 10u};
    const int32_t positions[2] = {100, 200};
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];
    actuator_buffered_command_t command;
    actuator_buffered_command_route_t route;
    int32_t anchor[ACTUATOR_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_JOINT_COUNT];
    const uint16_t flags = ACTUATOR_BUFFERED_FLAG_CANDIDATE |
        ACTUATOR_BUFFERED_FLAG_BEGIN | ACTUATOR_BUFFERED_FLAG_START;

    fill_limits(limits);
    CHECK(actuator_buffered_command_decode(
        payload, make_payload(payload, 10u, offsets, positions, 2u),
        flags, &command) == ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_init(&route, 2u, limits) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_admit(
        &route, &command, 7u, 0u, 5u, 100u) ==
          ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_start(&route, 0u, anchor) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_step(&route, 10u, output) ==
          ACTUATOR_BUFFERED_OUTPUT);
    CHECK(actuator_buffered_command_route_step(&route, 20u, output) ==
          ACTUATOR_BUFFERED_OUTPUT);
    CHECK(route.executor.diagnostics.state == ACTUATOR_BUFFERED_HOLD);
    CHECK(route.executor.diagnostics.reason ==
          ACTUATOR_BUFFERED_REASON_QUEUE_UNDERFLOW);
    CHECK(route.executor.diagnostics.safe_stop_required);
    CHECK(route.executor.diagnostics.queued_samples == 0u);
    CHECK(route.executor.diagnostics.applied_samples == 2u);
}

static void test_missed_apply_tick_requires_safe_stop(void) {
    uint8_t payload[8u + 2u * 52u];
    const uint32_t offsets[2] = {0u, 10u};
    const int32_t positions[2] = {100, 200};
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];
    actuator_buffered_command_t command;
    actuator_buffered_command_route_t route;
    int32_t anchor[ACTUATOR_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_JOINT_COUNT];
    const uint16_t flags = ACTUATOR_BUFFERED_FLAG_CANDIDATE |
        ACTUATOR_BUFFERED_FLAG_BEGIN | ACTUATOR_BUFFERED_FLAG_START |
        ACTUATOR_BUFFERED_FLAG_END;

    fill_limits(limits);
    CHECK(actuator_buffered_command_decode(
        payload, make_payload(payload, 10u, offsets, positions, 2u),
        flags, &command) == ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_init(&route, 2u, limits) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_admit(
        &route, &command, 8u, 0u, 5u, 100u) ==
          ACTUATOR_BUFFERED_COMMAND_OK);
    CHECK(actuator_buffered_command_route_start(&route, 0u, anchor) ==
          ACTUATOR_BUFFERED_OK);
    CHECK(actuator_buffered_command_route_step(&route, 11u, output) ==
          ACTUATOR_BUFFERED_TERMINAL);
    CHECK(route.executor.diagnostics.state == ACTUATOR_BUFFERED_HOLD);
    CHECK(route.executor.diagnostics.reason ==
          ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK);
    CHECK(route.executor.diagnostics.safe_stop_required);
    CHECK(route.executor.diagnostics.queued_samples == 0u);
    CHECK(route.executor.diagnostics.applied_samples == 0u);
}

static void run_test(const char *name, void (*test)(void)) {
    const int before = failures;
    test();
    if (failures == before) printf("PASS %s\n", name);
}

int main(void) {
    run_test("decode_rejects_wire_faults", test_decode_rejects_wire_faults);
    run_test("route_runs_and_encodes_status", test_route_runs_and_encodes_status);
    run_test("validation_refill_and_cancel_are_terminal",
             test_validation_refill_and_cancel_are_terminal);
    run_test("queue_underflow_requires_safe_stop",
             test_queue_underflow_requires_safe_stop);
    run_test("missed_apply_tick_requires_safe_stop",
             test_missed_apply_tick_requires_safe_stop);
    if (failures != 0) return 1;
    printf("All buffered command route tests passed.\n");
    return 0;
}
