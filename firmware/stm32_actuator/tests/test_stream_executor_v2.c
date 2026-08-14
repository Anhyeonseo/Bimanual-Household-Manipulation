#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/stream_executor_v2.h"

static int failures = 0;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void write_u16_le(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8u);
}

static void write_u32_le(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8u);
    destination[2] = (uint8_t)(value >> 16u);
    destination[3] = (uint8_t)(value >> 24u);
}

static actuator_v2_stream_hard_caps_t hard_caps(void) {
    actuator_v2_stream_hard_caps_t caps;
    memset(&caps, 0, sizeof(caps));
    caps.minimum_lead_ms = 20u;
    caps.maximum_lead_ms = 400u;
    caps.maximum_command_timeout_ms = 500u;
    caps.maximum_open_command_timeout_ms = 100u;
    caps.maximum_apply_lateness_ms = 5u;
    for (size_t joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        caps.tracking_error_limit_urad[joint] = 100000;
        caps.maximum_step_urad_per_tick[joint] = 10000;
    }
    return caps;
}

static void joint_limits(
    actuator_v2_joint_limit_t limits[ACTUATOR_V2_JOINT_COUNT]) {
    for (size_t joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        limits[joint].minimum_urad = -1000000;
        limits[joint].maximum_urad = 1000000;
    }
}

static void make_open(
    uint8_t payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE],
    uint16_t minimum_start_samples,
    uint32_t horizon_end_tick,
    uint32_t command_timeout_ms) {
    memset(payload, 0, ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE);
    write_u16_le(&payload[0], minimum_start_samples);
    payload[2] = ACTUATOR_V2_ARM_MASK_BOTH;
    write_u32_le(&payload[4], 20u);
    write_u32_le(&payload[8], horizon_end_tick);
    write_u32_le(&payload[12], 400u);
    write_u32_le(&payload[16], command_timeout_ms);
    write_u32_le(&payload[20], 5u);
    for (size_t joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        write_u32_le(&payload[24u + joint * 4u], 90000u);
        write_u32_le(&payload[72u + joint * 4u], 9000u);
    }
}

static size_t make_batch(
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE],
    uint32_t first_tick,
    uint32_t horizon_end_tick,
    uint32_t epoch,
    uint32_t splice_tick,
    uint8_t count,
    int32_t first_position,
    int32_t position_step) {
    const size_t length = ACTUATOR_V2_BATCH_HEADER_SIZE +
        (size_t)count * ACTUATOR_V2_SAMPLE_WIRE_SIZE;
    memset(payload, 0, length);
    write_u32_le(&payload[0], first_tick);
    write_u32_le(&payload[4], horizon_end_tick);
    write_u32_le(&payload[8], epoch);
    write_u32_le(&payload[12], splice_tick);
    payload[16] = count;
    payload[17] = ACTUATOR_V2_ARM_MASK_BOTH;
    for (uint8_t sample = 0u; sample < count; ++sample) {
        const size_t offset = ACTUATOR_V2_BATCH_HEADER_SIZE +
            (size_t)sample * ACTUATOR_V2_SAMPLE_WIRE_SIZE;
        write_u32_le(&payload[offset], (uint32_t)sample * 20u);
        for (size_t joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
            const int32_t position = first_position +
                (int32_t)sample * position_step + (int32_t)joint;
            write_u32_le(&payload[offset + 4u + joint * 4u],
                         (uint32_t)position);
        }
    }
    return length;
}

static void setup_executor(actuator_v2_stream_executor_t *executor) {
    actuator_v2_stream_hard_caps_t caps = hard_caps();
    actuator_v2_joint_limit_t limits[ACTUATOR_V2_JOINT_COUNT];
    joint_limits(limits);
    CHECK(actuator_v2_stream_executor_init(executor, &caps, limits) ==
          ACTUATOR_V2_EXECUTOR_OK);
}

static void test_finite_horizon_interpolates_and_succeeds(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 2u, 140u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 140u, 7u, 0u, 2u, 20000, 20000);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);

    CHECK(actuator_v2_stream_executor_step(&executor, 110u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 10000);
    CHECK(output[11] == 10005);
    CHECK(actuator_v2_stream_executor_step(&executor, 120u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 20000);
    CHECK(actuator_v2_stream_executor_step(&executor, 130u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 30000);
    CHECK(actuator_v2_stream_executor_step(&executor, 140u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(output[0] == 40000);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_SUCCEEDED);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_PLANNED_HORIZON);
    CHECK(!executor.diagnostics.safe_stop_required);
    CHECK(executor.diagnostics.applied_samples == 2u);
}

static void test_finite_horizon_underflow_fails_closed(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 160u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 160u, 4u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 120u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(actuator_v2_stream_executor_step(&executor, 125u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_ABORTED);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_QUEUE_UNDERFLOW);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void test_open_stream_holds_then_times_out(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 0u, 60u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 0u, 9u, 0u, 1u, 15000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 120u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(actuator_v2_stream_executor_step(&executor, 160u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 15000);
    CHECK(actuator_v2_stream_executor_step(&executor, 161u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_HOLD);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_COMMAND_TIMEOUT);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void test_splice_replaces_future_continuously(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 2u, 180u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 180u, 1u, 0u, 3u, 20000, 20000);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 120u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);

    length = make_batch(
        batch_payload, 140u, 180u, 2u, 140u, 3u, 40000, 10000);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_SPLICE, 120u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(executor.diagnostics.splice_count == 1u);
    CHECK(executor.session.validated_sample_count == 3u);
    CHECK(actuator_v2_stream_executor_step(&executor, 130u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 30000);
    CHECK(actuator_v2_stream_executor_step(&executor, 140u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    CHECK(output[0] == 40000);
}

static void test_limit_rejection_is_atomic(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 160u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 160u, 3u, 0u, 1u, 1000001, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_JOINT_LIMIT_REJECTED);
    CHECK(executor.session.validated_sample_count == 0u);
    CHECK(executor.diagnostics.accepted_samples == 0u);
}

static void test_minimum_prime_is_enforced(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 2u, 160u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 160u, 5u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_INSUFFICIENT_PRIME);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_PRIMING);
}

static void test_anchor_transition_obeys_step_limit(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 160u, 100u);
    length = make_batch(
        batch_payload, 120u, 160u, 5u, 0u, 1u, 36001, 0);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_PRIMING);
}

static void test_append_boundary_obeys_step_limit_atomically(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 2u, 200u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 200u, 5u, 0u, 2u, 10000, 10000);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 160u, 200u, 5u, 0u, 1u, 56001, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_BATCH_REJECTED);
    CHECK(executor.session.validated_sample_count == 2u);
    CHECK(executor.diagnostics.accepted_samples == 2u);
    CHECK(executor.diagnostics.last_contract_result ==
          ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY);
    length = make_batch(
        batch_payload, 160u, 200u, 5u, 0u, 1u, 56000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(executor.session.validated_sample_count == 3u);
}

static void test_missed_apply_tick_fails_closed(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 120u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 120u, 6u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 126u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_MISSED_APPLY_TICK);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void test_non_aligned_sample_has_zero_apply_lateness(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 122u, 100u);
    write_u32_le(&open_payload[20], 0u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 122u, 122u, 6u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 125u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_SUCCEEDED);
    CHECK(executor.diagnostics.maximum_apply_lateness_ms == 0u);
}

static void test_hardware_tick_phase_must_define_control_epoch(void) {
    actuator_v2_stream_executor_t unaligned;
    actuator_v2_stream_executor_t aligned;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;
    uint32_t tick;

    /* Reproduce the 0x24603 evidence exactly: admission at 22681 ms,
     * a first sample at 22753 ms, and hardware ticks ending in 0 or 5. */
    setup_executor(&unaligned);
    make_open(open_payload, 1u, 22753u, 100u);
    CHECK(actuator_v2_stream_executor_open(
        &unaligned, open_payload, sizeof(open_payload), 22673u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 22753u, 22753u, 1u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &unaligned, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 22681u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&unaligned, 22681u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    for (tick = 22685u; tick <= 22750u; tick += 5u) {
        CHECK(actuator_v2_stream_executor_step(&unaligned, tick, output) ==
              ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    }
    CHECK(actuator_v2_stream_executor_step(&unaligned, 22755u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(unaligned.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_INVALID_TIMELINE);

    /* Starting on the first real TIM6 event aligns both 5 ms grids. */
    setup_executor(&aligned);
    CHECK(actuator_v2_stream_executor_open(
        &aligned, open_payload, sizeof(open_payload), 22673u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_admit(
        &aligned, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 22681u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&aligned, 22685u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    for (tick = 22685u; tick <= 22750u; tick += 5u) {
        CHECK(actuator_v2_stream_executor_step(&aligned, tick, output) ==
              ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    }
    CHECK(actuator_v2_stream_executor_step(&aligned, 22755u, output) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(aligned.diagnostics.state == ACTUATOR_V2_EXECUTOR_SUCCEEDED);
    CHECK(aligned.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_PLANNED_HORIZON);
}

static void test_joint_specific_tracking_error_fails_closed(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t measured[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 160u, 100u);
    write_u32_le(&open_payload[24u + 11u * 4u], 50000u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 160u, 8u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 110u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);
    memcpy(measured, output, sizeof(measured));
    measured[0] -= 89999;
    measured[11] -= 50001;
    CHECK(actuator_v2_stream_executor_check_feedback(
        &executor, 110u, measured) == ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_TRACKING_ERROR);
    CHECK(executor.diagnostics.tracking_error_joint == 11u);
    CHECK(executor.diagnostics.maximum_tracking_error_urad[0] == 89999u);
    CHECK(executor.diagnostics.maximum_tracking_error_urad[11] == 50001u);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void test_async_joint_feedback_uses_captured_command(void) {
    actuator_v2_stream_executor_t executor;
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    int32_t anchor[ACTUATOR_V2_JOINT_COUNT] = {0};
    int32_t output[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length;

    setup_executor(&executor);
    make_open(open_payload, 1u, 160u, 100u);
    write_u32_le(&open_payload[24u + 7u * 4u], 30000u);
    CHECK(actuator_v2_stream_executor_open(
        &executor, open_payload, sizeof(open_payload), 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    length = make_batch(
        batch_payload, 120u, 160u, 8u, 0u, 1u, 10000, 0);
    CHECK(actuator_v2_stream_executor_admit(
        &executor, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u) ==
        ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_start(&executor, 100u, anchor) ==
          ACTUATOR_V2_EXECUTOR_OK);
    CHECK(actuator_v2_stream_executor_step(&executor, 110u, output) ==
          ACTUATOR_V2_EXECUTOR_OUTPUT_READY);

    CHECK(actuator_v2_stream_executor_check_joint_feedback(
        &executor, 111u, 7u, 20000, -9999) == ACTUATOR_V2_EXECUTOR_OK);
    CHECK(executor.diagnostics.maximum_tracking_error_urad[7] == 29999u);
    CHECK(actuator_v2_stream_executor_check_joint_feedback(
        &executor, 112u, 7u, 20000, -10001) ==
        ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_TRACKING_ERROR);
    CHECK(executor.diagnostics.tracking_error_joint == 7u);
    CHECK(executor.diagnostics.maximum_tracking_error_urad[7] == 30001u);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void test_async_joint_feedback_checks_final_succeeded_output(void) {
    actuator_v2_stream_executor_t executor;
    setup_executor(&executor);
    executor.diagnostics.state = ACTUATOR_V2_EXECUTOR_SUCCEEDED;
    executor.diagnostics.terminal_reason =
        ACTUATOR_V2_TERMINAL_PLANNED_HORIZON;
    executor.output_valid = true;
    executor.session.policy.tracking_error_limit_urad[2] = 30000;
    CHECK(actuator_v2_stream_executor_check_joint_feedback(
              &executor, 205u, 2u, 100000, 130001) ==
          ACTUATOR_V2_EXECUTOR_TERMINAL);
    CHECK(executor.diagnostics.state == ACTUATOR_V2_EXECUTOR_ABORTED);
    CHECK(executor.diagnostics.terminal_reason ==
          ACTUATOR_V2_TERMINAL_TRACKING_ERROR);
    CHECK(executor.diagnostics.safe_stop_required);
}

static void run_test(const char *name, void (*test)(void)) {
    const int before = failures;
    test();
    if (failures == before) {
        printf("PASS %s\n", name);
    }
}

int main(void) {
    run_test("finite_horizon_interpolates_and_succeeds",
             test_finite_horizon_interpolates_and_succeeds);
    run_test("finite_horizon_underflow_fails_closed",
             test_finite_horizon_underflow_fails_closed);
    run_test("open_stream_holds_then_times_out",
             test_open_stream_holds_then_times_out);
    run_test("splice_replaces_future_continuously",
             test_splice_replaces_future_continuously);
    run_test("limit_rejection_is_atomic", test_limit_rejection_is_atomic);
    run_test("minimum_prime_is_enforced", test_minimum_prime_is_enforced);
    run_test("anchor_transition_obeys_step_limit",
             test_anchor_transition_obeys_step_limit);
    run_test("append_boundary_obeys_step_limit_atomically",
             test_append_boundary_obeys_step_limit_atomically);
    run_test("missed_apply_tick_fails_closed",
             test_missed_apply_tick_fails_closed);
    run_test("non_aligned_sample_has_zero_apply_lateness",
             test_non_aligned_sample_has_zero_apply_lateness);
    run_test("hardware_tick_phase_must_define_control_epoch",
             test_hardware_tick_phase_must_define_control_epoch);
    run_test("joint_specific_tracking_error_fails_closed",
             test_joint_specific_tracking_error_fails_closed);
    run_test("async_joint_feedback_uses_captured_command",
             test_async_joint_feedback_uses_captured_command);
    run_test("async_joint_feedback_checks_final_succeeded_output",
             test_async_joint_feedback_checks_final_succeeded_output);
    return failures == 0 ? 0 : 1;
}
