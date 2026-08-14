#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/stream_session_v2.h"

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

static void make_open(uint8_t payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE]) {
    memset(payload, 0, ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE);
    write_u16_le(&payload[0], 2u);
    payload[2] = ACTUATOR_V2_ARM_MASK_BOTH;
    write_u32_le(&payload[4], 20u);
    write_u32_le(&payload[8], 1000u);
    write_u32_le(&payload[12], 400u);
    write_u32_le(&payload[16], 500u);
    write_u32_le(&payload[20], 5u);
    for (size_t joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        write_u32_le(&payload[24u + joint * 4u], 90000u);
        write_u32_le(&payload[72u + joint * 4u], 9000u);
    }
}

static size_t make_batch(
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE],
    uint32_t first_tick,
    uint32_t epoch,
    uint32_t splice_tick,
    uint8_t count) {
    const size_t length = ACTUATOR_V2_BATCH_HEADER_SIZE +
        (size_t)count * ACTUATOR_V2_SAMPLE_WIRE_SIZE;
    memset(payload, 0, length);
    write_u32_le(&payload[0], first_tick);
    write_u32_le(&payload[4], 1000u);
    write_u32_le(&payload[8], epoch);
    write_u32_le(&payload[12], splice_tick);
    payload[16] = count;
    payload[17] = ACTUATOR_V2_ARM_MASK_BOTH;
    for (uint8_t sample = 0u; sample < count; ++sample) {
        const size_t offset = ACTUATOR_V2_BATCH_HEADER_SIZE +
            (size_t)sample * ACTUATOR_V2_SAMPLE_WIRE_SIZE;
        write_u32_le(&payload[offset], (uint32_t)sample * 20u);
    }
    return length;
}

static void test_open_append_and_splice(void) {
    actuator_v2_stream_session_t session;
    actuator_v2_stream_hard_caps_t caps = hard_caps();
    uint8_t open_payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    uint8_t batch_payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_stream_session_result_t result;

    actuator_v2_stream_session_init(&session);
    make_open(open_payload);
    result = actuator_v2_stream_session_open(
        &session, open_payload, sizeof(open_payload), &caps, 100u);
    CHECK(result.status_code == ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY);
    CHECK(result.contract_result == ACTUATOR_V2_CONTRACT_OK);

    size_t length = make_batch(batch_payload, 250u, 7u, 0u, 3u);
    result = actuator_v2_stream_session_batch(
        &session, batch_payload, length, ACTUATOR_V2_BATCH_APPEND, 100u);
    CHECK(result.status_code == ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY);
    CHECK(result.arbiter_epoch == 7u);
    CHECK(result.validated_sample_count == 3u);
    CHECK(result.validated_tail_tick == 290u);

    length = make_batch(batch_payload, 270u, 8u, 270u, 2u);
    result = actuator_v2_stream_session_batch(
        &session, batch_payload, length, ACTUATOR_V2_BATCH_SPLICE, 100u);
    CHECK(result.status_code == ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY);
    CHECK(result.arbiter_epoch == 8u);
    CHECK(result.validated_sample_count == 3u);
    CHECK(result.validated_tail_tick == 290u);
}

static void test_rejected_open_preserves_session(void) {
    actuator_v2_stream_session_t session;
    actuator_v2_stream_hard_caps_t caps = hard_caps();
    uint8_t payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];
    actuator_v2_stream_session_result_t result;

    actuator_v2_stream_session_init(&session);
    make_open(payload);
    result = actuator_v2_stream_session_open(
        &session, payload, sizeof(payload), &caps, 100u);
    CHECK(result.status_code == ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY);
    write_u32_le(&payload[4], 19u);
    result = actuator_v2_stream_session_open(
        &session, payload, sizeof(payload), &caps, 100u);
    CHECK(result.status_code == ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED);
    CHECK(result.contract_result == ACTUATOR_V2_CONTRACT_MINIMUM_LEAD_TOO_SMALL);
    CHECK(session.open);
    CHECK(session.policy.minimum_lead_ms == 20u);
}

static void run_test(const char *name, void (*test)(void)) {
    const int before = failures;
    test();
    if (failures == before) {
        printf("PASS %s\n", name);
    }
}

int main(void) {
    run_test("open_append_and_splice", test_open_append_and_splice);
    run_test("rejected_open_preserves_session", test_rejected_open_preserves_session);
    return failures == 0 ? 0 : 1;
}
