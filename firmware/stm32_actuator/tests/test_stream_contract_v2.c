#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/stream_contract_v2.h"

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
    size_t joint;

    memset(&caps, 0, sizeof(caps));
    caps.minimum_lead_ms = 20u;
    caps.maximum_lead_ms = 400u;
    caps.maximum_command_timeout_ms = 500u;
    caps.maximum_open_command_timeout_ms = 100u;
    caps.maximum_apply_lateness_ms = 5u;
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        caps.tracking_error_limit_urad[joint] = 100000;
        caps.maximum_step_urad_per_tick[joint] = 10000;
    }
    return caps;
}

static actuator_v2_stream_policy_t finite_policy(void) {
    actuator_v2_stream_policy_t policy;
    size_t joint;

    memset(&policy, 0, sizeof(policy));
    policy.minimum_start_samples = 2u;
    policy.minimum_lead_ms = 20u;
    policy.horizon_end_tick = 300u;
    policy.maximum_lead_ms = 400u;
    policy.command_timeout_ms = 500u;
    policy.maximum_apply_lateness_ms = 5u;
    policy.arm_mask = ACTUATOR_V2_ARM_MASK_BOTH;
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        policy.tracking_error_limit_urad[joint] = 90000;
        policy.maximum_step_urad_per_tick[joint] = 9000;
    }
    return policy;
}

static size_t make_policy_payload(
    uint8_t payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE],
    const actuator_v2_stream_policy_t *policy) {
    size_t joint;

    memset(payload, 0, ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE);
    write_u16_le(&payload[0], policy->minimum_start_samples);
    payload[2] = policy->arm_mask;
    write_u32_le(&payload[4], policy->minimum_lead_ms);
    write_u32_le(&payload[8], policy->horizon_end_tick);
    write_u32_le(&payload[12], policy->maximum_lead_ms);
    write_u32_le(&payload[16], policy->command_timeout_ms);
    write_u32_le(&payload[20], policy->maximum_apply_lateness_ms);
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        write_u32_le(
            &payload[24u + (joint * 4u)],
            (uint32_t)policy->tracking_error_limit_urad[joint]);
        write_u32_le(
            &payload[72u + (joint * 4u)],
            (uint32_t)policy->maximum_step_urad_per_tick[joint]);
    }
    return ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE;
}

static size_t make_batch_payload(
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE],
    uint32_t first_tick,
    uint32_t horizon_end_tick,
    uint32_t epoch,
    uint32_t splice_at_tick,
    uint8_t arm_mask,
    uint8_t sample_count,
    int32_t first_position) {
    size_t sample;

    const size_t length = ACTUATOR_V2_BATCH_HEADER_SIZE +
        ((size_t)sample_count * ACTUATOR_V2_SAMPLE_WIRE_SIZE);
    memset(payload, 0, length);
    write_u32_le(&payload[0], first_tick);
    write_u32_le(&payload[4], horizon_end_tick);
    write_u32_le(&payload[8], epoch);
    write_u32_le(&payload[12], splice_at_tick);
    payload[16] = sample_count;
    payload[17] = arm_mask;
    for (sample = 0u; sample < sample_count; ++sample) {
        size_t joint;
        const size_t offset = ACTUATOR_V2_BATCH_HEADER_SIZE +
            (sample * ACTUATOR_V2_SAMPLE_WIRE_SIZE);
        write_u32_le(&payload[offset], (uint32_t)(sample * 20u));
        for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
            write_u32_le(
                &payload[offset + 4u + (joint * 4u)],
                (uint32_t)(first_position + (int32_t)sample));
        }
    }
    return length;
}

static void test_wire_sizes_fit_existing_payload_limit(void) {
    CHECK(ACTUATOR_V2_JOINT_COUNT == 12u);
    CHECK(ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE == 488u);
    CHECK(ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE <= 512u);
    CHECK(ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE == 120u);
}

static void test_policy_round_trip_and_all_arm_masks(void) {
    actuator_v2_stream_policy_t source = finite_policy();
    actuator_v2_stream_policy_t decoded;
    uint8_t payload[ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE];

    for (uint8_t arm_mask = ACTUATOR_V2_ARM_MASK_LEFT;
         arm_mask <= ACTUATOR_V2_ARM_MASK_BOTH;
         ++arm_mask) {
        source.arm_mask = arm_mask;
        CHECK(actuator_v2_stream_policy_decode(
            payload, make_policy_payload(payload, &source), &decoded) ==
            ACTUATOR_V2_CONTRACT_OK);
        CHECK(decoded.arm_mask == arm_mask);
        CHECK(decoded.minimum_start_samples == source.minimum_start_samples);
        CHECK(decoded.tracking_error_limit_urad[11] ==
              source.tracking_error_limit_urad[11]);
    }
    source.arm_mask = 0u;
    make_policy_payload(payload, &source);
    CHECK(actuator_v2_stream_policy_decode(
        payload, sizeof(payload), &decoded) ==
        ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK);
    source.arm_mask = ACTUATOR_V2_ARM_MASK_BOTH;
    make_policy_payload(payload, &source);
    payload[3] = 1u;
    CHECK(actuator_v2_stream_policy_decode(
        payload, sizeof(payload), &decoded) ==
        ACTUATOR_V2_CONTRACT_INVALID_RESERVED);
}

static void test_policy_rejects_every_loosened_hard_cap(void) {
    actuator_v2_stream_hard_caps_t caps = hard_caps();
    actuator_v2_stream_policy_t policy = finite_policy();

    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_OK);

    policy.minimum_lead_ms = 19u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_MINIMUM_LEAD_TOO_SMALL);
    policy = finite_policy();
    policy.maximum_lead_ms = 401u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_MAXIMUM_LEAD_TOO_LARGE);
    policy = finite_policy();
    policy.command_timeout_ms = 501u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_COMMAND_TIMEOUT_TOO_LARGE);
    policy = finite_policy();
    policy.maximum_apply_lateness_ms = 6u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_APPLY_LATENESS_TOO_LARGE);
    policy = finite_policy();
    policy.tracking_error_limit_urad[7] = 100001;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_TRACKING_ERROR_TOO_LARGE);
    policy = finite_policy();
    policy.maximum_step_urad_per_tick[4] = 10001;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_MAXIMUM_STEP_TOO_LARGE);
}

static void test_open_horizon_uses_tighter_timeout(void) {
    actuator_v2_stream_hard_caps_t caps = hard_caps();
    actuator_v2_stream_policy_t policy = finite_policy();

    policy.horizon_end_tick = 0u;
    policy.command_timeout_ms = 100u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_OK);
    policy.command_timeout_ms = 101u;
    CHECK(actuator_v2_stream_policy_validate(&policy, &caps, 100u) ==
          ACTUATOR_V2_CONTRACT_OPEN_TIMEOUT_TOO_LARGE);
}

static void test_batch_decode_carries_one_twelve_joint_sample(void) {
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_batch_t batch;
    const size_t length = make_batch_payload(
        payload, 140u, 400u, 8u, 0u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 123);

    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(batch.samples[0].apply_tick == 140u);
    CHECK(batch.samples[1].apply_tick == 160u);
    CHECK(batch.samples[0].position_urad[0] == 123);
    CHECK(batch.samples[0].position_urad[11] == 123);
    payload[17] = 4u;
    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK);
}

static void test_epoch_change_requires_splice(void) {
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_batch_t batch;
    actuator_v2_stream_policy_t policy = finite_policy();
    int32_t interpolated[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length = make_batch_payload(
        payload, 320u, 400u, 8u, 0u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 0);

    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_APPEND_EPOCH_MISMATCH);

    length = make_batch_payload(
        payload, 140u, 400u, 8u, 140u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 100);
    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_SPLICE, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_SPLICE, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_OK);
}

static void test_splice_is_lead_bounded_and_continuous(void) {
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_batch_t batch;
    actuator_v2_stream_policy_t policy = finite_policy();
    int32_t interpolated[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length = make_batch_payload(
        payload, 140u, 400u, 7u, 140u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 9000);

    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_SPLICE, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_SPLICE, &policy,
        121u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_SPLICE_TOO_LATE);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_SPLICE, &policy,
        100u, 7u, 300u, 130u, interpolated) ==
        ACTUATOR_V2_CONTRACT_SPLICE_AFTER_LAST_SAMPLE);

    batch.samples[0].position_urad[11] = 9001;
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_SPLICE, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_SPLICE_DISCONTINUITY);
}

static void test_horizon_cannot_regress_or_exclude_samples(void) {
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_batch_t batch;
    actuator_v2_stream_policy_t policy = finite_policy();
    int32_t interpolated[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length = make_batch_payload(
        payload, 320u, 400u, 7u, 0u,
        ACTUATOR_V2_ARM_MASK_LEFT, 2u, 0);

    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_OK);

    batch.horizon_end_tick = 299u;
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_HORIZON_BEFORE_LAST_SAMPLE);
    length = make_batch_payload(
        payload, 140u, 299u, 7u, 140u,
        ACTUATOR_V2_ARM_MASK_LEFT, 2u, 0);
    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_SPLICE, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_SPLICE, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_HORIZON_REGRESSION);
}

static void test_batch_obeys_lead_window_and_step_limit(void) {
    uint8_t payload[ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE];
    actuator_v2_batch_t batch;
    actuator_v2_stream_policy_t policy = finite_policy();
    int32_t interpolated[ACTUATOR_V2_JOINT_COUNT] = {0};
    size_t length = make_batch_payload(
        payload, 119u, 400u, 7u, 0u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 0);

    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 110u, interpolated) ==
        ACTUATOR_V2_CONTRACT_FIRST_SAMPLE_TOO_EARLY);

    length = make_batch_payload(
        payload, 481u, 600u, 7u, 0u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 0);
    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_LAST_SAMPLE_TOO_LATE);

    length = make_batch_payload(
        payload, 320u, 400u, 7u, 0u,
        ACTUATOR_V2_ARM_MASK_BOTH, 2u, 0);
    write_u32_le(
        &payload[ACTUATOR_V2_BATCH_HEADER_SIZE +
                 ACTUATOR_V2_SAMPLE_WIRE_SIZE + 4u],
        40001u);
    CHECK(actuator_v2_batch_decode(
        payload, length, ACTUATOR_V2_BATCH_APPEND, &batch) ==
        ACTUATOR_V2_CONTRACT_OK);
    CHECK(actuator_v2_batch_validate_transition(
        &batch, ACTUATOR_V2_BATCH_APPEND, &policy,
        100u, 7u, 300u, 300u, interpolated) ==
        ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY);
}

static void run_test(const char *name, void (*test)(void)) {
    const int before = failures;
    test();
    if (failures == before) {
        printf("PASS %s\n", name);
    }
}

int main(void) {
    run_test("wire_sizes_fit_existing_payload_limit",
             test_wire_sizes_fit_existing_payload_limit);
    run_test("policy_round_trip_and_all_arm_masks",
             test_policy_round_trip_and_all_arm_masks);
    run_test("policy_rejects_every_loosened_hard_cap",
             test_policy_rejects_every_loosened_hard_cap);
    run_test("open_horizon_uses_tighter_timeout",
             test_open_horizon_uses_tighter_timeout);
    run_test("batch_decode_carries_one_twelve_joint_sample",
             test_batch_decode_carries_one_twelve_joint_sample);
    run_test("epoch_change_requires_splice",
             test_epoch_change_requires_splice);
    run_test("splice_is_lead_bounded_and_continuous",
             test_splice_is_lead_bounded_and_continuous);
    run_test("horizon_cannot_regress_or_exclude_samples",
             test_horizon_cannot_regress_or_exclude_samples);
    run_test("batch_obeys_lead_window_and_step_limit",
             test_batch_obeys_lead_window_and_step_limit);
    if (failures != 0) {
        return 1;
    }
    printf("All protocol v2 stream contract tests passed.\n");
    return 0;
}
