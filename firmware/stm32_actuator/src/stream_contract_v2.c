#include "actuator_core/stream_contract_v2.h"

#include <limits.h>
#include <string.h>

static uint16_t read_u16_le(const uint8_t *source) {
    return (uint16_t)((uint16_t)source[0] | ((uint16_t)source[1] << 8u));
}

static uint32_t read_u32_le(const uint8_t *source) {
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8u) |
           ((uint32_t)source[2] << 16u) |
           ((uint32_t)source[3] << 24u);
}

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

static bool arm_mask_is_valid(uint8_t arm_mask) {
    return arm_mask != 0u &&
           (arm_mask & (uint8_t)(~ACTUATOR_V2_ARM_MASK_BOTH)) == 0u;
}

static int64_t absolute_difference(int32_t left, int32_t right) {
    int64_t difference = (int64_t)left - (int64_t)right;
    return difference < 0 ? -difference : difference;
}

actuator_v2_contract_result_t actuator_v2_stream_policy_decode(
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_stream_policy_t *policy) {
    size_t joint;

    if (payload == NULL || policy == NULL) {
        return ACTUATOR_V2_CONTRACT_NULL_ARGUMENT;
    }
    if (payload_length != ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE) {
        return ACTUATOR_V2_CONTRACT_INVALID_LENGTH;
    }
    if (!arm_mask_is_valid(payload[2])) {
        return ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK;
    }
    if (payload[3] != 0u) {
        return ACTUATOR_V2_CONTRACT_INVALID_RESERVED;
    }

    memset(policy, 0, sizeof(*policy));
    policy->minimum_start_samples = read_u16_le(&payload[0]);
    policy->arm_mask = payload[2];
    policy->minimum_lead_ms = read_u32_le(&payload[4]);
    policy->horizon_end_tick = read_u32_le(&payload[8]);
    policy->maximum_lead_ms = read_u32_le(&payload[12]);
    policy->command_timeout_ms = read_u32_le(&payload[16]);
    policy->maximum_apply_lateness_ms = read_u32_le(&payload[20]);

    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        policy->tracking_error_limit_urad[joint] =
            (int32_t)read_u32_le(&payload[24u + (joint * 4u)]);
        policy->maximum_step_urad_per_tick[joint] =
            (int32_t)read_u32_le(&payload[72u + (joint * 4u)]);
    }
    return ACTUATOR_V2_CONTRACT_OK;
}

actuator_v2_contract_result_t actuator_v2_stream_policy_validate(
    const actuator_v2_stream_policy_t *policy,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    uint32_t current_tick) {
    size_t joint;

    if (policy == NULL || hard_caps == NULL) {
        return ACTUATOR_V2_CONTRACT_NULL_ARGUMENT;
    }
    if (!arm_mask_is_valid(policy->arm_mask)) {
        return ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK;
    }
    if (policy->minimum_start_samples == 0u ||
        policy->minimum_start_samples > ACTUATOR_V2_QUEUE_CAPACITY) {
        return ACTUATOR_V2_CONTRACT_INVALID_MINIMUM_START_SAMPLES;
    }
    if (policy->minimum_lead_ms < hard_caps->minimum_lead_ms) {
        return ACTUATOR_V2_CONTRACT_MINIMUM_LEAD_TOO_SMALL;
    }
    if (policy->maximum_lead_ms > hard_caps->maximum_lead_ms) {
        return ACTUATOR_V2_CONTRACT_MAXIMUM_LEAD_TOO_LARGE;
    }
    if (policy->minimum_lead_ms > policy->maximum_lead_ms) {
        return ACTUATOR_V2_CONTRACT_LEAD_WINDOW_INVERTED;
    }
    if (policy->command_timeout_ms == 0u ||
        policy->command_timeout_ms > hard_caps->maximum_command_timeout_ms) {
        return ACTUATOR_V2_CONTRACT_COMMAND_TIMEOUT_TOO_LARGE;
    }
    if (policy->horizon_end_tick == 0u &&
        policy->command_timeout_ms >
            hard_caps->maximum_open_command_timeout_ms) {
        return ACTUATOR_V2_CONTRACT_OPEN_TIMEOUT_TOO_LARGE;
    }
    if (policy->maximum_apply_lateness_ms >
        hard_caps->maximum_apply_lateness_ms) {
        return ACTUATOR_V2_CONTRACT_APPLY_LATENESS_TOO_LARGE;
    }
    if (policy->horizon_end_tick != 0u &&
        !tick_is_after(policy->horizon_end_tick, current_tick)) {
        return ACTUATOR_V2_CONTRACT_STALE_HORIZON;
    }

    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        if (policy->tracking_error_limit_urad[joint] <= 0 ||
            hard_caps->tracking_error_limit_urad[joint] <= 0 ||
            policy->tracking_error_limit_urad[joint] >
                hard_caps->tracking_error_limit_urad[joint]) {
            return ACTUATOR_V2_CONTRACT_TRACKING_ERROR_TOO_LARGE;
        }
        if (policy->maximum_step_urad_per_tick[joint] <= 0 ||
            hard_caps->maximum_step_urad_per_tick[joint] <= 0 ||
            policy->maximum_step_urad_per_tick[joint] >
                hard_caps->maximum_step_urad_per_tick[joint]) {
            return ACTUATOR_V2_CONTRACT_MAXIMUM_STEP_TOO_LARGE;
        }
    }
    return ACTUATOR_V2_CONTRACT_OK;
}

actuator_v2_contract_result_t actuator_v2_batch_decode(
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    actuator_v2_batch_t *batch) {
    size_t expected_length;
    uint32_t previous_tick = 0u;
    size_t sample_index;

    if (payload == NULL || batch == NULL) {
        return ACTUATOR_V2_CONTRACT_NULL_ARGUMENT;
    }
    if (payload_length < ACTUATOR_V2_BATCH_HEADER_SIZE) {
        return ACTUATOR_V2_CONTRACT_INVALID_LENGTH;
    }
    if (payload[16] == 0u ||
        payload[16] > ACTUATOR_V2_WIRE_MAX_SAMPLES) {
        return ACTUATOR_V2_CONTRACT_INVALID_SAMPLE_COUNT;
    }
    expected_length = ACTUATOR_V2_BATCH_HEADER_SIZE +
        ((size_t)payload[16] * ACTUATOR_V2_SAMPLE_WIRE_SIZE);
    if (payload_length != expected_length) {
        return ACTUATOR_V2_CONTRACT_INVALID_LENGTH;
    }
    if (!arm_mask_is_valid(payload[17])) {
        return ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK;
    }
    if (read_u16_le(&payload[18]) != 0u) {
        return ACTUATOR_V2_CONTRACT_INVALID_RESERVED;
    }
    if ((kind == ACTUATOR_V2_BATCH_APPEND &&
         read_u32_le(&payload[12]) != 0u) ||
        (kind == ACTUATOR_V2_BATCH_SPLICE &&
         read_u32_le(&payload[12]) == 0u)) {
        return ACTUATOR_V2_CONTRACT_SPLICE_FIELD_MISMATCH;
    }

    memset(batch, 0, sizeof(*batch));
    batch->first_apply_tick = read_u32_le(&payload[0]);
    batch->horizon_end_tick = read_u32_le(&payload[4]);
    batch->arbiter_epoch = read_u32_le(&payload[8]);
    batch->splice_at_tick = read_u32_le(&payload[12]);
    batch->sample_count = payload[16];
    batch->arm_mask = payload[17];

    for (sample_index = 0u; sample_index < batch->sample_count; ++sample_index) {
        size_t joint;
        const size_t offset = ACTUATOR_V2_BATCH_HEADER_SIZE +
            (sample_index * ACTUATOR_V2_SAMPLE_WIRE_SIZE);
        const uint32_t tick_offset = read_u32_le(&payload[offset]);
        const uint32_t apply_tick = batch->first_apply_tick + tick_offset;

        if ((sample_index > 0u && !tick_is_after(apply_tick, previous_tick)) ||
            (sample_index == 0u && tick_offset != 0u)) {
            return ACTUATOR_V2_CONTRACT_NON_MONOTONIC_TICK;
        }
        previous_tick = apply_tick;
        batch->samples[sample_index].apply_tick = apply_tick;
        for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
            batch->samples[sample_index].position_urad[joint] =
                (int32_t)read_u32_le(&payload[offset + 4u + (joint * 4u)]);
        }
    }
    return ACTUATOR_V2_CONTRACT_OK;
}

actuator_v2_contract_result_t actuator_v2_batch_validate_transition(
    const actuator_v2_batch_t *batch,
    actuator_v2_batch_kind_t kind,
    const actuator_v2_stream_policy_t *policy,
    uint32_t current_tick,
    uint32_t current_epoch,
    uint32_t current_horizon_end_tick,
    uint32_t last_apply_tick,
    const int32_t interpolated_position_urad[ACTUATOR_V2_JOINT_COUNT]) {
    uint32_t final_apply_tick;
    size_t joint;

    if (batch == NULL || policy == NULL ||
        interpolated_position_urad == NULL) {
        return ACTUATOR_V2_CONTRACT_NULL_ARGUMENT;
    }
    if (!arm_mask_is_valid(batch->arm_mask) ||
        (batch->arm_mask & policy->arm_mask) != batch->arm_mask) {
        return ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK;
    }
    if (batch->sample_count == 0u ||
        batch->sample_count > ACTUATOR_V2_WIRE_MAX_SAMPLES) {
        return ACTUATOR_V2_CONTRACT_INVALID_SAMPLE_COUNT;
    }
    final_apply_tick = batch->samples[batch->sample_count - 1u].apply_tick;
    if (kind == ACTUATOR_V2_BATCH_APPEND &&
        (!tick_is_after(batch->first_apply_tick, current_tick) ||
         (batch->first_apply_tick - current_tick) < policy->minimum_lead_ms)) {
        return ACTUATOR_V2_CONTRACT_FIRST_SAMPLE_TOO_EARLY;
    }
    if ((final_apply_tick - current_tick) > policy->maximum_lead_ms) {
        return ACTUATOR_V2_CONTRACT_LAST_SAMPLE_TOO_LATE;
    }
    if (batch->horizon_end_tick != 0u &&
        tick_is_after(final_apply_tick, batch->horizon_end_tick)) {
        return ACTUATOR_V2_CONTRACT_HORIZON_BEFORE_LAST_SAMPLE;
    }
    if ((current_horizon_end_tick == 0u) !=
        (batch->horizon_end_tick == 0u)) {
        return ACTUATOR_V2_CONTRACT_HORIZON_REGRESSION;
    }
    if (current_horizon_end_tick != 0u &&
        batch->horizon_end_tick != current_horizon_end_tick &&
        !tick_is_after(batch->horizon_end_tick, current_horizon_end_tick)) {
        return ACTUATOR_V2_CONTRACT_HORIZON_REGRESSION;
    }

    for (size_t sample = 1u; sample < batch->sample_count; ++sample) {
        const uint32_t delta_ms =
            batch->samples[sample].apply_tick -
            batch->samples[sample - 1u].apply_tick;
        if (delta_ms == 0u ||
            (delta_ms % ACTUATOR_V2_CONTROL_TICK_MS) != 0u) {
            return ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY;
        }
        const int64_t output_ticks =
            (int64_t)(delta_ms / ACTUATOR_V2_CONTROL_TICK_MS);
        for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
            const int64_t maximum_delta =
                (int64_t)policy->maximum_step_urad_per_tick[joint] *
                output_ticks;
            if (absolute_difference(
                    batch->samples[sample].position_urad[joint],
                    batch->samples[sample - 1u].position_urad[joint]) >
                maximum_delta) {
                return ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY;
            }
        }
    }

    if (kind == ACTUATOR_V2_BATCH_APPEND) {
        if (batch->splice_at_tick != 0u) {
            return ACTUATOR_V2_CONTRACT_SPLICE_FIELD_MISMATCH;
        }
        if (batch->arbiter_epoch != current_epoch) {
            return ACTUATOR_V2_CONTRACT_APPEND_EPOCH_MISMATCH;
        }
        if (!tick_is_after(batch->first_apply_tick, last_apply_tick)) {
            return ACTUATOR_V2_CONTRACT_NON_MONOTONIC_TICK;
        }
        return ACTUATOR_V2_CONTRACT_OK;
    }

    if (kind != ACTUATOR_V2_BATCH_SPLICE || batch->splice_at_tick == 0u) {
        return ACTUATOR_V2_CONTRACT_SPLICE_FIELD_MISMATCH;
    }
    if (!tick_is_after(batch->splice_at_tick, current_tick) ||
        (batch->splice_at_tick - current_tick) <
            ACTUATOR_V2_MINIMUM_SPLICE_LEAD_MS ||
        (batch->splice_at_tick - current_tick) < policy->minimum_lead_ms) {
        return ACTUATOR_V2_CONTRACT_SPLICE_TOO_LATE;
    }
    if (tick_is_after(batch->splice_at_tick, last_apply_tick)) {
        return ACTUATOR_V2_CONTRACT_SPLICE_AFTER_LAST_SAMPLE;
    }
    if (batch->samples[0].apply_tick != batch->splice_at_tick) {
        return ACTUATOR_V2_CONTRACT_SPLICE_FIRST_TICK_MISMATCH;
    }

    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        if (absolute_difference(
                batch->samples[0].position_urad[joint],
                interpolated_position_urad[joint]) >
            policy->maximum_step_urad_per_tick[joint]) {
            return ACTUATOR_V2_CONTRACT_SPLICE_DISCONTINUITY;
        }
    }
    return ACTUATOR_V2_CONTRACT_OK;
}
