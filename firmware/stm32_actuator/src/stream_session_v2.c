#include "actuator_core/stream_session_v2.h"

#include <string.h>

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

static bool tick_is_before(uint32_t candidate, uint32_t reference) {
    return tick_is_after(reference, candidate);
}

static actuator_v2_stream_session_result_t result_from_session(
    const actuator_v2_stream_session_t *session,
    actuator_v2_stream_status_code_t status_code,
    actuator_v2_contract_result_t contract_result) {
    actuator_v2_stream_session_result_t result;

    memset(&result, 0, sizeof(result));
    result.status_code = status_code;
    result.contract_result = contract_result;
    if (session != NULL) {
        result.arm_mask = session->open ? session->policy.arm_mask : 0u;
        result.arbiter_epoch = session->arbiter_epoch;
        result.horizon_end_tick = session->horizon_end_tick;
        result.validated_sample_count = session->validated_sample_count;
        if (session->validated_sample_count > 0u) {
            result.validated_tail_tick = session->validated_samples[
                session->validated_sample_count - 1u].apply_tick;
        }
    }
    return result;
}

static bool interpolate_position(
    const actuator_v2_stream_session_t *session,
    uint32_t tick,
    int32_t output[ACTUATOR_V2_JOINT_COUNT]) {
    size_t index;

    if (session == NULL || output == NULL ||
        session->validated_sample_count == 0u) {
        return false;
    }
    for (index = 0u; index < session->validated_sample_count; ++index) {
        const actuator_v2_setpoint_t *sample =
            &session->validated_samples[index];
        if (sample->apply_tick == tick) {
            memcpy(output, sample->position_urad, sizeof(sample->position_urad));
            return true;
        }
        if (index + 1u < session->validated_sample_count) {
            const actuator_v2_setpoint_t *next =
                &session->validated_samples[index + 1u];
            if (tick_is_after(tick, sample->apply_tick) &&
                tick_is_before(tick, next->apply_tick)) {
                const uint32_t span = next->apply_tick - sample->apply_tick;
                const uint32_t elapsed = tick - sample->apply_tick;
                size_t joint;
                for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
                    const int64_t delta =
                        (int64_t)next->position_urad[joint] -
                        (int64_t)sample->position_urad[joint];
                    output[joint] = sample->position_urad[joint] +
                        (int32_t)((delta * (int64_t)elapsed) / (int64_t)span);
                }
                return true;
            }
        }
    }
    return false;
}

void actuator_v2_stream_session_init(actuator_v2_stream_session_t *session) {
    if (session != NULL) {
        memset(session, 0, sizeof(*session));
    }
}

actuator_v2_stream_session_result_t actuator_v2_stream_session_open(
    actuator_v2_stream_session_t *session,
    const uint8_t *payload,
    size_t payload_length,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    uint32_t current_tick) {
    actuator_v2_stream_policy_t policy;
    actuator_v2_contract_result_t contract_result;

    if (session == NULL) {
        return result_from_session(
            NULL,
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            ACTUATOR_V2_CONTRACT_NULL_ARGUMENT);
    }
    contract_result = actuator_v2_stream_policy_decode(
        payload, payload_length, &policy);
    if (contract_result == ACTUATOR_V2_CONTRACT_OK) {
        contract_result = actuator_v2_stream_policy_validate(
            &policy, hard_caps, current_tick);
    }
    if (contract_result != ACTUATOR_V2_CONTRACT_OK) {
        return result_from_session(
            session,
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            contract_result);
    }

    actuator_v2_stream_session_init(session);
    session->open = true;
    session->policy = policy;
    session->horizon_end_tick = policy.horizon_end_tick;
    return result_from_session(
        session,
        ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY,
        ACTUATOR_V2_CONTRACT_OK);
}

actuator_v2_stream_session_result_t actuator_v2_stream_session_batch(
    actuator_v2_stream_session_t *session,
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    uint32_t current_tick) {
    actuator_v2_batch_t batch;
    actuator_v2_contract_result_t contract_result;
    int32_t splice_position[ACTUATOR_V2_JOINT_COUNT] = {0};
    const int32_t *transition_position = splice_position;
    uint32_t current_epoch;
    uint32_t last_tick;
    uint8_t retained_count = 0u;
    size_t sample;

    if (session == NULL) {
        return result_from_session(
            NULL,
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            ACTUATOR_V2_CONTRACT_NULL_ARGUMENT);
    }
    if (!session->open) {
        return result_from_session(
            session,
            ACTUATOR_V2_STREAM_STATUS_NOT_OPEN,
            ACTUATOR_V2_CONTRACT_OK);
    }
    contract_result = actuator_v2_batch_decode(
        payload, payload_length, kind, &batch);
    if (contract_result != ACTUATOR_V2_CONTRACT_OK) {
        return result_from_session(
            session,
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            contract_result);
    }

    current_epoch = session->epoch_established ?
        session->arbiter_epoch : batch.arbiter_epoch;
    last_tick = session->validated_sample_count > 0u ?
        session->validated_samples[
            session->validated_sample_count - 1u].apply_tick : current_tick;

    if (kind == ACTUATOR_V2_BATCH_SPLICE) {
        if (!interpolate_position(session, batch.splice_at_tick,
                                  splice_position)) {
            return result_from_session(
                session,
                ACTUATOR_V2_STREAM_STATUS_SPLICE_POSITION_UNAVAILABLE,
                ACTUATOR_V2_CONTRACT_OK);
        }
    }
    contract_result = actuator_v2_batch_validate_transition(
        &batch,
        kind,
        &session->policy,
        current_tick,
        current_epoch,
        session->horizon_end_tick,
        last_tick,
        transition_position);
    if (contract_result != ACTUATOR_V2_CONTRACT_OK) {
        return result_from_session(
            session,
            ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
            contract_result);
    }

    if (kind == ACTUATOR_V2_BATCH_SPLICE) {
        while (retained_count < session->validated_sample_count &&
               tick_is_before(
                   session->validated_samples[retained_count].apply_tick,
                   batch.splice_at_tick)) {
            retained_count++;
        }
    } else {
        retained_count = session->validated_sample_count;
    }
    if ((size_t)retained_count + batch.sample_count >
        ACTUATOR_V2_QUEUE_CAPACITY) {
        return result_from_session(
            session,
            ACTUATOR_V2_STREAM_STATUS_QUEUE_OVERFLOW,
            ACTUATOR_V2_CONTRACT_OK);
    }
    for (sample = 0u; sample < batch.sample_count; ++sample) {
        session->validated_samples[retained_count + sample] =
            batch.samples[sample];
    }
    session->validated_sample_count =
        (uint8_t)(retained_count + batch.sample_count);
    session->epoch_established = true;
    session->arbiter_epoch = batch.arbiter_epoch;
    session->horizon_end_tick = batch.horizon_end_tick;
    return result_from_session(
        session,
        ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY,
        ACTUATOR_V2_CONTRACT_OK);
}
