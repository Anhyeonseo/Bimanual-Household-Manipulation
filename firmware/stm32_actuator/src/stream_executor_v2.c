#include "actuator_core/stream_executor_v2.h"

#include <string.h>

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

static bool tick_is_before(uint32_t candidate, uint32_t reference) {
    return tick_is_after(reference, candidate);
}

static uint32_t first_control_tick_at_or_after(
    const actuator_v2_stream_executor_t *executor,
    uint32_t apply_tick) {
    const uint32_t elapsed = apply_tick - executor->control_epoch_tick;
    const uint32_t control_ticks =
        (elapsed + ACTUATOR_V2_CONTROL_TICK_MS - 1u) /
        ACTUATOR_V2_CONTROL_TICK_MS;
    return executor->control_epoch_tick +
        control_ticks * ACTUATOR_V2_CONTROL_TICK_MS;
}

static bool positions_within_limits(
    const actuator_v2_stream_executor_t *executor,
    const int32_t positions[ACTUATOR_V2_JOINT_COUNT]) {
    size_t joint;

    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        if (positions[joint] < executor->joint_limits[joint].minimum_urad ||
            positions[joint] > executor->joint_limits[joint].maximum_urad) {
            return false;
        }
    }
    return true;
}

static void update_queue_diagnostics(actuator_v2_stream_executor_t *executor) {
    executor->diagnostics.queued_samples =
        executor->session.validated_sample_count;
    if (executor->diagnostics.queued_samples >
        executor->diagnostics.peak_queued_samples) {
        executor->diagnostics.peak_queued_samples =
            executor->diagnostics.queued_samples;
    }
}

static uint32_t absolute_difference(int32_t left, int32_t right) {
    int64_t difference = (int64_t)left - (int64_t)right;
    if (difference < 0) {
        difference = -difference;
    }
    return (uint32_t)difference;
}

static void record_output(
    actuator_v2_stream_executor_t *executor,
    const int32_t output_urad[ACTUATOR_V2_JOINT_COUNT]) {
    memcpy(executor->last_output_urad, output_urad,
           sizeof(executor->last_output_urad));
    executor->output_valid = true;
    executor->diagnostics.control_outputs++;
}

static actuator_v2_executor_result_t terminate(
    actuator_v2_stream_executor_t *executor,
    actuator_v2_executor_state_t state,
    actuator_v2_terminal_reason_t reason,
    uint32_t current_tick,
    bool safe_stop_required) {
    executor->diagnostics.state = state;
    executor->diagnostics.terminal_reason = reason;
    executor->diagnostics.terminal_tick = current_tick;
    executor->diagnostics.safe_stop_required = safe_stop_required;
    return ACTUATOR_V2_EXECUTOR_TERMINAL;
}

static void consume_first_sample(actuator_v2_stream_executor_t *executor) {
    actuator_v2_stream_session_t *session = &executor->session;

    if (session->validated_sample_count > 1u) {
        memmove(
            &session->validated_samples[0],
            &session->validated_samples[1],
            ((size_t)session->validated_sample_count - 1u) *
                sizeof(session->validated_samples[0]));
    }
    if (session->validated_sample_count > 0u) {
        session->validated_sample_count--;
    }
    update_queue_diagnostics(executor);
}

actuator_v2_executor_result_t actuator_v2_stream_executor_init(
    actuator_v2_stream_executor_t *executor,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    const actuator_v2_joint_limit_t joint_limits[ACTUATOR_V2_JOINT_COUNT]) {
    size_t joint;

    if (executor == NULL || hard_caps == NULL || joint_limits == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        if (joint_limits[joint].minimum_urad >=
            joint_limits[joint].maximum_urad) {
            return ACTUATOR_V2_EXECUTOR_INVALID_LIMIT;
        }
    }

    memset(executor, 0, sizeof(*executor));
    executor->hard_caps = *hard_caps;
    memcpy(executor->joint_limits, joint_limits,
           sizeof(executor->joint_limits));
    actuator_v2_stream_session_init(&executor->session);
    executor->diagnostics.state = ACTUATOR_V2_EXECUTOR_CLOSED;
    executor->diagnostics.last_stream_status =
        ACTUATOR_V2_STREAM_STATUS_NOT_OPEN;
    return ACTUATOR_V2_EXECUTOR_OK;
}

actuator_v2_executor_result_t actuator_v2_stream_executor_open(
    actuator_v2_stream_executor_t *executor,
    const uint8_t *payload,
    size_t payload_length,
    uint32_t current_tick) {
    actuator_v2_stream_session_result_t result;

    if (executor == NULL || payload == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state == ACTUATOR_V2_EXECUTOR_RUNNING) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }

    result = actuator_v2_stream_session_open(
        &executor->session,
        payload,
        payload_length,
        &executor->hard_caps,
        current_tick);
    executor->diagnostics.last_stream_status = result.status_code;
    executor->diagnostics.last_contract_result = result.contract_result;
    if (result.contract_result != ACTUATOR_V2_CONTRACT_OK) {
        return ACTUATOR_V2_EXECUTOR_CONTRACT_REJECTED;
    }

    memset(&executor->anchor, 0, sizeof(executor->anchor));
    memset(&executor->diagnostics, 0, sizeof(executor->diagnostics));
    executor->diagnostics.state = ACTUATOR_V2_EXECUTOR_PRIMING;
    executor->diagnostics.last_stream_status = result.status_code;
    executor->diagnostics.last_contract_result = result.contract_result;
    executor->anchor_valid = false;
    executor->output_valid = false;
    executor->last_step_valid = false;
    return ACTUATOR_V2_EXECUTOR_OK;
}

actuator_v2_executor_result_t actuator_v2_stream_executor_admit(
    actuator_v2_stream_executor_t *executor,
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    uint32_t current_tick) {
    actuator_v2_batch_t decoded;
    actuator_v2_contract_result_t decode_result;
    actuator_v2_stream_session_result_t session_result;
    size_t sample;

    if (executor == NULL || payload == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_PRIMING &&
        executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_RUNNING) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }

    decode_result = actuator_v2_batch_decode(
        payload, payload_length, kind, &decoded);
    if (decode_result != ACTUATOR_V2_CONTRACT_OK) {
        executor->diagnostics.last_contract_result = decode_result;
        return ACTUATOR_V2_EXECUTOR_CONTRACT_REJECTED;
    }
    for (sample = 0u; sample < decoded.sample_count; ++sample) {
        if (!positions_within_limits(
                executor, decoded.samples[sample].position_urad)) {
            return ACTUATOR_V2_EXECUTOR_JOINT_LIMIT_REJECTED;
        }
    }
    if (kind == ACTUATOR_V2_BATCH_APPEND &&
        executor->session.validated_sample_count > 0u) {
        const actuator_v2_setpoint_t *tail =
            &executor->session.validated_samples[
                executor->session.validated_sample_count - 1u];
        if (tick_is_after(decoded.first_apply_tick, tail->apply_tick)) {
            const uint32_t delta_ms =
                decoded.first_apply_tick - tail->apply_tick;
            if ((delta_ms % ACTUATOR_V2_CONTROL_TICK_MS) != 0u) {
                executor->diagnostics.last_contract_result =
                    ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY;
                return ACTUATOR_V2_EXECUTOR_BATCH_REJECTED;
            }
            for (size_t joint = 0u;
                 joint < ACTUATOR_V2_JOINT_COUNT;
                 ++joint) {
                const uint32_t difference = absolute_difference(
                    decoded.samples[0].position_urad[joint],
                    tail->position_urad[joint]);
                const uint64_t maximum_difference =
                    (uint64_t)executor->session.policy
                        .maximum_step_urad_per_tick[joint] *
                    (delta_ms / ACTUATOR_V2_CONTROL_TICK_MS);
                if ((uint64_t)difference > maximum_difference) {
                    executor->diagnostics.last_contract_result =
                        ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY;
                    return ACTUATOR_V2_EXECUTOR_BATCH_REJECTED;
                }
            }
        }
    }

    session_result = actuator_v2_stream_session_batch(
        &executor->session, payload, payload_length, kind, current_tick);
    executor->diagnostics.last_stream_status = session_result.status_code;
    executor->diagnostics.last_contract_result =
        session_result.contract_result;
    if (session_result.contract_result != ACTUATOR_V2_CONTRACT_OK ||
        session_result.status_code !=
            ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY) {
        return ACTUATOR_V2_EXECUTOR_BATCH_REJECTED;
    }

    executor->diagnostics.accepted_samples += decoded.sample_count;
    if (kind == ACTUATOR_V2_BATCH_SPLICE) {
        executor->diagnostics.splice_count++;
    }
    executor->diagnostics.last_command_tick = current_tick;
    update_queue_diagnostics(executor);
    return ACTUATOR_V2_EXECUTOR_OK;
}

actuator_v2_executor_result_t actuator_v2_stream_executor_start(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    const int32_t position_urad[ACTUATOR_V2_JOINT_COUNT]) {
    if (executor == NULL || position_urad == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_PRIMING) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }
    if (executor->session.validated_sample_count <
        executor->session.policy.minimum_start_samples) {
        return ACTUATOR_V2_EXECUTOR_INSUFFICIENT_PRIME;
    }
    if (!positions_within_limits(executor, position_urad) ||
        !tick_is_after(
            executor->session.validated_samples[0].apply_tick,
            current_tick)) {
        return ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR;
    }
    {
        const uint32_t span =
            executor->session.validated_samples[0].apply_tick - current_tick;
        const uint32_t output_ticks =
            (span + ACTUATOR_V2_CONTROL_TICK_MS - 1u) /
            ACTUATOR_V2_CONTROL_TICK_MS;
        size_t joint;
        for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
            const uint32_t difference = absolute_difference(
                executor->session.validated_samples[0].position_urad[joint],
                position_urad[joint]);
            const uint64_t maximum_difference =
                (uint64_t)executor->session.policy
                    .maximum_step_urad_per_tick[joint] * output_ticks;
            if ((uint64_t)difference > maximum_difference) {
                return ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR;
            }
        }
    }

    executor->anchor.apply_tick = current_tick;
    memcpy(executor->anchor.position_urad, position_urad,
           sizeof(executor->anchor.position_urad));
    executor->anchor_valid = true;
    executor->control_epoch_tick = current_tick;
    executor->last_step_tick = current_tick;
    executor->last_step_valid = true;
    executor->diagnostics.state = ACTUATOR_V2_EXECUTOR_RUNNING;
    return ACTUATOR_V2_EXECUTOR_OK;
}

actuator_v2_executor_result_t actuator_v2_stream_executor_step(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    int32_t output_urad[ACTUATOR_V2_JOINT_COUNT]) {
    actuator_v2_setpoint_t *next;
    uint32_t due_tick;
    uint32_t span;
    uint32_t elapsed;
    size_t joint;

    if (executor == NULL || output_urad == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state == ACTUATOR_V2_EXECUTOR_PRIMING) {
        return ACTUATOR_V2_EXECUTOR_WAITING;
    }
    if (executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_RUNNING ||
        !executor->anchor_valid) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }
    if ((executor->last_step_valid &&
         tick_is_before(current_tick, executor->last_step_tick)) ||
        tick_is_before(current_tick, executor->anchor.apply_tick)) {
        return terminate(
            executor,
            ACTUATOR_V2_EXECUTOR_ABORTED,
            ACTUATOR_V2_TERMINAL_INVALID_TIMELINE,
            current_tick,
            true);
    }
    executor->last_step_tick = current_tick;
    executor->last_step_valid = true;

    if (executor->session.horizon_end_tick == 0u &&
        (current_tick - executor->diagnostics.last_command_tick) >
            executor->session.policy.command_timeout_ms) {
        memcpy(output_urad, executor->anchor.position_urad,
               sizeof(executor->anchor.position_urad));
        record_output(executor, output_urad);
        return terminate(
            executor,
            ACTUATOR_V2_EXECUTOR_HOLD,
            ACTUATOR_V2_TERMINAL_COMMAND_TIMEOUT,
            current_tick,
            true);
    }

    if (executor->session.validated_sample_count == 0u) {
        if (executor->session.horizon_end_tick != 0u) {
            return terminate(
                executor,
                ACTUATOR_V2_EXECUTOR_ABORTED,
                ACTUATOR_V2_TERMINAL_QUEUE_UNDERFLOW,
                current_tick,
                true);
        }
        memcpy(output_urad, executor->anchor.position_urad,
               sizeof(executor->anchor.position_urad));
        record_output(executor, output_urad);
        return ACTUATOR_V2_EXECUTOR_OUTPUT_READY;
    }

    next = &executor->session.validated_samples[0];
    due_tick = first_control_tick_at_or_after(executor, next->apply_tick);
    if (!tick_is_before(current_tick, due_tick)) {
        const uint32_t lateness = current_tick - due_tick;
        if (lateness >
            executor->session.policy.maximum_apply_lateness_ms) {
            return terminate(
                executor,
                ACTUATOR_V2_EXECUTOR_ABORTED,
                ACTUATOR_V2_TERMINAL_MISSED_APPLY_TICK,
                current_tick,
                true);
        }
        if (lateness > executor->diagnostics.maximum_apply_lateness_ms) {
            executor->diagnostics.maximum_apply_lateness_ms = lateness;
        }
        executor->anchor = *next;
        memcpy(output_urad, next->position_urad,
               sizeof(next->position_urad));
        consume_first_sample(executor);
        executor->diagnostics.applied_samples++;
        record_output(executor, output_urad);
        if (executor->session.horizon_end_tick != 0u &&
            executor->anchor.apply_tick ==
                executor->session.horizon_end_tick &&
            executor->session.validated_sample_count == 0u) {
            return terminate(
                executor,
                ACTUATOR_V2_EXECUTOR_SUCCEEDED,
                ACTUATOR_V2_TERMINAL_PLANNED_HORIZON,
                current_tick,
                false);
        }
        return ACTUATOR_V2_EXECUTOR_OUTPUT_READY;
    }

    span = next->apply_tick - executor->anchor.apply_tick;
    elapsed = current_tick - executor->anchor.apply_tick;
    if (span == 0u || elapsed > span) {
        return terminate(
            executor,
            ACTUATOR_V2_EXECUTOR_ABORTED,
            ACTUATOR_V2_TERMINAL_INVALID_TIMELINE,
            current_tick,
            true);
    }
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        const int64_t delta =
            (int64_t)next->position_urad[joint] -
            (int64_t)executor->anchor.position_urad[joint];
        output_urad[joint] = executor->anchor.position_urad[joint] +
            (int32_t)((delta * (int64_t)elapsed) / (int64_t)span);
    }
    record_output(executor, output_urad);
    return ACTUATOR_V2_EXECUTOR_OUTPUT_READY;
}

actuator_v2_executor_result_t
actuator_v2_stream_executor_check_joint_feedback(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    uint8_t joint,
    int32_t commanded_urad,
    int32_t measured_urad) {
    uint32_t error;

    if (executor == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (joint >= ACTUATOR_V2_JOINT_COUNT ||
        ((executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_RUNNING) &&
         (executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_SUCCEEDED)) ||
        !executor->output_valid) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }
    error = absolute_difference(commanded_urad, measured_urad);
    if (error > executor->diagnostics.maximum_tracking_error_urad[joint]) {
        executor->diagnostics.maximum_tracking_error_urad[joint] = error;
    }
    if (error >
        (uint32_t)executor->session.policy.tracking_error_limit_urad[joint]) {
        executor->diagnostics.tracking_error_joint = joint;
        return terminate(
            executor,
            ACTUATOR_V2_EXECUTOR_ABORTED,
            ACTUATOR_V2_TERMINAL_TRACKING_ERROR,
            current_tick,
            true);
    }
    return ACTUATOR_V2_EXECUTOR_OK;
}

actuator_v2_executor_result_t actuator_v2_stream_executor_check_feedback(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    const int32_t measured_urad[ACTUATOR_V2_JOINT_COUNT]) {
    size_t joint;

    if (executor == NULL || measured_urad == NULL) {
        return ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state != ACTUATOR_V2_EXECUTOR_RUNNING ||
        !executor->output_valid) {
        return ACTUATOR_V2_EXECUTOR_BAD_STATE;
    }
    for (joint = 0u; joint < ACTUATOR_V2_JOINT_COUNT; ++joint) {
        const uint32_t error = absolute_difference(
            executor->last_output_urad[joint], measured_urad[joint]);
        if (error > executor->diagnostics.maximum_tracking_error_urad[joint]) {
            executor->diagnostics.maximum_tracking_error_urad[joint] = error;
        }
        if (error >
            (uint32_t)executor->session.policy.tracking_error_limit_urad[joint]) {
            executor->diagnostics.tracking_error_joint = (uint8_t)joint;
            return terminate(
                executor,
                ACTUATOR_V2_EXECUTOR_ABORTED,
                ACTUATOR_V2_TERMINAL_TRACKING_ERROR,
                current_tick,
                true);
        }
    }
    return ACTUATOR_V2_EXECUTOR_OK;
}

const actuator_v2_executor_diagnostics_t *
actuator_v2_stream_executor_diagnostics(
    const actuator_v2_stream_executor_t *executor) {
    return executor == NULL ? NULL : &executor->diagnostics;
}
