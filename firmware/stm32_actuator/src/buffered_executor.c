#include "actuator_core/buffered_executor.h"

#include <limits.h>
#include <string.h>

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

static bool state_accepts_commands(actuator_buffered_state_t state) {
    return state == ACTUATOR_BUFFERED_PRIMING ||
           state == ACTUATOR_BUFFERED_RUNNING;
}

static void update_queue_depth(actuator_buffered_executor_t *executor) {
    executor->diagnostics.queued_samples = executor->queue.count;
    if (executor->queue.count > executor->diagnostics.peak_queued_samples) {
        executor->diagnostics.peak_queued_samples = executor->queue.count;
    }
}

static actuator_buffered_result_t transition_terminal(
    actuator_buffered_executor_t *executor,
    actuator_buffered_state_t state,
    actuator_buffered_reason_t reason,
    bool safe_stop_required,
    uint32_t current_tick) {
    actuator_setpoint_queue_clear(&executor->queue);
    executor->diagnostics.state = state;
    executor->diagnostics.reason = reason;
    executor->diagnostics.safe_stop_required = safe_stop_required;
    executor->diagnostics.terminal_tick = current_tick;
    update_queue_depth(executor);
    return ACTUATOR_BUFFERED_TERMINAL;
}

actuator_buffered_result_t actuator_buffered_executor_init(
    actuator_buffered_executor_t *executor,
    size_t minimum_start_samples,
    uint32_t maximum_apply_lateness_ticks) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (minimum_start_samples == 0u ||
        minimum_start_samples > ACTUATOR_SETPOINT_QUEUE_CAPACITY) {
        return ACTUATOR_BUFFERED_NOT_PRIMED;
    }

    memset(executor, 0, sizeof(*executor));
    actuator_setpoint_queue_init(&executor->queue);
    executor->minimum_start_samples = minimum_start_samples;
    executor->maximum_apply_lateness_ticks = maximum_apply_lateness_ticks;
    executor->diagnostics.state = ACTUATOR_BUFFERED_PRIMING;
    executor->diagnostics.reason = ACTUATOR_BUFFERED_REASON_NONE;
    executor->diagnostics.last_queue_result = ACTUATOR_QUEUE_OK;
    return ACTUATOR_BUFFERED_OK;
}

actuator_buffered_result_t actuator_buffered_executor_admit_batch(
    actuator_buffered_executor_t *executor,
    const actuator_setpoint_t *samples,
    size_t sample_count,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    actuator_queue_result_t queue_result;

    if (executor == NULL || samples == NULL || limits == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state) ||
        executor->diagnostics.input_complete) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }

    queue_result = actuator_setpoint_queue_push_batch(
        &executor->queue,
        samples,
        sample_count,
        current_tick,
        minimum_lead_ticks,
        maximum_lead_ticks,
        limits);
    executor->diagnostics.last_queue_result = queue_result;
    update_queue_depth(executor);
    if (queue_result != ACTUATOR_QUEUE_OK) {
        return ACTUATOR_BUFFERED_QUEUE_REJECTED;
    }

    if (sample_count > UINT32_MAX - executor->diagnostics.accepted_samples) {
        executor->diagnostics.accepted_samples = UINT32_MAX;
    } else {
        executor->diagnostics.accepted_samples += (uint32_t)sample_count;
    }
    return ACTUATOR_BUFFERED_OK;
}

actuator_buffered_result_t actuator_buffered_executor_mark_input_complete(
    actuator_buffered_executor_t *executor) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state)) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    executor->diagnostics.input_complete = true;
    return ACTUATOR_BUFFERED_OK;
}

actuator_buffered_result_t actuator_buffered_executor_start(
    actuator_buffered_executor_t *executor,
    uint32_t anchor_tick,
    const int32_t anchor_positions_urad[ACTUATOR_JOINT_COUNT],
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    actuator_setpoint_t first;
    size_t joint;

    if (executor == NULL || anchor_positions_urad == NULL || limits == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state != ACTUATOR_BUFFERED_PRIMING) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    if (executor->queue.count == 0u ||
        (executor->queue.count < executor->minimum_start_samples &&
         !executor->diagnostics.input_complete)) {
        return ACTUATOR_BUFFERED_NOT_PRIMED;
    }
    if (actuator_setpoint_queue_peek(&executor->queue, &first) !=
            ACTUATOR_QUEUE_OK ||
        !tick_is_after(first.apply_tick, anchor_tick)) {
        return ACTUATOR_BUFFERED_INVALID_ANCHOR;
    }
    for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
        if (limits[joint].minimum_urad > limits[joint].maximum_urad ||
            anchor_positions_urad[joint] < limits[joint].minimum_urad ||
            anchor_positions_urad[joint] > limits[joint].maximum_urad) {
            return ACTUATOR_BUFFERED_INVALID_ANCHOR;
        }
    }

    executor->anchor.apply_tick = anchor_tick;
    memcpy(
        executor->anchor.position_urad,
        anchor_positions_urad,
        sizeof(executor->anchor.position_urad));
    executor->anchor_valid = true;
    executor->diagnostics.state = ACTUATOR_BUFFERED_RUNNING;
    return ACTUATOR_BUFFERED_OK;
}

actuator_buffered_result_t actuator_buffered_executor_step(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick,
    int32_t output_positions_urad[ACTUATOR_JOINT_COUNT]) {
    actuator_setpoint_t next;
    uint32_t apply_lateness;
    uint32_t segment_ticks;
    uint32_t elapsed_ticks;
    size_t joint;

    if (executor == NULL || output_positions_urad == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (executor->diagnostics.state != ACTUATOR_BUFFERED_RUNNING ||
        !executor->anchor_valid) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    if (executor->queue.count == 0u) {
        if (executor->diagnostics.input_complete) {
            return transition_terminal(
                executor,
                ACTUATOR_BUFFERED_SUCCEEDED,
                ACTUATOR_BUFFERED_REASON_NONE,
                false,
                current_tick);
        }
        return transition_terminal(
            executor,
            ACTUATOR_BUFFERED_HOLD,
            ACTUATOR_BUFFERED_REASON_QUEUE_UNDERFLOW,
            true,
            current_tick);
    }
    if (actuator_setpoint_queue_peek(&executor->queue, &next) !=
        ACTUATOR_QUEUE_OK) {
        return ACTUATOR_BUFFERED_QUEUE_REJECTED;
    }

    if (current_tick == next.apply_tick ||
        tick_is_after(current_tick, next.apply_tick)) {
        apply_lateness = current_tick - next.apply_tick;
        if (apply_lateness > executor->maximum_apply_lateness_ticks) {
            return transition_terminal(
                executor,
                ACTUATOR_BUFFERED_HOLD,
                ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK,
                true,
                current_tick);
        }
        actuator_queue_result_t take_result = actuator_setpoint_queue_take_due(
            &executor->queue,
            next.apply_tick,
            &next);
        executor->diagnostics.last_queue_result = take_result;
        if (take_result != ACTUATOR_QUEUE_OK) {
            return ACTUATOR_BUFFERED_QUEUE_REJECTED;
        }
        memcpy(
            output_positions_urad,
            next.position_urad,
            sizeof(next.position_urad));
        executor->anchor = next;
        executor->diagnostics.last_applied_tick = current_tick;
        executor->diagnostics.last_apply_lateness_ticks = apply_lateness;
        if (executor->diagnostics.applied_samples < UINT32_MAX) {
            executor->diagnostics.applied_samples++;
        }
        if (apply_lateness >
            executor->diagnostics.maximum_apply_lateness_ticks) {
            executor->diagnostics.maximum_apply_lateness_ticks =
                apply_lateness;
            /* Counted after the increment so the index is 1-based and matches
             * the applied sample the operator sees in the plan. */
            executor->diagnostics.maximum_apply_lateness_sample_index =
                executor->diagnostics.applied_samples;
        }
        {
            size_t bucket = (size_t)apply_lateness;
            if (bucket >= ACTUATOR_BUFFERED_LATENESS_BUCKETS) {
                bucket = ACTUATOR_BUFFERED_LATENESS_BUCKETS - 1u;
            }
            if (executor->diagnostics.apply_lateness_histogram[bucket] <
                UINT32_MAX) {
                executor->diagnostics.apply_lateness_histogram[bucket]++;
            }
        }
        update_queue_depth(executor);
        if (executor->queue.count == 0u) {
            if (executor->diagnostics.input_complete) {
                executor->diagnostics.state = ACTUATOR_BUFFERED_SUCCEEDED;
                executor->diagnostics.terminal_tick = current_tick;
            } else {
                (void)transition_terminal(
                    executor,
                    ACTUATOR_BUFFERED_HOLD,
                    ACTUATOR_BUFFERED_REASON_QUEUE_UNDERFLOW,
                    true,
                    current_tick);
            }
        }
        return ACTUATOR_BUFFERED_OUTPUT;
    }
    if (current_tick != executor->anchor.apply_tick &&
        !tick_is_after(current_tick, executor->anchor.apply_tick)) {
        return ACTUATOR_BUFFERED_INVALID_ANCHOR;
    }

    segment_ticks = next.apply_tick - executor->anchor.apply_tick;
    elapsed_ticks = current_tick - executor->anchor.apply_tick;
    if (elapsed_ticks > segment_ticks) {
        return transition_terminal(
            executor,
            ACTUATOR_BUFFERED_HOLD,
            ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK,
            true,
            current_tick);
    }

    for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
        const int64_t start = executor->anchor.position_urad[joint];
        const int64_t delta =
            (int64_t)next.position_urad[joint] - start;
        output_positions_urad[joint] = (int32_t)(
            start + (delta * (int64_t)elapsed_ticks) /
                        (int64_t)segment_ticks);
    }
    return ACTUATOR_BUFFERED_OUTPUT;
}

actuator_buffered_result_t actuator_buffered_executor_planned_hold(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state)) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    return transition_terminal(
        executor,
        ACTUATOR_BUFFERED_HOLD,
        ACTUATOR_BUFFERED_REASON_PLANNED_HOLD,
        false,
        current_tick);
}

actuator_buffered_result_t actuator_buffered_executor_cancel(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state)) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    return transition_terminal(
        executor,
        ACTUATOR_BUFFERED_CANCELED,
        ACTUATOR_BUFFERED_REASON_OPERATOR_CANCEL,
        true,
        current_tick);
}

actuator_buffered_result_t actuator_buffered_executor_connection_loss(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state)) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    return transition_terminal(
        executor,
        ACTUATOR_BUFFERED_ABORTED,
        ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS,
        true,
        current_tick);
}

actuator_buffered_result_t actuator_buffered_executor_tracking_error(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick) {
    if (executor == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!state_accepts_commands(executor->diagnostics.state)) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    return transition_terminal(
        executor,
        ACTUATOR_BUFFERED_ABORTED,
        ACTUATOR_BUFFERED_REASON_TRACKING_ERROR,
        true,
        current_tick);
}

const actuator_buffered_diagnostics_t *actuator_buffered_executor_diagnostics(
    const actuator_buffered_executor_t *executor) {
    if (executor == NULL) {
        return NULL;
    }
    return &executor->diagnostics;
}
