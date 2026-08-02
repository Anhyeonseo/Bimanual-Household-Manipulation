#include "actuator_core/setpoint_queue.h"

#include <stdbool.h>
#include <stddef.h>

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

static bool positions_are_valid(
    const actuator_setpoint_t *sample,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    size_t joint;
    for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
        if (limits[joint].minimum_urad > limits[joint].maximum_urad ||
            sample->position_urad[joint] < limits[joint].minimum_urad ||
            sample->position_urad[joint] > limits[joint].maximum_urad) {
            return false;
        }
    }
    return true;
}

void actuator_setpoint_queue_init(actuator_setpoint_queue_t *queue) {
    actuator_setpoint_queue_clear(queue);
}

void actuator_setpoint_queue_clear(actuator_setpoint_queue_t *queue) {
    if (queue != NULL) {
        queue->head = 0u;
        queue->count = 0u;
    }
}

actuator_queue_result_t actuator_setpoint_queue_push_batch(
    actuator_setpoint_queue_t *queue,
    const actuator_setpoint_t *samples,
    size_t sample_count,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    size_t index;
    uint32_t previous_tick;

    if (queue == NULL || samples == NULL || limits == NULL || sample_count == 0u) {
        return ACTUATOR_QUEUE_NULL_ARGUMENT;
    }
    if (sample_count > ACTUATOR_SETPOINT_QUEUE_CAPACITY - queue->count) {
        return ACTUATOR_QUEUE_CAPACITY_EXCEEDED;
    }
    if (minimum_lead_ticks > maximum_lead_ticks) {
        return ACTUATOR_QUEUE_TICK_TOO_FAR;
    }

    if (queue->count > 0u) {
        const size_t tail = (queue->head + queue->count - 1u) % ACTUATOR_SETPOINT_QUEUE_CAPACITY;
        previous_tick = queue->samples[tail].apply_tick;
    } else {
        previous_tick = current_tick;
    }

    for (index = 0u; index < sample_count; ++index) {
        const uint32_t lead = samples[index].apply_tick - current_tick;
        if (!tick_is_after(samples[index].apply_tick, current_tick) || lead < minimum_lead_ticks) {
            return ACTUATOR_QUEUE_STALE_TICK;
        }
        if (lead > maximum_lead_ticks) {
            return ACTUATOR_QUEUE_TICK_TOO_FAR;
        }
        if (!tick_is_after(samples[index].apply_tick, previous_tick)) {
            return ACTUATOR_QUEUE_NON_MONOTONIC;
        }
        if (!positions_are_valid(&samples[index], limits)) {
            return ACTUATOR_QUEUE_LIMIT_VIOLATION;
        }
        previous_tick = samples[index].apply_tick;
    }

    for (index = 0u; index < sample_count; ++index) {
        const size_t destination = (queue->head + queue->count) % ACTUATOR_SETPOINT_QUEUE_CAPACITY;
        queue->samples[destination] = samples[index];
        ++queue->count;
    }
    return ACTUATOR_QUEUE_OK;
}

actuator_queue_result_t actuator_setpoint_queue_take_due(
    actuator_setpoint_queue_t *queue,
    uint32_t current_tick,
    actuator_setpoint_t *sample) {
    const actuator_setpoint_t *front;

    if (queue == NULL || sample == NULL) {
        return ACTUATOR_QUEUE_NULL_ARGUMENT;
    }
    if (queue->count == 0u) {
        return ACTUATOR_QUEUE_EMPTY;
    }

    front = &queue->samples[queue->head];
    if (front->apply_tick != current_tick) {
        if (tick_is_after(front->apply_tick, current_tick)) {
            return ACTUATOR_QUEUE_NOT_DUE;
        }
        return ACTUATOR_QUEUE_STALE_TICK;
    }

    *sample = *front;
    queue->head = (queue->head + 1u) % ACTUATOR_SETPOINT_QUEUE_CAPACITY;
    --queue->count;
    return ACTUATOR_QUEUE_OK;
}

actuator_queue_result_t actuator_setpoint_queue_peek(
    const actuator_setpoint_queue_t *queue,
    actuator_setpoint_t *sample) {
    if (queue == NULL || sample == NULL) {
        return ACTUATOR_QUEUE_NULL_ARGUMENT;
    }
    if (queue->count == 0u) {
        return ACTUATOR_QUEUE_EMPTY;
    }

    *sample = queue->samples[queue->head];
    return ACTUATOR_QUEUE_OK;
}
