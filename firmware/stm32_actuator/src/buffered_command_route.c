#include "actuator_core/buffered_command_route.h"

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

static void write_u16_le(uint8_t *destination, uint16_t value) {
    destination[0] = (uint8_t)(value & UINT16_C(0x00FF));
    destination[1] = (uint8_t)(value >> 8u);
}

static void write_u32_le(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)(value & UINT32_C(0x000000FF));
    destination[1] = (uint8_t)((value >> 8u) & UINT32_C(0x000000FF));
    destination[2] = (uint8_t)((value >> 16u) & UINT32_C(0x000000FF));
    destination[3] = (uint8_t)(value >> 24u);
}

static bool tick_is_after(uint32_t candidate, uint32_t reference) {
    return (int32_t)(candidate - reference) > 0;
}

actuator_buffered_command_result_t actuator_buffered_command_decode(
    const uint8_t *payload,
    size_t payload_length,
    uint16_t frame_flags,
    actuator_buffered_command_t *command) {
    uint8_t sample_count;
    size_t expected_length;
    uint32_t previous_tick = 0u;
    size_t sample_index;

    if (payload == NULL || command == NULL) {
        return ACTUATOR_BUFFERED_COMMAND_NULL_ARGUMENT;
    }
    if (payload_length < ACTUATOR_BUFFERED_WIRE_HEADER_SIZE) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH;
    }
    sample_count = payload[4];
    if (sample_count == 0u ||
        sample_count > ACTUATOR_BUFFERED_WIRE_MAX_SAMPLES) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH;
    }
    expected_length = ACTUATOR_BUFFERED_WIRE_HEADER_SIZE +
        ((size_t)sample_count * ACTUATOR_BUFFERED_WIRE_SAMPLE_SIZE);
    if (payload_length != expected_length) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH;
    }
    if ((frame_flags & ACTUATOR_BUFFERED_FLAG_CANDIDATE) == 0u ||
        (frame_flags & (uint16_t)(~ACTUATOR_BUFFERED_FLAG_MASK)) != 0u) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS;
    }
    if (payload[5] != 1u) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_ARM_MASK;
    }
    if (read_u16_le(&payload[6]) != 0u) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_RESERVED;
    }

    memset(command, 0, sizeof(*command));
    command->flags = frame_flags;
    command->first_apply_tick = read_u32_le(payload);
    command->sample_count = sample_count;

    for (sample_index = 0u; sample_index < sample_count; ++sample_index) {
        const size_t offset = ACTUATOR_BUFFERED_WIRE_HEADER_SIZE +
            (sample_index * ACTUATOR_BUFFERED_WIRE_SAMPLE_SIZE);
        const uint32_t tick_offset = read_u32_le(&payload[offset]);
        const uint32_t apply_tick = command->first_apply_tick + tick_offset;
        size_t joint;

        if (sample_index > 0u && !tick_is_after(apply_tick, previous_tick)) {
            return ACTUATOR_BUFFERED_COMMAND_NON_MONOTONIC_TICK;
        }
        previous_tick = apply_tick;
        command->samples[sample_index].apply_tick = apply_tick;

        for (joint = 0u; joint < ACTUATOR_JOINT_COUNT; ++joint) {
            command->samples[sample_index].position_urad[joint] =
                (int32_t)read_u32_le(&payload[offset + 4u + (joint * 4u)]);
            if (read_u32_le(&payload[offset + 28u + (joint * 4u)]) != 0u) {
                return ACTUATOR_BUFFERED_COMMAND_UNSUPPORTED_RIGHT_SLOT;
            }
        }
    }
    return ACTUATOR_BUFFERED_COMMAND_OK;
}

actuator_buffered_result_t actuator_buffered_command_route_init(
    actuator_buffered_command_route_t *route,
    size_t minimum_start_samples,
    uint32_t maximum_apply_lateness_ticks,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]) {
    actuator_buffered_result_t result;
    if (route == NULL || limits == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    memset(route, 0, sizeof(*route));
    result = actuator_buffered_executor_init(
        &route->executor,
        minimum_start_samples,
        maximum_apply_lateness_ticks);
    if (result != ACTUATOR_BUFFERED_OK) {
        return result;
    }
    memcpy(route->limits, limits, sizeof(route->limits));
    return ACTUATOR_BUFFERED_OK;
}

actuator_buffered_command_result_t actuator_buffered_command_route_admit(
    actuator_buffered_command_route_t *route,
    const actuator_buffered_command_t *command,
    uint32_t request_sequence,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks) {
    const bool validation_only = command != NULL &&
        (command->flags & ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY) != 0u;
    const bool begin = command != NULL &&
        (command->flags & ACTUATOR_BUFFERED_FLAG_BEGIN) != 0u;
    const bool start = command != NULL &&
        (command->flags & ACTUATOR_BUFFERED_FLAG_START) != 0u;
    const bool end = command != NULL &&
        (command->flags & ACTUATOR_BUFFERED_FLAG_END) != 0u;
    actuator_buffered_result_t result;

    if (route == NULL || command == NULL) {
        return ACTUATOR_BUFFERED_COMMAND_NULL_ARGUMENT;
    }
    if ((command->flags & ACTUATOR_BUFFERED_FLAG_CANDIDATE) == 0u ||
        (command->flags & (uint16_t)(~ACTUATOR_BUFFERED_FLAG_MASK)) != 0u) {
        return ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS;
    }
    if ((begin && route->trajectory_active) ||
        (!begin && !route->trajectory_active) ||
        (start && route->start_requested)) {
        return ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }

    if (validation_only) {
        const actuator_setpoint_queue_t saved_queue = route->executor.queue;
        const actuator_buffered_diagnostics_t saved_diagnostics =
            route->executor.diagnostics;
        result = actuator_buffered_executor_admit_batch(
            &route->executor, command->samples, command->sample_count,
            current_tick, minimum_lead_ticks, maximum_lead_ticks, route->limits);
        route->executor.queue = saved_queue;
        route->executor.diagnostics = saved_diagnostics;
        return result == ACTUATOR_BUFFERED_OK ?
            ACTUATOR_BUFFERED_COMMAND_OK : ACTUATOR_BUFFERED_COMMAND_QUEUE_REJECTED;
    }

    result = actuator_buffered_executor_admit_batch(
        &route->executor, command->samples, command->sample_count,
        current_tick, minimum_lead_ticks, maximum_lead_ticks, route->limits);
    if (result != ACTUATOR_BUFFERED_OK) {
        return ACTUATOR_BUFFERED_COMMAND_QUEUE_REJECTED;
    }
    if (begin) {
        route->trajectory_active = true;
        route->trajectory_sequence = request_sequence;
    }
    if (start) {
        route->start_requested = true;
    }
    if (end && actuator_buffered_executor_mark_input_complete(&route->executor) !=
                   ACTUATOR_BUFFERED_OK) {
        return ACTUATOR_BUFFERED_COMMAND_BAD_STATE;
    }
    route->admitted_batches++;
    return ACTUATOR_BUFFERED_COMMAND_OK;
}

actuator_buffered_result_t actuator_buffered_command_route_start(
    actuator_buffered_command_route_t *route,
    uint32_t anchor_tick,
    const int32_t anchor_positions_urad[ACTUATOR_JOINT_COUNT]) {
    actuator_buffered_result_t result;
    if (route == NULL || anchor_positions_urad == NULL) {
        return ACTUATOR_BUFFERED_NULL_ARGUMENT;
    }
    if (!route->trajectory_active || !route->start_requested || route->started) {
        return ACTUATOR_BUFFERED_BAD_STATE;
    }
    result = actuator_buffered_executor_start(
        &route->executor, anchor_tick, anchor_positions_urad, route->limits);
    if (result == ACTUATOR_BUFFERED_OK) {
        route->started = true;
    }
    return result;
}

actuator_buffered_result_t actuator_buffered_command_route_step(
    actuator_buffered_command_route_t *route, uint32_t current_tick,
    int32_t output_positions_urad[ACTUATOR_JOINT_COUNT]) {
    return route == NULL ? ACTUATOR_BUFFERED_NULL_ARGUMENT :
        actuator_buffered_executor_step(&route->executor, current_tick,
                                        output_positions_urad);
}

actuator_buffered_result_t actuator_buffered_command_route_planned_hold(
    actuator_buffered_command_route_t *route, uint32_t current_tick) {
    return route == NULL ? ACTUATOR_BUFFERED_NULL_ARGUMENT :
        actuator_buffered_executor_planned_hold(&route->executor, current_tick);
}

actuator_buffered_result_t actuator_buffered_command_route_cancel(
    actuator_buffered_command_route_t *route, uint32_t current_tick) {
    return route == NULL ? ACTUATOR_BUFFERED_NULL_ARGUMENT :
        actuator_buffered_executor_cancel(&route->executor, current_tick);
}

actuator_buffered_result_t actuator_buffered_command_route_connection_loss(
    actuator_buffered_command_route_t *route, uint32_t current_tick) {
    return route == NULL ? ACTUATOR_BUFFERED_NULL_ARGUMENT :
        actuator_buffered_executor_connection_loss(&route->executor, current_tick);
}

actuator_buffered_result_t actuator_buffered_command_route_tracking_error(
    actuator_buffered_command_route_t *route, uint32_t current_tick) {
    return route == NULL ? ACTUATOR_BUFFERED_NULL_ARGUMENT :
        actuator_buffered_executor_tracking_error(&route->executor, current_tick);
}

bool actuator_buffered_status_encode(
    uint8_t *output, size_t output_capacity, size_t *output_length,
    uint8_t status_code, uint8_t sample_count, uint8_t safety_state,
    uint8_t detail, uint32_t request_sequence, uint32_t apply_tick,
    uint32_t calibration_hash,
    const actuator_buffered_diagnostics_t *diagnostics) {
    uint16_t queued;
    uint16_t peak;
    if (output == NULL || output_length == NULL || diagnostics == NULL ||
        output_capacity < ACTUATOR_BUFFERED_STATUS_LATENESS_SIZE) {
        return false;
    }
    memset(output, 0, ACTUATOR_BUFFERED_STATUS_LATENESS_SIZE);
    output[0] = status_code;
    output[1] = sample_count;
    output[2] = safety_state;
    output[3] = detail;
    write_u32_le(&output[4], request_sequence);
    write_u32_le(&output[8], apply_tick);
    write_u32_le(&output[12], calibration_hash);
    output[16] = (uint8_t)diagnostics->state;
    output[17] = (uint8_t)diagnostics->reason;
    output[18] = diagnostics->safe_stop_required ? 1u : 0u;
    output[19] = (uint8_t)diagnostics->last_queue_result;
    queued = diagnostics->queued_samples > UINT16_MAX ?
        UINT16_MAX : (uint16_t)diagnostics->queued_samples;
    peak = diagnostics->peak_queued_samples > UINT16_MAX ?
        UINT16_MAX : (uint16_t)diagnostics->peak_queued_samples;
    write_u16_le(&output[20], queued);
    write_u16_le(&output[22], peak);
    write_u32_le(&output[24], diagnostics->accepted_samples);
    write_u32_le(&output[28], diagnostics->applied_samples);
    for (size_t bucket = 0u;
         bucket < ACTUATOR_BUFFERED_LATENESS_BUCKETS;
         bucket++) {
        write_u32_le(
            &output[32u + (bucket * 4u)],
            diagnostics->apply_lateness_histogram[bucket]);
    }
    write_u32_le(
        &output[56], diagnostics->maximum_apply_lateness_sample_index);
    *output_length = ACTUATOR_BUFFERED_STATUS_LATENESS_SIZE;
    return true;
}
