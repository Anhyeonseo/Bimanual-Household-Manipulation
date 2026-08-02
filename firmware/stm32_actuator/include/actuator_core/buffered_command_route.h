#ifndef ACTUATOR_CORE_BUFFERED_COMMAND_ROUTE_H
#define ACTUATOR_CORE_BUFFERED_COMMAND_ROUTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "actuator_core/buffered_executor.h"

#define ACTUATOR_BUFFERED_WIRE_MAX_SAMPLES 9u
#define ACTUATOR_BUFFERED_WIRE_HEADER_SIZE 8u
#define ACTUATOR_BUFFERED_WIRE_SAMPLE_SIZE 52u
#define ACTUATOR_BUFFERED_STATUS_BASE_SIZE 16u
#define ACTUATOR_BUFFERED_STATUS_EXTENDED_SIZE 32u

#define ACTUATOR_BUFFERED_FLAG_VALIDATION_ONLY UINT16_C(0x0001)
#define ACTUATOR_BUFFERED_FLAG_CANDIDATE UINT16_C(0x0002)
#define ACTUATOR_BUFFERED_FLAG_BEGIN UINT16_C(0x0004)
#define ACTUATOR_BUFFERED_FLAG_START UINT16_C(0x0008)
#define ACTUATOR_BUFFERED_FLAG_END UINT16_C(0x0010)
#define ACTUATOR_BUFFERED_FLAG_MASK UINT16_C(0x001F)

typedef enum {
    ACTUATOR_BUFFERED_COMMAND_OK = 0,
    ACTUATOR_BUFFERED_COMMAND_NULL_ARGUMENT,
    ACTUATOR_BUFFERED_COMMAND_INVALID_LENGTH,
    ACTUATOR_BUFFERED_COMMAND_INVALID_FLAGS,
    ACTUATOR_BUFFERED_COMMAND_INVALID_ARM_MASK,
    ACTUATOR_BUFFERED_COMMAND_INVALID_RESERVED,
    ACTUATOR_BUFFERED_COMMAND_NON_MONOTONIC_TICK,
    ACTUATOR_BUFFERED_COMMAND_UNSUPPORTED_RIGHT_SLOT,
    ACTUATOR_BUFFERED_COMMAND_BAD_STATE,
    ACTUATOR_BUFFERED_COMMAND_QUEUE_REJECTED
} actuator_buffered_command_result_t;

typedef struct {
    uint16_t flags;
    uint32_t first_apply_tick;
    uint8_t sample_count;
    actuator_setpoint_t samples[ACTUATOR_BUFFERED_WIRE_MAX_SAMPLES];
} actuator_buffered_command_t;

typedef struct {
    actuator_buffered_executor_t executor;
    actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT];
    uint32_t trajectory_sequence;
    uint32_t admitted_batches;
    bool trajectory_active;
    bool start_requested;
    bool started;
} actuator_buffered_command_route_t;

actuator_buffered_command_result_t actuator_buffered_command_decode(
    const uint8_t *payload,
    size_t payload_length,
    uint16_t frame_flags,
    actuator_buffered_command_t *command);

actuator_buffered_result_t actuator_buffered_command_route_init(
    actuator_buffered_command_route_t *route,
    size_t minimum_start_samples,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]);

actuator_buffered_command_result_t actuator_buffered_command_route_admit(
    actuator_buffered_command_route_t *route,
    const actuator_buffered_command_t *command,
    uint32_t request_sequence,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks);

actuator_buffered_result_t actuator_buffered_command_route_start(
    actuator_buffered_command_route_t *route,
    uint32_t anchor_tick,
    const int32_t anchor_positions_urad[ACTUATOR_JOINT_COUNT]);

actuator_buffered_result_t actuator_buffered_command_route_step(
    actuator_buffered_command_route_t *route,
    uint32_t current_tick,
    int32_t output_positions_urad[ACTUATOR_JOINT_COUNT]);

actuator_buffered_result_t actuator_buffered_command_route_planned_hold(
    actuator_buffered_command_route_t *route,
    uint32_t current_tick);
actuator_buffered_result_t actuator_buffered_command_route_cancel(
    actuator_buffered_command_route_t *route,
    uint32_t current_tick);
actuator_buffered_result_t actuator_buffered_command_route_connection_loss(
    actuator_buffered_command_route_t *route,
    uint32_t current_tick);
actuator_buffered_result_t actuator_buffered_command_route_tracking_error(
    actuator_buffered_command_route_t *route,
    uint32_t current_tick);

bool actuator_buffered_status_encode(
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length,
    uint8_t status_code,
    uint8_t sample_count,
    uint8_t safety_state,
    uint8_t detail,
    uint32_t request_sequence,
    uint32_t apply_tick,
    uint32_t calibration_hash,
    const actuator_buffered_diagnostics_t *diagnostics);

#endif
