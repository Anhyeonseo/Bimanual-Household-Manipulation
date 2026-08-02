#ifndef ACTUATOR_CORE_BUFFERED_EXECUTOR_H
#define ACTUATOR_CORE_BUFFERED_EXECUTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "actuator_core/setpoint_queue.h"

typedef enum {
    ACTUATOR_BUFFERED_PRIMING = 0,
    ACTUATOR_BUFFERED_RUNNING,
    ACTUATOR_BUFFERED_HOLD,
    ACTUATOR_BUFFERED_SUCCEEDED,
    ACTUATOR_BUFFERED_CANCELED,
    ACTUATOR_BUFFERED_ABORTED
} actuator_buffered_state_t;

typedef enum {
    ACTUATOR_BUFFERED_REASON_NONE = 0,
    ACTUATOR_BUFFERED_REASON_PLANNED_HOLD,
    ACTUATOR_BUFFERED_REASON_OPERATOR_CANCEL,
    ACTUATOR_BUFFERED_REASON_QUEUE_UNDERFLOW,
    ACTUATOR_BUFFERED_REASON_MISSED_APPLY_TICK,
    ACTUATOR_BUFFERED_REASON_CONNECTION_LOSS,
    ACTUATOR_BUFFERED_REASON_TRACKING_ERROR
} actuator_buffered_reason_t;

typedef enum {
    ACTUATOR_BUFFERED_OK = 0,
    ACTUATOR_BUFFERED_WAITING,
    ACTUATOR_BUFFERED_OUTPUT,
    ACTUATOR_BUFFERED_TERMINAL,
    ACTUATOR_BUFFERED_NULL_ARGUMENT,
    ACTUATOR_BUFFERED_BAD_STATE,
    ACTUATOR_BUFFERED_NOT_PRIMED,
    ACTUATOR_BUFFERED_QUEUE_REJECTED,
    ACTUATOR_BUFFERED_INVALID_ANCHOR
} actuator_buffered_result_t;

typedef struct {
    actuator_buffered_state_t state;
    actuator_buffered_reason_t reason;
    actuator_queue_result_t last_queue_result;
    size_t queued_samples;
    size_t peak_queued_samples;
    uint32_t accepted_samples;
    uint32_t applied_samples;
    uint32_t last_applied_tick;
    uint32_t terminal_tick;
    bool input_complete;
    bool safe_stop_required;
} actuator_buffered_diagnostics_t;

typedef struct {
    actuator_setpoint_queue_t queue;
    actuator_setpoint_t anchor;
    actuator_buffered_diagnostics_t diagnostics;
    size_t minimum_start_samples;
    bool anchor_valid;
} actuator_buffered_executor_t;

actuator_buffered_result_t actuator_buffered_executor_init(
    actuator_buffered_executor_t *executor,
    size_t minimum_start_samples);

actuator_buffered_result_t actuator_buffered_executor_admit_batch(
    actuator_buffered_executor_t *executor,
    const actuator_setpoint_t *samples,
    size_t sample_count,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]);

actuator_buffered_result_t actuator_buffered_executor_mark_input_complete(
    actuator_buffered_executor_t *executor);

actuator_buffered_result_t actuator_buffered_executor_start(
    actuator_buffered_executor_t *executor,
    uint32_t anchor_tick,
    const int32_t anchor_positions_urad[ACTUATOR_JOINT_COUNT],
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]);

actuator_buffered_result_t actuator_buffered_executor_step(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick,
    int32_t output_positions_urad[ACTUATOR_JOINT_COUNT]);

actuator_buffered_result_t actuator_buffered_executor_planned_hold(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick);

actuator_buffered_result_t actuator_buffered_executor_cancel(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick);

actuator_buffered_result_t actuator_buffered_executor_connection_loss(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick);

actuator_buffered_result_t actuator_buffered_executor_tracking_error(
    actuator_buffered_executor_t *executor,
    uint32_t current_tick);

const actuator_buffered_diagnostics_t *actuator_buffered_executor_diagnostics(
    const actuator_buffered_executor_t *executor);

#endif
