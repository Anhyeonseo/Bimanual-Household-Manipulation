#ifndef ACTUATOR_CORE_STREAM_EXECUTOR_V2_H
#define ACTUATOR_CORE_STREAM_EXECUTOR_V2_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "actuator_core/stream_session_v2.h"

typedef struct {
    int32_t minimum_urad;
    int32_t maximum_urad;
} actuator_v2_joint_limit_t;

typedef enum {
    ACTUATOR_V2_EXECUTOR_CLOSED = 0,
    ACTUATOR_V2_EXECUTOR_PRIMING,
    ACTUATOR_V2_EXECUTOR_RUNNING,
    ACTUATOR_V2_EXECUTOR_HOLD,
    ACTUATOR_V2_EXECUTOR_SUCCEEDED,
    ACTUATOR_V2_EXECUTOR_ABORTED
} actuator_v2_executor_state_t;

typedef enum {
    ACTUATOR_V2_TERMINAL_NONE = 0,
    ACTUATOR_V2_TERMINAL_PLANNED_HORIZON,
    ACTUATOR_V2_TERMINAL_QUEUE_UNDERFLOW,
    ACTUATOR_V2_TERMINAL_COMMAND_TIMEOUT,
    ACTUATOR_V2_TERMINAL_MISSED_APPLY_TICK,
    ACTUATOR_V2_TERMINAL_TRACKING_ERROR,
    ACTUATOR_V2_TERMINAL_JOINT_LIMIT,
    ACTUATOR_V2_TERMINAL_INVALID_TIMELINE
} actuator_v2_terminal_reason_t;

typedef enum {
    ACTUATOR_V2_EXECUTOR_OK = 0,
    ACTUATOR_V2_EXECUTOR_WAITING,
    ACTUATOR_V2_EXECUTOR_OUTPUT_READY,
    ACTUATOR_V2_EXECUTOR_TERMINAL,
    ACTUATOR_V2_EXECUTOR_NULL_ARGUMENT,
    ACTUATOR_V2_EXECUTOR_BAD_STATE,
    ACTUATOR_V2_EXECUTOR_INVALID_LIMIT,
    ACTUATOR_V2_EXECUTOR_CONTRACT_REJECTED,
    ACTUATOR_V2_EXECUTOR_BATCH_REJECTED,
    ACTUATOR_V2_EXECUTOR_JOINT_LIMIT_REJECTED,
    ACTUATOR_V2_EXECUTOR_INSUFFICIENT_PRIME,
    ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR
} actuator_v2_executor_result_t;

typedef struct {
    actuator_v2_executor_state_t state;
    actuator_v2_terminal_reason_t terminal_reason;
    actuator_v2_stream_status_code_t last_stream_status;
    actuator_v2_contract_result_t last_contract_result;
    uint32_t accepted_samples;
    uint32_t applied_samples;
    uint32_t control_outputs;
    uint32_t splice_count;
    uint32_t last_command_tick;
    uint32_t terminal_tick;
    uint32_t maximum_apply_lateness_ms;
    uint32_t maximum_tracking_error_urad[ACTUATOR_V2_JOINT_COUNT];
    uint8_t queued_samples;
    uint8_t peak_queued_samples;
    uint8_t tracking_error_joint;
    bool safe_stop_required;
} actuator_v2_executor_diagnostics_t;

typedef struct {
    actuator_v2_stream_session_t session;
    actuator_v2_stream_hard_caps_t hard_caps;
    actuator_v2_joint_limit_t joint_limits[ACTUATOR_V2_JOINT_COUNT];
    actuator_v2_setpoint_t anchor;
    int32_t last_output_urad[ACTUATOR_V2_JOINT_COUNT];
    actuator_v2_executor_diagnostics_t diagnostics;
    uint32_t control_epoch_tick;
    uint32_t last_step_tick;
    bool anchor_valid;
    bool output_valid;
    bool last_step_valid;
} actuator_v2_stream_executor_t;

actuator_v2_executor_result_t actuator_v2_stream_executor_init(
    actuator_v2_stream_executor_t *executor,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    const actuator_v2_joint_limit_t joint_limits[ACTUATOR_V2_JOINT_COUNT]);

actuator_v2_executor_result_t actuator_v2_stream_executor_open(
    actuator_v2_stream_executor_t *executor,
    const uint8_t *payload,
    size_t payload_length,
    uint32_t current_tick);

actuator_v2_executor_result_t actuator_v2_stream_executor_admit(
    actuator_v2_stream_executor_t *executor,
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    uint32_t current_tick);

actuator_v2_executor_result_t actuator_v2_stream_executor_start(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    const int32_t position_urad[ACTUATOR_V2_JOINT_COUNT]);

actuator_v2_executor_result_t actuator_v2_stream_executor_step(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    int32_t output_urad[ACTUATOR_V2_JOINT_COUNT]);

actuator_v2_executor_result_t actuator_v2_stream_executor_check_feedback(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    const int32_t measured_urad[ACTUATOR_V2_JOINT_COUNT]);

actuator_v2_executor_result_t
actuator_v2_stream_executor_check_joint_feedback(
    actuator_v2_stream_executor_t *executor,
    uint32_t current_tick,
    uint8_t joint,
    int32_t commanded_urad,
    int32_t measured_urad);

const actuator_v2_executor_diagnostics_t *
actuator_v2_stream_executor_diagnostics(
    const actuator_v2_stream_executor_t *executor);

#endif
