#ifndef ACTUATOR_CORE_STREAM_CONTRACT_V2_H
#define ACTUATOR_CORE_STREAM_CONTRACT_V2_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_V2_PROTOCOL_VERSION 2u
#define ACTUATOR_V2_MSG_STREAM_OPEN UINT8_C(40)
#define ACTUATOR_V2_MSG_STREAM_STATUS UINT8_C(41)
#define ACTUATOR_V2_MSG_SPLICE UINT8_C(42)
#define ACTUATOR_V2_MSG_GET_EXECUTOR_DIAGNOSTICS UINT8_C(43)
#define ACTUATOR_V2_MSG_EXECUTOR_DIAGNOSTICS UINT8_C(44)
#define ACTUATOR_V2_MSG_PREPARE_SHADOW UINT8_C(45)
#define ACTUATOR_V2_MSG_SHADOW_SNAPSHOT UINT8_C(46)
#define ACTUATOR_V2_MSG_GET_DISPATCH_DIAGNOSTICS UINT8_C(47)
#define ACTUATOR_V2_MSG_DISPATCH_DIAGNOSTICS UINT8_C(58)
#define ACTUATOR_V2_MSG_GET_TRACKING_DIAGNOSTICS UINT8_C(59)
#define ACTUATOR_V2_MSG_TRACKING_DIAGNOSTICS UINT8_C(60)
#define ACTUATOR_V2_MSG_GET_FEEDBACK_SNAPSHOT UINT8_C(61)
#define ACTUATOR_V2_MSG_FEEDBACK_SNAPSHOT UINT8_C(62)
#define ACTUATOR_V2_ARM_COUNT 2u
#define ACTUATOR_V2_ARM_JOINT_COUNT 6u
#define ACTUATOR_V2_JOINT_COUNT 12u
#define ACTUATOR_V2_QUEUE_CAPACITY 16u
#define ACTUATOR_V2_WIRE_MAX_SAMPLES 9u
#define ACTUATOR_V2_STREAM_OPEN_WIRE_SIZE 120u
#define ACTUATOR_V2_HELLO_WIRE_SIZE 24u
#define ACTUATOR_V2_STATE_WIRE_SIZE 24u
#define ACTUATOR_V2_STREAM_STATUS_WIRE_SIZE 36u
#define ACTUATOR_V2_EXECUTOR_DIAGNOSTICS_WIRE_SIZE 60u
#define ACTUATOR_V2_SHADOW_SNAPSHOT_WIRE_SIZE 76u
#define ACTUATOR_V2_DISPATCH_DIAGNOSTICS_WIRE_SIZE 44u
#define ACTUATOR_V2_TRACKING_DIAGNOSTICS_WIRE_SIZE 76u
#define ACTUATOR_V2_FEEDBACK_SNAPSHOT_WIRE_SIZE 116u
#define ACTUATOR_V2_BATCH_HEADER_SIZE 20u
#define ACTUATOR_V2_SAMPLE_WIRE_SIZE 52u
#define ACTUATOR_V2_MAX_BATCH_PAYLOAD_SIZE \
    (ACTUATOR_V2_BATCH_HEADER_SIZE + \
     (ACTUATOR_V2_WIRE_MAX_SAMPLES * ACTUATOR_V2_SAMPLE_WIRE_SIZE))
#define ACTUATOR_V2_MINIMUM_SPLICE_LEAD_MS UINT32_C(20)
#define ACTUATOR_V2_CONTROL_TICK_MS UINT32_C(5)

typedef enum {
    ACTUATOR_V2_STREAM_STATUS_OK = 0,
    ACTUATOR_V2_STREAM_STATUS_CONTRACT_REJECTED,
    ACTUATOR_V2_STREAM_STATUS_NOT_OPEN,
    ACTUATOR_V2_STREAM_STATUS_QUEUE_OVERFLOW,
    ACTUATOR_V2_STREAM_STATUS_SPLICE_POSITION_UNAVAILABLE,
    ACTUATOR_V2_STREAM_STATUS_VALIDATION_ONLY
} actuator_v2_stream_status_code_t;

#define ACTUATOR_V2_ARM_MASK_LEFT UINT8_C(0x01)
#define ACTUATOR_V2_ARM_MASK_RIGHT UINT8_C(0x02)
#define ACTUATOR_V2_ARM_MASK_BOTH UINT8_C(0x03)

typedef enum {
    ACTUATOR_V2_CONTRACT_OK = 0,
    ACTUATOR_V2_CONTRACT_NULL_ARGUMENT,
    ACTUATOR_V2_CONTRACT_INVALID_LENGTH,
    ACTUATOR_V2_CONTRACT_INVALID_ARM_MASK,
    ACTUATOR_V2_CONTRACT_INVALID_RESERVED,
    ACTUATOR_V2_CONTRACT_INVALID_MINIMUM_START_SAMPLES,
    ACTUATOR_V2_CONTRACT_MINIMUM_LEAD_TOO_SMALL,
    ACTUATOR_V2_CONTRACT_MAXIMUM_LEAD_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_LEAD_WINDOW_INVERTED,
    ACTUATOR_V2_CONTRACT_COMMAND_TIMEOUT_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_OPEN_TIMEOUT_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_APPLY_LATENESS_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_TRACKING_ERROR_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_MAXIMUM_STEP_TOO_LARGE,
    ACTUATOR_V2_CONTRACT_STALE_HORIZON,
    ACTUATOR_V2_CONTRACT_INVALID_SAMPLE_COUNT,
    ACTUATOR_V2_CONTRACT_NON_MONOTONIC_TICK,
    ACTUATOR_V2_CONTRACT_APPEND_EPOCH_MISMATCH,
    ACTUATOR_V2_CONTRACT_HORIZON_REGRESSION,
    ACTUATOR_V2_CONTRACT_HORIZON_BEFORE_LAST_SAMPLE,
    ACTUATOR_V2_CONTRACT_SPLICE_FIELD_MISMATCH,
    ACTUATOR_V2_CONTRACT_SPLICE_TOO_LATE,
    ACTUATOR_V2_CONTRACT_SPLICE_AFTER_LAST_SAMPLE,
    ACTUATOR_V2_CONTRACT_SPLICE_FIRST_TICK_MISMATCH,
    ACTUATOR_V2_CONTRACT_SPLICE_DISCONTINUITY,
    ACTUATOR_V2_CONTRACT_FIRST_SAMPLE_TOO_EARLY,
    ACTUATOR_V2_CONTRACT_LAST_SAMPLE_TOO_LATE,
    ACTUATOR_V2_CONTRACT_SAMPLE_DISCONTINUITY
} actuator_v2_contract_result_t;

typedef enum {
    ACTUATOR_V2_BATCH_APPEND = 0,
    ACTUATOR_V2_BATCH_SPLICE
} actuator_v2_batch_kind_t;

typedef struct {
    uint16_t minimum_start_samples;
    uint32_t minimum_lead_ms;
    uint32_t horizon_end_tick;
    uint32_t maximum_lead_ms;
    uint32_t command_timeout_ms;
    uint32_t maximum_apply_lateness_ms;
    int32_t tracking_error_limit_urad[ACTUATOR_V2_JOINT_COUNT];
    int32_t maximum_step_urad_per_tick[ACTUATOR_V2_JOINT_COUNT];
    uint8_t arm_mask;
} actuator_v2_stream_policy_t;

typedef struct {
    uint32_t minimum_lead_ms;
    uint32_t maximum_lead_ms;
    uint32_t maximum_command_timeout_ms;
    uint32_t maximum_open_command_timeout_ms;
    uint32_t maximum_apply_lateness_ms;
    int32_t tracking_error_limit_urad[ACTUATOR_V2_JOINT_COUNT];
    int32_t maximum_step_urad_per_tick[ACTUATOR_V2_JOINT_COUNT];
} actuator_v2_stream_hard_caps_t;

typedef struct {
    uint32_t apply_tick;
    int32_t position_urad[ACTUATOR_V2_JOINT_COUNT];
} actuator_v2_setpoint_t;

typedef struct {
    uint32_t first_apply_tick;
    uint32_t horizon_end_tick;
    uint32_t arbiter_epoch;
    uint32_t splice_at_tick;
    uint8_t sample_count;
    uint8_t arm_mask;
    actuator_v2_setpoint_t samples[ACTUATOR_V2_WIRE_MAX_SAMPLES];
} actuator_v2_batch_t;

actuator_v2_contract_result_t actuator_v2_stream_policy_decode(
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_stream_policy_t *policy);

actuator_v2_contract_result_t actuator_v2_stream_policy_validate(
    const actuator_v2_stream_policy_t *policy,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    uint32_t current_tick);

actuator_v2_contract_result_t actuator_v2_batch_decode(
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    actuator_v2_batch_t *batch);

actuator_v2_contract_result_t actuator_v2_batch_validate_transition(
    const actuator_v2_batch_t *batch,
    actuator_v2_batch_kind_t kind,
    const actuator_v2_stream_policy_t *policy,
    uint32_t current_tick,
    uint32_t current_epoch,
    uint32_t current_horizon_end_tick,
    uint32_t last_apply_tick,
    const int32_t interpolated_position_urad[ACTUATOR_V2_JOINT_COUNT]);

#endif
