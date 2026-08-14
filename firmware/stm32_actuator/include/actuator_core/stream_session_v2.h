#ifndef ACTUATOR_CORE_STREAM_SESSION_V2_H
#define ACTUATOR_CORE_STREAM_SESSION_V2_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "actuator_core/stream_contract_v2.h"

typedef struct {
    actuator_v2_stream_status_code_t status_code;
    actuator_v2_contract_result_t contract_result;
    uint8_t arm_mask;
    uint32_t arbiter_epoch;
    uint32_t horizon_end_tick;
    uint32_t validated_tail_tick;
    uint8_t validated_sample_count;
} actuator_v2_stream_session_result_t;

typedef struct {
    bool open;
    bool epoch_established;
    actuator_v2_stream_policy_t policy;
    uint32_t arbiter_epoch;
    uint32_t horizon_end_tick;
    actuator_v2_setpoint_t validated_samples[ACTUATOR_V2_QUEUE_CAPACITY];
    uint8_t validated_sample_count;
} actuator_v2_stream_session_t;

void actuator_v2_stream_session_init(actuator_v2_stream_session_t *session);

actuator_v2_stream_session_result_t actuator_v2_stream_session_open(
    actuator_v2_stream_session_t *session,
    const uint8_t *payload,
    size_t payload_length,
    const actuator_v2_stream_hard_caps_t *hard_caps,
    uint32_t current_tick);

actuator_v2_stream_session_result_t actuator_v2_stream_session_batch(
    actuator_v2_stream_session_t *session,
    const uint8_t *payload,
    size_t payload_length,
    actuator_v2_batch_kind_t kind,
    uint32_t current_tick);

#endif
