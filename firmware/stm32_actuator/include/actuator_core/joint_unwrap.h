#ifndef ACTUATOR_CORE_JOINT_UNWRAP_H
#define ACTUATOR_CORE_JOINT_UNWRAP_H

#include <stdbool.h>
#include <stdint.h>

#define ACTUATOR_UNWRAP_RAW_MODULUS INT32_C(4096)
#define ACTUATOR_UNWRAP_HALF_TURN_RAW INT32_C(2048)

typedef enum {
    ACTUATOR_UNWRAP_OK = 0,
    ACTUATOR_UNWRAP_NULL_ARGUMENT,
    ACTUATOR_UNWRAP_RAW_OUT_OF_RANGE,
    ACTUATOR_UNWRAP_BAD_REFERENCE_WINDOW,
    ACTUATOR_UNWRAP_REFERENCE_TOO_FAR,
    ACTUATOR_UNWRAP_AMBIGUOUS_DELTA,
    ACTUATOR_UNWRAP_BAD_DIRECTION,
    ACTUATOR_UNWRAP_LIMIT_VIOLATION,
    ACTUATOR_UNWRAP_OVERFLOW
} actuator_unwrap_result_t;

typedef struct {
    bool bound;
    uint16_t previous_raw;
    int32_t unwrapped_raw;
} actuator_joint_unwrapper_t;

typedef struct {
    int32_t minimum_unwrapped_raw;
    int32_t maximum_unwrapped_raw;
} actuator_unwrapped_joint_limit_t;

void actuator_joint_unwrapper_reset(actuator_joint_unwrapper_t *state);

actuator_unwrap_result_t actuator_joint_unwrapper_bind(
    actuator_joint_unwrapper_t *state,
    uint16_t observed_raw,
    int32_t reference_unwrapped_raw,
    uint16_t maximum_reference_delta_raw);

actuator_unwrap_result_t actuator_joint_unwrapper_update(
    actuator_joint_unwrapper_t *state,
    uint16_t observed_raw,
    int32_t *unwrapped_raw);

actuator_unwrap_result_t actuator_unwrapped_raw_to_urad(
    uint16_t zero_raw,
    int8_t positive_raw_direction,
    int32_t unwrapped_raw,
    int32_t *position_urad);

actuator_unwrap_result_t actuator_unwrapped_urad_to_raw(
    uint16_t zero_raw,
    int8_t positive_raw_direction,
    int32_t position_urad,
    const actuator_unwrapped_joint_limit_t *limit,
    int32_t *unwrapped_raw,
    uint16_t *modulo_raw);

#endif
