#ifndef ACTUATOR_CORE_BIMANUAL_GOAL_MAP_H
#define ACTUATOR_CORE_BIMANUAL_GOAL_MAP_H

#include <stdint.h>

#define ACTUATOR_BIMANUAL_JOINT_COUNT UINT8_C(12)
#define ACTUATOR_BIMANUAL_ARM_JOINT_COUNT UINT8_C(6)

typedef struct {
    uint16_t zero_raw;
    int8_t positive_raw_direction;
    int32_t minimum_unwrapped_raw;
    int32_t maximum_unwrapped_raw;
} actuator_bimanual_joint_map_t;

typedef enum {
    ACTUATOR_BIMANUAL_GOAL_MAP_OK = 0,
    ACTUATOR_BIMANUAL_GOAL_MAP_NULL_ARGUMENT,
    ACTUATOR_BIMANUAL_GOAL_MAP_INVALID_CONFIG,
    ACTUATOR_BIMANUAL_GOAL_MAP_LIMIT_VIOLATION
} actuator_bimanual_goal_map_result_t;

/*
 * Convert one absolute 12-joint executor output into the two six-servo raw
 * arrays consumed by the arm buses. Validation is atomic: neither destination
 * is modified unless all twelve joints pass their unwrapped limits.
 */
actuator_bimanual_goal_map_result_t actuator_bimanual_goal_map(
    const actuator_bimanual_joint_map_t
        maps[ACTUATOR_BIMANUAL_JOINT_COUNT],
    const int32_t positions_urad[ACTUATOR_BIMANUAL_JOINT_COUNT],
    uint16_t left_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint16_t right_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint8_t *failed_joint);

#endif
