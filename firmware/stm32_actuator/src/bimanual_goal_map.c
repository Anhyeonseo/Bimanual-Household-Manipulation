#include "actuator_core/bimanual_goal_map.h"

#include "actuator_core/calibration.h"
#include "actuator_core/joint_unwrap.h"

#include <stddef.h>
#include <string.h>

static int64_t round_divide(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }
    return -((-numerator + denominator / 2) / denominator);
}

static int32_t modulo_raw(int32_t unwrapped_raw) {
    int32_t result = unwrapped_raw % ACTUATOR_UNWRAP_RAW_MODULUS;
    if (result < 0) {
        result += ACTUATOR_UNWRAP_RAW_MODULUS;
    }
    return result;
}

actuator_bimanual_goal_map_result_t actuator_bimanual_goal_map(
    const actuator_bimanual_joint_map_t
        maps[ACTUATOR_BIMANUAL_JOINT_COUNT],
    const int32_t positions_urad[ACTUATOR_BIMANUAL_JOINT_COUNT],
    uint16_t left_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint16_t right_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint8_t *failed_joint) {
    uint16_t candidate[ACTUATOR_BIMANUAL_JOINT_COUNT];
    uint8_t joint;

    if (maps == NULL || positions_urad == NULL || left_raw == NULL ||
        right_raw == NULL) {
        return ACTUATOR_BIMANUAL_GOAL_MAP_NULL_ARGUMENT;
    }

    for (joint = 0u; joint < ACTUATOR_BIMANUAL_JOINT_COUNT; ++joint) {
        const actuator_bimanual_joint_map_t *map = &maps[joint];
        const int64_t raw_delta = round_divide(
            (int64_t)positions_urad[joint] * ACTUATOR_RAW_UNITS_PER_TURN,
            ACTUATOR_TURN_URAD);
        const int64_t unwrapped_raw =
            (int64_t)map->zero_raw +
            (int64_t)map->positive_raw_direction * raw_delta;

        if (map->zero_raw >= ACTUATOR_UNWRAP_RAW_MODULUS ||
            (map->positive_raw_direction != 1 &&
             map->positive_raw_direction != -1) ||
            map->minimum_unwrapped_raw > map->maximum_unwrapped_raw) {
            if (failed_joint != NULL) {
                *failed_joint = joint;
            }
            return ACTUATOR_BIMANUAL_GOAL_MAP_INVALID_CONFIG;
        }
        if (unwrapped_raw < map->minimum_unwrapped_raw ||
            unwrapped_raw > map->maximum_unwrapped_raw) {
            if (failed_joint != NULL) {
                *failed_joint = joint;
            }
            return ACTUATOR_BIMANUAL_GOAL_MAP_LIMIT_VIOLATION;
        }
        candidate[joint] = (uint16_t)modulo_raw((int32_t)unwrapped_raw);
    }

    memcpy(left_raw, candidate,
           sizeof(uint16_t) * ACTUATOR_BIMANUAL_ARM_JOINT_COUNT);
    memcpy(right_raw,
           &candidate[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
           sizeof(uint16_t) * ACTUATOR_BIMANUAL_ARM_JOINT_COUNT);
    if (failed_joint != NULL) {
        *failed_joint = UINT8_MAX;
    }
    return ACTUATOR_BIMANUAL_GOAL_MAP_OK;
}
