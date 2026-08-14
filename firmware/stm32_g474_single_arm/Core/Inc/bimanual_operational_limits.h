#ifndef BIMANUAL_OPERATIONAL_LIMITS_H
#define BIMANUAL_OPERATIONAL_LIMITS_H

#include "actuator_core/bimanual_goal_map.h"
#include "actuator_core/joint_unwrap.h"
#include "actuator_core/stream_executor_v2.h"

#include <stdbool.h>
#include <stdint.h>

#define BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT UINT8_C(12)

typedef enum
{
    BIMANUAL_ARM_LEFT = 0,
    BIMANUAL_ARM_RIGHT = 1,
    BIMANUAL_ARM_COUNT = 2
} BimanualArm;

typedef struct
{
    uint16_t zero_raw;
    int8_t positive_raw_direction;
    actuator_unwrapped_joint_limit_t raw;
    actuator_v2_joint_limit_t urad;
} BimanualOperationalLimit;

/*
 * Runtime limits are the complete operator-traversed J0-D task envelope.
 * Calibration identity remains separate: changing an operational envelope
 * must not silently change servo identity, PID, physical zero, or direction.
 */
const BimanualOperationalLimit *BimanualOperationalLimits_Get(
    BimanualArm arm,
    uint8_t joint_index
);

bool BimanualOperationalLimits_ContainsUnwrappedRaw(
    BimanualArm arm,
    uint8_t joint_index,
    int32_t unwrapped_raw
);

bool BimanualOperationalLimits_UnwrapModuloRaw(
    BimanualArm arm,
    uint8_t joint_index,
    uint16_t modulo_raw,
    int32_t *unwrapped_raw
);

bool BimanualOperationalLimits_StepModuloRaw(
    BimanualArm arm,
    uint8_t joint_index,
    uint16_t present_modulo_raw,
    int16_t delta_raw,
    uint16_t *target_modulo_raw
);

/* Historical no-output J1-L candidate; never used by runtime output. */
void BimanualOperationalLimits_LoadJ1LShadow(
    actuator_v2_joint_limit_t
        limits[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]
);

void BimanualOperationalLimits_LoadExecutorLimits(
    actuator_v2_joint_limit_t
        limits[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]);

void BimanualOperationalLimits_LoadGoalMaps(
    actuator_bimanual_joint_map_t
        maps[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]);

actuator_bimanual_goal_map_result_t
BimanualOperationalLimits_MapExecutorOutput(
    const int32_t positions_urad[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT],
    uint16_t left_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint16_t right_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint8_t *failed_joint);

#endif
