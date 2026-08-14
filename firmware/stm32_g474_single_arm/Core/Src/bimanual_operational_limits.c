#include "bimanual_operational_limits.h"

#include <stddef.h>
#include <string.h>

/*
 * Canonical source:
 * config/bimanual_operational_limits.json
 *
 * These are inclusive limits from the complete operator-traversed J0-D
 * workspace. Shoulder entries use unwrapped raw coordinates so 4095 -> 0 is
 * continuous. Gripper limits are servo-command bounds, not jaw-gap geometry.
 */
static const BimanualOperationalLimit operational_limits
    [BIMANUAL_ARM_COUNT][BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT / 2U] = {
    [BIMANUAL_ARM_LEFT] = {
        {2048U,  1, { 983, 3041}, {-1633689, 1523243}},
        {2048U,  1, {1899, 4187}, { -228563, 3281185}},
        {2048U, -1, { 286, 2492}, { -681087, 2702874}},
        {2048U, -1, { 170, 2384}, { -515418, 2880816}},
        {2048U,  1, { 587, 2838}, {-2241146, 1211845}},
        {2048U, -1, {1872, 3257}, {-1854583,  269981}},
    },
    [BIMANUAL_ARM_RIGHT] = {
        {2048U,  1, {1108, 2996}, {-1441942, 1454214}},
        {2048U,  1, {1859, 4188}, { -289922, 3282719}},
        {2048U, -1, { 297, 2523}, { -728641, 2686000}},
        {2048U, -1, { 377, 2438}, { -598252, 2563282}},
        {2048U,  1, { 749, 2970}, {-1992641, 1414330}},
        {2048U, -1, {1907, 3299}, {-1919010,  216291}},
    },
};

/* Archived table for reproducible 0x00024100 no-output validation only. */
static const actuator_v2_joint_limit_t j1l_shadow_limits[
    BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT] = {
    {-1535515, 1425068},
    {-130388, 3183010},
    {-582913, 2604699},
    {-417243, 2782641},
    {-2142971, 1113670},
    {-6400000, 6400000},
    {-1343767, 1356039},
    {-191748, 3184544},
    {-630466, 2587825},
    {-500078, 2465107},
    {-1894466, 1316155},
    {-6400000, 6400000},
};

const BimanualOperationalLimit *BimanualOperationalLimits_Get(
    BimanualArm arm,
    uint8_t joint_index
)
{
    if ((arm >= BIMANUAL_ARM_COUNT) ||
        (joint_index >= (BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT / 2U)))
    {
        return NULL;
    }
    return &operational_limits[arm][joint_index];
}

bool BimanualOperationalLimits_ContainsUnwrappedRaw(
    BimanualArm arm,
    uint8_t joint_index,
    int32_t unwrapped_raw
)
{
    const BimanualOperationalLimit *limit =
        BimanualOperationalLimits_Get(arm, joint_index);
    return (limit != NULL) &&
        (unwrapped_raw >= limit->raw.minimum_unwrapped_raw) &&
        (unwrapped_raw <= limit->raw.maximum_unwrapped_raw);
}

bool BimanualOperationalLimits_UnwrapModuloRaw(
    BimanualArm arm,
    uint8_t joint_index,
    uint16_t modulo_raw,
    int32_t *unwrapped_raw
)
{
    int32_t match = 0;
    uint8_t match_count = 0U;

    if ((unwrapped_raw == NULL) ||
        (modulo_raw >= ACTUATOR_UNWRAP_RAW_MODULUS))
    {
        return false;
    }
    for (int32_t turn = -1; turn <= 1; turn++)
    {
        const int32_t candidate = (int32_t)modulo_raw +
            (turn * ACTUATOR_UNWRAP_RAW_MODULUS);
        if (BimanualOperationalLimits_ContainsUnwrappedRaw(
                arm, joint_index, candidate))
        {
            match = candidate;
            match_count++;
        }
    }
    if (match_count != 1U)
    {
        return false;
    }
    *unwrapped_raw = match;
    return true;
}

bool BimanualOperationalLimits_StepModuloRaw(
    BimanualArm arm,
    uint8_t joint_index,
    uint16_t present_modulo_raw,
    int16_t delta_raw,
    uint16_t *target_modulo_raw
)
{
    int32_t present_unwrapped = 0;
    int32_t target_unwrapped;
    int32_t target_modulo;

    if ((target_modulo_raw == NULL) ||
        !BimanualOperationalLimits_UnwrapModuloRaw(
            arm, joint_index, present_modulo_raw, &present_unwrapped))
    {
        return false;
    }
    target_unwrapped = present_unwrapped + (int32_t)delta_raw;
    if (!BimanualOperationalLimits_ContainsUnwrappedRaw(
            arm, joint_index, target_unwrapped))
    {
        return false;
    }
    target_modulo = target_unwrapped % ACTUATOR_UNWRAP_RAW_MODULUS;
    if (target_modulo < 0)
    {
        target_modulo += ACTUATOR_UNWRAP_RAW_MODULUS;
    }
    *target_modulo_raw = (uint16_t)target_modulo;
    return true;
}

void BimanualOperationalLimits_LoadExecutorLimits(
    actuator_v2_joint_limit_t
        limits[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]
)
{
    if (limits == NULL)
    {
        return;
    }
    for (uint8_t arm = 0U; arm < BIMANUAL_ARM_COUNT; arm++)
    {
        for (uint8_t joint = 0U;
             joint < (BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT / 2U);
             joint++)
        {
            limits[(arm * 6U) + joint] =
                operational_limits[arm][joint].urad;
        }
    }
}

#if HOST_BIMANUAL_DISPATCH_REFACTOR_BUILD
void BimanualOperationalLimits_LoadGoalMaps(
    actuator_bimanual_joint_map_t
        maps[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]
)
{
    if (maps == NULL)
    {
        return;
    }
    for (uint8_t arm = 0U; arm < BIMANUAL_ARM_COUNT; arm++)
    {
        for (uint8_t joint = 0U;
             joint < ACTUATOR_BIMANUAL_ARM_JOINT_COUNT;
             joint++)
        {
            const BimanualOperationalLimit *limit =
                &operational_limits[arm][joint];
            actuator_bimanual_joint_map_t *map =
                &maps[(arm * ACTUATOR_BIMANUAL_ARM_JOINT_COUNT) + joint];
            map->zero_raw = limit->zero_raw;
            map->positive_raw_direction = limit->positive_raw_direction;
            map->minimum_unwrapped_raw = limit->raw.minimum_unwrapped_raw;
            map->maximum_unwrapped_raw = limit->raw.maximum_unwrapped_raw;
        }
    }
}

actuator_bimanual_goal_map_result_t
BimanualOperationalLimits_MapExecutorOutput(
    const int32_t positions_urad[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT],
    uint16_t left_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint16_t right_raw[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT],
    uint8_t *failed_joint
)
{
    actuator_bimanual_joint_map_t
        maps[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT];

    BimanualOperationalLimits_LoadGoalMaps(maps);
    return actuator_bimanual_goal_map(
        maps,
        positions_urad,
        left_raw,
        right_raw,
        failed_joint
    );
}
#endif

void BimanualOperationalLimits_LoadJ1LShadow(
    actuator_v2_joint_limit_t
        limits[BIMANUAL_OPERATIONAL_LIMIT_JOINT_COUNT]
)
{
    if (limits == NULL)
    {
        return;
    }
    memcpy(limits, j1l_shadow_limits, sizeof(j1l_shadow_limits));
}
