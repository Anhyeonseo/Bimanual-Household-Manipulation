#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "actuator_core/bimanual_goal_map.h"

static int failures = 0;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; return; } } while (0)

static void fill_maps(
    actuator_bimanual_joint_map_t maps[ACTUATOR_BIMANUAL_JOINT_COUNT]) {
    uint8_t joint;
    for (joint = 0u; joint < ACTUATOR_BIMANUAL_JOINT_COUNT; ++joint) {
        maps[joint].zero_raw = 2048u;
        maps[joint].positive_raw_direction = 1;
        maps[joint].minimum_unwrapped_raw = 0;
        maps[joint].maximum_unwrapped_raw = 4095;
    }
}

static void test_zero_splits_atomically(void) {
    actuator_bimanual_joint_map_t maps[ACTUATOR_BIMANUAL_JOINT_COUNT];
    int32_t goals[ACTUATOR_BIMANUAL_JOINT_COUNT] = {0};
    uint16_t left[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};
    uint16_t right[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};
    uint8_t failed = 0u;
    uint8_t joint;

    fill_maps(maps);
    CHECK(actuator_bimanual_goal_map(
              maps, goals, left, right, &failed) ==
          ACTUATOR_BIMANUAL_GOAL_MAP_OK);
    CHECK(failed == UINT8_MAX);
    for (joint = 0u; joint < ACTUATOR_BIMANUAL_ARM_JOINT_COUNT; ++joint) {
        CHECK(left[joint] == 2048u);
        CHECK(right[joint] == 2048u);
    }
}

static void test_wrap_and_negative_direction(void) {
    actuator_bimanual_joint_map_t maps[ACTUATOR_BIMANUAL_JOINT_COUNT];
    int32_t goals[ACTUATOR_BIMANUAL_JOINT_COUNT] = {0};
    uint16_t left[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};
    uint16_t right[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};

    fill_maps(maps);
    maps[1].minimum_unwrapped_raw = 1899;
    maps[1].maximum_unwrapped_raw = 4187;
    goals[1] = 3281185;
    maps[2].positive_raw_direction = -1;
    maps[2].minimum_unwrapped_raw = 286;
    maps[2].maximum_unwrapped_raw = 2492;
    goals[2] = 2702874;

    CHECK(actuator_bimanual_goal_map(
              maps, goals, left, right, NULL) ==
          ACTUATOR_BIMANUAL_GOAL_MAP_OK);
    CHECK(left[1] == 91u);
    CHECK(left[2] == 286u);
}

static void test_limit_rejection_leaves_destinations_unchanged(void) {
    actuator_bimanual_joint_map_t maps[ACTUATOR_BIMANUAL_JOINT_COUNT];
    int32_t goals[ACTUATOR_BIMANUAL_JOINT_COUNT] = {0};
    uint16_t left[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT];
    uint16_t right[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT];
    uint16_t left_before[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT];
    uint16_t right_before[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT];
    uint8_t failed = UINT8_MAX;

    fill_maps(maps);
    memset(left, 0x55, sizeof(left));
    memset(right, 0xaau, sizeof(right));
    memcpy(left_before, left, sizeof(left));
    memcpy(right_before, right, sizeof(right));
    maps[8].minimum_unwrapped_raw = 1800;
    maps[8].maximum_unwrapped_raw = 2200;
    goals[8] = 500000;

    CHECK(actuator_bimanual_goal_map(
              maps, goals, left, right, &failed) ==
          ACTUATOR_BIMANUAL_GOAL_MAP_LIMIT_VIOLATION);
    CHECK(failed == 8u);
    CHECK(memcmp(left, left_before, sizeof(left)) == 0);
    CHECK(memcmp(right, right_before, sizeof(right)) == 0);
}

static void test_bad_config_is_rejected(void) {
    actuator_bimanual_joint_map_t maps[ACTUATOR_BIMANUAL_JOINT_COUNT];
    int32_t goals[ACTUATOR_BIMANUAL_JOINT_COUNT] = {0};
    uint16_t left[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};
    uint16_t right[ACTUATOR_BIMANUAL_ARM_JOINT_COUNT] = {0};
    uint8_t failed = UINT8_MAX;

    fill_maps(maps);
    maps[4].positive_raw_direction = 0;
    CHECK(actuator_bimanual_goal_map(
              maps, goals, left, right, &failed) ==
          ACTUATOR_BIMANUAL_GOAL_MAP_INVALID_CONFIG);
    CHECK(failed == 4u);
}

int main(void) {
    test_zero_splits_atomically();
    test_wrap_and_negative_direction();
    test_limit_rejection_leaves_destinations_unchanged();
    test_bad_config_is_rejected();
    if (failures != 0) {
        fprintf(stderr, "%d test(s) failed\n", failures);
        return 1;
    }
    puts("bimanual goal map tests passed");
    return 0;
}
