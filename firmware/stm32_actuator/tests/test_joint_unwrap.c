#include "actuator_core/joint_unwrap.h"

#include <stdint.h>
#include <stdio.h>

static int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", \
                __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void test_tracks_positive_and_negative_wraps(void) {
    actuator_joint_unwrapper_t state;
    int32_t value = 0;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 3919u, 3919, 512u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(actuator_joint_unwrapper_update(&state, 4059u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4059);
    CHECK(actuator_joint_unwrapper_update(&state, 65u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4161);
    CHECK(actuator_joint_unwrapper_update(&state, 4078u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4078);
}

static void test_binding_requires_a_close_verified_reference(void) {
    actuator_joint_unwrapper_t state;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 24u, 4120, 256u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(state.unwrapped_raw == 4120);

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 24u, 2048, 1024u) ==
          ACTUATOR_UNWRAP_REFERENCE_TOO_FAR);
    CHECK(!state.bound);
    CHECK(actuator_joint_unwrapper_bind(&state, 0u, 2048, 2047u) ==
          ACTUATOR_UNWRAP_AMBIGUOUS_DELTA);
}

static void test_half_turn_update_fails_closed(void) {
    actuator_joint_unwrapper_t state;
    int32_t value = 0;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 1000u, 1000, 64u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(actuator_joint_unwrapper_update(&state, 3048u, &value) ==
          ACTUATOR_UNWRAP_AMBIGUOUS_DELTA);
    CHECK(state.unwrapped_raw == 1000);
    CHECK(state.previous_raw == 1000u);
}

static void test_unwrapped_command_validates_before_modulo(void) {
    const actuator_unwrapped_joint_limit_t limit = {1859, 4188};
    int32_t unwrapped_raw = 0;
    int32_t position_urad = 0;
    uint16_t modulo_raw = 0u;

    CHECK(actuator_unwrapped_raw_to_urad(
              2048u, 1, 4188, &position_urad) == ACTUATOR_UNWRAP_OK);
    CHECK(actuator_unwrapped_urad_to_raw(
              2048u, 1, position_urad, &limit,
              &unwrapped_raw, &modulo_raw) == ACTUATOR_UNWRAP_OK);
    CHECK(unwrapped_raw == 4188);
    CHECK(modulo_raw == 92u);

    CHECK(actuator_unwrapped_raw_to_urad(
              2048u, 1, 4189, &position_urad) == ACTUATOR_UNWRAP_OK);
    CHECK(actuator_unwrapped_urad_to_raw(
              2048u, 1, position_urad, &limit,
              &unwrapped_raw, &modulo_raw) ==
          ACTUATOR_UNWRAP_LIMIT_VIOLATION);
}

int main(void) {
    test_tracks_positive_and_negative_wraps();
    test_binding_requires_a_close_verified_reference();
    test_half_turn_update_fails_closed();
    test_unwrapped_command_validates_before_modulo();
    if (failures != 0) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    puts("joint unwrap tests passed");
    return 0;
}
