#include "actuator_core/joint_unwrap.h"

#include "actuator_core/calibration.h"

#include <limits.h>
#include <stddef.h>

static int64_t absolute_i64(int64_t value) {
    return value < 0 ? -value : value;
}

static int64_t round_divide(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (numerator + (denominator / 2)) / denominator;
    }
    return -((-numerator + (denominator / 2)) / denominator);
}

static bool direction_is_valid(int8_t direction) {
    return direction == 1 || direction == -1;
}

void actuator_joint_unwrapper_reset(actuator_joint_unwrapper_t *state) {
    if (state == NULL) {
        return;
    }
    state->bound = false;
    state->previous_raw = 0u;
    state->unwrapped_raw = 0;
}

actuator_unwrap_result_t actuator_joint_unwrapper_bind(
    actuator_joint_unwrapper_t *state,
    uint16_t observed_raw,
    int32_t reference_unwrapped_raw,
    uint16_t maximum_reference_delta_raw) {
    const int64_t approximate_turn =
        ((int64_t)reference_unwrapped_raw - observed_raw) /
        ACTUATOR_UNWRAP_RAW_MODULUS;
    int64_t best_candidate = 0;
    int64_t best_distance = INT64_MAX;
    bool tied = false;

    if (state == NULL) {
        return ACTUATOR_UNWRAP_NULL_ARGUMENT;
    }
    if (observed_raw >= ACTUATOR_UNWRAP_RAW_MODULUS) {
        return ACTUATOR_UNWRAP_RAW_OUT_OF_RANGE;
    }
    if (maximum_reference_delta_raw == 0u ||
        maximum_reference_delta_raw >= ACTUATOR_UNWRAP_HALF_TURN_RAW) {
        return ACTUATOR_UNWRAP_BAD_REFERENCE_WINDOW;
    }

    for (int64_t offset = -1; offset <= 1; ++offset) {
        const int64_t candidate = observed_raw +
            ((approximate_turn + offset) * ACTUATOR_UNWRAP_RAW_MODULUS);
        const int64_t distance = absolute_i64(
            candidate - reference_unwrapped_raw);
        if (distance < best_distance) {
            best_candidate = candidate;
            best_distance = distance;
            tied = false;
        } else if (distance == best_distance) {
            tied = true;
        }
    }

    if (tied) {
        return ACTUATOR_UNWRAP_AMBIGUOUS_DELTA;
    }
    if (best_distance > maximum_reference_delta_raw) {
        return ACTUATOR_UNWRAP_REFERENCE_TOO_FAR;
    }
    if (best_candidate < INT32_MIN || best_candidate > INT32_MAX) {
        return ACTUATOR_UNWRAP_OVERFLOW;
    }

    state->bound = true;
    state->previous_raw = observed_raw;
    state->unwrapped_raw = (int32_t)best_candidate;
    return ACTUATOR_UNWRAP_OK;
}

actuator_unwrap_result_t actuator_joint_unwrapper_update(
    actuator_joint_unwrapper_t *state,
    uint16_t observed_raw,
    int32_t *unwrapped_raw) {
    int32_t delta;
    int64_t updated;

    if (state == NULL || unwrapped_raw == NULL) {
        return ACTUATOR_UNWRAP_NULL_ARGUMENT;
    }
    if (!state->bound) {
        return ACTUATOR_UNWRAP_REFERENCE_TOO_FAR;
    }
    if (observed_raw >= ACTUATOR_UNWRAP_RAW_MODULUS) {
        return ACTUATOR_UNWRAP_RAW_OUT_OF_RANGE;
    }

    delta = (int32_t)observed_raw - (int32_t)state->previous_raw;
    if (delta == ACTUATOR_UNWRAP_HALF_TURN_RAW ||
        delta == -ACTUATOR_UNWRAP_HALF_TURN_RAW) {
        return ACTUATOR_UNWRAP_AMBIGUOUS_DELTA;
    }
    if (delta > ACTUATOR_UNWRAP_HALF_TURN_RAW) {
        delta -= ACTUATOR_UNWRAP_RAW_MODULUS;
    } else if (delta < -ACTUATOR_UNWRAP_HALF_TURN_RAW) {
        delta += ACTUATOR_UNWRAP_RAW_MODULUS;
    }

    updated = (int64_t)state->unwrapped_raw + delta;
    if (updated < INT32_MIN || updated > INT32_MAX) {
        return ACTUATOR_UNWRAP_OVERFLOW;
    }
    state->previous_raw = observed_raw;
    state->unwrapped_raw = (int32_t)updated;
    *unwrapped_raw = state->unwrapped_raw;
    return ACTUATOR_UNWRAP_OK;
}

actuator_unwrap_result_t actuator_unwrapped_raw_to_urad(
    uint16_t zero_raw,
    int8_t positive_raw_direction,
    int32_t unwrapped_raw,
    int32_t *position_urad) {
    int64_t urad;

    if (position_urad == NULL) {
        return ACTUATOR_UNWRAP_NULL_ARGUMENT;
    }
    if (zero_raw >= ACTUATOR_UNWRAP_RAW_MODULUS) {
        return ACTUATOR_UNWRAP_RAW_OUT_OF_RANGE;
    }
    if (!direction_is_valid(positive_raw_direction)) {
        return ACTUATOR_UNWRAP_BAD_DIRECTION;
    }

    urad = round_divide(
        ((int64_t)unwrapped_raw - zero_raw) *
            positive_raw_direction * ACTUATOR_TURN_URAD,
        ACTUATOR_RAW_UNITS_PER_TURN);
    if (urad < INT32_MIN || urad > INT32_MAX) {
        return ACTUATOR_UNWRAP_OVERFLOW;
    }
    *position_urad = (int32_t)urad;
    return ACTUATOR_UNWRAP_OK;
}

actuator_unwrap_result_t actuator_unwrapped_urad_to_raw(
    uint16_t zero_raw,
    int8_t positive_raw_direction,
    int32_t position_urad,
    const actuator_unwrapped_joint_limit_t *limit,
    int32_t *unwrapped_raw,
    uint16_t *modulo_raw) {
    int64_t raw_delta;
    int64_t candidate;
    int64_t modulo;

    if (limit == NULL || unwrapped_raw == NULL || modulo_raw == NULL) {
        return ACTUATOR_UNWRAP_NULL_ARGUMENT;
    }
    if (zero_raw >= ACTUATOR_UNWRAP_RAW_MODULUS) {
        return ACTUATOR_UNWRAP_RAW_OUT_OF_RANGE;
    }
    if (!direction_is_valid(positive_raw_direction)) {
        return ACTUATOR_UNWRAP_BAD_DIRECTION;
    }
    if (limit->minimum_unwrapped_raw > limit->maximum_unwrapped_raw) {
        return ACTUATOR_UNWRAP_LIMIT_VIOLATION;
    }

    raw_delta = round_divide(
        (int64_t)position_urad * ACTUATOR_RAW_UNITS_PER_TURN,
        ACTUATOR_TURN_URAD);
    candidate = (int64_t)zero_raw +
        ((int64_t)positive_raw_direction * raw_delta);
    if (candidate < INT32_MIN || candidate > INT32_MAX) {
        return ACTUATOR_UNWRAP_OVERFLOW;
    }
    if (candidate < limit->minimum_unwrapped_raw ||
        candidate > limit->maximum_unwrapped_raw) {
        return ACTUATOR_UNWRAP_LIMIT_VIOLATION;
    }

    modulo = candidate % ACTUATOR_UNWRAP_RAW_MODULUS;
    if (modulo < 0) {
        modulo += ACTUATOR_UNWRAP_RAW_MODULUS;
    }
    *unwrapped_raw = (int32_t)candidate;
    *modulo_raw = (uint16_t)modulo;
    return ACTUATOR_UNWRAP_OK;
}
