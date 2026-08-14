#include "actuator_core/bimanual_dispatch.h"

#include <stddef.h>
#include <string.h>

static void observe_maximum(uint32_t *maximum, uint32_t value) {
    if (value > *maximum) {
        *maximum = value;
    }
}

static actuator_bimanual_dispatch_result_t complete(
    actuator_bimanual_dispatch_t *dispatch,
    bool left) {
    if (dispatch == NULL) {
        return ACTUATOR_BIMANUAL_DISPATCH_NULL_ARGUMENT;
    }
    if (!dispatch->snapshot.active || dispatch->snapshot.faulted) {
        return ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE;
    }
    if (left) {
        if (dispatch->snapshot.left_complete) {
            return ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE;
        }
        dispatch->snapshot.left_complete = true;
    } else {
        if (dispatch->snapshot.right_complete) {
            return ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE;
        }
        dispatch->snapshot.right_complete = true;
    }
    if (dispatch->snapshot.left_complete &&
        dispatch->snapshot.right_complete) {
        dispatch->snapshot.active = false;
        dispatch->snapshot.completed_count++;
    }
    return ACTUATOR_BIMANUAL_DISPATCH_OK;
}

void actuator_bimanual_dispatch_init(actuator_bimanual_dispatch_t *dispatch) {
    if (dispatch != NULL) {
        memset(dispatch, 0, sizeof(*dispatch));
    }
}

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_begin(
    actuator_bimanual_dispatch_t *dispatch,
    uint32_t control_tick_ms,
    uint32_t control_tick_started_us,
    uint32_t left_start_us,
    uint32_t right_start_us) {
    uint32_t skew_us;

    if (dispatch == NULL) {
        return ACTUATOR_BIMANUAL_DISPATCH_NULL_ARGUMENT;
    }
    if (dispatch->snapshot.faulted) {
        return ACTUATOR_BIMANUAL_DISPATCH_TRANSPORT_ERROR;
    }
    if (dispatch->snapshot.active) {
        return ACTUATOR_BIMANUAL_DISPATCH_BUSY;
    }
    if ((int32_t)(right_start_us - left_start_us) < 0) {
        return ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE;
    }

    skew_us = right_start_us - left_start_us;
    observe_maximum(&dispatch->snapshot.maximum_start_skew_us, skew_us);
    observe_maximum(
        &dispatch->snapshot.maximum_launch_lateness_us,
        left_start_us - control_tick_started_us);
    dispatch->control_tick_started_us = control_tick_started_us;
    dispatch->snapshot.last_control_tick_ms = control_tick_ms;
    dispatch->snapshot.last_left_start_us = left_start_us;
    dispatch->snapshot.last_right_start_us = right_start_us;
    dispatch->snapshot.left_complete = false;
    dispatch->snapshot.right_complete = false;
    dispatch->snapshot.active = true;
    dispatch->snapshot.launch_count++;
    return ACTUATOR_BIMANUAL_DISPATCH_OK;
}

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_complete_left(
    actuator_bimanual_dispatch_t *dispatch) {
    return complete(dispatch, true);
}

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_complete_right(
    actuator_bimanual_dispatch_t *dispatch) {
    return complete(dispatch, false);
}

void actuator_bimanual_dispatch_fail(
    actuator_bimanual_dispatch_t *dispatch) {
    if (dispatch == NULL) {
        return;
    }
    if (!dispatch->snapshot.faulted) {
        dispatch->snapshot.failure_count++;
    }
    dispatch->snapshot.faulted = true;
    dispatch->snapshot.active = false;
}

bool actuator_bimanual_dispatch_can_launch(
    const actuator_bimanual_dispatch_t *dispatch) {
    return dispatch != NULL && !dispatch->snapshot.active &&
        !dispatch->snapshot.faulted;
}

const actuator_bimanual_dispatch_snapshot_t *
actuator_bimanual_dispatch_snapshot(
    const actuator_bimanual_dispatch_t *dispatch) {
    return dispatch == NULL ? NULL : &dispatch->snapshot;
}
