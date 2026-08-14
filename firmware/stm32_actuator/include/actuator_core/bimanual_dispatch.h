#ifndef ACTUATOR_CORE_BIMANUAL_DISPATCH_H
#define ACTUATOR_CORE_BIMANUAL_DISPATCH_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ACTUATOR_BIMANUAL_DISPATCH_OK = 0,
    ACTUATOR_BIMANUAL_DISPATCH_NULL_ARGUMENT,
    ACTUATOR_BIMANUAL_DISPATCH_BUSY,
    ACTUATOR_BIMANUAL_DISPATCH_BAD_STATE,
    ACTUATOR_BIMANUAL_DISPATCH_TRANSPORT_ERROR
} actuator_bimanual_dispatch_result_t;

typedef struct {
    uint32_t launch_count;
    uint32_t completed_count;
    uint32_t failure_count;
    uint32_t maximum_start_skew_us;
    uint32_t maximum_launch_lateness_us;
    uint32_t last_control_tick_ms;
    uint32_t last_left_start_us;
    uint32_t last_right_start_us;
    bool active;
    bool left_complete;
    bool right_complete;
    bool faulted;
} actuator_bimanual_dispatch_snapshot_t;

typedef struct {
    actuator_bimanual_dispatch_snapshot_t snapshot;
    uint32_t control_tick_started_us;
} actuator_bimanual_dispatch_t;

void actuator_bimanual_dispatch_init(actuator_bimanual_dispatch_t *dispatch);

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_begin(
    actuator_bimanual_dispatch_t *dispatch,
    uint32_t control_tick_ms,
    uint32_t control_tick_started_us,
    uint32_t left_start_us,
    uint32_t right_start_us);

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_complete_left(
    actuator_bimanual_dispatch_t *dispatch);

actuator_bimanual_dispatch_result_t actuator_bimanual_dispatch_complete_right(
    actuator_bimanual_dispatch_t *dispatch);

void actuator_bimanual_dispatch_fail(
    actuator_bimanual_dispatch_t *dispatch);

bool actuator_bimanual_dispatch_can_launch(
    const actuator_bimanual_dispatch_t *dispatch);

const actuator_bimanual_dispatch_snapshot_t *
actuator_bimanual_dispatch_snapshot(
    const actuator_bimanual_dispatch_t *dispatch);

#endif
