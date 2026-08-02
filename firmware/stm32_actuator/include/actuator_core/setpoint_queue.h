#ifndef ACTUATOR_CORE_SETPOINT_QUEUE_H
#define ACTUATOR_CORE_SETPOINT_QUEUE_H

#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_JOINT_COUNT 6u
#define ACTUATOR_SETPOINT_QUEUE_CAPACITY 16u

typedef struct {
    int32_t minimum_urad;
    int32_t maximum_urad;
} actuator_joint_limit_t;

typedef struct {
    uint32_t apply_tick;
    int32_t position_urad[ACTUATOR_JOINT_COUNT];
} actuator_setpoint_t;

typedef struct {
    actuator_setpoint_t samples[ACTUATOR_SETPOINT_QUEUE_CAPACITY];
    size_t head;
    size_t count;
} actuator_setpoint_queue_t;

typedef enum {
    ACTUATOR_QUEUE_OK = 0,
    ACTUATOR_QUEUE_EMPTY,
    ACTUATOR_QUEUE_NOT_DUE,
    ACTUATOR_QUEUE_NULL_ARGUMENT,
    ACTUATOR_QUEUE_CAPACITY_EXCEEDED,
    ACTUATOR_QUEUE_STALE_TICK,
    ACTUATOR_QUEUE_TICK_TOO_FAR,
    ACTUATOR_QUEUE_NON_MONOTONIC,
    ACTUATOR_QUEUE_LIMIT_VIOLATION
} actuator_queue_result_t;

void actuator_setpoint_queue_init(actuator_setpoint_queue_t *queue);
void actuator_setpoint_queue_clear(actuator_setpoint_queue_t *queue);

actuator_queue_result_t actuator_setpoint_queue_push_batch(
    actuator_setpoint_queue_t *queue,
    const actuator_setpoint_t *samples,
    size_t sample_count,
    uint32_t current_tick,
    uint32_t minimum_lead_ticks,
    uint32_t maximum_lead_ticks,
    const actuator_joint_limit_t limits[ACTUATOR_JOINT_COUNT]);

actuator_queue_result_t actuator_setpoint_queue_take_due(
    actuator_setpoint_queue_t *queue,
    uint32_t current_tick,
    actuator_setpoint_t *sample);

actuator_queue_result_t actuator_setpoint_queue_peek(
    const actuator_setpoint_queue_t *queue,
    actuator_setpoint_t *sample);

#endif
