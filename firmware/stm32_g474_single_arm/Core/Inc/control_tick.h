#ifndef CONTROL_TICK_H
#define CONTROL_TICK_H

#include <stdint.h>

/*
 * F3.0 reserves TIM6 as the sole 5 ms control-clock source, but deliberately
 * performs no servo output yet. The cooperative execution path remains
 * authoritative until this timing baseline passes.
 */
#define CONTROL_TICK_PERIOD_US UINT32_C(5000)

typedef struct
{
    uint32_t period_max_us;
    uint32_t jitter_max_us;
    uint32_t work_max_us;
    uint32_t count;
} ControlTickSnapshot;

typedef struct
{
    uint32_t tick_ms;
    uint32_t started_us;
    uint32_t missed_count;
} ControlTickEvent;

void ControlTick_Init(void);
void ControlTick_OnInterrupt(void);
void ControlTick_Reset(void);
ControlTickSnapshot ControlTick_Snapshot(void);
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
uint8_t ControlTick_TakeEvent(ControlTickEvent *event);
void ControlTick_ClearPending(void);
#endif

#endif /* CONTROL_TICK_H */
