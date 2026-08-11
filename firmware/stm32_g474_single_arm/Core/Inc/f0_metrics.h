#ifndef F0_METRICS_H
#define F0_METRICS_H

#include <stdint.h>

typedef struct
{
    uint32_t loop_period_max_us;
    uint32_t loop_work_max_us;
    uint32_t servo_sync_write_max_us;
    uint32_t host_tx_max_us;
} F0MetricsSnapshot;

void F0Metrics_Reset(void);
void F0Metrics_LoopBegin(void);
void F0Metrics_LoopEnd(void);
void F0Metrics_ObserveServoSyncWrite(uint32_t elapsed_us);
void F0Metrics_ObserveHostTx(uint32_t elapsed_us);
F0MetricsSnapshot F0Metrics_Snapshot(void);

#endif /* F0_METRICS_H */
