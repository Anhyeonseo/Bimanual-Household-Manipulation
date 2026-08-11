#include "f0_metrics.h"

#include "timebase.h"

static F0MetricsSnapshot f0_metrics;
static uint32_t loop_started_us;
static uint32_t previous_loop_started_us;
static uint8_t loop_started;
static uint8_t previous_loop_valid;

static void F0Metrics_ObserveMaximum(uint32_t *maximum, uint32_t value)
{
    if (value > *maximum)
    {
        *maximum = value;
    }
}

void F0Metrics_Reset(void)
{
    f0_metrics = (F0MetricsSnapshot){0U, 0U, 0U, 0U};
    previous_loop_started_us = 0U;
    loop_started = 0U;
    previous_loop_valid = 0U;
}

void F0Metrics_LoopBegin(void)
{
    uint32_t now = Timebase_NowUs();
    if (previous_loop_valid != 0U)
    {
        F0Metrics_ObserveMaximum(
            &f0_metrics.loop_period_max_us,
            now - previous_loop_started_us
        );
    }
    previous_loop_started_us = now;
    previous_loop_valid = 1U;
    loop_started_us = now;
    loop_started = 1U;
}

void F0Metrics_LoopEnd(void)
{
    if (loop_started != 0U)
    {
        F0Metrics_ObserveMaximum(
            &f0_metrics.loop_work_max_us,
            Timebase_ElapsedUs(loop_started_us)
        );
        loop_started = 0U;
    }
}

void F0Metrics_ObserveServoSyncWrite(uint32_t elapsed_us)
{
    F0Metrics_ObserveMaximum(&f0_metrics.servo_sync_write_max_us, elapsed_us);
}

void F0Metrics_ObserveHostTx(uint32_t elapsed_us)
{
    F0Metrics_ObserveMaximum(&f0_metrics.host_tx_max_us, elapsed_us);
}

F0MetricsSnapshot F0Metrics_Snapshot(void)
{
    return f0_metrics;
}
