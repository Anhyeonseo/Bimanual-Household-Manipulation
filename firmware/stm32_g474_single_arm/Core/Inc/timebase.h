#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/* TIM2 is a free-running 1 MHz counter.  It is observation-only: no control
 * deadline, safety gate, or protocol timestamp depends on it in F0. */
void Timebase_Init(void);
uint32_t Timebase_NowUs(void);
uint32_t Timebase_ElapsedUs(uint32_t started_us);

#endif /* TIMEBASE_H */
