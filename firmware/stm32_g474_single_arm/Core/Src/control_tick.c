#include "control_tick.h"

#include "main.h"
#include "timebase.h"

static ControlTickSnapshot control_tick;
static uint32_t previous_started_us;
static uint8_t previous_started_valid;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
static volatile uint8_t control_tick_pending;
static volatile uint32_t control_tick_pending_ms;
static volatile uint32_t control_tick_pending_started_us;
static volatile uint32_t control_tick_missed_count;
#endif

static void ControlTick_ObserveMaximum(uint32_t *maximum, uint32_t value)
{
    if (value > *maximum)
    {
        *maximum = value;
    }
}

void ControlTick_Reset(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    control_tick = (ControlTickSnapshot){0U, 0U, 0U, 0U};
    previous_started_us = 0U;
    previous_started_valid = 0U;
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    control_tick_pending = 0U;
    control_tick_pending_ms = 0U;
    control_tick_pending_started_us = 0U;
    control_tick_missed_count = 0U;
#endif
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void ControlTick_Init(void)
{
    /* APB1 is 170 MHz. Generate an exact 5 ms update at 1 MHz without
     * involving the HAL tick; TIM2 remains the independent measurement clock.
     */
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->CR1 = 0U;
    TIM6->PSC = 169U;
    TIM6->ARR = (CONTROL_TICK_PERIOD_US - 1U);
    TIM6->CNT = 0U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;
    TIM6->DIER = TIM_DIER_UIE;
    ControlTick_Reset();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;
}

void ControlTick_OnInterrupt(void)
{
    const uint32_t started_us = Timebase_NowUs();
#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
    if (control_tick_pending != 0U)
    {
        if (control_tick_missed_count != UINT32_MAX)
        {
            control_tick_missed_count++;
        }
    }
    else
    {
        control_tick_pending_ms = HAL_GetTick();
        control_tick_pending_started_us = started_us;
        control_tick_pending = 1U;
    }
#endif

    if (previous_started_valid != 0U)
    {
        const uint32_t period_us = started_us - previous_started_us;
        const uint32_t jitter_us =
            (period_us >= CONTROL_TICK_PERIOD_US) ?
                (period_us - CONTROL_TICK_PERIOD_US) :
                (CONTROL_TICK_PERIOD_US - period_us);
        ControlTick_ObserveMaximum(&control_tick.period_max_us, period_us);
        ControlTick_ObserveMaximum(&control_tick.jitter_max_us, jitter_us);
    }

    previous_started_us = started_us;
    previous_started_valid = 1U;
    if (control_tick.count != UINT32_MAX)
    {
        control_tick.count++;
    }

    /* F3.0 intentionally stops here: no queue, servo write, safety poll, or
     * host parsing may migrate to this ISR until this timing baseline passes.
     */
    ControlTick_ObserveMaximum(
        &control_tick.work_max_us,
        Timebase_ElapsedUs(started_us)
    );
}

ControlTickSnapshot ControlTick_Snapshot(void)
{
    return control_tick;
}

#if HOST_BIMANUAL_DMA_DISPATCH_BUILD
uint8_t ControlTick_TakeEvent(ControlTickEvent *event)
{
    uint8_t available = 0U;
    const uint32_t primask = __get_PRIMASK();

    if (event == NULL)
    {
        return 0U;
    }
    __disable_irq();
    if (control_tick_pending != 0U)
    {
        event->tick_ms = control_tick_pending_ms;
        event->started_us = control_tick_pending_started_us;
        event->missed_count = control_tick_missed_count;
        control_tick_pending = 0U;
        control_tick_missed_count = 0U;
        available = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return available;
}

void ControlTick_ClearPending(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    control_tick_pending = 0U;
    control_tick_missed_count = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
#endif
