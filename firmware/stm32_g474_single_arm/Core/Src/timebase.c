#include "timebase.h"

#include "main.h"

void Timebase_Init(void)
{
    /* APB1 runs at the 170 MHz system clock on this board.  Do not use the
     * HAL tick here: it has only millisecond resolution and remains the
     * operational timebase until F3. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U;
    TIM2->PSC = 169U;
    TIM2->ARR = UINT32_MAX;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t Timebase_NowUs(void)
{
    return TIM2->CNT;
}

uint32_t Timebase_ElapsedUs(uint32_t started_us)
{
    return Timebase_NowUs() - started_us;
}
