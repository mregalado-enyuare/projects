#include "delay_us.h"

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT;

    uint32_t delayTicks =
        us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - startTick) < delayTicks);
}
