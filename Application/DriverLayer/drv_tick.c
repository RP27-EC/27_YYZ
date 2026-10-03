#include "drv_tick.h"
/* TIM2 is the HAL 1 MHz counter and rolls over every millisecond. */
uint32_t micros(void)
{
    uint32_t before, after, sub;
    do {
        before = HAL_GetTick();
        sub = TIM2->CNT;
        after = HAL_GetTick();
    } while (before != after);
    return before * 1000U + sub;
}
void delay_us(uint32_t us)
{
    /* Short bounded intervals avoid CYCCNT multiplication/wrap ambiguity. */
    while (us != 0U) {
        uint32_t part = us > 1000U ? 1000U : us;
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = part * (SystemCoreClock / 1000000U);
        while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
        us -= part;
    }
}
void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) { delay_us(1000U); }
}
