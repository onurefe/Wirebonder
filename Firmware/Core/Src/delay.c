#include "delay.h"
#include "stm32f4xx_hal.h" // Or your specific device header

// Initialize the DWT cycle counter
void DWT_Delay_Init(void)
{
    // Make sure the system clock variable is updated
    SystemCoreClockUpdate();

    // Enable TRC (Trace) and the DWT
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    // Reset and enable the DWT_CYCCNT counter
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Delay for `us` microseconds, using the DWT cycle counter
void DWT_Delay_us(uint32_t us)
{
    // Number of CPU cycles per microsecond
    const uint32_t clk_cycle_per_us = (SystemCoreClock / 1000000UL);

    // Record the start cycle count
    uint32_t start_tick = DWT->CYCCNT;
    // Calculate the target count
    uint32_t target_tick = start_tick + (us * clk_cycle_per_us);

    // Wait until the DWT cycle counter reaches the target
    while (DWT->CYCCNT < target_tick)
    {
        // spin
    }
}
