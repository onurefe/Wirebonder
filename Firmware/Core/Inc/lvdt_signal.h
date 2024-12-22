#ifndef LVDT_H
#define LVDT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * This module configures:
 *   - DAC Channel 2 (on pin PA5 for STM32F446)
 *   - Timer (TIM6 by default) to trigger the DAC updates
 *   - DMA in circular mode to feed the DAC with samples of a 1 kHz sine wave
 *
 * Usage Steps:
 *   1) Call LVDT_Signal_Init() once after HAL and system clock are initialized.
 *   2) Call LVDT_Signal_Start() to begin the continuous sine wave output.
 *   3) (Optional) Use LVDT_Signal_Stop() to stop the waveform.
 *   4) (Optional) Adjust the LUT generation or timer settings to change frequency, amplitude, etc.
 */

/**
 * @brief Initialize all resources needed to generate a 1 kHz sine wave:
 *        DAC channel 2, TIM6, DMA, and internal LUT.
 */
void LVDT_Signal_Init(void);

/**
 * @brief Start the continuous generation of the 1 kHz sine wave on DAC2 output.
 */
void LVDT_Signal_Start(void);

/**
 * @brief Stop the DAC and disable the timer trigger (if desired).
 */
void LVDT_Signal_Stop(void);

#endif /* LVDT_SIGNAL_H */
