#ifndef LVDT_PHASE_DETECTION_H
#define LVDT_PHASE_DETECTION_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * This module configures ADC2 to measure two inputs (A -> ADC2_IN6, B -> ADC2_IN7).
 * The conversion is triggered by a timer (e.g., TIM3) at >= 4kHz.
 * DMA in circular mode automatically stores the samples in a buffer.
 *
 * Usage:
 *   1) Call LVDT_Phase_Init() once after HAL and system clocks are initialized.
 *   2) Call LVDT_Phase_Start() to start periodic ADC sampling via the timer trigger.
 *   3) The ADC results will appear in an internal buffer "LVDT_Phase_Buffer".
 *   4) LVDT_Phase_Stop() can stop the sampling if needed.
 *   5) Access the buffer for your phase detection algorithm.
 */

/** 
 * @brief Initialize ADC2 with two channels (IN6, IN7) and configure the timer trigger (>=4kHz).
 */
void LVDT_PhaseDection_Init(void);

/**
 * @brief Start the timer-triggered ADC sampling with DMA in circular mode.
 */
void LVDT_PhaseDetection_Start(void);

/**
 * @brief Stop the ADC sampling (and optionally stop the timer).
 */
void LVDT_PhaseDetection_Stop(void);

/**
 * @brief Return pointer to the internal buffer where ADC samples are stored.
 *        The buffer is arranged as [CH6, CH7, CH6, CH7, ...].
 *
 * @return Pointer to the buffer of ADC results (16-bit raw values).
 */
uint16_t* LVDT_PhaseDetection_GetBuffer(void);

/**
 * @brief Return the length of the ADC buffer (in number of samples).
 *        E.g., if it's 2*N, it will return 2*N.
 *
 * @return The buffer length (total samples).
 */
uint32_t LVDT_PhaseDetection_GetBufferLength(void);

#endif /* LVDT_PHASE_H */
