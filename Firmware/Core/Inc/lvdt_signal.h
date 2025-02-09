#ifndef LVDT_H
#define LVDT_H

#include "stm32f4xx_hal.h"       // Adjust include for your target
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LVDT_Signal_Init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t dac_channel);
void LVDT_Signal_Start(void);
void LVDT_Signal_Stop(void);

#ifdef __cplusplus
}

#endif /* LVDT_SIGNAL_H */
