#ifndef LVDT_H
#define LVDT_H

#include "stm32f4xx_hal.h"       // Adjust include for your target
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*LVDT_MeasurementCallback_t)(float amplitude_difference);

void LVDT_Init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t dac_channel);
void LVDT_RegisterCallback(LVDT_MeasurementCallback_t callback);
void LVDT_Start(void);
void LVDT_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
