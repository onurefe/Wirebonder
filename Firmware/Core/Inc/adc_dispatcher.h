#ifndef ADC_DISPATCHER_H
#define ADC_DISPATCHER_H

#include "stm32f4xx_hal.h"  // Change if using a different series
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_DISPATCHER_MAX_CHANNELS   8

typedef void (*AdcDispatcher_Callback_t)(uint16_t value);

void AdcDispatcher_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
void AdcDispatcher_Start(void);
void AdcDispatcher_Stop(void);
int8_t AdcDispatcher_RegisterChannel(uint8_t conversionOrder, 
                                     uint16_t downsamplingFactor, 
                                     AdcDispatcher_Callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* ADC2_DISPATCHER_H */
