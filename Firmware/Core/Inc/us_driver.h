#ifndef US_DRIVER_H
#define US_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "generic.h"

/* Exported definitions ----------------------------------------------------*/
#define US_DRIVER_ADC_BUFFER_FULL_EVENT_ID 0
#define US_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID 1
#define US_DRIVER_OPERATION_COMPLETED_EVENT_ID 2

/* Exported types ----------------------------------------------------------*/
typedef void (*UsDriver_Callback_t)(uint16_t eventId, uint16_t *buffer);

/* Exported functions ------------------------------------------------------*/
void UsDriver_Init(TIM_HandleTypeDef *htim, 
                   ADC_HandleTypeDef *hadc, 
                   DAC_HandleTypeDef *hdac, 
                   uint32_t dacChannel);

void UsDriver_Start(uint16_t *dacBuffer, 
                    uint16_t *adcBuffer, 
                    uint16_t dacBufferSize, 
                    uint16_t adcBufferSize, 
                    Bool_t continuousOperation, 
                    UsDriver_Callback_t callback);

void UsDriver_Stop(void);

void UsDriver_ADCDMAHalfCallback(ADC_HandleTypeDef *hadc);
void UsDriver_ADCDMAFullCallback(ADC_HandleTypeDef *hadc);

void UsDriver_DACDMAHalfCallback(DAC_HandleTypeDef *hdac);
void UsDriver_DACDMAFullCallback(DAC_HandleTypeDef *hdac);

#ifdef __cplusplus
}
#endif

#endif
