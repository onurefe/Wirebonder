#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "generic.h"

/* Exported definitions ----------------------------------------------------*/
#define ULTRASONIC_DRIVER_ADC_BUFFER_FULL_EVENT_ID 0
#define ULTRASONIC_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID 1
#define ULTRASONIC_DRIVER_OPERATION_COMPLETED_EVENT_ID 2

/* Exported types ----------------------------------------------------------*/
typedef void (*UltrasonicDriver_Callback_t)(uint16_t eventId, uint16_t *buffer);

/* Exported functions ------------------------------------------------------*/
void UltrasonicDriver_Init(TIM_HandleTypeDef *htim, 
                           ADC_HandleTypeDef *hadc, 
                           DAC_HandleTypeDef *hdac, 
                           uint32_t dacChannel);

void UltrasonicDriver_Start(uint16_t *dacBuffer, 
                            uint16_t *adcBuffer, 
                            uint16_t dacBufferSize, 
                            uint16_t adcBufferSize, 
                            Bool_t continuousOperation, 
                            UltrasonicDriver_Callback_t callback);

void UltrasonicDriver_Stop(void);

void UltrasonicDriver_ADCDMAHalfCallback(ADC_HandleTypeDef *hadc);
void UltrasonicDriver_ADCDMAFullCallback(ADC_HandleTypeDef *hadc);

void UltrasonicDriver_DACDMAHalfCallback(DAC_HandleTypeDef *hdac);
void UltrasonicDriver_DACDMAFullCallback(DAC_HandleTypeDef *hdac);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_DRIVER_H
