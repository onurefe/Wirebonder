#include "stm32f4xx_hal.h"

extern "C" void __disable_irq(void) {}
extern "C" void __enable_irq(void)  {}
extern "C" void Error_Handler(void) {}
extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef *) {}

extern "C" HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *, uint32_t *, uint32_t)
    { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *)
    { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *)
    { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *)
    { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *, uint32_t,
                                                const uint32_t *, uint32_t, uint32_t)
    { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *, uint32_t)
    { return HAL_OK; }
