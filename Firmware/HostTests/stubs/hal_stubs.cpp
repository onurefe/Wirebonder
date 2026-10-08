#include "stm32f4xx_hal.h"

/* The mask is modelled rather than ignored: InterruptLock's save/restore
   behaviour is the thing several callback-registry guarantees rest on. */
extern "C" uint32_t g_hostPrimask = 0U;

extern "C" void     __disable_irq(void)  { g_hostPrimask = 1U; }
extern "C" void     __enable_irq(void)   { g_hostPrimask = 0U; }
extern "C" uint32_t __get_PRIMASK(void)  { return g_hostPrimask; }
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
