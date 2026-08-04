#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef enum { HAL_UNLOCKED=0, HAL_LOCKED=1 }                       HAL_LockTypeDef;

typedef struct { uint32_t dummy; } ADC_TypeDef;
typedef struct { uint32_t dummy; } DAC_TypeDef;
typedef struct { uint32_t dummy; } TIM_TypeDef;
typedef struct { uint32_t dummy; } I2C_TypeDef;
typedef struct { uint32_t dummy; } GPIO_TypeDef;

typedef struct __DMA_HandleTypeDef { void *dummy; } DMA_HandleTypeDef;

typedef struct __ADC_HandleTypeDef {
    ADC_TypeDef      *Instance;
    DMA_HandleTypeDef *DMA_Handle;
    HAL_LockTypeDef   Lock;
    uint32_t          State;
    uint32_t          ErrorCode;
} ADC_HandleTypeDef;

typedef struct __TIM_HandleTypeDef {
    TIM_TypeDef     *Instance;
    HAL_LockTypeDef  Lock;
    uint32_t         State;
} TIM_HandleTypeDef;

typedef struct __DAC_HandleTypeDef {
    DAC_TypeDef     *Instance;
    HAL_LockTypeDef  Lock;
    uint32_t         State;
    uint32_t         ErrorCode;
} DAC_HandleTypeDef;

typedef struct __I2C_HandleTypeDef {
    I2C_TypeDef     *Instance;
    HAL_LockTypeDef  Lock;
    uint32_t         State;
    uint32_t         ErrorCode;
} I2C_HandleTypeDef;

#define DAC_CHANNEL_1    0x00000000U
#define DAC_CHANNEL_2    0x00000010U
#define DAC_ALIGN_12B_R  0x00000000U

void __disable_irq(void);
void __enable_irq(void);
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,
                                     const uint32_t *pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);

#ifdef __cplusplus
}
#endif
