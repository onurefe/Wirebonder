#ifndef ZMOTOR_CONTROL_H
#define ZMOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // Change if using a different series
#include <stdint.h>

/* Exported constants --------------------------------------------*/
#define ZMOTOR_CONTROL_SETPOINT_ACHIEVED_EVENT_ID 0

/* Exported types ------------------------------------------------*/
typedef void (*ZMotorControl_Callback_t)(uint16_t eventId);

/* Exported functions --------------------------------------------*/
void ZMotorControl_Init(DAC_HandleTypeDef *lvdtDac, 
                        TIM_HandleTypeDef *lvdtHtim, 
                        uint32_t lvdtDacChannel);

void ZMotorControl_Start(void);
void ZMotorControl_Stop(void);
void ZMotorControl_SetPositionSetpoint(float position);
void ZMotorControl_RegisterCallback(ZMotorControl_Callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* ZMOTOR_CONTROL_H */
