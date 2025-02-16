#ifndef ZMOTOR_CONTROLLER_H
#define ZMOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configuration.h"
#include "generic.h"

/* Exported constants --------------------------------------------*/
#define ZMOTOR_CONTROLLER_SETPOINT_ACHIEVED_EVENT_ID 0
#define ZMOTOR_CONTROLLER_UNABLE_SET_POSITION_EVENT_ID 1

/* Exported types ------------------------------------------------*/
typedef void (*ZMotorController_Callback_t)(uint16_t eventId);

/* Exported functions --------------------------------------------*/
void ZMotorController_Init(DAC_HandleTypeDef *lvdtDac, 
                        TIM_HandleTypeDef *lvdtHtim, 
                        uint32_t lvdtDacChannel);

void ZMotorController_Start(float position);
void ZMotorController_Stop(void);
void ZMotorController_SetPositionSetpoint(float position);
void ZMotorController_RegisterCallback(ZMotorController_Callback_t callback);

#ifdef __cplusplus
}
#endif

#endif