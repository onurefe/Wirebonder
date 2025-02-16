#ifndef FORCE_COIL_DRIVER_H
#define FORCE_COIL_DRIVER_H

#include "configuration.h"
#include "generic.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------*/
typedef void (*ForceCoil_Callback_t)(uint16_t eventId);

/* Exported constants --------------------------------------------------*/
#define FORCE_COIL_SETPOINT_ACHIEVED_EVENT_ID           0
#define FORCE_COIL_UNABLE_TO_SET_CURRENT_EVENT_ID       1

/* Exported functions --------------------------------------------------*/
void ForceCoilDriver_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void ForceCoilDriver_Start(void);
void ForceCoilDriver_Stop(void);
void ForceCoilDriver_SetCurrentSetpoint(float currentSetpoint);
void ForceCoilDriver_RegisterCallback(ForceCoil_Callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_COIL_DRIVER_H */
