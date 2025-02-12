#ifndef FORCE_COIL_DRIVER_H
#define FORCE_COIL_DRIVER_H

#include "stm32f4xx_hal.h"       // Adjust include for your target
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FORCE_COIL_SETPOINT_ACHIEVED_EVENT_ID   0
#define FORCE_COIL_ERROR_EVENT_ID               1

typedef void (*ForceCoil_Callback_t)(uint16_t eventId);

void ForceCoilDriver_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void ForceCoilDriver_Start(void);
void ForceCoilDriver_Stop(void);
void ForceCoilDriver_SetCurrentSetpoint(float currentSetpoint);
void ForceCoilDriver_RegisterCallback(ForceCoil_Callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_COIL_DRIVER_H */
