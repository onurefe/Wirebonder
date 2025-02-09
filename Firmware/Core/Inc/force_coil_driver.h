#ifndef FORCE_COIL_DRIVER_H
#define FORCE_COIL_DRIVER_H

#include "stm32f4xx_hal.h"       // Adjust include for your target
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ForceCoilDriver_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void ForceCoilDriver_Start(void);
void ForceCoilDriver_Stop(void);
void ForceCoilDriver_SetCurrentSetpoint(float setpoint);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_COIL_DRIVER_H */
