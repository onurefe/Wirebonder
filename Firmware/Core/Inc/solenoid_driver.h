#ifndef SOLENOID_DRIVER_H
#define SOLENOID_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "generic.h"
#include <stdint.h>

/* Solenoid types  -------------------------------------------*/
enum
{
    SOLENOID_DRIVER_POSITION_LOW = 0,
    SOLENOID_DRIVER_POSITION_HIGH = 1,
    SOLENOID_DRIVER_POSITION_UNKNOWN = 2
};
typedef uint8_t SolenoidDriver_Position_t;

/* Exported types --------------------------------------------*/
typedef void (*SolenoidDriver_PositionChangedCallback_t)(void *solenoidHandle, 
                                                         SolenoidDriver_Position_t previousPosition,
                                                         SolenoidDriver_Position_t newPosition);

typedef struct
{
    FastIO_Pin_t highPin;
    FastIO_Pin_t lowPin;
    SolenoidDriver_Position_t currentPosition;
    SolenoidDriver_Position_t targetPosition;
    SolenoidDriver_PositionChangedCallback_t callback;
    SolenoidDriver_Position_t initialPosition;
    uint16_t stopWatch;
} SolenoidDriver_Handle_t;

/* Exported functions ----------------------------------------*/
void SolenoidDriver_Init(void);
void SolenoidDriver_Register(SolenoidDriver_Handle_t *handle,
                             GPIO_TypeDef *highGPIOx, 
                             GPIO_TypeDef *lowGPIOx, 
                             uint16_t highPin, 
                             uint16_t lowPin, 
                             SolenoidDriver_Position_t initialPosition,
                             SolenoidDriver_PositionChangedCallback_t callback);

void SolenoidDriver_Start(void);
void SolenoidDriver_Stop(void);
void SolenoidDriver_SetLowPosition(SolenoidDriver_Handle_t *handle);
void SolenoidDriver_SetHighPosition(SolenoidDriver_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif