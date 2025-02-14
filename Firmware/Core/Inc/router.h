#ifndef ROUTER_H
#define ROUTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "generic.h"
#include "configuration.h"
#include "stepper.h"

/* Exported types ----------------------------------------------------------*/
typedef void (*Router_MotionCompletedCallback_t)(void);

typedef struct
{
    float d;
    float vMax;
    float aMax;
    float tAcc;
    float dAcc;
    float tConst;
    float dConst;
} Router_RouteParams_t;

typedef struct
{
    float maxVelocity;
    float maxAcceleration;
    uint32_t numOfRenderedSegments;
    Router_RouteParams_t routeParams;
    qint55_8_t lastRenderedStepperPosition;
    Bool_t isBusy;
    Router_MotionCompletedCallback_t callback;
    Stepper_Handle_t stepper;
}Router_Handle_t;


/* Exported functions ------------------------------------------------------*/
void Router_Init(GPIO_TypeDef *enPort, 
                 uint16_t enPin, 
                 GPIO_TypeDef *rstPort, 
                 uint16_t rstPin);

void Router_Register(Router_Handle_t *handle, 
                     GPIO_TypeDef *stepPort, 
                     uint16_t stepPin, 
                     GPIO_TypeDef *dirPort, 
                     uint16_t dirPin,
                     float maxVelocity, 
                     float maxAcceleration, 
                     Router_MotionCompletedCallback_t callback);

void Router_Append(Router_Handle_t *handle, 
                   float displacement);


void Router_Start(void);
void Router_Execute(void);
void Router_TimerTick(void);

float Router_StepsToMeters(int32_t positionInSteps);

#ifdef __cplusplus
}
#endif

#endif