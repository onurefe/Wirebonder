#ifndef TIMER_EXPIRE_H
#define TIMER_EXPIRE_H

#include "generic.h"
#include "configuration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ----------------------------------------------------------*/
typedef void (*TimerExpire_Callback_t)(void *handle);

typedef struct
{
    Bool_t scheduled;
    Bool_t activated;
    Bool_t oneShot;
    uint32_t periodInTicks;
    uint32_t triggerTick;
    TimerExpire_Callback_t callback;
} TimerExpire_Handle_t;

/* Exported functions ------------------------------------------------------*/
void TimerExpire_Init(void);
void TimerExpire_Register(TimerExpire_Handle_t *handle, 
                          Bool_t oneShot, 
                          float period, 
                          TimerExpire_Callback_t callback);

void TimerExpire_Start(void);
void TimerExpire_Activate(TimerExpire_Handle_t *handle);
void TimerExpire_Stop(void);
uint32_t TimerExpire_GetTicks(void);
float TimerExpire_GetTickFrequency(void);

void TimerExpire_TickFunc(void);

#ifdef __cplusplus
}
#endif

#endif