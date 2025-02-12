#ifndef TIMER_H
#define TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "generic.h"
#include "configuration.h"

/* Exported types ----------------------------------------------------------*/
typedef void (*Timer_TickDelegate_t)(void);

/* Exported functions ------------------------------------------------------*/
void Timer_Init(TIM_HandleTypeDef *htim);
void Timer_Register(Timer_TickDelegate_t timerDelegate);
float Timer_GetTickFrequency(void);
void Timer_Start(void);
void Timer_Stop(void);
uint32_t Timer_GetTicks(void);

#ifdef __cplusplus
}
#endif

#endif