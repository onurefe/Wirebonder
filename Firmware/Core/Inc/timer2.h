#ifndef TIMER2_H
#define TIMER2_H

#include "generic.h"
#include "queue.h"
#include "configuration.h"

/* Exported types ----------------------------------------------------------*/
typedef void (*Timer2_TickDelegate_t)();

/* Exported functions ------------------------------------------------------*/
extern void Timer2_Init();
extern void Timer2_Register(Timer2_TickDelegate_t timerDelegate);
extern void Timer2_Start();
extern void Timer2_Stop();
extern uint32_t Timer2_GetTicks();

#endif