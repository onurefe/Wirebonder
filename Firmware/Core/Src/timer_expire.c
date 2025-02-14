#include "timer_expire.h"
#include "generic.h"

/* Private variables -------------------------------------------------------*/
static TimerExpire_Handle_t *g_registeredHandles[TIMER_EXPIRE_MAX_NUM_OF_HANDLES];
static uint8_t g_numOfHandles = 0;
static State_t g_state = STATE_UNINIT;
static volatile uint32_t g_tick = 0;

/* Exported functions ------------------------------------------------------*/
void TimerExpire_Init(void)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    g_numOfHandles = 0;
    g_tick = 0;
    g_state = STATE_READY;
}

void TimerExpire_Register(TimerExpire_Handle_t *handle, Bool_t oneShot, float period, TimerExpire_Callback_t callback)
{
    if (g_state != STATE_READY) 
    {
        return;
    }

    handle->callback = callback;
    handle->periodInTicks = (uint32_t)(TimerExpire_GetTicks() * period);
    handle->oneShot = oneShot;
    g_registeredHandles[g_numOfHandles] = handle;
    g_numOfHandles++;
}

void TimerExpire_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    for (uint16_t i = 0; i <g_numOfHandles; i++ )
    {
        g_registeredHandles[i]->scheduled = FALSE;
        g_registeredHandles[i]->activated = FALSE;
    }

    g_state = STATE_OPERATING;
}

void TimerExpire_Activate(TimerExpire_Handle_t *handle)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    
    handle->scheduled = TRUE;
}

void TimerExpire_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_state = STATE_READY;
}

uint32_t TimerExpire_GetTicks(void)
{
    return g_tick;
}

float TimerExpire_GetTickFrequency(void)
{
    return TIMER_EXPIRE_TICK_FREQUENCY;
}

void TimerExpire_TickFunc(void)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    g_tick++;
    
    for (uint8_t i = 0; i < g_numOfHandles; i++) 
    {
        TimerExpire_Handle_t *handle = g_registeredHandles[i];

        if (handle->activated)
        {
            if (g_tick >= handle->triggerTick)
            {
                handle->activated = FALSE;
                handle->scheduled = !handle->oneShot;
                handle->callback(handle);
            }
        }
        
        if (handle->scheduled)
        {
            handle->activated = TRUE;
            handle->scheduled = FALSE;
            handle->triggerTick = g_tick + handle->periodInTicks;
        } 
    }    
}
