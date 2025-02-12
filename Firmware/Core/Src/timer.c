#include "timer.h"
#include "generic.h"

/* Private variables -------------------------------------------------------*/
static TIM_HandleTypeDef *g_htim;

static Timer_TickDelegate_t g_registeredDelegates[TIMER_MAX_NUM_DELEGATES] = {NULL};

static uint8_t g_numOfDelegates = 0;

static State_t g_state = STATE_UNINIT;

static uint32_t g_ticks = 0;

/* Exported functions ------------------------------------------------------*/
void Timer_Init(TIM_HandleTypeDef *htim)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    g_htim = htim;
    g_numOfDelegates = 0;
    g_ticks = 0;
    g_state = STATE_READY;
}

void Timer_Register(Timer_TickDelegate_t timerDelegate)
{
    if (g_state != STATE_READY) 
    {
        return;
    }

    if (g_numOfDelegates < TIMER_MAX_NUM_DELEGATES) {
        g_registeredDelegates[g_numOfDelegates++] = timerDelegate;
    }
}

void Timer_Start(void)
{
    if (HAL_TIM_Base_Start_IT(g_htim) != HAL_OK) 
    {
        while (TRUE);
    }

    g_state = STATE_OPERATING;
}

void Timer_Stop(void)
{
    if (HAL_TIM_Base_Stop_IT(g_htim) != HAL_OK) 
    {
        while (TRUE);    
    }

    g_state = STATE_READY;
}

uint32_t Timer_GetTicks(void)
{
    return g_ticks;
}

float Timer_GetTickFrequency(void)
{
    return TIMER_TICK_FREQUENCY;
}

/* The HAL timer interrupt callback ----------------------------------------*/
/* This function is called inside HAL_TIM_IRQHandler when TIM2 update event occurs */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == g_htim->Instance) {
        if (g_state == STATE_OPERATING) {
            g_ticks++;
            for (uint8_t i = 0; i < g_numOfDelegates; i++) {
                if (g_registeredDelegates[i] != NULL) {
                    g_registeredDelegates[i]();
                }
            }
        }
    }
}
