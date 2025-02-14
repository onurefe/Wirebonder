#include "solenoid_driver.h"
#include "fast_io.h"
#include "timer_expire.h"

/* Private variables -----------------------------------------*/
SolenoidDriver_Handle_t *g_solenoidHandles[SOLENOID_DRIVER_MAX_SOLENOIDS];
uint8_t g_numSolenoids;
State_t g_state = STATE_UNINIT;

/* Exported functions ----------------------------------------*/
typedef void (*SolenoidDriver_PositionChangedCallback_t)(void *solenoidHandle, 
                                                         SolenoidDriver_Position_t previousPosition,
                                                         SolenoidDriver_Position_t newPosition);


/* Private functions -----------------------------------------*/
void transitionPeriodCompletedCallback(SolenoidDriver_Handle_t *handle)
{
    FastIO_Clear(&handle->lowPin);
    FastIO_Clear(&handle->highPin);

    handle->currentPosition = handle->targetPosition;
    handle->transitionCompleted = TRUE;
}

/* Exported functions ----------------------------------------*/
void SolenoidDriver_Init(void)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    g_numSolenoids = 0;
    g_state = STATE_READY;
}

void SolenoidDriver_Register(SolenoidDriver_Handle_t *handle,
                             GPIO_TypeDef *highGPIOx, 
                             GPIO_TypeDef *lowGPIOx, 
                             uint16_t highPin, 
                             uint16_t lowPin, 
                             SolenoidDriver_Position_t initalPosition,
                             SolenoidDriver_PositionChangedCallback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    handle->callback = callback;
    handle->currentPosition = SOLENOID_DRIVER_POSITION_UNKNOWN;
    handle->targetPosition = SOLENOID_DRIVER_POSITION_UNKNOWN;
    handle->initialPosition = initalPosition;

    FastIO_GetPinObject(highGPIOx, highPin, FALSE, &handle->highPin);
    FastIO_GetPinObject(lowGPIOx, lowPin, TRUE, &handle->lowPin);
    
    g_solenoidHandles[g_numSolenoids++] = handle;
}

void SolenoidDriver_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    for (uint8_t i = 0; i <g_numSolenoids; i++)
    {
        SolenoidDriver_Handle_t *handle = g_solenoidHandles[i];

        handle->currentPosition = SOLENOID_DRIVER_POSITION_UNKNOWN;
        handle->targetPosition = handle->initialPosition;
        handle->transitionCompleted = FALSE;

        TimerExpire_Register(&handle->transitionTimer, 
                             TRUE, 
                             SOLENOID_DRIVER_TRANSITION_TIME, 
                             transitionPeriodCompletedCallback);

        FastIO_Clear(&handle->highPin);
        FastIO_Clear(&handle->lowPin);
    }  

    HAL_Delay(SOLENOID_DRIVER_INITIALIZATION_DELAY_IN_MS);

    g_state = STATE_OPERATING;
}

void SolenoidDriver_Execute(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    for (uint8_t i = 0; i < g_numSolenoids; i++)
    {
        SolenoidDriver_Handle_t *handle = g_solenoidHandles[i];

        if (handle->transitionCompleted) 
        {
            handle->callback(handle, handle->currentPosition, handle->targetPosition);
            handle->transitionCompleted = FALSE;
        }
    }  
}

void SolenoidDriver_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    for (uint8_t i = 0; i <g_numSolenoids; i++)
    {
        SolenoidDriver_Handle_t *handle = g_solenoidHandles[i];

        FastIO_Clear(&handle->lowPin);
        FastIO_Clear(&handle->highPin);
    }  

    g_state = STATE_READY;
}

void SolenoidDriver_SetLowPosition(SolenoidDriver_Handle_t *handle)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* Check if there already is an ongoing operation. */
    if (handle->currentPosition != handle->targetPosition)
    {
        return;
    }

    /* If not already low, make transition to low position. */
    if (handle->currentPosition != SOLENOID_DRIVER_POSITION_LOW)
    {
        handle->targetPosition = SOLENOID_DRIVER_POSITION_LOW;
        
        TimerExpire_Activate(&handle->transitionTimer);

        FastIO_Set(&handle->lowPin);   
    }
}

void SolenoidDriver_SetHighPosition(SolenoidDriver_Handle_t *handle)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* Check if there already is an ongoing operation. */
    if (handle->currentPosition != handle->targetPosition)
    {
        return;
    }

    /* If not already high, make transition to high position. */
    if (handle->currentPosition != SOLENOID_DRIVER_POSITION_HIGH)
    {
        handle->targetPosition = SOLENOID_DRIVER_POSITION_HIGH;
        
        TimerExpire_Activate(&handle->transitionTimer);

        FastIO_SetPin(&handle->highPin);   
    }
}