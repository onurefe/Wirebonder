#include "solenoid_driver.h"
#include "fast_io.h"
#include "timer.h"

/* Private variables -----------------------------------------*/
SolenoidDriver_Handle_t *g_solenoidHandles[SOLENOID_DRIVER_MAX_SOLENOIDS];
uint8_t g_numSolenoids;

State_t g_state = STATE_UNINIT;

/* Exported functions ----------------------------------------*/
typedef void (*SolenoidDriver_PositionChangedCallback_t)(void *solenoidHandle, 
                                                         SolenoidDriver_Position_t previousPosition,
                                                         SolenoidDriver_Position_t newPosition);


/* Private functions -----------------------------------------*/
void deadzonePeriodCompleted(SolenoidDriver_Handle_t *handle)
{
    if (handle->targetPosition == SOLENOID_DRIVER_POSITION_LOW)
    {
        FastIO_SetPin(&handle->lowPin);
    }
    else if (handle->targetPosition == SOLENOID_DRIVER_POSITION_HIGH)
    {
        FastIO_ClearPin(&handle->highPin);
    }
}

void transitionPeriodCompleted(SolenoidDriver_Handle_t *handle)
{
    handle->callback(handle, handle->currentPosition, handle->targetPosition);
    handle->currentPosition = handle->targetPosition;

    FastIO_ClearPin(&handle->lowPin);
    FastIO_ClearPin(&handle->highPin);
}

Bool_t isTransiting(SolenoidDriver_Handle_t *handle)
{
    if (handle->currentPosition != handle->targetPosition) 
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t isElapsed(SolenoidDriver_Handle_t *handle, float time)
{
    float tick_frequency = Timer_GetTickFrequency();
    if (handle->stopWatch > (time * tick_frequency))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t isTransitionCompleted(SolenoidDriver_Handle_t *handle)
{
    float tick_frequency = Timer_GetTickFrequency();
    if (handle->stopWatch > (SOLENOID_DRIVER_TRANSITION_TIME * tick_frequency))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void timerTickCallback(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    for (uint8_t i = 0; i <g_numSolenoids; i++)
    {
        SolenoidDriver_Handle_t *handle = g_solenoidHandles[i];
        if (!isTransiting(handle))
        {
            continue;
        }
        
        handle->stopWatch++;

        if (isElapsed(handle, SOLENOID_DRIVER_TRANSITION_TIME))
        {
            transitionPeriodCompleted(handle);
        }
        else if (isElapsed(handle, SOLENOID_DRIVER_DEADZONE_DELAY))
        {
            deadzonePeriodCompleted(handle);
        }
    }    
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
    FastIO_GetPinObject(highGPIOx, highPin, &handle->highPin);
    FastIO_GetPinObject(lowGPIOx, lowPin, &handle->lowPin);
    
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
        handle->stopWatch = 0;

        FastIO_ClearPin(&handle->highPin);
        FastIO_ClearPin(&handle->lowPin);
    }  

    Timer_Register(timerTickCallback);
    g_state = STATE_OPERATING;
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

        FastIO_ClearPin(&handle->lowPin);
        FastIO_ClearPin(&handle->highPin);
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
        handle->stopWatch = 0;
        FastIO_SetPin(&handle->lowPin);   
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
        handle->stopWatch = 0;
        
        FastIO_SetPin(&handle->highPin);   
    }
}