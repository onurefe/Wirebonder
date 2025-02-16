#include "stepper.h"
#include "configuration.h"
#include "fast_io.h"
#include "delay.h"
#include <stdlib.h> 

/* Private definitions -----------------------------------------------------*/
#define TICKS_PER_SEGMENT \
  (STEPPER_TIMER_ISR_FREQUENCY / BLOCK_EXECUTER_SEGMENT_RENDER_FREQUENCY)

#define STEP_PIN_HIGH_TIME_IN_US 1
#define STEP_PIN_LOW_TIME_IN_US  1

/* Private variables -------------------------------------------------------*/
Stepper_Handle_t *g_stepperHandles[STEPPER_MAX_NUM_OF_MOTORS];
uint8_t g_numSteppers;
State_t g_state = STATE_UNINIT;

/* Private function prototypes ---------------------------------------------*/
static Bool_t isSegmentCompleted     (Stepper_Handle_t *handle);
static Bool_t isStepPulseRequired    (Stepper_Handle_t *handle);
static void   updateMotorPosition    (Stepper_Handle_t *handle);
static void   updateMotorDirection   (Stepper_Handle_t *handle);
static void   pulseStepPin           (Stepper_Handle_t *handle);
static void   setMotorDirectionForward  (Stepper_Handle_t *handle);
static void   setMotorDirectionBackward (Stepper_Handle_t *handle);

static FastIO_Pin_t g_enPin;
static FastIO_Pin_t g_rstPin;

/* Exported functions ------------------------------------------------------*/
void Stepper_Init(GPIO_TypeDef *enPort, 
                  uint16_t enPin, 
                  GPIO_TypeDef *rstPort, 
                  uint16_t rstPin)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    FastIO_GetPinObject(enPort, enPin, TRUE, &g_enPin);
    FastIO_GetPinObject(rstPort, rstPin, FALSE, &g_rstPin);

    g_numSteppers = 0;

    g_state = STATE_READY;
}

void Stepper_Register(Stepper_Handle_t *handle, 
                      GPIO_TypeDef *stepPort, 
                      uint16_t stepPin, 
                      GPIO_TypeDef *dirPort, 
                      uint16_t dirPin)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    FastIO_GetPinObject(stepPort, stepPin, FALSE, &handle->stepPin);
    FastIO_GetPinObject(dirPort, dirPin, FALSE, &handle->dirPin);

    g_stepperHandles[g_numSteppers++] = handle;
}

void Stepper_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    FastIO_ClearPin(&g_enPin);
    FastIO_SetPin(&g_rstPin);

    for (uint8_t i = 0; i < g_numSteppers; i++)
    {
        Stepper_Handle_t *handle = g_stepperHandles[i];

        // Initialize queue
        Queue_InitBuffer(&handle->segmentQueue,
                         handle->segmentQueueContainer,
                         STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS);

        // Reset variables
        handle->motorPosition            = 0;
        handle->segmentWorkingMemory     = 0;
        handle->segmentWorkingMemoryLoaded = FALSE;
        handle->segmentTick              = 0;
        handle->performedSteps           = 0;

        FastIO_ClearPin(&handle->stepPin);
        FastIO_ClearPin(&handle->dirPin);
    }
}

void Stepper_Restart(Stepper_Handle_t *handle)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    handle->segmentWorkingMemoryLoaded = FALSE;

    FastIO_ClearPin(&handle->stepPin);
    FastIO_ClearPin(&handle->dirPin);

    Queue_ClearBuffer(&handle->segmentQueue);
}

void Stepper_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    FastIO_SetPin(&g_enPin);

    for (uint8_t i = 0; i < g_numSteppers; i++)
    {
        Stepper_Handle_t *handle = g_stepperHandles[i];

        FastIO_ClearPin(&handle->stepPin);
        FastIO_ClearPin(&handle->dirPin);

        Queue_ClearBuffer(&handle->segmentQueue);
    }

    g_state = STATE_READY;
}

void Stepper_EnqueueSegment(Stepper_Handle_t *handle, qint7_8_t segment)
{
    Queue_Enqueue(&handle->segmentQueue, segment);
}

Bool_t Stepper_SegmentQueueIsAvailable(Stepper_Handle_t *handle)
{
    return (!Queue_IsFull(&handle->segmentQueue));
}

uint16_t Stepper_GetPendingSegmentCount(Stepper_Handle_t *handle)
{
    return Queue_GetElementCount(&handle->segmentQueue);
}

void Stepper_ClearSegmentQueue(Stepper_Handle_t *handle)
{
    Queue_ClearBuffer(&handle->segmentQueue);
}

int32_t Stepper_GetMotorPosition(Stepper_Handle_t *handle)
{
    return handle->motorPosition;
}

Bool_t Stepper_IsMovingForward(Stepper_Handle_t *handle)
{
    if ((handle->segmentWorkingMemory > 0) && handle->segmentWorkingMemoryLoaded)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Stepper_IsMovingBackward(Stepper_Handle_t *handle)
{
    if ((handle->segmentWorkingMemory < 0) && handle->segmentWorkingMemoryLoaded)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Stepper_TimerTick(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    for (uint8_t i = 0; i < g_numSteppers; i++)
    {
        Stepper_Handle_t *handle = g_stepperHandles[i];

        // If a segment is currently “active”
        if (handle->segmentWorkingMemoryLoaded)
        {
            handle->segmentTick++;

            if (isStepPulseRequired(handle))
            {
                pulseStepPin(handle);
                handle->performedSteps++;
            }

            if (isSegmentCompleted(handle))
            {
                updateMotorPosition(handle);
                handle->segmentWorkingMemoryLoaded = FALSE;
            }
        }

        // If the previous segment has completed, load the next one
        if (!handle->segmentWorkingMemoryLoaded)
        {
            if (!Queue_IsEmpty(&handle->segmentQueue))
            {
                handle->segmentWorkingMemory = Queue_Dequeue(&handle->segmentQueue);
                handle->segmentWorkingMemoryLoaded = TRUE;

                handle->performedSteps = 0;
                handle->segmentTick   = 0;

                updateMotorDirection(handle);
            }
        }
    }
}

/* Private functions -------------------------------------------------------*/
static void updateMotorPosition(Stepper_Handle_t *handle)
{
    // Add or subtract the steps performed
    if (handle->segmentWorkingMemory > 0)
    {
        handle->motorPosition += handle->performedSteps;
    }
    else
    {
        handle->motorPosition -= handle->performedSteps;
    }
}

static Bool_t isSegmentCompleted(Stepper_Handle_t *handle)
{
    return (handle->segmentTick >= TICKS_PER_SEGMENT);
}

static Bool_t isStepPulseRequired(Stepper_Handle_t *handle)
{
    quint8_8_t total_steps = abs(handle->segmentWorkingMemory);

    return (
      ((((quint24_8_t)handle->segmentTick) * total_steps) >> 8) >
      (((uint16_t)handle->performedSteps) * TICKS_PER_SEGMENT)
    );
}

static void updateMotorDirection(Stepper_Handle_t *handle)
{
    if (handle->segmentWorkingMemory >= 0)
    {
        setMotorDirectionForward(handle);
    }
    else
    {
        setMotorDirectionBackward(handle);
    }
}

static void pulseStepPin(Stepper_Handle_t *handle)
{
    FastIO_SetPin(&handle->stepPin);
    delayMicroseconds(STEP_PIN_HIGH_TIME_IN_US);
    FastIO_ClearPin(&handle->stepPin);
    delayMicroseconds(STEP_PIN_LOW_TIME_IN_US);
}

static void setMotorDirectionForward(Stepper_Handle_t *handle)
{
#if STEPPER_DIR_PIN_INVERT
    FastIO_SetPin(&handle->DirPin);
#else
    FastIO_ClearPin(&handle->dirPin);
#endif
}

static void setMotorDirectionBackward(Stepper_Handle_t *handle)
{
#if STEPPER_DIR_PIN_INVERT
    FastIO_ClearPin(&handle->DirPin);
#else
    FastIO_SetPin(&handle->dirPin);
#endif
}
