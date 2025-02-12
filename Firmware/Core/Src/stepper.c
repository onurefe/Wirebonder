#include "stepper.h"
#include "timer.h"    
#include "configuration.h"
#include "fast_io.h"
#include "delay.h"
#include <stdlib.h> 

/* Private definitions -----------------------------------------------------*/
#define TICKS_PER_SEGMENT \
  (TIMER2_ISR_FREQUENCY / BLOCK_EXECUTER_SEGMENT_RENDER_FREQUENCY)

#define STEP_PIN_HIGH_TIME_IN_US 1
#define STEP_PIN_LOW_TIME_IN_US  1

/* Private function prototypes ---------------------------------------------*/
static Bool_t isSegmentCompleted     (Stepper_Handle_t *handle);
static Bool_t isStepPulseRequired    (Stepper_Handle_t *handle);
static void   updateMotorPosition    (Stepper_Handle_t *handle);
static void   updateMotorDirection   (Stepper_Handle_t *handle);
static void   pulseStepPin           (Stepper_Handle_t *handle);
static void   setMotorDirectionForward  (Stepper_Handle_t *handle);
static void   setMotorDirectionBackward (Stepper_Handle_t *handle);

static GPIO_TypeDef *cfgEnPort = NULL;
static uint16_t cfgEnPin = 0xFFFF;

/* Exported functions ------------------------------------------------------*/
void Stepper_Init(Stepper_Handle_t *handle, 
                  GPIO_TypeDef *cfgEnPort, uint16_t cfgEnPin, 
                  GPIO_TypeDef *stepPort, uint16_t stepPin, 
                  GPIO_TypeDef *dirPort, uint16_t dirPin)
{
    // Initialize pins
    FastIO_GetPinObject(cfgEnPort, cfgEnPin, &handle->CfgEnPin);
    FastIO_GetPinObject(stepPort, stepPin, &handle->StepPin);
    FastIO_GetPinObject(dirPort, dirPin, &handle->DirPin);

    FastIO_PinMode(&handle->CfgEnPin, FASTIO_OUTPUT);
    FastIO_PinMode(&handle->StepPin,  FASTIO_OUTPUT);
    FastIO_PinMode(&handle->DirPin,   FASTIO_OUTPUT);

    FastIO_ClearPin(&handle->CfgEnPin);
    FastIO_ClearPin(&handle->StepPin);
    FastIO_ClearPin(&handle->DirPin);

    // Initialize queue
    Queue_InitBuffer(&handle->SegmentQueue,
                     handle->SegmentQueueContainer,
                     STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS);

    // Reset variables
    handle->MotorPosition            = 0;
    handle->State                    = STATE_READY;
    handle->SegmentWorkingMemory     = 0;
    handle->SegmentWorkingMemoryLoaded = FALSE;
    handle->SegmentTick              = 0;
    handle->PerformedSteps           = 0;
}

void Stepper_Start(Stepper_Handle_t *handle)
{
    // Register a single timer callback if needed.
    // Alternatively, you can call Stepper_TimerTick() from the timer ISR manually
    // for each stepper handle. For example:
    //
    // Timer2_RegisterCommonCallback(mySharedTimerFunction);
    // and inside mySharedTimerFunction:
    //   Stepper_TimerTick(&handle1);
    //   Stepper_TimerTick(&handle2);
    //
    // For simplicity, let's assume we just do it once globally in user code.

    handle->SegmentWorkingMemoryLoaded = FALSE;
    handle->State = STATE_OPERATING;
}

void Stepper_Restart(Stepper_Handle_t *handle)
{
    handle->State = STATE_READY;

    handle->SegmentWorkingMemoryLoaded = FALSE;

    FastIO_ClearPin(&handle->CfgEnPin);
    FastIO_ClearPin(&handle->StepPin);
    FastIO_ClearPin(&handle->DirPin);

    Queue_ClearBuffer(&handle->SegmentQueue);

    handle->State = STATE_OPERATING;
}

void Stepper_Execute(Stepper_Handle_t *handle)
{
    // In your architecture, this might be empty or used
    // if you want a "polling" approach instead of timer-based.
    // But here, logic is in Stepper_TimerTick, so we do nothing.
    if (handle->State != STATE_OPERATING)
    {
        return;
    }
}

void Stepper_Stop(Stepper_Handle_t *handle)
{
    FastIO_ClearPin(&handle->CfgEnPin);
    FastIO_ClearPin(&handle->StepPin);
    FastIO_ClearPin(&handle->DirPin);

    Queue_ClearBuffer(&handle->SegmentQueue);
    handle->State = STATE_READY;
}

void Stepper_EnqueueSegment(Stepper_Handle_t *handle, qint7_8_t segment)
{
    Queue_Enqueue(&handle->SegmentQueue, segment);
}

Bool_t Stepper_SegmentQueueIsAvailable(Stepper_Handle_t *handle)
{
    return (!Queue_IsFull(&handle->SegmentQueue));
}

uint16_t Stepper_GetPendingSegmentCount(Stepper_Handle_t *handle)
{
    return Queue_GetElementCount(&handle->SegmentQueue);
}

void Stepper_ClearSegmentQueue(Stepper_Handle_t *handle)
{
    Queue_ClearBuffer(&handle->SegmentQueue);
}

int32_t Stepper_GetMotorPosition(Stepper_Handle_t *handle)
{
    return handle->MotorPosition;
}

Bool_t Stepper_IsMovingForward(Stepper_Handle_t *handle)
{
    if ((handle->SegmentWorkingMemory > 0) && handle->SegmentWorkingMemoryLoaded)
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
    if ((handle->SegmentWorkingMemory < 0) && handle->SegmentWorkingMemoryLoaded)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Stepper_TimerTick(Stepper_Handle_t *handle)
{
    if (handle->State != STATE_OPERATING)
    {
        return;
    }

    // If a segment is currently “active”
    if (handle->SegmentWorkingMemoryLoaded)
    {
        handle->SegmentTick++;

        if (isStepPulseRequired(handle))
        {
            pulseStepPin(handle);
            handle->PerformedSteps++;
        }

        if (isSegmentCompleted(handle))
        {
            updateMotorPosition(handle);
            handle->SegmentWorkingMemoryLoaded = FALSE;
        }
    }

    // If the previous segment has completed, load the next one
    if (!handle->SegmentWorkingMemoryLoaded)
    {
        if (!Queue_IsEmpty(&handle->SegmentQueue))
        {
            handle->SegmentWorkingMemory = Queue_Dequeue(&handle->SegmentQueue);
            handle->SegmentWorkingMemoryLoaded = TRUE;

            handle->PerformedSteps = 0;
            handle->SegmentTick   = 0;

            updateMotorDirection(handle);
        }
    }
}

/* Private functions -------------------------------------------------------*/
static void updateMotorPosition(Stepper_Handle_t *handle)
{
    // Add or subtract the steps performed
    if (handle->SegmentWorkingMemory > 0)
    {
        handle->MotorPosition += handle->PerformedSteps;
    }
    else
    {
        handle->MotorPosition -= handle->PerformedSteps;
    }
}

static Bool_t isSegmentCompleted(Stepper_Handle_t *handle)
{
    return (handle->SegmentTick >= TICKS_PER_SEGMENT);
}

/*
 * Bresenham-like approach to decide if a step is due.
 * 
 * total_steps = absolute steps needed.
 * We compare a scaled “segmentTick * totalSteps” against “PerformedSteps * TICKS_PER_SEGMENT”.
 */
static Bool_t isStepPulseRequired(Stepper_Handle_t *handle)
{
    quint8_8_t total_steps = abs(handle->SegmentWorkingMemory);

    // Evaluate if:
    //  (SegmentTick * total_steps) / 256  >  PerformedSteps * TICKS_PER_SEGMENT / 256
    // But done with fixed-point style:
    //  (((quint24_8_t)handle->SegmentTick) * total_steps) >> 8
    //     vs
    //  ((uint16_t)handle->PerformedSteps) * TICKS_PER_SEGMENT
    return (
      ((((quint24_8_t)handle->SegmentTick) * total_steps) >> 8) >
      (((uint16_t)handle->PerformedSteps) * TICKS_PER_SEGMENT)
    );
}

static void updateMotorDirection(Stepper_Handle_t *handle)
{
    if (handle->SegmentWorkingMemory >= 0)
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
    FastIO_SetPin(&handle->StepPin);
    delayMicroseconds(STEP_PIN_HIGH_TIME_IN_US);
    FastIO_ClearPin(&handle->StepPin);
    delayMicroseconds(STEP_PIN_LOW_TIME_IN_US);
}

static void setMotorDirectionForward(Stepper_Handle_t *handle)
{
#if STEPPER_DIR_PIN_INVERT
    FastIO_SetPin(&handle->DirPin);
#else
    FastIO_ClearPin(&handle->DirPin);
#endif
}

static void setMotorDirectionBackward(Stepper_Handle_t *handle)
{
#if STEPPER_DIR_PIN_INVERT
    FastIO_ClearPin(&handle->DirPin);
#else
    FastIO_SetPin(&handle->DirPin);
#endif
}
