#ifndef STEPPER_H
#define STEPPER_H

#include "generic.h"
#include "fast_io.h"
#include "queue.h"        // or wherever Queue_Buffer_t etc. are defined
#include "configuration.h"

// Forward declarations or includes for any needed data types:
#include <stdint.h>       // for int32_t, etc.

#define STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS 16u // example

/* 
 * Each stepper has its own handle that carries:
 *  - The current state
 *  - The position
 *  - The segment queue
 *  - The internal counters for the Bresenham-like calculation
 *  - The pins (En, Step, Dir)
 */
typedef struct
{
    // Module control state
    volatile State_t State;

    // Bresenham-related
    volatile uint16_t SegmentTick;
    volatile uint16_t PerformedSteps;

    // Monitoring
    volatile int32_t MotorPosition;

    // Segment working memory
    volatile qint7_8_t SegmentWorkingMemory;
    volatile Bool_t    SegmentWorkingMemoryLoaded;

    // Segment queue
    Queue_Buffer_t SegmentQueue; 
    qint7_8_t SegmentQueueContainer[QUEUE_REQUIRED_BUFFER_SIZE(STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS)];

    // Fast IO pins
    FastIO_Pin_t CfgEnPin;
    FastIO_Pin_t StepPin;
    FastIO_Pin_t DirPin;
} Stepper_Handle_t;

/* Exported functions ------------------------------------------------------*/
void Stepper_Init(Stepper_Handle_t *handle, 
                  GPIO_TypeDef *cfgEnPort, uint16_t cfgEnPin, 
                  GPIO_TypeDef *stepPort, uint16_t stepPin, 
                  GPIO_TypeDef *dirPort, uint16_t dirPin);

void  Stepper_Start             (Stepper_Handle_t *handle);
void  Stepper_Restart           (Stepper_Handle_t *handle);
void  Stepper_Execute           (Stepper_Handle_t *handle);
void  Stepper_Stop              (Stepper_Handle_t *handle);

void  Stepper_EnqueueSegment    (Stepper_Handle_t *handle, qint7_8_t segment);
Bool_t Stepper_SegmentQueueIsAvailable(Stepper_Handle_t *handle);
uint16_t Stepper_GetPendingSegmentCount (Stepper_Handle_t *handle);
void  Stepper_ClearSegmentQueue (Stepper_Handle_t *handle);

int32_t Stepper_GetMotorPosition(Stepper_Handle_t *handle);
Bool_t  Stepper_IsMovingForward (Stepper_Handle_t *handle);
Bool_t  Stepper_IsMovingBackward(Stepper_Handle_t *handle);

/*
 * This function is called from your timer interrupt service routine (ISR).
 * In your timer ISR, you call Stepper_TimerTick(&handle1);
 * and Stepper_TimerTick(&handle2); for each stepper you have.
 */
void Stepper_TimerTick          (Stepper_Handle_t *handle);

#endif /* STEPPER_H */
