#ifndef STEPPER_H
#define STEPPER_H

#include "generic.h"
#include "fast_io.h"
#include "queue.h"        // or wherever Queue_Buffer_t etc. are defined
#include "configuration.h"

// Forward declarations or includes for any needed data types:
#include <stdint.h>       // for int32_t, etc.

#define STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS 16u // example

typedef struct
{
    // Bresenham-related
    volatile uint16_t segmentTick;
    volatile uint16_t performedSteps;

    // Monitoring
    volatile int32_t motorPosition;

    // Segment working memory
    volatile qint7_8_t segmentWorkingMemory;
    volatile Bool_t    segmentWorkingMemoryLoaded;

    // Segment queue
    Queue_Buffer_t segmentQueue; 
    qint7_8_t segmentQueueContainer[QUEUE_REQUIRED_BUFFER_SIZE(STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS)];

    // IO pins.
    FastIO_Pin_t stepPin;
    FastIO_Pin_t dirPin;
} Stepper_Handle_t;


/* Exported functions ------------------------------------------------------*/
void Stepper_Init(GPIO_TypeDef *enPort, 
                  uint16_t enPin,
                  GPIO_TypeDef *rstPort, 
                  uint16_t rstPin);

void Stepper_Register(Stepper_Handle_t *handle, 
                      GPIO_TypeDef *stepPort, uint16_t stepPin, 
                      GPIO_TypeDef *dirPort, uint16_t dirPin);

void  Stepper_Start(void);
void  Stepper_Restart(Stepper_Handle_t *handle);
void  Stepper_Stop(void);

void  Stepper_EnqueueSegment(Stepper_Handle_t *handle, qint7_8_t segment);
Bool_t Stepper_SegmentQueueIsAvailable(Stepper_Handle_t *handle);
uint16_t Stepper_GetPendingSegmentCount(Stepper_Handle_t *handle);
void  Stepper_ClearSegmentQueue(Stepper_Handle_t *handle);

int32_t Stepper_GetMotorPosition(Stepper_Handle_t *handle);
Bool_t  Stepper_IsMovingForward (Stepper_Handle_t *handle);
Bool_t  Stepper_IsMovingBackward(Stepper_Handle_t *handle);

void Stepper_TimerTick(void);

#endif /* STEPPER_H */
