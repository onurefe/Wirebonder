#ifndef STEPPER_SERVICE_HPP
#define STEPPER_SERVICE_HPP

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "generic.h"
#include "configuration.h"
#include "fast_io.hpp"
#include "queue.hpp"

// -----------------------------------------------------------------------
// Class: StepperChannel
// -----------------------------------------------------------------------
class StepperChannel {
public:
    StepperChannel(FastIO *stepPin, FastIO *dirPin);

    bool     enqueueSegment(qint7_8_t segment);
    bool     segmentQueueIsAvailable();
    uint16_t getPendingSegmentCount();
    bool     isIdle();
    void     clearSegmentQueue();

    int32_t getMotorPosition();
    bool    isMovingForward();
    bool    isMovingBackward();

    void restart();

    void tick();   // ISR context — called by StepperService
    void reset();  // task context — called by StepperService on start/stop

private:
    void clearStepPulse();
    void advanceActiveSegment();
    void loadNextSegmentIfIdle();

    void emitStepIfDue();
    void emitForwardStep();
    void emitBackwardStep();

    bool activeSegmentIsFinished() const;
    void finishActiveSegment();

    void updateDirectionPin();
    void setDirectionForward();
    void setDirectionBackward();

    void clearSegmentQueueUnlocked();

    FastIO *m_stepPin;
    FastIO *m_dirPin;

    volatile uint16_t  m_ticksInSegment;
    volatile int32_t   m_motorPosition;
    volatile qint7_8_t m_activeSegment;
    volatile bool      m_hasActiveSegment;
    volatile int32_t   m_stepAccumulatorQ8;

    qint7_8_t        m_segmentQueueStorage[STEPPER_SERVICE_SEGMENT_QUEUE_CAPACITY + 1U];
    Queue<qint7_8_t> m_segmentQueue;
};

// -----------------------------------------------------------------------
// Class: StepperService
// -----------------------------------------------------------------------
class StepperService {
public:
    StepperService(TIM_HandleTypeDef *htim, FastIO *enablePin, FastIO *resetPin);

    bool addChannel(StepperChannel *channel);

    void initService()    {}
    void startService();
    void stopService();
    void executeService() {}

    void handlePeriodElapsed(TIM_HandleTypeDef *htim);

    static void dispatchPeriodElapsed(TIM_HandleTypeDef *htim);

private:
    TIM_HandleTypeDef *m_htim;
    FastIO            *m_enablePin;
    FastIO            *m_resetPin;
    StepperChannel    *m_channels[STEPPER_SERVICE_MAX_MOTOR_COUNT];
    uint8_t            m_numChannels;
    ServiceState       m_state;
};

#endif /* STEPPER_SERVICE_HPP */
