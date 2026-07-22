#include "stepper_service.hpp"

static constexpr uint32_t kTicksPerSegment =
    static_cast<uint32_t>(STEPPER_ISR_FREQ / ROUTER_MODULE_SEGMENT_RENDER_FREQUENCY);

static constexpr int32_t kStepThresholdQ8 =
    static_cast<int32_t>(kTicksPerSegment * 256U);

static StepperService *g_instance = nullptr;

// =======================================================================
// StepperChannel
// =======================================================================
StepperChannel::StepperChannel(FastIO *stepPin, FastIO *dirPin)
    : m_stepPin(stepPin)
    , m_dirPin(dirPin)
    , m_ticksInSegment(0U)
    , m_motorPosition(0)
    , m_activeSegment(0)
    , m_hasActiveSegment(false)
    , m_stepAccumulatorQ8(0)
    , m_segmentQueueStorage{}
    , m_segmentQueue(m_segmentQueueStorage, STEPPER_SERVICE_SEGMENT_QUEUE_CAPACITY)
{
}

bool StepperChannel::enqueueSegment(qint7_8_t segment)
{
    InterruptLock lock;

    if (m_segmentQueue.isFull()) {
        return false;
    }

    m_segmentQueue.enqueue(segment);
    return true;
}

bool StepperChannel::segmentQueueIsAvailable()
{
    InterruptLock lock;
    return !m_segmentQueue.isFull();
}

uint16_t StepperChannel::getPendingSegmentCount()
{
    InterruptLock lock;
    return m_segmentQueue.getElementCount();
}

bool StepperChannel::isIdle()
{
    InterruptLock lock;
    return !m_hasActiveSegment && m_segmentQueue.isEmpty();
}

void StepperChannel::clearSegmentQueue()
{
    InterruptLock lock;
    clearSegmentQueueUnlocked();
}

int32_t StepperChannel::getMotorPosition()
{
    InterruptLock lock;
    return m_motorPosition;
}

bool StepperChannel::isMovingForward()
{
    InterruptLock lock;
    return m_hasActiveSegment && (m_activeSegment > 0);
}

bool StepperChannel::isMovingBackward()
{
    InterruptLock lock;
    return m_hasActiveSegment && (m_activeSegment < 0);
}

void StepperChannel::restart()
{
    InterruptLock lock;
    m_ticksInSegment    = 0U;
    m_activeSegment     = 0;
    m_hasActiveSegment  = false;
    m_stepAccumulatorQ8 = 0;
    clearSegmentQueueUnlocked();
    m_stepPin->clear();
    m_dirPin->clear();
}

void StepperChannel::reset()
{
    m_ticksInSegment    = 0U;
    m_activeSegment     = 0;
    m_hasActiveSegment  = false;
    m_stepAccumulatorQ8 = 0;
    m_segmentQueue.clear();
    m_stepPin->clear();
    m_dirPin->clear();
}

void StepperChannel::tick()
{
    clearStepPulse();
    advanceActiveSegment();
    loadNextSegmentIfIdle();
}

// -------------------------------------------------------------------------
// ISR helpers
// -------------------------------------------------------------------------
void StepperChannel::clearStepPulse()
{
    m_stepPin->clear();
}

void StepperChannel::advanceActiveSegment()
{
    if (!m_hasActiveSegment) {
        return;
    }

    m_ticksInSegment++;
    emitStepIfDue();

    if (activeSegmentIsFinished()) {
        finishActiveSegment();
    }
}

void StepperChannel::loadNextSegmentIfIdle()
{
    if (m_hasActiveSegment) {
        return;
    }

    if (m_segmentQueue.isEmpty()) {
        return;
    }

    m_activeSegment    = m_segmentQueue.dequeue();
    m_hasActiveSegment = true;
    m_ticksInSegment   = 0U;

    updateDirectionPin();
}

void StepperChannel::emitStepIfDue()
{
    m_stepAccumulatorQ8 += static_cast<int32_t>(m_activeSegment);

    if (m_stepAccumulatorQ8 >= kStepThresholdQ8) {
        m_stepAccumulatorQ8 -= kStepThresholdQ8;
        emitForwardStep();
        return;
    }

    if (m_stepAccumulatorQ8 <= -kStepThresholdQ8) {
        m_stepAccumulatorQ8 += kStepThresholdQ8;
        emitBackwardStep();
        return;
    }
}

void StepperChannel::emitForwardStep()
{
    m_stepPin->set();
    m_motorPosition++;
}

void StepperChannel::emitBackwardStep()
{
    m_stepPin->set();
    m_motorPosition--;
}

bool StepperChannel::activeSegmentIsFinished() const
{
    return m_ticksInSegment >= kTicksPerSegment;
}

void StepperChannel::finishActiveSegment()
{
    m_hasActiveSegment = false;
    m_activeSegment    = 0;
    m_ticksInSegment   = 0U;
}

void StepperChannel::updateDirectionPin()
{
    if (m_activeSegment > 0) {
        setDirectionForward();
        return;
    }

    if (m_activeSegment < 0) {
        setDirectionBackward();
        return;
    }
}

void StepperChannel::setDirectionForward()
{
#if STEPPER_SERVICE_DIR_PIN_INVERT
    m_dirPin->set();
#else
    m_dirPin->clear();
#endif
}

void StepperChannel::setDirectionBackward()
{
#if STEPPER_SERVICE_DIR_PIN_INVERT
    m_dirPin->clear();
#else
    m_dirPin->set();
#endif
}

void StepperChannel::clearSegmentQueueUnlocked()
{
    m_segmentQueue.clear();
}

// =======================================================================
// StepperService
// =======================================================================
StepperService::StepperService(TIM_HandleTypeDef *htim, FastIO *enablePin, FastIO *resetPin)
    : m_htim(htim)
    , m_enablePin(enablePin)
    , m_resetPin(resetPin)
    , m_numChannels(0)
{
    for (uint8_t i = 0; i < STEPPER_SERVICE_MAX_MOTOR_COUNT; i++) {
        m_channels[i] = nullptr;
    }

    g_instance = this;
}

bool StepperService::addChannel(StepperChannel *channel)
{
    if (channel == nullptr || m_numChannels >= STEPPER_SERVICE_MAX_MOTOR_COUNT) {
        return false;
    }

    m_channels[m_numChannels++] = channel;
    return true;
}

void StepperService::onStart()
{
    m_enablePin->clear();
    m_resetPin->set();

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->reset();
    }

    HAL_TIM_Base_Start_IT(m_htim);
}

void StepperService::onStop()
{
    HAL_TIM_Base_Stop_IT(m_htim);

    m_enablePin->set();

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->reset();
    }
}

void StepperService::handlePeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != m_htim->Instance) {
        return;
    }

    if (!isOperating()) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->tick();
    }
}

void StepperService::dispatchPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (g_instance != nullptr) {
        g_instance->handlePeriodElapsed(htim);
    }
}

// =======================================================================
// HAL callback
// =======================================================================
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    StepperService::dispatchPeriodElapsed(htim);
}
