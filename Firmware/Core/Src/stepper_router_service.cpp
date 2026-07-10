#include "stepper_router_service.hpp"

// =======================================================================
// RouterChannel
// =======================================================================
RouterChannel::RouterChannel(StepperChannel *stepper, float maxVel, float maxAcc)
    : m_stepper(stepper)
    , m_maxVelocity(maxVel)
    , m_maxAcceleration(maxAcc)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
    , m_routeParams{}
    , m_isBusy(false)
    , m_position(0.0f)
    , m_targetPosition(0.0f)
    , m_numOfRenderedSegments(0)
    , m_lastRenderedStepperServicePosition(0)
{
}

void RouterChannel::addMoveCompleteListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void RouterChannel::append(float displacement)
{
    moveTo(m_position + displacement);
}

void RouterChannel::moveTo(float position)
{
    if (m_isBusy || m_stepper == nullptr) {
        return;
    }

    m_targetPosition = position;
    m_lastRenderedStepperServicePosition = 0;
    m_numOfRenderedSegments              = 0;
    m_isBusy                             = true;

    formTrapezoidRoute(m_targetPosition - m_position);
}

float RouterChannel::getPosition() const
{
    return m_position;
}

bool RouterChannel::isBusy() const
{
    return m_isBusy;
}

float RouterChannel::stepsToMillimeters(int32_t positionInSteps)
{
    return static_cast<float>(positionInSteps) /
           ROUTER_MODULE_STEP_PER_MM;
}

void RouterChannel::start()
{
    m_lastRenderedStepperServicePosition = 0;
    m_numOfRenderedSegments              = 0;
    m_isBusy                             = false;
    m_position                           = 0.0f;
    m_targetPosition                     = 0.0f;
    m_routeParams                        = {};
}

void RouterChannel::stop()
{
    m_isBusy = false;
}

void RouterChannel::execute()
{
    if (!m_isBusy || m_stepper == nullptr) {
        return;
    }

    if (!m_stepper->segmentQueueIsAvailable()) {
        return;
    }

    bool endOfRoute         = false;
    const float position    = getRoutePosition(endOfRoute);
    const qint7_8_t segment = getStepperServiceSegment(position);
    const bool noNewSegment = endOfRoute && (segment == 0);

    if (noNewSegment) {
        if (m_stepper->isIdle()) {
            m_isBusy = false;
            m_position = m_targetPosition;

            if (m_callback != nullptr) {
                m_callback(m_callbackContext, this);
            }
        }
        return;
    }

    m_stepper->enqueueSegment(segment);
    m_numOfRenderedSegments++;
}

// -------------------------------------------------------------------------
// Motion planning
// -------------------------------------------------------------------------
void RouterChannel::formTrapezoidRoute(float displacement)
{
    const float absDisplacement = std::abs(displacement);

    if (absDisplacement <= 0.0f ||
        m_maxVelocity <= 0.0f  ||
        m_maxAcceleration <= 0.0f)
    {
        m_routeParams   = {};
        m_routeParams.d = displacement;
        return;
    }

    const float maxAchievableVelocity = sqrtf(absDisplacement * m_maxAcceleration);
    const float targetVelocity        = std::min(m_maxVelocity, maxAchievableVelocity);

    const float tAcc  = targetVelocity / m_maxAcceleration;
    const float dAcc  = 0.5f * m_maxAcceleration * tAcc * tAcc;
    const float dDec  = targetVelocity * tAcc - 0.5f * m_maxAcceleration * tAcc * tAcc;
    const float dConst = absDisplacement - (dAcc + dDec);
    const float tConst = dConst / targetVelocity;

    const float sign = (displacement < 0.0f) ? -1.0f : 1.0f;

    m_routeParams.d      = displacement;
    m_routeParams.vMax   = targetVelocity  * sign;
    m_routeParams.aMax   = m_maxAcceleration * sign;
    m_routeParams.tAcc   = tAcc;
    m_routeParams.dAcc   = dAcc  * sign;
    m_routeParams.tConst = tConst;
    m_routeParams.dConst = dConst * sign;
}

float RouterChannel::getPositionAccelerating(float t) const
{
    return 0.5f * m_routeParams.aMax * t * t;
}

float RouterChannel::getPositionConstantVelocity(float t) const
{
    const float tDelta = t - m_routeParams.tAcc;
    return m_routeParams.dAcc + m_routeParams.vMax * tDelta;
}

float RouterChannel::getPositionDecelerating(float t) const
{
    const float x0     = m_routeParams.dAcc + m_routeParams.dConst;
    const float t0     = m_routeParams.tAcc + m_routeParams.tConst;
    const float tDelta = t - t0;
    return x0 + m_routeParams.vMax * tDelta - 0.5f * m_routeParams.aMax * tDelta * tDelta;
}

float RouterChannel::getRoutePosition(bool &endOfRoute) const
{
    const float t = static_cast<float>(m_numOfRenderedSegments) /
                    static_cast<float>(ROUTER_MODULE_SEGMENT_RENDER_FREQUENCY);

    endOfRoute = false;

    if (m_routeParams.tAcc > t) {
        return getPositionAccelerating(t);
    }

    if ((m_routeParams.tAcc + m_routeParams.tConst) > t) {
        return getPositionConstantVelocity(t);
    }

    if ((m_routeParams.tAcc + m_routeParams.tConst + m_routeParams.tAcc) > t) {
        return getPositionDecelerating(t);
    }

    endOfRoute = true;
    return m_routeParams.d;
}

qint55_8_t RouterChannel::convertToStepperServicePosition(float position) const
{
    return static_cast<qint55_8_t>(
        position * 256.0f * ROUTER_MODULE_STEP_PER_MM
    );
}

qint7_8_t RouterChannel::getStepperServiceSegment(float destinationPosition)
{
    const qint55_8_t destinationInSteps = convertToStepperServicePosition(destinationPosition);

    qint55_8_t segmentDisplacement = destinationInSteps - m_lastRenderedStepperServicePosition;

    segmentDisplacement = std::clamp<qint55_8_t>(segmentDisplacement, MIN_INT16, MAX_INT16);

    m_lastRenderedStepperServicePosition += segmentDisplacement;

    return static_cast<qint7_8_t>(segmentDisplacement);
}

// =======================================================================
// StepperRouterService
// =======================================================================
StepperRouterService::StepperRouterService()
    : m_numChannels(0)
    , m_state(ServiceState::READY)
{
    for (uint8_t i = 0; i < ROUTER_MODULE_MAX_NUM_OF_ROUTERS; i++) {
        m_channels[i] = nullptr;
    }
}

bool StepperRouterService::addChannel(RouterChannel *channel)
{
    if (channel == nullptr || m_numChannels >= ROUTER_MODULE_MAX_NUM_OF_ROUTERS) {
        return false;
    }

    m_channels[m_numChannels++] = channel;
    return true;
}

void StepperRouterService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->start();
    }

    m_state = ServiceState::OPERATING;
}

void StepperRouterService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->stop();
    }

    m_state = ServiceState::READY;
}

void StepperRouterService::executeService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->execute();
    }
}
