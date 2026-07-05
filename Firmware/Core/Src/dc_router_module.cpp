#include "dc_router_module.hpp"
#include <cmath>

DcRouterModule::DcRouterModule(
    DcMotorPositionControllerModule *positionController,
    Timer *settlingTimer,
    float maxVelocity,
    float maxAcceleration)
    : m_positionController(positionController)
    , m_settlingTimer(settlingTimer)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
    , m_state(ServiceState::READY)
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_startingPosition(0.0f)
    , m_destinationPosition(0.0f)
    , m_moveInProgress(false)
    , m_segmentCounter(0)
{
    registerPeripheralCallbacks();
}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

void DcRouterModule::start(float initialPosition)
{
    if (!isReady()) {
        return;
    }

    m_state = ServiceState::OPERATING;

    m_positionController->start();

    beginMove(initialPosition);
}

void DcRouterModule::stop()
{
    if (!isOperating()) {
        return;
    }

    m_state = ServiceState::READY;

    m_settlingTimer->stop();
    m_positionController->stop();

    m_moveInProgress = false;
}

void DcRouterModule::beginMove(float destinationPosition)
{
    if (!isOperating()) {
        return;
    }

    m_startingPosition = m_positionController->getPosition();
    m_destinationPosition = destinationPosition;
    planTrapezoidProfile();

    m_moveInProgress = true;
    m_segmentCounter = 0;

    m_positionController->restartControlLoop();

    m_settlingTimer->start(true, DCROUTER_MODULE_SETTLING_TIME_WINDOW);
}

float DcRouterModule::getPosition() const
{
    return m_positionController->getPosition();
}

void DcRouterModule::addEventListenerCallback(EventOccurredCallback callback, void *context)
{
    m_callback = callback;
    m_callbackContext = context;
}

// ---------------------------------------------------------------------------
// Peripheral-Bridge-Callbacks
// ---------------------------------------------------------------------------

bool DcRouterModule::setpointSourceCallback(void *context, float *positionSetpoint)
{
    return static_cast<DcRouterModule *>(context)->onProvideSetpoint(positionSetpoint);
}

void DcRouterModule::settlingTimerCallback(void *context, Timer *timer)
{
    (void)timer;
    static_cast<DcRouterModule *>(context)->onSettlingTimerExpireServiced();
}

// ---------------------------------------------------------------------------
// Event-Handlers
// ---------------------------------------------------------------------------

bool DcRouterModule::onProvideSetpoint(float *positionSetpoint)
{
    if (positionSetpoint == nullptr) {
        return false;
    }

    if (!isOperating()) {
        return false;
    }

    if (m_moveInProgress) {
        bool segment_completed;
        float time;

        m_segmentCounter++;
        time = (float)m_segmentCounter / DCROUTER_MODULE_CONTROL_FREQUENCY;

        *positionSetpoint = plannedPosition(time, &segment_completed);

        m_moveInProgress &= !segment_completed;
    } else {
        *positionSetpoint = m_destinationPosition;
    }

    if (m_moveInProgress && setpointIsReached()) {
        completeMove();
    }

    return true;
}

void DcRouterModule::onSettlingTimerExpireServiced()
{
    if (!isOperating()) {
        return;
    }

    if (!m_moveInProgress) {
        return;
    }

    abortMove();
}

// ---------------------------------------------------------------------------
// Motion-Planning
// ---------------------------------------------------------------------------

void DcRouterModule::planTrapezoidProfile()
{
    const float dx = m_destinationPosition - m_startingPosition;

    m_profile.distance = fabsf(dx);
    m_profile.direction = (dx >= 0.0f) ? 1.0f : -1.0f;

    // Time and distance needed to accelerate from 0 to vMax.
    const float t_accel_to_vmax = m_maxVelocity / m_maxAcceleration;
    const float d_accel_to_vmax = 0.5f * m_maxAcceleration * t_accel_to_vmax * t_accel_to_vmax;

    // Check whether we can actually reach vMax (trapezoid vs. triangle).
    if (2.0f * d_accel_to_vmax >= m_profile.distance) {
        m_profile.accelTime = sqrtf(m_profile.distance / m_maxAcceleration);
        m_profile.cruiseTime = 0.0f;
        m_profile.peakVelocity = m_maxAcceleration * m_profile.accelTime;
    } else {
        m_profile.accelTime = t_accel_to_vmax;
        m_profile.peakVelocity = m_maxVelocity;

        const float d_cruise = m_profile.distance - 2.0f * d_accel_to_vmax;
        m_profile.cruiseTime = d_cruise / m_maxVelocity;
    }

    m_profile.totalTime = 2.0f * m_profile.accelTime + m_profile.cruiseTime;
}

float DcRouterModule::plannedPosition(float time, bool *segmentCompleted) const
{
    if (m_profile.distance <= 0.0f || time >= m_profile.totalTime) {
        *segmentCompleted = true;
        return m_destinationPosition;
    }

    *segmentCompleted = false;

    float planned_distance;

    if (time < m_profile.accelTime) {
        planned_distance = 0.5f * m_maxAcceleration * time * time;
    } else if (time < m_profile.accelTime + m_profile.cruiseTime) {
        const float d_accel = 0.5f * m_maxAcceleration * m_profile.accelTime * m_profile.accelTime;
        const float t_in_cruise = time - m_profile.accelTime;

        planned_distance = d_accel + m_profile.peakVelocity * t_in_cruise;
    } else {
        const float t_remaining = m_profile.totalTime - time;
        planned_distance = m_profile.distance - 0.5f * m_maxAcceleration * t_remaining * t_remaining;
    }

    return m_startingPosition + m_profile.direction * planned_distance;
}

// ---------------------------------------------------------------------------
// Move-Lifecycle
// ---------------------------------------------------------------------------

void DcRouterModule::completeMove()
{
    m_moveInProgress = false;
    m_settlingTimer->stop();

    publishEvent(Event::SETPOINT_ACHIEVED);
}

void DcRouterModule::abortMove()
{
    m_moveInProgress = false;
    m_destinationPosition = m_positionController->getPosition();

    m_settlingTimer->stop();
    publishEvent(Event::UNABLE_TO_REACH_SETPOINT);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void DcRouterModule::registerPeripheralCallbacks()
{
    m_positionController->addPositionSetpointControllerCallback(this, &DcRouterModule::setpointSourceCallback);
    m_settlingTimer->setExpirationListenerCallback(this, &DcRouterModule::settlingTimerCallback);
}

void DcRouterModule::publishEvent(Event event)
{
    if (m_callback == nullptr) {
        return;
    }

    m_callback(m_callbackContext, event);
}

bool DcRouterModule::isReady() const
{
    return m_state == ServiceState::READY;
}

bool DcRouterModule::isOperating() const
{
    return m_state == ServiceState::OPERATING;
}

bool DcRouterModule::setpointIsReached() const
{
    const float positionError =
        std::fabs(m_destinationPosition - m_positionController->getPosition());

    const float absoluteVelocity =
        std::fabs(m_positionController->getVelocity());

    return (positionError < DCROUTER_MODULE_MAX_POSITION_ERROR) &&
           (absoluteVelocity < DCROUTER_MODULE_MAX_VELOCITY_ERROR);
}
