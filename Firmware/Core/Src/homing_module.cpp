#include "homing_module.hpp"

#include "configuration.h"

static_assert(ROBOT_Y_AXIS_HOMING_DIRECTION == -1 ||
              ROBOT_Y_AXIS_HOMING_DIRECTION == 1,
              "Y homing direction must be -1 or +1");
static_assert(ROBOT_Y_AXIS_WORKSPACE_SIZE_MM > 0.0f,
              "Y workspace must be positive");
static_assert(ROBOT_Y_AXIS_HOMING_BACKOFF_MM > 0.0f &&
              ROBOT_Y_AXIS_HOMING_BACKOFF_MM < ROBOT_Y_AXIS_WORKSPACE_SIZE_MM,
              "Y homing backoff must lie inside the workspace");
static_assert(ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S > 0.0f &&
              ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S <= ROBOT_Y_AXIS_MAX_VELOCITY,
              "Y homing velocity must not exceed the router maximum");

HomingModule::HomingModule(RouterChannel *yAxisRouter,
                           PinMonitorChannel *yAxisLimitSwitch)
    : m_yAxisRouter(yAxisRouter)
    , m_yAxisLimitSwitch(yAxisLimitSwitch)
    , m_eventCallback(nullptr)
    , m_eventCallbackContext(nullptr)
    , m_limitActive(false)
    , m_homingState(HomingState::Idle)
{}

void HomingModule::addEventListenerCallback(void *context,
                                            EventCallback callback)
{
    m_eventCallbackContext = context;
    m_eventCallback = callback;
}

void HomingModule::onStart()
{
    if (m_yAxisRouter == nullptr || m_yAxisLimitSwitch == nullptr) {
        setProcessError();
        return;
    }

    m_yAxisLimitSwitch->addStateListenerCallback(
        this, &HomingModule::onLimitSwitchStateChanged);
    m_homingState = HomingState::Idle;
}

void HomingModule::onStop()
{
    abortHoming();
    if (m_yAxisLimitSwitch != nullptr) {
        m_yAxisLimitSwitch->addStateListenerCallback(nullptr, nullptr);
    }
}

bool HomingModule::home()
{
    if (!isOperating() ||
        (m_homingState != HomingState::Idle &&
         m_homingState != HomingState::Homed &&
         m_homingState != HomingState::Failed)) {
        return false;
    }

    m_yAxisRouter->stop();
    m_yAxisRouter->clearTravelLimits();
    m_limitActive.store(
        m_yAxisLimitSwitch->samplePinState() ==
            PinMonitorChannel::PinState::ACTIVE,
        std::memory_order_release);

    if (m_limitActive.load(std::memory_order_acquire)) {
        // Leave an already-operated switch before approaching it again. This
        // produces the same reference edge regardless of power-up position.
        if (!m_yAxisRouter->setPosition(0.0f)) {
            fail();
            return false;
        }
        m_homingState = HomingState::ClearingLimit;
        notify(Event::ClearingLimit);
        m_yAxisRouter->moveTo(
            -static_cast<float>(ROBOT_Y_AXIS_HOMING_DIRECTION) *
                ROBOT_Y_AXIS_HOMING_BACKOFF_MM,
            ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S);
        if (!m_yAxisRouter->isBusy()) {
            fail();
            return false;
        }
        return true;
    }

    startSeek();
    return m_homingState == HomingState::SeekingLimit;
}

void HomingModule::abortHoming()
{
    if (m_yAxisRouter != nullptr) {
        m_yAxisRouter->stop();
        m_yAxisRouter->clearTravelLimits();
    }
    m_homingState = HomingState::Idle;
}

void HomingModule::onExecute()
{
    if (m_yAxisRouter == nullptr) return;

    switch (m_homingState) {
    case HomingState::Idle:
    case HomingState::Homed:
    case HomingState::Failed:
        return;

    case HomingState::ClearingLimit:
        if (m_yAxisRouter->isBusy()) return;
        if (m_limitActive.load(std::memory_order_acquire)) {
            fail();
        } else {
            startSeek();
        }
        return;

    case HomingState::SeekingLimit:
        if (m_limitActive.load(std::memory_order_acquire)) {
            m_yAxisRouter->stop();
            const float homePosition = ROBOT_Y_AXIS_HOMING_DIRECTION < 0
                ? 0.0f
                : ROBOT_Y_AXIS_WORKSPACE_SIZE_MM;
            if (!m_yAxisRouter->setPosition(homePosition)) {
                fail();
                return;
            }
            m_homingState = HomingState::BackingOff;
            notify(Event::BackingOff);
            m_yAxisRouter->moveTo(
                homePosition -
                    static_cast<float>(ROBOT_Y_AXIS_HOMING_DIRECTION) *
                        ROBOT_Y_AXIS_HOMING_BACKOFF_MM,
                ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S);
            if (!m_yAxisRouter->isBusy()) fail();
            return;
        }
        if (!m_yAxisRouter->isBusy()) fail();
        return;

    case HomingState::BackingOff:
        if (m_yAxisRouter->isBusy()) return;
        if (m_limitActive.load(std::memory_order_acquire)) {
            fail();
            return;
        }
        m_yAxisRouter->setTravelLimits(0.0f,
                                       ROBOT_Y_AXIS_WORKSPACE_SIZE_MM);
        m_homingState = HomingState::Homed;
        notify(Event::Completed);
        return;
    }
}

bool HomingModule::isHomed() const
{
    return m_homingState == HomingState::Homed;
}

bool HomingModule::hasFailed() const
{
    return m_homingState == HomingState::Failed;
}

void HomingModule::startSeek()
{
    if (!m_yAxisRouter->setPosition(0.0f)) {
        fail();
        return;
    }

    m_homingState = HomingState::SeekingLimit;
    notify(Event::SeekingLimit);
    const float searchDistance =
        ROBOT_Y_AXIS_WORKSPACE_SIZE_MM +
        ROBOT_Y_AXIS_HOMING_SEARCH_MARGIN_MM;
    m_yAxisRouter->moveTo(
        static_cast<float>(ROBOT_Y_AXIS_HOMING_DIRECTION) * searchDistance,
        ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S);
    if (!m_yAxisRouter->isBusy()) fail();
}

void HomingModule::fail()
{
    if (m_yAxisRouter != nullptr) {
        m_yAxisRouter->stop();
        m_yAxisRouter->clearTravelLimits();
    }
    m_homingState = HomingState::Failed;
    notify(Event::Failed);
}

void HomingModule::notify(Event event)
{
    if (m_eventCallback != nullptr) {
        m_eventCallback(m_eventCallbackContext, event);
    }
}

void HomingModule::onLimitSwitchStateChanged(
    void *context,
    PinMonitorChannel::PinState state)
{
    HomingModule *module = static_cast<HomingModule *>(context);
    if (module == nullptr) return;
    module->m_limitActive.store(
        state == PinMonitorChannel::PinState::ACTIVE,
        std::memory_order_release);
}
