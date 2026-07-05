#include "solenoid_service.hpp"
#include "configuration.h"

// =======================================================================
// SolenoidChannel
// =======================================================================
SolenoidChannel::SolenoidChannel(FastIO *openPin, FastIO *closePin, DefaultState defaultState, Timer *timer)
    : m_openPin(openPin)
    , m_closePin(closePin)
    , m_defaultState(defaultState)
    , m_state(State::UNKNOWN)
    , m_timer(timer)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
{
    m_timer->setExpirationListenerCallback(this, onTransitionTimer);
}

void SolenoidChannel::addStateListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void SolenoidChannel::open()
{
    if (m_state == State::OPENING || m_state == State::CLOSING) return;
    if (m_state == State::OPENED) return;

    clearPins();
    m_openPin->set();
    m_state = State::OPENING;
    m_timer->start(true, SOLENOID_SERVICE_TRANSITION_TIME);
}

void SolenoidChannel::close()
{
    if (m_state == State::OPENING || m_state == State::CLOSING) return;
    if (m_state == State::CLOSED) return;

    clearPins();
    m_closePin->set();
    m_state = State::CLOSING;
    m_timer->start(true, SOLENOID_SERVICE_TRANSITION_TIME);
}

SolenoidChannel::State SolenoidChannel::getState() const
{
    return m_state;
}

SolenoidChannel::DefaultState SolenoidChannel::getDefaultState() const
{
    return m_defaultState;
}

void SolenoidChannel::start()
{
    clearPins();
    m_state = State::UNKNOWN;
}

void SolenoidChannel::stop()
{
    m_timer->stop();
    clearPins();
}

void SolenoidChannel::onTransitionTimer(void *context, Timer *timer)
{
    (void)timer;
    SolenoidChannel *self = static_cast<SolenoidChannel *>(context);

    self->clearPins();
    self->m_state = (self->m_state == State::OPENING) ? State::OPENED : State::CLOSED;

    if (self->m_callback != nullptr) {
        self->m_callback(self->m_callbackContext, self->m_state);
    }
}

void SolenoidChannel::clearPins()
{
    m_openPin->clear();
    m_closePin->clear();
}

// =======================================================================
// SolenoidService
// =======================================================================
SolenoidService::SolenoidService()
    : m_numChannels(0)
    , m_state(ServiceState::READY)
{
    for (uint8_t i = 0; i < SOLENOID_SERVICE_MAX_INSTANCES; i++) {
        m_channels[i] = nullptr;
    }
}

bool SolenoidService::addChannel(SolenoidChannel *channel)
{
    if (channel == nullptr || m_numChannels >= SOLENOID_SERVICE_MAX_INSTANCES) {
        return false;
    }

    m_channels[m_numChannels++] = channel;
    return true;
}

void SolenoidService::startService()
{
    if (m_state != ServiceState::READY) return;

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->start();
    }

    HAL_Delay(SOLENOID_SERVICE_INIT_DELAY_MS);

    for (uint8_t i = 0; i < m_numChannels; i++) {
        SolenoidChannel *ch = m_channels[i];

        if (ch->getDefaultState() == SolenoidChannel::DefaultState::OPENED) {
            ch->open();
        } else {
            ch->close();
        }
    }

    m_state = ServiceState::OPERATING;
}

void SolenoidService::stopService()
{
    if (m_state != ServiceState::OPERATING) return;

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->stop();
    }

    m_state = ServiceState::READY;
}
