#include "solenoid_service.hpp"
#include "configuration.h"

// =======================================================================
// DirectSolenoidChannel
// =======================================================================
DirectSolenoidChannel::DirectSolenoidChannel(FastIO *energizePin,
                                             FastIO *closePin,
                                             Timer *timer,
                                             float energizeTime,
                                             float deenergizeTime)
    : m_energizePin(energizePin)
    , m_closePin(closePin)
    , m_state(State::DEENERGIZED)
    , m_targetState(State::DEENERGIZED)
    , m_timer(timer)
    , m_energizeTime(energizeTime)
    , m_deenergizeTime(deenergizeTime)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
{
    m_timer->setExpirationListenerCallback(this, onTransitionTimer);
}

void DirectSolenoidChannel::addStateListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void DirectSolenoidChannel::energize()
{
    m_targetState = State::ENERGIZED;
}

void DirectSolenoidChannel::deenergize()
{
    m_targetState = State::DEENERGIZED;
}

void DirectSolenoidChannel::poll()
{
    if (isTransitioning()) {
        return;
    }

    if (m_state != m_targetState) {
        if (m_targetState == State::ENERGIZED) {
            m_closePin->clear();
            m_energizePin->set();
            m_state = State::ENERGIZING;
            m_timer->start(true, m_energizeTime);

            return;
        }

        if (m_targetState == State::DEENERGIZED) {
            m_closePin->clear();
            m_energizePin->clear();
            m_state = State::DEENERGIZING;
            m_timer->start(true, m_deenergizeTime);

            return;
        }
    }
}

bool DirectSolenoidChannel::isTransitioning() const
{
    return m_state == State::ENERGIZING || m_state == State::DEENERGIZING;
}

DirectSolenoidChannel::State DirectSolenoidChannel::getState() const
{
    return m_state;
}

void DirectSolenoidChannel::start()
{
    m_closePin->clear();
    m_energizePin->clear();
    m_state = State::DEENERGIZED;
    m_targetState = State::DEENERGIZED;
}

void DirectSolenoidChannel::stop()
{
    m_closePin->clear();
    m_energizePin->clear();
    m_timer->stop();
    m_state = State::DEENERGIZED;
    m_targetState = State::DEENERGIZED;
}

void DirectSolenoidChannel::onTransitionTimer(void *context, Timer *timer)
{
    (void)timer;
    DirectSolenoidChannel *self = static_cast<DirectSolenoidChannel *>(context);

    self->m_state = (self->m_state == State::ENERGIZING) ? State::ENERGIZED
                                                         : State::DEENERGIZED;

    if (self->m_callback != nullptr) {
        self->m_callback(self->m_callbackContext, self->m_state);
    }
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

    m_state = ServiceState::OPERATING;
}

void SolenoidService::executeService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->poll();
    }
}

void SolenoidService::stopService()
{
    if (m_state != ServiceState::OPERATING) return;

    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->stop();
    }

    m_state = ServiceState::READY;
}
