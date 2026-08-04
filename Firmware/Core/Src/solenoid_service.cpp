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
{}

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
    m_timer->setExpirationListenerCallback(this, onTransitionTimer);
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
    m_timer->setExpirationListenerCallback(nullptr, nullptr);
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
// PwmSolenoidChannel
// =======================================================================
PwmSolenoidChannel::PwmSolenoidChannel(DirectPwmChannel *pwmChannel,
                                       Timer *timer,
                                       float energizeTime,
                                       float deenergizeTime,
                                       float onDuty,
                                       float offDuty)
    : m_pwmChannel(pwmChannel)
    , m_state(State::DEENERGIZED)
    , m_targetState(State::DEENERGIZED)
    , m_timer(timer)
    , m_energizeTime(energizeTime)
    , m_deenergizeTime(deenergizeTime)
    , m_onDuty(onDuty)
    , m_offDuty(offDuty)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
{}

void PwmSolenoidChannel::addStateListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void PwmSolenoidChannel::setOnDuty(float duty)
{
    m_onDuty = duty;

    // Already holding: push the new duty out now. poll() only touches the PWM
    // output on a state change, so an energized coil would otherwise keep the
    // stale hold duty until the next energize cycle.
    if (m_state == State::ENERGIZED || m_state == State::ENERGIZING) {
        m_pwmChannel->setDuty(m_onDuty);
    }
}

void PwmSolenoidChannel::energize()
{
    m_targetState = State::ENERGIZED;
}

void PwmSolenoidChannel::deenergize()
{
    m_targetState = State::DEENERGIZED;
}

void PwmSolenoidChannel::poll()
{
    if (isTransitioning()) {
        return;
    }

    if (m_state != m_targetState) {
        if (m_targetState == State::ENERGIZED) {
            m_pwmChannel->setDuty(m_onDuty);
            m_state = State::ENERGIZING;
            m_timer->start(true, m_energizeTime);

            return;
        }

        if (m_targetState == State::DEENERGIZED) {
            m_pwmChannel->setDuty(m_offDuty);
            m_state = State::DEENERGIZING;
            m_timer->start(true, m_deenergizeTime);

            return;
        }
    }
}

bool PwmSolenoidChannel::isTransitioning() const
{
    return m_state == State::ENERGIZING || m_state == State::DEENERGIZING;
}

PwmSolenoidChannel::State PwmSolenoidChannel::getState() const
{
    return m_state;
}

void PwmSolenoidChannel::start()
{
    m_timer->setExpirationListenerCallback(this, onTransitionTimer);
    m_pwmChannel->start(m_offDuty);
    m_state = State::DEENERGIZED;
    m_targetState = State::DEENERGIZED;
}

void PwmSolenoidChannel::stop()
{
    m_pwmChannel->stop();
    m_timer->stop();
    m_state = State::DEENERGIZED;
    m_targetState = State::DEENERGIZED;
    m_timer->setExpirationListenerCallback(nullptr, nullptr);
}

void PwmSolenoidChannel::onTransitionTimer(void *context, Timer *timer)
{
    (void)timer;
    PwmSolenoidChannel *self = static_cast<PwmSolenoidChannel *>(context);

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

void SolenoidService::onStart()
{
    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->start();
    }

    HAL_Delay(SOLENOID_SERVICE_INIT_DELAY_MS);

}

void SolenoidService::onExecute()
{
    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->poll();
    }
}

void SolenoidService::onStop()
{
    for (uint8_t i = 0; i < m_numChannels; i++) {
        m_channels[i]->stop();
    }

}
