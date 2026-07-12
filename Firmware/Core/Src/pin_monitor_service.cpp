#include "pin_monitor_service.hpp"

// =======================================================================
// PinMonitorChannel
// =======================================================================
PinMonitorChannel::PinMonitorChannel(FastIO *pin, Level activeLevel)
    : m_pin(pin)
    , m_activeLevel(activeLevel)
    , m_lastLevel(Level::UNDETERMINED)
    , m_active(false)
    , m_blindTicks(0)
    , m_locked(false)
    , m_lockStartTick(0)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
{
}

void PinMonitorChannel::setBlindTicks(uint32_t blindTicks)
{
    m_blindTicks = blindTicks;
}

void PinMonitorChannel::addStateListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void PinMonitorChannel::start()
{
    m_lastLevel = Level::UNDETERMINED;
    m_locked    = false;
    m_active    = true;
}

void PinMonitorChannel::stop()
{
    m_active = false;
}

PinMonitorChannel::Level PinMonitorChannel::getLevel() const
{
    return m_lastLevel;
}

PinMonitorChannel::PinState PinMonitorChannel::getPinState() const
{
    return levelToState(m_lastLevel);
}

PinMonitorChannel::PinState PinMonitorChannel::levelToState(Level level) const
{
    return (level == m_activeLevel) ? PinState::ACTIVE : PinState::INACTIVE;
}

void PinMonitorChannel::update(uint32_t currentTick)
{
    if (!m_active || m_locked) return;

    Level    newLevel = m_pin->read() ? Level::HIGH : Level::LOW;
    PinState oldState = levelToState(m_lastLevel);
    PinState newState = levelToState(newLevel);
    m_lastLevel       = newLevel;

    if (newState != oldState && m_callback != nullptr) {
        if (m_blindTicks > 0) {
            m_locked       = true;
            m_lockStartTick = currentTick;
        }
        m_callback(m_callbackContext, newState);
    }
}

void PinMonitorChannel::clearLockIfExpired(uint32_t currentTick)
{
    if (m_locked && (currentTick - m_lockStartTick) >= m_blindTicks) {
        m_locked = false;
    }
}

// =======================================================================
// PinMonitorService
// =======================================================================
PinMonitorService::PinMonitorService(Timer *criticalTimer, Timer *normalTimer)
    : m_criticalTimer(criticalTimer)
    , m_normalTimer(normalTimer)
    , m_numCriticalChannels(0)
    , m_numNormalChannels(0)
    , m_state(ServiceState::READY)
{
    for (uint8_t i = 0; i < PIN_MONITOR_SERVICE_MAX_PINS; i++) {
        m_criticalChannels[i] = nullptr;
        m_normalChannels[i]   = nullptr;
    }

    m_criticalTimer->setExpirationListenerCallback(this, onCriticalTimerExpired);
    m_normalTimer->setExpirationListenerCallback(this, onNormalTimerExpired);
}

bool PinMonitorService::addChannel(PinMonitorChannel *channel, bool timeCritical, uint32_t blindMs)
{
    if (channel == nullptr) return false;

    channel->setBlindTicks(blindMs * TIMER_EXPIRE_SERVICE_TICK_FREQUENCY / 1000U);

    if (timeCritical) {
        if (m_numCriticalChannels >= PIN_MONITOR_SERVICE_MAX_PINS) return false;
        m_criticalChannels[m_numCriticalChannels++] = channel;
    } else {
        if (m_numNormalChannels >= PIN_MONITOR_SERVICE_MAX_PINS) return false;
        m_normalChannels[m_numNormalChannels++] = channel;
    }
    return true;
}

void PinMonitorService::startService()
{
    if (m_state != ServiceState::READY) return;

    for (uint8_t i = 0; i < m_numCriticalChannels; i++) m_criticalChannels[i]->start();
    for (uint8_t i = 0; i < m_numNormalChannels;   i++) m_normalChannels[i]->start();

    m_criticalTimer->start(false, 1.0f / PIN_MONITOR_CRITICAL_SAMPLING_FREQUENCY);
    m_normalTimer->start  (false, 1.0f / PIN_MONITOR_NORMAL_SAMPLING_FREQUENCY);

    m_state = ServiceState::OPERATING;
}

void PinMonitorService::stopService()
{
    if (m_state != ServiceState::OPERATING) return;

    m_criticalTimer->stop();
    m_normalTimer->stop();

    for (uint8_t i = 0; i < m_numCriticalChannels; i++) m_criticalChannels[i]->stop();
    for (uint8_t i = 0; i < m_numNormalChannels;   i++) m_normalChannels[i]->stop();

    m_state = ServiceState::READY;
}

void PinMonitorService::executeService()
{
    if (m_state != ServiceState::OPERATING) return;

    uint32_t currentTick = TimerExpireService::getTicks();
    for (uint8_t i = 0; i < m_numCriticalChannels; i++) {
        m_criticalChannels[i]->clearLockIfExpired(currentTick);
    }
}

void PinMonitorService::onCriticalTimerExpired(void *context, Timer *timer)
{
    (void)timer;
    PinMonitorService *self = static_cast<PinMonitorService *>(context);
    uint32_t tick = TimerExpireService::getTicks();

    for (uint8_t i = 0; i < self->m_numCriticalChannels; i++) {
        self->m_criticalChannels[i]->update(tick);
    }
}

void PinMonitorService::onNormalTimerExpired(void *context, Timer *timer)
{
    (void)timer;
    PinMonitorService *self = static_cast<PinMonitorService *>(context);

    for (uint8_t i = 0; i < self->m_numNormalChannels; i++) {
        self->m_normalChannels[i]->update(0);
    }
}
