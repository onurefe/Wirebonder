#include "timer_expire_service.hpp"

static TimerExpireService *g_instance = nullptr;

// =======================================================================
// Timer
// =======================================================================
Timer::Timer()
    : m_active(false)
    , m_startTick(0)
    , m_oneShot(false)
    , m_periodInTicks(0)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
{
}

void Timer::setExpirationListenerCallback(void *context, Callback cb)
{
    m_callbackContext = context;
    m_callback        = cb;
}

void Timer::start(bool oneShot, float periodInSeconds)
{
    uint32_t periodTicks = static_cast<uint32_t>(TIMER_EXPIRE_SERVICE_TICK_FREQUENCY * periodInSeconds);
    uint32_t startTick   = TimerExpireService::getTicks();

    InterruptLock lock;
    m_oneShot       = oneShot;
    m_periodInTicks = (periodTicks == 0U) ? 1U : periodTicks;
    m_startTick     = startTick;
    m_active        = true;
}

void Timer::stop()
{
    InterruptLock lock;
    m_active = false;
}

bool Timer::isActive() const
{
    return m_active;
}

void Timer::tick(uint32_t globalTick)
{
    if (!m_active) {
        return;
    }

    if ((globalTick - m_startTick) >= m_periodInTicks) {
        if (m_oneShot) {
            m_active = false;
        } else {
            m_startTick += m_periodInTicks;
        }

        if (m_callback != nullptr) {
            m_callback(m_callbackContext, this);
        }
    }
}

// =======================================================================
// TimerExpireService
// =======================================================================
TimerExpireService::TimerExpireService()
    : m_numCriticalTimers(0)
    , m_numNormalTimers(0)
    , m_globalTick(0)
    , m_lastCallTick(0)
{
    for (uint8_t i = 0; i < TIMER_EXPIRE_SERVICE_MAX_HANDLES; i++) {
        m_criticalTimers[i] = nullptr;
        m_normalTimers[i]   = nullptr;
    }

    g_instance = this;
}

bool TimerExpireService::addTimer(Timer *timer, bool timeCritical)
{
    if (timer == nullptr) {
        return false;
    }

    if (timeCritical) {
        if (m_numCriticalTimers >= TIMER_EXPIRE_SERVICE_MAX_HANDLES) return false;
        m_criticalTimers[m_numCriticalTimers++] = timer;
    } else {
        if (m_numNormalTimers >= TIMER_EXPIRE_SERVICE_MAX_HANDLES) return false;
        m_normalTimers[m_numNormalTimers++] = timer;
    }

    return true;
}

void TimerExpireService::onStart()
{
}

void TimerExpireService::onExecute()
{
    uint32_t currentTick = m_globalTick;

    if (m_lastCallTick != currentTick) {
        for (uint8_t i = 0; i < m_numNormalTimers; i++) {
            m_normalTimers[i]->tick(currentTick);
        }

        m_lastCallTick = currentTick;
    }
}

void TimerExpireService::onStop()
{
}

void TimerExpireService::tickFromISR()
{
    if (g_instance == nullptr || !g_instance->isOperating()) {
        return;
    }

    g_instance->m_globalTick++;

    for (uint8_t i = 0; i < g_instance->m_numCriticalTimers; i++) {
        g_instance->m_criticalTimers[i]->tick(g_instance->m_globalTick);
    }
}

uint32_t TimerExpireService::getTicks()
{
    InterruptLock lock;
    return g_instance ? g_instance->m_globalTick : 0;
}

float TimerExpireService::getTickFrequency()
{
    return static_cast<float>(TIMER_EXPIRE_SERVICE_TICK_FREQUENCY);
}

extern "C" void TimerExpire_TickFromISR(void)
{
    TimerExpireService::tickFromISR();
}
