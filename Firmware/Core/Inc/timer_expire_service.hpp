#ifndef TIMER_EXPIRE_SERVICE_HPP
#define TIMER_EXPIRE_SERVICE_HPP

#include "callback_list.hpp"
#include "generic.h"
#include "configuration.h"
#include "process.hpp"

// -----------------------------------------------------------------------
// Class: Timer
// -----------------------------------------------------------------------
class Timer {
public:
    using Callback = void (*)(void *context, Timer *timer);

    Timer();

    bool addExpirationListenerCallback(void *context, Callback cb);
    bool removeExpirationListenerCallback(void *context, Callback cb);
    void start(bool oneShot, float periodInSeconds);
    void stop();
    bool isActive() const;

    void tick(uint32_t globalTick);

private:
    volatile bool     m_active;
    volatile uint32_t m_startTick;
    bool              m_oneShot;
    uint32_t          m_periodInTicks;
    ListenerList<Timer *> m_callbacks;
};

// -----------------------------------------------------------------------
// Class: TimerExpireService
// -----------------------------------------------------------------------
class TimerExpireService : public Process {
public:
    TimerExpireService();

    bool addTimer(Timer *timer, bool timeCritical = false);

    static void     tickFromISR();
    static uint32_t getTicks();
    static float    getTickFrequency();

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    Timer            *m_criticalTimers[TIMER_EXPIRE_SERVICE_MAX_HANDLES];
    uint8_t           m_numCriticalTimers;

    Timer            *m_normalTimers[TIMER_EXPIRE_SERVICE_MAX_HANDLES];
    uint8_t           m_numNormalTimers;

    volatile uint32_t m_globalTick;
    uint32_t          m_lastCallTick;
};

#endif /* TIMER_EXPIRE_SERVICE_HPP */
