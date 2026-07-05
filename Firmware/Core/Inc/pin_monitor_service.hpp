#ifndef PIN_MONITOR_SERVICE_HPP
#define PIN_MONITOR_SERVICE_HPP

#include "configuration.h"
#include "generic.h"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"

// -----------------------------------------------------------------------
// Class: PinMonitorChannel
// -----------------------------------------------------------------------
class PinMonitorChannel {
public:
    enum class Transition { NONE, LOW_TO_HIGH, HIGH_TO_LOW };
    enum class Level      { LOW, HIGH, UNDETERMINED };

    using Callback = void (*)(void *context, Transition transition);

    explicit PinMonitorChannel(FastIO *pin);

    void addTransitionListenerCallback(void *context, Callback cb);
    void  start();
    void  stop();
    Level getLevel() const;

    // Poll the pin, fire callback on transition. For critical channels the
    // lock flag suppresses transitions for blindTicks after the callback fires.
    void update(uint32_t currentTick);

    // Release the lock once its duration has elapsed. Called from the main
    // loop (PinMonitorService::executeService), so unlock jitter is acceptable.
    void clearLockIfExpired(uint32_t currentTick);

    // Called by PinMonitorService::addChannel.
    void setBlindTicks(uint32_t blindTicks);

private:
    static Transition detectTransition(Level oldLevel, Level newLevel);

    FastIO    *m_pin;
    Level      m_lastLevel;
    bool       m_active;

    uint32_t   m_blindTicks;
    bool       m_locked;
    uint32_t   m_lockStartTick;

    Callback   m_callback;
    void      *m_callbackContext;
};

// -----------------------------------------------------------------------
// Class: PinMonitorService
// -----------------------------------------------------------------------
class PinMonitorService {
public:
    PinMonitorService(Timer *criticalTimer, Timer *normalTimer);

    // timeCritical=true  → ISR-based 1 ms sampling + blind-region lock
    // timeCritical=false → main-loop 20 ms polling, no lock
    bool addChannel(PinMonitorChannel *channel,
                    bool     timeCritical = false,
                    uint32_t blindMs      = 0);

    void initService() {}
    void startService();
    void stopService();

    // Clears expired blind-region locks for critical channels. Must be called
    // from the main loop (e.g. Robot::execute).
    void executeService();

private:
    static void onCriticalTimerExpired(void *context, Timer *timer);
    static void onNormalTimerExpired  (void *context, Timer *timer);

    Timer             *m_criticalTimer;
    Timer             *m_normalTimer;

    PinMonitorChannel *m_criticalChannels[PIN_MONITOR_SERVICE_MAX_PINS];
    uint8_t            m_numCriticalChannels;

    PinMonitorChannel *m_normalChannels[PIN_MONITOR_SERVICE_MAX_PINS];
    uint8_t            m_numNormalChannels;

    ServiceState       m_state;
};

#endif /* PIN_MONITOR_SERVICE_HPP */
