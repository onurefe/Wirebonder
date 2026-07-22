#ifndef PIN_MONITOR_SERVICE_HPP
#define PIN_MONITOR_SERVICE_HPP

#include "configuration.h"
#include "generic.h"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"
#include "process.hpp"

// -----------------------------------------------------------------------
// Class: PinMonitorChannel
//
// Reports logical pin states rather than electrical levels: the sensor
// front-ends (optocouplers) are active-low, so the constructor takes the
// electrical level that means ACTIVE. UNDETERMINED (not yet sampled) maps
// to INACTIVE.
// -----------------------------------------------------------------------
class PinMonitorChannel {
public:
    enum class Level    { LOW, HIGH, UNDETERMINED };
    enum class PinState { INACTIVE, ACTIVE };

    using Callback = void (*)(void *context, PinState state);

    PinMonitorChannel(FastIO *pin, Level activeLevel);

    void addStateListenerCallback(void *context, Callback cb);
    void  start();
    void  stop();
    Level getLevel() const;
    PinState getPinState() const;
    // Read the pin immediately and translate it to its logical sensor state.
    PinState samplePinState() const;

    // Poll the pin, fire callback on a pin-state change. The lock flag
    // suppresses further changes for blindTicks after the callback fires.
    void update(uint32_t currentTick);

    // Release the lock once its duration has elapsed. Called from the main
    // loop (PinMonitorService::onExecute), so unlock jitter is acceptable.
    void clearLockIfExpired(uint32_t currentTick);

    // Called by PinMonitorService::addChannel.
    void setBlindTicks(uint32_t blindTicks);

private:
    PinState levelToState(Level level) const;

    FastIO    *m_pin;
    Level      m_activeLevel;
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
class PinMonitorService : public Process {
public:
    PinMonitorService(Timer *criticalTimer, Timer *normalTimer);

    // timeCritical selects the sampling source only: true → ISR-based 1 ms
    // sampling, false → main-loop 20 ms polling. Blind-region locking works
    // identically on both.
    bool addChannel(PinMonitorChannel *channel,
                    bool     timeCritical = false,
                    uint32_t blindMs      = 0);

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    static void onCriticalTimerExpired(void *context, Timer *timer);
    static void onNormalTimerExpired  (void *context, Timer *timer);

    Timer             *m_criticalTimer;
    Timer             *m_normalTimer;

    PinMonitorChannel *m_criticalChannels[PIN_MONITOR_SERVICE_MAX_PINS];
    uint8_t            m_numCriticalChannels;

    PinMonitorChannel *m_normalChannels[PIN_MONITOR_SERVICE_MAX_PINS];
    uint8_t            m_numNormalChannels;
};

#endif /* PIN_MONITOR_SERVICE_HPP */
