#ifndef SOLENOID_SERVICE_HPP
#define SOLENOID_SERVICE_HPP

#include <cstdint>
#include "generic.h"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"

// -----------------------------------------------------------------------
// Class: SolenoidChannel
//
// Interface shared by the solenoid drive flavours so SolenoidService can
// manage them uniformly.
// -----------------------------------------------------------------------
class SolenoidChannel {
public:
    virtual ~SolenoidChannel() = default;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void poll() = 0;
};

// -----------------------------------------------------------------------
// Class: DirectSolenoidChannel
//
// Non-latching solenoid: the plunger is held only while the energize coil
// is powered, so the energize pin stays set for as long as the solenoid
// must stay engaged. The close pin is held clear at all times. The state
// callback fires once the mechanical energize/deenergize time has passed.
// -----------------------------------------------------------------------
class DirectSolenoidChannel : public SolenoidChannel {
public:
    enum class State        { ENERGIZING, DEENERGIZING, ENERGIZED, DEENERGIZED };

    using Callback = void (*)(void *context, State state);

    // energizeTime / deenergizeTime: mechanical transition times in seconds.
    DirectSolenoidChannel(FastIO *energizePin,
                          FastIO *closePin,
                          Timer *timer,
                          float energizeTime,
                          float deenergizeTime);

    void addStateListenerCallback(void *context, Callback cb);

    bool isTransitioning() const;

    void poll();
    void energize();
    void deenergize();

    State        getState()        const;

    void start() override;
    void stop() override;

private:
    static void onTransitionTimer(void *context, Timer *timer);

    FastIO          *m_energizePin;
    FastIO          *m_closePin;
    State           m_state;
    State           m_targetState;
    Timer           *m_timer;
    float           m_energizeTime;
    float           m_deenergizeTime;
    Callback        m_callback;
    void            *m_callbackContext;
};

// -----------------------------------------------------------------------
// Class: SolenoidService
// -----------------------------------------------------------------------
class SolenoidService {
public:
    SolenoidService();

    bool addChannel(SolenoidChannel *channel);

    void initService()    {}
    void startService();
    void stopService();
    void executeService();

private:
    SolenoidChannel *m_channels[SOLENOID_SERVICE_MAX_INSTANCES];
    uint8_t          m_numChannels;
    ServiceState     m_state;
};

#endif /* SOLENOID_SERVICE_HPP */
