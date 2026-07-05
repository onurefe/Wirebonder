#ifndef SOLENOID_SERVICE_HPP
#define SOLENOID_SERVICE_HPP

#include <cstdint>
#include "generic.h"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"

// -----------------------------------------------------------------------
// Class: SolenoidChannel
// -----------------------------------------------------------------------
class SolenoidChannel {
public:
    enum class State        { UNKNOWN, OPENING, CLOSING, OPENED, CLOSED };
    enum class DefaultState { OPENED, CLOSED };

    using Callback = void (*)(void *context, State state);

    SolenoidChannel(FastIO *openPin, FastIO *closePin, DefaultState defaultState, Timer *timer);

    void addStateListenerCallback(void *context, Callback cb);

    void open();
    void close();

    State        getState()        const;
    DefaultState getDefaultState() const;

    void start();
    void stop();

private:
    static void onTransitionTimer(void *context, Timer *timer);
    void clearPins();

    FastIO      *m_openPin;
    FastIO      *m_closePin;
    DefaultState m_defaultState;
    State        m_state;
    Timer       *m_timer;
    Callback     m_callback;
    void        *m_callbackContext;
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
    void executeService() {}

private:
    SolenoidChannel *m_channels[SOLENOID_SERVICE_MAX_INSTANCES];
    uint8_t          m_numChannels;
    ServiceState     m_state;
};

#endif /* SOLENOID_SERVICE_HPP */
