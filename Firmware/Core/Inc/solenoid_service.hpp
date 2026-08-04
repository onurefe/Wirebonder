#ifndef SOLENOID_SERVICE_HPP
#define SOLENOID_SERVICE_HPP

#include <cstdint>
#include "generic.h"
#include "fast_io.hpp"
#include "pwm_service.hpp"
#include "timer_expire_service.hpp"
#include "process.hpp"

// -----------------------------------------------------------------------
// Class: SolenoidChannel
//
// Interface shared by the solenoid drive flavours (DirectSolenoidChannel,
// PwmSolenoidChannel) so SolenoidService -- and any other caller that just
// needs to command a non-latching solenoid without caring how it's
// physically driven -- can manage them uniformly.
// -----------------------------------------------------------------------
class SolenoidChannel {
public:
    enum class State        { ENERGIZING, DEENERGIZING, ENERGIZED, DEENERGIZED };

    using Callback = void (*)(void *context, State state);

    virtual ~SolenoidChannel() = default;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void poll() = 0;

    virtual void addStateListenerCallback(void *context, Callback cb) = 0;
    virtual bool isTransitioning() const = 0;
    virtual void energize() = 0;
    virtual void deenergize() = 0;
    virtual State getState() const = 0;
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
    // energizeTime / deenergizeTime: mechanical transition times in seconds.
    DirectSolenoidChannel(FastIO *energizePin,
                          FastIO *closePin,
                          Timer *timer,
                          float energizeTime,
                          float deenergizeTime);

    void addStateListenerCallback(void *context, Callback cb) override;

    bool isTransitioning() const override;

    void poll() override;
    void energize() override;
    void deenergize() override;

    State        getState()        const override;

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
// Class: PwmSolenoidChannel
//
// Same non-latching behavior and state machine as DirectSolenoidChannel,
// but driven through a PWM channel instead of a digital pin: energizing
// sets the channel to onDuty, deenergizing sets it to offDuty. Lets the
// energized hold run below 100% duty (e.g. to cut continuous-hold heating)
// instead of only ever being fully on or fully off.
// -----------------------------------------------------------------------
class PwmSolenoidChannel : public SolenoidChannel {
public:
    // energizeTime / deenergizeTime: mechanical transition times in seconds.
    // onDuty / offDuty: normalized PWM duty (0.0-1.0) driven while energized
    // / deenergized.
    PwmSolenoidChannel(DirectPwmChannel *pwmChannel,
                       Timer *timer,
                       float energizeTime,
                       float deenergizeTime,
                       float onDuty = 1.0f,
                       float offDuty = 0.0f);

    void addStateListenerCallback(void *context, Callback cb) override;

    // Updates the energized-hold duty. Applied immediately when the coil is
    // already energized (or energizing), otherwise at the next energize().
    void setOnDuty(float duty);

    bool isTransitioning() const override;

    void poll() override;
    void energize() override;
    void deenergize() override;

    State        getState()        const override;

    void start() override;
    void stop() override;

private:
    static void onTransitionTimer(void *context, Timer *timer);

    DirectPwmChannel *m_pwmChannel;
    State           m_state;
    State           m_targetState;
    Timer           *m_timer;
    float           m_energizeTime;
    float           m_deenergizeTime;
    float           m_onDuty;
    float           m_offDuty;
    Callback        m_callback;
    void            *m_callbackContext;
};

// -----------------------------------------------------------------------
// Class: SolenoidService
// -----------------------------------------------------------------------
class SolenoidService : public Process {
public:
    SolenoidService();

    bool addChannel(SolenoidChannel *channel);

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    SolenoidChannel *m_channels[SOLENOID_SERVICE_MAX_INSTANCES];
    uint8_t          m_numChannels;
};

#endif /* SOLENOID_SERVICE_HPP */
