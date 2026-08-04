#ifndef DEBUG_ENVIRONMENT_SOLENOIDS_HPP
#define DEBUG_ENVIRONMENT_SOLENOIDS_HPP

#include <cstdint>

#include "configuration.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"
#include "solenoid_service.hpp"

// Sandbox for the solenoid drivers: clamp + auxiliary channels behind the
// SolenoidService, energized/deenergized individually or together.
class SolenoidsDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_SOLENOIDS;

    enum Command : uint16_t {
        CMD_SET = 1
    };

    SolenoidsDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_SET;
    }

    void handleCommand(uint16_t localCommand) override;
    void onPoll() override;

    bool isBusy() const override
    {
        return m_transitionActive;
    }

    void abort() override;

private:
    static constexpr uint8_t kChannelCount = 3U;

    void set();
    bool selectedChannelsReachedTarget() const;
    bool channelIsSelected(uint8_t index) const;

    // Hardware sandbox — exclusively owned by this environment.
    // DRIVES_SOL3H (PB15, clamp high side) is TIM12_CH2 PWM, not a plain
    // GPIO -- see m_clampPwmChannel.
    static FastIO m_clampLowPin;
    static FastIO m_sol1LowPin;
    static FastIO m_sol1HighPin;
    static FastIO m_sol2LowPin;
    static FastIO m_sol2HighPin;

    static TimerExpireService m_timerExpireService;
    static Timer m_clampSolenoidTimer;
    static Timer m_sol1SolenoidTimer;
    static Timer m_sol2SolenoidTimer;

    static DirectPwmChannel m_clampPwmChannel;
    static PwmSolenoidChannel m_clampSolenoidChannel;
    static DirectSolenoidChannel m_sol1SolenoidChannel;
    static DirectSolenoidChannel m_sol2SolenoidChannel;
    static SolenoidService m_solenoidService;

    static SolenoidChannel *const m_channels[kChannelCount];

    uint8_t m_selectedChannel = 0U;
    SolenoidChannel::State m_targetState =
        SolenoidChannel::State::DEENERGIZED;
    bool m_selectAll = false;
    bool m_transitionActive = false;
};

#endif /* DEBUG_ENVIRONMENT_SOLENOIDS_HPP */
