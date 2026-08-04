#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_SOLENOIDS

#include "DebugEnvironment/debug_environment_solenoids.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim12;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the solenoids,
// but owned here outright.
// -----------------------------------------------------------------------------

// Sol1 and Sol2 drivers are unavailable. The clamp coil is connected to the
// working Sol3 driver, so keep the logical clamp names mapped to Sol3.
// DRIVES_SOL3H (PB15) is TIM12_CH2 PWM instead of a plain GPIO; see
// m_clampPwmChannel below.
FastIO SolenoidsDebugEnvironment::m_clampLowPin (DRIVES_SOL3L_GPIO_Port, DRIVES_SOL3L_Pin, TRUE);
FastIO SolenoidsDebugEnvironment::m_sol1LowPin  (DRIVES_SOL1L_GPIO_Port, DRIVES_SOL1L_Pin, TRUE);
FastIO SolenoidsDebugEnvironment::m_sol1HighPin (DRIVES_SOL1H_GPIO_Port, DRIVES_SOL1H_Pin, FALSE);
FastIO SolenoidsDebugEnvironment::m_sol2LowPin  (DRIVES_SOL2L_GPIO_Port, DRIVES_SOL2L_Pin, TRUE);
FastIO SolenoidsDebugEnvironment::m_sol2HighPin (DRIVES_SOL2H_GPIO_Port, DRIVES_SOL2H_Pin, FALSE);

TimerExpireService SolenoidsDebugEnvironment::m_timerExpireService;
Timer SolenoidsDebugEnvironment::m_clampSolenoidTimer;
Timer SolenoidsDebugEnvironment::m_sol1SolenoidTimer;
Timer SolenoidsDebugEnvironment::m_sol2SolenoidTimer;

DirectPwmChannel SolenoidsDebugEnvironment::m_clampPwmChannel(&htim12, TIM_CHANNEL_2);
PwmSolenoidChannel SolenoidsDebugEnvironment::m_clampSolenoidChannel(
    &SolenoidsDebugEnvironment::m_clampPwmChannel,
    &SolenoidsDebugEnvironment::m_clampSolenoidTimer,
    CLAMP_SOLENOID_ENERGIZE_TIME,
    CLAMP_SOLENOID_DEENERGIZE_TIME,
    CLAMP_SOLENOID_VOLTAGE_DEFAULT / CLAMP_SOLENOID_SUPPLY_VOLTAGE);

DirectSolenoidChannel SolenoidsDebugEnvironment::m_sol1SolenoidChannel(
    &SolenoidsDebugEnvironment::m_sol1HighPin,
    &SolenoidsDebugEnvironment::m_sol1LowPin,
    &SolenoidsDebugEnvironment::m_sol1SolenoidTimer,
    SOL1_SOLENOID_ENERGIZE_TIME,
    SOL1_SOLENOID_DEENERGIZE_TIME);

DirectSolenoidChannel SolenoidsDebugEnvironment::m_sol2SolenoidChannel(
    &SolenoidsDebugEnvironment::m_sol2HighPin,
    &SolenoidsDebugEnvironment::m_sol2LowPin,
    &SolenoidsDebugEnvironment::m_sol2SolenoidTimer,
    SOL2_SOLENOID_ENERGIZE_TIME,
    SOL2_SOLENOID_DEENERGIZE_TIME);

SolenoidService SolenoidsDebugEnvironment::m_solenoidService;

// Indexed by the CMD_SET channel argument; index == kChannelCount selects all.
SolenoidChannel *const SolenoidsDebugEnvironment::m_channels[kChannelCount] = {
    &SolenoidsDebugEnvironment::m_clampSolenoidChannel,
    &SolenoidsDebugEnvironment::m_sol1SolenoidChannel,
    &SolenoidsDebugEnvironment::m_sol2SolenoidChannel
};

SolenoidsDebugEnvironment::SolenoidsDebugEnvironment()
{
    m_timerExpireService.addTimer(&m_clampSolenoidTimer, false);
    m_timerExpireService.addTimer(&m_sol1SolenoidTimer,  false);
    m_timerExpireService.addTimer(&m_sol2SolenoidTimer,  false);

    m_solenoidService.addChannel(&m_clampSolenoidChannel);
    m_solenoidService.addChannel(&m_sol1SolenoidChannel);
    m_solenoidService.addChannel(&m_sol2SolenoidChannel);

    addProcess(&m_timerExpireService);
    addProcess(&m_solenoidService);
}

void SolenoidsDebugEnvironment::handleCommand(uint16_t localCommand)
{
    if (localCommand != CMD_SET) {
        setError(ERROR_UNSUPPORTED_COMMAND);
        return;
    }

    set();
}

void SolenoidsDebugEnvironment::onPoll()
{
    if (!m_transitionActive || !selectedChannelsReachedTarget()) {
        return;
    }

    m_transitionActive = false;
    setDone(ERROR_NONE, m_selectAll ? kChannelCount : 1U);
}

void SolenoidsDebugEnvironment::abort()
{
    for (uint8_t i = 0U; i < kChannelCount; ++i) {
        m_channels[i]->deenergize();
    }

    m_transitionActive = false;
}

void SolenoidsDebugEnvironment::set()
{
    const uint32_t selectedChannel = static_cast<uint32_t>(arg(0));
    const float requestedState = arg(1);

    if (selectedChannel > kChannelCount ||
        (requestedState != 0.0f && requestedState != 1.0f)) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_selectAll = selectedChannel == kChannelCount;
    m_selectedChannel = static_cast<uint8_t>(selectedChannel);
    m_targetState = requestedState > 0.5f
        ? SolenoidChannel::State::ENERGIZED
        : SolenoidChannel::State::DEENERGIZED;

    for (uint8_t i = 0U; i < kChannelCount; ++i) {
        if (!channelIsSelected(i)) {
            continue;
        }

        if (m_targetState == SolenoidChannel::State::ENERGIZED) {
            m_channels[i]->energize();
        } else {
            m_channels[i]->deenergize();
        }
    }

    m_transitionActive = true;
    setBusy();
}

bool SolenoidsDebugEnvironment::selectedChannelsReachedTarget() const
{
    for (uint8_t i = 0U; i < kChannelCount; ++i) {
        if (channelIsSelected(i) && m_channels[i]->getState() != m_targetState) {
            return false;
        }
    }

    return true;
}

bool SolenoidsDebugEnvironment::channelIsSelected(uint8_t index) const
{
    return m_selectAll || index == m_selectedChannel;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_SOLENOIDS
