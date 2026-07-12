#include "debug_solenoids.hpp"

DebugSolenoids::DebugSolenoids()
{
}

void DebugSolenoids::init(DirectSolenoidChannel *const *channels,
                          uint8_t channelCount)
{
    m_channels = channels;
    m_channelCount = channelCount;
}

void DebugSolenoids::handleCommand(uint16_t localCommand)
{
    if (localCommand != CMD_SET) {
        setError(ERROR_UNSUPPORTED_COMMAND);
        return;
    }

    set();
}

void DebugSolenoids::poll()
{
    if (!m_busy || !selectedChannelsReachedTarget()) {
        return;
    }

    m_busy = false;
    setDone(ERROR_NONE, m_selectAll ? m_channelCount : 1U);
}

void DebugSolenoids::abort()
{
    if (m_channels != nullptr) {
        for (uint8_t i = 0U; i < m_channelCount; ++i) {
            m_channels[i]->deenergize();
        }
    }

    m_busy = false;
}

void DebugSolenoids::set()
{
    const uint32_t selectedChannel = static_cast<uint32_t>(arg(0));
    const float requestedState = arg(1);

    if (m_channels == nullptr || m_channelCount == 0U ||
        selectedChannel > m_channelCount ||
        (requestedState != 0.0f && requestedState != 1.0f)) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_selectAll = selectedChannel == m_channelCount;
    m_selectedChannel = static_cast<uint8_t>(selectedChannel);
    m_targetState = requestedState > 0.5f
        ? DirectSolenoidChannel::State::ENERGIZED
        : DirectSolenoidChannel::State::DEENERGIZED;

    for (uint8_t i = 0U; i < m_channelCount; ++i) {
        if (!channelIsSelected(i)) {
            continue;
        }

        if (m_targetState == DirectSolenoidChannel::State::ENERGIZED) {
            m_channels[i]->energize();
        } else {
            m_channels[i]->deenergize();
        }
    }

    m_busy = true;
    setBusy();
}

bool DebugSolenoids::selectedChannelsReachedTarget() const
{
    for (uint8_t i = 0U; i < m_channelCount; ++i) {
        if (channelIsSelected(i) && m_channels[i]->getState() != m_targetState) {
            return false;
        }
    }

    return true;
}

bool DebugSolenoids::channelIsSelected(uint8_t index) const
{
    return m_selectAll || index == m_selectedChannel;
}
