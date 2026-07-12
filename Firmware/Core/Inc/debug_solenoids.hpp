#ifndef DEBUG_SOLENOIDS_HPP
#define DEBUG_SOLENOIDS_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "solenoid_service.hpp"

class DebugSolenoids : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_SOLENOIDS;

    enum Command : uint16_t {
        CMD_SET = 1
    };

    DebugSolenoids();

    void init(DirectSolenoidChannel *const *channels, uint8_t channelCount);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_SET;
    }

    void handleCommand(uint16_t localCommand) override;
    void poll() override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    void set();
    bool selectedChannelsReachedTarget() const;
    bool channelIsSelected(uint8_t index) const;

    DirectSolenoidChannel *const *m_channels = nullptr;
    uint8_t m_channelCount = 0U;
    uint8_t m_selectedChannel = 0U;
    DirectSolenoidChannel::State m_targetState =
        DirectSolenoidChannel::State::DEENERGIZED;
    bool m_selectAll = false;
    bool m_busy = false;
};

#endif /* DEBUG_SOLENOIDS_HPP */
