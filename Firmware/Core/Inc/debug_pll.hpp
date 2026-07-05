#ifndef PLL_DEBUG_CHANNEL_HPP
#define PLL_DEBUG_CHANNEL_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "pll_module.hpp"

class DebugPll : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_PLL;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_BONDING_COMPLETED = 0,
        RESULT_DURATION_TIMEOUT = 1
    };

    DebugPll();

    void init(PllModule *pll);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    static void onPllEvent(void *context, PllModule::Event event);

    void start();
    void stop();
    void handlePllEvent(PllModule::Event event);

    PllModule *m_pll = nullptr;

    PllModule::TelemetrySample
        m_telemetry[DEBUG_PLL_TELEMETRY_DEPTH] = {};

    bool m_busy = false;
};

#endif /* PLL_DEBUG_CHANNEL_HPP */
