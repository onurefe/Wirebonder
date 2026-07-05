#ifndef DEBUG_FORCE_COIL_HPP
#define DEBUG_FORCE_COIL_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "force_coil_module.hpp"

class DebugForceCoil : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_FORCE_COIL;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_CAPTURE_COMPLETE = 0
    };

    DebugForceCoil();

    void init(ForceCoilDriverModule *forceCoil);

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
    static void onCurrentMeasured(void *context, float measuredCurrent);

    void start();
    void stop();
    void recordCurrent(float measuredCurrent);

    ForceCoilDriverModule *m_forceCoil = nullptr;

    float m_telemetry[DEBUG_FORCE_COIL_TELEMETRY_DEPTH] = {};
    uint32_t m_sampleIdx = 0;
    uint32_t m_sampleLimit = 0;
    bool m_busy = false;
};

#endif /* DEBUG_FORCE_COIL_HPP */
