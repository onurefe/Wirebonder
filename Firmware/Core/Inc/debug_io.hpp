#ifndef DEBUG_IO_HPP
#define DEBUG_IO_HPP

#include <cstdint>

#include "configuration.h"
#include "pin_monitor_service.hpp"
#include "debug_service.hpp"

#define IO_DEBUG_LOG_DEPTH 64

struct IoDebugResult {
    volatile uint32_t eventCount;
    volatile uint8_t eventPin[IO_DEBUG_LOG_DEPTH];
    volatile uint8_t eventState[IO_DEBUG_LOG_DEPTH];  // 1 = ACTIVE, 2 = INACTIVE

    // Latest PinMonitorChannel::PinState per pin (0 = INACTIVE, 1 = ACTIVE),
    // refreshed every main-loop iteration while listening.
    volatile uint8_t states[8];
};

class DebugIo : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_IO;

    static constexpr uint8_t kMaxPins = 8U;

    enum Command : uint16_t {
        CMD_LISTEN = 1,
        CMD_STOP = 2
    };

    DebugIo();

    void init(PinMonitorChannel *const *pins, uint8_t pinCount);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        (void)localCommand;
        return true;
    }

    void handleCommand(uint16_t localCommand) override;
    void poll() override;

    bool isBusy() const override
    {
        return m_listening;
    }

    void abort() override;

private:
    struct PinCallbackContext {
        DebugIo *owner;
        uint8_t index;
    };

    static void onPinStateChanged(void *context,
                                  PinMonitorChannel::PinState state);

    void startListening();
    void stopListening();
    void resetResult();
    void logStateChange(uint8_t index, PinMonitorChannel::PinState state);

    PinMonitorChannel *const *m_pins = nullptr;
    uint8_t m_pinCount = 0U;

    PinCallbackContext m_pinContexts[kMaxPins] = {};

    IoDebugResult m_result = {};
    bool m_listening = false;
};

#endif /* DEBUG_IO_HPP */
