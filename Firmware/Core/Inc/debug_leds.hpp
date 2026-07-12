#ifndef DEBUG_LEDS_HPP
#define DEBUG_LEDS_HPP

#include <cstdint>

#include "configuration.h"
#include "control_panel_service.hpp"
#include "debug_service.hpp"

class DebugLeds : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_LEDS;

    enum Command : uint16_t {
        CMD_SET = 1
    };

    enum Led : uint32_t {
        LED_TEST = 0,
        LED_SETUP = 1,
        LED_CLAMP_OPEN = 2,
        LED_MANUAL = 3,
        LED_ALL = 4
    };

    DebugLeds();

    void init(LedChannel *const *leds,
              uint8_t ledCount,
              ControlPanelService *controlPanel);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    void handleCommand(uint16_t localCommand) override;
    void poll() override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    static constexpr uint8_t kMaxOutputWrites = 2U;

    static void onOutputWriteQueued(void *context, uint8_t transactionId);
    static void onOutputWriteCompleted(void *context, uint8_t transactionId);

    void set();
    bool writesCompleted() const;

    LedChannel *const *m_leds = nullptr;
    ControlPanelService *m_controlPanel = nullptr;
    uint8_t m_ledCount = 0U;
    uint8_t m_expectedWriteIds[kMaxOutputWrites] = {};
    uint8_t m_expectedWriteCount = 0U;
    uint8_t m_completedWriteMask = 0U;
    uint32_t m_changedLedCount = 0U;
    bool m_busy = false;
};

#endif /* DEBUG_LEDS_HPP */
