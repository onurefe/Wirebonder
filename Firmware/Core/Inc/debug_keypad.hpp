#ifndef KEYPAD_DEBUG_CHANNEL_HPP
#define KEYPAD_DEBUG_CHANNEL_HPP

#include <cstdint>

#include "configuration.h"
#include "control_panel_service.hpp"
#include "debug_service.hpp"

#define KEYPAD_DEBUG_LOG_DEPTH 64

struct KeypadDebugResult {
    volatile uint32_t keyCount;
    volatile uint8_t keyLog[KEYPAD_DEBUG_LOG_DEPTH];

    volatile uint32_t stateCount;
    volatile uint16_t states[KEYPAD_DEBUG_LOG_DEPTH];
};

class DebugKeypad;

class KeypadDaemonButton : public ButtonChannel {
public:
    explicit KeypadDaemonButton(DebugKeypad *owner);

    void update(uint16_t state) override;

private:
    DebugKeypad *m_owner = nullptr;
};

class DebugKeypad : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_KEYPAD;

    enum Command : uint16_t {
        CMD_LISTEN = 1,
        CMD_STOP = 2
    };

    DebugKeypad();

    void init(ButtonChannel *const *buttons,
              uint8_t buttonCount);

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

    bool isBusy() const override
    {
        return m_listening;
    }

    void abort() override;

    void logState(uint16_t state);

private:
    struct ButtonCallbackContext {
        DebugKeypad *owner;
        uint8_t index;
    };

    static void onButtonPressed(void *context);

    void startListening();
    void stopListening();
    void resetResult();
    void logButton(uint8_t index);

    ButtonChannel *const *m_buttons = nullptr;
    uint8_t m_buttonCount = 0;

    ButtonCallbackContext m_buttonContexts[KEYPAD_DEBUG_LOG_DEPTH] = {};

    KeypadDaemonButton m_daemonButton;

    KeypadDebugResult m_result = {};
    bool m_listening = false;
};

#endif /* KEYPAD_DEBUG_CHANNEL_HPP */
