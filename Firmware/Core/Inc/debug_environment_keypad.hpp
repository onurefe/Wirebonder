#ifndef DEBUG_ENVIRONMENT_KEYPAD_HPP
#define DEBUG_ENVIRONMENT_KEYPAD_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_environment.hpp"
#include "timer_expire_service.hpp"
#include "io_expander_service.hpp"
#include "control_panel_service.hpp"

#define KEYPAD_DEBUG_LOG_DEPTH 64

struct KeypadDebugResult {
    volatile uint32_t keyCount;
    volatile uint8_t keyLog[KEYPAD_DEBUG_LOG_DEPTH];

    volatile uint32_t stateCount;
    volatile uint16_t states[KEYPAD_DEBUG_LOG_DEPTH];
};

// Sandbox for keypad bring-up: the I2C expander + ControlPanelService with
// every panel button registered, logging presses into a GDB-readable ring.
class KeypadDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_KEYPAD;

    enum Command : uint16_t {
        CMD_LISTEN = 1,
        CMD_STOP = 2
    };

    KeypadDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
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

private:
    struct ButtonCallbackContext {
        KeypadDebugEnvironment *owner;
        uint8_t index;
    };

    static void onButtonPressed(void *context);

    void startListening();
    void stopListening();
    void resetResult();
    void logButton(uint8_t index);

    // Hardware sandbox — exclusively owned by this environment.
    static IoExpanderService m_ioExpanderService;
    static Pca9535ExpanderChannel m_keypadExpanderChannel;

    static TimerExpireService m_timerExpireService;
    static Timer m_controlPanelPollTimer;

    static ControlPanelService m_controlPanelService;

    static ButtonChannel m_buttons[];
    static const uint8_t kButtonCount;

    ButtonCallbackContext m_buttonContexts[KEYPAD_DEBUG_LOG_DEPTH] = {};

    KeypadDebugResult m_result = {};
    bool m_listening = false;
};

#endif /* DEBUG_ENVIRONMENT_KEYPAD_HPP */
