#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_KEYPAD

#include "DebugEnvironment/debug_environment_keypad.hpp"

extern I2C_HandleTypeDef hi2c1;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the keypad, but
// owned here outright.
// -----------------------------------------------------------------------------

IoExpanderService KeypadDebugEnvironment::m_ioExpanderService(&hi2c1);

Pca9535ExpanderChannel KeypadDebugEnvironment::m_keypadExpanderChannel(
    KEYPAD_EXPANDER_I2C_ADDRESS,
    KEYPAD_EXPANDER_PORT0_DIR,
    KEYPAD_EXPANDER_PORT1_DIR);

TimerExpireService KeypadDebugEnvironment::m_timerExpireService;
Timer KeypadDebugEnvironment::m_controlPanelPollTimer;

ControlPanelService KeypadDebugEnvironment::m_controlPanelService(
    &KeypadDebugEnvironment::m_keypadExpanderChannel,
    &KeypadDebugEnvironment::m_controlPanelPollTimer);

// The button log index reported through keyLog is the position in this array.
ButtonChannel KeypadDebugEnvironment::m_buttons[] = {
    {KEYPAD_BTN_UP_A,           KEYPAD_BTN_UP_B},
    {KEYPAD_BTN_DOWN_A,         KEYPAD_BTN_DOWN_B},
    {KEYPAD_BTN_LEFT_A,         KEYPAD_BTN_LEFT_B},
    {KEYPAD_BTN_RIGHT_A,        KEYPAD_BTN_RIGHT_B},
    {KEYPAD_BTN_PLUS_A,         KEYPAD_BTN_PLUS_B},
    {KEYPAD_BTN_MINUS_A,        KEYPAD_BTN_MINUS_B},
    {KEYPAD_BTN_SAVE_A,         KEYPAD_BTN_SAVE_B},
    {KEYPAD_BTN_LOAD_A,         KEYPAD_BTN_LOAD_B},
    {KEYPAD_BTN_TAIL_PLUS_A,    KEYPAD_BTN_TAIL_PLUS_B},
    {KEYPAD_BTN_TAIL_MINUS_A,   KEYPAD_BTN_TAIL_MINUS_B},
    {KEYPAD_BTN_LOOP_PLUS_A,    KEYPAD_BTN_LOOP_PLUS_B},
    {KEYPAD_BTN_LOOP_MINUS_A,   KEYPAD_BTN_LOOP_MINUS_B},
    {KEYPAD_BTN_SEARCH_PLUS_A,  KEYPAD_BTN_SEARCH_PLUS_B},
    {KEYPAD_BTN_SEARCH_MINUS_A, KEYPAD_BTN_SEARCH_MINUS_B},
    {KEYPAD_BTN_STEP_PLUS_A,    KEYPAD_BTN_STEP_PLUS_B},
    {KEYPAD_BTN_STEP_MINUS_A,   KEYPAD_BTN_STEP_MINUS_B},
    {KEYPAD_BTN_RESET_A,        KEYPAD_BTN_RESET_B},
    {KEYPAD_BTN_ENTER_A,        KEYPAD_BTN_ENTER_B},
    {KEYPAD_BTN_MANUAL_A,       KEYPAD_BTN_MANUAL_B},
    {KEYPAD_BTN_ESC_DEL_A,      KEYPAD_BTN_ESC_DEL_B},
    {KEYPAD_BTN_ADD_A,          KEYPAD_BTN_ADD_B},
    {KEYPAD_BTN_TEST_A,         KEYPAD_BTN_TEST_B},
    {KEYPAD_BTN_SETUP_A,        KEYPAD_BTN_SETUP_B},
    {KEYPAD_BTN_LIGHT_A,        KEYPAD_BTN_LIGHT_B},
    {KEYPAD_BTN_CLAMP_OPEN_A,   KEYPAD_BTN_CLAMP_OPEN_B},
    {KEYPAD_BTN_HIGH_RESET_A,   KEYPAD_BTN_HIGH_RESET_B}
};

const uint8_t KeypadDebugEnvironment::kButtonCount =
    sizeof(KeypadDebugEnvironment::m_buttons) /
    sizeof(KeypadDebugEnvironment::m_buttons[0]);

KeypadDebugEnvironment::KeypadDebugEnvironment()
{
    m_timerExpireService.addTimer(&m_controlPanelPollTimer, false);

    m_ioExpanderService.addExpander(&m_keypadExpanderChannel);

    for (uint8_t i = 0; i < kButtonCount; i++) {
        m_controlPanelService.addButton(&m_buttons[i]);
    }

    addProcess(&m_timerExpireService);
    addProcess(&m_ioExpanderService);
    addProcess(&m_controlPanelService);
}

void KeypadDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_LISTEN:
        startListening();
        break;

    case CMD_STOP:
        stopListening();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void KeypadDebugEnvironment::resetResult()
{
    m_result.keyCount = 0;
    m_result.stateCount = 0;

    for (uint16_t i = 0; i < KEYPAD_DEBUG_LOG_DEPTH; i++) {
        m_result.keyLog[i] = 0;
        m_result.states[i] = 0;
    }
}

void KeypadDebugEnvironment::startListening()
{
    for (uint8_t i = 0; i < kButtonCount; i++) {
        if (i < KEYPAD_DEBUG_LOG_DEPTH) {
            m_buttonContexts[i].owner = this;
            m_buttonContexts[i].index = i;

            m_buttons[i].addPressListenerCallback(
                &m_buttonContexts[i],
                &KeypadDebugEnvironment::onButtonPressed);
        }
    }

    resetResult();

    m_listening = true;
    setBusy();
    setResultPointer(0, &m_result);
}

void KeypadDebugEnvironment::stopListening()
{
    m_listening = false;
    setIdle();
}

void KeypadDebugEnvironment::abort()
{
    stopListening();
}

void KeypadDebugEnvironment::onButtonPressed(void *context)
{
    auto *buttonContext = static_cast<ButtonCallbackContext *>(context);

    if (buttonContext != nullptr && buttonContext->owner != nullptr) {
        buttonContext->owner->logButton(buttonContext->index);
    }
}

void KeypadDebugEnvironment::logButton(uint8_t index)
{
    if (!m_listening) {
        return;
    }

    const uint32_t count = m_result.keyCount;

    m_result.keyLog[count % KEYPAD_DEBUG_LOG_DEPTH] = index;
    m_result.keyCount = count + 1;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_KEYPAD
