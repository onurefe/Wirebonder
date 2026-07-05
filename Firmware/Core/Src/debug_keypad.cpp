#include "debug_keypad.hpp"
#include <cstdint>

KeypadDaemonButton::KeypadDaemonButton(DebugKeypad *owner)
    : ButtonChannel(1, 1),
      m_owner(owner)
{
}

void KeypadDaemonButton::update(uint16_t state)
{
    if (m_owner != nullptr) {
        m_owner->logState(state);
    }
}

DebugKeypad::DebugKeypad()
    : m_daemonButton(this)
{
}

void DebugKeypad::init(ButtonChannel *const *buttons,
                              uint8_t buttonCount)
{
    m_buttons = buttons;
    m_buttonCount = buttonCount;
}

void DebugKeypad::handleCommand(uint16_t localCommand)
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

void DebugKeypad::resetResult()
{
    m_result.keyCount = 0;
    m_result.stateCount = 0;

    for (uint16_t i = 0; i < KEYPAD_DEBUG_LOG_DEPTH; i++) {
        m_result.keyLog[i] = 0;
        m_result.states[i] = 0;
    }
}

void DebugKeypad::startListening()
{
    if (m_buttons == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    for (uint8_t i = 0; i < m_buttonCount; i++) {
        if (m_buttons[i] != nullptr && i < KEYPAD_DEBUG_LOG_DEPTH) {
            m_buttonContexts[i].owner = this;
            m_buttonContexts[i].index = i;

            m_buttons[i]->addPressListenerCallback(&m_buttonContexts[i],
                                                   &DebugKeypad::onButtonPressed);
        }
    }

    resetResult();

    m_listening = true;
    setBusy();
    setResultPointer(0, &m_result);
}

void DebugKeypad::stopListening()
{
    m_listening = false;
    setIdle();
}

void DebugKeypad::abort()
{
    stopListening();
}

void DebugKeypad::onButtonPressed(void *context)
{
    auto *buttonContext = static_cast<ButtonCallbackContext *>(context);

    if (buttonContext != nullptr && buttonContext->owner != nullptr) {
        buttonContext->owner->logButton(buttonContext->index);
    }
}

void DebugKeypad::logButton(uint8_t index)
{
    if (!m_listening) {
        return;
    }

    const uint32_t count = m_result.keyCount;

    m_result.keyLog[count % KEYPAD_DEBUG_LOG_DEPTH] = index;
    m_result.keyCount = count + 1;
}

void DebugKeypad::logState(uint16_t state)
{
    if (!m_listening) {
        return;
    }

    const uint32_t count = m_result.stateCount;

    m_result.states[count % KEYPAD_DEBUG_LOG_DEPTH] = state;
    m_result.stateCount = count + 1;
}
