#include "debug_io.hpp"

DebugIo::DebugIo()
{
}

void DebugIo::init(PinMonitorChannel *const *pins, uint8_t pinCount)
{
    m_pins = pins;
    m_pinCount = (pinCount <= kMaxPins) ? pinCount : kMaxPins;
}

void DebugIo::handleCommand(uint16_t localCommand)
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

void DebugIo::poll()
{
    if (!m_listening) {
        return;
    }

    for (uint8_t i = 0U; i < m_pinCount; ++i) {
        m_result.states[i] = static_cast<uint8_t>(m_pins[i]->getPinState());
    }
}

void DebugIo::abort()
{
    stopListening();
}

void DebugIo::startListening()
{
    if (m_pins == nullptr || m_pinCount == 0U) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    resetResult();

    // Note: PinMonitorChannel holds a single listener slot, so this replaces
    // any callback the application registered on these channels.
    for (uint8_t i = 0U; i < m_pinCount; ++i) {
        m_pinContexts[i].owner = this;
        m_pinContexts[i].index = i;

        m_pins[i]->addStateListenerCallback(&m_pinContexts[i],
                                            &DebugIo::onPinStateChanged);
    }

    m_listening = true;
    setBusy();
    setResultPointer(0, &m_result);
}

void DebugIo::stopListening()
{
    m_listening = false;
    setIdle();
}

void DebugIo::resetResult()
{
    m_result.eventCount = 0U;

    for (uint16_t i = 0U; i < IO_DEBUG_LOG_DEPTH; ++i) {
        m_result.eventPin[i] = 0U;
        m_result.eventState[i] = 0U;
    }

    for (uint8_t i = 0U; i < kMaxPins; ++i) {
        m_result.states[i] =
            static_cast<uint8_t>(PinMonitorChannel::PinState::INACTIVE);
    }
}

void DebugIo::onPinStateChanged(void *context,
                                PinMonitorChannel::PinState state)
{
    auto *pinContext = static_cast<PinCallbackContext *>(context);

    if (pinContext != nullptr && pinContext->owner != nullptr) {
        pinContext->owner->logStateChange(pinContext->index, state);
    }
}

// Critical pins (TIP, YLIM) fire this from the 1 ms sampling ISR; keep it
// minimal, matching the keypad channel's logging.
void DebugIo::logStateChange(uint8_t index,
                             PinMonitorChannel::PinState state)
{
    if (!m_listening) {
        return;
    }

    const uint32_t count = m_result.eventCount;
    const uint32_t slot = count % IO_DEBUG_LOG_DEPTH;

    m_result.eventPin[slot] = index;
    m_result.eventState[slot] =
        (state == PinMonitorChannel::PinState::ACTIVE) ? 1U : 2U;
    m_result.eventCount = count + 1U;
}
