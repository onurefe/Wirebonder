#include "debug_leds.hpp"

DebugLeds::DebugLeds()
{
}

void DebugLeds::init(LedChannel *const *leds,
                     uint8_t ledCount,
                     ControlPanelService *controlPanel)
{
    m_leds = leds;
    m_ledCount = ledCount;
    m_controlPanel = controlPanel;

    if (m_controlPanel != nullptr) {
        m_controlPanel->setOutputWriteListenerCallbacks(
            this, onOutputWriteQueued, onOutputWriteCompleted);
    }
}

void DebugLeds::handleCommand(uint16_t localCommand)
{
    if (localCommand != CMD_SET) {
        setError(ERROR_UNSUPPORTED_COMMAND);
        return;
    }

    set();
}

void DebugLeds::poll()
{
    if (!m_busy) {
        return;
    }

    if (m_expectedWriteCount == 0U) {
        if (!m_controlPanel->ledOutputsMatch()) {
            return;
        }

        m_busy = false;
        setDone(ERROR_NONE, m_changedLedCount);
        return;
    }

    if (!writesCompleted()) {
        return;
    }

    m_busy = false;
    setDone(ERROR_NONE, m_changedLedCount);
}

void DebugLeds::abort()
{
    m_busy = false;
    m_expectedWriteCount = 0U;
    m_completedWriteMask = 0U;
}

void DebugLeds::set()
{
    const uint32_t led = static_cast<uint32_t>(arg(0));
    const float requestedState = arg(1);

    if (m_leds == nullptr || m_ledCount == 0U || m_controlPanel == nullptr ||
        requestedState < 0.0f || requestedState > 1.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    const bool isOn = requestedState >= 0.5f;
    if (led == LED_ALL) {
        for (uint8_t i = 0U; i < m_ledCount; ++i) {
            m_leds[i]->set(isOn);
        }
        m_changedLedCount = m_ledCount;
    } else if (led < m_ledCount) {
        m_leds[led]->set(isOn);
        m_changedLedCount = 1U;
    } else {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_expectedWriteCount = 0U;
    m_completedWriteMask = 0U;
    m_busy = true;
    setBusy();
}

void DebugLeds::onOutputWriteQueued(void *context, uint8_t transactionId)
{
    DebugLeds *self = static_cast<DebugLeds *>(context);
    if (self == nullptr || !self->m_busy ||
        self->m_expectedWriteCount >= kMaxOutputWrites) {
        return;
    }

    self->m_expectedWriteIds[self->m_expectedWriteCount++] = transactionId;
}

void DebugLeds::onOutputWriteCompleted(void *context, uint8_t transactionId)
{
    DebugLeds *self = static_cast<DebugLeds *>(context);
    if (self == nullptr || !self->m_busy) {
        return;
    }

    for (uint8_t i = 0U; i < self->m_expectedWriteCount; ++i) {
        if (self->m_expectedWriteIds[i] == transactionId) {
            self->m_completedWriteMask |= static_cast<uint8_t>(1U << i);
            return;
        }
    }
}

bool DebugLeds::writesCompleted() const
{
    const uint8_t completedMask = static_cast<uint8_t>(
        (1U << m_expectedWriteCount) - 1U);
    return m_completedWriteMask == completedMask;
}
