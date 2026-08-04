#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LEDS

#include "DebugEnvironment/debug_environment_leds.hpp"

extern I2C_HandleTypeDef hi2c1;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the panel LEDs,
// but owned here outright.
// -----------------------------------------------------------------------------

IoExpanderService LedsDebugEnvironment::m_ioExpanderService(&hi2c1);

Pca9535ExpanderChannel LedsDebugEnvironment::m_keypadExpanderChannel(
    KEYPAD_EXPANDER_I2C_ADDRESS,
    KEYPAD_EXPANDER_PORT0_DIR,
    KEYPAD_EXPANDER_PORT1_DIR);

TimerExpireService LedsDebugEnvironment::m_timerExpireService;
Timer LedsDebugEnvironment::m_controlPanelPollTimer;

ControlPanelService LedsDebugEnvironment::m_controlPanelService(
    &LedsDebugEnvironment::m_keypadExpanderChannel,
    &LedsDebugEnvironment::m_controlPanelPollTimer);

// Indexed by the Led command argument.
LedChannel LedsDebugEnvironment::m_leds[] = {
    LedChannel(KEYPAD_LED_TEST),
    LedChannel(KEYPAD_LED_SETUP),
    LedChannel(KEYPAD_LED_CLAMP_OPEN),
    LedChannel(KEYPAD_LED_MANUAL)
};

const uint8_t LedsDebugEnvironment::kLedCount =
    sizeof(LedsDebugEnvironment::m_leds) /
    sizeof(LedsDebugEnvironment::m_leds[0]);

LedsDebugEnvironment::LedsDebugEnvironment()
{
    m_timerExpireService.addTimer(&m_controlPanelPollTimer, false);

    m_ioExpanderService.addExpander(&m_keypadExpanderChannel);

    for (uint8_t i = 0; i < kLedCount; i++) {
        m_controlPanelService.addLed(&m_leds[i]);
    }

    m_controlPanelService.setOutputWriteListenerCallbacks(
        this,
        &LedsDebugEnvironment::onOutputWriteQueued,
        &LedsDebugEnvironment::onOutputWriteCompleted);

    addProcess(&m_timerExpireService);
    addProcess(&m_ioExpanderService);
    addProcess(&m_controlPanelService);
}

void LedsDebugEnvironment::handleCommand(uint16_t localCommand)
{
    if (localCommand != CMD_SET) {
        setError(ERROR_UNSUPPORTED_COMMAND);
        return;
    }

    set();
}

void LedsDebugEnvironment::onPoll()
{
    if (!m_writeActive) {
        return;
    }

    if (m_expectedWriteCount == 0U) {
        if (!m_controlPanelService.ledOutputsMatch()) {
            return;
        }

        m_writeActive = false;
        setDone(ERROR_NONE, m_changedLedCount);
        return;
    }

    if (!writesCompleted()) {
        return;
    }

    m_writeActive = false;
    setDone(ERROR_NONE, m_changedLedCount);
}

void LedsDebugEnvironment::abort()
{
    m_writeActive = false;
    m_expectedWriteCount = 0U;
    m_completedWriteMask = 0U;
}

void LedsDebugEnvironment::set()
{
    const uint32_t led = static_cast<uint32_t>(arg(0));
    const float requestedState = arg(1);

    if (requestedState < 0.0f || requestedState > 1.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    const bool isOn = requestedState >= 0.5f;
    if (led == LED_ALL) {
        for (uint8_t i = 0U; i < kLedCount; ++i) {
            m_leds[i].set(isOn);
        }
        m_changedLedCount = kLedCount;
    } else if (led < kLedCount) {
        m_leds[led].set(isOn);
        m_changedLedCount = 1U;
    } else {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_expectedWriteCount = 0U;
    m_completedWriteMask = 0U;
    m_writeActive = true;
    setBusy();
}

void LedsDebugEnvironment::onOutputWriteQueued(void *context,
                                               uint8_t transactionId)
{
    auto *self = static_cast<LedsDebugEnvironment *>(context);
    if (self == nullptr || !self->m_writeActive ||
        self->m_expectedWriteCount >= kMaxOutputWrites) {
        return;
    }

    self->m_expectedWriteIds[self->m_expectedWriteCount++] = transactionId;
}

void LedsDebugEnvironment::onOutputWriteCompleted(void *context,
                                                  uint8_t transactionId)
{
    auto *self = static_cast<LedsDebugEnvironment *>(context);
    if (self == nullptr || !self->m_writeActive) {
        return;
    }

    for (uint8_t i = 0U; i < self->m_expectedWriteCount; ++i) {
        if (self->m_expectedWriteIds[i] == transactionId) {
            self->m_completedWriteMask |= static_cast<uint8_t>(1U << i);
            return;
        }
    }
}

bool LedsDebugEnvironment::writesCompleted() const
{
    const uint8_t completedMask = static_cast<uint8_t>(
        (1U << m_expectedWriteCount) - 1U);
    return m_completedWriteMask == completedMask;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LEDS
