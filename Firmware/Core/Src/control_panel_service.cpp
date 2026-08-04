#include "control_panel_service.hpp"
#include "configuration.h"

// =============================================================================
// ButtonChannel
// =============================================================================

// A threshold of 0xFFFFFFFF disables auto-repeat: the hold duration cannot
// reach it within any realistic press.
ButtonChannel::ButtonChannel(uint8_t idcPin0, uint8_t idcPin1)
    : ButtonChannel(idcPin0, idcPin1, 0xFFFFFFFFU, 0xFFFFFFFFU)
{
}

ButtonChannel::ButtonChannel(uint8_t idcPin0, uint8_t idcPin1, uint32_t prolongedPressThresholdMs, uint32_t prolongedPressCallbackIntervalMs)
    : m_prolongedPressThresholdMs(prolongedPressThresholdMs)
    // A zero interval would divide the repeat rate by nothing to wait on; clamp
    // it to one tick so the rate degrades to the poll period instead.
    , m_prolongedPressCallbackIntervalMs(
          (prolongedPressCallbackIntervalMs == 0U) ? 1U : prolongedPressCallbackIntervalMs)
    , m_mask(static_cast<uint16_t>((1U << (idcPin0 - 1U)) | (1U << (idcPin1 - 1U))))
    , m_wasPressed(false)
    , m_lastRawLevel(false)
    , m_initialized(false)
    , m_lastChangeTick(0U)
    , m_pressStartTick(0U)
    , m_lastProlongedTick(0U)
    , m_pressCallbacks{}
    , m_prolongedPressCallbacks{}
    , m_pressCallbackCount(0)
    , m_prolongedPressCallbackCount(0)
{
}

bool ButtonChannel::addPressListenerCallback(void *context, PressCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_pressCallbackCount; i++) {
        if (m_pressCallbacks[i].context == context && m_pressCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_pressCallbackCount < kMaxCallbacks) {
        m_pressCallbacks[m_pressCallbackCount++] = CallbackRegistration{callback, context};
        return true;
    }

    return false;
}

bool ButtonChannel::addProlongedPressListenerCallback(void *context, ProlongedPressCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_prolongedPressCallbackCount; i++) {
        if (m_prolongedPressCallbacks[i].context == context && m_prolongedPressCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_prolongedPressCallbackCount < kMaxCallbacks) {
        m_prolongedPressCallbacks[m_prolongedPressCallbackCount++] =
            ProlongedCallbackRegistration{callback, context};
        return true;
    }

    return false;
}

bool ButtonChannel::removePressListenerCallback(void *context, PressCallback callback)
{
    for (uint8_t i = 0; i < m_pressCallbackCount; ++i) {
        if (m_pressCallbacks[i].context != context ||
            m_pressCallbacks[i].callback != callback) {
            continue;
        }

        for (uint8_t j = static_cast<uint8_t>(i + 1U);
             j < m_pressCallbackCount; ++j) {
            m_pressCallbacks[j - 1U] = m_pressCallbacks[j];
        }
        --m_pressCallbackCount;
        m_pressCallbacks[m_pressCallbackCount] = {};
        return true;
    }
    return false;
}

bool ButtonChannel::removeProlongedPressListenerCallback(void *context, ProlongedPressCallback callback)
{
    for (uint8_t i = 0; i < m_prolongedPressCallbackCount; ++i) {
        if (m_prolongedPressCallbacks[i].context != context ||
            m_prolongedPressCallbacks[i].callback != callback) {
            continue;
        }

        for (uint8_t j = static_cast<uint8_t>(i + 1U);
             j < m_prolongedPressCallbackCount; ++j) {
            m_prolongedPressCallbacks[j - 1U] = m_prolongedPressCallbacks[j];
        }
        --m_prolongedPressCallbackCount;
        m_prolongedPressCallbacks[m_prolongedPressCallbackCount] = {};
        return true;
    }
    return false;
}

void ButtonChannel::update(uint16_t state)
{
    bool pressed = (state & m_mask) == m_mask;

    if (!m_initialized) {
        m_initialized   = true;
        m_lastRawLevel  = pressed;
        return;
    }

    // Edges are detected on the raw level, which is what bounces. m_wasPressed
    // tracks something narrower: whether an *accepted* press is currently
    // held. Keeping the two apart matters because a rejected edge must still
    // move the debounce reference without ever arming the repeat.
    if (pressed == m_lastRawLevel) {
        return;
    }

    // The two DPST poles never settle in sync, so "both bits high" flickers
    // around each press and release. A press edge only counts when the
    // released state had been stable for the debounce interval; the bounced
    // edges arrive within it and are dropped.
    const uint32_t tick = HAL_GetTick();
    if (pressed) {
        if ((tick - m_lastChangeTick) >= KEYPAD_BTN_DEBOUNCE_MS) {
            // Hold time is measured from the accepted press edge, not from
            // m_lastChangeTick, which also moves on rejected edges.
            m_pressStartTick    = tick;
            m_lastProlongedTick = tick;
            m_wasPressed        = true;

            for (uint8_t i = 0; i < m_pressCallbackCount; i++) {
                m_pressCallbacks[i].callback(m_pressCallbacks[i].context);
            }
        }
        // A rejected press edge leaves m_wasPressed false: otherwise the
        // bounce that follows a release would re-arm the repeat with the
        // original press tick — already past every hold threshold — and
        // resume repeating at full step size after the operator let go.
    } else {
        m_wasPressed = false;
    }

    m_lastRawLevel   = pressed;
    m_lastChangeTick = tick;
}

void ButtonChannel::tick()
{
    if (!m_wasPressed || m_prolongedPressCallbackCount == 0U) {
        return;
    }

    const uint32_t tick   = HAL_GetTick();
    const uint32_t heldMs = tick - m_pressStartTick;

    if (heldMs < m_prolongedPressThresholdMs) {
        return;
    }

    // Compare against the last fire rather than testing the hold duration for
    // divisibility: the duration is only sampled once per poll, so an exact
    // multiple of the interval is almost never observed.
    if ((tick - m_lastProlongedTick) < m_prolongedPressCallbackIntervalMs) {
        return;
    }
    m_lastProlongedTick = tick;

    for (uint8_t i = 0; i < m_prolongedPressCallbackCount; i++) {
        m_prolongedPressCallbacks[i].callback(m_prolongedPressCallbacks[i].context, heldMs);
    }
}

void ButtonChannel::reset()
{
    const uint32_t tick = HAL_GetTick();

    m_wasPressed        = false;
    m_lastRawLevel      = false;
    m_initialized       = false;  // next update() re-seeds without firing
    m_lastChangeTick    = tick;
    m_pressStartTick    = tick;
    m_lastProlongedTick = tick;
}

// =============================================================================
// LedChannel
// =============================================================================

LedChannel::LedChannel(uint8_t idcPin)
    : m_bitIndex(static_cast<uint8_t>(idcPin - 1U))
    , m_on(false)
{
}

void LedChannel::on()  { m_on = true;  }
void LedChannel::off() { m_on = false; }

void LedChannel::set(bool value)
{
    if (value) on(); else off();
}

// =============================================================================
// ControlPanelService
// =============================================================================

ControlPanelService::ControlPanelService(Pca9535ExpanderChannel *expander, Timer *pollTimer)
    : m_expander(expander)
    , m_pollTimer(pollTimer)
    , m_outputPort0(0x00U)
    , m_outputPort1(0x00U)
    , m_buttons{}
    , m_buttonCount(0U)
    , m_leds{}
    , m_ledCount(0U)
    , m_outputWriteQueuedCallback(nullptr)
    , m_outputWriteCompletedCallback(nullptr)
    , m_outputWriteCallbackContext(nullptr)
{}

bool ControlPanelService::addButton(ButtonChannel *button)
{
    if (m_buttonCount >= kMaxButtons) return false;
    m_buttons[m_buttonCount++] = button;
    return true;
}

bool ControlPanelService::addLed(LedChannel *led)
{
    if (m_ledCount >= kMaxLeds) return false;
    m_leds[m_ledCount++] = led;
    return true;
}

void ControlPanelService::onStart()
{
    if (m_expander == nullptr || m_pollTimer == nullptr) {
        setProcessError();
        return;
    }
    m_pollTimer->setExpirationListenerCallback(this, onPollTimerExpired);

    m_outputPort0 = 0x00U;
    m_outputPort1 = 0x00U;

    for (uint8_t i = 0U; i < m_buttonCount; ++i) {
        m_buttons[i]->reset();
    }

    m_expander->setTransferListenerCallbacks(
        this, onKeypadStateChanged, onExpanderWriteCompleted);
    m_pollTimer->start(false, 1.0f / CONTROL_PANEL_POLL_FREQUENCY);
}

void ControlPanelService::onStop()
{
    if (m_pollTimer != nullptr) {
        m_pollTimer->stop();
        m_pollTimer->setExpirationListenerCallback(nullptr, nullptr);
    }
    if (m_expander != nullptr) {
        m_expander->setTransferListenerCallbacks(nullptr, nullptr, nullptr);
    }

    // A button still physically held here would otherwise stay latched as
    // pressed, since update() stops arriving once the expander is detached.
    for (uint8_t i = 0U; i < m_buttonCount; ++i) {
        m_buttons[i]->reset();
    }
}

void ControlPanelService::onExecute()
{
    uint8_t new_output_port0;
    uint8_t new_output_port1;

    concatenateLedStates(&new_output_port0, &new_output_port1);

    if (new_output_port0 != m_outputPort0) {
        uint8_t transactionId;
        if (m_expander->setPort0OutputValues(new_output_port0, &transactionId)) {
            m_outputPort0 = new_output_port0;
            if (m_outputWriteQueuedCallback != nullptr) {
                m_outputWriteQueuedCallback(m_outputWriteCallbackContext, transactionId);
            }
        }
    }
    if (new_output_port1 != m_outputPort1) {
        uint8_t transactionId;
        if (m_expander->setPort1OutputValues(new_output_port1, &transactionId)) {
            m_outputPort1 = new_output_port1;
            if (m_outputWriteQueuedCallback != nullptr) {
                m_outputWriteQueuedCallback(m_outputWriteCallbackContext, transactionId);
            }
        }
    }
}

bool ControlPanelService::ledOutputsMatch() const
{
    uint8_t outputPort0;
    uint8_t outputPort1;
    concatenateLedStates(&outputPort0, &outputPort1);
    return outputPort0 == m_outputPort0 && outputPort1 == m_outputPort1;
}

void ControlPanelService::setOutputWriteListenerCallbacks(
    void *context,
    OutputWriteCallback queuedCallback,
    OutputWriteCallback completedCallback)
{
    m_outputWriteCallbackContext = context;
    m_outputWriteQueuedCallback = queuedCallback;
    m_outputWriteCompletedCallback = completedCallback;
}

void ControlPanelService::onPollTimerExpired(void *context, Timer *timer)
{
    (void)timer;
    ControlPanelService *self = static_cast<ControlPanelService *>(context);
    if (!self->isOperating()) {
        return;
    }

    self->m_expander->readPortInputValues();

    for (uint8_t i = 0U; i < self->m_buttonCount; ++i) {
        self->m_buttons[i]->tick();
    }
}

void ControlPanelService::concatenateLedStates(uint8_t *outputPort0,
                                                uint8_t *outputPort1) const
{
    uint16_t port_val = 0;

    for (uint8_t i = 0U; i < m_ledCount; ++i) {
        uint8_t  bit  = m_leds[i]->getBitIndex();
        bool     on   = m_leds[i]->isOn();
        uint16_t mask = static_cast<uint16_t>(1U << bit);
        if (on) port_val |= mask;
    }

    *outputPort0 = static_cast<uint8_t>(port_val & 0x00FFU);
    *outputPort1 = static_cast<uint8_t>((port_val & 0xFF00U) >> 8U);
}

void ControlPanelService::onKeypadStateChanged(void *context, uint8_t port0, uint8_t port1)
{
    ControlPanelService *self  = static_cast<ControlPanelService *>(context);

    if (self == nullptr || !self->isOperating()) {
        return;
    }

    uint16_t             state = static_cast<uint16_t>(port0)
                               | static_cast<uint16_t>(static_cast<uint16_t>(port1) << 8U);

    for (uint8_t i = 0U; i < self->m_buttonCount; ++i) {
        self->m_buttons[i]->update(state);
    }
}

void ControlPanelService::onExpanderWriteCompleted(void *context,
                                                    uint8_t transactionId)
{
    ControlPanelService *self = static_cast<ControlPanelService *>(context);
    if (self == nullptr || self->m_outputWriteCompletedCallback == nullptr) {
        return;
    }

    self->m_outputWriteCompletedCallback(
        self->m_outputWriteCallbackContext, transactionId);
}
