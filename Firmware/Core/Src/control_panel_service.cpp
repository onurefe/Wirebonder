#include "control_panel_service.hpp"
#include "configuration.h"

// =============================================================================
// ButtonChannel
// =============================================================================

ButtonChannel::ButtonChannel(uint8_t idcPin0, uint8_t idcPin1)
    : m_mask(static_cast<uint16_t>((1U << (idcPin0 - 1U)) | (1U << (idcPin1 - 1U))))
    , m_wasPressed(false)
    , m_initialized(false)
    , m_callbacks{}
    , m_callbackCount(0)
{
}

bool ButtonChannel::addPressListenerCallback(void *context, PressCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_callbackCount; i++) {
        if (m_callbacks[i].context == context && m_callbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_callbackCount < kMaxCallbacks) {
        m_callbacks[m_callbackCount++] = CallbackRegistration{callback, context};
        return true;
    }

    return false;
}

void ButtonChannel::update(uint16_t state)
{
    bool pressed = (state & m_mask) == m_mask;

    if (!m_initialized) {
        m_initialized = true;
        m_wasPressed  = pressed;
        return;
    }

    if (pressed && !m_wasPressed) {
        for (uint8_t i = 0; i < m_callbackCount; i++) {
            m_callbacks[i].callback(m_callbacks[i].context);
        }
    }
    m_wasPressed = pressed;
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
    , m_state(ServiceState::READY)
{
    m_pollTimer->setExpirationListenerCallback(this, onPollTimerExpired);
}

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

void ControlPanelService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    m_outputPort0 = 0x00U;
    m_outputPort1 = 0x00U;

    m_expander->setTransferListenerCallbacks(
        this, onKeypadStateChanged, onExpanderWriteCompleted);
    m_pollTimer->start(false, 1.0f / CONTROL_PANEL_POLL_FREQUENCY);
    m_state = ServiceState::OPERATING;
}

void ControlPanelService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    m_pollTimer->stop();
    m_expander->setTransferListenerCallbacks(nullptr, nullptr, nullptr);
    m_state = ServiceState::READY;
}

void ControlPanelService::executeService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

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
    if (self->m_state == ServiceState::OPERATING) {
        self->m_expander->readPortInputValues();
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

    if (self == nullptr || self->m_state != ServiceState::OPERATING) {
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
