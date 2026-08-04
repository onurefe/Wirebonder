#ifndef CONTROL_PANEL_SERVICE_HPP
#define CONTROL_PANEL_SERVICE_HPP

#include "io_expander_service.hpp"
#include "timer_expire_service.hpp"
#include "process.hpp"
#include "generic.h"
#include <cstdint>

// ---------------------------------------------------------------------------
// ButtonChannel
//
// Represents one DPST button wired to two IDC connector pins.
// Both pins are inputs on PCA9535. The button is considered pressed when
// both pins read HIGH simultaneously.
//
// IDC pin → PCA9535 bit index: pin N → index N-1
//   Index  0– 7 = Port 0, bits 0–7  (IDC pins  1– 8)
//   Index  8–15 = Port 1, bits 0–7  (IDC pins  9–16)
// ---------------------------------------------------------------------------
class ButtonChannel {
public:
    using PressCallback = void (*)(void *context);
    // Prolonged presses report how long the button has been held, measured
    // from the accepted press edge. Listeners that scale their action with
    // the hold time need no state of their own, which keeps them off the
    // ISR/main-loop boundary: presses arrive from the I2C completion
    // interrupt, repeats from the poll timer in the main loop.
    using ProlongedPressCallback = void (*)(void *context, uint32_t heldMs);

    // idcPin0, idcPin1: the two IDC connector pins wired to this button (1-based).
    ButtonChannel(uint8_t idcPin0, uint8_t idcPin1);
    ButtonChannel(uint8_t idcPin0, uint8_t idcPin1, uint32_t prolongedPressThresholdMs, uint32_t prolongedPressCallbackIntervalMs);
    virtual ~ButtonChannel() = default;

    bool addPressListenerCallback(void *context, PressCallback callback);
    bool addProlongedPressListenerCallback(void *context, ProlongedPressCallback callback);
    bool removePressListenerCallback(void *context, PressCallback callback);
    bool removeProlongedPressListenerCallback(void *context, ProlongedPressCallback callback);

    // Called by ControlPanelService with the latest 16-bit port state.
    // Fires the press callback on the LOW→HIGH transition of both bits.
    // Virtual so diagnostic channels can observe the raw state directly.
    virtual void update(uint16_t state);

    // Called by ControlPanelService on every poll. Fires the prolonged press
    // callbacks at a fixed interval once the button has been held longer than
    // the threshold. The repeat rate is capped by the poll period.
    virtual void tick();

    // Drops any latched press state. Called when the owning service starts or
    // stops so a button held across that boundary cannot keep repeating.
    virtual void reset();

    uint16_t getMask() const { return m_mask; }

private:
    struct CallbackRegistration {
        PressCallback callback;
        void *context;
    };

    struct ProlongedCallbackRegistration {
        ProlongedPressCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxCallbacks = 4U;

    uint32_t                m_prolongedPressThresholdMs;
    uint32_t                m_prolongedPressCallbackIntervalMs;
    uint16_t                m_mask;
    bool                    m_wasPressed;        // an accepted press is held
    bool                    m_lastRawLevel;      // last sampled level, bounces included
    bool                    m_initialized;
    uint32_t                m_lastChangeTick;    // debounce: last raw edge (ms)
    uint32_t                m_pressStartTick;    // last accepted press edge (ms)
    uint32_t                m_lastProlongedTick; // last prolonged callback fire (ms)
    CallbackRegistration    m_pressCallbacks[kMaxCallbacks];
    ProlongedCallbackRegistration m_prolongedPressCallbacks[kMaxCallbacks];
    uint8_t                 m_pressCallbackCount;
    uint8_t                 m_prolongedPressCallbackCount;
};

// ---------------------------------------------------------------------------
// LedChannel
//
// Represents one indicator LED wired to a single IDC connector pin (the anode).
// The common cathode is a fixed LOW output configured in the Pca9535ExpanderChannel.
// ---------------------------------------------------------------------------
class LedChannel {
public:
    // idcPin: the IDC connector pin tied to the LED anode (1-based).
    explicit LedChannel(uint8_t idcPin);

    void on();
    void off();
    void set(bool value);
    bool isOn() const { return m_on; }

    // Internal use by ControlPanelService.
    uint8_t getBitIndex() const { return m_bitIndex; }

private:
    uint8_t m_bitIndex;
    bool    m_on;
};

// ---------------------------------------------------------------------------
// ControlPanelService
//
// Polls a Pca9535ExpanderChannel on a timer, propagates input state to all
// registered ButtonChannels, and drives output state for all LedChannels.
//
// Setup responsibilities (caller):
//   - Configure Pca9535ExpanderChannel with button pins as inputs (direction bit = 1)
//     and LED/cathode pins as outputs (direction bit = 0).
//   - Set port initial output to 0x00 so cathode lines start LOW.
//   - Register the injected timer with TimerExpireService (timeCritical = false).
//   - Start the process before entering the main loop.
//   - Execute the process every main-loop iteration for LED updates.
// ---------------------------------------------------------------------------
class ControlPanelService : public Process {
public:
    using OutputWriteCallback = void (*)(void *context, uint8_t transactionId);

    ControlPanelService(Pca9535ExpanderChannel *expander, Timer *pollTimer);

    bool addButton(ButtonChannel *button);
    bool addLed(LedChannel *led);

    bool ledOutputsMatch() const;
    void setOutputWriteListenerCallbacks(void *context,
                                         OutputWriteCallback queuedCallback,
                                         OutputWriteCallback completedCallback);

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    static void onPollTimerExpired(void *context, Timer *timer);
    static void onKeypadStateChanged(void *context, uint8_t port0, uint8_t port1);
    static void onExpanderWriteCompleted(void *context, uint8_t transactionId);
    void concatenateLedStates(uint8_t *outputPort0, uint8_t *outputPort1) const;

    Pca9535ExpanderChannel *m_expander;
    Timer                  *m_pollTimer;

    uint8_t  m_outputPort0;
    uint8_t  m_outputPort1;

    static constexpr uint8_t kMaxButtons = 32U;
    static constexpr uint8_t kMaxLeds    = 16U;

    ButtonChannel *m_buttons[kMaxButtons];
    uint8_t        m_buttonCount;
    LedChannel    *m_leds[kMaxLeds];
    uint8_t        m_ledCount;
    OutputWriteCallback m_outputWriteQueuedCallback;
    OutputWriteCallback m_outputWriteCompletedCallback;
    void                *m_outputWriteCallbackContext;
};

#endif /* CONTROL_PANEL_SERVICE_HPP */
