#ifndef DEBUG_ENVIRONMENT_LEDS_HPP
#define DEBUG_ENVIRONMENT_LEDS_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_environment.hpp"
#include "timer_expire_service.hpp"
#include "io_expander_service.hpp"
#include "control_panel_service.hpp"

// Sandbox for the panel LEDs: the keypad I2C expander + ControlPanelService
// with the four LED outputs registered.
class LedsDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_LEDS;

    enum Command : uint16_t {
        CMD_SET = 1
    };

    enum Led : uint32_t {
        LED_TEST = 0,
        LED_SETUP = 1,
        LED_CLAMP_OPEN = 2,
        LED_MANUAL = 3,
        LED_ALL = 4
    };

    LedsDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    void handleCommand(uint16_t localCommand) override;
    void onPoll() override;

    bool isBusy() const override
    {
        return m_writeActive;
    }

    void abort() override;

private:
    static constexpr uint8_t kMaxOutputWrites = 2U;

    static void onOutputWriteQueued(void *context, uint8_t transactionId);
    static void onOutputWriteCompleted(void *context, uint8_t transactionId);

    void set();
    bool writesCompleted() const;

    // Hardware sandbox — exclusively owned by this environment.
    static IoExpanderService m_ioExpanderService;
    static Pca9535ExpanderChannel m_keypadExpanderChannel;

    static TimerExpireService m_timerExpireService;
    static Timer m_controlPanelPollTimer;

    static ControlPanelService m_controlPanelService;

    static LedChannel m_leds[];
    static const uint8_t kLedCount;

    uint8_t m_expectedWriteIds[kMaxOutputWrites] = {};
    uint8_t m_expectedWriteCount = 0U;
    uint8_t m_completedWriteMask = 0U;
    uint32_t m_changedLedCount = 0U;
    bool m_writeActive = false;
};

#endif /* DEBUG_ENVIRONMENT_LEDS_HPP */
