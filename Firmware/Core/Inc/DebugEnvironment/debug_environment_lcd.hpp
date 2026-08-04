#ifndef DEBUG_ENVIRONMENT_LCD_HPP
#define DEBUG_ENVIRONMENT_LCD_HPP

#include <cstdint>

#include "configuration.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "timer_expire_service.hpp"
#include "io_expander_service.hpp"
#include "lcd_controller_module.hpp"

// Sandbox for the character LCD behind its I2C expander.
class LcdDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_LCD;

    static constexpr uint8_t kRows = 4U;
    static constexpr uint8_t kColumns = 20U;

    enum Command : uint16_t {
        CMD_WRITE_LINE = 1,
        CMD_CLEAR = 2
    };

    LcdDebugEnvironment();

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
    void writeLine();
    void clear();

    // Hardware sandbox — exclusively owned by this environment.
    static IoExpanderService m_ioExpanderService;
    static Pca9538ExpanderChannel m_lcdExpanderChannel;

    static TimerExpireService m_timerExpireService;
    static Timer m_lcdDelayTimer;

    static LcdControllerModule m_lcdController;

    // Line text for CMD_WRITE_LINE. Filled in by the debug bridge (GDB writes
    // the string here before issuing the command); NUL-terminated. Static so
    // the host addresses it as LcdDebugEnvironment::m_text.
    static char m_text[kColumns + 1U];

    uint32_t m_writtenCharCount = 0U;
    bool m_writeActive = false;
};

#endif /* DEBUG_ENVIRONMENT_LCD_HPP */
