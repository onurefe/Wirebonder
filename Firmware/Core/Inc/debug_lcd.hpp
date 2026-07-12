#ifndef DEBUG_LCD_HPP
#define DEBUG_LCD_HPP

#include <cstdint>

#include "configuration.h"
#include "lcd_module.hpp"
#include "debug_service.hpp"

class DebugLcd : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_LCD;

    static constexpr uint8_t kRows = 4U;
    static constexpr uint8_t kColumns = 20U;

    enum Command : uint16_t {
        CMD_WRITE_LINE = 1,
        CMD_CLEAR = 2
    };

    DebugLcd();

    void init(LcdModule *lcd);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    void handleCommand(uint16_t localCommand) override;
    void poll() override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    void writeLine();
    void clear();

    LcdModule *m_lcd = nullptr;

    // Line text for CMD_WRITE_LINE. Filled in by the debug bridge (GDB writes
    // the string here before issuing the command); NUL-terminated.
    char m_text[kColumns + 1U] = {};

    uint32_t m_writtenCharCount = 0U;
    bool m_busy = false;
};

#endif /* DEBUG_LCD_HPP */
