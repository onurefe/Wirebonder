#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LCD

#include "DebugEnvironment/debug_environment_lcd.hpp"

extern I2C_HandleTypeDef hi2c1;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the LCD, but
// owned here outright.
// -----------------------------------------------------------------------------

IoExpanderService LcdDebugEnvironment::m_ioExpanderService(&hi2c1);

Pca9538ExpanderChannel LcdDebugEnvironment::m_lcdExpanderChannel(
    LCD_EXPANDER_I2C_ADDRESS,
    LCD_EXPANDER_DIRECTION);

TimerExpireService LcdDebugEnvironment::m_timerExpireService;
Timer LcdDebugEnvironment::m_lcdDelayTimer;

LcdControllerModule LcdDebugEnvironment::m_lcdController(
    &LcdDebugEnvironment::m_lcdExpanderChannel,
    &LcdDebugEnvironment::m_lcdDelayTimer);

char LcdDebugEnvironment::m_text[LcdDebugEnvironment::kColumns + 1U] = {};

LcdDebugEnvironment::LcdDebugEnvironment()
{
    m_timerExpireService.addTimer(&m_lcdDelayTimer, false);

    m_ioExpanderService.addExpander(&m_lcdExpanderChannel);

    addProcess(&m_timerExpireService);
    addProcess(&m_ioExpanderService);
    addProcess(&m_lcdController);
}

void LcdDebugEnvironment::handleCommand(uint16_t localCommand)
{
    if (!m_lcdController.isOperating()) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    switch (localCommand) {
    case CMD_WRITE_LINE:
        writeLine();
        break;
    case CMD_CLEAR:
        clear();
        break;
    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void LcdDebugEnvironment::onPoll()
{
    if (!m_writeActive) {
        return;
    }

    if (!m_lcdController.isIdle()) {
        return;
    }

    m_writeActive = false;
    setDone(ERROR_NONE, m_writtenCharCount);
}

void LcdDebugEnvironment::abort()
{
    m_writeActive = false;
}

void LcdDebugEnvironment::writeLine()
{
    const float requestedRow = arg(0);
    const uint32_t row = static_cast<uint32_t>(requestedRow);

    if (requestedRow < 0.0f || row >= kRows) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_text[kColumns] = '\0';

    m_lcdController.setCursor(0U, static_cast<uint8_t>(row));

    // Pad with spaces so the whole line is overwritten.
    bool terminated = false;
    uint32_t written = 0U;
    for (uint8_t col = 0U; col < kColumns; ++col) {
        char c = m_text[col];

        if (c == '\0') {
            terminated = true;
        }

        if (terminated) {
            c = ' ';
        } else {
            written++;
        }

        m_lcdController.printChar(c);
    }

    m_writtenCharCount = written;
    m_writeActive = true;
    setBusy();
}

void LcdDebugEnvironment::clear()
{
    m_lcdController.clear();

    m_writtenCharCount = 0U;
    m_writeActive = true;
    setBusy();
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LCD
