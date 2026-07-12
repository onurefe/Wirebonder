#include "debug_lcd.hpp"

DebugLcd::DebugLcd()
{
}

void DebugLcd::init(LcdModule *lcd)
{
    m_lcd = lcd;
}

void DebugLcd::handleCommand(uint16_t localCommand)
{
    if (m_lcd == nullptr || !m_lcd->isOperating()) {
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

void DebugLcd::poll()
{
    if (!m_busy) {
        return;
    }

    if (!m_lcd->isIdle()) {
        return;
    }

    m_busy = false;
    setDone(ERROR_NONE, m_writtenCharCount);
}

void DebugLcd::abort()
{
    m_busy = false;
}

void DebugLcd::writeLine()
{
    const float requestedRow = arg(0);
    const uint32_t row = static_cast<uint32_t>(requestedRow);

    if (requestedRow < 0.0f || row >= kRows) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    m_text[kColumns] = '\0';

    m_lcd->setCursor(0U, static_cast<uint8_t>(row));

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

        m_lcd->printChar(c);
    }

    m_writtenCharCount = written;
    m_busy = true;
    setBusy();
}

void DebugLcd::clear()
{
    m_lcd->clear();

    m_writtenCharCount = 0U;
    m_busy = true;
    setBusy();
}
