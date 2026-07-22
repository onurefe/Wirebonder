#include "lcd_controller_module.hpp"

LcdControllerModule::LcdControllerModule(Pca9538ExpanderChannel *expander, Timer *delayTimer)
    : m_expander(expander)
    , m_delayTimer(delayTimer)
    , m_cmdBuffer{}
    , m_cmdQueue(m_cmdBuffer, kCommandQueueDepth)
    , m_activeCommand{}
    , m_timerExpired(false)
    , m_lcdWriteCompleted(false)
    , m_channelWriteCounter(0U)
    , m_processorState(CommandProcessorState::kIdle)
{}

// ---------------------------------------------------------------------------
// start — enqueues the HD44780 power-on sequence
// ---------------------------------------------------------------------------
void LcdControllerModule::onStart()
{
    if (m_expander == nullptr || m_delayTimer == nullptr) {
        setProcessError();
        return;
    }
    m_expander->setTransferListenerCallbacks(
        this, nullptr, onWriteCompleted);
    m_delayTimer->setExpirationListenerCallback(this, onDelayExpired);

    enqueueDelay(Hd44780Constants::kPowerOnDelayMs);

    enqueueNibble(Hd44780Constants::kInitFuncSet8, Hd44780Constants::kFuncSet1DelayMs);
    enqueueNibble(Hd44780Constants::kInitFuncSet8, Hd44780Constants::kFuncSet2DelayMs);
    enqueueNibble(Hd44780Constants::kInitFuncSet8);
    enqueueNibble(Hd44780Constants::kInitSet4bit);

    enqueueByte(Hd44780Constants::kFunctionSet,  false);
    enqueueByte(Hd44780Constants::kDisplayCtrl,  false);
    enqueueByte(Hd44780Constants::kClearDisplay, false, Hd44780Constants::kClearDelayMs);
    enqueueByte(Hd44780Constants::kEntryModeSet, false);
    enqueueByte(static_cast<uint8_t>(Hd44780Constants::kDisplayCtrl | Hd44780Constants::kDisplayBit), false);

    m_processorState = CommandProcessorState::kIdle;
}

void LcdControllerModule::onStop()
{
    if (m_delayTimer != nullptr) m_delayTimer->stop();
    m_cmdQueue.clear();
    m_processorState = CommandProcessorState::kIdle;
    m_timerExpired = false;
    m_lcdWriteCompleted = false;
    m_channelWriteCounter = 0U;
    if (m_expander != nullptr) {
        m_expander->setTransferListenerCallbacks(nullptr, nullptr, nullptr);
    }
    if (m_delayTimer != nullptr) {
        m_delayTimer->setExpirationListenerCallback(nullptr, nullptr);
    }
}

bool LcdControllerModule::isIdle()
{
    if (!isOperating()) {
        return true;
    }

    return m_processorState == CommandProcessorState::kIdle && m_cmdQueue.isEmpty();
}

// ---------------------------------------------------------------------------
// execute — drives the state machine; call every main-loop iteration
// ---------------------------------------------------------------------------
void LcdControllerModule::onExecute()
{
    switch (m_processorState)
    {
    default:
    case CommandProcessorState::kIdle:
        idleStateHandler();
        break;
    
    case CommandProcessorState::kSendingBytes:
        sendingBytesStateHandler();
        break;

    case CommandProcessorState::kWaitingForDelay:
        waitingForDelayStateHandler();
        break;
    }   
}

void LcdControllerModule::idleStateHandler()
{
    if (!m_cmdQueue.isEmpty()) {
        m_activeCommand = m_cmdQueue.dequeue();
        m_timerExpired = false;
        m_lcdWriteCompleted = false;

        if ((m_activeCommand.type == CmdType::kDelayOnly) && (m_activeCommand.postDelayMs > 0U)) {
            m_processorState = CommandProcessorState::kWaitingForDelay;
            armPostDelay();
        } else if (m_activeCommand.type == CmdType::kNibble) {
            m_processorState = CommandProcessorState::kSendingBytes;
            m_channelWriteCounter = 0;
            sendNibble(m_activeCommand.value, m_activeCommand.isData);
        } else if (m_activeCommand.type == CmdType::kByte) {
            m_processorState = CommandProcessorState::kSendingBytes;
            m_channelWriteCounter = 0;
            sendByte(m_activeCommand.value, m_activeCommand.isData);
        }
    }
}

void LcdControllerModule::sendingBytesStateHandler()
{
    if (m_lcdWriteCompleted) {
        if (m_activeCommand.postDelayMs > 0) {
            m_processorState = CommandProcessorState::kWaitingForDelay;
            armPostDelay();
        } else {
            m_processorState = CommandProcessorState::kIdle;
        }

        m_lcdWriteCompleted = false;
    }
}

void LcdControllerModule::waitingForDelayStateHandler()
{
    if (m_timerExpired) {
        m_processorState = CommandProcessorState::kIdle;
        m_timerExpired = false;
    }
}

// ---------------------------------------------------------------------------
// Public LCD operations
// ---------------------------------------------------------------------------
void LcdControllerModule::clear()
{
    if (!isOperating()) return;
    enqueueByte(Hd44780Constants::kClearDisplay, false, Hd44780Constants::kClearDelayMs);
}

void LcdControllerModule::home()
{
    if (!isOperating()) return;
    enqueueByte(Hd44780Constants::kReturnHome, false, Hd44780Constants::kClearDelayMs);
}

void LcdControllerModule::setCursor(uint8_t col, uint8_t row)
{
    if (!isOperating()) return;
    uint8_t addr = static_cast<uint8_t>(
        Hd44780Constants::kDdramAddrBase | (Hd44780Constants::kRowAddr[row & 0x03U] + col));
    enqueueByte(addr, false);
}

void LcdControllerModule::printChar(char c)
{
    if (!isOperating()) return;
    enqueueByte(static_cast<uint8_t>(c), true);
}

void LcdControllerModule::printString(const char *str)
{
    if (!isOperating()) return;
    while (*str != '\0' && !m_cmdQueue.isFull()) {
        enqueueByte(static_cast<uint8_t>(*str++), true);
    }
}

void LcdControllerModule::setDisplay(bool displayOn, bool cursorOn, bool blinkOn)
{
    if (!isOperating()) return;
    uint8_t ctrl = static_cast<uint8_t>(
        Hd44780Constants::kDisplayCtrl
        | (displayOn ? Hd44780Constants::kDisplayBit : 0x00U)
        | (cursorOn  ? Hd44780Constants::kCursorBit  : 0x00U)
        | (blinkOn   ? Hd44780Constants::kBlinkBit   : 0x00U));
    enqueueByte(ctrl, false);
}

// ---------------------------------------------------------------------------
// Enqueue helpers
// ---------------------------------------------------------------------------
void LcdControllerModule::enqueueDelay(uint16_t ms)
{
    m_cmdQueue.enqueue({ CmdType::kDelayOnly, 0x00U, false, ms });
}

void LcdControllerModule::enqueueNibble(uint8_t nibble, uint16_t postDelayMs)
{
    m_cmdQueue.enqueue({ CmdType::kNibble, nibble, false, postDelayMs });
}

void LcdControllerModule::enqueueByte(uint8_t byte, bool isData, uint16_t postDelayMs)
{
    m_cmdQueue.enqueue({ CmdType::kByte, byte, isData, postDelayMs });
}

// ---------------------------------------------------------------------------
// Low-level write helpers
// ---------------------------------------------------------------------------
void LcdControllerModule::writeNibble(uint8_t nibble, bool isData)
{
    uint8_t base = static_cast<uint8_t>((nibble << 3) | (isData ? kPinRS : 0x00U));
    m_expander->setOutput(base);
    m_expander->setOutput(static_cast<uint8_t>(base | kPinEN));
    m_expander->setOutput(base);
}

void LcdControllerModule::sendNibble(uint8_t nibble, bool isData)
{
    writeNibble(nibble, isData);
}

void LcdControllerModule::sendByte(uint8_t byte, bool isData)
{
    writeNibble(static_cast<uint8_t>((byte >> 4) & 0x0FU), isData);
    writeNibble(static_cast<uint8_t>(byte        & 0x0FU), isData);
}

void LcdControllerModule::armPostDelay()
{
    m_delayTimer->start(true, m_activeCommand.postDelayMs / 1000.0f);
}

// ---------------------------------------------------------------------------
// ISR callbacks
// ---------------------------------------------------------------------------

// static — called from I2C ISR for each completed setOutput() write
void LcdControllerModule::onWriteCompleted(void *context)
{
    LcdControllerModule *self = static_cast<LcdControllerModule *>(context);

    if (self == nullptr || !self->isOperating()) {
        return;
    }
    
    self->m_channelWriteCounter++;
    uint8_t num_writes = self->m_activeCommand.type == CmdType::kNibble ? kWritesPerNibble:kWritesPerByte; 
    
    if (self->m_channelWriteCounter >= num_writes) {
        self->m_lcdWriteCompleted = true;
    }
}

// static — called from TimerExpireService ISR when the one-shot delay fires
void LcdControllerModule::onDelayExpired(void *context, Timer *timer)
{
    (void)timer;
    LcdControllerModule *self = static_cast<LcdControllerModule *>(context);
    if (self != nullptr && self->isOperating()) {
        self->m_timerExpired = true;
    }
}
