#include "lcd_module.hpp"

LcdModule::LcdModule(Pca9538ExpanderChannel *expander, Timer *delayTimer)
    : m_expander(expander)
    , m_delayTimer(delayTimer)
    , m_cmdBuffer{}
    , m_cmdQueue(m_cmdBuffer, kCommandQueueDepth)
    , m_activeCommand{}
    , m_timerExpired(false)
    , m_lcdWriteCompleted(false)
    , m_channelWriteCounter(0U)
    , m_processorState(CommandProcessorState::kIdle)
    , m_state(ServiceState::READY)
{
    m_expander->setTransferListenerCallbacks(this, nullptr, onWriteCompleted);
    m_delayTimer->setExpirationListenerCallback(this, onDelayExpired);
}

// ---------------------------------------------------------------------------
// start — enqueues the HD44780 power-on sequence
// ---------------------------------------------------------------------------
void LcdModule::start()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    m_state = ServiceState::OPERATING;

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

void LcdModule::stop()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    m_delayTimer->stop();
    m_cmdQueue.clear();
    m_processorState = CommandProcessorState::kIdle;
    m_timerExpired = false;
    m_lcdWriteCompleted = false;
    m_channelWriteCounter = 0U;
    m_state = ServiceState::READY;
}

bool LcdModule::isIdle()
{
    if (m_state != ServiceState::OPERATING) {
        return true;
    }

    return m_processorState == CommandProcessorState::kIdle && m_cmdQueue.isEmpty();
}

// ---------------------------------------------------------------------------
// execute — drives the state machine; call every main-loop iteration
// ---------------------------------------------------------------------------
void LcdModule::execute()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

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

void LcdModule::idleStateHandler()
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

void LcdModule::sendingBytesStateHandler()
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

void LcdModule::waitingForDelayStateHandler()
{
    if (m_timerExpired) {
        m_processorState = CommandProcessorState::kIdle;
        m_timerExpired = false;
    }
}

// ---------------------------------------------------------------------------
// Public LCD operations
// ---------------------------------------------------------------------------
void LcdModule::clear()
{
    if (m_state != ServiceState::OPERATING) return;
    enqueueByte(Hd44780Constants::kClearDisplay, false, Hd44780Constants::kClearDelayMs);
}

void LcdModule::home()
{
    if (m_state != ServiceState::OPERATING) return;
    enqueueByte(Hd44780Constants::kReturnHome, false, Hd44780Constants::kClearDelayMs);
}

void LcdModule::setCursor(uint8_t col, uint8_t row)
{
    if (m_state != ServiceState::OPERATING) return;
    uint8_t addr = static_cast<uint8_t>(
        Hd44780Constants::kDdramAddrBase | (Hd44780Constants::kRowAddr[row & 0x03U] + col));
    enqueueByte(addr, false);
}

void LcdModule::printChar(char c)
{
    if (m_state != ServiceState::OPERATING) return;
    enqueueByte(static_cast<uint8_t>(c), true);
}

void LcdModule::printString(const char *str)
{
    if (m_state != ServiceState::OPERATING) return;
    while (*str != '\0' && !m_cmdQueue.isFull()) {
        enqueueByte(static_cast<uint8_t>(*str++), true);
    }
}

void LcdModule::setDisplay(bool displayOn, bool cursorOn, bool blinkOn)
{
    if (m_state != ServiceState::OPERATING) return;
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
void LcdModule::enqueueDelay(uint16_t ms)
{
    m_cmdQueue.enqueue({ CmdType::kDelayOnly, 0x00U, false, ms });
}

void LcdModule::enqueueNibble(uint8_t nibble, uint16_t postDelayMs)
{
    m_cmdQueue.enqueue({ CmdType::kNibble, nibble, false, postDelayMs });
}

void LcdModule::enqueueByte(uint8_t byte, bool isData, uint16_t postDelayMs)
{
    m_cmdQueue.enqueue({ CmdType::kByte, byte, isData, postDelayMs });
}

// ---------------------------------------------------------------------------
// Low-level write helpers
// ---------------------------------------------------------------------------
void LcdModule::writeNibble(uint8_t nibble, bool isData)
{
    uint8_t base = static_cast<uint8_t>((nibble << 3) | (isData ? kPinRS : 0x00U));
    m_expander->setOutput(base);
    m_expander->setOutput(static_cast<uint8_t>(base | kPinEN));
    m_expander->setOutput(base);
}

void LcdModule::sendNibble(uint8_t nibble, bool isData)
{
    writeNibble(nibble, isData);
}

void LcdModule::sendByte(uint8_t byte, bool isData)
{
    writeNibble(static_cast<uint8_t>((byte >> 4) & 0x0FU), isData);
    writeNibble(static_cast<uint8_t>(byte        & 0x0FU), isData);
}

void LcdModule::armPostDelay()
{
    m_delayTimer->start(true, m_activeCommand.postDelayMs / 1000.0f);
}

// ---------------------------------------------------------------------------
// ISR callbacks
// ---------------------------------------------------------------------------

// static — called from I2C ISR for each completed setOutput() write
void LcdModule::onWriteCompleted(void *context)
{
    LcdModule *self = static_cast<LcdModule *>(context);

    if (self == nullptr || self->m_state != ServiceState::OPERATING) {
        return;
    }
    
    self->m_channelWriteCounter++;
    uint8_t num_writes = self->m_activeCommand.type == CmdType::kNibble ? kWritesPerNibble:kWritesPerByte; 
    
    if (self->m_channelWriteCounter >= num_writes) {
        self->m_lcdWriteCompleted = true;
    }
}

// static — called from TimerExpireService ISR when the one-shot delay fires
void LcdModule::onDelayExpired(void *context, Timer *timer)
{
    (void)timer;
    LcdModule *self = static_cast<LcdModule *>(context);
    if (self != nullptr && self->m_state == ServiceState::OPERATING) {
        self->m_timerExpired = true;
    }
}
