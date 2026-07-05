#ifndef LCD_MODULE_HPP
#define LCD_MODULE_HPP

#include "io_expander_service.hpp"
#include "timer_expire_service.hpp"
#include "queue.hpp"
#include "generic.h"
#include <cstdint>

// HD44780-compatible LCD driver in 4-bit mode over PCA9538.
//
// PCA9538 wiring: P0=RS  P1=RW  P2=EN  P3=D4  P4=D5  P5=D6  P6=D7
//
// delayTimer must be registered with TimerExpireService before start() is called.
//
// Usage:
//   Timer     lcdTimer(false, 0.001f);   // period overridden per-command via start()
//   LcdModule lcd(&myPca9538, &lcdTimer);
//   timerService.addTimer(&lcdTimer);
//   lcd.start();    // enqueues the HD44780 init sequence; call before the main loop
//   // inside main loop:
//   lcd.execute();  // drives state machine
class LcdModule {
public:
    LcdModule(Pca9538ExpanderChannel *expander, Timer *delayTimer);

    // Enqueues the full HD44780 power-on sequence. Call once before the main loop.
    void start();
    void stop();

    // Drives the internal state machine. Call every main-loop iteration.
    void execute();
    bool isOperating() const { return m_state == ServiceState::OPERATING; }

    bool isIdle();

    void clear();
    void home();
    void setCursor(uint8_t col, uint8_t row);
    void printChar(char c);
    void printString(const char *str);
    void setDisplay(bool displayOn, bool cursorOn, bool blinkOn);

private:
    enum class CmdType : uint8_t { kDelayOnly, kNibble, kByte };

    struct LcdCommand {
        CmdType  type;
        uint8_t  value;
        bool     isData;
        uint16_t postDelayMs;
    };

    enum class CommandProcessorState : uint8_t { kIdle, kSendingBytes, kWaitingForDelay };

    static constexpr uint8_t kCommandQueueDepth = 96U;
    static constexpr uint8_t kPinRS             = 0x01U;  // P0
    static constexpr uint8_t kPinEN             = 0x04U;  // P2
    static constexpr uint8_t kWritesPerByte     = 6U;     // 2 nibbles × 3 setOutput() calls
    static constexpr uint8_t kWritesPerNibble   = 3U;

    struct Hd44780Constants {
        static constexpr uint8_t  kClearDisplay  = 0x01U;
        static constexpr uint8_t  kReturnHome    = 0x02U;
        static constexpr uint8_t  kEntryModeSet  = 0x06U;
        static constexpr uint8_t  kDisplayCtrl   = 0x08U;
        static constexpr uint8_t  kFunctionSet   = 0x28U;
        static constexpr uint8_t  kDdramAddrBase = 0x80U;

        static constexpr uint8_t  kInitFuncSet8  = 0x03U;
        static constexpr uint8_t  kInitSet4bit   = 0x02U;

        static constexpr uint8_t  kDisplayBit    = 0x04U;
        static constexpr uint8_t  kCursorBit     = 0x02U;
        static constexpr uint8_t  kBlinkBit      = 0x01U;

        static constexpr uint16_t kPowerOnDelayMs  = 50U;
        static constexpr uint16_t kFuncSet1DelayMs =  5U;
        static constexpr uint16_t kFuncSet2DelayMs =  1U;
        static constexpr uint16_t kClearDelayMs    =  2U;

        static constexpr uint8_t  kRowAddr[4] = { 0x00U, 0x40U, 0x14U, 0x54U };
    };

    void idleStateHandler();
    void sendingBytesStateHandler();
    void waitingForDelayStateHandler();

    void enqueueDelay(uint16_t ms);
    void enqueueNibble(uint8_t nibble, uint16_t postDelayMs = 0U);
    void enqueueByte(uint8_t byte, bool isData, uint16_t postDelayMs = 0U);

    void writeNibble(uint8_t nibble, bool isData);
    void sendNibble(uint8_t nibble, bool isData);
    void sendByte(uint8_t byte, bool isData);

    void armPostDelay();

    static void onWriteCompleted(void *context);
    static void onDelayExpired(void *context, Timer *timer);

    Pca9538ExpanderChannel   *m_expander;
    Timer             *m_delayTimer;

    LcdCommand        m_cmdBuffer[kCommandQueueDepth + 1U];
    Queue<LcdCommand> m_cmdQueue;

    LcdCommand                      m_activeCommand;
    volatile bool                   m_timerExpired;
    volatile bool                   m_lcdWriteCompleted;
    volatile uint8_t                m_channelWriteCounter;
    volatile CommandProcessorState  m_processorState;
    ServiceState                    m_state;
};

#endif /* LCD_MODULE_HPP */
