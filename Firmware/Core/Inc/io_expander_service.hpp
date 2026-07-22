#ifndef IO_EXPANDER_SERVICE_HPP
#define IO_EXPANDER_SERVICE_HPP

#include "stm32f4xx_hal.h"
#include "queue.hpp"
#include "configuration.h"
#include "generic.h"
#include "process.hpp"
#include <cstdint>

// ---------------------------------------------------------------------------
// Transaction
// ---------------------------------------------------------------------------
struct IoExpanderTransaction {
    uint8_t   id;
    uint8_t   deviceAddress;
    uint8_t   registerAddress;
    bool      isRead;
    uint16_t  length;
    uint8_t   data[4];
};

// ---------------------------------------------------------------------------
// IoExpanderChannel — interface every expander must implement to participate in
// the IoExpanderService bus scheduler.
// ---------------------------------------------------------------------------
class IoExpanderChannel {
public:
    virtual ~IoExpanderChannel() = default;

    // Called when the owning process starts — enqueues initial writes.
    virtual bool initializePorts() = 0;

    // Called while the owning process executes to detect pending work.
    virtual bool hasPendingTransaction() const = 0;

    // Dequeue and return the next transaction to execute.
    // Only called when hasPendingTransaction() is true.
    virtual IoExpanderTransaction getNextTransaction() = 0;
    virtual void ontransactionCompleted(IoExpanderTransaction &transaction) = 0;
};


// ---------------------------------------------------------------------------
// Pca9535ExpanderChannel — 16-bit I/O expander (two 8-bit ports)
// ---------------------------------------------------------------------------
class Pca9535ExpanderChannel : public IoExpanderChannel {
public:
    using InputChangedCallback = void (*)(void *context, uint8_t port0, uint8_t port1);
    using WriteCompletedCallback = void (*)(void *context, uint8_t transactionId);

    // deviceAddress    : 7-bit I2C deviceAddress (0x20–0x27, set by A0–A2 pins)
    // port0/1Dir : 1 = input pin, 0 = output pin
    Pca9535ExpanderChannel(uint8_t deviceAddress,
                    uint8_t port0Direction,
                    uint8_t port1Direction,
                    uint8_t port0InitialOutput = 0x00U,
                    uint8_t port1InitialOutput = 0x00U);

    bool readPortInputValues(void);
    bool setPortOutputValues(uint8_t port0, uint8_t port1);
    bool setPort0OutputValues(uint8_t value, uint8_t *transactionId = nullptr);
    bool setPort1OutputValues(uint8_t value, uint8_t *transactionId = nullptr);
    bool setPin(uint8_t port, uint8_t pin, bool value);

    void setTransferListenerCallbacks(void *context, 
        InputChangedCallback inputChangedCb, 
        WriteCompletedCallback writeCompletedCb);

    // IoExpanderChannel
    bool                  initializePorts()            override;
    bool                  hasPendingTransaction() const override;
    IoExpanderTransaction getNextTransaction()  override;
    void ontransactionCompleted(IoExpanderTransaction &transaction) override;

private:
    static constexpr uint8_t kQueueDepth    = 32U;
    static constexpr uint8_t kInputPort0Addr  = 0x00U;
    static constexpr uint8_t kOutputPort0Addr = 0x02U;
    static constexpr uint8_t kRegConfigPort0 = 0x06U;

    bool enqueueWrite(uint8_t registerAddress,
                      uint8_t *data,
                      uint16_t length,
                      uint8_t *transactionId = nullptr);
    bool enqueuePortRead();

    uint8_t  m_deviceAddress;
    uint8_t  m_port0Direction;
    uint8_t  m_port1Direction;
    uint8_t  m_outputPort0;
    uint8_t  m_outputPort1;
    uint8_t  m_inputPort0;
    uint8_t  m_inputPort1;

    uint8_t                         m_transactionEnumerator;

    IoExpanderTransaction           m_transactionQueueBuffer[kQueueDepth + 1U];
    Queue<IoExpanderTransaction>    m_transactionQueue;

    InputChangedCallback            m_inputChangedCallback;
    WriteCompletedCallback          m_writeCompletedCallback;
    void                            *m_callbackContext;
};

// ---------------------------------------------------------------------------
// Pca9538ExpanderChannel — 8-bit I/O expander (single port)
// ---------------------------------------------------------------------------
class Pca9538ExpanderChannel : public IoExpanderChannel {
public:
    using InputChangedCallback  = void (*)(void *context, uint8_t port);
    using WriteCompletedCallback = void (*)(void *context);

    // deviceAddress : 7-bit I2C address (0x70–0x73, set by A0–A1 pins)
    // direction     : 1 = input pin, 0 = output pin
    Pca9538ExpanderChannel(uint8_t deviceAddress,
                    uint8_t direction,
                    uint8_t initialOutput = 0x00U);

    bool readPortInputValues();
    bool setOutput(uint8_t value);
    bool setPin(uint8_t pin, bool value);

    void setTransferListenerCallbacks(void *context,
        InputChangedCallback  readCompletedCb,
        WriteCompletedCallback writeCompletedCb);

    // IoExpanderChannel
    bool                  initializePorts()                                   override;
    bool                  hasPendingTransaction()                       const  override;
    IoExpanderTransaction getNextTransaction()                                 override;
    void                  ontransactionCompleted(IoExpanderTransaction &transaction) override;

private:
    static constexpr uint8_t kQueueDepth     = 8U;
    static constexpr uint8_t kInputPortAddr  = 0x00U;
    static constexpr uint8_t kOutputPortAddr = 0x01U;
    static constexpr uint8_t kRegConfig      = 0x03U;

    bool enqueueWrite(uint8_t registerAddress, uint8_t *data, uint16_t length);
    bool enqueuePortRead();

    uint8_t m_deviceAddress;
    uint8_t m_direction;
    uint8_t m_output;
    uint8_t m_input;

    uint8_t                      m_transactionEnumerator;

    IoExpanderTransaction        m_transactionQueueBuffer[kQueueDepth + 1U];
    Queue<IoExpanderTransaction> m_transactionQueue;

    InputChangedCallback        m_inputChangedCallback;
    WriteCompletedCallback       m_writeCompletedCallback;
    void                        *m_callbackContext;
};

// ---------------------------------------------------------------------------
// IoExpanderService — I2C bus scheduler
// ---------------------------------------------------------------------------
class IoExpanderService : public Process {
public:
    IoExpanderService(I2C_HandleTypeDef *hi2c);

    bool addExpander(IoExpanderChannel *expander);

    bool isBusy() const { return m_busy; }

    void onI2cComplete(bool isRead);
    static IoExpanderService*   getServiceObject(I2C_HandleTypeDef *hi2c);

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    bool startReadTransaction(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length);
    bool startWriteTransaction(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length);
    void iterateExpanderPointer();

    static IoExpanderService    *g_services[IO_EXPANDER_SERVICE_MAX_HANDLES];
    static uint8_t              g_numServices;
    
    I2C_HandleTypeDef           *m_hi2c;
    IoExpanderChannel           *m_channels[IO_EXPANDER_MAX_CHANNELS_PER_SERVICE];
    uint8_t                     m_channelCount;
    uint8_t                     m_expanderPointer;

    IoExpanderTransaction       m_activeTransaction;
    volatile bool               m_busy;
};

#endif /* IO_EXPANDER_SERVICE_HPP */
