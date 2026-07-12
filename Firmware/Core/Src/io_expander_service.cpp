#include "io_expander_service.hpp"
#include <cstring>

// =============================================================================
// IoExpanderService
// =============================================================================
IoExpanderService *IoExpanderService::g_services[IO_EXPANDER_SERVICE_MAX_HANDLES] = {nullptr};
uint8_t    IoExpanderService::g_numServices = 0;

IoExpanderService::IoExpanderService(I2C_HandleTypeDef *hi2c)
    : m_hi2c(hi2c)
    , m_channels{}
    , m_channelCount(0U)
    , m_expanderPointer(0U)
    , m_activeTransaction{}
    , m_busy(false)
    , m_state(ServiceState::READY)
{
    if (g_numServices < IO_EXPANDER_SERVICE_MAX_HANDLES) {
        g_services[g_numServices++] = this;
    }
}

bool IoExpanderService::addExpander(IoExpanderChannel *expander)
{
    if (expander == nullptr || m_channelCount >= IO_EXPANDER_MAX_CHANNELS_PER_SERVICE) {
        return false;
    }

    m_channels[m_channelCount++] = expander;
    return true;
}

void IoExpanderService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    for (uint8_t i = 0U; i < m_channelCount; ++i) {
        m_channels[i]->initializePorts();
    }

    m_state = ServiceState::OPERATING;
}

void IoExpanderService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    m_busy = false;
    m_state = ServiceState::READY;
}

void IoExpanderService::executeService()
{
    if (m_state != ServiceState::OPERATING || m_channelCount == 0U) {
        return;
    }

    if (m_busy) {
        return;
    }

    IoExpanderChannel *channel = m_channels[m_expanderPointer];

    if (channel->hasPendingTransaction()) {
        m_activeTransaction = channel->getNextTransaction();

        bool result;
        if (m_activeTransaction.isRead) {
            result = startReadTransaction(m_activeTransaction.deviceAddress, 
                m_activeTransaction.registerAddress, 
                m_activeTransaction.data, 
                m_activeTransaction.length);
        } else {
            result = startWriteTransaction(m_activeTransaction.deviceAddress, 
                m_activeTransaction.registerAddress, 
                m_activeTransaction.data, 
                m_activeTransaction.length);
        }

        if (result) {
            m_busy = true;
        }
    } else {
        iterateExpanderPointer();
    }
}

void IoExpanderService::iterateExpanderPointer(void)
{
    m_expanderPointer++;

        if (m_expanderPointer >= m_channelCount) {
            m_expanderPointer -= m_channelCount; 
        }
}

bool IoExpanderService::startReadTransaction(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read_IT(m_hi2c, 
                                static_cast<uint16_t>(deviceAddress), 
                                static_cast<uint16_t>(registerAddress), 
                                I2C_MEMADD_SIZE_8BIT, 
                                data, 
                                length);
    return status == HAL_OK;
}

bool IoExpanderService::startWriteTransaction(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write_IT(m_hi2c, 
                                static_cast<uint16_t>(deviceAddress), 
                                static_cast<uint16_t>(registerAddress), 
                                I2C_MEMADD_SIZE_8BIT, 
                                data, 
                                length);

    return status == HAL_OK;
}

IoExpanderService* IoExpanderService::getServiceObject(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < g_numServices; i++) {
        if (g_services[i]->m_state == ServiceState::OPERATING &&
            g_services[i]->m_hi2c->Instance == hi2c->Instance) {
            return g_services[i];
        }
    }
    return nullptr;
}

void IoExpanderService::onI2cComplete(bool isRead)
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    m_busy = false;
    m_channels[m_expanderPointer]->ontransactionCompleted(m_activeTransaction);
}

// =============================================================================
// Pca9535ExpanderChannel
// =============================================================================
Pca9535ExpanderChannel::Pca9535ExpanderChannel(uint8_t deviceAddress,
                                   uint8_t port0Direction,
                                   uint8_t port1Direction,
                                   uint8_t port0InitialOutput,
                                   uint8_t port1InitialOutput)
    : m_deviceAddress(static_cast<uint8_t>(deviceAddress << 1U))
    , m_port0Direction(port0Direction)
    , m_port1Direction(port1Direction)
    , m_outputPort0(port0InitialOutput)
    , m_outputPort1(port1InitialOutput)
    , m_inputPort0(0xFFU)
    , m_inputPort1(0xFFU)
    , m_transactionEnumerator(0U)
    , m_transactionQueueBuffer{}
    , m_transactionQueue(m_transactionQueueBuffer, kQueueDepth)
    , m_inputChangedCallback(nullptr)
    , m_writeCompletedCallback(nullptr)
    , m_callbackContext(nullptr)
{
}

bool Pca9535ExpanderChannel::initializePorts()
{
    uint8_t output_ports[2] = { m_outputPort0, m_outputPort1 };
    if (!enqueueWrite(kOutputPort0Addr, output_ports, 2U)) {
        return false;
    }

    uint8_t port_directions[2] = { m_port0Direction, m_port1Direction };
    if (!enqueueWrite(kRegConfigPort0, port_directions, 2U)) {
        return false;
    }

    return true;
}

bool Pca9535ExpanderChannel::hasPendingTransaction() const
{
    return !m_transactionQueue.isEmpty();
}

IoExpanderTransaction Pca9535ExpanderChannel::getNextTransaction()
{
    return m_transactionQueue.dequeue();
}

bool Pca9535ExpanderChannel::readPortInputValues(void)
{
    return enqueuePortRead();
}

bool Pca9535ExpanderChannel::setPortOutputValues(uint8_t port0, uint8_t port1)
{
    m_outputPort0 = port0;
    m_outputPort1 = port1;
    uint8_t data[2] = { port0, port1 };
    return enqueueWrite(kOutputPort0Addr, data, 2U);
}

bool Pca9535ExpanderChannel::setPort0OutputValues(uint8_t value, uint8_t *transactionId)
{
    m_outputPort0 = value;
    return enqueueWrite(kOutputPort0Addr, &m_outputPort0, 1U, transactionId);
}

bool Pca9535ExpanderChannel::setPort1OutputValues(uint8_t value, uint8_t *transactionId)
{
    m_outputPort1 = value;
    return enqueueWrite(kOutputPort0Addr + 1U, &m_outputPort1, 1U, transactionId);
}

bool Pca9535ExpanderChannel::setPin(uint8_t port, uint8_t pin, bool value)
{
    uint8_t &registerAddress  = (port == 0U) ? m_outputPort0 : m_outputPort1;
    uint8_t  addr = kOutputPort0Addr + port;

    if (value) registerAddress |=  static_cast<uint8_t>(1U << pin);
    else       registerAddress &= ~static_cast<uint8_t>(1U << pin);

    return enqueueWrite(addr, &registerAddress, 1U);
}

void Pca9535ExpanderChannel::setTransferListenerCallbacks(void *context, 
        InputChangedCallback readCompletedCb, 
        WriteCompletedCallback writeCompletedCb)
{
    m_callbackContext = context;
    m_inputChangedCallback   = readCompletedCb;
    m_writeCompletedCallback = writeCompletedCb;
}

bool Pca9535ExpanderChannel::enqueueWrite(uint8_t registerAddress,
                                           uint8_t *data,
                                           uint16_t length,
                                           uint8_t *transactionId)
{
    if (m_transactionQueue.isFull()) {
        return false;
    }

    IoExpanderTransaction transaction{};
    transaction.id              = m_transactionEnumerator;
    transaction.deviceAddress   = m_deviceAddress;
    transaction.registerAddress = registerAddress;
    transaction.isRead          = false;
    transaction.length          = length;

    std::memcpy(transaction.data, data, length);

    if (!m_transactionQueue.enqueue(transaction)) {
        return false;
    }

    if (transactionId != nullptr) {
        *transactionId = transaction.id;
    }

    m_transactionEnumerator = m_transactionEnumerator + 1U;
    return true;
}

bool Pca9535ExpanderChannel::enqueuePortRead()
{
    if (m_transactionQueue.isFull()) {
        return false;
    }

    IoExpanderTransaction       transaction{};
    transaction.id              = m_transactionEnumerator;
    transaction.deviceAddress   = m_deviceAddress;
    transaction.registerAddress = kInputPort0Addr;
    transaction.isRead          = true;
    transaction.length          = 2U;

    if (!m_transactionQueue.enqueue(transaction)) {
        return false;
    } 

    m_transactionEnumerator     = m_transactionEnumerator + 1U;

    return true;
}

void Pca9535ExpanderChannel::ontransactionCompleted(IoExpanderTransaction &transaction)
{
    if (transaction.isRead) {
        uint8_t new_port0 = transaction.data[0];
        uint8_t new_port1 = transaction.data[1];

        if (new_port0 == m_inputPort0 && new_port1 == m_inputPort1) return;

        m_inputPort0 = new_port0;
        m_inputPort1 = new_port1;

        if (m_inputChangedCallback) {
            m_inputChangedCallback(m_callbackContext, m_inputPort0, m_inputPort1);
        }
    } else {
        if (m_writeCompletedCallback) {
            m_writeCompletedCallback(m_callbackContext, transaction.id);
        }
    }
}

// =============================================================================
// Pca9538ExpanderChannel
// =============================================================================
Pca9538ExpanderChannel::Pca9538ExpanderChannel(uint8_t deviceAddress,
                                   uint8_t direction,
                                   uint8_t initialOutput)
    : m_deviceAddress(static_cast<uint8_t>(deviceAddress << 1U))
    , m_direction(direction)
    , m_output(initialOutput)
    , m_input(0xFFU)
    , m_transactionEnumerator(0U)
    , m_transactionQueueBuffer{}
    , m_transactionQueue(m_transactionQueueBuffer, kQueueDepth)
    , m_inputChangedCallback(nullptr)
    , m_writeCompletedCallback(nullptr)
    , m_callbackContext(nullptr)
{
}

bool Pca9538ExpanderChannel::initializePorts()
{
    if (!enqueueWrite(kOutputPortAddr, &m_output, 1U)) {
        return false;
    }

    if (!enqueueWrite(kRegConfig, &m_direction, 1U)) {
        return false;
    }

    return true;
}

bool Pca9538ExpanderChannel::hasPendingTransaction() const
{
    return !m_transactionQueue.isEmpty();
}

IoExpanderTransaction Pca9538ExpanderChannel::getNextTransaction()
{
    return m_transactionQueue.dequeue();
}

bool Pca9538ExpanderChannel::readPortInputValues()
{
    return enqueuePortRead();
}

bool Pca9538ExpanderChannel::setOutput(uint8_t value)
{
    m_output = value;
    return enqueueWrite(kOutputPortAddr, &m_output, 1U);
}

bool Pca9538ExpanderChannel::setPin(uint8_t pin, bool value)
{
    if (pin > 7U) return false;

    if (value) m_output |=  static_cast<uint8_t>(1U << pin);
    else       m_output &= ~static_cast<uint8_t>(1U << pin);

    return enqueueWrite(kOutputPortAddr, &m_output, 1U);
}

void Pca9538ExpanderChannel::setTransferListenerCallbacks(void *context,
        InputChangedCallback  readCompletedCb,
        WriteCompletedCallback writeCompletedCb)
{
    m_callbackContext        = context;
    m_inputChangedCallback  = readCompletedCb;
    m_writeCompletedCallback = writeCompletedCb;
}

bool Pca9538ExpanderChannel::enqueueWrite(uint8_t registerAddress, uint8_t *data, uint16_t length)
{
    if (m_transactionQueue.isFull()) {
        return false;
    }

    IoExpanderTransaction tx{};
    tx.id              = m_transactionEnumerator++;
    tx.deviceAddress   = m_deviceAddress;
    tx.registerAddress = registerAddress;
    tx.isRead          = false;
    tx.length          = length;
    std::memcpy(tx.data, data, length);

    return m_transactionQueue.enqueue(tx);
}

bool Pca9538ExpanderChannel::enqueuePortRead()
{
    if (m_transactionQueue.isFull()) {
        return false;
    }

    IoExpanderTransaction tx{};
    tx.id              = m_transactionEnumerator++;
    tx.deviceAddress   = m_deviceAddress;
    tx.registerAddress = kInputPortAddr;
    tx.isRead          = true;
    tx.length          = 1U;

    return m_transactionQueue.enqueue(tx);
}

void Pca9538ExpanderChannel::ontransactionCompleted(IoExpanderTransaction &transaction)
{
    if (transaction.isRead) {
        uint8_t newInput = transaction.data[0];

        if (newInput == m_input) return;

        m_input = newInput;

        if (m_inputChangedCallback) {
            m_inputChangedCallback(m_callbackContext, m_input);
        }
    } else {
        if (m_writeCompletedCallback) {
            m_writeCompletedCallback(m_callbackContext);
        }
    }
}

// =============================================================================
// HAL callbacks
// =============================================================================
extern "C" {
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        IoExpanderService *service = IoExpanderService::getServiceObject(hi2c);

        if (service) service->onI2cComplete(true);
    }

    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        IoExpanderService *service = IoExpanderService::getServiceObject(hi2c);

        if (service) service->onI2cComplete(false);
    }
}
