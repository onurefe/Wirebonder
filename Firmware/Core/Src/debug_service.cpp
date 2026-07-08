#include "debug_service.hpp"

alignas(8) uint8_t DebugService::s_telemetryBuffer[DEBUG_TELEMETRY_BUFFER_SIZE_BYTES] = {};

// -----------------------------------------------------------------------------
// DebugChannel
// -----------------------------------------------------------------------------

void DebugChannel::bindServiceBlock(DebugServiceBlock *serviceBlock)
{
    m_serviceBlock = serviceBlock;
}

void DebugChannel::setTelemetryBufferPtr(void *telemetryBufferPtr)
{
    m_telemetryBufferPtr = telemetryBufferPtr;
}

void DebugChannel::setDependencyCallback(void *context, DependencyCallback callback)
{
    m_dependencyCallbackContext = context;
    m_dependencyCallback = callback;
}

void DebugChannel::setDependencyReleaseCallback(void *context, DependencyReleaseCallback callback)
{
    m_dependencyReleaseCallbackContext = context;
    m_dependencyReleaseCallback = callback;
}

bool DebugChannel::requestDependencies(uint16_t localCommand)
{
    if (m_dependencyCallback != nullptr) {
        m_dependenciesActive = m_dependencyCallback(m_dependencyCallbackContext, localCommand);
        if (m_dependenciesActive) {
            m_dependencyLocalCommand = localCommand;
        }
        return m_dependenciesActive;
    }

    return false;
}

void DebugChannel::releaseDependencies()
{
    if (!m_dependenciesActive) {
        return;
    }

    if (m_dependencyReleaseCallback != nullptr) {
        m_dependencyReleaseCallback(m_dependencyReleaseCallbackContext,
                                    m_dependencyLocalCommand);
    }

    m_dependenciesActive = false;
    m_dependencyLocalCommand = 0u;
}

float DebugChannel::arg(uint8_t index) const
{
    return m_serviceBlock->args[index];
}

void DebugChannel::setArg(uint8_t index, float value)
{
    m_serviceBlock->args[index] = value;
}

void *DebugChannel::telemetryBufferPtr() const
{
    return m_telemetryBufferPtr;
}

uint32_t DebugChannel::status() const
{
    return m_serviceBlock->status;
}

void DebugChannel::clearResultPointers()
{
    for (uint8_t i = 0; i < 4; i++) {
        m_serviceBlock->resultPointers[i] = 0u;
    }
}

void DebugChannel::setResultPointer(uint8_t index, const void *ptr)
{
    if (index < 4) {
        m_serviceBlock->resultPointers[index] =
            reinterpret_cast<uintptr_t>(ptr);
    }
}

void DebugChannel::setIdle()
{
    m_serviceBlock->status = DEBUG_STATUS_IDLE;
}

void DebugChannel::setBusy()
{
    m_serviceBlock->resultCode = ERROR_NONE;
    m_serviceBlock->resultCount = 0;
    clearResultPointers();

    m_serviceBlock->status = DEBUG_STATUS_BUSY;
}

void DebugChannel::setDone(uint32_t resultCode, uint32_t resultCount)
{
    m_serviceBlock->resultCode = resultCode;
    m_serviceBlock->resultCount = resultCount;

    m_serviceBlock->status = DEBUG_STATUS_DONE;

    // Important: keep this last.
    //
    // The debugger may watch transactionCounter. When it changes, all result
    // fields above should already be valid.
    m_serviceBlock->transactionCounter++;
}

void DebugChannel::setError(uint32_t errorCode)
{
    m_serviceBlock->resultCode = errorCode;
    m_serviceBlock->resultCount = 0;
    clearResultPointers();

    m_serviceBlock->status = DEBUG_STATUS_ERROR;

    // Important: keep this last.
    m_serviceBlock->transactionCounter++;
}

// -----------------------------------------------------------------------------
// DebugService
// -----------------------------------------------------------------------------

void DebugService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    m_debugServiceBlock.command = DEBUG_COMMAND_NONE;
    m_debugServiceBlock.status = DEBUG_STATUS_IDLE;
    m_debugServiceBlock.resultCode = DEBUG_SERVICE_ERROR_NONE;
    m_debugServiceBlock.resultCount = 0;

    for (uint8_t i = 0; i < 5; i++) {
        m_debugServiceBlock.args[i] = 0.0f;
    }

    for (uint8_t i = 0; i < 4; i++) {
        m_debugServiceBlock.resultPointers[i] = 0u;
    }

    m_debugServiceBlock.transactionCounter = 0u;

    m_activeChannel = nullptr;
    m_pendingChannel = nullptr;
    m_pendingLocalCommand = 0u;
    m_state = ServiceState::OPERATING;
}

void DebugService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_channelCount; i++) {
        DebugChannel *channel = m_channels[i];

        if (channel != nullptr) {
            channel->abort();
            channel->releaseDependencies();
        }
    }

    m_debugServiceBlock.command = DEBUG_COMMAND_NONE;
    m_debugServiceBlock.status = DEBUG_STATUS_IDLE;

    m_activeChannel = nullptr;
    m_pendingChannel = nullptr;
    m_pendingLocalCommand = 0u;
    m_state = ServiceState::READY;
}

void DebugService::setServiceError(uint32_t errorCode)
{
    m_debugServiceBlock.resultCode = errorCode;
    m_debugServiceBlock.resultCount = 0;

    for (uint8_t i = 0; i < 4; i++) {
        m_debugServiceBlock.resultPointers[i] = 0u;
    }

    m_debugServiceBlock.status = DEBUG_STATUS_ERROR;

    // Important: keep this last.
    m_debugServiceBlock.transactionCounter++;
}

bool DebugService::addChannel(DebugChannel *channel)
{
    if (channel == nullptr) {
        return false;
    }

    if (channel->channelId() == 0u) {
        setServiceError(DEBUG_SERVICE_ERROR_RESERVED_CHANNEL_ID);
        return false;
    }

    if (m_channelCount >= MaxChannels) {
        setServiceError(DEBUG_SERVICE_ERROR_TOO_MANY_CHANNELS);
        return false;
    }

    if (findChannel(channel->channelId()) != nullptr) {
        setServiceError(DEBUG_SERVICE_ERROR_DUPLICATE_CHANNEL);
        return false;
    }

    channel->bindServiceBlock(&m_debugServiceBlock);
    channel->setTelemetryBufferPtr(s_telemetryBuffer);

    m_channels[m_channelCount] = channel;
    m_channelCount++;

    return true;
}

DebugChannel *DebugService::findChannel(uint16_t channelId) const
{
    for (uint8_t i = 0; i < m_channelCount; i++) {
        DebugChannel *channel = m_channels[i];

        if (channel != nullptr && channel->channelId() == channelId) {
            return channel;
        }
    }

    return nullptr;
}

void DebugService::refreshActiveChannel()
{
    if (m_activeChannel != nullptr && !m_activeChannel->isBusy()) {
        m_activeChannel->releaseDependencies();
        m_activeChannel = nullptr;
    }
}

void DebugService::dispatchPendingCommand()
{
    if (m_pendingChannel == nullptr) {
        return;
    }

    DebugChannel *channel = m_pendingChannel;
    const uint16_t localCommand = m_pendingLocalCommand;

    m_pendingChannel = nullptr;
    m_pendingLocalCommand = 0u;

    dispatchCommand(channel, localCommand);
}

void DebugService::dispatchCommand(DebugChannel *channel, uint16_t localCommand)
{
    const bool wasActiveChannel = (m_activeChannel == channel);

    channel->handleCommand(localCommand);

    if (channel->isBusy()) {
        m_activeChannel = channel;
    } else if (wasActiveChannel || m_activeChannel == channel) {
        channel->releaseDependencies();
        m_activeChannel = nullptr;
    } else {
        channel->releaseDependencies();
        m_activeChannel = nullptr;
    }
}

bool DebugService::canDispatch(DebugChannel *channel,
                               uint16_t localCommand) const
{
    if (m_pendingChannel != nullptr) {
        return false;
    }

    if (m_debugServiceBlock.status != DEBUG_STATUS_BUSY) {
        return true;
    }

    if (channel == nullptr) {
        return false;
    }

    return channel->canRunWhileBusy(localCommand);
}

void DebugService::executeService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_channelCount; i++) {
        DebugChannel *channel = m_channels[i];

        if (channel != nullptr) {
            channel->poll();
        }
    }

    refreshActiveChannel();
    dispatchPendingCommand();

    const uint32_t rawCommand = m_debugServiceBlock.command;
    if (rawCommand == DEBUG_COMMAND_NONE) {
        return;
    }

    m_debugServiceBlock.command = DEBUG_COMMAND_NONE;

    const uint16_t channelId = DEBUG_COMMAND_CHANNEL_ID(rawCommand);
    const uint16_t localCommand = DEBUG_COMMAND_LOCAL_ID(rawCommand);

    DebugChannel *channel = findChannel(channelId);
    if (channel == nullptr) {
        setServiceError(DEBUG_SERVICE_ERROR_UNKNOWN_CHANNEL);
        return;
    }

    if (!canDispatch(channel, localCommand)) {
        setServiceError(DEBUG_SERVICE_ERROR_BUSY);
        return;
    }

    if (channel->requestDependencies(localCommand)) {
        m_pendingChannel = channel;
        m_pendingLocalCommand = localCommand;
        return;
    }

    dispatchCommand(channel, localCommand);
}
