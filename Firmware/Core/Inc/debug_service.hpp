#ifndef DEBUG_SERVICE_HPP
#define DEBUG_SERVICE_HPP

#include <cstdint>
#include "configuration.h"
#include "generic.h"

// -----------------------------------------------------------------------------
// Debug command encoding
// -----------------------------------------------------------------------------
//
// 32-bit command word:
//
//   bits 31..16 : channel id
//   bits 15..0  : channel-local command id
//
// Concrete DebugChannel classes own their own channel ids.

#define DEBUG_COMMAND_NONE 0u

#define DEBUG_COMMAND_MAKE(channelId, localCommand) \
    ((((uint32_t)(channelId)) << 16) | ((uint32_t)(localCommand) & 0xFFFFu))

#define DEBUG_COMMAND_CHANNEL_ID(command) \
    ((uint16_t)(((command) >> 16) & 0xFFFFu))

#define DEBUG_COMMAND_LOCAL_ID(command) \
    ((uint16_t)((command) & 0xFFFFu))

// -----------------------------------------------------------------------------
// Service-level status/errors
// -----------------------------------------------------------------------------

enum DebugStatus : uint32_t {
    DEBUG_STATUS_IDLE = 0,
    DEBUG_STATUS_BUSY = 1,
    DEBUG_STATUS_DONE = 2,
    DEBUG_STATUS_ERROR = 3
};

enum DebugServiceError : uint32_t {
    DEBUG_SERVICE_ERROR_NONE = 0,
    DEBUG_SERVICE_ERROR_UNKNOWN_CHANNEL = 1,
    DEBUG_SERVICE_ERROR_DUPLICATE_CHANNEL = 2,
    DEBUG_SERVICE_ERROR_BUSY = 3,
    DEBUG_SERVICE_ERROR_TOO_MANY_CHANNELS = 4,
    DEBUG_SERVICE_ERROR_RESERVED_CHANNEL_ID = 5
};

// -----------------------------------------------------------------------------
// GDB-visible service block
// -----------------------------------------------------------------------------

struct DebugServiceBlock {
    // Written by debugger, cleared by service.
    volatile uint32_t command;

    // DebugStatus.
    volatile uint32_t status;

    // Service error, channel error, or command-specific result code.
    volatile uint32_t resultCode;

    // Command-specific result count.
    volatile uint32_t resultCount;

    // Command arguments.
    volatile float args[5];

    // Result buffers published by the active channel.
    //
    // Meaning is channel-specific:
    //   impedance: [0]=V phasors, [1]=I phasors, [2]=impedances
    //   PLL:       [0]=telemetry buffer
    //   tone:      [0]=ToneDebugResult
    //   keypad:    [0]=KeypadDebugResult
    //   force coil:[0]=current telemetry buffer
    volatile uintptr_t resultPointers[4];

    // Incremented whenever a command reaches a terminal state:
    //
    //   DONE or ERROR
    //
    // GDB can place a watchpoint on this field. Since this is updated after
    // status/resultCode/resultCount/resultPointers are prepared, the debugger
    // can safely read the result fields when the watchpoint fires.
    volatile uint32_t transactionCounter;
};

// -----------------------------------------------------------------------------
// DebugChannel
// -----------------------------------------------------------------------------

class DebugChannel {
public:
    using DependencyCallback = bool (*)(void *context, uint16_t localCommand);
    using DependencyReleaseCallback = void (*)(void *context, uint16_t localCommand);

    enum CommonError : uint32_t {
        ERROR_NONE = 0,
        ERROR_INVALID_ARGUMENT = 100,
        ERROR_NOT_INITIALIZED = 101,
        ERROR_UNSUPPORTED_COMMAND = 102
    };

    virtual ~DebugChannel() = default;

    void bindServiceBlock(DebugServiceBlock *serviceBlock);
    void setTelemetryBufferPtr(void *telemetryBufferPtr);
    void setDependencyCallback(void *context, DependencyCallback callback);
    void setDependencyReleaseCallback(void *context, DependencyReleaseCallback callback);

    virtual uint16_t channelId() const = 0;

    virtual bool canRunWhileBusy(uint16_t localCommand) const
    {
        (void)localCommand;
        return false;
    }

    virtual void handleCommand(uint16_t localCommand) = 0;

    virtual void poll()
    {
    }

    virtual bool isBusy() const
    {
        return false;
    }

    virtual void abort()
    {
    }

    bool requestDependencies(uint16_t localCommand);
    void releaseDependencies();

protected:
    float arg(uint8_t index) const;
    void setArg(uint8_t index, float value);
    void *telemetryBufferPtr() const;

    uint32_t status() const;

    void clearResultPointers();
    void setResultPointer(uint8_t index, const void *ptr);

    void setIdle();
    void setBusy();

    void setDone(uint32_t resultCode = ERROR_NONE,
                 uint32_t resultCount = 0);

    void setError(uint32_t errorCode);

private:
    DebugServiceBlock *m_serviceBlock = nullptr;
    void *m_telemetryBufferPtr = nullptr;
    void *m_dependencyCallbackContext = nullptr;
    DependencyCallback m_dependencyCallback = nullptr;
    void *m_dependencyReleaseCallbackContext = nullptr;
    DependencyReleaseCallback m_dependencyReleaseCallback = nullptr;
    uint16_t m_dependencyLocalCommand = 0u;
    bool m_dependenciesActive = false;
};

// -----------------------------------------------------------------------------
// DebugService
// -----------------------------------------------------------------------------

class DebugService {
public:
    static constexpr uint8_t MaxChannels = 12;

    bool addChannel(DebugChannel *channel);

    void startService();
    void stopService();
    void executeService();
    bool isOperating() const { return m_state == ServiceState::OPERATING; }

    DebugServiceBlock *serviceBlock()
    {
        return &m_debugServiceBlock;
    }

    const DebugServiceBlock *serviceBlock() const
    {
        return &m_debugServiceBlock;
    }

private:
    DebugChannel *findChannel(uint16_t channelId) const;

    bool canDispatch(DebugChannel *channel,
                     uint16_t localCommand) const;

    void refreshActiveChannel();
    void dispatchPendingCommand();
    void dispatchCommand(DebugChannel *channel, uint16_t localCommand);

    void setServiceError(uint32_t errorCode);

    static uint8_t s_telemetryBuffer[DEBUG_TELEMETRY_BUFFER_SIZE_BYTES];

    DebugServiceBlock m_debugServiceBlock = {};

    DebugChannel *m_channels[MaxChannels] = {};
    uint8_t m_channelCount = 0;

    DebugChannel *m_activeChannel = nullptr;
    DebugChannel *m_pendingChannel = nullptr;
    uint16_t m_pendingLocalCommand = 0u;
    ServiceState m_state = ServiceState::READY;
};

#endif /* DEBUG_SERVICE_HPP */
