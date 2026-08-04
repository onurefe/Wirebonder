#ifndef DEBUG_ENVIRONMENT_HPP
#define DEBUG_ENVIRONMENT_HPP

#include <cstdint>
#include "configuration.h"
#include "generic.h"
#include "process.hpp"

// -----------------------------------------------------------------------------
// Debug command encoding
// -----------------------------------------------------------------------------
//
// 32-bit command word, written by the debugger:
//
//   bits 31..16 : environment id (DEBUG_ENVIRONMENT_ID_*)
//   bits 15..0  : environment-local command id
//
// Exactly one environment is compiled into a debug image. The id in the word
// is checked against it so a host script driving the wrong image gets a
// deterministic error instead of a silently misrouted command.

#define DEBUG_COMMAND_NONE 0u

#define DEBUG_COMMAND_MAKE(environmentId, localCommand) \
    ((((uint32_t)(environmentId)) << 16) | ((uint32_t)(localCommand) & 0xFFFFu))

#define DEBUG_COMMAND_ENVIRONMENT_ID(command) \
    ((uint16_t)(((command) >> 16) & 0xFFFFu))

#define DEBUG_COMMAND_LOCAL_ID(command) \
    ((uint16_t)((command) & 0xFFFFu))

enum DebugStatus : uint32_t {
    DEBUG_STATUS_IDLE = 0,
    DEBUG_STATUS_BUSY = 1,
    DEBUG_STATUS_DONE = 2,
    DEBUG_STATUS_ERROR = 3
};

// -----------------------------------------------------------------------------
// GDB-visible command block
// -----------------------------------------------------------------------------

struct DebugCommandBlock {
    // Written by debugger, cleared by the environment.
    volatile uint32_t command;

    // DebugStatus.
    volatile uint32_t status;

    // Common error, environment error, or command-specific result code.
    volatile uint32_t resultCode;

    // Command-specific result count.
    volatile uint32_t resultCount;

    // Command arguments.
    volatile float args[5];

    // Result buffers published by the environment; meaning is
    // environment-specific.
    volatile uintptr_t resultPointers[4];

    // Incremented whenever a command reaches a terminal state (DONE or
    // ERROR). Updated after status/resultCode/resultCount/resultPointers, so
    // a debugger watchpoint on this field can safely read the result fields
    // when it fires.
    volatile uint32_t transactionCounter;
};

// -----------------------------------------------------------------------------
// DebugEnvironment
// -----------------------------------------------------------------------------
//
// Base class for the self-contained debug sandboxes. A concrete environment
// constructs and exclusively owns every service/module it exercises,
// registers the long-lived ones with addProcess() (started in registration
// order, stopped in reverse), and implements handleCommand() for its
// debugger-driven operations. main.c boots exactly one environment through
// the App_* entry points (app.cpp) when FIRMWARE_MODE selects a debug image.

class DebugEnvironment {
public:
    enum CommonError : uint32_t {
        ERROR_NONE = 0,
        ERROR_INVALID_ARGUMENT = 100,
        ERROR_NOT_INITIALIZED = 101,
        ERROR_UNSUPPORTED_COMMAND = 102,
        ERROR_BUSY = 103,
        ERROR_WRONG_ENVIRONMENT = 104
    };

    void start();
    void execute();
    void stop();

protected:
    DebugEnvironment() = default;
    DebugEnvironment(const DebugEnvironment&) = delete;
    DebugEnvironment& operator=(const DebugEnvironment&) = delete;

    virtual uint16_t environmentId() const = 0;

    // Debugger-driven operations.
    virtual void handleCommand(uint16_t localCommand) = 0;

    virtual bool canRunWhileBusy(uint16_t localCommand) const
    {
        (void)localCommand;
        return false;
    }

    virtual bool isBusy() const
    {
        return false;
    }

    // Called every execute() tick after the owned processes have run.
    virtual void onPoll()
    {
    }

    // Lifecycle hooks around the owned processes' start/stop.
    virtual void onStart()
    {
    }

    virtual void onStop()
    {
    }

    // Cancels any in-flight operation; invoked from stop().
    virtual void abort()
    {
    }

    // Registers a long-lived component of the sandbox. Registration order is
    // start/execute order; stop runs in reverse.
    void addProcess(Process *process);

    // Mailbox helpers — same protocol as the debugger side expects.
    float arg(uint8_t index) const;
    void setArg(uint8_t index, float value);
    void *telemetryBuffer() const;

    uint32_t status() const;

    void clearResultPointers();
    void setResultPointer(uint8_t index, const void *ptr);

    void setIdle();
    void setBusy();

    void setDone(uint32_t resultCode = ERROR_NONE,
                 uint32_t resultCount = 0);

    void setError(uint32_t errorCode);

private:
    static constexpr uint8_t kMaxProcesses = 16;

    void resetCommandBlock();

    // GDB-visible state; static so host tooling can address it by symbol
    // regardless of which environment the image contains.
    static DebugCommandBlock s_commandBlock;
    static volatile uint32_t s_started;
    static uint8_t s_telemetryBuffer[DEBUG_TELEMETRY_BUFFER_SIZE_BYTES];

    Process *m_processes[kMaxProcesses] = {};
    uint8_t m_processCount = 0;
};

#endif /* DEBUG_ENVIRONMENT_HPP */
