#include "configuration.h"

#if FIRMWARE_MODE != FIRMWARE_MODE_NORMAL

#include "DebugEnvironment/debug_environment.hpp"

DebugCommandBlock DebugEnvironment::s_commandBlock = {};
volatile uint32_t DebugEnvironment::s_started = 0u;

alignas(8) uint8_t DebugEnvironment::s_telemetryBuffer[DEBUG_TELEMETRY_BUFFER_SIZE_BYTES] = {};

void DebugEnvironment::addProcess(Process *process)
{
    if (process != nullptr && m_processCount < kMaxProcesses) {
        m_processes[m_processCount++] = process;
    }
}

void DebugEnvironment::resetCommandBlock()
{
    s_commandBlock.command = DEBUG_COMMAND_NONE;
    s_commandBlock.status = DEBUG_STATUS_IDLE;
    s_commandBlock.resultCode = ERROR_NONE;
    s_commandBlock.resultCount = 0;

    for (uint8_t i = 0; i < 5; i++) {
        s_commandBlock.args[i] = 0.0f;
    }

    clearResultPointers();

    s_commandBlock.transactionCounter = 0u;
}

void DebugEnvironment::start()
{
    resetCommandBlock();

    for (uint8_t i = 0; i < m_processCount; i++) {
        m_processes[i]->start();
    }

    onStart();

    s_started = 1u;
}

void DebugEnvironment::execute()
{
    for (uint8_t i = 0; i < m_processCount; i++) {
        m_processes[i]->execute();
    }

    onPoll();

    const uint32_t rawCommand = s_commandBlock.command;
    if (rawCommand == DEBUG_COMMAND_NONE) {
        return;
    }

    s_commandBlock.command = DEBUG_COMMAND_NONE;

    if (DEBUG_COMMAND_ENVIRONMENT_ID(rawCommand) != environmentId()) {
        setError(ERROR_WRONG_ENVIRONMENT);
        return;
    }

    const uint16_t localCommand = DEBUG_COMMAND_LOCAL_ID(rawCommand);

    if (s_commandBlock.status == DEBUG_STATUS_BUSY &&
        !canRunWhileBusy(localCommand)) {
        setError(ERROR_BUSY);
        return;
    }

    handleCommand(localCommand);
}

void DebugEnvironment::stop()
{
    s_started = 0u;

    abort();
    onStop();

    for (uint8_t i = m_processCount; i > 0; i--) {
        m_processes[i - 1]->stop();
    }

    s_commandBlock.command = DEBUG_COMMAND_NONE;
    s_commandBlock.status = DEBUG_STATUS_IDLE;
}

float DebugEnvironment::arg(uint8_t index) const
{
    return s_commandBlock.args[index];
}

void DebugEnvironment::setArg(uint8_t index, float value)
{
    s_commandBlock.args[index] = value;
}

void *DebugEnvironment::telemetryBuffer() const
{
    return s_telemetryBuffer;
}

uint32_t DebugEnvironment::status() const
{
    return s_commandBlock.status;
}

void DebugEnvironment::clearResultPointers()
{
    for (uint8_t i = 0; i < 4; i++) {
        s_commandBlock.resultPointers[i] = 0u;
    }
}

void DebugEnvironment::setResultPointer(uint8_t index, const void *ptr)
{
    if (index < 4) {
        s_commandBlock.resultPointers[index] =
            reinterpret_cast<uintptr_t>(ptr);
    }
}

void DebugEnvironment::setIdle()
{
    s_commandBlock.status = DEBUG_STATUS_IDLE;
}

void DebugEnvironment::setBusy()
{
    s_commandBlock.resultCode = ERROR_NONE;
    s_commandBlock.resultCount = 0;
    clearResultPointers();

    s_commandBlock.status = DEBUG_STATUS_BUSY;
}

void DebugEnvironment::setDone(uint32_t resultCode, uint32_t resultCount)
{
    s_commandBlock.resultCode = resultCode;
    s_commandBlock.resultCount = resultCount;

    s_commandBlock.status = DEBUG_STATUS_DONE;

    // Important: keep this last.
    //
    // The debugger may watch transactionCounter. When it changes, all result
    // fields above should already be valid.
    s_commandBlock.transactionCounter++;
}

void DebugEnvironment::setError(uint32_t errorCode)
{
    s_commandBlock.resultCode = errorCode;
    s_commandBlock.resultCount = 0;
    clearResultPointers();

    s_commandBlock.status = DEBUG_STATUS_ERROR;

    // Important: keep this last.
    s_commandBlock.transactionCounter++;
}

#endif // FIRMWARE_MODE != FIRMWARE_MODE_NORMAL
