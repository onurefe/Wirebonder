#include "debug_force_coil.hpp"

DebugForceCoil::DebugForceCoil()
{
}

void DebugForceCoil::init(ForceCoilDriverModule *forceCoil)
{
    m_forceCoil = forceCoil;

    if (m_forceCoil != nullptr) {
        m_forceCoil->addCurrentListenerCallback(
            this,
            &DebugForceCoil::onCurrentMeasured);
    }
}

void DebugForceCoil::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        start();
        break;

    case CMD_STOP:
        stop();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void DebugForceCoil::start()
{
    if (m_forceCoil == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    m_telemetry = static_cast<float *>(telemetryBufferPtr());
    if (m_telemetry == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    const float currentSetpoint = arg(0);
    const float duration = arg(1);
    const bool bypassPid = arg(2) > 0.0f;

    if (duration <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    uint32_t sampleLimit =
        static_cast<uint32_t>(
            duration *
            static_cast<float>(FORCE_COIL_MODULE_CONTROL_FREQUENCY));

    if (sampleLimit == 0u) {
        sampleLimit = 1u;
    }

    if (sampleLimit > DEBUG_FORCE_COIL_TELEMETRY_DEPTH) {
        sampleLimit = DEBUG_FORCE_COIL_TELEMETRY_DEPTH;
    }

    m_sampleIdx = 0;
    m_sampleLimit = sampleLimit;
    m_busy = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassPid) {
        m_forceCoil->enablePidBypass();
    } else {
        m_forceCoil->disablePidBypass();
    }

    m_forceCoil->start();
    m_forceCoil->setCurrentSetpoint(currentSetpoint);
}

void DebugForceCoil::stop()
{
    if (m_forceCoil != nullptr) {
        m_forceCoil->setCurrentSetpoint(0.0f);
        m_forceCoil->disablePidBypass();
        m_forceCoil->stop();
    }

    m_busy = false;
    setIdle();
}

void DebugForceCoil::abort()
{
    stop();
}

void DebugForceCoil::onCurrentMeasured(void *context, float measuredCurrent)
{
    auto *self = static_cast<DebugForceCoil *>(context);

    if (self != nullptr) {
        self->recordCurrent(measuredCurrent);
    }
}

void DebugForceCoil::recordCurrent(float measuredCurrent)
{
    if (!m_busy) {
        return;
    }

    if (m_sampleIdx >= m_sampleLimit) {
        return;
    }

    m_telemetry[m_sampleIdx++] = measuredCurrent;

    if (m_sampleIdx >= m_sampleLimit) {
        m_busy = false;

        if (m_forceCoil != nullptr) {
            m_forceCoil->setCurrentSetpoint(0.0f);
            m_forceCoil->disablePidBypass();
            m_forceCoil->stop();
        }

        setDone(RESULT_CAPTURE_COMPLETE, m_sampleIdx);
    }
}
