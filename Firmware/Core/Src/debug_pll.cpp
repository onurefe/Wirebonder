#include "debug_pll.hpp"

DebugPll::DebugPll()
{
}

void DebugPll::init(PllModule *pll)
{
    m_pll = pll;

    if (m_pll != nullptr) {
        m_pll->addEventListenerCallback(this, &DebugPll::onPllEvent);
    }
}

void DebugPll::handleCommand(uint16_t localCommand)
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

void DebugPll::start()
{
    if (m_pll == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    m_telemetry = static_cast<PllModule::TelemetrySample *>(telemetryBufferPtr());
    if (m_telemetry == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    float centerFrequency = arg(0);
    float amplitude = arg(1);
    float bondingEnergy = arg(2);
    float maxDuration = arg(3);

    if (amplitude <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if (centerFrequency <= 0.0f) {
        centerFrequency = PLL_MODULE_CENTER_FREQUENCY;
    }

    if (bondingEnergy <= 0.0f) {
        bondingEnergy = DEBUG_PLL_DEFAULT_BONDING_ENERGY;
    }

    if (maxDuration <= 0.0f) {
        maxDuration = DEBUG_PLL_DEFAULT_MAX_DURATION;
    }

    m_busy = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    m_pll->setTelemetryBuffer(m_telemetry,
                              DEBUG_PLL_TELEMETRY_DEPTH);

    m_pll->start(centerFrequency,
                 amplitude,
                 bondingEnergy,
                 maxDuration);
}

void DebugPll::stop()
{
    if (m_pll != nullptr) {
        m_pll->stop();
    }

    m_busy = false;
    setIdle();
}

void DebugPll::abort()
{
    stop();
}

void DebugPll::onPllEvent(void *context, PllModule::Event event)
{
    auto *self = static_cast<DebugPll *>(context);

    if (self != nullptr) {
        self->handlePllEvent(event);
    }
}

void DebugPll::handlePllEvent(PllModule::Event event)
{
    if (!m_busy) {
        return;
    }

    m_busy = false;

    const uint32_t resultCode =
        event == PllModule::Event::BondingCompleted
            ? RESULT_BONDING_COMPLETED
            : RESULT_DURATION_TIMEOUT;

    const uint32_t resultCount =
        m_pll != nullptr ? m_pll->getTelemetryCount() : 0u;

    setDone(resultCode, resultCount);
}
