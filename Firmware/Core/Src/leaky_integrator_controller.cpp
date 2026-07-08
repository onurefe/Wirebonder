#include "leaky_integrator_controller.hpp"

LeakyIntegratorController::LeakyIntegratorController(const Config& config)
    : m_config(config)
    , m_capacitorVoltage(0.0f)
    , m_internalState(STATE_READY)
    , m_bypassEnabled(false)
{
}

void LeakyIntegratorController::start()
{
    if (m_internalState != STATE_READY) {
        return;
    }

    m_capacitorVoltage = 0.0f;
    m_internalState = STATE_OPERATING;
}

float LeakyIntegratorController::execute(float setpoint, float measuredValue)
{
    if (m_internalState != STATE_OPERATING) {
        return 0.0f;
    }

    if (m_bypassEnabled) {
        return clampOutput(setpoint);
    }

    if (m_config.Ri <= 0.0f ||
        m_config.Rf <= 0.0f ||
        m_config.Cf <= 0.0f ||
        m_config.dt <= 0.0f) {
        return 0.0f;
    }

    /* Update the capacitor voltage. */
    const float error = (setpoint - measuredValue) * m_config.preamplifierGain;
    const float leakage_current = m_capacitorVoltage / m_config.Rf;
    const float input_current = error / m_config.Ri;
    const float capacitor_current = input_current - leakage_current;

    const float raw_capacitor_voltage =
        m_capacitorVoltage + capacitor_current * m_config.dt / m_config.Cf;
    
    m_capacitorVoltage = clampOutput(raw_capacitor_voltage);

    return m_capacitorVoltage;
}

void LeakyIntegratorController::stop()
{
    m_internalState = STATE_READY;
}

void LeakyIntegratorController::updateConfig(const Config& newConfig)
{
    m_config = newConfig;
}

void LeakyIntegratorController::enableBypass()
{
    m_bypassEnabled = true;
}

void LeakyIntegratorController::disableBypass()
{
    m_bypassEnabled = false;
}

bool LeakyIntegratorController::isBypassEnabled() const
{
    return m_bypassEnabled;
}

float LeakyIntegratorController::clampOutput(float rawOutput) const
{
    if (rawOutput > m_config.outputMax) {
        return m_config.outputMax;
    }

    if (rawOutput < m_config.outputMin) {
        return m_config.outputMin;
    }

    return rawOutput;
}
