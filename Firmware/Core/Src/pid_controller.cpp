#include "pid_controller.hpp"

// -----------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------
PidController::PidController(const Config& config) 
    : m_config(config)
    , m_internalState(STATE_READY)
    , m_bypassEnabled(false)
    , m_prevFilteredError(0.0f)
{
    m_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    updateConfig(config); // Calculate alpha
}

void PidController::updateConfig(const Config& newConfig) {
    m_config = newConfig;
    
    // Recalculate Filter Alpha: alpha = dt / (Tf + dt)
    if (m_config.filterTc > 0.0f) {
        m_filterAlpha = m_config.dt / (m_config.filterTc + m_config.dt);
    } else {
        m_filterAlpha = 1.0f; // No filtering
    }
}

void PidController::enableBypass()
{
    m_bypassEnabled = true;
}

void PidController::disableBypass()
{
    m_bypassEnabled = false;
}

bool PidController::isBypassEnabled() const
{
    return m_bypassEnabled;
}

// -----------------------------------------------------------------------
// Control Flow
// -----------------------------------------------------------------------
void PidController::start(void) {
    if (m_internalState != STATE_READY) return;

    m_state.integral = 0.0f;
    m_state.errorFiltered = 0.0f;
    m_prevFilteredError = 0.0f;
    m_state.output = 0.0f;
    
    m_internalState = STATE_OPERATING;
}

void PidController::stop() {
    m_internalState = STATE_READY;
}

float PidController::execute(float setpoint, float measuredValue) {
    if (m_internalState != STATE_OPERATING) return 0.0f;

    if (m_bypassEnabled) {
        m_state.measured = measuredValue;
        m_state.errorRaw = 0.0f;
        m_state.errorFiltered = 0.0f;
        m_state.derivative = 0.0f;
        saturateOutput(setpoint);
        return m_state.output;
    }

    // 1. Calculate raw error from inputs
    calculateError(setpoint, measuredValue);

    // 2. Apply Low-Pass Filter to Error
    applyFilter();

    // 3. Calculate D term (Rate of change of error)
    calculateDerivative();

    // 4. Calculate I term (Accumulated error)
    updateIntegral();

    // 5. Combine P, I, D terms
    float rawOutput = computePidOutput();

    // 6. Clamp output to safe limits
    saturateOutput(rawOutput);

    // 7. Store history for next cycle
    updateHistory();

    return m_state.output;
}

// -----------------------------------------------------------------------
// Private Helpers (The Math)
// -----------------------------------------------------------------------
void PidController::calculateError(float setpoint, float measured) {
    m_state.measured = measured;
    m_state.errorRaw = setpoint - measured;
}

void PidController::applyFilter() {
    m_state.errorFiltered = (m_filterAlpha * m_state.errorRaw) + 
                            ((1.0f - m_filterAlpha) * m_prevFilteredError);
}

void PidController::calculateDerivative() {
    m_state.derivative = (m_state.errorFiltered - m_prevFilteredError) / m_config.dt;
}

void PidController::updateIntegral() {
    m_state.integral += m_state.errorFiltered * m_config.dt;
}

float PidController::computePidOutput() {
    float integralTerm = 0.0f;
    if (m_config.integralTc > 0.0f) {
        integralTerm = m_state.integral / m_config.integralTc;
    }
    float derivativeTerm = m_config.derivativeTc * m_state.derivative;

    return m_config.gain * (m_state.errorFiltered + integralTerm + derivativeTerm);
}

void PidController::saturateOutput(float rawOutput) {
    if (rawOutput > m_config.outputMax) {
        m_state.output = m_config.outputMax;
    } else if (rawOutput < m_config.outputMin) {
        m_state.output = m_config.outputMin;
    } else {
        m_state.output = rawOutput;
    }
}

void PidController::updateHistory() {
    m_prevFilteredError = m_state.errorFiltered;
}
