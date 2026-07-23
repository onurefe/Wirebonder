#include "force_coil_module.hpp"
#include <cmath>

/* ----------------- Public Functions ----------------- */
ForceCoilDriverModule::ForceCoilDriverModule(AnalogChannel *iSensChannel,
                                 PwmRampChannel *iDriveChannel)
    : m_iSensChannel(iSensChannel)
    , m_iDriveChannel(iDriveChannel)
    , m_pidCtrl(PidController::Config{
        FORCE_COIL_MODULE_PID_GAIN,
        FORCE_COIL_MODULE_PID_INTEGRAL_TC,
        FORCE_COIL_MODULE_PID_DERIVATIVE_TC,
        1.0f / static_cast<float>(FORCE_COIL_MODULE_CONTROL_FREQUENCY),
        FORCE_COIL_MODULE_PID_INPUT_FILTER_TC,
        FORCE_COIL_MODULE_PID_OUTPUT_MIN,
        FORCE_COIL_MODULE_PID_OUTPUT_MAX})
    , m_controlState(ControlState::Disabled)
    , m_callback(nullptr)
    , m_currentListenerCallbacks{}
    , m_currentListenerCallbackCount(0)
    , m_currentSetpoint(0.0f)
    , m_targetSetpoint(0.0f)
    , m_targetDuty(FORCE_COIL_MODULE_MIN_DUTY)
    , m_newSetpoint(false)
{}

void ForceCoilDriverModule::onStart()
{
    if (m_iDriveChannel == nullptr || m_iSensChannel == nullptr) {
        setProcessError();
        return;
    }
    if (!m_iDriveChannel->addTargetDutyControllerCallback(
            this, &ForceCoilDriverModule::iDriveCallback) ||
        !m_iSensChannel->addMeasurementListenerCallback(
            this, &ForceCoilDriverModule::iSensCallback)) {
        setProcessError();
    }
}

void ForceCoilDriverModule::onStop()
{
    disableControl();
}

bool ForceCoilDriverModule::enableControl()
{
    if (!isOperating() || m_controlState != ControlState::Disabled) {
        return false;
    }

    m_currentSetpoint = 0.0f;
    m_targetSetpoint = 0.0f;
    m_targetDuty = FORCE_COIL_MODULE_MIN_DUTY;
    m_newSetpoint = false;

    m_pidCtrl.start();
    m_controlState = ControlState::Enabled;
    if (!m_iDriveChannel->start(FORCE_COIL_MODULE_MIN_DUTY)) {
        m_pidCtrl.stop();
        m_controlState = ControlState::Disabled;
        return false;
    }
    return true;
}

void ForceCoilDriverModule::disableControl()
{
    if (m_controlState != ControlState::Enabled) return;

    m_iDriveChannel->stop();
    m_pidCtrl.stop();

    m_currentSetpoint = 0.0f;
    m_targetSetpoint = 0.0f;
    m_targetDuty = FORCE_COIL_MODULE_MIN_DUTY;
    m_newSetpoint = false;
    m_controlState = ControlState::Disabled;
}

bool ForceCoilDriverModule::isControlEnabled() const
{
    return m_controlState == ControlState::Enabled;
}

void ForceCoilDriverModule::setCurrentSetpoint(float currentSetpoint)
{
    if (!isControlEnabled()) {
        return;
    }

    m_targetSetpoint = currentSetpoint;
    m_newSetpoint = true;
}

void ForceCoilDriverModule::addEventListenerCallback(ForceCoilCallback callback)
{
    m_callback = callback;
}

bool ForceCoilDriverModule::addCurrentListenerCallback(void *context, CurrentListenerCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_currentListenerCallbackCount; i++) {
        if (m_currentListenerCallbacks[i].context == context &&
            m_currentListenerCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_currentListenerCallbackCount >= kMaxCurrentListenerCallbacks) {
        return false;
    }

    m_currentListenerCallbacks[m_currentListenerCallbackCount++] =
        CurrentListenerRegistration{callback, context};

    return true;
}

void ForceCoilDriverModule::enablePidBypass()
{
    m_pidCtrl.enableBypass();
}

void ForceCoilDriverModule::disablePidBypass()
{
    m_pidCtrl.disableBypass();
}

void ForceCoilDriverModule::onCurrentMeasured(float measuredCurrent)
{
    if (!isControlEnabled()) {
        return;
    }

    for (uint8_t i = 0; i < m_currentListenerCallbackCount; i++) {
        m_currentListenerCallbacks[i].callback(
            m_currentListenerCallbacks[i].context, measuredCurrent);
    }

    advanceSetpointRamp();
    m_targetDuty = m_pidCtrl.execute(m_currentSetpoint, measuredCurrent);

    if (m_newSetpoint) {
        // Compared against the ultimate target, not the ramped intermediate:
        // this only fires once the ramp has caught up and the PID has
        // settled there, same observable contract as before the ramp existed.
        float error = fabsf(measuredCurrent - m_targetSetpoint);
        if (error < FORCE_COIL_MODULE_CURRENT_ERROR_TOLERANCE) {
            m_newSetpoint = false;

            if (m_callback) {
                m_callback(Event::SetpointAchieved);
            }
        }
    }
}

void ForceCoilDriverModule::advanceSetpointRamp()
{
    // Slew-limits the setpoint the PID chases, rather than stepping it
    // directly: an instant reference step (e.g. dropping force right after
    // a weld) drives a step in commanded duty within a single control tick,
    // which can excite a transient overshoot/kick on the force-coil lever's
    // own mechanical resonance right when the freshly-welded wire is most
    // fragile. Ramping the reference removes that step without slowing down
    // the PID's own response to disturbances at the (slowly moving) setpoint.
    const float maxStep = FORCE_COIL_MODULE_SETPOINT_SLEW_RATE /
        static_cast<float>(FORCE_COIL_MODULE_CONTROL_FREQUENCY);

    const float delta = m_targetSetpoint - m_currentSetpoint;
    if (delta > maxStep) {
        m_currentSetpoint += maxStep;
    } else if (delta < -maxStep) {
        m_currentSetpoint -= maxStep;
    } else {
        m_currentSetpoint = m_targetSetpoint;
    }
}

bool ForceCoilDriverModule::onPwmUpdate(float *value)
{
    if (value == nullptr) {
        return false;
    }

    if (!isControlEnabled()) {
        return false;
    }

    // This module owns the force-coil PWM channel while operating.
    *value = m_targetDuty;
    return true;
}

void ForceCoilDriverModule::iSensCallback(void *context, float value)
{
    static_cast<ForceCoilDriverModule *>(context)->onCurrentMeasured(value);
}

bool ForceCoilDriverModule::iDriveCallback(void *context, float *value)
{
    return static_cast<ForceCoilDriverModule *>(context)->onPwmUpdate(value);
}
