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
    , m_state(STATE_READY)
    , m_callback(nullptr)
    , m_currentListenerCallbacks{}
    , m_currentListenerCallbackCount(0)
    , m_currentSetpoint(0.0f)
    , m_targetDuty(FORCE_COIL_MODULE_MIN_DUTY)
    , m_newSetpoint(false)
{
    m_iDriveChannel->addTargetDutyControllerCallback(this, &ForceCoilDriverModule::iDriveCallback);
    m_iSensChannel->addMeasurementListenerCallback(this, &ForceCoilDriverModule::iSensCallback);
}

void ForceCoilDriverModule::start(void)
{
    if (m_state != STATE_READY) {
        return;
    }

    m_currentSetpoint = 0.0f;
    m_targetDuty = FORCE_COIL_MODULE_MIN_DUTY;
    m_newSetpoint = false;

    m_pidCtrl.start();
    m_state = STATE_OPERATING;
    m_iDriveChannel->start(FORCE_COIL_MODULE_MIN_DUTY);
}

void ForceCoilDriverModule::stop(void)
{
    if (m_state != STATE_OPERATING) {
        return;
    }

    m_iDriveChannel->stop();
    m_pidCtrl.stop();

    m_currentSetpoint = 0.0f;
    m_targetDuty = FORCE_COIL_MODULE_MIN_DUTY;
    m_newSetpoint = false;
    m_state = STATE_READY;
}

void ForceCoilDriverModule::setCurrentSetpoint(float currentSetpoint)
{
    if (m_state != STATE_OPERATING) {
        return;
    }

    m_currentSetpoint = currentSetpoint;
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
    if (m_state != STATE_OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_currentListenerCallbackCount; i++) {
        m_currentListenerCallbacks[i].callback(
            m_currentListenerCallbacks[i].context, measuredCurrent);
    }

    m_targetDuty = m_pidCtrl.execute(m_currentSetpoint, measuredCurrent);

    if (m_newSetpoint) {
        float error = fabsf(measuredCurrent - m_currentSetpoint);
        if (error < FORCE_COIL_MODULE_CURRENT_ERROR_TOLERANCE) {
            m_newSetpoint = false;

            if (m_callback) {
                m_callback(Event::SetpointAchieved);
            }
        }
    }
}

bool ForceCoilDriverModule::onPwmUpdate(float *value)
{
    if (value == nullptr) {
        return false;
    }

    if (m_state != STATE_OPERATING) {
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
