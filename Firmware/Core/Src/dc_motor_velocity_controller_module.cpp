#include "dc_motor_velocity_controller_module.hpp"

DcMotorVelocityControllerModule::DcMotorVelocityControllerModule(
    AnalogChannel *tachometerChannel,
    PwmRampChannel *pwmChannel)
    : m_tachometerChannel(tachometerChannel)
    , m_pwmChannel(pwmChannel)
    , m_velocityPid(PidController::Config{
        DCMOTOR_VELOCITY_MODULE_PID_GAIN,
        DCMOTOR_VELOCITY_MODULE_PID_INTEGRAL_TC,
        DCMOTOR_VELOCITY_MODULE_PID_DERIVATIVE_TC,
        1.0f / static_cast<float>(DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY),
        DCMOTOR_VELOCITY_MODULE_PID_INPUT_FILTER_TC,
        DCMOTOR_VELOCITY_MODULE_PID_OUTPUT_MIN,
        DCMOTOR_VELOCITY_MODULE_PID_OUTPUT_MAX})
    , m_controlState(ControlState::Disabled)
    , m_velocityListenerCallbacks{}
    , m_velocityListenerCallbackCount(0)
    , m_velocityControllerCallbacks{}
    , m_velocityControllerCallbackCount(0)
    , m_velocityMeasurement(0.0f)
    , m_targetDuty(DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY)
{}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::onStart()
{
    if (m_tachometerChannel == nullptr || m_pwmChannel == nullptr) {
        setProcessError();
        return;
    }
    registerPeripheralCallbacks();
}

void DcMotorVelocityControllerModule::onStop()
{
    disableControl();
}

bool DcMotorVelocityControllerModule::enableControl()
{
    if (!isOperating() || m_controlState != ControlState::Disabled) {
        return false;
    }

    m_targetDuty = DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY;

    m_velocityPid.start();
    m_controlState = ControlState::Enabled;
    if (!m_pwmChannel->start(DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY)) {
        m_velocityPid.stop();
        m_controlState = ControlState::Disabled;
        return false;
    }
    return true;
}

void DcMotorVelocityControllerModule::disableControl()
{
    if (m_controlState != ControlState::Enabled) return;

    m_pwmChannel->stop();
    m_velocityPid.stop();

    m_targetDuty = DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY;
    m_controlState = ControlState::Disabled;
}

bool DcMotorVelocityControllerModule::addVelocityListenerCallback(void *context, VelocityListenerCallback cb)
{
    if (cb == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_velocityListenerCallbackCount; i++) {
        if (m_velocityListenerCallbacks[i].context == context &&
            m_velocityListenerCallbacks[i].callback == cb) {
            return true;
        }
    }

    if (m_velocityListenerCallbackCount >= kMaxVelocityListenerCallbacks) {
        return false;
    }

    m_velocityListenerCallbacks[m_velocityListenerCallbackCount++] =
        VelocityListenerRegistration{cb, context};
    return true;
}

bool DcMotorVelocityControllerModule::addVelocityControllerCallback(void *context, VelocityControllerCallback cb)
{
    if (cb == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_velocityControllerCallbackCount; i++) {
        if (m_velocityControllerCallbacks[i].context == context &&
            m_velocityControllerCallbacks[i].callback == cb) {
            return true;
        }
    }

    if (m_velocityControllerCallbackCount >= kMaxVelocityControllerCallbacks) {
        return false;
    }

    m_velocityControllerCallbacks[m_velocityControllerCallbackCount++] =
        VelocityControllerRegistration{cb, context};
    return true;
}

float DcMotorVelocityControllerModule::getVelocity() const
{
    return m_velocityMeasurement;
}

void DcMotorVelocityControllerModule::enablePidBypass()
{
    m_velocityPid.enableBypass();
}

void DcMotorVelocityControllerModule::disablePidBypass()
{
    m_velocityPid.disableBypass();
}

// ---------------------------------------------------------------------------
// Peripheral-Bridge-Callbacks
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::tachometerCallback(void *context, float velocity)
{
    static_cast<DcMotorVelocityControllerModule *>(context)->onTachometerMeasured(velocity);
}

bool DcMotorVelocityControllerModule::pwmUpdateCallback(void *context, float *value)
{
    return static_cast<DcMotorVelocityControllerModule *>(context)->onPwmUpdate(value);
}

// ---------------------------------------------------------------------------
// Peripheral-Event-Handlers
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::onTachometerMeasured(float velocity)
{
    if (!isControlEnabled()) {
        return;
    }

    m_velocityMeasurement = velocity;

    // Notify observers of the fresh measurement.
    for (uint8_t i = 0; i < m_velocityListenerCallbackCount; i++) {
        m_velocityListenerCallbacks[i].callback(
            m_velocityListenerCallbacks[i].context, m_velocityMeasurement);
    }

    // Pull the fresh setpoint at the exact instant the loop consumes it: the
    // first active controller wins, otherwise the target stays at zero.
    float target_velocity = 0.0f;
    for (uint8_t i = 0; i < m_velocityControllerCallbackCount; i++) {
        if (m_velocityControllerCallbacks[i].callback(
                m_velocityControllerCallbacks[i].context, &target_velocity)) {
            break;
        }
    }

    float drive = m_velocityPid.execute(
        target_velocity,
        m_velocityMeasurement);

    m_targetDuty = computeTargetDuty(drive);
}

bool DcMotorVelocityControllerModule::onPwmUpdate(float *value)
{
    if (value == nullptr) {
        return false;
    }

    // This module owns the PWM channel: it is always the active duty source
    // while the ramp is running, holding the zero-velocity duty when idle.
    *value = isControlEnabled() ? m_targetDuty : DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY;
    return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::registerPeripheralCallbacks()
{
    if (!m_tachometerChannel->addMeasurementListenerCallback(
            this, &DcMotorVelocityControllerModule::tachometerCallback) ||
        !m_pwmChannel->addTargetDutyControllerCallback(
            this, &DcMotorVelocityControllerModule::pwmUpdateCallback)) {
        setProcessError();
    }
}

bool DcMotorVelocityControllerModule::isControlEnabled() const
{
    return m_controlState == ControlState::Enabled;
}

float DcMotorVelocityControllerModule::computeTargetDuty(float velocityControlOutput) const
{
    float raw_duty = DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY + velocityControlOutput * DCMOTOR_VELOCITY_MODULE_VOLTAGE_TO_DUTY_SCALE;
    
    return clampDuty(raw_duty);
}

float DcMotorVelocityControllerModule::clampDuty(float duty)
{
#if defined(DCMOTOR_VELOCITY_MODULE_MIN_DUTY) && defined(DCMOTOR_VELOCITY_MODULE_MAX_DUTY)
    if (duty < DCMOTOR_VELOCITY_MODULE_MIN_DUTY) {
        return DCMOTOR_VELOCITY_MODULE_MIN_DUTY;
    }

    if (duty > DCMOTOR_VELOCITY_MODULE_MAX_DUTY) {
        return DCMOTOR_VELOCITY_MODULE_MAX_DUTY;
    }
#endif

    return duty;
}
