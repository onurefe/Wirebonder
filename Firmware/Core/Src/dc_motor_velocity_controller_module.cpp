#include "dc_motor_velocity_controller_module.hpp"

DcMotorVelocityControllerModule::DcMotorVelocityControllerModule(
    LvdtSensorModule *lvdtSensor,
    PwmRampChannel *pwmChannel)
    : m_lvdtSensorModule(lvdtSensor)
    , m_pwmChannel(pwmChannel)
    , m_velocityPid(PidController::Config{
        DCMOTOR_VELOCITY_MODULE_PID_GAIN,
        DCMOTOR_VELOCITY_MODULE_PID_INTEGRAL_TC,
        DCMOTOR_VELOCITY_MODULE_PID_DERIVATIVE_TC,
        1.0f / static_cast<float>(DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY),
        DCMOTOR_VELOCITY_MODULE_PID_INPUT_FILTER_TC,
        DCMOTOR_VELOCITY_MODULE_PID_LEAKAGE_TC,
        DCMOTOR_VELOCITY_MODULE_PID_OUTPUT_MIN,
        DCMOTOR_VELOCITY_MODULE_PID_OUTPUT_MAX})
    , m_estimator(ZAxisKalmanFilter::Config{
        ZMOTOR_MODEL_GAIN,
        ZMOTOR_MODEL_TIME_CONSTANT,
        1.0f / static_cast<float>(DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY),
        ZAXIS_KALMAN_POSITION_NOISE,
        ZAXIS_KALMAN_ACCELERATION_NOISE,
        ZAXIS_KALMAN_DISTURBANCE_DRIFT})
    , m_controlState(ControlState::Disabled)
    , m_hasEstimate(false)
    , m_positionMeasurement(0.0f)
    , m_velocityEstimate(0.0f)
    , m_appliedVoltage(0.0f)
    , m_targetDuty(DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY)
{}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::onStart()
{
    if (m_lvdtSensorModule == nullptr || m_pwmChannel == nullptr) {
        setProcessError();
        return;
    }

    // Solved here rather than in the constructor: it is a few hundred
    // double-precision Riccati steps, which want the clocks already up.
    if (!m_estimator.initialize()) {
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
    m_appliedVoltage = 0.0f;
    m_hasEstimate = false;

    m_velocityPid.start();
    m_controlState = ControlState::Enabled;
    if (!m_pwmChannel->start(DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY)) {
        m_velocityPid.stop();
        m_controlState = ControlState::Disabled;
        return false;
    }

    // The PWM holds zero drive until the first measurement arrives and seeds
    // the filter; only then does the loop start acting.
    if (!m_lvdtSensorModule->startMeasurement()) {
        m_pwmChannel->stop();
        m_velocityPid.stop();
        m_controlState = ControlState::Disabled;
        return false;
    }

    return true;
}

void DcMotorVelocityControllerModule::disableControl()
{
    if (m_controlState != ControlState::Enabled) return;

    m_lvdtSensorModule->stopMeasurement();
    m_pwmChannel->stop();
    m_velocityPid.stop();

    m_targetDuty = DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY;
    m_appliedVoltage = 0.0f;
    m_hasEstimate = false;
    m_controlState = ControlState::Disabled;
}

bool DcMotorVelocityControllerModule::addVelocityListenerCallback(void *context, VelocityListenerCallback cb)
{
    return m_velocityListenerCallbacks.add(context, cb);
}

bool DcMotorVelocityControllerModule::removeVelocityListenerCallback(void *context, VelocityListenerCallback cb)
{
    return m_velocityListenerCallbacks.remove(context, cb);
}

bool DcMotorVelocityControllerModule::addPositionListenerCallback(void *context, PositionListenerCallback cb)
{
    return m_positionListenerCallbacks.add(context, cb);
}

bool DcMotorVelocityControllerModule::removePositionListenerCallback(void *context, PositionListenerCallback cb)
{
    return m_positionListenerCallbacks.remove(context, cb);
}

bool DcMotorVelocityControllerModule::addVelocityControllerCallback(void *context, VelocityControllerCallback cb)
{
    return m_velocityControllerCallbacks.add(context, cb);
}

bool DcMotorVelocityControllerModule::removeVelocityControllerCallback(void *context, VelocityControllerCallback cb)
{
    return m_velocityControllerCallbacks.remove(context, cb);
}

float DcMotorVelocityControllerModule::getPosition() const
{
    return m_positionMeasurement;
}

float DcMotorVelocityControllerModule::getVelocity() const
{
    return m_velocityEstimate;
}

float DcMotorVelocityControllerModule::getDisturbance() const
{
    return m_estimator.getDisturbance();
}

float DcMotorVelocityControllerModule::getAppliedVoltage() const
{
    return m_appliedVoltage;
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

void DcMotorVelocityControllerModule::lvdtCallback(void *context, float position, float magA, float magB)
{
    static_cast<DcMotorVelocityControllerModule *>(context)->onLvdtMeasured(position, magA, magB);
}

bool DcMotorVelocityControllerModule::pwmUpdateCallback(void *context, float *value)
{
    return static_cast<DcMotorVelocityControllerModule *>(context)->onPwmUpdate(value);
}

// ---------------------------------------------------------------------------
// Peripheral-Event-Handlers
// ---------------------------------------------------------------------------

void DcMotorVelocityControllerModule::onLvdtMeasured(float position, float magA, float magB)
{
    if (!isControlEnabled()) {
        return;
    }

    if (m_hasEstimate) {
        m_estimator.correct(position);
    } else {
        m_estimator.reset(position);
        m_hasEstimate = true;
    }

    m_positionMeasurement = position;
    m_velocityEstimate = m_estimator.getVelocity();

    // The position loop caches this before it is asked for a target below.
    m_positionListenerCallbacks.invoke(position, magA, magB);

    // Pull the fresh setpoint at the exact instant the loop consumes it: the
    // first active controller wins, otherwise the target stays at zero.
    float target_velocity = 0.0f;
    (void)m_velocityControllerCallbacks.invokeFirst(&target_velocity);

    // A listener or controller may have shut the loop down from inside this
    // tick; leave the zero-drive state disableControl() set alone.
    if (!isControlEnabled()) {
        return;
    }

    float drive = m_velocityPid.execute(
        target_velocity,
        m_velocityEstimate);

    m_targetDuty = computeTargetDuty(drive);

    /* The model is driven by what the bridge will actually see, after the
       duty clamp, not by what the PID asked for -- otherwise a saturated
       drive would be credited with motion it never produced. The PWM ramps
       to this duty over the next segment, a lag the model does not carry. */
    m_appliedVoltage = dutyToVoltage(m_targetDuty);
    m_estimator.predict(m_appliedVoltage);

    m_velocityListenerCallbacks.invoke(m_velocityEstimate);
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
    if (!m_lvdtSensorModule->addMeasurementListenerCallback(
            this, &DcMotorVelocityControllerModule::lvdtCallback) ||
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
    /* ZMOTOR_DRIVE_DIRECTION carries which way duty above the zero-velocity
       point moves the head. It is load-bearing for the closed loop -- see
       the constraint in configuration.h -- so it is not the place to correct
       the direction of a single open-loop move. */
    float raw_duty = DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY +
                     (ZMOTOR_DRIVE_DIRECTION * velocityControlOutput *
                      DCMOTOR_VELOCITY_MODULE_VOLTAGE_TO_DUTY_SCALE);

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

// Inverse of computeTargetDuty(): the up-positive voltage a duty applies.
// ZMOTOR_DRIVE_DIRECTION is +/-1, so it is its own inverse.
float DcMotorVelocityControllerModule::dutyToVoltage(float duty)
{
    return ZMOTOR_DRIVE_DIRECTION *
           (duty - static_cast<float>(DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY)) /
           DCMOTOR_VELOCITY_MODULE_VOLTAGE_TO_DUTY_SCALE;
}
