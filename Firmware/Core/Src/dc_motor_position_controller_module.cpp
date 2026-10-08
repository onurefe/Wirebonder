#include "dc_motor_position_controller_module.hpp"

DcMotorPositionControllerModule::DcMotorPositionControllerModule(
    LvdtSensorModule *lvdtSensor,
    DcMotorVelocityControllerModule *velocityController)
    : m_lvdtSensorModule(lvdtSensor)
    , m_velocityController(velocityController)
    , m_controlState(ControlState::Disabled)
    , m_bypassEnabled(false)
    , m_hasPositionMeasurement(false)
    , m_positionMeasurement(0.0f)
    , m_lvdtMagnitudeA(0.0f)
    , m_lvdtMagnitudeB(0.0f)
    , m_outputMin(-ZMOTOR_MAX_DOWNWARD_SPEED_DEFAULT)
    , m_outputMax(ZMOTOR_MAX_UPWARD_SPEED_DEFAULT)
{}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::setOutputLimits(float minVelocity,
                                                      float maxVelocity)
{
    if (minVelocity > maxVelocity) return;

    m_outputMin = minVelocity;
    m_outputMax = maxVelocity;
}

void DcMotorPositionControllerModule::onStart()
{
    if (m_lvdtSensorModule == nullptr || m_velocityController == nullptr) {
        setProcessError();
        return;
    }
    registerPeripheralCallbacks();
}

void DcMotorPositionControllerModule::onStop()
{
    disableControl();
}

bool DcMotorPositionControllerModule::enableControl()
{
    if (!isOperating() || m_controlState != ControlState::Disabled) {
        return false;
    }

    m_hasPositionMeasurement = false;

    if (!m_lvdtSensorModule->startMeasurement()) return false;
    if (!m_velocityController->enableControl()) {
        m_lvdtSensorModule->stopMeasurement();
        return false;
    }

    m_controlState = ControlState::Enabled;
    return true;
}

void DcMotorPositionControllerModule::disableControl()
{
    if (m_controlState != ControlState::Enabled) return;

    m_lvdtSensorModule->stopMeasurement();
    m_velocityController->disableControl();
    m_bypassEnabled = false;
    m_hasPositionMeasurement = false;
    m_controlState = ControlState::Disabled;
}

void DcMotorPositionControllerModule::restartControlLoop()
{
    m_bypassEnabled = false;
}

bool DcMotorPositionControllerModule::addPositionSetpointControllerCallback(
    void *context,
    SetpointCallback callback)
{
    return m_setpointControllerCallbacks.add(context, callback);
}

bool DcMotorPositionControllerModule::removePositionSetpointControllerCallback(
    void *context,
    SetpointCallback callback)
{
    return m_setpointControllerCallbacks.remove(context, callback);
}

bool DcMotorPositionControllerModule::addEventListenerCallback(
    void *context, EventCallback callback)
{
    return m_eventCallbacks.add(context, callback);
}

bool DcMotorPositionControllerModule::removeEventListenerCallback(
    void *context, EventCallback callback)
{
    return m_eventCallbacks.remove(context, callback);
}

bool DcMotorPositionControllerModule::addVelocityListenerCallback(
    void *context, VelocityListenerCallback callback)
{
    return m_velocityListenerCallbacks.add(context, callback);
}

bool DcMotorPositionControllerModule::removeVelocityListenerCallback(
    void *context, VelocityListenerCallback callback)
{
    return m_velocityListenerCallbacks.remove(context, callback);
}

bool DcMotorPositionControllerModule::addPositionListenerCallback(
    void *context, PositionListenerCallback callback)
{
    return m_positionListenerCallbacks.add(context, callback);
}

bool DcMotorPositionControllerModule::removePositionListenerCallback(
    void *context, PositionListenerCallback callback)
{
    return m_positionListenerCallbacks.remove(context, callback);
}

float DcMotorPositionControllerModule::getPosition() const
{
    return m_positionMeasurement;
}

float DcMotorPositionControllerModule::getVelocity() const
{
    return m_velocityController->getVelocity();
}

float DcMotorPositionControllerModule::execute(float positionSetpoint,
                                               float velocityFeedforward)
{
    if (!isControlEnabled()) {
        return 0.0f;
    }

    if (!m_hasPositionMeasurement) {
        return 0.0f;
    }

    if (m_bypassEnabled) {
        return clampOutput(positionSetpoint);
    }

    /* The feedforward carries the motion the provider already knows it is
       asking for; the proportional term is left to correct what is actually
       missing. Both are summed before the clamp, so the machine's velocity
       limit still bounds the total command. */
    const float positionError = positionSetpoint - m_positionMeasurement;
    const float targetVelocity =
        (DCMOTOR_POSITION_MODULE_PROPORTIONAL_GAIN * positionError) +
        (DCMOTOR_POSITION_MODULE_VELOCITY_FEEDFORWARD_GAIN * velocityFeedforward);

    return clampOutput(targetVelocity);
}

float DcMotorPositionControllerModule::getLvdtMagnitudeA() const
{
    return m_lvdtMagnitudeA;
}

float DcMotorPositionControllerModule::getLvdtMagnitudeB() const
{
    return m_lvdtMagnitudeB;
}

void DcMotorPositionControllerModule::enableBypass()
{
    m_bypassEnabled = true;
}

void DcMotorPositionControllerModule::disableBypass()
{
    m_bypassEnabled = false;
}

void DcMotorPositionControllerModule::enableDriveBypass()
{
    m_velocityController->enablePidBypass();
}

void DcMotorPositionControllerModule::disableDriveBypass()
{
    m_velocityController->disablePidBypass();
}

// ---------------------------------------------------------------------------
// Peripheral-Bridge-Callbacks
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::lvdtCallback(void *context, float position, float magA, float magB)
{
    static_cast<DcMotorPositionControllerModule *>(context)->onLvdtMeasured(position, magA, magB);
}

bool DcMotorPositionControllerModule::controlUpdateCallback(void *context, float *targetVelocity)
{
    return static_cast<DcMotorPositionControllerModule *>(context)->onControlUpdate(targetVelocity);
}

void DcMotorPositionControllerModule::velocityMeasuredCallback(void *context, float velocity)
{
    static_cast<DcMotorPositionControllerModule *>(context)->onVelocityMeasured(velocity);
}

// ---------------------------------------------------------------------------
// Peripheral-Event-Handlers
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::onLvdtMeasured(float position, float magA, float magB)
{
    if (!isControlEnabled()) {
        return;
    }

    // Cache only; a bonder or debug callback provides the position target at
    // the moment the velocity loop asks for a setpoint.
    m_positionMeasurement = position;
    m_lvdtMagnitudeA = magA;
    m_lvdtMagnitudeB = magB;
    m_hasPositionMeasurement = true;

    m_positionListenerCallbacks.invoke(m_positionMeasurement);
}

void DcMotorPositionControllerModule::onVelocityMeasured(float velocity)
{
    m_velocityListenerCallbacks.invoke(velocity);
}

bool DcMotorPositionControllerModule::onControlUpdate(float *targetVelocity)
{
    if (targetVelocity == nullptr) {
        return false;
    }

    if (!isControlEnabled()) {
        return false;
    }

    if (!m_hasPositionMeasurement) {
        return false;
    }

    float positionSetpoint = m_positionMeasurement;
    float velocityFeedforward = 0.0f;
    const bool setpointProvided =
        m_setpointControllerCallbacks.invokeFirst(&positionSetpoint,
                                                  &velocityFeedforward);

    if (setpointProvided &&
        fabsf(positionSetpoint - m_positionMeasurement) <
            DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR) {
        m_eventCallbacks.invoke(Event::SetpointReached);
    }

    *targetVelocity = execute(positionSetpoint, velocityFeedforward);
    return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::registerPeripheralCallbacks()
{
    m_lvdtSensorModule->addMeasurementListenerCallback(this,
        &DcMotorPositionControllerModule::lvdtCallback);

    if (!m_velocityController->addVelocityControllerCallback(
            this, &DcMotorPositionControllerModule::controlUpdateCallback)) {
        setProcessError();
    }

    if (!m_velocityController->addVelocityListenerCallback(
            this, &DcMotorPositionControllerModule::velocityMeasuredCallback)) {
        setProcessError();
    }
}

bool DcMotorPositionControllerModule::isControlEnabled() const
{
    return m_controlState == ControlState::Enabled;
}

float DcMotorPositionControllerModule::clampOutput(float rawOutput) const
{
    if (rawOutput > m_outputMax) {
        return m_outputMax;
    }

    if (rawOutput < m_outputMin) {
        return m_outputMin;
    }

    return rawOutput;
}
