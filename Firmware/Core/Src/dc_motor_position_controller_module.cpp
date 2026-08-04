#include "dc_motor_position_controller_module.hpp"

DcMotorPositionControllerModule::DcMotorPositionControllerModule(
    LvdtSensorModule *lvdtSensor,
    DcMotorVelocityControllerModule *velocityController)
    : m_lvdtSensorModule(lvdtSensor)
    , m_velocityController(velocityController)
    , m_controlState(ControlState::Disabled)
    , m_bypassEnabled(false)
    , m_hasPositionMeasurement(false)
    , m_setpointControllerCallbacks{}
    , m_setpointControllerCallbackCount(0)
    , m_velocityListenerCallbacks{}
    , m_velocityListenerCallbackCount(0)
    , m_positionListenerCallbacks{}
    , m_positionListenerCallbackCount(0)
    , m_eventCallback(nullptr)
    , m_eventCallbackContext(nullptr)
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
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_setpointControllerCallbackCount; i++) {
        if (m_setpointControllerCallbacks[i].context == context &&
            m_setpointControllerCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_setpointControllerCallbackCount >= kMaxSetpointControllerCallbacks) {
        return false;
    }

    m_setpointControllerCallbacks[m_setpointControllerCallbackCount++] =
        SetpointControllerRegistration{callback, context};
    return true;
}

void DcMotorPositionControllerModule::addEventListenerCallback(
    void *context, EventCallback callback)
{
    m_eventCallbackContext = context;
    m_eventCallback = callback;
}

bool DcMotorPositionControllerModule::addVelocityListenerCallback(
    void *context, VelocityListenerCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_velocityListenerCallbackCount; i++) {
        if (m_velocityListenerCallbacks[i].context == context &&
            m_velocityListenerCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_velocityListenerCallbackCount >= kMaxVelocityListenerCallbacks) {
        return false;
    }

    m_velocityListenerCallbacks[m_velocityListenerCallbackCount++] =
        VelocityListenerRegistration{callback, context};
    return true;
}

bool DcMotorPositionControllerModule::addPositionListenerCallback(
    void *context, PositionListenerCallback callback)
{
    if (callback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_positionListenerCallbackCount; i++) {
        if (m_positionListenerCallbacks[i].context == context &&
            m_positionListenerCallbacks[i].callback == callback) {
            return true;
        }
    }

    if (m_positionListenerCallbackCount >= kMaxPositionListenerCallbacks) {
        return false;
    }

    m_positionListenerCallbacks[m_positionListenerCallbackCount++] =
        PositionListenerRegistration{callback, context};
    return true;
}

float DcMotorPositionControllerModule::getPosition() const
{
    return m_positionMeasurement;
}

float DcMotorPositionControllerModule::getVelocity() const
{
    return m_velocityController->getVelocity();
}

float DcMotorPositionControllerModule::execute(float positionSetpoint)
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

    const float positionError = positionSetpoint - m_positionMeasurement;
    const float targetVelocity =
        DCMOTOR_POSITION_MODULE_PROPORTIONAL_GAIN * positionError;

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

    for (uint8_t i = 0; i < m_positionListenerCallbackCount; i++) {
        m_positionListenerCallbacks[i].callback(
            m_positionListenerCallbacks[i].context, m_positionMeasurement);
    }
}

void DcMotorPositionControllerModule::onVelocityMeasured(float velocity)
{
    for (uint8_t i = 0; i < m_velocityListenerCallbackCount; i++) {
        m_velocityListenerCallbacks[i].callback(
            m_velocityListenerCallbacks[i].context, velocity);
    }
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
    bool setpointProvided = false;
    for (uint8_t i = 0; i < m_setpointControllerCallbackCount; i++) {
        if (m_setpointControllerCallbacks[i].callback(
                m_setpointControllerCallbacks[i].context, &positionSetpoint)) {
            setpointProvided = true;
            break;
        }
    }

    if (setpointProvided && m_eventCallback != nullptr &&
        fabsf(positionSetpoint - m_positionMeasurement) <
            DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR) {
        m_eventCallback(m_eventCallbackContext, Event::SetpointReached);
    }

    *targetVelocity = execute(positionSetpoint);
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
