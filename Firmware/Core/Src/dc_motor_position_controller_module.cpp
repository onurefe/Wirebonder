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
    , m_eventCallback(nullptr)
    , m_eventCallbackContext(nullptr)
    , m_positionMeasurement(0.0f)
    , m_lvdtMagnitudeA(0.0f)
    , m_lvdtMagnitudeB(0.0f)
{}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

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
}

bool DcMotorPositionControllerModule::isControlEnabled() const
{
    return m_controlState == ControlState::Enabled;
}

float DcMotorPositionControllerModule::clampOutput(float rawOutput) const
{
    if (rawOutput > DCMOTOR_POSITION_MODULE_OUTPUT_MAX) {
        return DCMOTOR_POSITION_MODULE_OUTPUT_MAX;
    }

    if (rawOutput < DCMOTOR_POSITION_MODULE_OUTPUT_MIN) {
        return DCMOTOR_POSITION_MODULE_OUTPUT_MIN;
    }

    return rawOutput;
}
