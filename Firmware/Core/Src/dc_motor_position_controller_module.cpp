#include "dc_motor_position_controller_module.hpp"

DcMotorPositionControllerModule::DcMotorPositionControllerModule(
    LvdtSensorModule *lvdtSensor,
    DcMotorVelocityControllerModule *velocityController)
    : m_lvdtSensorModule(lvdtSensor)
    , m_velocityController(velocityController)
    , m_positionPid(PidController::Config{
        DCMOTOR_POSITION_MODULE_PID_GAIN,
        DCMOTOR_POSITION_MODULE_PID_INTEGRAL_TC,
        DCMOTOR_POSITION_MODULE_PID_DERIVATIVE_TC,
        1.0f / static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY),
        DCMOTOR_POSITION_MODULE_PID_FILTER_TC,
        DCMOTOR_POSITION_MODULE_PID_OUTPUT_MIN,
        DCMOTOR_POSITION_MODULE_PID_OUTPUT_MAX})
    , m_state(ServiceState::READY)
    , m_setpointControllerCallbacks{}
    , m_setpointControllerCallbackCount(0)
    , m_positionMeasurement(0.0f)
{
    registerPeripheralCallbacks();
}

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::start()
{
    if (!isReady()) {
        return;
    }

    m_state = ServiceState::OPERATING;

    m_velocityController->start();
    m_lvdtSensorModule->start();

    m_positionPid.start();
}

void DcMotorPositionControllerModule::stop()
{
    if (!isOperating()) {
        return;
    }

    m_state = ServiceState::READY;

    m_lvdtSensorModule->stop();
    m_positionPid.stop();
    m_velocityController->stop();
}

void DcMotorPositionControllerModule::restartControlLoop()
{
    m_positionPid.start();
}

bool DcMotorPositionControllerModule::addPositionSetpointControllerCallback(void *context, SetpointCallback callback)
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

float DcMotorPositionControllerModule::getPosition() const
{
    return m_positionMeasurement;
}

float DcMotorPositionControllerModule::getVelocity() const
{
    return m_velocityController->getVelocity();
}

// ---------------------------------------------------------------------------
// Peripheral-Bridge-Callbacks
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::lvdtCallback(void *context, float position)
{
    static_cast<DcMotorPositionControllerModule *>(context)->onLvdtMeasured(position);
}

bool DcMotorPositionControllerModule::controlUpdateCallback(void *context, float *targetVelocity)
{
    return static_cast<DcMotorPositionControllerModule *>(context)->onControlUpdate(targetVelocity);
}

// ---------------------------------------------------------------------------
// Peripheral-Event-Handlers
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::onLvdtMeasured(float position)
{
    if (!isOperating()) {
        return;
    }

    // Cache only; the control law runs when the velocity loop pulls a
    // setpoint (see onControlUpdate).
    m_positionMeasurement = position;
}

// Multiple controllers may register against the same object without conflict:
// registration is non-exclusive, and each controller reports its activity as a
// byproduct of the call (the bool return). This module acts as a velocity
// controller for the inner loop — active only while operating.
bool DcMotorPositionControllerModule::onControlUpdate(float *targetVelocity)
{
    if (targetVelocity == nullptr) {
        return false;
    }

    if (!isOperating()) {
        return false;
    }

    // First active setpoint controller wins; otherwise hold current position.
    float position_setpoint = m_positionMeasurement;
    for (uint8_t i = 0; i < m_setpointControllerCallbackCount; i++) {
        if (m_setpointControllerCallbacks[i].callback(
                m_setpointControllerCallbacks[i].context, &position_setpoint)) {
            break;
        }
    }

    *targetVelocity = m_positionPid.execute(position_setpoint, m_positionMeasurement);
    return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void DcMotorPositionControllerModule::registerPeripheralCallbacks()
{
    m_lvdtSensorModule->addMeasurementListenerCallback(this, 
        &DcMotorPositionControllerModule::lvdtCallback);
        
    m_velocityController->addVelocityControllerCallback(this,
        &DcMotorPositionControllerModule::controlUpdateCallback);
}

bool DcMotorPositionControllerModule::isReady() const
{
    return m_state == ServiceState::READY;
}

bool DcMotorPositionControllerModule::isOperating() const
{
    return m_state == ServiceState::OPERATING;
}
