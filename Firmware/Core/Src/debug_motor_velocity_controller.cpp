#include "debug_motor_velocity_controller.hpp"

DebugMotorVelocityController::DebugMotorVelocityController()
{
}

void DebugMotorVelocityController::init(DcMotorVelocityControllerModule *velocityController)
{
    m_velocityController = velocityController;

    if (m_velocityController != nullptr) {
        m_velocityController->addVelocityListenerCallback(this, 
            &DebugMotorVelocityController::onVelocityMeasured);
        m_velocityController->addVelocityControllerCallback(this,
            &DebugMotorVelocityController::onVelocityControlUpdate);
    }
}

void DebugMotorVelocityController::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        start();
        break;

    case CMD_STOP:
        stop();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void DebugMotorVelocityController::start()
{
    if (m_velocityController == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    m_telemetry = static_cast<float *>(telemetryBufferPtr());
    if (m_telemetry == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    const float stepValue = arg(0);
    const float duration = arg(1);
    const bool bypassController = arg(2) > 0.0f;

    if (duration <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    uint32_t sampleLimit =
        static_cast<uint32_t>(
            duration *
            static_cast<float>(DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY));

    if (sampleLimit == 0u) {
        sampleLimit = 1u;
    }

    if (sampleLimit > DEBUG_MOTOR_VELOCITY_CONTROLLER_TELEMETRY_DEPTH) {
        sampleLimit = DEBUG_MOTOR_VELOCITY_CONTROLLER_TELEMETRY_DEPTH;
    }

    m_busy = true;
    m_sampleIdx = 0;
    m_sampleLimit = sampleLimit;
    m_stepValue = stepValue;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassController) {
        m_velocityController->enablePidBypass();
    } else {
        m_velocityController->disablePidBypass();
    }

    m_velocityController->start();
}

void DebugMotorVelocityController::stop()
{
    if (m_velocityController != nullptr) {
        m_velocityController->disablePidBypass();
        m_velocityController->stop();
    }

    m_busy = false;
    setIdle();
}

void DebugMotorVelocityController::abort()
{
    stop();
}

void DebugMotorVelocityController::onVelocityMeasured(void *context, float measuredVelocity)
{
    DebugMotorVelocityController *self;
    self = reinterpret_cast<DebugMotorVelocityController *>(context);

    if (!self->m_busy) {
        return;
    }

    if (self->m_sampleIdx >= self->m_sampleLimit) {
        return;
    }

    self->m_telemetry[self->m_sampleIdx++] = measuredVelocity;

    if (self->m_sampleIdx >= self->m_sampleLimit) {
        self->m_busy = false;

        if (self->m_velocityController != nullptr) {
            self->m_velocityController->disablePidBypass();
            self->m_velocityController->stop();
        }

        self->setDone(RESULT_SETPOINT_ACHIEVED, self->m_sampleIdx);
    }
}

bool DebugMotorVelocityController::onVelocityControlUpdate(void *context, float *targetVelocity)
{
    DebugMotorVelocityController *self;
    self = reinterpret_cast<DebugMotorVelocityController *>(context);

    if (!self->m_busy) {
        return false;
    }

    if (targetVelocity != nullptr) {
        *targetVelocity = self->m_stepValue;
    }

    return true;
}
