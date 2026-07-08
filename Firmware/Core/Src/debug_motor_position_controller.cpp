#include "debug_motor_position_controller.hpp"

DebugMotorPositionController::DebugMotorPositionController()
{
}

void DebugMotorPositionController::init(
    DcMotorPositionControllerModule *positionController)
{
    m_positionController = positionController;

    if (m_positionController != nullptr) {
        m_positionController->addPositionSetpointControllerCallback(
            this,
            &DebugMotorPositionController::onProvidePositionSetpoint);
    }
}

void DebugMotorPositionController::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        start();
        break;

    case CMD_STALL_SCAN:
        startStallScan();
        break;

    case CMD_STOP:
        stop();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void DebugMotorPositionController::start()
{
    if (m_positionController == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    m_telemetry =
        static_cast<DebugMotorPositionTelemetrySample *>(telemetryBufferPtr());
    if (m_telemetry == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    const float targetPosition = arg(0);
    const float duration = arg(1);
    const bool bypassController = arg(2) > 0.0f;

    if (duration <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    uint32_t sampleLimit =
        static_cast<uint32_t>(
            duration *
            static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY));

    if (sampleLimit == 0u) {
        sampleLimit = 1u;
    }

    if (sampleLimit > DEBUG_MOTOR_POSITION_CONTROLLER_TELEMETRY_DEPTH) {
        sampleLimit = DEBUG_MOTOR_POSITION_CONTROLLER_TELEMETRY_DEPTH;
    }

    m_sampleIdx = 0;
    m_sampleLimit = sampleLimit;
    m_settleIdx = 0;
    m_settleLimit = 1;
    m_relaxIdx = 0;
    m_relaxLimit = 1;
    m_mode = Mode::STEP;
    m_stepValue = targetPosition;
    m_busy = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassController) {
        m_positionController->enableBypass();
    } else {
        m_positionController->disableBypass();
    }

    m_positionController->start();
}

void DebugMotorPositionController::startStallScan()
{
    if (m_positionController == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    m_stallTelemetry =
        static_cast<DebugMotorPositionStallTelemetrySample *>(telemetryBufferPtr());
    if (m_stallTelemetry == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    const float startDrive = arg(0);
    const float endDrive = arg(1);
    const float driveStep = arg(2);
    const float settleSeconds = arg(3);
    const float relaxSeconds = arg(4);

    if (driveStep == 0.0f || settleSeconds <= 0.0f || relaxSeconds < 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if ((endDrive > startDrive && driveStep < 0.0f) ||
        (endDrive < startDrive && driveStep > 0.0f)) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    uint32_t settleLimit =
        static_cast<uint32_t>(
            settleSeconds *
            static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY));

    if (settleLimit == 0u) {
        settleLimit = 1u;
    }

    uint32_t relaxLimit =
        static_cast<uint32_t>(
            relaxSeconds *
            static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY));

    m_sampleIdx = 0;
    m_sampleLimit = DEBUG_MOTOR_POSITION_CONTROLLER_TELEMETRY_DEPTH;
    m_settleIdx = 0;
    m_settleLimit = settleLimit;
    m_relaxIdx = 0;
    m_relaxLimit = relaxLimit;
    m_mode = Mode::STALL_SCAN;
    m_stallScanPhase = StallScanPhase::SETTLING;
    m_currentDrive = startDrive;
    m_endDrive = endDrive;
    m_driveStep = driveStep;
    m_relaxDrive =
        (driveStep > 0.0f) ?
        -DEBUG_MOTOR_POSITION_STALL_RELAX_DRIVE :
        DEBUG_MOTOR_POSITION_STALL_RELAX_DRIVE;
    m_busy = true;

    setBusy();
    setResultPointer(0, m_stallTelemetry);

    m_positionController->enableBypass();
    m_positionController->enableDriveBypass();
    m_positionController->start();
}

void DebugMotorPositionController::stop()
{
    if (m_positionController != nullptr) {
        m_positionController->disableDriveBypass();
        m_positionController->disableBypass();
        m_positionController->stop();
    }

    m_busy = false;
    setIdle();
}

void DebugMotorPositionController::abort()
{
    stop();
}

bool DebugMotorPositionController::onProvidePositionSetpoint(void *context, float *positionSetpoint)
{
    DebugMotorPositionController *self =
        static_cast<DebugMotorPositionController *>(context);

    if (!self->m_busy) {
        return false;
    }

    if (self->m_mode == Mode::STALL_SCAN) {
        return self->provideStallScanSetpoint(positionSetpoint);
    }

    return self->provideStepSetpoint(positionSetpoint);
}

bool DebugMotorPositionController::provideStepSetpoint(float *positionSetpoint)
{
    if (m_sampleIdx >= m_sampleLimit) {
        return false;
    }

    m_telemetry[m_sampleIdx++] = DebugMotorPositionTelemetrySample{
        m_positionController->getPosition(),
        m_positionController->getLvdtMagnitudeA(),
        m_positionController->getLvdtMagnitudeB()
    };

    if (positionSetpoint != nullptr) {
        *positionSetpoint = m_stepValue;
    }

    if (m_sampleIdx >= m_sampleLimit) {
        finish(m_sampleIdx);
        return false;
    }

    return true;
}

bool DebugMotorPositionController::provideStallScanSetpoint(float *positionSetpoint)
{
    if (m_stallScanPhase == StallScanPhase::RELAXING) {
        if (positionSetpoint != nullptr) {
            *positionSetpoint = m_relaxDrive;
        }

        if (++m_relaxIdx >= m_relaxLimit) {
            m_relaxIdx = 0;
            m_settleIdx = 0;
            m_stallScanPhase = StallScanPhase::SETTLING;
        }

        return true;
    }

    if (positionSetpoint != nullptr) {
        *positionSetpoint = m_currentDrive;
    }

    if (++m_settleIdx < m_settleLimit) {
        return true;
    }

    m_settleIdx = 0;

    if (m_sampleIdx < m_sampleLimit) {
        m_stallTelemetry[m_sampleIdx++] = DebugMotorPositionStallTelemetrySample{
            m_currentDrive,
            m_positionController->getPosition(),
            m_positionController->getLvdtMagnitudeA(),
            m_positionController->getLvdtMagnitudeB()
        };
    }

    const float nextDrive = m_currentDrive + m_driveStep;
    const bool scanDone =
        (m_driveStep > 0.0f && nextDrive > m_endDrive) ||
        (m_driveStep < 0.0f && nextDrive < m_endDrive) ||
        (m_sampleIdx >= m_sampleLimit);

    if (scanDone) {
        finish(m_sampleIdx);
        return false;
    }

    m_currentDrive = nextDrive;
    if (m_relaxLimit > 0u) {
        m_relaxIdx = 0;
        m_stallScanPhase = StallScanPhase::RELAXING;
    }
    return true;
}

void DebugMotorPositionController::finish(uint32_t sampleCount)
{
    m_busy = false;

    if (m_positionController != nullptr) {
        m_positionController->disableDriveBypass();
        m_positionController->disableBypass();
        m_positionController->stop();
    }

    setDone(RESULT_CAPTURE_COMPLETE, sampleCount);
}
