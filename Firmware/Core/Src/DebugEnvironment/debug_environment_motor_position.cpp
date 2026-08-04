#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_POSITION

#include "DebugEnvironment/debug_environment_motor_position.hpp"

extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the Z-axis
// position chain (LVDT + velocity loop), but owned here outright.
// -----------------------------------------------------------------------------

uint16_t MotorPositionDebugEnvironment::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
uint16_t MotorPositionDebugEnvironment::m_dac2Buffer[2 * DAC2_SAMPLES];
uint16_t MotorPositionDebugEnvironment::m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

AnalogChannel MotorPositionDebugEnvironment::m_tachometerChannel(
    ADC_CHANNEL_ZMOTOR_TACHOMETER_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_ZMOTOR_TACHOMETER_OVERSAMPLING_RATIO),
    ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC,
    -ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE *
        ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC);

IQDemodulatorChannel MotorPositionDebugEnvironment::m_lvdtAChannel(
    ADC_CHANNEL_LVDT_A_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

IQDemodulatorChannel MotorPositionDebugEnvironment::m_lvdtBChannel(
    ADC_CHANNEL_LVDT_B_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

SineGeneratorChannel MotorPositionDebugEnvironment::m_lvdtExcitationChannel(
    &hdac, &htim5, DAC_CHANNEL_2,
    MotorPositionDebugEnvironment::m_dac2Buffer, 2 * DAC2_SAMPLES,
    DAC2_SAMPLES, DAC2_BITS, DAC2_VOLTAGE_RANGE);

PwmRampChannel MotorPositionDebugEnvironment::m_zMotorPwmChannel(
    &htim1, TIM_CHANNEL_2,
    MotorPositionDebugEnvironment::m_pwmChannel2Buffer,
    2 * TIM1_PWM_CHANNEL2_SAMPLES,
    TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH2 (PWM) + CH2N (nPWM)

AdcService MotorPositionDebugEnvironment::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE, ADC2_NUM_CONVERSIONS,
    MotorPositionDebugEnvironment::m_adc2Buffer,
    2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

DacService MotorPositionDebugEnvironment::m_dacService(&hdac);

PwmService MotorPositionDebugEnvironment::m_tim1PwmService;

LvdtSensorModule MotorPositionDebugEnvironment::m_lvdtSensorModule(
    &MotorPositionDebugEnvironment::m_lvdtExcitationChannel,
    &MotorPositionDebugEnvironment::m_lvdtAChannel,
    &MotorPositionDebugEnvironment::m_lvdtBChannel,
    LVDT_MODULE_STROKE_MM);

DcMotorVelocityControllerModule MotorPositionDebugEnvironment::m_velocityController(
    &MotorPositionDebugEnvironment::m_tachometerChannel,
    &MotorPositionDebugEnvironment::m_zMotorPwmChannel);

DcMotorPositionControllerModule MotorPositionDebugEnvironment::m_positionController(
    &MotorPositionDebugEnvironment::m_lvdtSensorModule,
    &MotorPositionDebugEnvironment::m_velocityController);

MotorPositionDebugEnvironment::MotorPositionDebugEnvironment()
{
    // ADC2 channels (conversion-order ascending).
    m_adc2Service.addChannel(&m_tachometerChannel);
    m_adc2Service.addChannel(&m_lvdtAChannel);
    m_adc2Service.addChannel(&m_lvdtBChannel);

    m_dacService.addChannel(&m_lvdtExcitationChannel);

    m_tim1PwmService.addChannel(&m_zMotorPwmChannel);

    addProcess(&m_tim1PwmService);
    addProcess(&m_dacService);
    addProcess(&m_adc2Service);
    addProcess(&m_velocityController);
    addProcess(&m_lvdtSensorModule);
    addProcess(&m_positionController);

    m_positionController.addPositionSetpointControllerCallback(
        this, &MotorPositionDebugEnvironment::onProvidePositionSetpoint);
}

void MotorPositionDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        startStep();
        break;

    case CMD_STALL_SCAN:
        startStallScan();
        break;

    case CMD_STOP:
        stopCapture();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void MotorPositionDebugEnvironment::startStep()
{
    m_telemetry =
        static_cast<DebugMotorPositionTelemetrySample *>(telemetryBuffer());

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
    m_captureActive = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassController) {
        m_positionController.enableBypass();
    } else {
        m_positionController.disableBypass();
    }

    if (!m_positionController.enableControl()) {
        m_captureActive = false;
        setError(ERROR_NOT_INITIALIZED);
    }
}

void MotorPositionDebugEnvironment::startStallScan()
{
    m_stallTelemetry =
        static_cast<DebugMotorPositionStallTelemetrySample *>(telemetryBuffer());

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
    m_captureActive = true;

    setBusy();
    setResultPointer(0, m_stallTelemetry);

    m_positionController.enableBypass();
    m_positionController.enableDriveBypass();
    if (!m_positionController.enableControl()) {
        m_captureActive = false;
        setError(ERROR_NOT_INITIALIZED);
    }
}

void MotorPositionDebugEnvironment::stopCapture()
{
    m_positionController.disableDriveBypass();
    m_positionController.disableBypass();
    m_positionController.disableControl();

    m_captureActive = false;
    setIdle();
}

void MotorPositionDebugEnvironment::abort()
{
    stopCapture();
}

bool MotorPositionDebugEnvironment::onProvidePositionSetpoint(
    void *context,
    float *positionSetpoint)
{
    auto *self = static_cast<MotorPositionDebugEnvironment *>(context);

    if (self == nullptr || !self->m_captureActive) {
        return false;
    }

    if (self->m_mode == Mode::STALL_SCAN) {
        return self->provideStallScanSetpoint(positionSetpoint);
    }

    return self->provideStepSetpoint(positionSetpoint);
}

bool MotorPositionDebugEnvironment::provideStepSetpoint(float *positionSetpoint)
{
    if (m_sampleIdx >= m_sampleLimit) {
        return false;
    }

    m_telemetry[m_sampleIdx++] = DebugMotorPositionTelemetrySample{
        m_positionController.getPosition(),
        m_positionController.getLvdtMagnitudeA(),
        m_positionController.getLvdtMagnitudeB()
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

bool MotorPositionDebugEnvironment::provideStallScanSetpoint(float *positionSetpoint)
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
            m_positionController.getPosition(),
            m_positionController.getLvdtMagnitudeA(),
            m_positionController.getLvdtMagnitudeB()
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

void MotorPositionDebugEnvironment::finish(uint32_t sampleCount)
{
    m_captureActive = false;

    m_positionController.disableDriveBypass();
    m_positionController.disableBypass();
    m_positionController.disableControl();

    setDone(RESULT_CAPTURE_COMPLETE, sampleCount);
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_POSITION
