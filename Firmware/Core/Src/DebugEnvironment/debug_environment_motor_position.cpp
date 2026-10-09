#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_POSITION

#include "DebugEnvironment/debug_environment_motor_position.hpp"

#include <cmath>

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
uint16_t MotorPositionDebugEnvironment::m_pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
uint16_t MotorPositionDebugEnvironment::m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

AnalogChannel MotorPositionDebugEnvironment::m_forceCoilISensChannel(
    ADC_CHANNEL_FORCE_COIL_ISENS_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_FORCE_COIL_ISENS_OVERSAMPLING_RATIO),
    FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR,
    -FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR * FORCE_COIL_MODULE_ZERO_CURRENT_VOLTAGE);

PwmRampChannel MotorPositionDebugEnvironment::m_forceCoilPwmChannel(
    &htim1, TIM_CHANNEL_1,
    MotorPositionDebugEnvironment::m_pwmChannel1Buffer,
    2 * TIM1_PWM_CHANNEL1_SAMPLES,
    TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH1 (PWM) + CH1N (nPWM)

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

ForceCoilDriverModule MotorPositionDebugEnvironment::m_forceCoil(
    &MotorPositionDebugEnvironment::m_forceCoilISensChannel,
    &MotorPositionDebugEnvironment::m_forceCoilPwmChannel);

DcMotorVelocityControllerModule MotorPositionDebugEnvironment::m_velocityController(
    &MotorPositionDebugEnvironment::m_lvdtSensorModule,
    &MotorPositionDebugEnvironment::m_zMotorPwmChannel);

DcMotorPositionControllerModule MotorPositionDebugEnvironment::m_positionController(
    &MotorPositionDebugEnvironment::m_velocityController);

MotorPositionDebugEnvironment::MotorPositionDebugEnvironment()
{
    // ADC2 channels (conversion-order ascending).
    m_adc2Service.addChannel(&m_forceCoilISensChannel);
    m_adc2Service.addChannel(&m_lvdtAChannel);
    m_adc2Service.addChannel(&m_lvdtBChannel);

    m_dacService.addChannel(&m_lvdtExcitationChannel);

    m_tim1PwmService.addChannel(&m_forceCoilPwmChannel);
    m_tim1PwmService.addChannel(&m_zMotorPwmChannel);

    addProcess(&m_tim1PwmService);
    addProcess(&m_dacService);
    addProcess(&m_adc2Service);
    addProcess(&m_velocityController);
    addProcess(&m_lvdtSensorModule);
    addProcess(&m_positionController);
    addProcess(&m_forceCoil);

    m_positionController.addPositionSetpointControllerCallback(
        this, &MotorPositionDebugEnvironment::onProvidePositionSetpoint);
    m_forceCoil.addCurrentListenerCallback(
        this, &MotorPositionDebugEnvironment::onCoilCurrentMeasured);
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
    // Optional: hold the force coil at this many grams (0 = off) from
    // coilDelay seconds into the capture, as the protocols do during Z moves.
    const float coilGrams = arg(3);
    const float coilDelay = arg(4);
    // Optional: walk to the target like a ZMOVE, at this speed (mm/s) and
    // acceleration (mm/s^2; 0 takes the profile default).
    const float profileSpeed = arg(5);
    const float profileAccel = arg(6);

    if (duration <= 0.0f || coilGrams < 0.0f || coilDelay < 0.0f ||
        profileSpeed < 0.0f || profileAccel < 0.0f) {
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

    // The sample is wider than the depth was sized for; the buffer decides.
    constexpr uint32_t kMaxSamples =
        DEBUG_TELEMETRY_BUFFER_SIZE_BYTES / sizeof(DebugMotorPositionTelemetrySample);
    if (sampleLimit > kMaxSamples) {
        sampleLimit = kMaxSamples;
    }

    // Same conversion as BonderCommandSetForce, without the setup offset.
    m_coilAmps = (coilGrams > 0.0f)
        ? (coilGrams - FORCE_COIL_CURRENT_TO_GRAMS_OFFSET) /
              FORCE_COIL_CURRENT_TO_GRAMS_SCALE
        : 0.0f;
    m_profileSpeed = profileSpeed;
    m_profileAccel = (profileAccel > 0.0f)
        ? profileAccel
        : static_cast<float>(BONDER_MODULE_DEFAULT_ZMOVE_MAX_ACCELERATION);
    m_profileStarted = false;

    m_coilDelaySamples = static_cast<uint32_t>(
        coilDelay * static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY));
    m_coilEnabled = false;
    if (m_coilAmps > 0.0f) {
        m_forceCoil.setCurrentSetpoint(0.0f);
        if (!m_forceCoil.enableControl()) {
            setError(ERROR_NOT_INITIALIZED);
            return;
        }
        m_coilEnabled = true;
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
        releaseCoil();
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
    releaseCoil();
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
    float *positionSetpoint,
    float *velocityFeedforward)
{
    auto *self = static_cast<MotorPositionDebugEnvironment *>(context);

    if (self == nullptr || !self->m_captureActive) {
        return false;
    }

    // Stall scans and plain steps carry no profile; a profiled step feeds its
    // own velocity forward.
    if (velocityFeedforward != nullptr) {
        *velocityFeedforward = 0.0f;
    }

    if (self->m_mode == Mode::STALL_SCAN) {
        return self->provideStallScanSetpoint(positionSetpoint);
    }

    return self->provideStepSetpoint(positionSetpoint, velocityFeedforward);
}

bool MotorPositionDebugEnvironment::provideStepSetpoint(float *positionSetpoint,
                                                        float *velocityFeedforward)
{
    if (m_sampleIdx >= m_sampleLimit) {
        return false;
    }

    if (m_coilEnabled && m_sampleIdx == m_coilDelaySamples) {
        m_forceCoil.setCurrentSetpoint(m_coilAmps);
    }

    float setpoint = m_stepValue;
    if (m_profileSpeed > 0.0f) {
        if (!m_profileStarted) {
            m_profileSetpoint = m_positionController.getPosition();
            m_profileVelocity = 0.0f;
            m_profileStarted = true;
        }
        advanceProfile();
        setpoint = m_profileSetpoint;
        if (velocityFeedforward != nullptr) {
            *velocityFeedforward = m_profileVelocity;
        }
    }

    m_telemetry[m_sampleIdx++] = DebugMotorPositionTelemetrySample{
        m_positionController.getPosition(),
        m_positionController.getLvdtMagnitudeA(),
        m_positionController.getLvdtMagnitudeB(),
        m_positionController.getVelocity(),
        m_velocityController.getAppliedVoltage(),
        m_coilCurrent,
        setpoint
    };

    if (positionSetpoint != nullptr) {
        *positionSetpoint = setpoint;
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
    releaseCoil();

    m_positionController.disableDriveBypass();
    m_positionController.disableBypass();
    m_positionController.disableControl();

    setDone(RESULT_CAPTURE_COMPLETE, sampleCount);
}

// One tick of BonderCommandZMove's trapezoid, so a profiled step moves the
// way the protocols do: full speed towards the target, given up early enough
// to stop on it, slewed at the acceleration, and never let to lead the
// carriage by more than the following-error bound.
void MotorPositionDebugEnvironment::advanceProfile()
{
    const float period = 1.0f / static_cast<float>(DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY);
    const float remaining = m_stepValue - m_profileSetpoint;
    const bool descending = (remaining < 0.0f);

    float target = descending ? -m_profileSpeed : m_profileSpeed;
    const float limit = sqrtf(2.0f * m_profileAccel * fabsf(remaining));
    target = descending ? fmaxf(target, -limit) : fminf(target, limit);

    const float step = m_profileAccel * period;
    m_profileVelocity = (m_profileVelocity < target)
        ? fminf(m_profileVelocity + step, target)
        : fmaxf(m_profileVelocity - step, target);
    m_profileSetpoint += m_profileVelocity * period;

    const bool overshot = descending ? (m_profileSetpoint < m_stepValue)
                                     : (m_profileSetpoint > m_stepValue);
    if (overshot) {
        m_profileSetpoint = m_stepValue;
        m_profileVelocity = 0.0f;
    }

    const float measured = m_positionController.getPosition();
    const float lead = BONDER_COMMAND_MZDRIVE_MAX_FOLLOWING_ERROR;
    m_profileSetpoint = fminf(fmaxf(m_profileSetpoint, measured - lead), measured + lead);
}

void MotorPositionDebugEnvironment::releaseCoil()
{
    if (!m_coilEnabled) {
        return;
    }

    m_forceCoil.setCurrentSetpoint(0.0f);
    m_forceCoil.disableControl();
    m_coilEnabled = false;
}

void MotorPositionDebugEnvironment::onCoilCurrentMeasured(void *context,
                                                          float measuredCurrent)
{
    auto *self = static_cast<MotorPositionDebugEnvironment *>(context);

    if (self != nullptr) {
        self->m_coilCurrent = measuredCurrent;
    }
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_POSITION
