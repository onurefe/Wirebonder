#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY

#include "debug_environment_motor_velocity.hpp"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the Z-motor
// velocity loop, but owned here outright.
// -----------------------------------------------------------------------------

uint16_t MotorVelocityDebugEnvironment::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
uint16_t MotorVelocityDebugEnvironment::m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

AnalogChannel MotorVelocityDebugEnvironment::m_tachometerChannel(
    ADC_CHANNEL_ZMOTOR_TACHOMETER_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_ZMOTOR_TACHOMETER_OVERSAMPLING_RATIO),
    ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC,
    -ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE *
        ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC);

PwmRampChannel MotorVelocityDebugEnvironment::m_zMotorPwmChannel(
    &htim1, TIM_CHANNEL_2,
    MotorVelocityDebugEnvironment::m_pwmChannel2Buffer,
    2 * TIM1_PWM_CHANNEL2_SAMPLES,
    TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH2 (PWM) + CH2N (nPWM)

AdcService MotorVelocityDebugEnvironment::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE, ADC2_NUM_CONVERSIONS,
    MotorVelocityDebugEnvironment::m_adc2Buffer,
    2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

PwmService MotorVelocityDebugEnvironment::m_tim1PwmService;

DcMotorVelocityControllerModule MotorVelocityDebugEnvironment::m_velocityController(
    &MotorVelocityDebugEnvironment::m_tachometerChannel,
    &MotorVelocityDebugEnvironment::m_zMotorPwmChannel);

MotorVelocityDebugEnvironment::MotorVelocityDebugEnvironment()
{
    m_adc2Service.addChannel(&m_tachometerChannel);
    m_tim1PwmService.addChannel(&m_zMotorPwmChannel);

    addProcess(&m_tim1PwmService);
    addProcess(&m_adc2Service);
    addProcess(&m_velocityController);

    m_velocityController.addVelocityListenerCallback(
        this, &MotorVelocityDebugEnvironment::onVelocityMeasured);
    m_velocityController.addVelocityControllerCallback(
        this, &MotorVelocityDebugEnvironment::onVelocityControlUpdate);
}

void MotorVelocityDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        startCapture();
        break;

    case CMD_STOP:
        stopCapture();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void MotorVelocityDebugEnvironment::startCapture()
{
    m_telemetry = static_cast<float *>(telemetryBuffer());

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

    m_captureActive = true;
    m_sampleIdx = 0;
    m_sampleLimit = sampleLimit;
    m_stepValue = stepValue;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassController) {
        m_velocityController.enablePidBypass();
    } else {
        m_velocityController.disablePidBypass();
    }

    if (!m_velocityController.enableControl()) {
        m_captureActive = false;
        setError(ERROR_NOT_INITIALIZED);
    }
}

void MotorVelocityDebugEnvironment::stopCapture()
{
    m_velocityController.disablePidBypass();
    m_velocityController.disableControl();

    m_captureActive = false;
    setIdle();
}

void MotorVelocityDebugEnvironment::abort()
{
    stopCapture();
}

void MotorVelocityDebugEnvironment::onVelocityMeasured(void *context,
                                                       float measuredVelocity)
{
    auto *self = static_cast<MotorVelocityDebugEnvironment *>(context);

    if (self != nullptr) {
        self->recordVelocity(measuredVelocity);
    }
}

void MotorVelocityDebugEnvironment::recordVelocity(float measuredVelocity)
{
    if (!m_captureActive) {
        return;
    }

    if (m_sampleIdx >= m_sampleLimit) {
        return;
    }

    m_telemetry[m_sampleIdx++] = measuredVelocity;

    if (m_sampleIdx >= m_sampleLimit) {
        m_captureActive = false;

        m_velocityController.disablePidBypass();
        m_velocityController.disableControl();

        setDone(RESULT_SETPOINT_ACHIEVED, m_sampleIdx);
    }
}

bool MotorVelocityDebugEnvironment::onVelocityControlUpdate(void *context,
                                                            float *targetVelocity)
{
    auto *self = static_cast<MotorVelocityDebugEnvironment *>(context);

    if (self == nullptr || !self->m_captureActive) {
        return false;
    }

    if (targetVelocity != nullptr) {
        *targetVelocity = self->m_stepValue;
    }

    return true;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY
