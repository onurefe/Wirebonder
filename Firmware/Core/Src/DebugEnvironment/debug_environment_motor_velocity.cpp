#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY

#include "DebugEnvironment/debug_environment_motor_velocity.hpp"

extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the Z-motor
// velocity loop (LVDT + Kalman estimate), but owned here outright.
// -----------------------------------------------------------------------------

uint16_t MotorVelocityDebugEnvironment::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
uint16_t MotorVelocityDebugEnvironment::m_dac2Buffer[2 * DAC2_SAMPLES];
uint16_t MotorVelocityDebugEnvironment::m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

IQDemodulatorChannel MotorVelocityDebugEnvironment::m_lvdtAChannel(
    ADC_CHANNEL_LVDT_A_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

IQDemodulatorChannel MotorVelocityDebugEnvironment::m_lvdtBChannel(
    ADC_CHANNEL_LVDT_B_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

SineGeneratorChannel MotorVelocityDebugEnvironment::m_lvdtExcitationChannel(
    &hdac, &htim5, DAC_CHANNEL_2,
    MotorVelocityDebugEnvironment::m_dac2Buffer, 2 * DAC2_SAMPLES,
    DAC2_SAMPLES, DAC2_BITS, DAC2_VOLTAGE_RANGE);

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

DacService MotorVelocityDebugEnvironment::m_dacService(&hdac);

PwmService MotorVelocityDebugEnvironment::m_tim1PwmService;

LvdtSensorModule MotorVelocityDebugEnvironment::m_lvdtSensorModule(
    &MotorVelocityDebugEnvironment::m_lvdtExcitationChannel,
    &MotorVelocityDebugEnvironment::m_lvdtAChannel,
    &MotorVelocityDebugEnvironment::m_lvdtBChannel,
    LVDT_MODULE_STROKE_MM);

DcMotorVelocityControllerModule MotorVelocityDebugEnvironment::m_velocityController(
    &MotorVelocityDebugEnvironment::m_lvdtSensorModule,
    &MotorVelocityDebugEnvironment::m_zMotorPwmChannel);

MotorVelocityDebugEnvironment::MotorVelocityDebugEnvironment()
{
    // ADC2 channels (conversion-order ascending).
    m_adc2Service.addChannel(&m_lvdtAChannel);
    m_adc2Service.addChannel(&m_lvdtBChannel);

    m_dacService.addChannel(&m_lvdtExcitationChannel);

    m_tim1PwmService.addChannel(&m_zMotorPwmChannel);

    addProcess(&m_tim1PwmService);
    addProcess(&m_dacService);
    addProcess(&m_adc2Service);
    addProcess(&m_lvdtSensorModule);
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
    m_telemetry =
        static_cast<DebugMotorVelocityTelemetrySample *>(telemetryBuffer());

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

    // Fired at the end of the tick, so the applied voltage is the one just
    // set for the coming period.
    m_telemetry[m_sampleIdx++] = DebugMotorVelocityTelemetrySample{
        m_velocityController.getPosition(),
        measuredVelocity,
        m_velocityController.getAppliedVoltage()
    };

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
