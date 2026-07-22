#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_FORCE_COIL

#include "debug_environment_force_coil.hpp"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the force coil
// loop, but owned here outright.
// -----------------------------------------------------------------------------

uint16_t ForceCoilDebugEnvironment::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
uint16_t ForceCoilDebugEnvironment::m_pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];

AnalogChannel ForceCoilDebugEnvironment::m_forceCoilISensChannel(
    ADC_CHANNEL_FORCE_COIL_ISENS_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_FORCE_COIL_ISENS_OVERSAMPLING_RATIO),
    FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR,
    -FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR * FORCE_COIL_MODULE_ZERO_CURRENT_VOLTAGE);

PwmRampChannel ForceCoilDebugEnvironment::m_forceCoilPwmChannel(
    &htim1, TIM_CHANNEL_1,
    ForceCoilDebugEnvironment::m_pwmChannel1Buffer,
    2 * TIM1_PWM_CHANNEL1_SAMPLES,
    TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH1 (PWM) + CH1N (nPWM)

AdcService ForceCoilDebugEnvironment::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE,
    ForceCoilDebugEnvironment::m_adc2Buffer,
    2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

PwmService ForceCoilDebugEnvironment::m_tim1PwmService;

ForceCoilDriverModule ForceCoilDebugEnvironment::m_forceCoil(
    &ForceCoilDebugEnvironment::m_forceCoilISensChannel,
    &ForceCoilDebugEnvironment::m_forceCoilPwmChannel);

ForceCoilDebugEnvironment::ForceCoilDebugEnvironment()
{
    m_adc2Service.addChannel(&m_forceCoilISensChannel);
    m_tim1PwmService.addChannel(&m_forceCoilPwmChannel);

    addProcess(&m_tim1PwmService);
    addProcess(&m_adc2Service);
    addProcess(&m_forceCoil);

    m_forceCoil.addCurrentListenerCallback(
        this, &ForceCoilDebugEnvironment::onCurrentMeasured);
}

void ForceCoilDebugEnvironment::handleCommand(uint16_t localCommand)
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

void ForceCoilDebugEnvironment::startCapture()
{
    m_telemetry = static_cast<float *>(telemetryBuffer());

    const float currentSetpoint = arg(0);
    const float duration = arg(1);
    const bool bypassPid = arg(2) > 0.0f;

    if (duration <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    uint32_t sampleLimit =
        static_cast<uint32_t>(
            duration *
            static_cast<float>(FORCE_COIL_MODULE_CONTROL_FREQUENCY));

    if (sampleLimit == 0u) {
        sampleLimit = 1u;
    }

    if (sampleLimit > DEBUG_FORCE_COIL_TELEMETRY_DEPTH) {
        sampleLimit = DEBUG_FORCE_COIL_TELEMETRY_DEPTH;
    }

    m_sampleIdx = 0;
    m_sampleLimit = sampleLimit;
    m_captureActive = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    if (bypassPid) {
        m_forceCoil.enablePidBypass();
    } else {
        m_forceCoil.disablePidBypass();
    }

    if (!m_forceCoil.enableControl()) {
        m_captureActive = false;
        setError(ERROR_NOT_INITIALIZED);
        return;
    }
    m_forceCoil.setCurrentSetpoint(currentSetpoint);
}

void ForceCoilDebugEnvironment::stopCapture()
{
    m_forceCoil.setCurrentSetpoint(0.0f);
    m_forceCoil.disablePidBypass();
    m_forceCoil.disableControl();

    m_captureActive = false;
    setIdle();
}

void ForceCoilDebugEnvironment::abort()
{
    stopCapture();
}

void ForceCoilDebugEnvironment::onCurrentMeasured(void *context,
                                                  float measuredCurrent)
{
    auto *self = static_cast<ForceCoilDebugEnvironment *>(context);

    if (self != nullptr) {
        self->recordCurrent(measuredCurrent);
    }
}

void ForceCoilDebugEnvironment::recordCurrent(float measuredCurrent)
{
    if (!m_captureActive) {
        return;
    }

    if (m_sampleIdx >= m_sampleLimit) {
        return;
    }

    m_telemetry[m_sampleIdx++] = measuredCurrent;

    if (m_sampleIdx >= m_sampleLimit) {
        m_captureActive = false;

        m_forceCoil.setCurrentSetpoint(0.0f);
        m_forceCoil.disablePidBypass();
        m_forceCoil.disableControl();

        setDone(RESULT_CAPTURE_COMPLETE, m_sampleIdx);
    }
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_FORCE_COIL
