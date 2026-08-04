#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_TONE_GENERATOR

#include "DebugEnvironment/debug_environment_tone_generator.hpp"
#include <cmath>

namespace {

constexpr float Pi = 3.14159265358979323846f;

} // namespace

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the ultrasonic
// chain, but owned here outright.
// -----------------------------------------------------------------------------

uint16_t ToneGeneratorDebugEnvironment::m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
uint16_t ToneGeneratorDebugEnvironment::m_dac1Buffer[2 * DAC1_SAMPLES];

IQDemodulatorChannel ToneGeneratorDebugEnvironment::m_ultrasonicVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_VSENS_GAIN);

IQDemodulatorChannel ToneGeneratorDebugEnvironment::m_ultrasonicIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_ISENS_GAIN);

SineGeneratorChannel ToneGeneratorDebugEnvironment::m_ultrasonicDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    ToneGeneratorDebugEnvironment::m_dac1Buffer, 2 * DAC1_SAMPLES,
    DAC1_SAMPLES, DAC1_BITS, DAC1_VOLTAGE_RANGE);

AdcService ToneGeneratorDebugEnvironment::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE, ADC1_NUM_CONVERSIONS,
    ToneGeneratorDebugEnvironment::m_adc1Buffer,
    2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

DacService ToneGeneratorDebugEnvironment::m_dacService(&hdac);

ToneGeneratorDebugEnvironment::ToneGeneratorDebugEnvironment()
{
    // VSENS before ISENS so voltage dispatches first.
    m_adc1Service.addChannel(&m_ultrasonicVsensChannel);
    m_adc1Service.addChannel(&m_ultrasonicIsensChannel);

    m_dacService.addChannel(&m_ultrasonicDacChannel);

    addProcess(&m_dacService);
    addProcess(&m_adc1Service);

    m_ultrasonicVsensChannel.addMeasurementListenerCallback(
        this, &ToneGeneratorDebugEnvironment::onVoltageMeasured);
    m_ultrasonicIsensChannel.addMeasurementListenerCallback(
        this, &ToneGeneratorDebugEnvironment::onCurrentMeasured);

    // Register as a (non-exclusive) controller; activity is reported by the
    // callback's bool return, gated on m_active.
    m_ultrasonicVsensChannel.addFrequencyControllerCallback(
        this, &ToneGeneratorDebugEnvironment::onDemodulationFrequencyRequested);
    m_ultrasonicIsensChannel.addFrequencyControllerCallback(
        this, &ToneGeneratorDebugEnvironment::onDemodulationFrequencyRequested);
    m_ultrasonicDacChannel.addWaveformControllerCallback(
        this, &ToneGeneratorDebugEnvironment::onToneSample);
}

void ToneGeneratorDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        restartTone();
        break;

    case CMD_STOP:
        stopTone();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void ToneGeneratorDebugEnvironment::restartTone()
{
    if (m_active) {
        stopTone();
    }

    startTone();
}

void ToneGeneratorDebugEnvironment::resetResult()
{
    m_result.voltageMagnitude = 0.0f;
    m_result.voltageUpdates = 0;

    m_result.currentMagnitude = 0.0f;
    m_result.currentUpdates = 0;

    m_result.voltagePhasor = complexf_create(0.0f, 0.0f);
    m_result.currentPhasor = complexf_create(0.0f, 0.0f);
}

void ToneGeneratorDebugEnvironment::startTone()
{
    float frequency = arg(0);
    float amplitudeValue = arg(1);

    if (amplitudeValue <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if (frequency <= 0.0f) {
        frequency = PLL_MODULE_CENTER_FREQUENCY;
    }

    const float maxAmplitude =
        0.5f * static_cast<float>(DAC1_VOLTAGE_RANGE);

    if (amplitudeValue > maxAmplitude) {
        amplitudeValue = maxAmplitude;
    }

    m_amplitude = amplitudeValue;

    m_normalizedFrequency =
        frequency / static_cast<float>(DAC1_SAMPLING_FREQ);

    const float theta =
        2.0f * Pi * frequency * ADC_CHANNEL_US_VI_SKEW_SECONDS +
        ADC_CHANNEL_US_ISENS_PHASE_LEAD_RAD;

    m_currentSkewRotator = complexf_create(cosf(theta), -sinf(theta));

    resetResult();

    setBusy();
    setResultPointer(0, &m_result);

    m_ultrasonicVsensChannel.enable();
    m_ultrasonicIsensChannel.enable();

    m_ultrasonicDacChannel.start(m_amplitude,
                                 0.5f * static_cast<float>(DAC1_VOLTAGE_RANGE),
                                 m_normalizedFrequency);

    m_active = true;
}

void ToneGeneratorDebugEnvironment::stopTone()
{
    m_ultrasonicDacChannel.stop();
    m_ultrasonicVsensChannel.disable();
    m_ultrasonicIsensChannel.disable();

    m_active = false;
    setIdle();
}

void ToneGeneratorDebugEnvironment::abort()
{
    stopTone();
}

bool ToneGeneratorDebugEnvironment::onToneSample(
    void *context,
    float *amplitudeValue,
    float *average,
    float *targetNormalizedGeneratorFrequency)
{
    auto *self = static_cast<ToneGeneratorDebugEnvironment *>(context);

    if (self == nullptr) {
        return false;
    }

    return self->provideToneSample(amplitudeValue,
                                   average,
                                   targetNormalizedGeneratorFrequency);
}

bool ToneGeneratorDebugEnvironment::provideToneSample(
    float *amplitudeValue,
    float *average,
    float *targetNormalizedGeneratorFrequency)
{
    if (!m_active) {
        return false;
    }

    if (amplitudeValue != nullptr) {
        *amplitudeValue = m_amplitude;
    }

    if (average != nullptr) {
        *average = 0.5f * static_cast<float>(DAC1_VOLTAGE_RANGE);
    }

    if (targetNormalizedGeneratorFrequency != nullptr) {
        *targetNormalizedGeneratorFrequency = m_normalizedFrequency;
    }

    return true;
}

void ToneGeneratorDebugEnvironment::onVoltageMeasured(void *context,
                                                      float re,
                                                      float im)
{
    auto *self = static_cast<ToneGeneratorDebugEnvironment *>(context);

    if (self != nullptr) {
        self->handleVoltageMeasurement(re, im);
    }
}

void ToneGeneratorDebugEnvironment::handleVoltageMeasurement(float re,
                                                             float im)
{
    if (!m_active) {
        return;
    }

    m_result.voltagePhasor = complexf_create(re, im);
    m_result.voltageMagnitude = sqrtf(re * re + im * im);
    m_result.voltageUpdates = m_result.voltageUpdates + 1;
}

void ToneGeneratorDebugEnvironment::onCurrentMeasured(void *context,
                                                      float re,
                                                      float im)
{
    auto *self = static_cast<ToneGeneratorDebugEnvironment *>(context);

    if (self != nullptr) {
        self->handleCurrentMeasurement(re, im);
    }
}

void ToneGeneratorDebugEnvironment::handleCurrentMeasurement(float re,
                                                             float im)
{
    if (!m_active) {
        return;
    }

    const complexf rawCurrent = complexf_create(re, im);
    const complexf correctedCurrent =
        complexf_mul(rawCurrent, m_currentSkewRotator);

    m_result.currentPhasor = correctedCurrent;
    m_result.currentMagnitude = sqrtf(re * re + im * im);
    m_result.currentUpdates = m_result.currentUpdates + 1;
}

bool ToneGeneratorDebugEnvironment::onDemodulationFrequencyRequested(
    void *context,
    float *targetNormalizedIQFrequency)
{
    auto *self = static_cast<ToneGeneratorDebugEnvironment *>(context);

    if (self == nullptr) {
        return false;
    }

    return self->provideDemodulationFrequency(targetNormalizedIQFrequency);
}

bool ToneGeneratorDebugEnvironment::provideDemodulationFrequency(
    float *targetNormalizedIQFrequency)
{
    if (!m_active) {
        return false;
    }

    if (targetNormalizedIQFrequency != nullptr) {
        *targetNormalizedIQFrequency = m_normalizedFrequency;
    }

    return true;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_TONE_GENERATOR
