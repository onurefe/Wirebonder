#include "debug_tone_generator.hpp"
#include <cmath>
#include "configuration.h"

namespace {

constexpr float Pi = 3.14159265358979323846f;

} // namespace

DebugToneGenerator::DebugToneGenerator()
{
}

void DebugToneGenerator::init(SineGeneratorChannel *generator,
                            IQDemodulatorChannel *voltageDemodulator,
                            IQDemodulatorChannel *currentDemodulator)
{
    m_generator = generator;
    m_voltageDemodulator = voltageDemodulator;
    m_currentDemodulator = currentDemodulator;
}

void DebugToneGenerator::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        restart();
        break;

    case CMD_STOP:
        stop();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void DebugToneGenerator::restart()
{
    if (m_active) {
        stop();
    }

    start();
}

void DebugToneGenerator::resetResult()
{
    m_result.voltageMagnitude = 0.0f;
    m_result.voltageUpdates = 0;

    m_result.currentMagnitude = 0.0f;
    m_result.currentUpdates = 0;

    m_result.voltagePhasor = complexf_create(0.0f, 0.0f);
    m_result.currentPhasor = complexf_create(0.0f, 0.0f);
}

void DebugToneGenerator::start()
{
    if (m_generator == nullptr ||
        m_voltageDemodulator == nullptr ||
        m_currentDemodulator == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

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
        2.0f * Pi * frequency * ADC_CHANNEL_US_VI_SKEW_SECONDS;

    m_currentSkewRotator = complexf_create(cosf(theta), -sinf(theta));

    resetResult();

    setBusy();
    setResultPointer(0, &m_result);

    m_voltageDemodulator->addMeasurementListenerCallback(
        this,
        &DebugToneGenerator::onVoltageMeasured);

    m_currentDemodulator->addMeasurementListenerCallback(
        this,
        &DebugToneGenerator::onCurrentMeasured);

    // Register as a (non-exclusive) controller; activity is reported by the
    // callback's bool return, gated on m_active.
    m_voltageDemodulator->addFrequencyControllerCallback(
        this,
        &DebugToneGenerator::onDemodulationFrequencyRequested);
    m_currentDemodulator->addFrequencyControllerCallback(
        this,
        &DebugToneGenerator::onDemodulationFrequencyRequested);
    m_generator->addWaveformControllerCallback(this,
                                               &DebugToneGenerator::onToneSample);

    m_voltageDemodulator->enable();
    m_currentDemodulator->enable();

    m_generator->start(m_amplitude,
                       0.5f * static_cast<float>(DAC1_VOLTAGE_RANGE),
                       m_normalizedFrequency);

    m_active = true;
}

void DebugToneGenerator::stop()
{
    if (m_generator != nullptr) {
        m_generator->stop();
    }

    if (m_voltageDemodulator != nullptr) {
        m_voltageDemodulator->disable();
    }

    if (m_currentDemodulator != nullptr) {
        m_currentDemodulator->disable();
    }

    m_active = false;
    setIdle();
}

void DebugToneGenerator::abort()
{
    stop();
}

bool DebugToneGenerator::onToneSample(
    void *context,
    float *amplitudeValue,
    float *average,
    float *targetNormalizedGeneratorFrequency)
{
    auto *self = static_cast<DebugToneGenerator *>(context);

    if (self == nullptr) {
        return false;
    }

    return self->provideToneSample(amplitudeValue,
                                   average,
                                   targetNormalizedGeneratorFrequency);
}

bool DebugToneGenerator::provideToneSample(
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

void DebugToneGenerator::onVoltageMeasured(
    void *context,
    float re,
    float im)
{
    auto *self = static_cast<DebugToneGenerator *>(context);

    if (self != nullptr) {
        self->handleVoltageMeasurement(re,
                                       im);
    }
}

void DebugToneGenerator::handleVoltageMeasurement(
    float re,
    float im)
{
    if (!m_active) {
        return;
    }

    m_result.voltagePhasor = complexf_create(re, im);
    m_result.voltageMagnitude = sqrtf(re * re + im * im);
    m_result.voltageUpdates = m_result.voltageUpdates + 1;
}

void DebugToneGenerator::onCurrentMeasured(
    void *context,
    float re,
    float im)
{
    auto *self = static_cast<DebugToneGenerator *>(context);

    if (self != nullptr) {
        self->handleCurrentMeasurement(re,
                                       im);
    }
}

void DebugToneGenerator::handleCurrentMeasurement(
    float re,
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

bool DebugToneGenerator::onDemodulationFrequencyRequested(
    void *context,
    float *targetNormalizedIQFrequency)
{
    auto *self = static_cast<DebugToneGenerator *>(context);

    if (self == nullptr) {
        return false;
    }

    return self->provideDemodulationFrequency(targetNormalizedIQFrequency);
}

bool DebugToneGenerator::provideDemodulationFrequency(
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
