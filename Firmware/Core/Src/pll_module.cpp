#include "pll_module.hpp"
#include <cmath>

PllModule::PllModule(SineGeneratorChannel *sinusoid,
    IQDemodulatorChannel *voltageDemodulator,
    IQDemodulatorChannel *currentDemodulator,
    float samplingFrequency,
    float controlFrequency)
    : m_sinusoid(sinusoid)
    , m_voltageDemodulator(voltageDemodulator)
    , m_currentDemodulator(currentDemodulator)
    , m_frequencyController(PidController::Config{
        PLL_MODULE_FREQ_PID_GAIN,
        PLL_MODULE_FREQ_PID_INTEGRAL_TC,
        PLL_MODULE_FREQ_PID_DERIVATIVE_TC,
        1.0f / static_cast<float>(PLL_MODULE_CONTROL_FREQ),
        PLL_MODULE_FREQ_PID_FILTER_TC,
        PLL_MODULE_FREQ_PID_MIN_DEVIATION,
        PLL_MODULE_FREQ_PID_MAX_DEVIATION})
    , m_transferState(TransferState::Idle)
    , m_callbacks{}
    , m_callbackCount(0)
    , m_voltage(complexf_create(0.0f, 0.0f))
    , m_current(complexf_create(0.0f, 0.0f))
    , m_currentSkewRotator(complexf_create(1.0f, 0.0f))
    , m_voltageReady(false)
    , m_currentReady(false)
    , m_samplingFrequency(samplingFrequency)
    , m_controlFrequency(controlFrequency)
    , m_centerFrequency(0.0f)
    , m_targetBondingEnergy(0.0f)
    , m_maxBondingDuration(0.0f)
    , m_bondingDuration(0.0f)
    , m_bondingEnergy(0.0f)
    , m_freqQueueBuffer{}
    , m_freqQueue(m_freqQueueBuffer, PLL_MODULE_FREQ_CORRECTION_QUEUE_DEPTH)
    , m_driveAmplitude(0.0f)
    , m_targetNormalizedIQFrequency(0.0f)
    , m_telemetryBuffer(nullptr)
    , m_telemetryCapacity(0)
    , m_telemetryCount(0)
{
    m_voltageDemodulator->disable();
    m_currentDemodulator->disable();
}

bool PllModule::addEventListenerCallback(void *context, Callback cb)
{
    if (cb == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_callbackCount; i++) {
        if (m_callbacks[i].context == context && m_callbacks[i].callback == cb) {
            return true;
        }
    }

    if (m_callbackCount < kMaxCallbacks) {
        m_callbacks[m_callbackCount++] = CallbackRegistration{cb, context};
        return true;
    }

    return false;
}

void PllModule::publishEvent(Event event)
{
    for (uint8_t i = 0; i < m_callbackCount; i++) {
        m_callbacks[i].callback(m_callbacks[i].context, event);
    }
}

void PllModule::setTelemetryBuffer(TelemetrySample *buffer, uint16_t capacity)
{
    m_telemetryBuffer = buffer;
    m_telemetryCapacity = (buffer != nullptr) ? capacity : 0;
    m_telemetryCount = 0;
}

uint16_t PllModule::getTelemetryCount() const
{
    return m_telemetryCount;
}

float PllModule::getBondingEnergy() const
{
    return m_bondingEnergy;
}

float PllModule::getBondingDuration() const
{
    return m_bondingDuration;
}

float PllModule::getAveragePower() const
{
    return m_bondingDuration > 0.0f
        ? m_bondingEnergy / m_bondingDuration
        : 0.0f;
}

void PllModule::onStart()
{
    if (m_sinusoid == nullptr || m_voltageDemodulator == nullptr ||
        m_currentDemodulator == nullptr) {
        setProcessError();
        return;
    }

    if (!m_voltageDemodulator->addMeasurementListenerCallback(
            this, &PllModule::onVoltageMeasured) ||
        !m_currentDemodulator->addMeasurementListenerCallback(
            this, &PllModule::onCurrentMeasured) ||
        !m_voltageDemodulator->addFrequencyControllerCallback(
            this, &PllModule::onIqFrequencyRequested) ||
        !m_currentDemodulator->addFrequencyControllerCallback(
            this, &PllModule::onIqFrequencyRequested) ||
        !m_sinusoid->addWaveformControllerCallback(
            this, &PllModule::onSinusoidSample)) {
        setProcessError();
    }
}

void PllModule::onStop()
{
    abortTransfer();
}

bool PllModule::beginTransfer(
    float centerFrequency,
    float driveAmplitude,
    float bondingEnergyJoules,
    float maxBondingDurationSeconds)
{
    if (!isOperating() || m_transferState != TransferState::Idle) {
        return false;
    }

    if (driveAmplitude < 0.0f || bondingEnergyJoules <= 0.0f || maxBondingDurationSeconds <= 0.0f) {
        return false;
    }

    m_centerFrequency = centerFrequency;
    m_targetBondingEnergy = bondingEnergyJoules;
    m_maxBondingDuration = maxBondingDurationSeconds;

    m_voltage = complexf_create(0.0f, 0.0f);
    m_current = complexf_create(0.0f, 0.0f);

    // The I-sense phasor arrives advanced by 2*pi*f*skew (sequential ADC
    // ranks) plus the current-transformer phase lead; rotating by the
    // conjugate restores the true phase. The PID's +-1 kHz corrections
    // move this angle by < 0.3 deg, so the center frequency is accurate
    // enough.
    {
        float theta = 2.0f * (float)M_PI * centerFrequency *
                      ADC_CHANNEL_US_VI_SKEW_SECONDS +
                      ADC_CHANNEL_US_ISENS_PHASE_LEAD_RAD;
        m_currentSkewRotator = complexf_create(cosf(theta), -sinf(theta));
    }

    m_voltageReady = false;
    m_currentReady = false;

    m_bondingDuration = 0.0f;
    m_bondingEnergy = 0.0f;

    m_freqQueue.clear();
    m_freqQueue.enqueue(0.0f);
    m_freqQueue.enqueue(0.0f);

    m_telemetryCount = 0;

    // driveAmplitude is the requested transducer voltage amplitude
    // (physical volts, matching the scanner's units); the sine generator is
    // programmed in DAC volts.
    m_driveAmplitude = driveAmplitude / static_cast<float>(US_DRIVE_CHAIN_GAIN);

    // Cap below the guard in SineGeneratorChannel::updateParameters, which
    // would otherwise rewrite an over-range amplitude to full scale.
    const float dacCeiling = 0.5f * static_cast<float>(DAC1_VOLTAGE_RANGE);
    if (m_driveAmplitude > dacCeiling) {
        m_driveAmplitude = dacCeiling;
    }

    m_targetNormalizedIQFrequency = centerFrequency / m_samplingFrequency;

    m_frequencyController.start();

    m_transferState = TransferState::Transferring;
    m_voltageDemodulator->enable();
    m_currentDemodulator->enable();

    // Mid-rail average: the DAC cannot output below 0 V, and the sine
    // generator's clipping guard rewrites amplitude/average otherwise.
    m_sinusoid->start(m_driveAmplitude,
                      0.5f * DAC1_VOLTAGE_RANGE,
                      m_centerFrequency / m_samplingFrequency);
    return true;
}

void PllModule::abortTransfer()
{
    if (m_transferState != TransferState::Transferring) return;

    m_sinusoid->stop();

    m_voltageDemodulator->disable();
    m_currentDemodulator->disable();

    m_frequencyController.stop();

    m_voltageReady = false;
    m_currentReady = false;

    m_transferState = TransferState::Idle;
}

bool PllModule::isTransferring() const
{
    return m_transferState == TransferState::Transferring;
}

bool PllModule::onSinusoidSample(void *context, float *amplitude, float *average, float *targetNormalizedGeneratorFrequency)
{
    PllModule *self = static_cast<PllModule *>(context);

    if (self == nullptr || !self->isTransferring()) {
        return false;
    }

    if (amplitude != nullptr) {
        *amplitude = self->m_driveAmplitude;
    }

    if (average != nullptr) {
        *average = 0.5f * DAC1_VOLTAGE_RANGE;
    }

    uint16_t n_elements = self->m_freqQueue.getElementCount();
    float correction = self->m_freqQueue.peek(n_elements - 1);

    if (targetNormalizedGeneratorFrequency != nullptr) {
        *targetNormalizedGeneratorFrequency = (self->m_centerFrequency + correction) / self->m_samplingFrequency;
    }

    return true;
}

void PllModule::onVoltageMeasured(void *context, float re, float im)
{
    PllModule *self = static_cast<PllModule *>(context);
    if (self == nullptr || !self->isTransferring()) return;

    self->m_voltage = complexf_create(re, im);
    self->m_voltageReady = true;

    if (!self->m_voltageReady || !self->m_currentReady) {
        return;
    }

    self->updateController();
}

void PllModule::onCurrentMeasured(void *context, float re, float im)
{
    PllModule *self = static_cast<PllModule *>(context);
    if (self == nullptr || !self->isTransferring()) return;

    self->m_current = complexf_mul(complexf_create(re, im),
                                   self->m_currentSkewRotator);
    self->m_currentReady = true;

    if (!self->m_voltageReady || !self->m_currentReady) {
        return;
    }

    self->updateController();
}

bool PllModule::onIqFrequencyRequested(void *context, float *targetNormalizedIQFrequency)
{
    PllModule *self = static_cast<PllModule *>(context);

    if (self == nullptr || !self->isTransferring()) {
        return false;
    }

    if (targetNormalizedIQFrequency != nullptr) {
        *targetNormalizedIQFrequency = self->m_targetNormalizedIQFrequency;
    }

    return true;
}

void PllModule::updateController()
{
    m_voltageReady = false;
    m_currentReady = false;

    float real_power = measuredRealPower();
    float phase_error = measuredPhaseError();

    float new_correction = m_frequencyController.execute(0.0f, phase_error);

    if (m_telemetryBuffer != nullptr && m_telemetryCount < m_telemetryCapacity) {
        m_telemetryBuffer[m_telemetryCount] =
            TelemetrySample{phase_error, new_correction, real_power, m_bondingEnergy};
        m_telemetryCount = m_telemetryCount + 1;
    }

    float iq_frequency = m_centerFrequency + m_freqQueue.dequeue();
    m_freqQueue.enqueue(new_correction);

    float normalized_iq_frequency = iq_frequency / m_samplingFrequency;
    m_targetNormalizedIQFrequency = normalized_iq_frequency;

    // Retune both demodulators explicitly; the controller callback only
    // reaches whichever channel completed the measurement pair.
    m_voltageDemodulator->setDemodulationFrequency(normalized_iq_frequency);
    m_currentDemodulator->setDemodulationFrequency(normalized_iq_frequency);

    float control_period = 1.0f / m_controlFrequency;

    m_bondingEnergy += real_power * control_period;
    m_bondingDuration += control_period;

    if (m_bondingEnergy >= m_targetBondingEnergy) {
        abortTransfer();
        publishEvent(Event::BondingCompleted);
        return;
    }

    if (m_bondingDuration >= m_maxBondingDuration) {
        abortTransfer();
        publishEvent(Event::InsufficientBondingPower);
        return;
    }
}

float PllModule::measuredRealPower() const
{
    const complexf complex_power = complexf_mul(m_voltage, complexf_conj(m_current));

    return fabsf(complex_power.re);
}

float PllModule::measuredPhaseError() const
{
    const complexf complex_power = complexf_mul(m_voltage, complexf_conj(m_current));

    return atan2f(complex_power.im, complex_power.re);
}
