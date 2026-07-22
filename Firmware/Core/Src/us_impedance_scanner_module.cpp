#include "us_impedance_scanner_module.hpp"
#include "arm_math.h"

UsImpedanceScannerModule::UsImpedanceScannerModule(
    RawDacChannel *dacChannel,
    RawAdcChannel *vChannel,
    RawAdcChannel *iChannel,
    uint16_t *synthesisBuffer,
    uint32_t  synthesisBufferSize,
    uint16_t  numFrequencies,
    float     minFrequency,
    float     frequencyStep,
    float     dacSamplingFrequency,
    uint8_t   dacBits,
    float     dacVoltageRange,
    float     adcSamplingFrequency,
    uint8_t   adcBits,
    float     adcVoltageRange,
    float     vGain,
    float     iGain,
    uint16_t  warmupIterations)
    : m_dacChannel(dacChannel)
    , m_vChannel(vChannel)
    , m_iChannel(iChannel)
    , m_synthesisBuffer(synthesisBuffer)
    , m_synthesisBufferSize(synthesisBufferSize)
    , m_dacSamplingFrequency(dacSamplingFrequency)
    , m_dacBits(dacBits)
    , m_dacVoltageRange(dacVoltageRange)
    , m_numFrequencies(numFrequencies)
    , m_minFrequency(minFrequency)
    , m_frequencyStep(frequencyStep)
    , m_adcSamplingFrequency(adcSamplingFrequency)
    , m_adcBits(adcBits)
    , m_adcVoltageRange(adcVoltageRange)
    , m_vGain(vGain)
    , m_iGain(iGain)
    , m_scanState(ScanState::Idle)
    , m_warmupIterations(warmupIterations)
    , m_vWarmupRemaining(0)
    , m_iWarmupRemaining(0)
    , m_vReady(false)
    , m_iReady(false)
    , m_vCapture(nullptr)
    , m_iCapture(nullptr)
    , m_captureLength(0)
    , m_voltagePhasors(nullptr)
    , m_currentPhasors(nullptr)
    , m_impedances(nullptr)
    , m_callbacks{}
    , m_callbackCount(0)
{
    updateSynthesisBuffer();

    m_vChannel->disable();
    m_iChannel->disable();
}

// ---------------------------------------------------------------------------
// Public: update scan parameters and re-compute synthesis waveform
// ---------------------------------------------------------------------------
void UsImpedanceScannerModule::setScanParameters(uint16_t numFrequencies, float minFrequency, float frequencyStep)
{
    if (isScanning()) return;

    // The synthesis buffer loops every m_synthesisBufferSize samples, so
    // only frequencies on the resulting grid are generated coherently
    // (off-grid tones get a phase discontinuity at every buffer wrap and
    // smear leakage over the whole sweep). Snap to the grid.
    float bin = m_dacSamplingFrequency / static_cast<float>(m_synthesisBufferSize);
    minFrequency = bin * roundf(minFrequency / bin);
    frequencyStep = bin * roundf(frequencyStep / bin);
    if (frequencyStep < bin) {
        frequencyStep = bin;
    }

    if (m_numFrequencies == numFrequencies &&
        m_minFrequency   == minFrequency   &&
        m_frequencyStep  == frequencyStep) {
        return;
    }

    m_numFrequencies = numFrequencies;
    m_minFrequency   = minFrequency;
    m_frequencyStep  = frequencyStep;

    updateSynthesisBuffer();
}

// ---------------------------------------------------------------------------
// Public: re-compute multi-tone synthesis waveform (also called from ctor)
// ---------------------------------------------------------------------------
// One sample of the unit-per-tone multitone. Newman phases (pi*k^2/N)
// spread the tone alignment so the crest stays near sqrt(2N) instead of N,
// letting every tone run correspondingly hotter in the same DAC range.
float UsImpedanceScannerModule::synthesisSample(uint32_t n) const
{
    float sum = 0.0f;

    for (uint16_t k = 0; k < m_numFrequencies; k++) {
        float freq  = m_minFrequency + static_cast<float>(k) * m_frequencyStep;
        float phase = PI * static_cast<float>(k) * static_cast<float>(k) /
                      static_cast<float>(m_numFrequencies);
        sum += arm_sin_f32(2.0f * PI * freq * static_cast<float>(n) /
                               m_dacSamplingFrequency +
                           phase);
    }

    return sum;
}

void UsImpedanceScannerModule::updateSynthesisBuffer()
{
    float midCode  = IDacChannel::voltsToDacCode(m_dacVoltageRange * 0.5f, m_dacBits, m_dacVoltageRange);
    uint16_t maxCode = static_cast<uint16_t>((1u << m_dacBits) - 1u);

    // First pass: measure the actual multitone peak (the crest-factor
    // formula is only approximate for a frequency grid offset from DC).
    float peak = 0.0f;
    for (uint32_t n = 0; n < m_synthesisBufferSize; n++) {
        float magnitude = fabsf(synthesisSample(n));
        if (magnitude > peak) {
            peak = magnitude;
        }
    }

    if (peak <= 0.0f) {
        for (uint32_t n = 0; n < m_synthesisBufferSize; n++) {
            m_synthesisBuffer[n] = static_cast<uint16_t>(midCode);
        }
        return;
    }

    // Scale so the measured peak lands at the configured fraction of the
    // half range; the clamp below only catches rounding.
    float ampCode = midCode * SCANNER_SYNTHESIS_PEAK_HEADROOM / peak;

    for (uint32_t n = 0; n < m_synthesisBufferSize; n++) {
        int32_t code = static_cast<int32_t>(
            midCode + ampCode * synthesisSample(n) + 0.5f);
        if (code < 0)                               code = 0;
        if (static_cast<uint32_t>(code) > maxCode)  code = static_cast<int32_t>(maxCode);
        m_synthesisBuffer[n] = static_cast<uint16_t>(code);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool UsImpedanceScannerModule::addScanCompleteListenerCallback(void *context, Callback cb)
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

void UsImpedanceScannerModule::onStart()
{
    if (m_dacChannel == nullptr || m_vChannel == nullptr ||
        m_iChannel == nullptr) {
        setProcessError();
        return;
    }

    if (!m_vChannel->addCaptureCompleteListenerCallback(
            this, onVoltageCaptureDone) ||
        !m_iChannel->addCaptureCompleteListenerCallback(
            this, onCurrentCaptureDone)) {
        setProcessError();
    }
}

void UsImpedanceScannerModule::onStop()
{
    abortScan();
}

bool UsImpedanceScannerModule::beginScan(
    complexf *voltagePhasors,
    complexf *currentPhasors,
    complexf *impedances)
{
    if (!isOperating() || isScanning() || voltagePhasors == nullptr ||
        currentPhasors == nullptr || impedances == nullptr) {
        return false;
    }

    m_voltagePhasors = voltagePhasors;
    m_currentPhasors = currentPhasors;
    m_impedances     = impedances;
    m_vWarmupRemaining = m_warmupIterations;
    m_iWarmupRemaining = m_warmupIterations;
    m_vReady           = false;
    m_iReady           = false;
    m_vCapture         = nullptr;
    m_iCapture         = nullptr;

    m_vChannel->reset();
    m_iChannel->reset();
    m_scanState = ScanState::Scanning;
    m_vChannel->enable();
    m_iChannel->enable();

    m_dacChannel->start();
    return true;
}

void UsImpedanceScannerModule::onExecute()
{
    if (!isScanning()) return;
    if (!m_vReady || !m_iReady) return;

    m_dacChannel->stop();

    m_vChannel->disable();
    m_iChannel->disable();

    analyzeChannel(m_vCapture, m_captureLength, m_vGain, m_voltagePhasors);
    analyzeChannel(m_iCapture, m_captureLength, m_iGain, m_currentPhasors);

    // Undo the V/I ADC sequencing skew: the I-sense rank converts
    // ADC_CHANNEL_US_VI_SKEW_SECONDS after V-sense, advancing its phase
    // by 2*pi*f*skew (frequency-dependent across the sweep), plus the
    // frequency-independent current-transformer phase lead.
    for (uint16_t k = 0; k < m_numFrequencies; k++) {
        float freq  = m_minFrequency + static_cast<float>(k) * m_frequencyStep;
        float theta = 2.0f * PI * freq * ADC_CHANNEL_US_VI_SKEW_SECONDS +
                      ADC_CHANNEL_US_ISENS_PHASE_LEAD_RAD;
        m_currentPhasors[k] = complexf_mul(
            m_currentPhasors[k],
            complexf_create(arm_cos_f32(theta), -arm_sin_f32(theta)));
    }

    computeImpedances();

    m_scanState = ScanState::Idle;

    publishScanComplete();
}

void UsImpedanceScannerModule::abortScan()
{
    if (!isScanning()) return;

    m_dacChannel->stop();

    m_vChannel->disable();
    m_iChannel->disable();

    m_vReady = false;
    m_iReady = false;
    m_scanState = ScanState::Idle;
}

bool UsImpedanceScannerModule::isScanning() const
{
    return m_scanState == ScanState::Scanning;
}

// ---------------------------------------------------------------------------
// Static ADC callbacks (ISR context)
// ---------------------------------------------------------------------------
void UsImpedanceScannerModule::onVoltageCaptureDone(void *context, uint16_t *buffer, uint32_t numSamples)
{
    UsImpedanceScannerModule *self = static_cast<UsImpedanceScannerModule *>(context);
    if (!self->isScanning() || self->m_vReady) return;

    if (self->m_vWarmupRemaining > 0) {
        self->m_vWarmupRemaining--;
        self->m_vChannel->reset();
        return;
    }

    self->m_vCapture      = buffer;
    self->m_captureLength = numSamples;
    self->m_vReady        = true;
}

void UsImpedanceScannerModule::onCurrentCaptureDone(void *context, uint16_t *buffer, uint32_t numSamples)
{
    UsImpedanceScannerModule *self = static_cast<UsImpedanceScannerModule *>(context);
    if (!self->isScanning() || self->m_iReady) return;

    if (self->m_iWarmupRemaining > 0) {
        self->m_iWarmupRemaining--;
        self->m_iChannel->reset();
        return;
    }

    self->m_iCapture = buffer;
    self->m_iReady   = true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------
void UsImpedanceScannerModule::analyzeChannel(const uint16_t *samples, uint32_t n, float gain, complexf *phasors)
{
    float codeToPhysical = IAdcChannel::adcToVolts(1.0f, m_adcBits, m_adcVoltageRange) * gain;
    float normScale      = codeToPhysical * 2.0f / static_cast<float>(n);

    for (uint16_t k = 0; k < m_numFrequencies; k++) {
        float freq  = m_minFrequency + static_cast<float>(k) * m_frequencyStep;
        float omega = 2.0f * PI * freq / m_adcSamplingFrequency;
        float coeff = 2.0f * arm_cos_f32(omega);
        float s1 = 0.0f, s2 = 0.0f;

        for (uint32_t i = 0; i < n; i++) {
            float s0 = static_cast<float>(samples[i]) + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        phasors[k].re = (s1 * arm_cos_f32(omega) - s2) * normScale;
        phasors[k].im = (s1 * arm_sin_f32(omega))       * normScale;
    }
}

void UsImpedanceScannerModule::computeImpedances()
{
    for (uint16_t i = 0; i < m_numFrequencies; i++) {
        m_impedances[i] = complexf_div(m_voltagePhasors[i], m_currentPhasors[i]);
    }
}

void UsImpedanceScannerModule::publishScanComplete()
{
    for (uint8_t i = 0; i < m_callbackCount; i++) {
        m_callbacks[i].callback(
            m_callbacks[i].context,
            m_voltagePhasors,
            m_currentPhasors,
            m_impedances);
    }
}
