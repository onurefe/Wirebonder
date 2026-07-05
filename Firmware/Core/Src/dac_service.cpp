#include "dac_service.hpp"
#include "arm_math.h"

// =======================================================================
// SineGeneratorChannel
// =======================================================================
SineGeneratorChannel::SineGeneratorChannel(
    DAC_HandleTypeDef *hdac,
    TIM_HandleTypeDef *triggerTimer,
    uint32_t halChannel,
    uint16_t *buffer,
    uint32_t bufferSize,
    uint32_t updatePeriodInNumSamples,
    uint8_t dacBits,
    float dacVoltageRange)
    : m_hdac(hdac)
    , m_triggerTimer(triggerTimer)
    , m_halChannel(halChannel)
    , m_buffer(buffer)
    , m_bufferSize(bufferSize)
    , m_phasor{1.0f, 0.0f}
    , m_rotatorStep{1.0f, 0.0f}
    , m_sampleCounter(0)
    , m_amplitude(0.0f)
    , m_average(0.0f)
    , m_normalizedFrequency(0.0f)
    , m_updatePeriodInNumSamples(updatePeriodInNumSamples)
    , m_dacVoltageRange(dacVoltageRange)
    , m_dacBits(dacBits)
    , m_waveformControllerCallbacks{}
    , m_waveformControllerCallbackCount(0)
{
}

bool SineGeneratorChannel::addWaveformControllerCallback(void *context, SinusoidCallback cb)
{
    if (cb == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_waveformControllerCallbackCount; i++) {
        if (m_waveformControllerCallbacks[i].context == context &&
            m_waveformControllerCallbacks[i].callback == cb) {
            return true;
        }
    }

    if (m_waveformControllerCallbackCount >= kMaxWaveformControllerCallbacks) {
        return false;
    }

    m_waveformControllerCallbacks[m_waveformControllerCallbackCount++] =
        WaveformControllerRegistration{cb, context};
    return true;
}

uint32_t SineGeneratorChannel::getHalChannel(void)
{
    return m_halChannel;
}

DAC_HandleTypeDef *SineGeneratorChannel::getDACHandle(void)
{
    return m_hdac;
}

void SineGeneratorChannel::start(float amplitude, float average, float normalizedFrequency)
{
    m_amplitude             = amplitude;
    m_average               = average;
    m_normalizedFrequency   = normalizedFrequency;

    calculateRotator();

    uint32_t halfSize = m_bufferSize / 2;
    fillBuffer(m_buffer, halfSize);
    fillBuffer(&m_buffer[halfSize], halfSize);
    m_sampleCounter = 0;

    m_isActive = true;

    HAL_DAC_Start_DMA(
        m_hdac,
        m_halChannel,
        reinterpret_cast<uint32_t *>(m_buffer),
        m_bufferSize,
        DAC_ALIGN_12B_R);

    if (m_triggerTimer != nullptr) {
        HAL_TIM_Base_Start(m_triggerTimer);
    }
}

void SineGeneratorChannel::stop()
{
    m_isActive = false;

    HAL_DAC_Stop_DMA(m_hdac, m_halChannel);

    if (m_triggerTimer != nullptr) {
        HAL_TIM_Base_Stop(m_triggerTimer);
    }
}

void SineGeneratorChannel::setFrequency(float normalizedFrequency)
{
    InterruptLock lock;

    m_normalizedFrequency = normalizedFrequency;
    calculateRotator();
}

void SineGeneratorChannel::calculateRotator()
{
    m_rotatorStep.re = arm_cos_f32(2.0f * PI * m_normalizedFrequency);
    m_rotatorStep.im = arm_sin_f32(2.0f * PI * m_normalizedFrequency);
}

void SineGeneratorChannel::updateParameters()
{
    // First active controller supplies the waveform parameters; if none is
    // active amplitude/average/frequency are left unchanged.
    for (uint8_t i = 0; i < m_waveformControllerCallbackCount; i++) {
        if (m_waveformControllerCallbacks[i].callback(
                m_waveformControllerCallbacks[i].context,
                &m_amplitude,
                &m_average,
                &m_normalizedFrequency)) {
            break;
        }
    }

    // Prevent DAC clipping.
    if ((m_average + m_amplitude) > m_dacVoltageRange ||
        (m_average - m_amplitude) < 0.0f) {
        m_amplitude = m_dacVoltageRange * 0.5f;
        m_average   = m_dacVoltageRange * 0.5f;
    }

    calculateRotator();

    // Deliberately NOT resetting m_phasor: the output must stay
    // phase-continuous across segment updates. Resetting to (1,0) here
    // put a phase discontinuity at every segment boundary for any
    // frequency that is not an integer number of cycles per segment
    // (e.g. 59.51 kHz at 360-sample segments), smearing the spectrum and
    // collapsing the coherent demodulation. fillBuffer's renormalization
    // keeps the phasor magnitude bounded.
    m_sampleCounter = 0;
}

void SineGeneratorChannel::fillBuffer(uint16_t *buffer, uint32_t count)
{
    const float dacOffset = voltsToDacCode(m_average,   m_dacBits, m_dacVoltageRange);
    const float dacAmp    = voltsToDacCode(m_amplitude, m_dacBits, m_dacVoltageRange);

    for (uint32_t i = 0; i < count; i++) {
        buffer[i] = static_cast<uint16_t>(dacOffset + dacAmp * m_phasor.re);
        m_phasor  = complexf_mul(m_phasor, m_rotatorStep);
    }

    float magSq      = complexf_abs2(m_phasor);
    float adj        = 1.5f - 0.5f * magSq;
    m_phasor.re      = m_phasor.re * adj;
    m_phasor.im      = m_phasor.im * adj;

    m_sampleCounter += count;
}

void SineGeneratorChannel::bufferFillRequestHandler(bool secondHalf)
{
    if (m_buffer == nullptr || m_bufferSize == 0 || m_normalizedFrequency <= 0.0f) {
        return;
    }

    uint32_t halfSize  = m_bufferSize / 2;
    uint16_t *writePtr = secondHalf ? &m_buffer[halfSize] : m_buffer;

    uint32_t num_update_samples = (m_updatePeriodInNumSamples > 0U) ? m_updatePeriodInNumSamples : 1U;

    uint32_t num_written_samples = 0;

    while (num_written_samples < halfSize) {
        if (m_sampleCounter >= num_update_samples) {
            updateParameters();
        }

        uint32_t samplesNeeded = num_update_samples - m_sampleCounter;
        uint32_t available     = halfSize - num_written_samples;
        uint32_t chunk         = (available < samplesNeeded) ? available : samplesNeeded;

        fillBuffer(writePtr + num_written_samples, chunk);
        num_written_samples += chunk;
    }
}

// =======================================================================
// RawDacChannel
// =======================================================================
RawDacChannel::RawDacChannel(DAC_HandleTypeDef *hdac,
                              TIM_HandleTypeDef *triggerTimer,
                              uint32_t halChannel,
                              uint16_t *dmaBuffer,
                              uint32_t dmaBufferSize,
                              const uint16_t *synthesisBuffer,
                              uint32_t synthesisBufferSize)
    : m_hdac(hdac)
    , m_triggerTimer(triggerTimer)
    , m_halChannel(halChannel)
    , m_dmaBuffer(dmaBuffer)
    , m_dmaBufferSize(dmaBufferSize)
    , m_synthesisBuffer(synthesisBuffer)
    , m_synthesisBufferSize(synthesisBufferSize)
    , m_synthesisOffset(0)
{
}

uint32_t RawDacChannel::getHalChannel(void) { return m_halChannel; }
DAC_HandleTypeDef *RawDacChannel::getDACHandle(void) { return m_hdac; }

void RawDacChannel::start()
{
    m_synthesisOffset = 0;

    bufferFillRequestHandler(false);
    bufferFillRequestHandler(true);

    m_isActive = true;

    HAL_DAC_Start_DMA(
        m_hdac,
        m_halChannel,
        reinterpret_cast<uint32_t *>(m_dmaBuffer),
        m_dmaBufferSize,
        DAC_ALIGN_12B_R);

    if (m_triggerTimer != nullptr) {
        HAL_TIM_Base_Start(m_triggerTimer);
    }
}

void RawDacChannel::stop()
{
    m_isActive = false;

    HAL_DAC_Stop_DMA(m_hdac, m_halChannel);

    if (m_triggerTimer != nullptr) {
        HAL_TIM_Base_Stop(m_triggerTimer);
    }
}

void RawDacChannel::bufferFillRequestHandler(bool secondHalf)
{
    uint32_t halfSize = m_dmaBufferSize / 2;
    uint16_t *dst     = secondHalf ? &m_dmaBuffer[halfSize] : m_dmaBuffer;

    for (uint32_t i = 0; i < halfSize; i++) {
        dst[i]            = m_synthesisBuffer[m_synthesisOffset];
        m_synthesisOffset = (m_synthesisOffset + 1) % m_synthesisBufferSize;
    }
}

// =======================================================================
// DacService
// =======================================================================
static DacService *g_instance = nullptr;

DacService::DacService(DAC_HandleTypeDef *hdac)
    : m_numChannels(0)
    , m_state(ServiceState::READY)
{
    (void)hdac;
    for (uint8_t i = 0; i < DAC_SERVICE_MAX_CHANNELS; i++) {
        m_channels[i] = nullptr;
    }

    g_instance = this;
}

bool DacService::addChannel(IDacChannel *channel)
{
    if (channel == nullptr) {
        return false;
    }

    if (m_numChannels >= DAC_SERVICE_MAX_CHANNELS) {
        return false;
    }

    m_channels[m_numChannels++] = channel;

    return true;
}

void DacService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    m_state = ServiceState::OPERATING;
}

void DacService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0; i < m_numChannels; i++) {
        if (m_channels[i] != nullptr && m_channels[i]->isActive()) {
            m_channels[i]->stop();
        }
    }

    m_state = ServiceState::READY;
}

IDacChannel *DacService::findChannel(DAC_HandleTypeDef *hdac, uint32_t halChannel)
{
    if (g_instance == nullptr || g_instance->m_state != ServiceState::OPERATING) {
        return nullptr;
    }

    for (uint8_t i = 0; i < g_instance->m_numChannels; i++) {
        IDacChannel *ch = g_instance->m_channels[i];
        if (ch->isActive() &&
            ch->getDACHandle()->Instance == hdac->Instance &&
            ch->getHalChannel() == halChannel) {
            return ch;
        }
    }

    return nullptr;
}

// =======================================================================
// HAL DMA callbacks
// =======================================================================
extern "C" {
    void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
    {
        if (IDacChannel *ch = DacService::findChannel(hdac, DAC_CHANNEL_1)) {
            ch->bufferFillRequestHandler(false);
        }
    }

    void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
    {
        if (IDacChannel *ch = DacService::findChannel(hdac, DAC_CHANNEL_1)) {
            ch->bufferFillRequestHandler(true);
        }
    }

    void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
    {
        if (IDacChannel *ch = DacService::findChannel(hdac, DAC_CHANNEL_2)) {
            ch->bufferFillRequestHandler(false);
        }
    }

    void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
    {
        if (IDacChannel *ch = DacService::findChannel(hdac, DAC_CHANNEL_2)) {
            ch->bufferFillRequestHandler(true);
        }
    }
}
