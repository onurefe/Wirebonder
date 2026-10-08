#include "adc_service.hpp"
#include "configuration.h"
#include "arm_math.h"

// =======================================================================
// InterleavedBuffer
// =======================================================================
InterleavedBuffer::InterleavedBuffer(volatile uint16_t *buffer,
                                     uint16_t offset,
                                     uint16_t interleave)
    : m_buffer(buffer)
    , m_iterator(offset)
    , m_offset(offset)
    , m_interleave(interleave)
{
}

void InterleavedBuffer::resetIterator(void)
{
    m_iterator = m_offset;
}

void InterleavedBuffer::setIterator(uint16_t idx)
{
    m_iterator = m_offset + idx * m_interleave;
}

// =======================================================================
// AnalogChannel
// =======================================================================
AnalogChannel::AnalogChannel(uint16_t conversionOrder, uint32_t oversampling, float gain, float bias)
    : m_conversionOrder(conversionOrder)
    , m_oversampling(oversampling)
    , m_gain(gain)
    , m_bias(bias)
    , m_sum(0.0f)
    , m_sampleCounter(0)
{
}

bool AnalogChannel::addMeasurementListenerCallback(void *context, AnalogCallback cb)
{
    return m_callbacks.add(context, cb);
}

bool AnalogChannel::removeMeasurementListenerCallback(void *context, AnalogCallback cb)
{
    return m_callbacks.remove(context, cb);
}

uint16_t AnalogChannel::getConversionOrder() const { return m_conversionOrder; }

void AnalogChannel::report(uint8_t bits, float voltageRange)
{
    if (!m_callbacks.isEmpty()) {
        float avg = m_sum / (float)m_oversampling;
        float val = adcToVolts(avg, bits, voltageRange) * m_gain + m_bias;
        m_callbacks.invoke(val);
    }
}

void AnalogChannel::resetAccumulators(void)
{
    m_sum          = 0.0f;
    m_sampleCounter = 0;
}

void AnalogChannel::process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange)
{
    uint32_t processed = 0;

    buffer.setIterator(0);

    while (processed < numSamples) {
        uint32_t remaining = m_oversampling - m_sampleCounter;
        uint32_t available = numSamples - processed;
        uint32_t chunk     = (available < remaining) ? available : remaining;

        for (uint32_t i = 0; i < chunk; i++) {
            m_sum += static_cast<float>(buffer.iterate());
            m_sampleCounter++;
        }

        processed += chunk;

        if (m_sampleCounter >= m_oversampling) {
            report(bits, voltageRange);
            resetAccumulators();
        }
    }

    buffer.resetIterator();
}

// =======================================================================
// RawAdcChannel
// =======================================================================
RawAdcChannel::RawAdcChannel(uint16_t conversionOrder, uint16_t *buffer, uint32_t bufferSize)
    : m_conversionOrder(conversionOrder)
    , m_buffer(buffer)
    , m_bufferSize(bufferSize)
    , m_writeIndex(0)
    , m_done(false)
{
}

bool RawAdcChannel::addCaptureCompleteListenerCallback(void *context, Callback cb)
{
    return m_callbacks.add(context, cb);
}

bool RawAdcChannel::removeCaptureCompleteListenerCallback(void *context, Callback cb)
{
    return m_callbacks.remove(context, cb);
}

void RawAdcChannel::reset()
{
    m_writeIndex = 0;
    m_done       = false;
}

uint16_t RawAdcChannel::getConversionOrder() const { return m_conversionOrder; }

void RawAdcChannel::process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange)
{
    (void)bits;
    (void)voltageRange;

    if (m_callbacks.isEmpty() || m_done) return;

    buffer.setIterator(0);

    for (uint32_t i = 0; i < numSamples; i++) {
        m_buffer[m_writeIndex++] = buffer.iterate();

        if (m_writeIndex >= m_bufferSize) {
            m_done = true;
            m_callbacks.invoke(m_buffer, m_bufferSize);
            break;
        }
    }

    buffer.resetIterator();
}

// =======================================================================
// IQDemodulatorChannel
// =======================================================================
IQDemodulatorChannel::IQDemodulatorChannel(uint16_t conversionOrder, uint32_t samplesPerMeasurement, float targetFreq, float gain)
    : m_conversionOrder(conversionOrder)
    , m_normalizedFrequency(targetFreq)
    , m_gain(gain)
    , m_s1(0.0f)
    , m_s2(0.0f)
    , m_coeff(0.0f)
    , m_cosOmega(1.0f)
    , m_sinOmega(0.0f)
    , m_sampleCounter(0)
    , m_samplesPerMeasurement(samplesPerMeasurement)
    , m_dropLast(0)
{
    updateDemodulator();
    resetAccumulators();
}

bool IQDemodulatorChannel::addMeasurementListenerCallback(void *context, MeasurementListenerCallback cb)
{
    return m_measurementListenerCallbacks.add(context, cb);
}

bool IQDemodulatorChannel::removeMeasurementListenerCallback(void *context, MeasurementListenerCallback cb)
{
    return m_measurementListenerCallbacks.remove(context, cb);
}

bool IQDemodulatorChannel::addFrequencyControllerCallback(void *context, FrequencyControllerCallback cb)
{
    return m_frequencyControllerCallbacks.add(context, cb);
}

bool IQDemodulatorChannel::removeFrequencyControllerCallback(void *context, FrequencyControllerCallback cb)
{
    return m_frequencyControllerCallbacks.remove(context, cb);
}

uint16_t IQDemodulatorChannel::getConversionOrder() const { return m_conversionOrder; }

void IQDemodulatorChannel::updateDemodulator(void)
{
    float omega = 2.0f * PI * m_normalizedFrequency;
    m_cosOmega  = arm_cos_f32(omega);
    m_sinOmega  = arm_sin_f32(omega);
    m_coeff     = 2.0f * m_cosOmega;

    if (m_normalizedFrequency > 0.0f) {
        uint32_t completeCycles   = static_cast<uint32_t>(static_cast<float>(m_samplesPerMeasurement) * m_normalizedFrequency);
        uint32_t effectiveSamples = static_cast<uint32_t>(static_cast<float>(completeCycles) / m_normalizedFrequency + 0.5f);
        if (effectiveSamples > m_samplesPerMeasurement) {
            effectiveSamples = m_samplesPerMeasurement;
        }
        m_dropLast = m_samplesPerMeasurement - effectiveSamples;
    } else {
        m_dropLast = 0;
    }
}

void IQDemodulatorChannel::resetAccumulators(void)
{
    m_s1            = 0.0f;
    m_s2            = 0.0f;
    m_sampleCounter = 0;
}

void IQDemodulatorChannel::report(uint8_t bits, float voltageRange, float *targetNormalizedFrequency)
{
    uint32_t effectiveSamples = m_samplesPerMeasurement - m_dropLast;
    float normScale = adcToVolts(1.0f, bits, voltageRange) * m_gain * 2.0f / static_cast<float>(effectiveSamples);
    float re = (m_s1 * m_cosOmega - m_s2) * normScale;
    float im = (m_s1 * m_sinOmega)         * normScale;

    m_measurementListenerCallbacks.invoke(re, im);

    // First active controller supplies the demodulation frequency; if none is
    // active the pointed-to value is left unchanged (no retune).
    (void)m_frequencyControllerCallbacks.invokeFirst(targetNormalizedFrequency);
}

void IQDemodulatorChannel::setDemodulationFrequency(float normalizedFrequency)
{
    InterruptLock lock;

    m_normalizedFrequency = normalizedFrequency;
    updateDemodulator();
    resetAccumulators();
}

void IQDemodulatorChannel::process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange)
{
    uint32_t processed        = 0;
    uint32_t effectiveSamples = m_samplesPerMeasurement - m_dropLast;

    buffer.setIterator(0);

    while (processed < numSamples) {
        uint32_t remaining = m_samplesPerMeasurement - m_sampleCounter;
        uint32_t available = numSamples - processed;
        uint32_t chunk     = (available < remaining) ? available : remaining;

        uint32_t goertzelCount;
        uint32_t skipCount;

        if (m_sampleCounter >= effectiveSamples) {
            goertzelCount = 0;
            skipCount     = chunk;
        } else if (m_sampleCounter + chunk <= effectiveSamples) {
            goertzelCount = chunk;
            skipCount     = 0;
        } else {
            goertzelCount = effectiveSamples - m_sampleCounter;
            skipCount     = chunk - goertzelCount;
        }

        float s1    = m_s1;
        float s2    = m_s2;
        float coeff = m_coeff;

        for (uint32_t i = 0; i < goertzelCount; i++) {
            float s0 = static_cast<float>(buffer.iterate()) + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        m_s1 = s1;
        m_s2 = s2;

        for (uint32_t i = 0; i < skipCount; i++) {
            buffer.iterate();
        }

        m_sampleCounter += chunk;
        processed       += chunk;

        if (m_sampleCounter >= m_samplesPerMeasurement) {
            report(bits, voltageRange, &m_normalizedFrequency);
            updateDemodulator();
            effectiveSamples = m_samplesPerMeasurement - m_dropLast;
            resetAccumulators();
        }
    }

    buffer.resetIterator();
}

// =======================================================================
// AdcTickSyncChannel
// =======================================================================
AdcTickSyncChannel::AdcTickSyncChannel() = default;

bool AdcTickSyncChannel::addTickListenerCallback(void *context, TickListenerCallback cb)
{
    return m_callbacks.add(context, cb);
}

bool AdcTickSyncChannel::removeTickListenerCallback(void *context, TickListenerCallback cb)
{
    return m_callbacks.remove(context, cb);
}

uint16_t AdcTickSyncChannel::getConversionOrder() const { return 0; }

void AdcTickSyncChannel::process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange)
{
    (void)buffer;
    (void)numSamples;
    (void)bits;
    (void)voltageRange;

    m_callbacks.invoke();
}

// =======================================================================
// AdcService
// =======================================================================
static AdcService *g_registry[ADC_SERVICE_MAX_HANDLES] = {nullptr};
static uint8_t    g_numControllers = 0;

AdcService::AdcService(
        ADC_HandleTypeDef *hadc,
        TIM_HandleTypeDef *htim,
        uint8_t bits,
        float voltageRange,
        uint8_t numConversions,
        uint16_t *buffer,
        uint32_t bufferSize)
    : m_hadc(hadc)
    , m_htim(htim)
    , m_buffer(buffer)
    , m_bufferSize(bufferSize)
    , m_numChannels(0)
    , m_interleave(numConversions)
    , m_bits(bits)
    , m_voltageRange(voltageRange)
{
    if (g_numControllers < ADC_SERVICE_MAX_HANDLES) {
        g_registry[g_numControllers++] = this;
    }
}

AdcService *AdcService::getController(ADC_HandleTypeDef *hadc)
{
    for (uint8_t i = 0; i < g_numControllers; i++) {
        if (g_registry[i]->isOperating() &&
            g_registry[i]->m_hadc->Instance == hadc->Instance) return g_registry[i];
    }
    return nullptr;
}

bool AdcService::addChannel(IAdcChannel *channel)
{
    if (m_numChannels >= ADC_SERVICE_MAX_CHANNELS) {
        return false;
    }

    m_channels[m_numChannels++] = channel;
    return true;
}

void AdcService::setBuffer(uint16_t *buffer, uint32_t bufferSize)
{
    if (!isReady() || buffer == nullptr || bufferSize == 0U) return;
    m_buffer = buffer;
    m_bufferSize = bufferSize;
}

void AdcService::onStart()
{
    if (m_buffer == nullptr || m_bufferSize == 0U) {
        setProcessError();
        return;
    }
    
    HAL_ADC_Start_DMA(m_hadc, (uint32_t *)m_buffer, m_bufferSize);
    HAL_TIM_Base_Start(m_htim);
}

void AdcService::onStop()
{
    HAL_ADC_Stop_DMA(m_hadc);
    HAL_TIM_Base_Stop(m_htim);
}

void AdcService::handleDmaInterrupt(bool secondHalf)
{
    if (!isOperating()) {
        return;
    }

    if (m_numChannels == 0 || m_interleave == 0) {
        return;
    }

    uint32_t halfSize          = m_bufferSize / 2;
    uint32_t samplesPerChannel = halfSize / m_interleave;
    uint16_t *halfPtr          = secondHalf ? &m_buffer[halfSize] : m_buffer;

    for (uint8_t i = 0; i < m_numChannels; i++) {
        IAdcChannel *ch = m_channels[i];
        if (!ch->isActive()) continue;

        uint16_t offset = ch->getConversionOrder();
        InterleavedBuffer buf(reinterpret_cast<volatile uint16_t *>(halfPtr), offset, m_interleave);
        ch->process(buf, samplesPerChannel, m_bits, m_voltageRange);
    }
}

// =======================================================================
// HAL DMA callbacks
// =======================================================================
extern "C" {
    void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
    {
        if (AdcService *ctrl = AdcService::getController(hadc)) {
            ctrl->handleDmaInterrupt(false);
        }
    }

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
    {
        if (AdcService *ctrl = AdcService::getController(hadc)) {
            ctrl->handleDmaInterrupt(true);
        }
    }
}
