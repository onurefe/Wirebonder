#ifndef ADC_SERVICE_HPP
#define ADC_SERVICE_HPP

#include <cstdint>
#include <cmath>

#include "stm32f4xx_hal.h"
#include "configuration.h"
#include "process.hpp"
#include "generic.h"

// -----------------------------------------------------------------------
// Class: InterleavedBuffer
// -----------------------------------------------------------------------
class InterleavedBuffer {
public:
    InterleavedBuffer(volatile uint16_t *buffer, uint16_t offset, uint16_t interleave);

    __attribute__((always_inline)) inline uint16_t iterate(void) {
        uint16_t val = m_buffer[m_iterator];
        m_iterator += m_interleave;
        return val;
    }

    void setIterator(uint16_t idx);
    void resetIterator(void);

private:
    volatile uint16_t *m_buffer;
    uint16_t m_iterator;
    uint16_t m_offset;
    uint16_t m_interleave;
};

// -----------------------------------------------------------------------
// Interface: IAdcChannel
// -----------------------------------------------------------------------
class IAdcChannel {
public:
    virtual ~IAdcChannel() = default;
    virtual void process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange) = 0;
    virtual uint16_t getConversionOrder() const = 0;

    void enable()  { m_isActive = true; }
    void disable() { m_isActive = false; }
    bool isActive() const { return m_isActive; }

    static inline float adcToVolts(float sample, uint8_t bits, float voltageRange) {
        return voltageRange * (sample / (float)(1u << bits));
    }

private:
    bool m_isActive = true;
};

// -----------------------------------------------------------------------
// Class: RawAdcChannel
// -----------------------------------------------------------------------
class RawAdcChannel : public IAdcChannel {
public:
    using Callback = void (*)(void *context, uint16_t *buffer, uint32_t numSamples);

    RawAdcChannel(uint16_t conversionOrder, uint16_t *buffer, uint32_t bufferSize);

    bool addCaptureCompleteListenerCallback(void *context, Callback cb);
    void reset();
    void process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange) override;
    uint16_t getConversionOrder() const override;

private:
    struct CallbackRegistration {
        Callback callback;
        void *context;
    };

    static constexpr uint8_t kMaxCallbacks = 4U;

    uint16_t  m_conversionOrder;
    uint16_t *m_buffer;
    uint32_t  m_bufferSize;
    uint32_t  m_writeIndex;
    bool      m_done;
    CallbackRegistration m_callbacks[kMaxCallbacks];
    uint8_t   m_callbackCount;
};

// -----------------------------------------------------------------------
// Class: AnalogChannel
// -----------------------------------------------------------------------
class AnalogChannel : public IAdcChannel {
public:
    using AnalogCallback = void(*)(void *context, float voltage);

    AnalogChannel(uint16_t conversionOrder, uint32_t oversampling, float gain, float bias);

    bool addMeasurementListenerCallback(void *context, AnalogCallback cb);
    void process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange) override;
    uint16_t getConversionOrder() const override;
    
private:
    struct CallbackRegistration {
        AnalogCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxCallbacks = 4U;

    void report(uint8_t bits, float voltageRange);
    void resetAccumulators(void);

    uint16_t m_conversionOrder;
    uint32_t m_oversampling;
    float    m_gain;
    float    m_bias;
    float    m_sum;
    uint32_t m_sampleCounter;
    CallbackRegistration m_callbacks[kMaxCallbacks];
    uint8_t m_callbackCount;
};

// -----------------------------------------------------------------------
// Class: IQDemodulatorChannel
// -----------------------------------------------------------------------
class IQDemodulatorChannel : public IAdcChannel {
public:
    using MeasurementListenerCallback = void(*)(void *context, float re, float im);
    // Returns true when the controller is active; the demodulation frequency is
    // set through the pointer. The first active controller wins (see report()).
    using FrequencyControllerCallback = bool(*)(void *context, float *targetNormalizedFreq);

    IQDemodulatorChannel(uint16_t conversionOrder, uint32_t samplesPerMeasurement, float targetNormalizedFreq, float gain);

    bool addMeasurementListenerCallback(void *context, MeasurementListenerCallback cb);
    bool addFrequencyControllerCallback(void *context, FrequencyControllerCallback cb);
    void setDemodulationFrequency(float normalizedFrequency);
    void process(InterleavedBuffer &buffer, uint32_t numSamples, uint8_t bits, float voltageRange) override;
    uint16_t getConversionOrder() const override;

private:
    struct MeasurementListenerRegistration {
        MeasurementListenerCallback callback;
        void *context;
    };

    struct FrequencyControllerRegistration {
        FrequencyControllerCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxMeasurementListenerCallbacks = 4U;
    static constexpr uint8_t kMaxFrequencyControllerCallbacks = 4U;

    void updateDemodulator(void);
    void report(uint8_t bits, float voltageRange, float *targetNormalizedFrequency);
    void resetAccumulators(void);

    uint16_t m_conversionOrder;
    float    m_normalizedFrequency;
    float    m_gain;

    float    m_s1;
    float    m_s2;
    float    m_coeff;
    float    m_cosOmega;
    float    m_sinOmega;
    uint32_t m_sampleCounter;
    uint32_t m_samplesPerMeasurement;
    uint32_t m_dropLast;

    MeasurementListenerRegistration m_measurementListenerCallbacks[kMaxMeasurementListenerCallbacks];
    uint8_t m_measurementListenerCallbackCount;

    FrequencyControllerRegistration m_frequencyControllerCallbacks[kMaxFrequencyControllerCallbacks];
    uint8_t m_frequencyControllerCallbackCount;
};

// -----------------------------------------------------------------------
// Class: AdcService
// -----------------------------------------------------------------------
class AdcService : public Process {
public:
    AdcService(
        ADC_HandleTypeDef *hadc,
        TIM_HandleTypeDef *htim,
        uint8_t bits,
        float voltageRange,
        uint16_t *buffer = nullptr,
        uint32_t bufferSize = 0
    );

    bool addChannel(IAdcChannel *channel);

    void setBuffer(uint16_t *buffer, uint32_t bufferSize);

    void handleDmaInterrupt(bool secondHalf);

    static AdcService *getController(ADC_HandleTypeDef *hadc);

private:
    void onStart() override;
    void onStop() override;

    ADC_HandleTypeDef *m_hadc;
    TIM_HandleTypeDef *m_htim;
    uint16_t *m_buffer;
    uint32_t  m_bufferSize;

    IAdcChannel *m_channels[ADC_SERVICE_MAX_CHANNELS];
    uint8_t m_numChannels;
    uint8_t m_interleave;

    uint8_t m_bits;
    float   m_voltageRange;

};

#endif // ADC_SERVICE_HPP
