#ifndef DAC_SERVICE_HPP
#define DAC_SERVICE_HPP

#include <cstdint>

#include "stm32f4xx_hal.h"
#include "complex.h"
#include "configuration.h"
#include "process.hpp"
#include "generic.h"

// -----------------------------------------------------------------------
// Interface: IDacChannel
// -----------------------------------------------------------------------
class IDacChannel {
public:
    virtual ~IDacChannel() = default;
    virtual void bufferFillRequestHandler(bool secondHalf) = 0;
    virtual uint32_t getHalChannel(void) = 0;
    virtual DAC_HandleTypeDef *getDACHandle(void) = 0;
    virtual void stop() = 0;

    void enable()  { m_isActive = true; }
    void disable() { m_isActive = false; }
    bool isActive() const { return m_isActive; }

    static inline float voltsToDacCode(float volts, uint8_t bits, float voltageRange) {
        return volts * (float)((1u << bits) - 1u) / voltageRange;
    }

protected:
    bool m_isActive = false;
};

// -----------------------------------------------------------------------
// Class: RawDacChannel
// -----------------------------------------------------------------------
class RawDacChannel : public IDacChannel {
public:
    RawDacChannel(DAC_HandleTypeDef *hdac,
                  TIM_HandleTypeDef *triggerTimer,
                  uint32_t halChannel,
                  uint16_t *dmaBuffer,
                  uint32_t dmaBufferSize,
                  const uint16_t *synthesisBuffer,
                  uint32_t synthesisBufferSize);

    void start();
    void stop() override;

    void bufferFillRequestHandler(bool secondHalf) override;
    uint32_t getHalChannel(void) override;
    DAC_HandleTypeDef *getDACHandle(void) override;

private:
    DAC_HandleTypeDef *m_hdac;
    TIM_HandleTypeDef *m_triggerTimer;
    uint32_t           m_halChannel;
    uint16_t          *m_dmaBuffer;
    uint32_t           m_dmaBufferSize;
    const uint16_t    *m_synthesisBuffer;
    uint32_t           m_synthesisBufferSize;
    uint32_t           m_synthesisOffset;
};

// -----------------------------------------------------------------------
// Class: SineGeneratorChannel
// -----------------------------------------------------------------------
class SineGeneratorChannel : public IDacChannel {
public:
    // Returns true when the controller is active; amplitude/average/frequency
    // are written through the pointers. The first active controller wins.
    using SinusoidCallback = bool (*)(void *context,
                                      float *amp,
                                      float *avg,
                                      float *freq);

    SineGeneratorChannel(
        DAC_HandleTypeDef *hdac,
        TIM_HandleTypeDef *triggerTimer,
        uint32_t halChannel,
        uint16_t *buffer,
        uint32_t bufferSize,
        uint32_t updatePeriodInNumSamples,
        uint8_t dacBits,
        float dacVoltageRange);

    void start(float amplitude, float average, float normalizedFrequency);
    void stop() override;

    bool addWaveformControllerCallback(void *context, SinusoidCallback cb);
    void setFrequency(float normalizedFrequency);

    void bufferFillRequestHandler(bool secondHalf) override;
    uint32_t getHalChannel(void) override;
    DAC_HandleTypeDef *getDACHandle(void) override;

private:
    void calculateRotator();
    void updateParameters();
    void fillBuffer(uint16_t *buffer, uint32_t count);

    DAC_HandleTypeDef *m_hdac;
    TIM_HandleTypeDef *m_triggerTimer;
    uint32_t m_halChannel;

    uint16_t *m_buffer;
    uint32_t  m_bufferSize;

    complexf m_phasor;
    complexf m_rotatorStep;
    uint32_t m_sampleCounter;

    float    m_amplitude;
    float    m_average;
    float    m_normalizedFrequency;
    uint32_t m_updatePeriodInNumSamples;

    float    m_dacVoltageRange;
    uint8_t  m_dacBits;

    struct WaveformControllerRegistration {
        SinusoidCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxWaveformControllerCallbacks = 4U;

    WaveformControllerRegistration m_waveformControllerCallbacks[kMaxWaveformControllerCallbacks];
    uint8_t m_waveformControllerCallbackCount;
};

// -----------------------------------------------------------------------
// Class: DacService
// -----------------------------------------------------------------------
class DacService : public Process {
public:
    explicit DacService(DAC_HandleTypeDef *hdac);
    bool addChannel(IDacChannel *channel);

    static IDacChannel *findChannel(DAC_HandleTypeDef *hdac, uint32_t halChannel);

private:
    void onStart() override;
    void onStop() override;

    IDacChannel *m_channels[DAC_SERVICE_MAX_CHANNELS];
    uint8_t      m_numChannels;
};

#endif // DAC_SERVICE_HPP
