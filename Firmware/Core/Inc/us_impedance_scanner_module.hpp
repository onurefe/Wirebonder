#ifndef US_IMPEDANCE_SCANNER_MODULE_HPP
#define US_IMPEDANCE_SCANNER_MODULE_HPP

#include <cstdint>
#include <cmath>

#include "generic.h"
#include "configuration.h"
#include "complex.h"
#include "adc_service.hpp"
#include "dac_service.hpp"

class UsImpedanceScannerModule {
public:
    using Callback = void (*)(void *context,
                              complexf *voltagePhasors,
                              complexf *currentPhasors,
                              complexf *impedances);

    UsImpedanceScannerModule(
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
        uint16_t  warmupIterations);

    void updateSynthesisBuffer();
    void setScanParameters(uint16_t numFrequencies, float minFrequency, float frequencyStep);

    bool addScanCompleteListenerCallback(void *context, Callback cb);
    void start(complexf *voltagePhasors, complexf *currentPhasors, complexf *impedances);
    void execute();
    void stop();

private:
    struct CallbackRegistration {
        Callback callback;
        void *context;
    };

    static constexpr uint8_t kMaxCallbacks = 4U;

    static void onVoltageCaptureDone(void *context, uint16_t *buffer, uint32_t numSamples);
    static void onCurrentCaptureDone(void *context, uint16_t *buffer, uint32_t numSamples);

    void analyzeChannel(const uint16_t *samples, uint32_t n, float gain, complexf *phasors);
    void computeImpedances();
    void publishScanComplete();

    RawDacChannel *m_dacChannel;
    RawAdcChannel *m_vChannel;
    RawAdcChannel *m_iChannel;

    uint16_t *m_synthesisBuffer;
    uint32_t  m_synthesisBufferSize;
    float     m_dacSamplingFrequency;
    uint8_t   m_dacBits;
    float     m_dacVoltageRange;

    uint16_t m_numFrequencies;
    float    m_minFrequency;
    float    m_frequencyStep;
    float    m_adcSamplingFrequency;
    uint8_t  m_adcBits;
    float    m_adcVoltageRange;
    float    m_vGain;
    float    m_iGain;

    ServiceState m_state;

    uint16_t         m_warmupIterations;
    volatile uint16_t m_vWarmupRemaining;
    volatile uint16_t m_iWarmupRemaining;

    volatile bool    m_vReady;
    volatile bool    m_iReady;
    const uint16_t  *m_vCapture;
    const uint16_t  *m_iCapture;
    uint32_t         m_captureLength;

    complexf *m_voltagePhasors;
    complexf *m_currentPhasors;
    complexf *m_impedances;

    CallbackRegistration m_callbacks[kMaxCallbacks];
    uint8_t m_callbackCount;
};

#endif /* US_IMPEDANCE_SCANNER_MODULE_HPP */
