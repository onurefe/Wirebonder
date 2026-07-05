#ifndef PLL_MODULE_HPP
#define PLL_MODULE_HPP

#include <cstdint>

#include "generic.h"
#include "complex.h"
#include "configuration.h"
#include "queue.hpp"
#include "pid_controller.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"

class PllModule {
public:
    enum class Event {
        BondingCompleted,
        InsufficientBondingPower
    };

    using Callback = void (*)(void *context, Event event);

    // One control-tick snapshot for bring-up/diagnostics; recording is off
    // unless a buffer is attached with setTelemetryBuffer().
    struct TelemetrySample {
        float phaseError;           // rad
        float frequencyCorrection;  // Hz, output of the frequency PID
        float realPower;
        float bondingEnergy;        // J, accumulated
    };

    PllModule(
        SineGeneratorChannel *sinusoid,
        IQDemodulatorChannel *voltageDemodulator,
        IQDemodulatorChannel *currentDemodulator,
        float samplingFrequency,
        float controlFrequency);

    bool addEventListenerCallback(void *context, Callback cb);

    // Recording starts at the next start() and stops when the buffer is
    // full; pass nullptr/0 to disable.
    void setTelemetryBuffer(TelemetrySample *buffer, uint16_t capacity);
    uint16_t getTelemetryCount() const;

    void start(float centerFrequency,
               float driveAmplitude,
               float bondingEnergyJoules,
               float maxBondingDurationSeconds);

    void stop();

    static void onVoltageMeasured(void *context, float re, float im);
    static void onCurrentMeasured(void *context, float re, float im);
    static bool onIqFrequencyRequested(void *context, float *targetNormalizedIQFrequency);
    static bool onSinusoidSample(void *context, float *amplitude, float *average, float *targetNormalizedGeneratorFrequency);

private:
    struct CallbackRegistration {
        Callback callback;
        void *context;
    };

    static constexpr uint8_t kMaxCallbacks = 4U;

    void updateController();
    void publishEvent(Event event);

    float measuredRealPower() const;
    float measuredPhaseError() const;

    SineGeneratorChannel *m_sinusoid;
    IQDemodulatorChannel *m_voltageDemodulator;
    IQDemodulatorChannel *m_currentDemodulator;
    PidController m_frequencyController;

    ServiceState m_state;

    CallbackRegistration m_callbacks[kMaxCallbacks];
    uint8_t m_callbackCount;

    complexf m_voltage;
    complexf m_current;
    complexf m_currentSkewRotator;   // undoes the V/I ADC sequencing skew

    bool m_voltageReady;
    bool m_currentReady;
    
    float m_samplingFrequency;
    float m_controlFrequency;

    float m_centerFrequency;
    float m_targetBondingEnergy;
    float m_maxBondingDuration;

    float m_bondingDuration;
    float m_bondingEnergy;

    float          m_freqQueueBuffer[PLL_MODULE_FREQ_CORRECTION_QUEUE_DEPTH + 1];
    Queue<float>   m_freqQueue;
    float          m_driveAmplitude;
    float          m_targetNormalizedIQFrequency;

    TelemetrySample *m_telemetryBuffer;
    uint16_t         m_telemetryCapacity;
    volatile uint16_t m_telemetryCount;
};

#endif /* PLL_MODULE_HPP */
