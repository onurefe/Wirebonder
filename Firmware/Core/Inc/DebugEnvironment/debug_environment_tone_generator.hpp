#ifndef DEBUG_ENVIRONMENT_TONE_GENERATOR_HPP
#define DEBUG_ENVIRONMENT_TONE_GENERATOR_HPP

#include <cstdint>

#include "configuration.h"
#include "complex.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"

struct ToneDebugResult {
    volatile float voltageMagnitude;
    volatile uint32_t voltageUpdates;

    volatile float currentMagnitude;
    volatile uint32_t currentUpdates;

    complexf voltagePhasor;
    complexf currentPhasor;
};

// Sandbox for open-loop ultrasonic drive: a fixed-frequency DAC tone with
// IQ-demodulated V/I sense readback, no PLL in the loop.
class ToneGeneratorDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_TONE_GENERATOR;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    ToneGeneratorDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_START ||
               localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;

    bool isBusy() const override
    {
        return m_active;
    }

    void abort() override;

private:
    static bool onToneSample(void *context,
                             float *amplitude,
                             float *average,
                             float *targetNormalizedGeneratorFrequency);

    static void onVoltageMeasured(void *context, float re, float im);
    static void onCurrentMeasured(void *context, float re, float im);

    static bool onDemodulationFrequencyRequested(
        void *context,
        float *targetNormalizedIQFrequency);

    void startTone();
    void stopTone();
    void restartTone();

    void resetResult();

    bool provideToneSample(float *amplitude,
                           float *average,
                           float *targetNormalizedGeneratorFrequency);

    void handleVoltageMeasurement(float re, float im);
    void handleCurrentMeasurement(float re, float im);

    bool provideDemodulationFrequency(float *targetNormalizedIQFrequency);

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t m_dac1Buffer[2 * DAC1_SAMPLES];

    static IQDemodulatorChannel m_ultrasonicVsensChannel;
    static IQDemodulatorChannel m_ultrasonicIsensChannel;
    static SineGeneratorChannel m_ultrasonicDacChannel;

    static AdcService m_adc1Service;
    static DacService m_dacService;

    bool m_active = false;

    float m_amplitude = 0.0f;
    float m_normalizedFrequency = 0.0f;

    complexf m_currentSkewRotator = {1.0f, 0.0f};

    ToneDebugResult m_result = {};
};

#endif /* DEBUG_ENVIRONMENT_TONE_GENERATOR_HPP */
