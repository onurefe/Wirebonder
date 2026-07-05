#ifndef TONE_DEBUG_CHANNEL_HPP
#define TONE_DEBUG_CHANNEL_HPP

#include <cstdint>

#include "configuration.h"
#include "complex.h"
#include "debug_service.hpp"
#include "pll_module.hpp"

struct ToneDebugResult {
    volatile float voltageMagnitude;
    volatile uint32_t voltageUpdates;

    volatile float currentMagnitude;
    volatile uint32_t currentUpdates;

    complexf voltagePhasor;
    complexf currentPhasor;
};

class DebugToneGenerator : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_TONE_GENERATOR;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    DebugToneGenerator();

    void init(SineGeneratorChannel *generator,
              IQDemodulatorChannel *voltageDemodulator,
              IQDemodulatorChannel *currentDemodulator);

    uint16_t channelId() const override
    {
        return ChannelId;
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

    static void onVoltageMeasured(void *context,
                                  float re,
                                  float im);

    static void onCurrentMeasured(void *context,
                                  float re,
                                  float im);

    static bool onDemodulationFrequencyRequested(
        void *context,
        float *targetNormalizedIQFrequency);

    void start();
    void stop();
    void restart();

    void resetResult();

    bool provideToneSample(float *amplitude,
                           float *average,
                           float *targetNormalizedGeneratorFrequency);

    void handleVoltageMeasurement(float re,
                                  float im);

    void handleCurrentMeasurement(float re,
                                  float im);

    bool provideDemodulationFrequency(float *targetNormalizedIQFrequency);

    SineGeneratorChannel *m_generator = nullptr;
    IQDemodulatorChannel *m_voltageDemodulator = nullptr;
    IQDemodulatorChannel *m_currentDemodulator = nullptr;

    bool m_active = false;

    float m_amplitude = 0.0f;
    float m_normalizedFrequency = 0.0f;

    complexf m_currentSkewRotator = {1.0f, 0.0f};

    ToneDebugResult m_result = {};
};

#endif /* TONE_DEBUG_CHANNEL_HPP */
