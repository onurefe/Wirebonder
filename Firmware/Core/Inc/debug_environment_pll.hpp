#ifndef DEBUG_ENVIRONMENT_PLL_HPP
#define DEBUG_ENVIRONMENT_PLL_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_environment.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "pll_module.hpp"

// Sandbox for PLL bring-up: owns the ultrasonic drive/sense chain (DAC sine
// output, ADC1 IQ demodulation) and the PllModule, with nothing else running.
class PllDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_PLL;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_BONDING_COMPLETED = 0,
        RESULT_DURATION_TIMEOUT = 1
    };

    PllDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;

    bool isBusy() const override
    {
        return m_transferActive;
    }

    void abort() override;

private:
    static void onPllEvent(void *context, PllModule::Event event);

    void startTransfer();
    void stopTransfer();
    void handlePllEvent(PllModule::Event event);

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t m_dac1Buffer[2 * DAC1_SAMPLES];

    static IQDemodulatorChannel m_ultrasonicVsensChannel;
    static IQDemodulatorChannel m_ultrasonicIsensChannel;
    static SineGeneratorChannel m_ultrasonicDacChannel;

    static AdcService m_adc1Service;
    static DacService m_dacService;

    static PllModule m_pllModule;

    PllModule::TelemetrySample *m_telemetry = nullptr;

    bool m_transferActive = false;
};

#endif /* DEBUG_ENVIRONMENT_PLL_HPP */
