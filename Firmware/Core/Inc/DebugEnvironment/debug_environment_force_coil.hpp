#ifndef DEBUG_ENVIRONMENT_FORCE_COIL_HPP
#define DEBUG_ENVIRONMENT_FORCE_COIL_HPP

#include <cstdint>

#include "configuration.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "adc_service.hpp"
#include "pwm_service.hpp"
#include "force_coil_module.hpp"

// Sandbox for the force coil current loop: I-sense (ADC2) driving the coil
// PWM through ForceCoilDriverModule.
class ForceCoilDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_FORCE_COIL;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_CAPTURE_COMPLETE = 0
    };

    ForceCoilDebugEnvironment();

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
        return m_captureActive;
    }

    void abort() override;

private:
    static void onCurrentMeasured(void *context, float measuredCurrent);

    void startCapture();
    void stopCapture();
    void recordCurrent(float measuredCurrent);

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
    static uint16_t m_pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];

    static AnalogChannel m_forceCoilISensChannel;
    static PwmRampChannel m_forceCoilPwmChannel;

    static AdcService m_adc2Service;
    static PwmService m_tim1PwmService;

    static ForceCoilDriverModule m_forceCoil;

    float *m_telemetry = nullptr;
    uint32_t m_sampleIdx = 0;
    uint32_t m_sampleLimit = 0;
    bool m_captureActive = false;
};

#endif /* DEBUG_ENVIRONMENT_FORCE_COIL_HPP */
