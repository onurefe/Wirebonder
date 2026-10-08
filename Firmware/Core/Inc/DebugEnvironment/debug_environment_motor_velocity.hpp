#ifndef DEBUG_ENVIRONMENT_MOTOR_VELOCITY_HPP
#define DEBUG_ENVIRONMENT_MOTOR_VELOCITY_HPP

#include <cstdint>

#include "configuration.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "pwm_service.hpp"
#include "lvdt_module.hpp"
#include "dc_motor_velocity_controller_module.hpp"

// One control tick: the LVDT position, the velocity the Kalman filter
// estimated from it, and the voltage then applied for the coming period.
// Position and voltage are what a model fit needs; the velocity shows what
// the loop acted on.
struct DebugMotorVelocityTelemetrySample {
    float position;
    float velocity;
    float voltage;
};

// Sandbox for the Z-motor velocity loop: LVDT excitation/demodulation feeding
// the Kalman velocity estimate that drives the H-bridge PWM through
// DcMotorVelocityControllerModule.
class MotorVelocityDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId =
        DEBUG_ENVIRONMENT_ID_MOTOR_VELOCITY_CONTROLLER;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_SETPOINT_ACHIEVED = 0
    };

    MotorVelocityDebugEnvironment();

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
    static void onVelocityMeasured(void *context, float measuredVelocity);
    static bool onVelocityControlUpdate(void *context, float *targetVelocity);

    void startCapture();
    void stopCapture();
    void recordVelocity(float measuredVelocity);

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
    static uint16_t m_dac2Buffer[2 * DAC2_SAMPLES];
    static uint16_t m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

    static IQDemodulatorChannel m_lvdtAChannel;
    static IQDemodulatorChannel m_lvdtBChannel;
    static SineGeneratorChannel m_lvdtExcitationChannel;
    static PwmRampChannel m_zMotorPwmChannel;

    static AdcService m_adc2Service;
    static DacService m_dacService;
    static PwmService m_tim1PwmService;

    static LvdtSensorModule m_lvdtSensorModule;
    static DcMotorVelocityControllerModule m_velocityController;

    DebugMotorVelocityTelemetrySample *m_telemetry = nullptr;
    uint32_t m_sampleIdx = 0;
    uint32_t m_sampleLimit = 0;
    bool m_captureActive = false;

    float m_stepValue = 0.0f;
};

#endif /* DEBUG_ENVIRONMENT_MOTOR_VELOCITY_HPP */
