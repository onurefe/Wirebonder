#ifndef DEBUG_ENVIRONMENT_MOTOR_POSITION_HPP
#define DEBUG_ENVIRONMENT_MOTOR_POSITION_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_environment.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "pwm_service.hpp"
#include "lvdt_module.hpp"
#include "dc_motor_velocity_controller_module.hpp"
#include "dc_motor_position_controller_module.hpp"

struct DebugMotorPositionTelemetrySample {
    float position;
    float magA;
    float magB;
};

struct DebugMotorPositionStallTelemetrySample {
    float drive;
    float position;
    float magA;
    float magB;
};

// Sandbox for the Z-axis position loop: LVDT excitation/demodulation feeding
// DcMotorPositionControllerModule on top of the tachometer velocity loop.
class MotorPositionDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId =
        DEBUG_ENVIRONMENT_ID_MOTOR_POSITION_CONTROLLER;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2,
        CMD_STALL_SCAN = 3
    };

    enum ResultCode : uint32_t {
        RESULT_CAPTURE_COMPLETE = 0
    };

    MotorPositionDebugEnvironment();

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
    enum class Mode {
        STEP,
        STALL_SCAN
    };

    enum class StallScanPhase {
        SETTLING,
        RELAXING
    };

    static bool onProvidePositionSetpoint(void *context, float *positionSetpoint);

    void startStep();
    void startStallScan();
    void stopCapture();
    bool provideStepSetpoint(float *positionSetpoint);
    bool provideStallScanSetpoint(float *positionSetpoint);
    void finish(uint32_t sampleCount);

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
    static uint16_t m_dac2Buffer[2 * DAC2_SAMPLES];
    static uint16_t m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

    static AnalogChannel m_tachometerChannel;
    static IQDemodulatorChannel m_lvdtAChannel;
    static IQDemodulatorChannel m_lvdtBChannel;
    static SineGeneratorChannel m_lvdtExcitationChannel;
    static PwmRampChannel m_zMotorPwmChannel;

    static AdcService m_adc2Service;
    static DacService m_dacService;
    static PwmService m_tim1PwmService;

    static LvdtSensorModule m_lvdtSensorModule;
    static DcMotorVelocityControllerModule m_velocityController;
    static DcMotorPositionControllerModule m_positionController;

    DebugMotorPositionTelemetrySample *m_telemetry = nullptr;
    DebugMotorPositionStallTelemetrySample *m_stallTelemetry = nullptr;
    uint32_t m_sampleIdx = 0;
    uint32_t m_sampleLimit = 0;
    uint32_t m_settleIdx = 0;
    uint32_t m_settleLimit = 0;
    uint32_t m_relaxIdx = 0;
    uint32_t m_relaxLimit = 0;
    bool m_captureActive = false;

    Mode m_mode = Mode::STEP;
    StallScanPhase m_stallScanPhase = StallScanPhase::SETTLING;
    float m_stepValue = 0.0f;
    float m_currentDrive = 0.0f;
    float m_endDrive = 0.0f;
    float m_driveStep = 0.0f;
    float m_relaxDrive = 0.0f;
};

#endif /* DEBUG_ENVIRONMENT_MOTOR_POSITION_HPP */
