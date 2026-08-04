#ifndef DEBUG_ENVIRONMENT_BONDER_HPP
#define DEBUG_ENVIRONMENT_BONDER_HPP

#include <cstdint>

#include "configuration.h"
#include "complex.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "fast_io.hpp"
#include "timer_expire_service.hpp"
#include "pin_monitor_service.hpp"
#include "solenoid_service.hpp"
#include "stepper_service.hpp"
#include "stepper_router_service.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "pwm_service.hpp"
#include "lvdt_module.hpp"
#include "dc_motor_velocity_controller_module.hpp"
#include "dc_motor_position_controller_module.hpp"
#include "force_coil_module.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"
#include "bonder_module.hpp"

#define BONDER_DEBUG_LOG_DEPTH 64

// Ring log of completed (or failed) VM instructions. Recording is always on;
// the count only resets when a new bonding run starts. Host tooling reads it
// by symbol (BonderDebugEnvironment::s_stepLog).
struct BonderDebugLog {
    volatile uint32_t count;
    BonderModule::Telemetry entries[BONDER_DEBUG_LOG_DEPTH];
};

// Live VM snapshot refreshed every main-loop tick, so the debugger can halt at
// any moment and see exactly where the protocol is and what it is blocked on.
// Host tooling reads it by symbol (BonderDebugEnvironment::s_status).
struct BonderDebugStatus {
    volatile uint8_t  bonderRunning;
    volatile uint8_t  pc;
    volatile uint8_t  opcode;          // valid while bonderRunning
    volatile uint8_t  clampState;      // SolenoidChannel::State
    volatile uint32_t waitMask;        // mask of the current instruction
    volatile uint32_t eventFlags;      // latched VM event flags
    volatile uint32_t stepCount;       // mirrors s_stepLog.count
    volatile uint8_t  rightButtonPhysical;
    volatile uint8_t  rightButtonVirtual;
    volatile uint8_t  leftButtonPhysical;
    volatile uint8_t  leftButtonVirtual;
    volatile float    zPosition;
    volatile float    yPosition;
    volatile float    tPosition;
};

// Sandbox for the full bonding state machine: the complete physical plant
// (Z-axis chain, force coil, ultrasonics, steppers, clamp, contact sensors)
// without UI/LCD/keypad, homing, or the EEPROM configuration store.
//
// Debug workflow: the protocol VM free-runs on its own internal dynamics
// (moves, scans, timers, contact sensing) which are never injected from the
// debugger. Only operator inputs can be supplied virtually — CMD_SET_BUTTON
// feeds the same notifyRightButton()/notifyLeftButton() relay the physical
// mouse buttons use, and the two sources are OR-combined so either works.
// Every completed instruction lands in s_stepLog; s_status shows what the VM
// is currently blocked on.
class BonderDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_BONDER;

    enum Command : uint16_t {
        CMD_START_BONDER = 1,
        CMD_STOP_BONDER = 2,
        CMD_SET_BUTTON = 3
    };

    enum ButtonId : uint16_t {
        BUTTON_RIGHT = 0,
        BUTTON_LEFT = 1
    };

    BonderDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    void handleCommand(uint16_t localCommand) override;
    void onStart() override;
    void onPoll() override;
    void abort() override;

private:
    static void onTelemetry(void *context,
                            const BonderModule::Telemetry &telemetry);

    // Physical mouse buttons (the user-interface layer's job in the normal
    // firmware; here the environment owns the channels).
    static void onMouseRightButtonStateChanged(void *ctx,
                                               PinMonitorChannel::PinState state);
    static void onMouseLeftButtonStateChanged(void *ctx,
                                              PinMonitorChannel::PinState state);

    // Recomputes the physical|virtual OR and relays only real transitions,
    // so e.g. a virtual release cannot fake a button edge while the physical
    // button is still held.
    static void updateCombinedButtons();

    void startBonder();
    void stopBonder();
    void setButton();
    void resetLog();
    void publishStatus();
    void recordTelemetry(const BonderModule::Telemetry &telemetry);
    void copyImpedanceCurve(BonderModule::Telemetry &telemetry);

    // =========================================================================
    // Hardware sandbox — exclusively owned by this environment. The wiring
    // mirrors the Robot's bonding plant.
    // =========================================================================

    /* DMA / processing buffers. */
    static uint16_t m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
    static uint16_t m_dac1Buffer[2 * DAC1_SAMPLES];
    static uint16_t m_dac2Buffer[2 * DAC2_SAMPLES];
    static uint16_t m_synthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
    static uint16_t m_scannerVsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
    static uint16_t m_scannerIsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
    static uint16_t m_pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
    static uint16_t m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

    /* GPIO. */
    static FastIO m_stepperEnablePin;
    static FastIO m_stepperResetPin;
    static FastIO m_yAxisStepPin;
    static FastIO m_yAxisDirPin;
    static FastIO m_tAxisStepPin;
    static FastIO m_tAxisDirPin;
    // DRIVES_SOL3H (PB15, clamp high side) is TIM12_CH2 PWM, not a plain
    // GPIO -- see m_clampPwmChannel.
    static FastIO m_clampLowPin;
    static FastIO m_contactSensorPin;
    static FastIO m_mouseRightButtonPin;
    static FastIO m_mouseLeftButtonPin;

    /* Timers. */
    static TimerExpireService m_timerExpireService;
    static Timer m_clampSolenoidTimer;
    static Timer m_bonderTimer;
    static Timer m_pinMonitorCriticalTimer;
    static Timer m_pinMonitorNormalTimer;

    /* Digital inputs. */
    static PinMonitorService m_pinMonitorService;
    static PinMonitorChannel m_contactSensorChannel;
    static PinMonitorChannel m_mouseRightButtonChannel;
    static PinMonitorChannel m_mouseLeftButtonChannel;

    /* Clamp solenoid. */
    static DirectPwmChannel m_clampPwmChannel;
    static PwmSolenoidChannel m_clampSolenoidChannel;
    static SolenoidService m_solenoidService;

    /* Steppers / routers. */
    static StepperService m_stepperService;
    static StepperChannel m_yAxisStepperChannel;
    static StepperChannel m_tAxisStepperChannel;
    static RouterChannel m_yAxisRouterChannel;
    static RouterChannel m_tAxisRouterChannel;
    static StepperRouterService m_routerService;

    /* Signal channels. */
    static IQDemodulatorChannel m_ultrasonicVsensChannel;
    static IQDemodulatorChannel m_ultrasonicIsensChannel;
    static RawAdcChannel m_scannerVsensChannel;
    static RawAdcChannel m_scannerIsensChannel;
    static AnalogChannel m_tachometerChannel;
    static IQDemodulatorChannel m_lvdtAChannel;
    static IQDemodulatorChannel m_lvdtBChannel;
    static AnalogChannel m_forceCoilISensChannel;
    static SineGeneratorChannel m_ultrasonicDacChannel;
    static RawDacChannel m_scannerDacChannel;
    static SineGeneratorChannel m_lvdtExcitationChannel;
    static PwmRampChannel m_forceCoilPwmChannel;
    static PwmRampChannel m_zMotorPwmChannel;

    /* Peripheral services. */
    static AdcService m_adc1Service;
    static AdcService m_adc2Service;
    static DacService m_dacService;
    static PwmService m_tim1PwmService;

    /* Subsystem modules. */
    static LvdtSensorModule m_lvdtSensorModule;
    static ForceCoilDriverModule m_forceCoilControllerModule;
    static DcMotorVelocityControllerModule m_zMotorVelocityControllerModule;
    static DcMotorPositionControllerModule m_zMotorPositionControllerModule;
    static PllModule m_pllModule;
    static UsImpedanceScannerModule m_impedanceScannerModule;

    /* Bonding state machine. */
    static BonderModule m_bonder;

    // =========================================================================
    // GDB-visible observation state (addressed by symbol from the host).
    // =========================================================================

    static BonderDebugLog s_stepLog;
    static BonderDebugStatus s_status;

    // Operator-input state: physical pin level and virtual command level,
    // OR-combined before entering the bonder.
    static bool s_physicalRightButton;
    static bool s_physicalLeftButton;
    static bool s_virtualRightButton;
    static bool s_virtualLeftButton;
    static bool s_combinedRightButton;
    static bool s_combinedLeftButton;

    complexf *m_curveBuffer = nullptr;
    uint32_t m_curveCapacity = 0U;
    uint32_t m_curveCount = 0U;
};

#endif /* DEBUG_ENVIRONMENT_BONDER_HPP */
