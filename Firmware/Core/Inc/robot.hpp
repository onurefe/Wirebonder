#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "configuration.h"
#include "generic.h"
#include "fast_io.hpp"
#include "pin_monitor_service.hpp"
#include "timer_expire_service.hpp"
#include "solenoid_service.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "pwm_service.hpp"
#include "lvdt_module.hpp"
#include "dc_motor_velocity_controller_module.hpp"
#include "dc_motor_position_controller_module.hpp"
#include "force_coil_module.hpp"
#include "stepper_service.hpp"
#include "stepper_router_service.hpp"
#include "homing_module.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"
#include "bonder_module.hpp"
#include "eeprom_emulator.hpp"
#include "io_expander_service.hpp"
#include "lcd_controller_module.hpp"
#include "control_panel_service.hpp"
#include "configuration_manager.hpp"
#include "user_interface_module.hpp"

class Robot {
public:
    Robot();

    void start();
    void execute();
    void stop();

private:
    static constexpr uint8_t kComponentCount = 21U;

    // Component indices order components so dependencies precede their
    // dependents: startups walk a list ascending, stop() walks all
    // components descending.
    enum class Component : uint8_t {
        TimerExpireService = 0U,
        IoExpanderService,
        PinMonitorService,
        SolenoidService,
        StepperService,
        RouterService,
        Tim1PwmService,
        DacService,
        Adc1Service,
        Adc2Service,
        ForceCoilModule,
        ZMotorVelocityModule,
        LvdtModule,
        ZMotorPositionModule,
        ControlPanelService,
        LcdControllerModule,
        UserInterfaceModule,
        HomingModule,
        PllModule,
        ImpedanceScannerModule,
        BonderModule
    };

    // Started at boot: everything the operator interface and Y homing need.
    static const Component kBootComponents[];
    static const uint8_t kBootComponentCount;

    // Started once the configuration is confirmed and Y is homed: the
    // bonding plant, ending with the bonder itself.
    static const Component kBonderComponents[];
    static const uint8_t kBonderComponentCount;

    void startComponents(const Component *components, uint8_t count);
    void startComponent(Component component);
    void stopComponent(Component component);

    // =========================================================================
    // Callbacks
    // =========================================================================

    static void onYAxisHomingEvent(void *context, HomingModule::Event event);
    static void onBonderModuleStateChanged(bool isIdle);
    static void onBonderModuleErrorOccurred(BonderModule::Error error);
    static void onUltrasonicReport(
        void *context, const BonderModule::UltrasonicReport& report);

    // UI events are triggers, not commands: Robot arbitrates them
    // against machine state (homing, bonder activity) before acting.
    static void onUserInterfaceEvent(void *ctx, UserInterfaceModule::Event event);
    static void onMouseButtonEvent(void *ctx,
                                   UserInterfaceModule::MouseButtonEvent event);
    static void onControlPanelButtonEvent(
        void *ctx,
        UserInterfaceModule::ControlPanelButtonEvent event);

    void onSetupButtonPressed();
    void onTestButtonPressed();
    void onResetButtonPressed();
    void onClampOpenButtonPressed();
    void onLightButtonPressed();

    static bool isYAxisHomed();
    static bool isBondingInterlocked();

    // Points the bonder at another protocol without disengaging it: accepted
    // while idle or still parked at the start gate, refused once a bond cycle
    // is engaged. A force setup superseded at its own gate is cleaned up.
    static bool retaskBonder(const BonderProtocol& protocol);
    static void updateClampCommandLed();
    void restoreConfiguredBondingProtocol();
    static void requestBonderStartIfReady();

    // Emergency-stop latch for unrecoverable faults: stops the bonder,
    // raises a latched error on the LCD (which also locks the keypad) and
    // refuses all operator commands except Reset until the MCU is reset.
    static void lockSystem(const char *message);

    // =========================================================================
    // Flags
    // =========================================================================
    static bool m_bonderConfigUpdated;   // true when config update is queued while bonder is busy
    static bool m_ultrasonicTestActive;
    static bool m_forceSetupActive;
    static bool m_manualClampOpen;
    static bool m_manualClampCommandActive;
    static bool m_systemLocked;          // latched by lockSystem(); cleared only by MCU reset
    bool m_areaLightOn{false};

    // =========================================================================
    // Buffers  —  raw DMA / processing memory
    // =========================================================================

    /* ADC capture buffers (double-buffered: 2× per channel). */
    static uint16_t m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];

    /* DAC output buffers (double-buffered). */
    static uint16_t m_dac1Buffer[2 * DAC1_SAMPLES];
    static uint16_t m_dac2Buffer[2 * DAC2_SAMPLES];

    /* Impedance scanner working buffers. */
    static uint16_t m_scannerSynthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
    static uint16_t m_scannerVsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
    static uint16_t m_scannerIsensBuffer[SCANNER_ADC_CAPTURE_SIZE];

    /* PWM segment buffers (double-buffered). */
    static uint16_t m_tim1pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
    static uint16_t m_tim1pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

    // =========================================================================
    // GPIO  —  bare pin wrappers
    // =========================================================================

    /* Stepper enable / reset (shared across both axes). */
    static FastIO m_stepperEnablePin;
    static FastIO m_stepperResetPin;

    /* Y-axis stepper. */
    static FastIO m_yAxisStepPin;
    static FastIO m_yAxisDirPin;

    /* T-axis stepper (tear / tail). */
    static FastIO m_tAxisStepPin;
    static FastIO m_tAxisDirPin;

    /* Solenoid drive lines. */
    static FastIO m_clampLowPin;
    static FastIO m_clampHighPin;
    static FastIO m_sol1LowPin;
    static FastIO m_sol1HighPin;
    static FastIO m_sol2LowPin;
    static FastIO m_sol2HighPin;

    /* Digital inputs / contact sensors. */
    static FastIO m_contactSensorPin;
    static FastIO m_mouseRightButtonPin;
    static FastIO m_mouseLeftButtonPin;
    static FastIO m_yAxisLimitSwitchPin;

    // =========================================================================
    // I/O expander bus  —  I2C expanders and their scheduler
    // =========================================================================

    static IoExpanderService        m_ioExpanderService;      /* I2C bus scheduler              */
    static Pca9538ExpanderChannel   m_lcdExpanderChannel;     /* 8-bit expander for LCD         */
    static Pca9535ExpanderChannel   m_keypadExpanderChannel;  /* 16-bit expander for all buttons/LEDs */

    // =========================================================================
    // Signal channels  —  ADC / DAC / PWM channel objects
    // =========================================================================

    /* ADC1 — ultrasonic (IQ-demodulated) and impedance scanner (raw). */
    static IQDemodulatorChannel m_ultrasonicVsensChannel;
    static IQDemodulatorChannel m_ultrasonicIsensChannel;
    static RawAdcChannel        m_scannerVsensChannel;
    static RawAdcChannel        m_scannerIsensChannel;

    /* ADC2 — Z-motor tachometer, LVDT (IQ-demodulated), force-coil current sense. */
    static AnalogChannel        m_tachometerChannel;
    static IQDemodulatorChannel m_lvdtAChannel;
    static IQDemodulatorChannel m_lvdtBChannel;
    static AnalogChannel        m_forceCoilISensChannel;

    /* DAC — ultrasonic drive, impedance scanner drive, LVDT excitation. */
    static SineGeneratorChannel m_ultrasonicDacChannel;
    static RawDacChannel        m_scannerDacChannel;
    static SineGeneratorChannel m_lvdtExcitationChannel;

    /* PWM — force coil and Z-motor H-bridge drive (ramp segments). */
    static PwmRampChannel m_forceCoilPwmChannel;
    static PwmRampChannel m_zMotorPwmChannel;

    // =========================================================================
    // Timers  —  service + all timer instances grouped by owner
    // =========================================================================

    static TimerExpireService m_timerExpireService;

    static Timer m_clampSolenoidTimer;
    static Timer m_sol1SolenoidTimer;
    static Timer m_sol2SolenoidTimer;
    static Timer m_bonderTimer;
    static Timer m_pinMonitorCriticalTimer;
    static Timer m_pinMonitorNormalTimer;
    static Timer m_controlPanelPollTimer;
    static Timer m_lcdDelayTimer;

    // =========================================================================
    // Peripheral services  —  ADC / DAC / PWM / stepper bus schedulers
    // =========================================================================

    static AdcService  m_adc1Service;
    static AdcService  m_adc2Service;
    static DacService  m_dacService;
    static PwmService  m_tim1PwmService;
    static StepperService m_stepperService;

    // =========================================================================
    // Input monitors  —  digital input polling and panel buttons
    // =========================================================================

    /* Pin monitors — contact and limit-switch debouncing. */
    static PinMonitorService  m_pinMonitorService;

    static PinMonitorChannel  m_contactSensorChannel;
    static PinMonitorChannel  m_mouseRightButtonChannel;
    static PinMonitorChannel  m_mouseLeftButtonChannel;
    static PinMonitorChannel  m_yAxisLimitSwitchChannel;

    /* Panel buttons — two DPST pins each (TODO: verify IDC assignments). */
    static ControlPanelService   m_controlPanelService;    /* keypad expander — all buttons */

    static ButtonChannel  m_btnUp;
    static ButtonChannel  m_btnDown;
    static ButtonChannel  m_btnLeft;
    static ButtonChannel  m_btnRight;
    static ButtonChannel  m_btnPlus;
    static ButtonChannel  m_btnMinus;
    static ButtonChannel  m_btnSave;
    static ButtonChannel  m_btnLoad;
    static ButtonChannel  m_btnTailPlus;
    static ButtonChannel  m_btnTailMinus;
    static ButtonChannel  m_btnLoopPlus;
    static ButtonChannel  m_btnLoopMinus;
    static ButtonChannel  m_btnSearchPlus;
    static ButtonChannel  m_btnSearchMinus;
    static ButtonChannel  m_btnStepPlus;
    static ButtonChannel  m_btnStepMinus;
    static ButtonChannel  m_btnReset;
    static ButtonChannel  m_btnEnter;
    static ButtonChannel  m_btnManual;
    static ButtonChannel  m_btnEscDel;
    static ButtonChannel  m_btnAdd;
    static ButtonChannel  m_btnTest;
    static ButtonChannel  m_btnSetup;
    static ButtonChannel  m_btnLight;
    static ButtonChannel  m_btnClampOpen;
    static ButtonChannel  m_btnHighReset;

    static LedChannel     m_ledTest;
    static LedChannel     m_ledSetup;
    static LedChannel     m_ledClampOpen;
    static LedChannel     m_ledManual;

    // =========================================================================
    // Output actuators
    // =========================================================================
    
    static DirectSolenoidChannel m_clampSolenoidChannel;
    static DirectSolenoidChannel m_sol1SolenoidChannel;
    static DirectSolenoidChannel m_sol2SolenoidChannel;
    static SolenoidService m_solenoidService;
    static DirectPwmChannel m_areaLightPwmChannel;

    // =========================================================================
    // Motion  —  stepper channels and router
    // =========================================================================

    static StepperChannel m_yAxisStepperChannel;
    static StepperChannel m_tAxisStepperChannel;
    static RouterChannel  m_yAxisRouterChannel;
    static RouterChannel  m_tAxisRouterChannel;
    static StepperRouterService  m_routerService;

    // =========================================================================
    // Subsystem modules  —  physical plant control
    // =========================================================================

    static LvdtSensorModule                 m_lvdtSensorModule;
    static ForceCoilDriverModule            m_forceCoilControllerModule;
    static DcMotorVelocityControllerModule  m_zMotorVelocityControllerModule;
    static DcMotorPositionControllerModule  m_zMotorPositionControllerModule;
    static HomingModule                     m_yAxisHomingModule;
    static PllModule                        m_pllModule;
    static UsImpedanceScannerModule         m_impedanceScannerModule;

    // =========================================================================
    // LCD
    // =========================================================================

    static LcdControllerModule m_lcdController;

    // =========================================================================
    // Bonder  —  top-level bonding state machine
    // =========================================================================

    static BonderModule m_bonder;

    // =========================================================================
    // Persistence and active bonding configuration
    // =========================================================================

    static EepromEmulator m_eepromEmulator;
    static ConfigurationManager m_configurationManager;
    static bool m_configurationConfirmed;

    // =========================================================================
    // User interface  —  LCD, configuration editor, and operator controls
    // =========================================================================

    static UserInterfaceModule m_userInterface;

    // Deferred bonder-chain start; requested from configuration/homing/bonder
    // callbacks and processed at the top of execute().
    static bool m_bonderStartPending;

    Process *m_components[kComponentCount]{};
};

#endif /* ROBOT_HPP */
