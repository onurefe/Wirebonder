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
#include "dc_router_module.hpp"
#include "force_coil_module.hpp"
#include "stepper_service.hpp"
#include "stepper_router_service.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"
#include "bonder_module.hpp"
#include "eeprom_emulator.hpp"
#include "io_expander_service.hpp"
#include "lcd_module.hpp"
#include "control_panel_service.hpp"
#include "ui_module.hpp"

#if DEBUG_ENABLED
#include "debug_service.hpp"
#include "debug_impedance_scanner.hpp"
#include "debug_keypad.hpp"
#include "debug_pll.hpp"
#include "debug_tone_generator.hpp"
#include "debug_motor_velocity_controller.hpp"
#include "debug_force_coil.hpp"
#endif

class Robot {
public:
    Robot();

    void start();
    void execute();
    void stop();

private:
    // =========================================================================
    // Callbacks
    // =========================================================================

    static void onYAxisLimitSwitchTransition(void *context, PinMonitorChannel::Transition transition);
    static void onBonderModuleStateChanged(bool isIdle);
    static void onBonderModuleErrorOccurred(BonderModule::Error error);

    static void onConfigChanged(void *ctx, const BonderConfig& config);
    static void onSave(void *ctx, const BonderConfig& config);
    static void onLoad(void *ctx, BonderConfig& config);
    static void onBonderConfigCompactification(void *ctx);

    // =========================================================================
    // Flags
    // =========================================================================
    static bool m_bonderConfigUpdated;   // true when config update is queued while bonder is busy

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
    static uint32_t m_tim1pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
    static uint32_t m_tim1pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

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
    static FastIO m_sol2LowPin;
    static FastIO m_sol2HighPin;
    static FastIO m_sol3LowPin;
    static FastIO m_sol3HighPin;

    /* Digital inputs / contact sensors. */
    static FastIO m_contactSensorPin;
    static FastIO m_mouseRightButtonPin;
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
    static Timer m_sol2SolenoidTimer;
    static Timer m_sol3SolenoidTimer;
    static Timer m_bonderTimer;
    static Timer m_zMotorSettlingTimer;
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

    // =========================================================================
    // Output actuators
    // =========================================================================
    
    static SolenoidChannel m_clampSolenoidChannel;
    static SolenoidChannel m_sol2SolenoidChannel;
    static SolenoidChannel m_sol3SolenoidChannel;
    static SolenoidService m_solenoidService;

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
    static DcRouterModule                   m_zMotorRouterModule;
    static PllModule                        m_pllModule;
    static UsImpedanceScannerModule         m_impedanceScannerModule;

    // =========================================================================
    // LCD
    // =========================================================================

    static LcdModule m_lcd;

    // =========================================================================
    // Bonder  —  top-level bonding state machine
    // =========================================================================

    static BonderModule m_bonder;

    // =========================================================================
    // Persistence  —  EEPROM emulator and live bonding configuration
    // =========================================================================

    static EepromEmulator m_eepromEmulator;
    static BonderConfig   m_bonderConfig;   /* live editable copy; registered with EEPROM emulator */

    // =========================================================================
    // UI  —  LCD parameter editor
    // =========================================================================

    static UiModule m_ui;

    // =========================================================================
    // Debug modules
    // =========================================================================
#if DEBUG_ENABLED
    static bool startImpedanceScannerDebugDependencies(void *context, uint16_t localCommand);
    static bool startPllDebugDependencies(void *context, uint16_t localCommand);
    static bool startToneGeneratorDebugDependencies(void *context, uint16_t localCommand);
    static bool startKeypadDebugDependencies(void *context, uint16_t localCommand);
    static bool startMotorVelocityDebugDependencies(void *context, uint16_t localCommand);
    static bool startForceCoilDebugDependencies(void *context, uint16_t localCommand);
    static void stopImpedanceScannerDebugDependencies(void *context, uint16_t localCommand);
    static void stopPllDebugDependencies(void *context, uint16_t localCommand);
    static void stopToneGeneratorDebugDependencies(void *context, uint16_t localCommand);
    static void stopKeypadDebugDependencies(void *context, uint16_t localCommand);
    static void stopMotorVelocityDebugDependencies(void *context, uint16_t localCommand);
    static void stopForceCoilDebugDependencies(void *context, uint16_t localCommand);

    static DebugService                 m_debugService;
    static DebugImpedanceScanner        m_debugChannelImpedanceScanner;
    static DebugKeypad                  m_debugChannelKeypad;
    static DebugToneGenerator           m_debugChannelToneGenerator;
    static DebugPll                     m_debugChannelPll;
    static DebugMotorVelocityController m_debugChannelMotorVelocityController;
    static DebugForceCoil               m_debugChannelForceCoil;
#endif
    bool m_startDebugService;
    bool m_startIoExpanderService;
    bool m_startTimerExpireService;
    bool m_startPinMonitorService;
    bool m_startSolenoidService;
    bool m_startStepperService;
    bool m_startRouterService;
    bool m_startTim1PwmService;
    bool m_startDacService;
    bool m_startAdc1Service;
    bool m_startAdc2Service;
    bool m_startForceCoilControllerModule;
    bool m_startZmotorVelocityControllerModule;
    bool m_startZmotorPositionControllerModule;
    bool m_startZMotorRouterModule;
    bool m_startBonderModule;
    bool m_startControlPanelService;
    bool m_startLcdModule;
    bool m_startUiModule;

    bool m_stopDebugService;
    bool m_stopIoExpanderService;
    bool m_stopTimerExpireService;
    bool m_stopPinMonitorService;
    bool m_stopSolenoidService;
    bool m_stopStepperService;
    bool m_stopRouterService;
    bool m_stopTim1PwmService;
    bool m_stopDacService;
    bool m_stopAdc1Service;
    bool m_stopAdc2Service;
    bool m_stopForceCoilControllerModule;
    bool m_stopZmotorVelocityControllerModule;
    bool m_stopZmotorPositionControllerModule;
    bool m_stopZMotorRouterModule;
    bool m_stopBonderModule;
    bool m_stopControlPanelService;
    bool m_stopLcdModule;
    bool m_stopUiModule;
};

#endif /* ROBOT_HPP */
