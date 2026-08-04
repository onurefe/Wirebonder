#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "configuration.h"
#include "generic.h"
#include "queue.hpp"
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
#include "machine_settings.hpp"
#include "user_interface_module.hpp"
#include "cstring"

class RobotRequest {
    public:
        enum class RequestState:uint8_t {
            Pending,
            Active,
            Completed,
            Locked
        };

        enum class RequestCode :uint8_t {
            Home,
            Initialize,
            TestUs,
            TestForce,
            CalibrateTachometer,
            ExecuteBondingProtocol,
        };

        RobotRequest();
        explicit RobotRequest(RequestCode requestCode, const char *requestMessage, bool critical);

        /* m_infoQueue is bound to this object's own m_infoQueueContainer, so a
           bitwise copy would leave the copy's queue pointing into the source's
           storage. Both copy operations therefore rebind — the destination
           keeps its own (empty) info queue. Copies are only ever made of the
           pristine request prototypes, which carry no pending info lines; the
           one object that accumulates them (Robot::m_activeRequest) is never
           copied from. */
        RobotRequest(const RobotRequest &other);
        RobotRequest &operator=(const RobotRequest &other);

        // Number of info lines which can be buffered before requestCompleted().
        static const uint16_t INFO_QUEUE_CAPACITY = 4;
        static const uint16_t MSG_RENDER_BUFFER_CAPACITY = 100;

        void setRequestMessage(const char *msg);
        bool appendInfoMessage(const char *msg);
        
        char *requestStarted(bool success);
        char *requestCompleted(bool success);
        
        char *flushQueueToMessage(void);

        RequestCode getRequestCode() const {return m_requestCode;}
        RequestState getRequestState() const {return m_requestState;}
        const char *getRequestName() const {return m_requestName;}
        bool getSuccess() const {return m_success;}
        bool getCritical() const {return m_critical;}

        void setRequestCode(RequestCode code) {m_requestCode = code;}
        void setRequestState(RequestState state) {m_requestState = state;}
    private:
        RequestCode m_requestCode;
        RequestState m_requestState;

        static char m_renderBuffer[MSG_RENDER_BUFFER_CAPACITY];
        const char *m_requestName;
        const char *m_infoQueueContainer[INFO_QUEUE_CAPACITY + 1];
        Queue<const char *> m_infoQueue =
            Queue<const char *>(m_infoQueueContainer, INFO_QUEUE_CAPACITY);
        
        bool m_success;
        bool m_critical;
};

class Robot {
public:
    Robot();

    void start();
    void execute();

private:
    enum class RobotState : uint8_t {
        Idle,
        Busy,
        Error
    };

    enum RobotEvents : uint32_t {
        OperationCompleted              = (1 << 1),
        OperationFailed                 = (1 << 2)
    };

    static const uint16_t REQUEST_QUEUE_DEPTH = 8;

    // =========================================================================
    // Helpers
    // =========================================================================
    void startExecutingRequest(RobotRequest &request);
    void completeExecutingRequest(RobotRequest &request, bool success);
    bool startHoming();
    bool startBonderInitializing();
    bool startUsTest();
    bool startForceTest();
    bool startTachometerCalibration();
    bool startBondingProtocol();

    bool requestStartProtocol(const BonderProtocol &protocol);
    /* Hands the active configuration to the bonder with the machine-wide
       values (setup tracking force, force-coil offset) stamped over whatever
       the persisted profile carried. Every configure() goes through here so
       a stale profile copy can never reach the VM. */
    void configureBonderModule();
    void updateAreaLightDrive();
    void updateSpotlightDrive();
    void updateClampDrive();
    static void updateClampIndicator();
    /* Lights the Test / Setup keypad LED for as long as its request is the
       one actually running. */
    static void updateRequestIndicators();
    void updateTachometerOffsetCorrection();
    void updateZPositionSpeedLimits();

    static void turnoffPeripherals(void);

    // =========================================================================
    // Callbacks
    // =========================================================================

    static void onYAxisHomingEvent(void *context, HomingModule::Event event);
    static void onBonderModuleStateChanged(void *context, bool isIdle);
    static void onBonderModuleErrorOccurred(void *context, BonderModule::Error error);
    static void onUltrasonicReport(
        void *context, const BonderModule::UltrasonicReport& report);
    static void onTachCalReport(void *context, float offsetResidual);
    static void onUserInterfaceEvent(void *ctx, UserInterfaceModule::Event event);
    static void onMouseButtonEvent(void *ctx,
                                   UserInterfaceModule::MouseButtonEvent event);
    static void onControlPanelButtonEvent(
        void *ctx,
        UserInterfaceModule::ControlPanelButtonEvent event);

    // =========================================================================
    // States
    // =========================================================================
    static RobotState                       m_robotState;
    static RobotRequest                     m_activeRequest;

    static bool                             m_isAreaLightEnergized;
    static bool                             m_isClampEnergized;
    
    static uint32_t                         m_events;
    
    // =========================================================================
    // Request Queue & Request.
    // =========================================================================
    static RobotRequest                     m_requestQueueContainer[REQUEST_QUEUE_DEPTH+1];
    Queue<RobotRequest>                     m_requestQueue = Queue<RobotRequest>(m_requestQueueContainer, REQUEST_QUEUE_DEPTH);

    /* Pristine prototypes. Enqueuing copies one of these into the queue; the
       copy that reaches the head is moved into m_activeRequest, which is the
       only instance that accumulates info lines. */
    static const RobotRequest               m_homingRequest;
    static const RobotRequest               m_initializeRequest;
    static const RobotRequest               m_testUsRequest;
    static const RobotRequest               m_testForceRequest;
    static const RobotRequest               m_calibrateTachometerRequest;
    static const RobotRequest               m_executeBondingRequest;

    // =========================================================================
    // Buffers  —  raw DMA / processing memory
    // =========================================================================

    /* ADC capture buffers (double-buffered: 2× per channel). */
    static uint16_t                         m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t                         m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];

    /* DAC output buffers (double-buffered). */
    static uint16_t                         m_dac1Buffer[2 * DAC1_SAMPLES];
    static uint16_t                         m_dac2Buffer[2 * DAC2_SAMPLES];

    /* Impedance scanner working buffers. */
    static uint16_t                         m_scannerSynthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
    static uint16_t                         m_scannerVsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
    static uint16_t                         m_scannerIsensBuffer[SCANNER_ADC_CAPTURE_SIZE];

    /* PWM segment buffers (double-buffered). */
    static uint16_t                         m_tim1pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
    static uint16_t                         m_tim1pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

    // =========================================================================
    // GPIO  —  bare pin wrappers
    // =========================================================================

    /* Stepper enable / reset (shared across both axes). */
    static FastIO                           m_stepperEnablePin;
    static FastIO                           m_stepperResetPin;

    /* Y-axis stepper. */
    static FastIO                           m_yAxisStepPin;
    static FastIO                           m_yAxisDirPin;

    /* T-axis stepper (tear / tail). */
    static FastIO                           m_tAxisStepPin;
    static FastIO                           m_tAxisDirPin;

    /* Solenoid drive lines. Clamp high side (DRIVES_SOL3H/PB15) is now
       TIM12_CH2 PWM (see m_clampPwmChannel); the low side stays a plain
       GPIO, held clear once at startup. */
    static FastIO                           m_clampLowPin;
    static FastIO                           m_sol1LowPin;
    static FastIO                           m_sol1HighPin;
    static FastIO                           m_sol2LowPin;
    static FastIO                           m_sol2HighPin;

    /* Digital inputs / contact sensors. */
    static FastIO                           m_contactSensorPin;
    static FastIO                           m_mouseRightButtonPin;
    static FastIO                           m_mouseLeftButtonPin;
    static FastIO                           m_yAxisLimitSwitchPin;

    // =========================================================================
    // I/O expander bus  —  I2C expanders and their scheduler
    // =========================================================================

    static IoExpanderService                m_ioExpanderService;      /* I2C bus scheduler              */
    static Pca9538ExpanderChannel           m_lcdExpanderChannel;     /* 8-bit expander for LCD         */
    static Pca9535ExpanderChannel           m_keypadExpanderChannel;  /* 16-bit expander for all buttons/LEDs */

    // =========================================================================
    // Signal channels  —  ADC / DAC / PWM channel objects
    // =========================================================================

    /* ADC1 — ultrasonic (IQ-demodulated) and impedance scanner (raw). */
    static IQDemodulatorChannel             m_ultrasonicVsensChannel;
    static IQDemodulatorChannel             m_ultrasonicIsensChannel;
    static RawAdcChannel                    m_scannerVsensChannel;
    static RawAdcChannel                    m_scannerIsensChannel;
    /* Data-free channel giving PllModule a way to phase-align the DAC's
       unsynchronized trigger timer to ADC1's tick grid. */
    static AdcTickSyncChannel               m_ultrasonicTickSyncChannel;

    /* ADC2 — Z-motor tachometer, LVDT (IQ-demodulated), force-coil current sense. */
    static AnalogChannel                    m_tachometerChannel;
    static IQDemodulatorChannel             m_lvdtAChannel;
    static IQDemodulatorChannel             m_lvdtBChannel;
    static AnalogChannel                    m_forceCoilISensChannel;

    /* DAC — ultrasonic drive, impedance scanner drive, LVDT excitation. */
    static SineGeneratorChannel             m_ultrasonicDacChannel;
    static RawDacChannel                    m_scannerDacChannel;
    static SineGeneratorChannel             m_lvdtExcitationChannel;

    /* PWM — force coil and Z-motor H-bridge drive (ramp segments). */
    static PwmRampChannel                   m_forceCoilPwmChannel;
    static PwmRampChannel                   m_zMotorPwmChannel;

    // =========================================================================
    // Timers  —  service + all timer instances grouped by owner
    // =========================================================================

    static TimerExpireService               m_timerExpireService;

    static Timer                            m_clampSolenoidTimer;
    static Timer                            m_sol1SolenoidTimer;
    static Timer                            m_sol2SolenoidTimer;
    static Timer                            m_bonderTimer;
    static Timer                            m_pinMonitorCriticalTimer;
    static Timer                            m_pinMonitorNormalTimer;
    static Timer                            m_controlPanelPollTimer;
    static Timer                            m_lcdDelayTimer;

    // =========================================================================
    // Peripheral services  —  ADC / DAC / PWM / stepper bus schedulers
    // =========================================================================

    static AdcService                       m_adc1Service;
    static AdcService                       m_adc2Service;
    static DacService                       m_dacService;
    static PwmService                       m_tim1PwmService;
    static StepperService                   m_stepperService;

    // =========================================================================
    // Input monitors  —  digital input polling and panel buttons
    // =========================================================================

    /* Pin monitors — contact and limit-switch debouncing. */
    static PinMonitorService                m_pinMonitorService;

    static PinMonitorChannel                m_contactSensorChannel;
    static PinMonitorChannel                m_mouseRightButtonChannel;
    static PinMonitorChannel                m_mouseLeftButtonChannel;
    static PinMonitorChannel                m_yAxisLimitSwitchChannel;

    static ControlPanelService              m_controlPanelService;    /* keypad expander — all buttons */

    static ButtonChannel                    m_btnUp;
    static ButtonChannel                    m_btnDown;
    static ButtonChannel                    m_btnLeft;
    static ButtonChannel                    m_btnRight;
    static ButtonChannel                    m_btnPlus;
    static ButtonChannel                    m_btnMinus;
    static ButtonChannel                    m_btnSave;
    static ButtonChannel                    m_btnLoad;
    static ButtonChannel                    m_btnTailPlus;
    static ButtonChannel                    m_btnTailMinus;
    static ButtonChannel                    m_btnLoopPlus;
    static ButtonChannel                    m_btnLoopMinus;
    static ButtonChannel                    m_btnSearchPlus;
    static ButtonChannel                    m_btnSearchMinus;
    static ButtonChannel                    m_btnStepPlus;
    static ButtonChannel                    m_btnStepMinus;
    static ButtonChannel                    m_btnReset;
    static ButtonChannel                    m_btnEnter;
    static ButtonChannel                    m_btnManual;
    static ButtonChannel                    m_btnEscDel;
    static ButtonChannel                    m_btnAdd;
    static ButtonChannel                    m_btnTest;
    static ButtonChannel                    m_btnSetup;
    static ButtonChannel                    m_btnLight;
    static ButtonChannel                    m_btnClampOpen;
    /* Polled so the keypad scan stays complete; intentionally unbound. */
    static ButtonChannel                    m_btnHighReset;

    static LedChannel                       m_ledTest;
    static LedChannel                       m_ledSetup;
    static LedChannel                       m_ledClampOpen;
    static LedChannel                       m_ledManual;

    // =========================================================================
    // Output actuators
    // =========================================================================

    static DirectPwmChannel                 m_clampPwmChannel;
    static PwmSolenoidChannel               m_clampSolenoidChannel;
    static DirectSolenoidChannel            m_sol1SolenoidChannel;
    static DirectSolenoidChannel            m_sol2SolenoidChannel;
    static SolenoidService                  m_solenoidService;
    static DirectPwmChannel                 m_areaLightPwmChannel;
    static DirectPwmChannel                 m_spotlightPwmChannel;

    // =========================================================================
    // Motion  —  stepper channels and router
    // =========================================================================

    static StepperChannel                   m_yAxisStepperChannel;
    static StepperChannel                   m_tAxisStepperChannel;
    static RouterChannel                    m_yAxisRouterChannel;
    static RouterChannel                    m_tAxisRouterChannel;
    static StepperRouterService             m_routerService;

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

    static LcdControllerModule              m_lcdControllerModule;

    // =========================================================================
    // Bonder  —  top-level bonding state machine
    // =========================================================================

    static BonderModule                     m_bonderModule;

    // =========================================================================
    // Persistence and active bonding configuration
    // =========================================================================

    static EepromEmulator                   m_eepromEmulator;
    static ConfigurationManager             m_configurationManager;
    static BonderConfig                     m_activeBonderConfiguration;
    static MachineSettingsStore             m_machineSettingsStore;

    // =========================================================================
    // User interface  —  LCD, configuration editor, and operator controls
    // =========================================================================

    static UserInterfaceModule              m_userInterfaceModule;
};

#endif /* ROBOT_HPP */
