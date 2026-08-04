#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_NORMAL

#include "robot.hpp"
#include "main.h"
#include "Protocol/protocol_semi_auto.hpp"
#include "Protocol/protocol_manual.hpp"
#include "Protocol/protocol_table_tear.hpp"
#include "Protocol/protocol_lange_coupling.hpp"
#include "Protocol/protocol_ultrasonic_test.hpp"
#include "Protocol/protocol_force_setup.hpp"
#include "Protocol/protocol_tach_cal.hpp"
#include "Protocol/protocol_reset_prologue.hpp"
#include "stdio.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern DAC_HandleTypeDef hdac;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

extern I2C_HandleTypeDef hi2c1;

// =============================================================================
// Static member definitions
//
// Ordered lowest-complexity first, highest last — mirrors robot.hpp.
// All HAL handles are passed by address only; actual HAL activity begins in
// start() after MX_xxx_Init() has completed and Robot_Init() constructs Robot.
// =============================================================================

// Protocol accessors are defined further down, next to the callbacks that use
// them; the startXxx() helpers reach for them before that point.
static const BonderProtocol& resetProtocol();
static const BonderProtocol& ultrasonicTestProtocol();
static const BonderProtocol& forceSetupProtocol();
static const BonderProtocol& tachCalProtocol();
static const BonderProtocol& protocolForMode(BondingMode mode);

// -----------------------------------------------------------------------------
// States
// -----------------------------------------------------------------------------
Robot::RobotState Robot::m_robotState = RobotState::Idle;
RobotRequest Robot::m_activeRequest;

/* The area light comes up lit; the LIGHT button toggles it off. */
bool Robot::m_isAreaLightEnergized = true;
bool Robot::m_isClampEnergized     = false;

uint32_t Robot::m_events = 0U;

// -----------------------------------------------------------------------------
// Buffers  —  raw DMA / processing memory
// -----------------------------------------------------------------------------
uint16_t Robot::m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
uint16_t Robot::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];

uint16_t Robot::m_dac1Buffer[2 * DAC1_SAMPLES];
uint16_t Robot::m_dac2Buffer[2 * DAC2_SAMPLES];

uint16_t Robot::m_scannerSynthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
uint16_t Robot::m_scannerVsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
uint16_t Robot::m_scannerIsensBuffer[SCANNER_ADC_CAPTURE_SIZE];

uint16_t Robot::m_tim1pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
uint16_t Robot::m_tim1pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

// -----------------------------------------------------------------------------
// GPIO  —  bare pin wrappers
// -----------------------------------------------------------------------------
FastIO Robot::m_stepperEnablePin(STEPPER_EN_GPIO_Port,            STEPPER_EN_Pin,            FALSE);
FastIO Robot::m_stepperResetPin (STEPPER_RESET_GPIO_Port,         STEPPER_RESET_Pin,         FALSE);

FastIO Robot::m_yAxisStepPin(STEPPER_Y_STEP_GPIO_Port,    STEPPER_Y_STEP_Pin,    FALSE);
FastIO Robot::m_yAxisDirPin (STEPPER_Y_DIR_GPIO_Port,     STEPPER_Y_DIR_Pin,     FALSE);
FastIO Robot::m_tAxisStepPin(STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin, FALSE);
FastIO Robot::m_tAxisDirPin (STEPPER_TEAR_DIR_GPIO_Port,  STEPPER_TEAR_DIR_Pin,  FALSE);

FastIO Robot::m_clampLowPin (DRIVES_SOL3L_GPIO_Port, DRIVES_SOL3L_Pin, TRUE);
FastIO Robot::m_sol1LowPin  (DRIVES_SOL1L_GPIO_Port, DRIVES_SOL1L_Pin, TRUE);
FastIO Robot::m_sol1HighPin (DRIVES_SOL1H_GPIO_Port, DRIVES_SOL1H_Pin, FALSE);
FastIO Robot::m_sol2LowPin  (DRIVES_SOL2L_GPIO_Port, DRIVES_SOL2L_Pin, TRUE);
FastIO Robot::m_sol2HighPin (DRIVES_SOL2H_GPIO_Port, DRIVES_SOL2H_Pin, FALSE);

FastIO Robot::m_contactSensorPin    (CONTACT_SENSORS_TIP_GPIO_Port,         CONTACT_SENSORS_TIP_Pin,         FALSE);
FastIO Robot::m_mouseRightButtonPin (CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port, CONTACT_SENSORS_MOUSE_RIGHT_Pin, FALSE);
FastIO Robot::m_mouseLeftButtonPin  (CONTACT_SENSORS_MOUSE_LEFT_GPIO_Port,  CONTACT_SENSORS_MOUSE_LEFT_Pin,  FALSE);
FastIO Robot::m_yAxisLimitSwitchPin (CONTACT_SENSORS_YLIM_GPIO_Port,        CONTACT_SENSORS_YLIM_Pin,        FALSE);

// -----------------------------------------------------------------------------
// I/O expander bus
// -----------------------------------------------------------------------------
IoExpanderService Robot::m_ioExpanderService(&hi2c1);

Pca9538ExpanderChannel Robot::m_lcdExpanderChannel(LCD_EXPANDER_I2C_ADDRESS, LCD_EXPANDER_DIRECTION);
Pca9535ExpanderChannel Robot::m_keypadExpanderChannel(KEYPAD_EXPANDER_I2C_ADDRESS, KEYPAD_EXPANDER_PORT0_DIR, KEYPAD_EXPANDER_PORT1_DIR);

// -----------------------------------------------------------------------------
// Signal channels  —  ADC
// -----------------------------------------------------------------------------
IQDemodulatorChannel Robot::m_ultrasonicVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_VSENS_GAIN);

IQDemodulatorChannel Robot::m_ultrasonicIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_ISENS_GAIN);

RawAdcChannel Robot::m_scannerVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    Robot::m_scannerVsensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

RawAdcChannel Robot::m_scannerIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    Robot::m_scannerIsensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

AdcTickSyncChannel Robot::m_ultrasonicTickSyncChannel;

AnalogChannel Robot::m_tachometerChannel(
    ADC_CHANNEL_ZMOTOR_TACHOMETER_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_ZMOTOR_TACHOMETER_OVERSAMPLING_RATIO),
    ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC,
    -ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE *
        ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC);

IQDemodulatorChannel Robot::m_lvdtAChannel(
    ADC_CHANNEL_LVDT_A_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

IQDemodulatorChannel Robot::m_lvdtBChannel(
    ADC_CHANNEL_LVDT_B_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

AnalogChannel Robot::m_forceCoilISensChannel(
    ADC_CHANNEL_FORCE_COIL_ISENS_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_FORCE_COIL_ISENS_OVERSAMPLING_RATIO),
    FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR,
    -FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR * FORCE_COIL_MODULE_ZERO_CURRENT_VOLTAGE);

// -----------------------------------------------------------------------------
// Signal channels  —  DAC
// -----------------------------------------------------------------------------
SineGeneratorChannel Robot::m_ultrasonicDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    Robot::m_dac1Buffer, 2 * DAC1_SAMPLES,
    DAC1_SAMPLES, DAC1_BITS, DAC1_VOLTAGE_RANGE);

RawDacChannel Robot::m_scannerDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    Robot::m_dac1Buffer, 2 * DAC1_SAMPLES,
    Robot::m_scannerSynthesisBuffer, SCANNER_SYNTHESIS_BUFFER_SIZE);

SineGeneratorChannel Robot::m_lvdtExcitationChannel(
    &hdac, &htim5, DAC_CHANNEL_2,
    Robot::m_dac2Buffer, 2 * DAC2_SAMPLES,
    DAC2_SAMPLES, DAC2_BITS, DAC2_VOLTAGE_RANGE);

// -----------------------------------------------------------------------------
// Signal channels  —  PWM
// -----------------------------------------------------------------------------
PwmRampChannel Robot::m_forceCoilPwmChannel(
    &htim1, TIM_CHANNEL_1,
    Robot::m_tim1pwmChannel1Buffer, 2 * TIM1_PWM_CHANNEL1_SAMPLES,
    TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH1 (PWM) + CH1N (nPWM)

PwmRampChannel Robot::m_zMotorPwmChannel(
    &htim1, TIM_CHANNEL_2,
    Robot::m_tim1pwmChannel2Buffer, 2 * TIM1_PWM_CHANNEL2_SAMPLES,
    TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH2 (PWM) + CH2N (nPWM)

DirectPwmChannel Robot::m_spotlightPwmChannel(&htim8, TIM_CHANNEL_1);
DirectPwmChannel Robot::m_areaLightPwmChannel(&htim8, TIM_CHANNEL_2);

// -----------------------------------------------------------------------------
// Timers
// -----------------------------------------------------------------------------
TimerExpireService Robot::m_timerExpireService;

Timer Robot::m_clampSolenoidTimer;
Timer Robot::m_sol1SolenoidTimer;
Timer Robot::m_sol2SolenoidTimer;
Timer Robot::m_bonderTimer;
Timer Robot::m_pinMonitorCriticalTimer;
Timer Robot::m_pinMonitorNormalTimer;
Timer Robot::m_controlPanelPollTimer;
Timer Robot::m_lcdDelayTimer;

// -----------------------------------------------------------------------------
// Peripheral services  —  ADC / DAC / PWM / stepper bus schedulers
// -----------------------------------------------------------------------------
AdcService Robot::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE, ADC1_NUM_CONVERSIONS,
    Robot::m_adc1Buffer, 2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

AdcService Robot::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE, ADC2_NUM_CONVERSIONS,
    Robot::m_adc2Buffer, 2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

DacService     Robot::m_dacService(&hdac);
PwmService     Robot::m_tim1PwmService;
StepperService Robot::m_stepperService(&htim6, &Robot::m_stepperEnablePin, &Robot::m_stepperResetPin);

// -----------------------------------------------------------------------------
// Input monitors  —  pin monitor service + panel buttons
// -----------------------------------------------------------------------------
PinMonitorService Robot::m_pinMonitorService(&Robot::m_pinMonitorCriticalTimer, &Robot::m_pinMonitorNormalTimer);
// All contact-sensor inputs are behind active-low optocoupler front-ends.
PinMonitorChannel Robot::m_contactSensorChannel   (&Robot::m_contactSensorPin,    PinMonitorChannel::Level::LOW);
PinMonitorChannel Robot::m_mouseRightButtonChannel(&Robot::m_mouseRightButtonPin, PinMonitorChannel::Level::LOW);
PinMonitorChannel Robot::m_mouseLeftButtonChannel (&Robot::m_mouseLeftButtonPin,  PinMonitorChannel::Level::LOW);
PinMonitorChannel Robot::m_yAxisLimitSwitchChannel(&Robot::m_yAxisLimitSwitchPin, PinMonitorChannel::Level::LOW);

ControlPanelService Robot::m_controlPanelService(&Robot::m_keypadExpanderChannel, &Robot::m_controlPanelPollTimer);

// Value and navigation keys auto-repeat while held; the menu decides what a
// repeat means per page and how far it steps. Everything below them is
// deliberately press-only: one hold must never save, load, delete or toggle
// more than once.
#define KEYPAD_BTN_REPEAT KEYPAD_BTN_REPEAT_THRESHOLD_MS, KEYPAD_BTN_REPEAT_INTERVAL_MS

ButtonChannel Robot::m_btnUp          (KEYPAD_BTN_UP_A,           KEYPAD_BTN_UP_B,           KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnDown        (KEYPAD_BTN_DOWN_A,         KEYPAD_BTN_DOWN_B,         KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnLeft        (KEYPAD_BTN_LEFT_A,         KEYPAD_BTN_LEFT_B,         KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnRight       (KEYPAD_BTN_RIGHT_A,        KEYPAD_BTN_RIGHT_B,        KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnPlus        (KEYPAD_BTN_PLUS_A,         KEYPAD_BTN_PLUS_B,         KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnMinus       (KEYPAD_BTN_MINUS_A,        KEYPAD_BTN_MINUS_B,        KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnSave        (KEYPAD_BTN_SAVE_A,         KEYPAD_BTN_SAVE_B);
ButtonChannel Robot::m_btnLoad        (KEYPAD_BTN_LOAD_A,         KEYPAD_BTN_LOAD_B);
ButtonChannel Robot::m_btnEnter       (KEYPAD_BTN_ENTER_A,        KEYPAD_BTN_ENTER_B);
ButtonChannel Robot::m_btnTailPlus    (KEYPAD_BTN_TAIL_PLUS_A,    KEYPAD_BTN_TAIL_PLUS_B,    KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnTailMinus   (KEYPAD_BTN_TAIL_MINUS_A,   KEYPAD_BTN_TAIL_MINUS_B,   KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnLoopPlus    (KEYPAD_BTN_LOOP_PLUS_A,    KEYPAD_BTN_LOOP_PLUS_B,    KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnLoopMinus   (KEYPAD_BTN_LOOP_MINUS_A,   KEYPAD_BTN_LOOP_MINUS_B,   KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnSearchPlus  (KEYPAD_BTN_SEARCH_PLUS_A,  KEYPAD_BTN_SEARCH_PLUS_B,  KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnSearchMinus (KEYPAD_BTN_SEARCH_MINUS_A, KEYPAD_BTN_SEARCH_MINUS_B, KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnStepPlus    (KEYPAD_BTN_STEP_PLUS_A,    KEYPAD_BTN_STEP_PLUS_B,    KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnStepMinus   (KEYPAD_BTN_STEP_MINUS_A,   KEYPAD_BTN_STEP_MINUS_B,   KEYPAD_BTN_REPEAT);
ButtonChannel Robot::m_btnReset       (KEYPAD_BTN_RESET_A,         KEYPAD_BTN_RESET_B);
ButtonChannel Robot::m_btnManual      (KEYPAD_BTN_MANUAL_A,        KEYPAD_BTN_MANUAL_B);
ButtonChannel Robot::m_btnEscDel      (KEYPAD_BTN_ESC_DEL_A,       KEYPAD_BTN_ESC_DEL_B);
ButtonChannel Robot::m_btnAdd         (KEYPAD_BTN_ADD_A,           KEYPAD_BTN_ADD_B);
ButtonChannel Robot::m_btnTest        (KEYPAD_BTN_TEST_A,          KEYPAD_BTN_TEST_B);
ButtonChannel Robot::m_btnSetup       (KEYPAD_BTN_SETUP_A,         KEYPAD_BTN_SETUP_B);
ButtonChannel Robot::m_btnLight       (KEYPAD_BTN_LIGHT_A,         KEYPAD_BTN_LIGHT_B);
ButtonChannel Robot::m_btnClampOpen   (KEYPAD_BTN_CLAMP_OPEN_A,    KEYPAD_BTN_CLAMP_OPEN_B);
ButtonChannel Robot::m_btnHighReset   (KEYPAD_BTN_HIGH_RESET_A,    KEYPAD_BTN_HIGH_RESET_B);

LedChannel Robot::m_ledTest     (KEYPAD_LED_TEST);
LedChannel Robot::m_ledSetup    (KEYPAD_LED_SETUP);
LedChannel Robot::m_ledClampOpen(KEYPAD_LED_CLAMP_OPEN);
LedChannel Robot::m_ledManual   (KEYPAD_LED_MANUAL);

// -----------------------------------------------------------------------------
// Output actuators
// -----------------------------------------------------------------------------
// The clamp solenoid is non-latching: it holds the clamp open only while
// energized. Driven through TIM12_CH2 (PB15/DRIVES_SOL3H) instead of a
// plain digital pin so the energized hold can run below 100% duty --
// continuous full-voltage hold was overheating the coil. The coil is rated
// below the supply rail, so the hold duty is scaled down to match. The
// initial ratio here just seeds the channel; the real hold duty is
// recomputed fresh from MachineSettingsData::clampSolenoidVoltage right
// before every energize() call (see onClampOpenButtonPressed).
DirectPwmChannel Robot::m_clampPwmChannel(&htim12, TIM_CHANNEL_2);
PwmSolenoidChannel Robot::m_clampSolenoidChannel(
    &Robot::m_clampPwmChannel,
    &Robot::m_clampSolenoidTimer,
    CLAMP_SOLENOID_ENERGIZE_TIME,
    CLAMP_SOLENOID_DEENERGIZE_TIME,
    CLAMP_SOLENOID_VOLTAGE_DEFAULT / CLAMP_SOLENOID_SUPPLY_VOLTAGE);

DirectSolenoidChannel Robot::m_sol1SolenoidChannel(
    &Robot::m_sol1HighPin,
    &Robot::m_sol1LowPin,
    &Robot::m_sol1SolenoidTimer,
    SOL1_SOLENOID_ENERGIZE_TIME,
    SOL1_SOLENOID_DEENERGIZE_TIME);

DirectSolenoidChannel Robot::m_sol2SolenoidChannel(
    &Robot::m_sol2HighPin,
    &Robot::m_sol2LowPin,
    &Robot::m_sol2SolenoidTimer,
    SOL2_SOLENOID_ENERGIZE_TIME,
    SOL2_SOLENOID_DEENERGIZE_TIME);

SolenoidService Robot::m_solenoidService;

// -----------------------------------------------------------------------------
// Motion  —  stepper channels and router
// -----------------------------------------------------------------------------
StepperChannel Robot::m_yAxisStepperChannel(&Robot::m_yAxisStepPin, &Robot::m_yAxisDirPin);
StepperChannel Robot::m_tAxisStepperChannel(&Robot::m_tAxisStepPin, &Robot::m_tAxisDirPin);

RouterChannel Robot::m_yAxisRouterChannel(
    &Robot::m_yAxisStepperChannel,
    ROBOT_Y_AXIS_MAX_VELOCITY,
    ROBOT_Y_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_Y_AXIS_STEPS_PER_MM);

RouterChannel Robot::m_tAxisRouterChannel(
    &Robot::m_tAxisStepperChannel,
    ROBOT_T_AXIS_MAX_VELOCITY,
    ROBOT_T_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_T_AXIS_STEPS_PER_MM);

StepperRouterService Robot::m_routerService;

// -----------------------------------------------------------------------------
// Subsystem modules  —  physical plant control
// -----------------------------------------------------------------------------
LvdtSensorModule Robot::m_lvdtSensorModule(
    &Robot::m_lvdtExcitationChannel,
    &Robot::m_lvdtAChannel,
    &Robot::m_lvdtBChannel,
    LVDT_MODULE_STROKE_MM);

ForceCoilDriverModule Robot::m_forceCoilControllerModule(
    &Robot::m_forceCoilISensChannel,
    &Robot::m_forceCoilPwmChannel);

DcMotorVelocityControllerModule Robot::m_zMotorVelocityControllerModule(
    &Robot::m_tachometerChannel,
    &Robot::m_zMotorPwmChannel);

DcMotorPositionControllerModule Robot::m_zMotorPositionControllerModule(
    &Robot::m_lvdtSensorModule,
    &Robot::m_zMotorVelocityControllerModule);

HomingModule Robot::m_yAxisHomingModule(
    &Robot::m_yAxisRouterChannel,
    &Robot::m_yAxisLimitSwitchChannel);

PllModule Robot::m_pllModule(
    &Robot::m_ultrasonicDacChannel,
    &Robot::m_ultrasonicVsensChannel,
    &Robot::m_ultrasonicIsensChannel,
    &Robot::m_ultrasonicTickSyncChannel,
    static_cast<float>(ADC1_SAMPLING_FREQ),
    static_cast<float>(PLL_MODULE_CONTROL_FREQ));

UsImpedanceScannerModule Robot::m_impedanceScannerModule(
    &Robot::m_scannerDacChannel,
    &Robot::m_scannerVsensChannel,
    &Robot::m_scannerIsensChannel,
    Robot::m_scannerSynthesisBuffer,
    SCANNER_SYNTHESIS_BUFFER_SIZE,
    BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES,
    BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY,
    BONDER_MODULE_DEFAULT_SCAN_FREQUENCY_STEP,
    static_cast<float>(DAC1_SAMPLING_FREQ),
    DAC1_BITS,
    DAC1_VOLTAGE_RANGE,
    static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC1_BITS,
    ADC1_VOLTAGE_RANGE,
    ADC_CHANNEL_US_VSENS_GAIN,
    ADC_CHANNEL_US_ISENS_GAIN,
    SCANNER_WARMUP_ITERATIONS);

// -----------------------------------------------------------------------------
// LCD
// -----------------------------------------------------------------------------
LcdControllerModule Robot::m_lcdControllerModule(&Robot::m_lcdExpanderChannel, &Robot::m_lcdDelayTimer);

// -----------------------------------------------------------------------------
// Bonder  —  top-level bonding state machine
// -----------------------------------------------------------------------------
BonderModule Robot::m_bonderModule(
    &Robot::m_zMotorPositionControllerModule,
    &Robot::m_forceCoilControllerModule,
    &Robot::m_yAxisRouterChannel,
    &Robot::m_tAxisRouterChannel,
    &Robot::m_pllModule,
    &Robot::m_impedanceScannerModule,
    &Robot::m_clampSolenoidChannel,
    &Robot::m_contactSensorChannel,
    &Robot::m_bonderTimer);

// -----------------------------------------------------------------------------
// Persistence  —  EEPROM emulator and live bonding configuration
// -----------------------------------------------------------------------------
EepromEmulator Robot::m_eepromEmulator;
ConfigurationManager Robot::m_configurationManager(&Robot::m_eepromEmulator);
BonderConfig Robot::m_activeBonderConfiguration{};
MachineSettingsStore Robot::m_machineSettingsStore(&Robot::m_eepromEmulator);


// -----------------------------------------------------------------------------
// User interface  —  LCD, configuration editor, and operator controls
// -----------------------------------------------------------------------------
UserInterfaceModule Robot::m_userInterfaceModule(
    &Robot::m_lcdControllerModule,
    &Robot::m_configurationManager,
    &Robot::m_activeBonderConfiguration,
    &Robot::m_machineSettingsStore,
    Menu::NavigationButtons{
        &Robot::m_btnUp,           &Robot::m_btnDown,
        &Robot::m_btnLeft,         &Robot::m_btnRight
    },
    Menu::ConfigurationButtons{
        &Robot::m_btnPlus,         &Robot::m_btnMinus,
        &Robot::m_btnSave,         &Robot::m_btnLoad,
        &Robot::m_btnEnter,        &Robot::m_btnAdd,
        &Robot::m_btnEscDel,
        &Robot::m_btnTailPlus,     &Robot::m_btnTailMinus,
        &Robot::m_btnLoopPlus,     &Robot::m_btnLoopMinus,
        &Robot::m_btnSearchPlus,   &Robot::m_btnSearchMinus,
        &Robot::m_btnStepPlus,     &Robot::m_btnStepMinus
    },
    &Robot::m_ledManual,
    &Robot::m_mouseRightButtonChannel,
    &Robot::m_mouseLeftButtonChannel,
    UserInterfaceModule::ControlPanelButtons{
        &Robot::m_btnSetup,
        &Robot::m_btnTest,
        &Robot::m_btnReset,
        &Robot::m_btnClampOpen,
        &Robot::m_btnLight,
        &Robot::m_btnManual
    });

// -----------------------------------------------------------------------------
// Request prototypes and the render buffer they share
// -----------------------------------------------------------------------------
char RobotRequest::m_renderBuffer[RobotRequest::MSG_RENDER_BUFFER_CAPACITY];

RobotRequest Robot::m_requestQueueContainer[Robot::REQUEST_QUEUE_DEPTH + 1];

const RobotRequest Robot::m_homingRequest(
    RobotRequest::RequestCode::Home, "HOMING", true);
const RobotRequest Robot::m_initializeRequest(
    RobotRequest::RequestCode::Initialize, "BONDER INIT", true);
const RobotRequest Robot::m_testUsRequest(
    RobotRequest::RequestCode::TestUs, "US TEST", false);
const RobotRequest Robot::m_testForceRequest(
    RobotRequest::RequestCode::TestForce, "FORCE TEST", false);
const RobotRequest Robot::m_calibrateTachometerRequest(
    RobotRequest::RequestCode::CalibrateTachometer, "TACH. CAL.", false);
const RobotRequest Robot::m_executeBondingRequest(
    RobotRequest::RequestCode::ExecuteBondingProtocol, "BONDING", false);

// =============================================================================
// Constructor Robot Request
// =============================================================================
RobotRequest::RobotRequest()
    : m_requestCode(RequestCode::Home),
      m_requestState(RequestState::Pending),
      m_requestName(NULL),
      m_infoQueueContainer{},
      m_success(false),
      m_critical(false)
{
}

RobotRequest::RobotRequest(RequestCode requestCode, const char *requestMessage, bool critical)
    : m_requestCode(requestCode),
      m_requestState(RequestState::Pending),
      m_requestName(requestMessage),
      m_infoQueueContainer{},
      m_success(false),
      m_critical(critical)
{
}

/* Both copy operations deliberately leave m_infoQueue bound to this object's
   own container and empty; see the note in robot.hpp. */
RobotRequest::RobotRequest(const RobotRequest &other)
    : m_requestCode(other.m_requestCode),
      m_requestState(other.m_requestState),
      m_requestName(other.m_requestName),
      m_infoQueueContainer{},
      m_success(other.m_success),
      m_critical(other.m_critical)
{
}

RobotRequest &RobotRequest::operator=(const RobotRequest &other)
{
    if (this != &other) {
        m_requestCode  = other.m_requestCode;
        m_requestState = other.m_requestState;
        m_requestName  = other.m_requestName;
        m_success      = other.m_success;
        m_critical     = other.m_critical;
        m_infoQueue.clear();
    }
    return *this;
}

void RobotRequest::setRequestMessage(const char *msg)
{
    m_requestName = msg;
}

bool RobotRequest::appendInfoMessage(const char *msg)
{
    if (msg == NULL) {
        return false;
    }

    return m_infoQueue.enqueue(msg);
}

char *RobotRequest::requestStarted(bool success)
{
    if (m_infoQueue.isFull()) {
        m_infoQueue.discard(2);
    }

    static const char msg_success[] = "STARTED";
    static const char msg_lock[]    = "ERROR SYSTEM LOCKED";
    static const char msg_fail[]    = "CAN'T START";

    m_infoQueue.enqueue(m_requestName);
    m_success = success;

    if (success) {
        m_infoQueue.enqueue(msg_success);
        m_requestState = RequestState::Active;
    } else {
        if (m_critical) {
            m_infoQueue.enqueue(msg_lock);
            m_requestState = RequestState::Locked;
        } else {
            m_infoQueue.enqueue(msg_fail);
            m_requestState = RequestState::Completed;
        }
    }

    return flushQueueToMessage();
}

char *RobotRequest::requestCompleted(bool success)
{
    if (m_infoQueue.isFull()) {
        m_infoQueue.discard(1);
    }

    static const char msg_success[] = "OPERATION SUCCEEDED";
    static const char msg_lock[]    = "ERROR SYSTEM LOCKED";
    static const char msg_fail[]    = "OPERATION FAILED";

    m_success = success;

    if (success) {
        m_infoQueue.enqueue(msg_success);
        m_requestState = RequestState::Completed;
    } else {
        if (m_critical) {
            m_infoQueue.enqueue(msg_lock);
            m_requestState = RequestState::Locked;
        } else {
            m_infoQueue.enqueue(msg_fail);
            m_requestState = RequestState::Completed;
        }
    }

    return flushQueueToMessage();
}

char *RobotRequest::flushQueueToMessage(void)
{
    uint8_t length = 0;
    const uint8_t limit = MSG_RENDER_BUFFER_CAPACITY - 1;

    while (m_infoQueue.getElementCount() > 0) {
        const char *line = m_infoQueue.dequeue();
        if (line == NULL) {
            continue;
        }

        size_t lineLength = strlen(line);
        if (lineLength > (size_t)(limit - length)) {
            lineLength = (size_t)(limit - length);
        }

        memcpy(&m_renderBuffer[length], line, lineLength);
        length += (uint8_t)lineLength;

        if (length < limit) {
            m_renderBuffer[length++] = '\n';
        }
    }

    m_renderBuffer[length] = '\0';
    return m_renderBuffer;
}

// =============================================================================
// Constructor Robot
// =============================================================================
Robot::Robot()
{
    // Timers.
    m_timerExpireService.addTimer(&m_clampSolenoidTimer,      false);
    m_timerExpireService.addTimer(&m_sol1SolenoidTimer,       false);
    m_timerExpireService.addTimer(&m_sol2SolenoidTimer,       false);
    m_timerExpireService.addTimer(&m_bonderTimer,             false);
    m_timerExpireService.addTimer(&m_pinMonitorCriticalTimer, true);
    m_timerExpireService.addTimer(&m_pinMonitorNormalTimer,   false);
    m_timerExpireService.addTimer(&m_controlPanelPollTimer,   false);
    m_timerExpireService.addTimer(&m_lcdDelayTimer,           false);

    // I/O expander bus.
    m_ioExpanderService.addExpander(&m_lcdExpanderChannel);
    m_ioExpanderService.addExpander(&m_keypadExpanderChannel);

    // ADC1 channels. VSENS before ISENS so voltage dispatches first.
    m_adc1Service.addChannel(&m_ultrasonicVsensChannel);
    m_adc1Service.addChannel(&m_ultrasonicIsensChannel);
    m_adc1Service.addChannel(&m_scannerVsensChannel);
    m_adc1Service.addChannel(&m_scannerIsensChannel);
    m_adc1Service.addChannel(&m_ultrasonicTickSyncChannel);

    // ADC2 channels (conversion-order ascending).
    m_adc2Service.addChannel(&m_forceCoilISensChannel);
    m_adc2Service.addChannel(&m_tachometerChannel);
    m_adc2Service.addChannel(&m_lvdtAChannel);
    m_adc2Service.addChannel(&m_lvdtBChannel);

    // DAC channels.
    m_dacService.addChannel(&m_ultrasonicDacChannel);
    m_dacService.addChannel(&m_scannerDacChannel);
    m_dacService.addChannel(&m_lvdtExcitationChannel);

    // PWM channels.
    m_tim1PwmService.addChannel(&m_forceCoilPwmChannel);
    m_tim1PwmService.addChannel(&m_zMotorPwmChannel);

    // Stepper channels.
    m_stepperService.addChannel(&m_yAxisStepperChannel);
    m_stepperService.addChannel(&m_tAxisStepperChannel);

    // Pin monitor channels.
    m_pinMonitorService.addChannel(&m_contactSensorChannel,    true,  PIN_MONITOR_BLIND_REGION_MS);
    m_pinMonitorService.addChannel(&m_yAxisLimitSwitchChannel, true,  PIN_MONITOR_BLIND_REGION_MS);
    m_pinMonitorService.addChannel(&m_mouseRightButtonChannel, true,  PIN_MONITOR_BLIND_REGION_MS);
    m_pinMonitorService.addChannel(&m_mouseLeftButtonChannel,  true,  PIN_MONITOR_BLIND_REGION_MS);

    // Keypad buttons.
    m_controlPanelService.addButton(&m_btnUp);
    m_controlPanelService.addButton(&m_btnDown);
    m_controlPanelService.addButton(&m_btnLeft);
    m_controlPanelService.addButton(&m_btnRight);
    m_controlPanelService.addButton(&m_btnPlus);
    m_controlPanelService.addButton(&m_btnMinus);
    m_controlPanelService.addButton(&m_btnSave);
    m_controlPanelService.addButton(&m_btnLoad);
    m_controlPanelService.addButton(&m_btnEnter);
    m_controlPanelService.addButton(&m_btnTailPlus);
    m_controlPanelService.addButton(&m_btnTailMinus);
    m_controlPanelService.addButton(&m_btnLoopPlus);
    m_controlPanelService.addButton(&m_btnLoopMinus);
    m_controlPanelService.addButton(&m_btnSearchPlus);
    m_controlPanelService.addButton(&m_btnSearchMinus);
    m_controlPanelService.addButton(&m_btnStepPlus);
    m_controlPanelService.addButton(&m_btnStepMinus);
    m_controlPanelService.addButton(&m_btnReset);
    m_controlPanelService.addButton(&m_btnManual);
    m_controlPanelService.addButton(&m_btnEscDel);
    m_controlPanelService.addButton(&m_btnAdd);
    m_controlPanelService.addButton(&m_btnTest);
    m_controlPanelService.addButton(&m_btnSetup);
    m_controlPanelService.addButton(&m_btnLight);
    m_controlPanelService.addButton(&m_btnClampOpen);
    m_controlPanelService.addButton(&m_btnHighReset);

    m_controlPanelService.addLed(&m_ledTest);
    m_controlPanelService.addLed(&m_ledSetup);
    m_controlPanelService.addLed(&m_ledClampOpen);
    m_controlPanelService.addLed(&m_ledManual);

    // Solenoid channels.
    m_solenoidService.addChannel(&m_clampSolenoidChannel);
    m_solenoidService.addChannel(&m_sol1SolenoidChannel);
    m_solenoidService.addChannel(&m_sol2SolenoidChannel);

    // Router channels.
    m_routerService.addChannel(&m_yAxisRouterChannel);
    m_routerService.addChannel(&m_tAxisRouterChannel);

    // Bonder.
    m_bonderModule.addEventListenerCallbacks(
        this, &Robot::onBonderModuleStateChanged, &Robot::onBonderModuleErrorOccurred);
    m_bonderModule.setUltrasonicReportListenerCallback(
        this, &Robot::onUltrasonicReport);
    m_bonderModule.setTachCalReportListenerCallback(
        this, &Robot::onTachCalReport);

    // Y-axis homing.
    m_yAxisHomingModule.addEventListenerCallback(
        this, &Robot::onYAxisHomingEvent);

    // UI events are relayed here for machine-state arbitration.
    m_userInterfaceModule.setEventListenerCallback(this, &Robot::onUserInterfaceEvent);
    m_userInterfaceModule.setMouseButtonListenerCallback(this, &Robot::onMouseButtonEvent);
    m_userInterfaceModule.setControlPanelButtonListenerCallback(
        this, &Robot::onControlPanelButtonEvent);
}

void Robot::start()
{
    m_timerExpireService.start();
    m_ioExpanderService.start();
    m_pinMonitorService.start();
    m_solenoidService.start();
    m_stepperService.start();
    m_routerService.start();
    m_controlPanelService.start();
    m_tim1PwmService.start();
    m_dacService.start();
    m_adc1Service.start();
    m_adc2Service.start();

    m_lcdControllerModule.start();
    m_userInterfaceModule.start();
    m_yAxisHomingModule.start();
    m_forceCoilControllerModule.start();
    m_lvdtSensorModule.start();
    m_zMotorVelocityControllerModule.start();
    m_zMotorPositionControllerModule.start(); 
    m_pllModule.start();
    m_impedanceScannerModule.start();
    m_bonderModule.start();

    m_ledTest.off();
    m_ledSetup.off();
    m_ledClampOpen.off();
    m_ledManual.off();

    /* Both TIM8 channels are already running and parked at their start-up
       levels by MX_TIM8_Init() (area light lit at the default brightness,
       spotlight off), so no start() call here -- re-starting the area light
       at duty 1.0 would blink the lamp off until updateAreaLightDrive()
       below. The stored levels are applied there. */
    m_clampLowPin.clear();

    m_isClampEnergized = false;
    m_isAreaLightEnergized = true;

    /* m_userInterfaceModule.start() has loaded the stored machine settings by
       now; push them into the actuators that depend on them. Without this the
       persisted clamp hold voltage, light levels and tachometer offset would
       only take effect after the operator edits a setting. */
    updateClampDrive();
    updateClampIndicator();
    updateAreaLightDrive();
    updateSpotlightDrive();
    updateTachometerOffsetCorrection();
    updateZPositionSpeedLimits();

    /* Nothing may move until the Y axis has a position reference. */
    m_requestQueue.enqueue(m_homingRequest);
    m_requestQueue.enqueue(m_initializeRequest);
}

void Robot::execute()
{
    /* Execute services. */
    m_timerExpireService.execute();
    m_ioExpanderService.execute();
    m_pinMonitorService.execute();
    m_solenoidService.execute();
    m_stepperService.execute();
    m_routerService.execute();
    m_controlPanelService.execute();
    m_tim1PwmService.execute();
    m_dacService.execute();
    m_adc1Service.execute();
    m_adc2Service.execute();

    m_lcdControllerModule.execute();
    m_userInterfaceModule.execute();
    m_yAxisHomingModule.execute();
    m_forceCoilControllerModule.execute();
    m_lvdtSensorModule.execute();

    m_pllModule.execute();
    m_impedanceScannerModule.execute();
    m_bonderModule.execute();

    if (m_robotState == RobotState::Idle) {
        if (m_requestQueue.getElementCount() > 0) {
            m_activeRequest = m_requestQueue.dequeue();
            startExecutingRequest(m_activeRequest);
        }
    } else if (m_robotState == RobotState::Busy) {
        /* failVm() reports the error and then reports the module going idle,
           so a failure arrives with both bits set. Failure wins, and both are
           consumed so a stale Completed can't terminate the next request. */
        if (m_events & RobotEvents::OperationFailed) {
            m_events &= ~(RobotEvents::OperationFailed |
                          RobotEvents::OperationCompleted);
            completeExecutingRequest(m_activeRequest, false);
        } else if (m_events & RobotEvents::OperationCompleted) {
            m_events &= ~RobotEvents::OperationCompleted;
            completeExecutingRequest(m_activeRequest, true);
        }
    }
}


/* Helpers -------------------------------------------------------------------*/
void Robot::startExecutingRequest(RobotRequest &request)
{
    bool success = false;
    RobotRequest::RequestCode request_code = request.getRequestCode();

    /* Discard anything a previous request left latched. */
    m_events = 0U;

    if (request_code == RobotRequest::RequestCode::Home) {
        success = startHoming();
    }
    else if (request_code == RobotRequest::RequestCode::Initialize) {
        success = startBonderInitializing();
    }
    else if (request_code == RobotRequest::RequestCode::TestUs) {
        success = startUsTest();
    }
    else if (request_code == RobotRequest::RequestCode::TestForce) {
        success = startForceTest();
    }
    else if (request_code == RobotRequest::RequestCode::CalibrateTachometer) {
        success = startTachometerCalibration();
    }
    else if (request_code == RobotRequest::RequestCode::ExecuteBondingProtocol) {
        success = startBondingProtocol();
    }
    
    if (success) {
        m_userInterfaceModule.notifyUser(request.requestStarted(true));
        m_robotState = RobotState::Busy;
    } else {
        /* A critical request that cannot even be started locks the machine. */
        if (request.getCritical()) {
            m_userInterfaceModule.raiseError(request.requestStarted(false));
            m_robotState = RobotState::Error;
        } else {
            m_userInterfaceModule.warnUser(request.requestStarted(false));
            m_robotState = RobotState::Idle;
        }
    }

    updateRequestIndicators();
}

void Robot::completeExecutingRequest(RobotRequest &request, bool success)
{
    /* Flushes whatever the callbacks queued during execution (US report rows,
       bonder error strings) along with the terminal line. */
    char *message = request.requestCompleted(success);

    if (success) {
        m_userInterfaceModule.notifyUser(message);
        m_robotState = RobotState::Idle;

        /* A completed force setup ends with the operator holding a reading
           off their own gauge; ask for it so the offset can be stored. A
           failed or aborted run has nothing meaningful to enter. */
        if (request.getRequestCode() == RobotRequest::RequestCode::TestForce) {
            m_userInterfaceModule.promptForceMeasurement();
        }
    } else if (request.getCritical()) {
        m_userInterfaceModule.raiseError(message);
        m_robotState = RobotState::Error;
    } else {
        m_userInterfaceModule.warnUser(message);
        m_robotState = RobotState::Idle;
    }

    updateRequestIndicators();
}

bool Robot::startHoming()
{
    return m_yAxisHomingModule.home();
}

bool Robot::startBonderInitializing()
{    
    configureBonderModule();
    m_bonderModule.setProtocol(resetProtocol());
    
    return m_bonderModule.engage();
}

bool Robot::startUsTest()
{
    configureBonderModule();
    m_bonderModule.setProtocol(ultrasonicTestProtocol());

    return m_bonderModule.engage();
}

bool Robot::startForceTest()
{
    configureBonderModule();
    m_bonderModule.setProtocol(forceSetupProtocol());

    return m_bonderModule.engage();
}

bool Robot::startTachometerCalibration()
{
    configureBonderModule();
    m_bonderModule.setProtocol(tachCalProtocol());

    return m_bonderModule.engage();
}

bool Robot::startBondingProtocol()
{
    const BondingMode mode =
        m_userInterfaceModule.activeConfiguration().bondingMode;

    configureBonderModule();
    m_bonderModule.setProtocol(protocolForMode(mode));

    return m_bonderModule.engage();
}

void Robot::configureBonderModule()
{
    BonderConfig configuration = m_userInterfaceModule.activeConfiguration();
    const MachineSettingsData &settings = m_machineSettingsStore.data();

    /* Machine-wide, not per profile: whatever the persisted record carried
       in these two fields is irrelevant. */
    configuration.forceSetupTrackingForce = settings.forceSetupTrackingForce;
    configuration.forceCoilForceOffset = settings.forceCoilForceOffset;

    m_bonderModule.configure(configuration);
}

void Robot::turnoffPeripherals(void)
{
    m_ledTest.off();
    m_ledSetup.off();
    m_ledClampOpen.off();
    m_bonderModule.stop();
}

void Robot::updateRequestIndicators()
{
    /* Derived from state rather than toggled at each call site, so the LED
       cannot survive a request that ended down an error path. */
    const bool busy = (m_robotState == RobotState::Busy);
    const RobotRequest::RequestCode code = m_activeRequest.getRequestCode();

    m_ledTest.set(busy &&
        (code == RobotRequest::RequestCode::TestUs));
    m_ledSetup.set(busy &&
        (code == RobotRequest::RequestCode::TestForce));
}

void Robot::updateClampIndicator()
{
    if (m_isClampEnergized){
        m_ledClampOpen.on();
    } else {
        m_ledClampOpen.off();
    }
}

void Robot::updateClampDrive()
{
    m_clampSolenoidChannel.setOnDuty(
        m_machineSettingsStore.data().clampSolenoidVoltage /
        CLAMP_SOLENOID_SUPPLY_VOLTAGE);

    if (m_isClampEnergized) {
        m_clampSolenoidChannel.energize();
    } else {
        m_clampSolenoidChannel.deenergize();
    }
}

void Robot::updateAreaLightDrive()
{
    if (m_isAreaLightEnergized) {
        const float onRatio =
            (m_machineSettingsStore.data().areaLightLevel / 100.0f) *
            (AREA_LIGHT_MAX_VOLTAGE / AREA_LIGHT_SUPPLY_VOLTAGE);
        m_areaLightPwmChannel.setDuty(1.0f - onRatio);
    } else {
        m_areaLightPwmChannel.setDuty(1.0f);
    }
}

void Robot::updateSpotlightDrive()
{
    const MachineSettingsData& data = m_machineSettingsStore.data();
    if (data.spotlightOn) {
        m_spotlightPwmChannel.setDuty(data.spotlightLevel / 100.0f);
    } else {
        m_spotlightPwmChannel.setDuty(0.0f);
    }
}

void Robot::updateTachometerOffsetCorrection()
{
    const MachineSettingsData& data = m_machineSettingsStore.data();
    m_zMotorVelocityControllerModule.setVelocityOffset(data.tachometerVelocityOffset);
}

void Robot::updateZPositionSpeedLimits()
{
    const MachineSettingsData& data = m_machineSettingsStore.data();
    /* Settings hold positive magnitudes; the position loop's lower bound is
       the downward direction, so it takes the negated value. */
    m_zMotorPositionControllerModule.setOutputLimits(
        -data.zMotorMaxDownwardSpeed, data.zMotorMaxUpwardSpeed);
}

static const BonderProtocol& protocolForMode(BondingMode mode)
{
    static const SemiAutoBondingProtocol semiAutoProtocol;
    static const ManualBondingProtocol manualProtocol;
    static const TableTearBondingProtocol tableTearProtocol;
    static const LangeCouplingBondingProtocol langeCouplingProtocol;

    switch (mode) {
    case BondingMode::Manual:
        return manualProtocol;
    case BondingMode::TableTear:
        return tableTearProtocol;
    case BondingMode::LangeCoupling:
        return langeCouplingProtocol;
    case BondingMode::SemiAutomatic:
    default:
        return semiAutoProtocol;
    }
}

static const BonderProtocol& ultrasonicTestProtocol()
{
    static const UltrasonicTestProtocol protocol;
    return protocol;
}

static const BonderProtocol& forceSetupProtocol()
{
    static const ForceSetupProtocol protocol;
    return protocol;
}

static const BonderProtocol& tachCalProtocol()
{
    static const TachCalProtocol protocol;
    return protocol;
}

static const BonderProtocol& resetProtocol()
{
    static const ProtocolResetPrologue protocol;
    return protocol;
}

/* Callbacks -----------------------------------------------------------------*/
void Robot::onYAxisHomingEvent(void *context, HomingModule::Event event)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    if (m_robotState == RobotState::Busy) {
        if (robot->m_activeRequest.getRequestCode() !=
            RobotRequest::RequestCode::Home) {
            return;
        }

        switch (event) {
        case HomingModule::Event::Completed:
            m_events |= RobotEvents::OperationCompleted;
            break;

        case HomingModule::Event::Failed:
            m_events |= RobotEvents::OperationFailed;
            break;

        default:
            /* ClearingLimit / SeekingLimit / BackingOff are progress
            notifications; only the terminal events matter here. */
            break;
        }
    }
}

/* True for the requests that are carried out by running a bonder protocol, so
   the bonder's idle/error reports belong to them. */
static bool isBonderDrivenRequest(RobotRequest::RequestCode code)
{
    return (code == RobotRequest::RequestCode::ExecuteBondingProtocol) ||
           (code == RobotRequest::RequestCode::Initialize) ||
           (code == RobotRequest::RequestCode::TestForce) ||
           (code == RobotRequest::RequestCode::TestUs) ||
           (code == RobotRequest::RequestCode::CalibrateTachometer);
}

void Robot::onBonderModuleStateChanged(void *context, bool isIdle)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;
    if (!isIdle) return;

    if (m_robotState == RobotState::Busy) {
        if (!isBonderDrivenRequest(robot->m_activeRequest.getRequestCode())) {
            return;
        }

        m_events |= RobotEvents::OperationCompleted;
    }
}

void Robot::onBonderModuleErrorOccurred(void *context, BonderModule::Error error)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    if (m_robotState == RobotState::Busy) {
        RobotRequest &active_request = robot->m_activeRequest;
        if (!isBonderDrivenRequest(active_request.getRequestCode())) {
            return;
        }

        m_events |= RobotEvents::OperationFailed;

        switch (error) {
        case BonderModule::Error::InsufficientBondingPower:
            active_request.appendInfoMessage("LOW BONDING POWER");
            break;
        case BonderModule::Error::UnableToSetForceCoilCurrent:
            active_request.appendInfoMessage("COIL CURR. UNSTABLE");
            break;
        case BonderModule::Error::UnableToSetPosition:
            active_request.appendInfoMessage("Z POS. NOT SETTLED");
            break;
        case BonderModule::Error::ProtocolTimeout:
            active_request.appendInfoMessage("TIMEOUT");
            break;
        }
    }
}

void Robot::onTachCalReport(void *context, float offsetResidual)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    if (m_robotState == RobotState::Busy) {
        if (robot->m_activeRequest.getRequestCode() !=
            RobotRequest::RequestCode::CalibrateTachometer) {
            return;
        }

        MachineSettingsData& data = robot->m_machineSettingsStore.mutableData();
        data.tachometerVelocityOffset += offsetResidual;

        robot->m_machineSettingsStore.save();
        robot->updateTachometerOffsetCorrection();
    }
}

void Robot::onUltrasonicReport(
    void *context, const BonderModule::UltrasonicReport& report)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    if (m_robotState == RobotState::Busy) {
        RobotRequest &active_request = robot->m_activeRequest;
        if (active_request.getRequestCode() != RobotRequest::RequestCode::TestUs) {
            return;
        }

        /* The info queue stores the pointers, not the text, and is not
           flushed until the request completes — so these must outlive the
           callback. Only one US report is ever in flight (bonder execute()
           runs from the main loop), so a single static set is enough. */
        static char rows[3][21];
        const uint32_t frequencyHz =
            static_cast<uint32_t>(report.resonanceFrequency + 0.5f);
        const uint32_t qualityTenths =
            static_cast<uint32_t>(report.qualityFactor * 10.0f + 0.5f);
        const uint32_t powerTenths =
            static_cast<uint32_t>(report.transferredPower * 10.0f + 0.5f);

        snprintf(rows[0], sizeof(rows[0]), "Freq:%lu.%03lu kHz",
                static_cast<unsigned long>(frequencyHz / 1000U),
                static_cast<unsigned long>(frequencyHz % 1000U));
        snprintf(rows[1], sizeof(rows[1]), "Q:%lu.%lu",
                static_cast<unsigned long>(qualityTenths / 10U),
                static_cast<unsigned long>(qualityTenths % 10U));
        snprintf(rows[2], sizeof(rows[2]), "Power:%lu.%lu W",
                static_cast<unsigned long>(powerTenths / 10U),
                static_cast<unsigned long>(powerTenths % 10U));

        active_request.appendInfoMessage(rows[0]);
        active_request.appendInfoMessage(rows[1]);
        active_request.appendInfoMessage(rows[2]);
    }
}

void Robot::onUserInterfaceEvent(void *ctx, UserInterfaceModule::Event event)
{
    Robot *self = static_cast<Robot *>(ctx);
    if (self == nullptr) return;


    switch (event) {
    case UserInterfaceModule::Event::ActiveConfigurationChanged:
    case UserInterfaceModule::Event::ConfigurationConfirmed:
    case UserInterfaceModule::Event::ConfigurationSelectionStarted: 
        break;

    case UserInterfaceModule::Event::StartTachCalRequested:
        if ((m_robotState == RobotState::Idle) && \
            (self->m_requestQueue.getElementCount() == 0)) {
             self->m_requestQueue.enqueue(self->m_calibrateTachometerRequest);
        } else {
            m_userInterfaceModule.warnUser("BUSY! TRY LATER");
        }
        break;

    case UserInterfaceModule::Event::MachineSettingsChanged:
        self->updateClampDrive();
        self->updateAreaLightDrive();
        self->updateSpotlightDrive();
        self->updateZPositionSpeedLimits();
        break;
    }
}

void Robot::onControlPanelButtonEvent(
    void *ctx,
    UserInterfaceModule::ControlPanelButtonEvent event)
{
    Robot *self = static_cast<Robot *>(ctx);

    switch (event) {
    case UserInterfaceModule::ControlPanelButtonEvent::SetupPressed:
        if ((m_robotState == RobotState::Idle) && \
            (self->m_requestQueue.getElementCount() == 0)) {
            self->m_requestQueue.enqueue(self->m_testForceRequest);
        } else {
            m_userInterfaceModule.warnUser("BUSY! TRY LATER");
        }
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::TestPressed:
        if ((m_robotState == RobotState::Idle) && \
            (self->m_requestQueue.getElementCount() == 0)) {
            self->m_requestQueue.enqueue(self->m_testUsRequest);
        } else {
            m_userInterfaceModule.warnUser("BUSY! TRY LATER");
        }
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::ClampOpenPressed:
        if ((m_robotState == RobotState::Idle) && \
            (self->m_requestQueue.getElementCount() == 0)) {
            self->m_isClampEnergized = !self->m_isClampEnergized;

            self->updateClampIndicator();
            self->updateClampDrive();
        } else {
            m_userInterfaceModule.warnUser("BUSY! TRY LATER");   
        }
        break; 

    case UserInterfaceModule::ControlPanelButtonEvent::LightPressed:
        self->m_isAreaLightEnergized = !self->m_isAreaLightEnergized;
        self->updateAreaLightDrive();
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::ResetPressed:
        NVIC_SystemReset();
        break;
    }
}

void Robot::onMouseButtonEvent(void *ctx, UserInterfaceModule::MouseButtonEvent event)
{
    Robot *self = static_cast<Robot *>(ctx);
    if (self == nullptr) return;
    if (m_robotState == RobotState::Error) return;

    if (m_robotState == RobotState::Idle) {
        if ((self->m_requestQueue.getElementCount() == 0) && \
            (event == UserInterfaceModule::MouseButtonEvent::RightPressed)) {
            self->m_requestQueue.enqueue(self->m_executeBondingRequest);
        }
    } 
    else if (m_robotState == RobotState::Busy) {
        /* Only protocols that read the mouse get the events. Force setup is
           driven entirely by the right button (press to descend under the
           tracking force, release to finish), so it has to be relayed too —
           without this its opening WAIT never completes and the machine sits
           in Busy until reset. */
        const RobotRequest::RequestCode code =
            self->m_activeRequest.getRequestCode();
        if ((code != RobotRequest::RequestCode::ExecuteBondingProtocol) &&
            (code != RobotRequest::RequestCode::TestForce)) {
            return;
        }

        switch (event) {
        case UserInterfaceModule::MouseButtonEvent::RightPressed:
            m_bonderModule.notifyRightButton(true);
            break;
        case UserInterfaceModule::MouseButtonEvent::RightReleased:
            m_bonderModule.notifyRightButton(false);
            break;
        case UserInterfaceModule::MouseButtonEvent::LeftPressed:
            m_bonderModule.notifyLeftButton(true);
            break;
        case UserInterfaceModule::MouseButtonEvent::LeftReleased:
            m_bonderModule.notifyLeftButton(false);
            break;
        }
    }
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_NORMAL
