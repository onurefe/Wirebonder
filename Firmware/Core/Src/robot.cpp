#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_NORMAL

#include "robot.hpp"
#include "main.h"
#include "protocol_semi_auto.hpp"
#include "protocol_manual.hpp"
#include "protocol_table_tear.hpp"
#include "protocol_lange_coupling.hpp"
#include "protocol_ultrasonic_test.hpp"
#include "protocol_force_setup.hpp"
#include <cstring>

// Maps the configured bonding mode to its protocol program; BonderModule
// itself is protocol-agnostic.
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

extern I2C_HandleTypeDef hi2c1;

// =============================================================================
// Static member definitions
//
// Ordered lowest-complexity first, highest last — mirrors robot.hpp.
// All HAL handles are passed by address only; actual HAL activity begins in
// start() after MX_xxx_Init() has completed and Robot_Init() constructs Robot.
// =============================================================================

// -----------------------------------------------------------------------------
// Flags
// -----------------------------------------------------------------------------
bool Robot::m_bonderConfigUpdated = false;
bool Robot::m_ultrasonicTestActive = false;
bool Robot::m_forceSetupActive = false;
bool Robot::m_manualClampOpen = false;
bool Robot::m_manualClampCommandActive = false;
bool Robot::m_systemLocked = false;

// Startup lists, dependency order (matches the Component enum ordering).
const Robot::Component Robot::kBootComponents[] = {
    Component::TimerExpireService,
    Component::IoExpanderService,
    Component::PinMonitorService,
    Component::SolenoidService,
    Component::StepperService,
    Component::RouterService,
    Component::ControlPanelService,
    Component::LcdControllerModule,
    Component::UserInterfaceModule,
    Component::HomingModule
};

const uint8_t Robot::kBootComponentCount =
    sizeof(Robot::kBootComponents) / sizeof(Robot::kBootComponents[0]);

const Robot::Component Robot::kBonderComponents[] = {
    Component::Tim1PwmService,
    Component::DacService,
    Component::Adc1Service,
    Component::Adc2Service,
    Component::ForceCoilModule,
    Component::ZMotorVelocityModule,
    Component::LvdtModule,
    Component::ZMotorPositionModule,
    Component::PllModule,
    Component::ImpedanceScannerModule,
    Component::BonderModule
};

const uint8_t Robot::kBonderComponentCount =
    sizeof(Robot::kBonderComponents) / sizeof(Robot::kBonderComponents[0]);

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

// Sol1 and Sol2 drivers are unavailable. The clamp coil is connected to the
// working Sol3 driver, so keep the logical clamp names mapped to Sol3.
FastIO Robot::m_clampLowPin (DRIVES_SOL3L_GPIO_Port, DRIVES_SOL3L_Pin, TRUE);
FastIO Robot::m_clampHighPin(DRIVES_SOL3H_GPIO_Port, DRIVES_SOL3H_Pin, FALSE);
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

// PC6 / TIM8 CH1 drives an active-low area-light PWM input.
DirectPwmChannel Robot::m_areaLightPwmChannel(&htim8, TIM_CHANNEL_1);

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

ButtonChannel Robot::m_btnUp          (KEYPAD_BTN_UP_A,           KEYPAD_BTN_UP_B);
ButtonChannel Robot::m_btnDown        (KEYPAD_BTN_DOWN_A,         KEYPAD_BTN_DOWN_B);
ButtonChannel Robot::m_btnLeft        (KEYPAD_BTN_LEFT_A,         KEYPAD_BTN_LEFT_B);
ButtonChannel Robot::m_btnRight       (KEYPAD_BTN_RIGHT_A,        KEYPAD_BTN_RIGHT_B);
ButtonChannel Robot::m_btnPlus        (KEYPAD_BTN_PLUS_A,         KEYPAD_BTN_PLUS_B);
ButtonChannel Robot::m_btnMinus       (KEYPAD_BTN_MINUS_A,        KEYPAD_BTN_MINUS_B);
ButtonChannel Robot::m_btnSave        (KEYPAD_BTN_SAVE_A,         KEYPAD_BTN_SAVE_B);
ButtonChannel Robot::m_btnLoad        (KEYPAD_BTN_LOAD_A,         KEYPAD_BTN_LOAD_B);
ButtonChannel Robot::m_btnEnter       (KEYPAD_BTN_ENTER_A,        KEYPAD_BTN_ENTER_B);
ButtonChannel Robot::m_btnTailPlus    (KEYPAD_BTN_TAIL_PLUS_A,    KEYPAD_BTN_TAIL_PLUS_B);
ButtonChannel Robot::m_btnTailMinus   (KEYPAD_BTN_TAIL_MINUS_A,   KEYPAD_BTN_TAIL_MINUS_B);
ButtonChannel Robot::m_btnLoopPlus    (KEYPAD_BTN_LOOP_PLUS_A,    KEYPAD_BTN_LOOP_PLUS_B);
ButtonChannel Robot::m_btnLoopMinus   (KEYPAD_BTN_LOOP_MINUS_A,   KEYPAD_BTN_LOOP_MINUS_B);
ButtonChannel Robot::m_btnSearchPlus  (KEYPAD_BTN_SEARCH_PLUS_A,  KEYPAD_BTN_SEARCH_PLUS_B);
ButtonChannel Robot::m_btnSearchMinus (KEYPAD_BTN_SEARCH_MINUS_A, KEYPAD_BTN_SEARCH_MINUS_B);
ButtonChannel Robot::m_btnStepPlus    (KEYPAD_BTN_STEP_PLUS_A,    KEYPAD_BTN_STEP_PLUS_B);
ButtonChannel Robot::m_btnStepMinus   (KEYPAD_BTN_STEP_MINUS_A,   KEYPAD_BTN_STEP_MINUS_B);
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
// energized, so it is driven through the direct channel.
DirectSolenoidChannel Robot::m_clampSolenoidChannel(
    &Robot::m_clampHighPin,
    &Robot::m_clampLowPin,
    &Robot::m_clampSolenoidTimer,
    CLAMP_SOLENOID_ENERGIZE_TIME,
    CLAMP_SOLENOID_DEENERGIZE_TIME);

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
LcdControllerModule Robot::m_lcdController(&Robot::m_lcdExpanderChannel, &Robot::m_lcdDelayTimer);

// -----------------------------------------------------------------------------
// Bonder  —  top-level bonding state machine
// -----------------------------------------------------------------------------
BonderModule Robot::m_bonder(
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
bool Robot::m_configurationConfirmed = false;
bool Robot::m_bonderStartPending = false;

// -----------------------------------------------------------------------------
// User interface  —  LCD, configuration editor, and operator controls
// -----------------------------------------------------------------------------
UserInterfaceModule Robot::m_userInterface(
    &Robot::m_lcdController,
    &Robot::m_configurationManager,
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
        &Robot::m_btnLight
    });

// =============================================================================
// Constructor
// =============================================================================
Robot::Robot()
{
    m_components[static_cast<uint8_t>(Component::TimerExpireService)] =
        &m_timerExpireService;
    m_components[static_cast<uint8_t>(Component::IoExpanderService)] =
        &m_ioExpanderService;
    m_components[static_cast<uint8_t>(Component::PinMonitorService)] =
        &m_pinMonitorService;
    m_components[static_cast<uint8_t>(Component::SolenoidService)] =
        &m_solenoidService;
    m_components[static_cast<uint8_t>(Component::StepperService)] =
        &m_stepperService;
    m_components[static_cast<uint8_t>(Component::RouterService)] =
        &m_routerService;
    m_components[static_cast<uint8_t>(Component::Tim1PwmService)] =
        &m_tim1PwmService;
    m_components[static_cast<uint8_t>(Component::DacService)] =
        &m_dacService;
    m_components[static_cast<uint8_t>(Component::Adc1Service)] =
        &m_adc1Service;
    m_components[static_cast<uint8_t>(Component::Adc2Service)] =
        &m_adc2Service;
    m_components[static_cast<uint8_t>(Component::ForceCoilModule)] =
        &m_forceCoilControllerModule;
    m_components[static_cast<uint8_t>(Component::ZMotorVelocityModule)] =
        &m_zMotorVelocityControllerModule;
    m_components[static_cast<uint8_t>(Component::LvdtModule)] =
        &m_lvdtSensorModule;
    m_components[static_cast<uint8_t>(Component::ZMotorPositionModule)] =
        &m_zMotorPositionControllerModule;
    m_components[static_cast<uint8_t>(Component::ControlPanelService)] =
        &m_controlPanelService;
    m_components[static_cast<uint8_t>(Component::LcdControllerModule)] = &m_lcdController;
    m_components[static_cast<uint8_t>(Component::UserInterfaceModule)] =
        &m_userInterface;
    m_components[static_cast<uint8_t>(Component::HomingModule)] =
        &m_yAxisHomingModule;
    m_components[static_cast<uint8_t>(Component::PllModule)] =
        &m_pllModule;
    m_components[static_cast<uint8_t>(Component::ImpedanceScannerModule)] =
        &m_impedanceScannerModule;
    m_components[static_cast<uint8_t>(Component::BonderModule)] = &m_bonder;

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
    m_bonder.addEventListenerCallbacks(&Robot::onBonderModuleStateChanged, &Robot::onBonderModuleErrorOccurred);
    m_bonder.setUltrasonicReportListenerCallback(
        this, &Robot::onUltrasonicReport);

    // Y-axis homing.
    m_yAxisHomingModule.addEventListenerCallback(
        this, &Robot::onYAxisHomingEvent);

    // UI events are relayed here for machine-state arbitration.
    m_userInterface.setEventListenerCallback(this, &Robot::onUserInterfaceEvent);
    m_userInterface.setMouseButtonListenerCallback(this, &Robot::onMouseButtonEvent);
    m_userInterface.setControlPanelButtonListenerCallback(
        this, &Robot::onControlPanelButtonEvent);

    m_bonderConfigUpdated = false;
}

void Robot::startComponents(const Component *components, uint8_t count)
{
    for (uint8_t i = 0U; i < count; ++i) {
        startComponent(components[i]);
    }
}

void Robot::startComponent(Component component)
{
    const uint8_t index = static_cast<uint8_t>(component);
    if (index >= kComponentCount || m_components[index] == nullptr) {
        return;
    }

    if (component == Component::HomingModule) {
        m_userInterface.notifyUser("Homing Y axis");
    } else if (component == Component::BonderModule) {
        m_bonder.configure(m_userInterface.activeConfiguration());
        const BonderProtocol& protocol =
            protocolForMode(m_userInterface.activeConfiguration().bondingMode);
        m_bonder.setProtocol(protocol);
    }

    m_components[index]->start();

    if (component == Component::HomingModule &&
        m_components[index]->isOperating()) {
        (void)m_yAxisHomingModule.home();
    } else if (component == Component::BonderModule &&
               m_components[index]->isOperating() &&
               !isBondingInterlocked()) {
        (void)m_bonder.engage();
    }

    if (component == Component::BonderModule) {
        m_bonderConfigUpdated = false;
    }
}

void Robot::stopComponent(Component component)
{
    const uint8_t index = static_cast<uint8_t>(component);
    if (index < kComponentCount && m_components[index] != nullptr) {
        m_components[index]->stop();
    }
}
bool Robot::isYAxisHomed()
{
    return m_yAxisHomingModule.isHomed();
}

bool Robot::isBondingInterlocked()
{
    return m_ultrasonicTestActive || m_forceSetupActive || m_manualClampOpen;
}

void Robot::updateClampCommandLed()
{
    if (!m_manualClampCommandActive) return;

    const DirectSolenoidChannel::State completedState = m_manualClampOpen
        ? DirectSolenoidChannel::State::ENERGIZED
        : DirectSolenoidChannel::State::DEENERGIZED;
    if (!m_clampSolenoidChannel.isTransitioning() &&
        m_clampSolenoidChannel.getState() == completedState) {
        m_manualClampCommandActive = false;
        // The LED mirrors the latched manual-open state, not the command:
        // it stays lit the whole time the clamp is held open.
        m_ledClampOpen.set(m_manualClampOpen);
    }
}

void Robot::restoreConfiguredBondingProtocol()
{
    if (!m_configurationManager.isReady() || m_bonder.isActive()) return;

    const BonderConfig& config = m_userInterface.activeConfiguration();
    m_bonder.configure(config);
    m_bonder.setProtocol(protocolForMode(config.bondingMode));
    m_bonderConfigUpdated = false;
}

void Robot::lockSystem(const char *message)
{
    m_systemLocked = true;
    m_ultrasonicTestActive = false;
    m_forceSetupActive = false;
    m_manualClampCommandActive = false;
    m_ledTest.off();
    m_ledSetup.off();
    m_ledClampOpen.off();
    m_bonder.stop();
    m_userInterface.raiseError(message);
}

void Robot::requestBonderStartIfReady()
{
    // An engaged bonder is already armed (or working); nothing to request.
    if (m_systemLocked || m_bonder.isActive()) return;

    if (m_configurationConfirmed && isYAxisHomed() && !isBondingInterlocked()) {
        m_bonderStartPending = true;
    } else if (m_ultrasonicTestActive) {
        m_userInterface.notifyUser("TEST IN PROGRESS");
    } else if (m_forceSetupActive) {
        m_userInterface.notifyUser("SETUP IN PROGRESS");
    } else if (m_manualClampOpen) {
        m_userInterface.notifyUser("CLOSE CLAMP FIRST");
    }
}

void Robot::start()
{
    m_configurationConfirmed = false;
    m_bonderStartPending = false;
    m_ultrasonicTestActive = false;
    m_forceSetupActive = false;
    m_manualClampOpen = false;
    m_manualClampCommandActive = false;
    m_systemLocked = false;
    m_areaLightOn = false;
    m_ledTest.off();
    m_ledSetup.off();
    m_ledClampOpen.off();
    m_ledManual.off();
    // The light driver is active-low, so a raw duty of 1 keeps it off.
    m_areaLightPwmChannel.start(1.0f);
    // The bonder chain is intentionally absent: configuration confirmation
    // and homing completion request it later through the startup gate.
    startComponents(kBootComponents, kBootComponentCount);
}

void Robot::execute()
{
    if (m_bonderStartPending) {
        m_bonderStartPending = false;
        if (!m_systemLocked && !isBondingInterlocked()) {
            startComponents(kBonderComponents, kBonderComponentCount);
        }
    }

    for (uint8_t i = 0U; i < kComponentCount; ++i) {
        if (m_components[i] != nullptr) m_components[i]->execute();
    }

    updateClampCommandLed();

    if (m_configurationConfirmed &&
        isYAxisHomed() &&
        m_bonderConfigUpdated &&
        m_bonder.isIdle()) {
        m_bonder.configure(m_userInterface.activeConfiguration());
        const BonderProtocol& protocol =
            protocolForMode(m_userInterface.activeConfiguration().bondingMode);
        m_bonder.setProtocol(protocol);
        m_bonderConfigUpdated = false;
    }
}

void Robot::stop()
{
    m_configurationConfirmed = false;
    m_bonderStartPending = false;
    m_ultrasonicTestActive = false;
    m_forceSetupActive = false;
    m_manualClampOpen = false;
    m_manualClampCommandActive = false;
    m_areaLightOn = false;
    m_ledTest.off();
    m_ledSetup.off();
    m_ledClampOpen.off();
    m_ledManual.off();
    // Keep TIM8 running at inactive-high. Stopping the channel would return
    // its active-low output to the configured reset level.
    m_areaLightPwmChannel.setDuty(1.0f);

    for (uint8_t i = kComponentCount; i > 0U; --i) {
        stopComponent(static_cast<Component>(i - 1U));
    }
}

void Robot::onYAxisHomingEvent(void *context, HomingModule::Event event)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    if (event == HomingModule::Event::Completed) {
        robot->m_userInterface.notifyUser("Y axis homed");
        robot->requestBonderStartIfReady();
        return;
    }

    if (event != HomingModule::Event::Failed) {
        // Progress notification; only the terminal events matter here.
        return;
    }

    // Without a Y reference nothing can move safely: latch the machine.
    robot->m_bonderStartPending = false;
    lockSystem("Y HOMING FAILED");
}

void Robot::onBonderModuleStateChanged(bool isIdle)
{
    if (!isIdle) return;

    if (m_forceSetupActive) {
        m_forceSetupActive = false;
        m_ledSetup.off();
        const BonderConfig& config = m_userInterface.activeConfiguration();
        m_bonder.configure(config);
        m_bonder.setProtocol(protocolForMode(config.bondingMode));
        m_bonderConfigUpdated = false;
    } else if (m_ultrasonicTestActive) {
        m_ultrasonicTestActive = false;
        m_ledTest.off();
        const BonderConfig& config = m_userInterface.activeConfiguration();
        m_bonder.configure(config);
        m_bonder.setProtocol(protocolForMode(config.bondingMode));
        m_bonderConfigUpdated = false;
    }

    // A finished (or aborted-by-error) cycle must leave the machine armed for
    // the next one; the guards inside reject locked, unconfirmed or
    // interlocked states, so intentional stops stay stopped.
    requestBonderStartIfReady();
}

void Robot::onBonderModuleErrorOccurred(BonderModule::Error error)
{
    switch (error) {
    case BonderModule::Error::InsufficientBondingPower:
        // The bond cycle already aborted, but the machine itself is intact:
        // a process-quality problem, not a machine fault.
        m_userInterface.warnUser(m_ultrasonicTestActive
            ? "US TEST FAILED"
            : "LOW BONDING POWER");
        break;
    case BonderModule::Error::UnableToSetForceCoilCurrent:
        // Actuator control faults may leave the mechanics in an unknown
        // state; latch the machine until reset.
        lockSystem("FORCE COIL CURRENT");
        return;
    case BonderModule::Error::UnableToSetPosition:
        lockSystem("Z AXIS POSITION");
        return;
    case BonderModule::Error::ProtocolTimeout:
        lockSystem("BONDING TIMEOUT");
        return;
    }

    if (!m_ultrasonicTestActive) return;

    m_ultrasonicTestActive = false;
    m_ledTest.off();
    const BonderConfig& config = m_userInterface.activeConfiguration();
    m_bonder.configure(config);
    m_bonder.setProtocol(protocolForMode(config.bondingMode));
}

void Robot::onUltrasonicReport(
    void *context, const BonderModule::UltrasonicReport& report)
{
    Robot *robot = static_cast<Robot *>(context);
    if (robot == nullptr) return;

    robot->m_userInterface.ultrasonicInfo(
        report.resonanceFrequency,
        report.qualityFactor,
        report.transferredPower,
        report.bondingDuration);
}

void Robot::onUserInterfaceEvent(void *ctx, UserInterfaceModule::Event event)
{
    Robot *robot = static_cast<Robot *>(ctx);
    if (robot == nullptr) return;

    // Configuration navigation must not stop or replace a force setup that is
    // already controlling the Z axis and force coil.
    if (m_forceSetupActive) return;

    switch (event) {
    case UserInterfaceModule::Event::ActiveConfigurationChanged:
        m_bonderConfigUpdated = true;
        break;

    case UserInterfaceModule::Event::ConfigurationConfirmed:
        m_configurationConfirmed = true;
        robot->requestBonderStartIfReady();
        break;

    case UserInterfaceModule::Event::ConfigurationSelectionStarted: {
        m_configurationConfirmed = false;
        robot->m_bonderStartPending = false;
        // Stopping an engaged bonder emergency-closes the clamp; drop a
        // latched manual-open so the interlock and LED keep matching the
        // physical clamp.
        const bool wasEngaged = m_bonder.isActive();
        m_bonder.stop();
        if (wasEngaged && m_manualClampOpen) {
            m_manualClampOpen = false;
            m_manualClampCommandActive = false;
            m_ledClampOpen.off();
        }
        break;
    }
    }
}

void Robot::onMouseButtonEvent(void *ctx,
                               UserInterfaceModule::MouseButtonEvent event)
{
    (void)ctx;
    if (m_systemLocked) return;

    // The bonder stays engaged through the clamp and test interlocks; the
    // operator's controls are blocked here instead, with a notice on the
    // display. Releases still pass so a held jog can never stick on.
    const char *blockedBy = nullptr;
    if (m_manualClampOpen) {
        blockedBy = "CLOSE CLAMP FIRST";
    } else if (m_ultrasonicTestActive) {
        blockedBy = "TEST IN PROGRESS";
    }
    if (blockedBy != nullptr) {
        if (event == UserInterfaceModule::MouseButtonEvent::RightReleased) {
            m_bonder.notifyRightButton(false);
        } else if (event ==
                   UserInterfaceModule::MouseButtonEvent::LeftReleased) {
            m_bonder.notifyLeftButton(false);
        } else {
            m_userInterface.notifyUser(blockedBy);
        }
        return;
    }

    switch (event) {
    case UserInterfaceModule::MouseButtonEvent::RightPressed:
        m_bonder.notifyRightButton(true);
        break;
    case UserInterfaceModule::MouseButtonEvent::RightReleased:
        m_bonder.notifyRightButton(false);
        break;
    case UserInterfaceModule::MouseButtonEvent::LeftPressed:
        m_bonder.notifyLeftButton(true);
        break;
    case UserInterfaceModule::MouseButtonEvent::LeftReleased:
        m_bonder.notifyLeftButton(false);
        break;
    }
}

void Robot::onControlPanelButtonEvent(
    void *ctx,
    UserInterfaceModule::ControlPanelButtonEvent event)
{
    Robot *robot = static_cast<Robot *>(ctx);
    if (robot == nullptr) return;

    // Reset remains an emergency exit and the area light is independent of
    // bonder motion. Other control-panel commands cannot interrupt setup.
    if (m_forceSetupActive &&
        event != UserInterfaceModule::ControlPanelButtonEvent::ResetPressed &&
        event != UserInterfaceModule::ControlPanelButtonEvent::LightPressed) {
        return;
    }

    switch (event) {
    case UserInterfaceModule::ControlPanelButtonEvent::SetupPressed:
        robot->onSetupButtonPressed();
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::TestPressed:
        robot->onTestButtonPressed();
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::ResetPressed:
        robot->onResetButtonPressed();
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::ClampOpenPressed:
        robot->onClampOpenButtonPressed();
        break;

    case UserInterfaceModule::ControlPanelButtonEvent::LightPressed:
        robot->onLightButtonPressed();
        break;
    }
}

bool Robot::retaskBonder(const BonderProtocol& protocol)
{
    if (!m_bonder.setProtocol(protocol)) return false;
    // A force setup still parked at its own start gate is superseded by the
    // retask; natural completion cleans up via the idle callback instead.
    if (m_forceSetupActive) {
        m_forceSetupActive = false;
        m_ledSetup.off();
    }
    return true;
}

void Robot::onSetupButtonPressed()
{
    if (m_systemLocked || m_forceSetupActive) return;
    if (m_manualClampOpen) {
        m_userInterface.notifyUser("CLOSE CLAMP FIRST");
        return;
    }
    if (m_ultrasonicTestActive) {
        m_userInterface.notifyUser("TEST IN PROGRESS");
        return;
    }
    if (!m_bonder.isOperating()) {
        m_userInterface.notifyUser("SETUP NOT READY");
        return;
    }
    if (!retaskBonder(forceSetupProtocol())) {
        m_userInterface.notifyUser("BONDER ACTIVE");
        return;
    }

    m_bonderStartPending = false;
    m_bonder.configure(m_userInterface.activeConfiguration());
    m_forceSetupActive = true;

    if (m_bonder.isIdle() && !m_bonder.engage()) {
        m_forceSetupActive = false;
        restoreConfiguredBondingProtocol();
        m_userInterface.notifyUser("SETUP START FAILED");
        return;
    }

    m_ledSetup.on();
    m_userInterface.notifyUser("FORCE SETUP");
}

void Robot::onTestButtonPressed()
{
    if (m_systemLocked) return;
    if (m_ultrasonicTestActive) {
        m_userInterface.notifyUser("TEST IN PROGRESS");
        return;
    }
    if (!m_bonder.isOperating()) {
        m_userInterface.notifyUser("TEST NOT READY");
        return;
    }
    if (!retaskBonder(ultrasonicTestProtocol())) {
        m_userInterface.notifyUser("BONDER ACTIVE");
        return;
    }

    m_bonderStartPending = false;
    m_bonder.configure(m_userInterface.activeConfiguration());
    m_ultrasonicTestActive = true;

    if (m_bonder.isIdle() && !m_bonder.engage()) {
        m_ultrasonicTestActive = false;
        restoreConfiguredBondingProtocol();
        m_userInterface.notifyUser("TEST START FAILED");
        return;
    }

    m_ledTest.on();
    m_userInterface.notifyUser("US TEST RUNNING");
}

void Robot::onResetButtonPressed()
{
    NVIC_SystemReset();
}

void Robot::onClampOpenButtonPressed()
{
    if (m_systemLocked) return;
    // UltrasonicTestProtocol declares requiresMotionControl() == false and
    // never touches the clamp opcode/Z-motor path, so it has no physical
    // interaction with the clamp; exempt it from the generic "bonder busy"
    // guard below instead of blocking the two unconditionally.
    if (m_bonder.isActive() && !m_bonder.isAwaitingStartTrigger() &&
        !m_ultrasonicTestActive) {
        m_userInterface.notifyUser("BONDER ACTIVE");
        return;
    }

    // The bonder stays engaged at its gate; while the clamp is open the
    // operator's bonding controls are blocked instead (onMouseButtonEvent).
    m_manualClampOpen = !m_manualClampOpen;
    m_manualClampCommandActive = true;
    m_ledClampOpen.on();
    if (m_manualClampOpen) {
        m_clampSolenoidChannel.energize();
    } else {
        m_clampSolenoidChannel.deenergize();
        // Closing the clamp releases the bonding interlock; re-arm the
        // bonder so the next trigger is accepted.
        requestBonderStartIfReady();
    }
}

void Robot::onLightButtonPressed()
{
    m_areaLightOn = !m_areaLightOn;
    const float rawDuty = m_areaLightOn
        ? (1.0f - AREA_LIGHT_PWM_DUTY_RATIO)
        : 1.0f;
    m_areaLightPwmChannel.setDuty(rawDuty);
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_NORMAL
