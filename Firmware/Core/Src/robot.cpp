#include "robot.hpp"
#include "robot.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern DAC_HandleTypeDef hdac;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

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

uint32_t Robot::m_tim1pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
uint32_t Robot::m_tim1pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

// -----------------------------------------------------------------------------
// GPIO  —  bare pin wrappers
// -----------------------------------------------------------------------------
FastIO Robot::m_stepperEnablePin(STEPPER_EN_GPIO_Port,            STEPPER_EN_Pin,            FALSE);
FastIO Robot::m_stepperResetPin (STEPPER_RESET_GPIO_Port,         STEPPER_RESET_Pin,         FALSE);

FastIO Robot::m_yAxisStepPin(STEPPER_Y_STEP_GPIO_Port,    STEPPER_Y_STEP_Pin,    FALSE);
FastIO Robot::m_yAxisDirPin (STEPPER_Y_DIR_GPIO_Port,     STEPPER_Y_DIR_Pin,     FALSE);
FastIO Robot::m_tAxisStepPin(STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin, FALSE);
FastIO Robot::m_tAxisDirPin (STEPPER_TEAR_DIR_GPIO_Port,  STEPPER_TEAR_DIR_Pin,  FALSE);

FastIO Robot::m_clampLowPin (DRIVES_SOL1L_GPIO_Port, DRIVES_SOL1L_Pin, FALSE);
FastIO Robot::m_clampHighPin(DRIVES_SOL1H_GPIO_Port, DRIVES_SOL1H_Pin, FALSE);
FastIO Robot::m_sol2LowPin  (DRIVES_SOL2L_GPIO_Port, DRIVES_SOL2L_Pin, FALSE);
FastIO Robot::m_sol2HighPin (DRIVES_SOL2H_GPIO_Port, DRIVES_SOL2H_Pin, FALSE);
FastIO Robot::m_sol3LowPin  (DRIVES_SOL3L_GPIO_Port, DRIVES_SOL3L_Pin, FALSE);
FastIO Robot::m_sol3HighPin (DRIVES_SOL3H_GPIO_Port, DRIVES_SOL3H_Pin, FALSE);

FastIO Robot::m_contactSensorPin    (CONTACT_SENSORS_TIP_GPIO_Port,         CONTACT_SENSORS_TIP_Pin,         FALSE);
FastIO Robot::m_mouseRightButtonPin (CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port, CONTACT_SENSORS_MOUSE_RIGHT_Pin, FALSE);
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
    ZMOTOR_MODULE_TACHOMETER_V_TO_RPM,
    -ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE * ZMOTOR_MODULE_TACHOMETER_V_TO_RPM);

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
    0.0f);

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

// -----------------------------------------------------------------------------
// Timers
// -----------------------------------------------------------------------------
TimerExpireService Robot::m_timerExpireService;

Timer Robot::m_clampSolenoidTimer;
Timer Robot::m_sol2SolenoidTimer;
Timer Robot::m_sol3SolenoidTimer;
Timer Robot::m_bonderTimer;
Timer Robot::m_zMotorSettlingTimer;
Timer Robot::m_pinMonitorCriticalTimer;
Timer Robot::m_pinMonitorNormalTimer;
Timer Robot::m_controlPanelPollTimer;
Timer Robot::m_lcdDelayTimer;

// -----------------------------------------------------------------------------
// Peripheral services  —  ADC / DAC / PWM / stepper bus schedulers
// -----------------------------------------------------------------------------
AdcService Robot::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE,
    Robot::m_adc1Buffer, 2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

AdcService Robot::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE,
    Robot::m_adc2Buffer, 2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

DacService     Robot::m_dacService(&hdac);
PwmService     Robot::m_tim1PwmService;
StepperService Robot::m_stepperService(&htim6, &Robot::m_stepperEnablePin, &Robot::m_stepperResetPin);

// -----------------------------------------------------------------------------
// Input monitors  —  pin monitor service + panel buttons
// -----------------------------------------------------------------------------
PinMonitorService Robot::m_pinMonitorService(&Robot::m_pinMonitorCriticalTimer, &Robot::m_pinMonitorNormalTimer);
PinMonitorChannel Robot::m_contactSensorChannel   (&Robot::m_contactSensorPin);
PinMonitorChannel Robot::m_mouseRightButtonChannel(&Robot::m_mouseRightButtonPin);
PinMonitorChannel Robot::m_yAxisLimitSwitchChannel(&Robot::m_yAxisLimitSwitchPin);

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

// -----------------------------------------------------------------------------
// Output actuators
// -----------------------------------------------------------------------------
SolenoidChannel Robot::m_clampSolenoidChannel(
    &Robot::m_clampHighPin,
    &Robot::m_clampLowPin,
    SolenoidChannel::DefaultState::CLOSED,
    &Robot::m_clampSolenoidTimer);

SolenoidChannel Robot::m_sol2SolenoidChannel(
    &Robot::m_sol2HighPin,
    &Robot::m_sol2LowPin,
    SolenoidChannel::DefaultState::CLOSED,
    &Robot::m_sol2SolenoidTimer);

SolenoidChannel Robot::m_sol3SolenoidChannel(
    &Robot::m_sol3HighPin,
    &Robot::m_sol3LowPin,
    SolenoidChannel::DefaultState::CLOSED,
    &Robot::m_sol3SolenoidTimer);

SolenoidService Robot::m_solenoidService;

// -----------------------------------------------------------------------------
// Motion  —  stepper channels and router
// -----------------------------------------------------------------------------
StepperChannel Robot::m_yAxisStepperChannel(&Robot::m_yAxisStepPin, &Robot::m_yAxisDirPin);
StepperChannel Robot::m_tAxisStepperChannel(&Robot::m_tAxisStepPin, &Robot::m_tAxisDirPin);

RouterChannel Robot::m_yAxisRouterChannel(
    &Robot::m_yAxisStepperChannel,
    ROBOT_Y_AXIS_MAX_VELOCITY,
    ROBOT_Y_AXIS_MAX_ACCELERATION);

RouterChannel Robot::m_tAxisRouterChannel(
    &Robot::m_tAxisStepperChannel,
    ROBOT_T_AXIS_MAX_VELOCITY,
    ROBOT_T_AXIS_MAX_ACCELERATION);

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

DcRouterModule Robot::m_zMotorRouterModule(
    &Robot::m_zMotorPositionControllerModule,
    &Robot::m_zMotorSettlingTimer,
    ROBOT_ZMOTOR_MAX_VELOCITY,
    ROBOT_ZMOTOR_MAX_ACCELERATION);

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
LcdModule Robot::m_lcd(&Robot::m_lcdExpanderChannel, &Robot::m_lcdDelayTimer);

// -----------------------------------------------------------------------------
// Bonder  —  top-level bonding state machine
// -----------------------------------------------------------------------------
BonderModule Robot::m_bonder(
    &Robot::m_zMotorRouterModule,
    &Robot::m_forceCoilControllerModule,
    &Robot::m_yAxisRouterChannel,
    &Robot::m_tAxisRouterChannel,
    &Robot::m_pllModule,
    &Robot::m_impedanceScannerModule,
    &Robot::m_clampSolenoidChannel,
    &Robot::m_contactSensorChannel,
    &Robot::m_mouseRightButtonChannel,
    &Robot::m_bonderTimer);

// -----------------------------------------------------------------------------
// Persistence  —  EEPROM emulator and live bonding configuration
// -----------------------------------------------------------------------------
EepromEmulator Robot::m_eepromEmulator;
BonderConfig   Robot::m_bonderConfig;

// -----------------------------------------------------------------------------
// UI  —  LCD parameter editor
// -----------------------------------------------------------------------------
UiModule Robot::m_ui(
    &Robot::m_lcd,
    UiModule::Buttons{
        &Robot::m_btnUp,           &Robot::m_btnDown,
        &Robot::m_btnLeft,         &Robot::m_btnRight,
        &Robot::m_btnPlus,         &Robot::m_btnMinus,
        &Robot::m_btnSave,         &Robot::m_btnLoad,
        &Robot::m_btnTailPlus,     &Robot::m_btnTailMinus,
        &Robot::m_btnLoopPlus,     &Robot::m_btnLoopMinus,
        &Robot::m_btnSearchPlus,   &Robot::m_btnSearchMinus,
        &Robot::m_btnStepPlus,     &Robot::m_btnStepMinus,
        &Robot::m_btnReset,        &Robot::m_btnEnter
    });

#if DEBUG_ENABLED
DebugService                 Robot::m_debugService;
DebugImpedanceScanner        Robot::m_debugChannelImpedanceScanner;
DebugKeypad                  Robot::m_debugChannelKeypad;
DebugToneGenerator           Robot::m_debugChannelToneGenerator;
DebugPll                     Robot::m_debugChannelPll;
DebugMotorVelocityController Robot::m_debugChannelMotorVelocityController;
DebugForceCoil               Robot::m_debugChannelForceCoil;
#endif

// =============================================================================
// Constructor
// =============================================================================
Robot::Robot()
{
    // Timers.
    m_timerExpireService.addTimer(&m_clampSolenoidTimer,      false);
    m_timerExpireService.addTimer(&m_sol2SolenoidTimer,       false);
    m_timerExpireService.addTimer(&m_sol3SolenoidTimer,       false);
    m_timerExpireService.addTimer(&m_bonderTimer,             false);
    m_timerExpireService.addTimer(&m_zMotorSettlingTimer,     false);
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
    m_pinMonitorService.addChannel(&m_mouseRightButtonChannel, false);
    m_yAxisLimitSwitchChannel.addTransitionListenerCallback(nullptr, &Robot::onYAxisLimitSwitchTransition);

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

    // Solenoid channels.
    m_solenoidService.addChannel(&m_clampSolenoidChannel);
    m_solenoidService.addChannel(&m_sol2SolenoidChannel);
    m_solenoidService.addChannel(&m_sol3SolenoidChannel);

    // Router channels.
    m_routerService.addChannel(&m_yAxisRouterChannel);
    m_routerService.addChannel(&m_tAxisRouterChannel);

    // Bonder.
    m_bonder.addEventListenerCallbacks(&Robot::onBonderModuleStateChanged, &Robot::onBonderModuleErrorOccurred);
    m_bonder.init();

    // EEPROM: register bonder config buffer.
    m_eepromEmulator.registerObject(ROBOT_BONDER_CONFIG_OBJECT_ID, reinterpret_cast<uint8_t *>(&m_bonderConfig),
        static_cast<uint16_t>(sizeof(m_bonderConfig)), &Robot::onBonderConfigCompactification, nullptr);

    // UI callbacks.
    m_ui.setConfigChangedListenerCallback(nullptr, &Robot::onConfigChanged);
    m_ui.setPersistenceControllerCallbacks(nullptr, &Robot::onSave, &Robot::onLoad);

    m_eepromEmulator.init();

#if DEBUG_ENABLED
    static ButtonChannel *const kBridgeButtons[] = {
        &m_btnUp,           &m_btnDown,
        &m_btnLeft,         &m_btnRight,
        &m_btnPlus,         &m_btnMinus,
        &m_btnSave,         &m_btnLoad,
        &m_btnTailPlus,     &m_btnTailMinus,
        &m_btnLoopPlus,     &m_btnLoopMinus,
        &m_btnSearchPlus,   &m_btnSearchMinus,
        &m_btnStepPlus,     &m_btnStepMinus,
        &m_btnReset,        &m_btnEnter,
        &m_btnManual,       &m_btnEscDel,
        &m_btnAdd,          &m_btnTest,
        &m_btnSetup,        &m_btnLight,
        &m_btnClampOpen,    &m_btnHighReset
    };

    m_debugChannelToneGenerator.init(&m_ultrasonicDacChannel,
        &m_ultrasonicVsensChannel,
        &m_ultrasonicIsensChannel);

    m_debugChannelImpedanceScanner.init(&m_impedanceScannerModule);

    m_debugChannelKeypad.init(kBridgeButtons, sizeof(kBridgeButtons) / sizeof(kBridgeButtons[0]));
    m_debugChannelPll.init(&m_pllModule);
    m_debugChannelMotorVelocityController.init(&m_zMotorVelocityControllerModule);
    m_debugChannelForceCoil.init(&m_forceCoilControllerModule);

    m_debugChannelImpedanceScanner.setDependencyCallback(
        this,
        &Robot::startImpedanceScannerDebugDependencies);
    m_debugChannelPll.setDependencyCallback(
        this,
        &Robot::startPllDebugDependencies);
    m_debugChannelToneGenerator.setDependencyCallback(
        this,
        &Robot::startToneGeneratorDebugDependencies);
    m_debugChannelKeypad.setDependencyCallback(
        this,
        &Robot::startKeypadDebugDependencies);
    m_debugChannelMotorVelocityController.setDependencyCallback(
        this,
        &Robot::startMotorVelocityDebugDependencies);
    m_debugChannelForceCoil.setDependencyCallback(
        this,
        &Robot::startForceCoilDebugDependencies);

    m_debugChannelImpedanceScanner.setDependencyReleaseCallback(
        this,
        &Robot::stopImpedanceScannerDebugDependencies);
    m_debugChannelPll.setDependencyReleaseCallback(
        this,
        &Robot::stopPllDebugDependencies);
    m_debugChannelToneGenerator.setDependencyReleaseCallback(
        this,
        &Robot::stopToneGeneratorDebugDependencies);
    m_debugChannelKeypad.setDependencyReleaseCallback(
        this,
        &Robot::stopKeypadDebugDependencies);
    m_debugChannelMotorVelocityController.setDependencyReleaseCallback(
        this,
        &Robot::stopMotorVelocityDebugDependencies);
    m_debugChannelForceCoil.setDependencyReleaseCallback(
        this,
        &Robot::stopForceCoilDebugDependencies);

    // Register every channel with the dispatcher so the service block's
    // command word is routed by channel id (see debug_service.hpp).
    m_debugService.addChannel(&m_debugChannelImpedanceScanner);
    m_debugService.addChannel(&m_debugChannelPll);
    m_debugService.addChannel(&m_debugChannelToneGenerator);
    m_debugService.addChannel(&m_debugChannelKeypad);
    m_debugService.addChannel(&m_debugChannelMotorVelocityController);
    m_debugService.addChannel(&m_debugChannelForceCoil);
#endif
}

#if DEBUG_ENABLED
bool Robot::startImpedanceScannerDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugImpedanceScanner::CMD_SCAN) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startDacService = true;
    robot->m_startAdc1Service = true;
    return true;
}

bool Robot::startPllDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugPll::CMD_START) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startDacService = true;
    robot->m_startAdc1Service = true;
    return true;
}

bool Robot::startToneGeneratorDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugToneGenerator::CMD_START) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startDacService = true;
    robot->m_startAdc1Service = true;
    return true;
}

bool Robot::startKeypadDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugKeypad::CMD_LISTEN) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startIoExpanderService = true;
    robot->m_startTimerExpireService = true;
    robot->m_startControlPanelService = true;
    return true;
}

bool Robot::startMotorVelocityDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugMotorVelocityController::CMD_START) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startTim1PwmService = true;
    robot->m_startAdc2Service = true;
    return true;
}

bool Robot::startForceCoilDebugDependencies(void *context, uint16_t localCommand)
{
    if (localCommand != DebugForceCoil::CMD_START) {
        return false;
    }

    Robot *robot = static_cast<Robot *>(context);
    robot->m_startTim1PwmService = true;
    robot->m_startAdc2Service = true;
    return true;
}

void Robot::stopImpedanceScannerDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopDacService = true;
    robot->m_stopAdc1Service = true;
}

void Robot::stopPllDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopDacService = true;
    robot->m_stopAdc1Service = true;
}

void Robot::stopToneGeneratorDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopDacService = true;
    robot->m_stopAdc1Service = true;
}

void Robot::stopKeypadDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopControlPanelService = true;
    robot->m_stopIoExpanderService = true;
    robot->m_stopTimerExpireService = true;
}

void Robot::stopMotorVelocityDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopTim1PwmService = true;
    robot->m_stopAdc2Service = true;
}

void Robot::stopForceCoilDebugDependencies(void *context, uint16_t localCommand)
{
    (void)localCommand;

    Robot *robot = static_cast<Robot *>(context);
    robot->m_stopTim1PwmService = true;
    robot->m_stopAdc2Service = true;
}
#endif

void Robot::start()
{
#if DEBUG_ENABLED
    m_startDebugService = true;
    m_startIoExpanderService = false;
    m_startTimerExpireService = false;
    m_startPinMonitorService = false;
    m_startSolenoidService = false;
    m_startStepperService = false;
    m_startRouterService = false;
    m_startTim1PwmService = false;
    m_startDacService = false;
    m_startAdc1Service = false;
    m_startAdc2Service = false;
    m_startForceCoilControllerModule = false;
    m_startZmotorVelocityControllerModule = false;
    m_startZmotorPositionControllerModule = false;
    m_startZMotorRouterModule = false;
    m_startBonderModule = false;
    m_startControlPanelService = false;
    m_startLcdModule = false;
    m_startUiModule = false;
    m_stopDebugService = false;
    m_stopIoExpanderService = false;
    m_stopTimerExpireService = false;
    m_stopPinMonitorService = false;
    m_stopSolenoidService = false;
    m_stopStepperService = false;
    m_stopRouterService = false;
    m_stopTim1PwmService = false;
    m_stopDacService = false;
    m_stopAdc1Service = false;
    m_stopAdc2Service = false;
    m_stopForceCoilControllerModule = false;
    m_stopZmotorVelocityControllerModule = false;
    m_stopZmotorPositionControllerModule = false;
    m_stopZMotorRouterModule = false;
    m_stopBonderModule = false;
    m_stopControlPanelService = false;
    m_stopLcdModule = false;
    m_stopUiModule = false;
#else
    m_startDebugService = false;
    m_startIoExpanderService = true;
    m_startTimerExpireService = true;
    m_startPinMonitorService = true;
    m_startSolenoidService = true;
    m_startStepperService = true;
    m_startRouterService = true;
    m_startTim1PwmService = true;
    m_startDacService = true;
    m_startAdc1Service = true;
    m_startAdc2Service = true;
    m_startForceCoilControllerModule = true;
    m_startZmotorVelocityControllerModule = true;
    m_startZmotorPositionControllerModule = true;
    m_startZMotorRouterModule = true;
    m_startBonderModule = true;
    m_startControlPanelService = true;
    m_startLcdModule = true;
    m_startUiModule = true;
    m_stopDebugService = false;
    m_stopIoExpanderService = false;
    m_stopTimerExpireService = false;
    m_stopPinMonitorService = false;
    m_stopSolenoidService = false;
    m_stopStepperService = false;
    m_stopRouterService = false;
    m_stopTim1PwmService = false;
    m_stopDacService = false;
    m_stopAdc1Service = false;
    m_stopAdc2Service = false;
    m_stopForceCoilControllerModule = false;
    m_stopZmotorVelocityControllerModule = false;
    m_stopZmotorPositionControllerModule = false;
    m_stopZMotorRouterModule = false;
    m_stopBonderModule = false;
    m_stopControlPanelService = false;
    m_stopLcdModule = false;
    m_stopUiModule = false;
#endif
}

void Robot::execute()
{
    if (m_stopDebugService) {
#if DEBUG_ENABLED
        m_debugService.stopService();
#endif
        m_stopDebugService = false;
    }

    if (m_stopBonderModule) {
        m_bonder.stop();
        m_stopBonderModule = false;
    }

    if (m_stopZMotorRouterModule) {
        m_zMotorRouterModule.stop();
        m_stopZMotorRouterModule = false;
    }

    if (m_stopZmotorPositionControllerModule) {
        m_zMotorPositionControllerModule.stop();
        m_stopZmotorPositionControllerModule = false;
    }

    if (m_stopZmotorVelocityControllerModule) {
        m_zMotorVelocityControllerModule.stop();
        m_stopZmotorVelocityControllerModule = false;
    }

    if (m_stopForceCoilControllerModule) {
        m_forceCoilControllerModule.stop();
        m_stopForceCoilControllerModule = false;
    }

    if (m_stopUiModule) {
        m_ui.stopService();
        m_stopUiModule = false;
    }

    if (m_stopLcdModule) {
        m_lcd.stop();
        m_stopLcdModule = false;
    }

    if (m_stopControlPanelService) {
        m_controlPanelService.stopService();
        m_stopControlPanelService = false;
    }

    if (m_stopRouterService) {
        m_routerService.stopService();
        m_stopRouterService = false;
    }

    if (m_stopStepperService) {
        m_stepperService.stopService();
        m_stopStepperService = false;
    }

    if (m_stopSolenoidService) {
        m_solenoidService.stopService();
        m_stopSolenoidService = false;
    }

    if (m_stopPinMonitorService) {
        m_pinMonitorService.stopService();
        m_stopPinMonitorService = false;
    }

    if (m_stopAdc1Service) {
        m_adc1Service.stopService();
        m_stopAdc1Service = false;
    }

    if (m_stopAdc2Service) {
        m_adc2Service.stopService();
        m_stopAdc2Service = false;
    }

    if (m_stopDacService) {
        m_dacService.stopService();
        m_stopDacService = false;
    }

    if (m_stopTim1PwmService) {
        m_tim1PwmService.stopService();
        m_stopTim1PwmService = false;
    }

    if (m_stopIoExpanderService) {
        m_ioExpanderService.stopService();
        m_stopIoExpanderService = false;
    }

    if (m_stopTimerExpireService) {
        m_timerExpireService.stopService();
        m_stopTimerExpireService = false;
    }

#if DEBUG_ENABLED
    if (m_startDebugService) {
        m_debugService.startService();
        m_startDebugService = false;
    }
#endif

    if (m_startIoExpanderService) {
        m_ioExpanderService.startService();
        m_startIoExpanderService = false;
    }

    if (m_startTimerExpireService) {
        m_timerExpireService.startService();
        m_startTimerExpireService = false;
    }

    if (m_startPinMonitorService) {
        m_pinMonitorService.startService();
        m_startPinMonitorService = false;
    }

    if (m_startSolenoidService) {
        m_solenoidService.startService();
        m_startSolenoidService = false;
    }

    if (m_startStepperService) {
        m_stepperService.startService();
        m_startStepperService = false;
    }

    if (m_startRouterService) {
        m_routerService.startService();
        m_startRouterService = false;
    }

    if (m_startTim1PwmService) {
        m_tim1PwmService.startService();
        m_startTim1PwmService = false;
    }

    if (m_startDacService) {
        m_dacService.startService();
        m_startDacService = false;
    }

    if (m_startAdc1Service) {
        m_adc1Service.startService();
        m_startAdc1Service = false;
    }

    if (m_startAdc2Service) {
        m_adc2Service.startService();
        m_startAdc2Service = false;
    }

    if (m_startForceCoilControllerModule) {
        m_forceCoilControllerModule.start();
        m_startForceCoilControllerModule = false;
    }

    if (m_startZmotorVelocityControllerModule) {
        m_zMotorVelocityControllerModule.start();
        m_startZmotorVelocityControllerModule = false;
    }

    if (m_startZmotorPositionControllerModule) {
        m_zMotorPositionControllerModule.start();
        m_startZmotorPositionControllerModule = false;
    }

    if (m_startZMotorRouterModule) {
        m_zMotorRouterModule.start(0.0f);
        m_startZMotorRouterModule = false;
    }

    if (m_startBonderModule) {
        m_eepromEmulator.loadObject(ROBOT_BONDER_CONFIG_OBJECT_ID);
        m_bonder.configure(m_bonderConfig);
        m_bonder.start();
        m_startBonderModule = false;
    }

    if (m_startControlPanelService) {
        m_controlPanelService.startService();
        m_startControlPanelService = false;
    }

    if (m_startLcdModule) {
        m_lcd.start();
        m_startLcdModule = false;
    }

    if (m_startUiModule) {
        m_ui.startService();
        m_startUiModule = false;
    }

#if DEBUG_ENABLED
    m_debugService.executeService();
#endif

    m_timerExpireService.executeService();
    m_pinMonitorService.executeService();

    m_ioExpanderService.executeService();
    m_controlPanelService.executeService();
    m_lcd.execute();
    m_ui.execute();

    m_routerService.executeService();
    m_bonder.execute();

    if (m_bonderConfigUpdated && m_bonder.isIdle()) {
        m_bonder.configure(m_bonderConfig);
        m_bonderConfigUpdated = false;
    }   
}

void Robot::stop()
{
    m_startDebugService = false;
    m_startIoExpanderService = false;
    m_startTimerExpireService = false;
    m_startPinMonitorService = false;
    m_startSolenoidService = false;
    m_startStepperService = false;
    m_startRouterService = false;
    m_startTim1PwmService = false;
    m_startDacService = false;
    m_startAdc1Service = false;
    m_startAdc2Service = false;
    m_startForceCoilControllerModule = false;
    m_startZmotorVelocityControllerModule = false;
    m_startZmotorPositionControllerModule = false;
    m_startZMotorRouterModule = false;
    m_startBonderModule = false;
    m_startControlPanelService = false;
    m_startLcdModule = false;
    m_startUiModule = false;

#if DEBUG_ENABLED
    m_stopDebugService = true;
    m_stopIoExpanderService = true;
    m_stopTimerExpireService = true;
    m_stopTim1PwmService = true;
    m_stopDacService = true;
    m_stopAdc1Service = true;
    m_stopAdc2Service = true;
    m_stopForceCoilControllerModule = true;
    m_stopZmotorVelocityControllerModule = true;
    m_stopControlPanelService = true;
#else
    m_stopDebugService = false;
    m_stopIoExpanderService = true;
    m_stopTimerExpireService = true;
    m_stopPinMonitorService = true;
    m_stopSolenoidService = true;
    m_stopStepperService = true;
    m_stopRouterService = true;
    m_stopTim1PwmService = true;
    m_stopDacService = true;
    m_stopAdc1Service = true;
    m_stopAdc2Service = true;
    m_stopForceCoilControllerModule = true;
    m_stopZmotorVelocityControllerModule = true;
    m_stopZmotorPositionControllerModule = true;
    m_stopZMotorRouterModule = true;
    m_stopBonderModule = true;
    m_stopControlPanelService = true;
    m_stopLcdModule = true;
    m_stopUiModule = true;
#endif
}

void Robot::onYAxisLimitSwitchTransition(
    void *context,
    PinMonitorChannel::Transition transition)
{
    (void)context;
    (void)transition;

    // TODO: homing logic.
}

void Robot::onBonderModuleStateChanged(bool isIdle)
{
    (void)isIdle;
}

void Robot::onBonderModuleErrorOccurred(BonderModule::Error error)
{
    (void)error;
}

void Robot::onConfigChanged(void *ctx, const BonderConfig& config)
{
    (void)ctx;
    m_bonderConfig = config;
    m_bonderConfigUpdated = true;
}

void Robot::onSave(void *ctx, const BonderConfig& config)
{
    (void)ctx;
    m_bonderConfig = config;
    m_eepromEmulator.saveObject(ROBOT_BONDER_CONFIG_OBJECT_ID);
}

void Robot::onLoad(void *ctx, BonderConfig& config)
{
    (void)ctx;
    if (m_eepromEmulator.loadObject(ROBOT_BONDER_CONFIG_OBJECT_ID)) {
        config = m_bonderConfig;
        m_bonderConfigUpdated = true;
    }
}

void Robot::onBonderConfigCompactification(void *ctx)
{
    (void)ctx;
    // m_bonderConfig in RAM is always the authoritative copy; nothing to do before compaction.
}

// -----------------------------------------------------------------------------
// C-linkage entry points, callable from main.c
// -----------------------------------------------------------------------------
extern "C" {

static Robot *g_robot = nullptr;

void Robot_Init(void)
{
    if (g_robot) {
        return;
    }

    static Robot instance;
    g_robot = &instance;
}

void Robot_Start(void)
{
    if (g_robot) {
        g_robot->start();
    }
}

void Robot_Execute(void)
{
    if (g_robot) {
        g_robot->execute();
    }
}

void Robot_Stop(void)
{
    if (g_robot) {
        g_robot->stop();
    }
}

} // extern "C"
