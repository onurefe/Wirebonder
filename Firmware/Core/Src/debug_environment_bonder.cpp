#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_BONDER

#include "debug_environment_bonder.hpp"
#include "main.h"
#include "protocol_semi_auto.hpp"
#include "protocol_manual.hpp"
#include "protocol_table_tear.hpp"
#include "protocol_lange_coupling.hpp"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

// Maps the requested bonding mode to its protocol program; BonderModule
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

// -----------------------------------------------------------------------------
// Static member definitions — the Robot's bonding plant, owned here outright.
// -----------------------------------------------------------------------------

uint16_t BonderDebugEnvironment::m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
uint16_t BonderDebugEnvironment::m_adc2Buffer[2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS];
uint16_t BonderDebugEnvironment::m_dac1Buffer[2 * DAC1_SAMPLES];
uint16_t BonderDebugEnvironment::m_dac2Buffer[2 * DAC2_SAMPLES];
uint16_t BonderDebugEnvironment::m_synthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
uint16_t BonderDebugEnvironment::m_scannerVsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
uint16_t BonderDebugEnvironment::m_scannerIsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
uint16_t BonderDebugEnvironment::m_pwmChannel1Buffer[2 * TIM1_PWM_CHANNEL1_SAMPLES];
uint16_t BonderDebugEnvironment::m_pwmChannel2Buffer[2 * TIM1_PWM_CHANNEL2_SAMPLES];

FastIO BonderDebugEnvironment::m_stepperEnablePin(STEPPER_EN_GPIO_Port,    STEPPER_EN_Pin,    FALSE);
FastIO BonderDebugEnvironment::m_stepperResetPin (STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin, FALSE);
FastIO BonderDebugEnvironment::m_yAxisStepPin(STEPPER_Y_STEP_GPIO_Port,    STEPPER_Y_STEP_Pin,    FALSE);
FastIO BonderDebugEnvironment::m_yAxisDirPin (STEPPER_Y_DIR_GPIO_Port,     STEPPER_Y_DIR_Pin,     FALSE);
FastIO BonderDebugEnvironment::m_tAxisStepPin(STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin, FALSE);
FastIO BonderDebugEnvironment::m_tAxisDirPin (STEPPER_TEAR_DIR_GPIO_Port,  STEPPER_TEAR_DIR_Pin,  FALSE);

// Sol1 and Sol2 drivers are unavailable. The clamp coil is connected to the
// working Sol3 driver, so keep the logical clamp names mapped to Sol3.
FastIO BonderDebugEnvironment::m_clampLowPin (DRIVES_SOL3L_GPIO_Port, DRIVES_SOL3L_Pin, TRUE);
FastIO BonderDebugEnvironment::m_clampHighPin(DRIVES_SOL3H_GPIO_Port, DRIVES_SOL3H_Pin, FALSE);

FastIO BonderDebugEnvironment::m_contactSensorPin    (CONTACT_SENSORS_TIP_GPIO_Port,         CONTACT_SENSORS_TIP_Pin,         FALSE);
FastIO BonderDebugEnvironment::m_mouseRightButtonPin (CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port, CONTACT_SENSORS_MOUSE_RIGHT_Pin, FALSE);
FastIO BonderDebugEnvironment::m_mouseLeftButtonPin  (CONTACT_SENSORS_MOUSE_LEFT_GPIO_Port,  CONTACT_SENSORS_MOUSE_LEFT_Pin,  FALSE);

TimerExpireService BonderDebugEnvironment::m_timerExpireService;
Timer BonderDebugEnvironment::m_clampSolenoidTimer;
Timer BonderDebugEnvironment::m_bonderTimer;
Timer BonderDebugEnvironment::m_pinMonitorCriticalTimer;
Timer BonderDebugEnvironment::m_pinMonitorNormalTimer;

PinMonitorService BonderDebugEnvironment::m_pinMonitorService(
    &BonderDebugEnvironment::m_pinMonitorCriticalTimer,
    &BonderDebugEnvironment::m_pinMonitorNormalTimer);

// All contact-sensor inputs are behind active-low optocoupler front-ends.
PinMonitorChannel BonderDebugEnvironment::m_contactSensorChannel   (&BonderDebugEnvironment::m_contactSensorPin,    PinMonitorChannel::Level::LOW);
PinMonitorChannel BonderDebugEnvironment::m_mouseRightButtonChannel(&BonderDebugEnvironment::m_mouseRightButtonPin, PinMonitorChannel::Level::LOW);
PinMonitorChannel BonderDebugEnvironment::m_mouseLeftButtonChannel (&BonderDebugEnvironment::m_mouseLeftButtonPin,  PinMonitorChannel::Level::LOW);

DirectSolenoidChannel BonderDebugEnvironment::m_clampSolenoidChannel(
    &BonderDebugEnvironment::m_clampHighPin,
    &BonderDebugEnvironment::m_clampLowPin,
    &BonderDebugEnvironment::m_clampSolenoidTimer,
    CLAMP_SOLENOID_ENERGIZE_TIME,
    CLAMP_SOLENOID_DEENERGIZE_TIME);

SolenoidService BonderDebugEnvironment::m_solenoidService;

StepperService BonderDebugEnvironment::m_stepperService(
    &htim6,
    &BonderDebugEnvironment::m_stepperEnablePin,
    &BonderDebugEnvironment::m_stepperResetPin);

StepperChannel BonderDebugEnvironment::m_yAxisStepperChannel(
    &BonderDebugEnvironment::m_yAxisStepPin,
    &BonderDebugEnvironment::m_yAxisDirPin);

StepperChannel BonderDebugEnvironment::m_tAxisStepperChannel(
    &BonderDebugEnvironment::m_tAxisStepPin,
    &BonderDebugEnvironment::m_tAxisDirPin);

RouterChannel BonderDebugEnvironment::m_yAxisRouterChannel(
    &BonderDebugEnvironment::m_yAxisStepperChannel,
    ROBOT_Y_AXIS_MAX_VELOCITY,
    ROBOT_Y_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_Y_AXIS_STEPS_PER_MM);

RouterChannel BonderDebugEnvironment::m_tAxisRouterChannel(
    &BonderDebugEnvironment::m_tAxisStepperChannel,
    ROBOT_T_AXIS_MAX_VELOCITY,
    ROBOT_T_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_T_AXIS_STEPS_PER_MM);

StepperRouterService BonderDebugEnvironment::m_routerService;

IQDemodulatorChannel BonderDebugEnvironment::m_ultrasonicVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_VSENS_GAIN);

IQDemodulatorChannel BonderDebugEnvironment::m_ultrasonicIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_ISENS_GAIN);

RawAdcChannel BonderDebugEnvironment::m_scannerVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    BonderDebugEnvironment::m_scannerVsensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

RawAdcChannel BonderDebugEnvironment::m_scannerIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    BonderDebugEnvironment::m_scannerIsensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

AnalogChannel BonderDebugEnvironment::m_tachometerChannel(
    ADC_CHANNEL_ZMOTOR_TACHOMETER_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_ZMOTOR_TACHOMETER_OVERSAMPLING_RATIO),
    ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC,
    -ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE *
        ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC);

IQDemodulatorChannel BonderDebugEnvironment::m_lvdtAChannel(
    ADC_CHANNEL_LVDT_A_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

IQDemodulatorChannel BonderDebugEnvironment::m_lvdtBChannel(
    ADC_CHANNEL_LVDT_B_CONVERSION_ORDER,
    ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES,
    static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) / static_cast<float>(ADC2_SAMPLING_FREQ),
    1.0f);

AnalogChannel BonderDebugEnvironment::m_forceCoilISensChannel(
    ADC_CHANNEL_FORCE_COIL_ISENS_CONVERSION_ORDER,
    static_cast<uint32_t>(ADC_CHANNEL_FORCE_COIL_ISENS_OVERSAMPLING_RATIO),
    FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR,
    -FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR * FORCE_COIL_MODULE_ZERO_CURRENT_VOLTAGE);

SineGeneratorChannel BonderDebugEnvironment::m_ultrasonicDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    BonderDebugEnvironment::m_dac1Buffer, 2 * DAC1_SAMPLES,
    DAC1_SAMPLES, DAC1_BITS, DAC1_VOLTAGE_RANGE);

RawDacChannel BonderDebugEnvironment::m_scannerDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    BonderDebugEnvironment::m_dac1Buffer, 2 * DAC1_SAMPLES,
    BonderDebugEnvironment::m_synthesisBuffer, SCANNER_SYNTHESIS_BUFFER_SIZE);

SineGeneratorChannel BonderDebugEnvironment::m_lvdtExcitationChannel(
    &hdac, &htim5, DAC_CHANNEL_2,
    BonderDebugEnvironment::m_dac2Buffer, 2 * DAC2_SAMPLES,
    DAC2_SAMPLES, DAC2_BITS, DAC2_VOLTAGE_RANGE);

PwmRampChannel BonderDebugEnvironment::m_forceCoilPwmChannel(
    &htim1, TIM_CHANNEL_1,
    BonderDebugEnvironment::m_pwmChannel1Buffer, 2 * TIM1_PWM_CHANNEL1_SAMPLES,
    TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH1 (PWM) + CH1N (nPWM)

PwmRampChannel BonderDebugEnvironment::m_zMotorPwmChannel(
    &htim1, TIM_CHANNEL_2,
    BonderDebugEnvironment::m_pwmChannel2Buffer, 2 * TIM1_PWM_CHANNEL2_SAMPLES,
    TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES,
    /* complementaryOutput = */ true);   // drives CH2 (PWM) + CH2N (nPWM)

AdcService BonderDebugEnvironment::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE, ADC1_NUM_CONVERSIONS,
    BonderDebugEnvironment::m_adc1Buffer,
    2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

AdcService BonderDebugEnvironment::m_adc2Service(
    &hadc2, &htim3,
    ADC2_BITS, ADC2_VOLTAGE_RANGE, ADC2_NUM_CONVERSIONS,
    BonderDebugEnvironment::m_adc2Buffer,
    2 * ADC2_SAMPLES_PER_CHANNEL * ADC2_NUM_CONVERSIONS);

DacService BonderDebugEnvironment::m_dacService(&hdac);
PwmService BonderDebugEnvironment::m_tim1PwmService;

LvdtSensorModule BonderDebugEnvironment::m_lvdtSensorModule(
    &BonderDebugEnvironment::m_lvdtExcitationChannel,
    &BonderDebugEnvironment::m_lvdtAChannel,
    &BonderDebugEnvironment::m_lvdtBChannel,
    LVDT_MODULE_STROKE_MM);

ForceCoilDriverModule BonderDebugEnvironment::m_forceCoilControllerModule(
    &BonderDebugEnvironment::m_forceCoilISensChannel,
    &BonderDebugEnvironment::m_forceCoilPwmChannel);

DcMotorVelocityControllerModule BonderDebugEnvironment::m_zMotorVelocityControllerModule(
    &BonderDebugEnvironment::m_tachometerChannel,
    &BonderDebugEnvironment::m_zMotorPwmChannel);

DcMotorPositionControllerModule BonderDebugEnvironment::m_zMotorPositionControllerModule(
    &BonderDebugEnvironment::m_lvdtSensorModule,
    &BonderDebugEnvironment::m_zMotorVelocityControllerModule);

PllModule BonderDebugEnvironment::m_pllModule(
    &BonderDebugEnvironment::m_ultrasonicDacChannel,
    &BonderDebugEnvironment::m_ultrasonicVsensChannel,
    &BonderDebugEnvironment::m_ultrasonicIsensChannel,
    static_cast<float>(ADC1_SAMPLING_FREQ),
    static_cast<float>(PLL_MODULE_CONTROL_FREQ));

UsImpedanceScannerModule BonderDebugEnvironment::m_impedanceScannerModule(
    &BonderDebugEnvironment::m_scannerDacChannel,
    &BonderDebugEnvironment::m_scannerVsensChannel,
    &BonderDebugEnvironment::m_scannerIsensChannel,
    BonderDebugEnvironment::m_synthesisBuffer,
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

BonderDebugLog BonderDebugEnvironment::s_stepLog = {};
BonderDebugStatus BonderDebugEnvironment::s_status = {};

bool BonderDebugEnvironment::s_physicalRightButton = false;
bool BonderDebugEnvironment::s_physicalLeftButton = false;
bool BonderDebugEnvironment::s_virtualRightButton = false;
bool BonderDebugEnvironment::s_virtualLeftButton = false;
bool BonderDebugEnvironment::s_combinedRightButton = false;
bool BonderDebugEnvironment::s_combinedLeftButton = false;

BonderModule BonderDebugEnvironment::m_bonder(
    &BonderDebugEnvironment::m_zMotorPositionControllerModule,
    &BonderDebugEnvironment::m_forceCoilControllerModule,
    &BonderDebugEnvironment::m_yAxisRouterChannel,
    &BonderDebugEnvironment::m_tAxisRouterChannel,
    &BonderDebugEnvironment::m_pllModule,
    &BonderDebugEnvironment::m_impedanceScannerModule,
    &BonderDebugEnvironment::m_clampSolenoidChannel,
    &BonderDebugEnvironment::m_contactSensorChannel,
    &BonderDebugEnvironment::m_bonderTimer);

BonderDebugEnvironment::BonderDebugEnvironment()
{
    // Timers.
    m_timerExpireService.addTimer(&m_clampSolenoidTimer,      false);
    m_timerExpireService.addTimer(&m_bonderTimer,             false);
    m_timerExpireService.addTimer(&m_pinMonitorCriticalTimer, true);
    m_timerExpireService.addTimer(&m_pinMonitorNormalTimer,   false);

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
    m_pinMonitorService.addChannel(&m_mouseRightButtonChannel, true,  PIN_MONITOR_BLIND_REGION_MS);
    m_pinMonitorService.addChannel(&m_mouseLeftButtonChannel,  true,  PIN_MONITOR_BLIND_REGION_MS);

    // Mouse-button relay into the bonder.
    m_mouseRightButtonChannel.addStateListenerCallback(
        nullptr, &BonderDebugEnvironment::onMouseRightButtonStateChanged);
    m_mouseLeftButtonChannel.addStateListenerCallback(
        nullptr, &BonderDebugEnvironment::onMouseLeftButtonStateChanged);

    // Solenoid channels.
    m_solenoidService.addChannel(&m_clampSolenoidChannel);

    // Router channels.
    m_routerService.addChannel(&m_yAxisRouterChannel);
    m_routerService.addChannel(&m_tAxisRouterChannel);

    // Lifecycle order mirrors the Robot's component ordering; the bonder
    // stays idle until CMD_START_BONDER calls engage(), then free-runs
    // on its internal dynamics with only operator inputs supplied externally.
    addProcess(&m_timerExpireService);
    addProcess(&m_pinMonitorService);
    addProcess(&m_solenoidService);
    addProcess(&m_stepperService);
    addProcess(&m_routerService);
    addProcess(&m_tim1PwmService);
    addProcess(&m_dacService);
    addProcess(&m_adc1Service);
    addProcess(&m_adc2Service);
    addProcess(&m_forceCoilControllerModule);
    addProcess(&m_zMotorVelocityControllerModule);
    addProcess(&m_lvdtSensorModule);
    addProcess(&m_zMotorPositionControllerModule);
    addProcess(&m_pllModule);
    addProcess(&m_impedanceScannerModule);
    addProcess(&m_bonder);

    const BonderConfig defaults{};
    m_bonder.configure(defaults);
    m_bonder.setProtocol(protocolForMode(defaults.bondingMode));

    m_bonder.setTelemetryListenerCallback(this, &BonderDebugEnvironment::onTelemetry);
}

void BonderDebugEnvironment::onStart()
{
    // Recording is always on; the log ring and impedance-curve arena live for
    // the whole session and reset when a new bonding run starts.
    m_curveBuffer = static_cast<complexf *>(telemetryBuffer());
    m_curveCapacity = DEBUG_TELEMETRY_BUFFER_SIZE_BYTES / sizeof(complexf);
    m_curveCount = 0U;
    resetLog();
    publishStatus();
}

void BonderDebugEnvironment::onPoll()
{
    publishStatus();
}

void BonderDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START_BONDER:
        startBonder();
        break;

    case CMD_STOP_BONDER:
        stopBonder();
        break;

    case CMD_SET_BUTTON:
        setButton();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void BonderDebugEnvironment::abort()
{
    m_bonder.disengage();
}

void BonderDebugEnvironment::startBonder()
{
    // arg(0): BondingMode value; anything out of range falls back to
    // SemiAutomatic. Reconfiguration only lands while the bonder is idle.
    if (m_bonder.isIdle()) {
        BonderConfig config{};

        const uint32_t requestedMode = static_cast<uint32_t>(arg(0));
        if (requestedMode <= static_cast<uint32_t>(BondingMode::LangeCoupling)) {
            config.bondingMode = static_cast<BondingMode>(requestedMode);
        }

        m_bonder.configure(config);
        m_bonder.setProtocol(protocolForMode(config.bondingMode));
    }

    resetLog();
    m_curveCount = 0U;

    if (!m_bonder.engage()) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

    setDone(ERROR_NONE, 0U);
}

void BonderDebugEnvironment::stopBonder()
{
    m_bonder.disengage();
    setDone(ERROR_NONE, 0U);
}

void BonderDebugEnvironment::setButton()
{
    // arg(0): ButtonId, arg(1): 0 = released, 1 = pressed. Replicates the
    // operator input virtually; it merges with the physical pin rather than
    // overriding it.
    const uint32_t button = static_cast<uint32_t>(arg(0));
    const bool pressed = arg(1) != 0.0f;

    if (button == BUTTON_RIGHT) {
        s_virtualRightButton = pressed;
    } else if (button == BUTTON_LEFT) {
        s_virtualLeftButton = pressed;
    } else {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    updateCombinedButtons();
    setDone(ERROR_NONE, pressed ? 1U : 0U);
}

void BonderDebugEnvironment::resetLog()
{
    s_stepLog.count = 0U;

    for (uint16_t i = 0U; i < BONDER_DEBUG_LOG_DEPTH; ++i) {
        s_stepLog.entries[i] = {};
    }
}

void BonderDebugEnvironment::publishStatus()
{
    const BonderModule::VmStatus vm = m_bonder.getVmStatus();

    s_status.bonderRunning = vm.running ? 1U : 0U;
    s_status.pc = vm.pc;
    s_status.opcode = vm.opcode;
    s_status.waitMask = vm.mask;
    s_status.eventFlags = vm.eventFlags;
    s_status.stepCount = s_stepLog.count;
    s_status.clampState =
        static_cast<uint8_t>(m_clampSolenoidChannel.getState());
    s_status.rightButtonPhysical = s_physicalRightButton ? 1U : 0U;
    s_status.rightButtonVirtual = s_virtualRightButton ? 1U : 0U;
    s_status.leftButtonPhysical = s_physicalLeftButton ? 1U : 0U;
    s_status.leftButtonVirtual = s_virtualLeftButton ? 1U : 0U;
    s_status.zPosition = m_zMotorPositionControllerModule.getPosition();
    s_status.yPosition = m_yAxisRouterChannel.getPosition();
    s_status.tPosition = m_tAxisRouterChannel.getPosition();
}

void BonderDebugEnvironment::onTelemetry(void *context,
                                         const BonderModule::Telemetry &telemetry)
{
    auto *self = static_cast<BonderDebugEnvironment *>(context);

    if (self != nullptr) {
        self->recordTelemetry(telemetry);
    }
}

void BonderDebugEnvironment::onMouseRightButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    (void)ctx;
    s_physicalRightButton = (state == PinMonitorChannel::PinState::ACTIVE);
    updateCombinedButtons();
}

void BonderDebugEnvironment::onMouseLeftButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    (void)ctx;
    s_physicalLeftButton = (state == PinMonitorChannel::PinState::ACTIVE);
    updateCombinedButtons();
}

void BonderDebugEnvironment::updateCombinedButtons()
{
    const bool right = s_physicalRightButton || s_virtualRightButton;
    const bool left = s_physicalLeftButton || s_virtualLeftButton;

    if (right != s_combinedRightButton) {
        s_combinedRightButton = right;
        m_bonder.notifyRightButton(right);
    }

    if (left != s_combinedLeftButton) {
        s_combinedLeftButton = left;
        m_bonder.notifyLeftButton(left);
    }
}

void BonderDebugEnvironment::recordTelemetry(const BonderModule::Telemetry &telemetry)
{
    BonderModule::Telemetry copy = telemetry;
    copyImpedanceCurve(copy);

    const uint32_t slot = s_stepLog.count % BONDER_DEBUG_LOG_DEPTH;
    s_stepLog.entries[slot] = copy;
    s_stepLog.count = s_stepLog.count + 1U;
}

void BonderDebugEnvironment::copyImpedanceCurve(BonderModule::Telemetry &telemetry)
{
    // Scan results ride on the record that consumed EVENT_SCAN_COMPLETED;
    // all other records carry a null curve pointer.
    if (m_curveBuffer == nullptr || telemetry.impedances == nullptr ||
        telemetry.scanCount == 0U ||
        m_curveCount + telemetry.scanCount > m_curveCapacity) {
        telemetry.impedances = nullptr;
        return;
    }

    complexf *destination = &m_curveBuffer[m_curveCount];
    for (uint16_t i = 0U; i < telemetry.scanCount; ++i) {
        destination[i] = telemetry.impedances[i];
    }

    telemetry.impedances = destination;
    m_curveCount += telemetry.scanCount;
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_BONDER
