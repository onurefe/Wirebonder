#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_PLL

#include "DebugEnvironment/debug_environment_pll.hpp"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the ultrasonic
// chain, but owned here outright.
// -----------------------------------------------------------------------------

uint16_t PllDebugEnvironment::m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
uint16_t PllDebugEnvironment::m_dac1Buffer[2 * DAC1_SAMPLES];

IQDemodulatorChannel PllDebugEnvironment::m_ultrasonicVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_VSENS_GAIN);

IQDemodulatorChannel PllDebugEnvironment::m_ultrasonicIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    ADC_CHANNEL_PLL_DEMODULATION_SAMPLES,
    static_cast<float>(PLL_MODULE_CENTER_FREQUENCY) / static_cast<float>(ADC1_SAMPLING_FREQ),
    ADC_CHANNEL_US_ISENS_GAIN);

SineGeneratorChannel PllDebugEnvironment::m_ultrasonicDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    PllDebugEnvironment::m_dac1Buffer, 2 * DAC1_SAMPLES,
    DAC1_SAMPLES, DAC1_BITS, DAC1_VOLTAGE_RANGE);

AdcService PllDebugEnvironment::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE, ADC1_NUM_CONVERSIONS,
    PllDebugEnvironment::m_adc1Buffer,
    2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

DacService PllDebugEnvironment::m_dacService(&hdac);

PllModule PllDebugEnvironment::m_pllModule(
    &PllDebugEnvironment::m_ultrasonicDacChannel,
    &PllDebugEnvironment::m_ultrasonicVsensChannel,
    &PllDebugEnvironment::m_ultrasonicIsensChannel,
    static_cast<float>(ADC1_SAMPLING_FREQ),
    static_cast<float>(PLL_MODULE_CONTROL_FREQ));

PllDebugEnvironment::PllDebugEnvironment()
{
    // VSENS before ISENS so voltage dispatches first.
    m_adc1Service.addChannel(&m_ultrasonicVsensChannel);
    m_adc1Service.addChannel(&m_ultrasonicIsensChannel);

    m_dacService.addChannel(&m_ultrasonicDacChannel);

    // Start order: converters before the control loop that uses them.
    addProcess(&m_dacService);
    addProcess(&m_adc1Service);
    addProcess(&m_pllModule);

    m_pllModule.addEventListenerCallback(this, &PllDebugEnvironment::onPllEvent);
}

void PllDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_START:
        startTransfer();
        break;

    case CMD_STOP:
        stopTransfer();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void PllDebugEnvironment::startTransfer()
{
    m_telemetry = static_cast<PllModule::TelemetrySample *>(telemetryBuffer());

    float centerFrequency = arg(0);
    float amplitude = arg(1);
    float bondingEnergy = arg(2);
    float maxDuration = arg(3);

    if (amplitude <= 0.0f) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if (centerFrequency <= 0.0f) {
        centerFrequency = PLL_MODULE_CENTER_FREQUENCY;
    }

    if (bondingEnergy <= 0.0f) {
        bondingEnergy = DEBUG_PLL_DEFAULT_BONDING_ENERGY;
    }

    if (maxDuration <= 0.0f) {
        maxDuration = DEBUG_PLL_DEFAULT_MAX_DURATION;
    }

    m_transferActive = true;

    setBusy();
    setResultPointer(0, m_telemetry);

    m_pllModule.setTelemetryBuffer(m_telemetry,
                                   DEBUG_PLL_TELEMETRY_DEPTH);

    if (!m_pllModule.beginTransfer(centerFrequency,
                                   amplitude,
                                   bondingEnergy,
                                   maxDuration)) {
        m_transferActive = false;
        setError(ERROR_NOT_INITIALIZED);
    }
}

void PllDebugEnvironment::stopTransfer()
{
    m_pllModule.abortTransfer();

    m_transferActive = false;
    setIdle();
}

void PllDebugEnvironment::abort()
{
    stopTransfer();
}

void PllDebugEnvironment::onPllEvent(void *context, PllModule::Event event)
{
    auto *self = static_cast<PllDebugEnvironment *>(context);

    if (self != nullptr) {
        self->handlePllEvent(event);
    }
}

void PllDebugEnvironment::handlePllEvent(PllModule::Event event)
{
    if (!m_transferActive) {
        return;
    }

    m_transferActive = false;

    const uint32_t resultCode =
        event == PllModule::Event::BondingCompleted
            ? RESULT_BONDING_COMPLETED
            : RESULT_DURATION_TIMEOUT;

    setDone(resultCode, m_pllModule.getTelemetryCount());
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_PLL
