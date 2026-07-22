#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_IMPEDANCE_SCANNER

#include "debug_environment_impedance_scanner.hpp"

#include <cmath>

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the scanner
// chain, but owned here outright.
// -----------------------------------------------------------------------------

uint16_t ImpedanceScannerDebugEnvironment::m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
uint16_t ImpedanceScannerDebugEnvironment::m_dac1Buffer[2 * DAC1_SAMPLES];
uint16_t ImpedanceScannerDebugEnvironment::m_synthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
uint16_t ImpedanceScannerDebugEnvironment::m_vsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
uint16_t ImpedanceScannerDebugEnvironment::m_isensBuffer[SCANNER_ADC_CAPTURE_SIZE];

RawAdcChannel ImpedanceScannerDebugEnvironment::m_scannerVsensChannel(
    ADC_CHANNEL_US_VSENS_CONVERSION_ORDER,
    ImpedanceScannerDebugEnvironment::m_vsensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

RawAdcChannel ImpedanceScannerDebugEnvironment::m_scannerIsensChannel(
    ADC_CHANNEL_US_ISENS_CONVERSION_ORDER,
    ImpedanceScannerDebugEnvironment::m_isensBuffer,
    SCANNER_ADC_CAPTURE_SIZE);

RawDacChannel ImpedanceScannerDebugEnvironment::m_scannerDacChannel(
    &hdac, &htim4, DAC_CHANNEL_1,
    ImpedanceScannerDebugEnvironment::m_dac1Buffer, 2 * DAC1_SAMPLES,
    ImpedanceScannerDebugEnvironment::m_synthesisBuffer,
    SCANNER_SYNTHESIS_BUFFER_SIZE);

AdcService ImpedanceScannerDebugEnvironment::m_adc1Service(
    &hadc1, &htim2,
    ADC1_BITS, ADC1_VOLTAGE_RANGE,
    ImpedanceScannerDebugEnvironment::m_adc1Buffer,
    2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS);

DacService ImpedanceScannerDebugEnvironment::m_dacService(&hdac);

UsImpedanceScannerModule ImpedanceScannerDebugEnvironment::m_scannerModule(
    &ImpedanceScannerDebugEnvironment::m_scannerDacChannel,
    &ImpedanceScannerDebugEnvironment::m_scannerVsensChannel,
    &ImpedanceScannerDebugEnvironment::m_scannerIsensChannel,
    ImpedanceScannerDebugEnvironment::m_synthesisBuffer,
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

ImpedanceScannerDebugEnvironment::ImpedanceScannerDebugEnvironment()
{
    // VSENS before ISENS so voltage dispatches first.
    m_adc1Service.addChannel(&m_scannerVsensChannel);
    m_adc1Service.addChannel(&m_scannerIsensChannel);

    m_dacService.addChannel(&m_scannerDacChannel);

    addProcess(&m_dacService);
    addProcess(&m_adc1Service);
    addProcess(&m_scannerModule);

    m_scannerModule.addScanCompleteListenerCallback(
        this, &ImpedanceScannerDebugEnvironment::onScanFinished);
}

void ImpedanceScannerDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_SCAN:
        startScan();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void ImpedanceScannerDebugEnvironment::startScan()
{
    float requestedCount = arg(0);
    float minFrequency = arg(1);
    float frequencyStep = arg(2);

    uint16_t numFrequencies =
        requestedCount > 0.0f
            ? static_cast<uint16_t>(requestedCount)
            : static_cast<uint16_t>(BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES);

    if (numFrequencies > BONDER_MODULE_SCAN_MAX_FREQUENCIES) {
        numFrequencies = BONDER_MODULE_SCAN_MAX_FREQUENCIES;
    }

    if (minFrequency <= 0.0f) {
        minFrequency = BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY;
    }

    if (frequencyStep <= 0.0f) {
        frequencyStep = BONDER_MODULE_DEFAULT_SCAN_FREQUENCY_STEP;
    }

    const float bin =
        static_cast<float>(DAC1_SAMPLING_FREQ) /
        static_cast<float>(SCANNER_SYNTHESIS_BUFFER_SIZE);

    minFrequency = bin * roundf(minFrequency / bin);
    frequencyStep = bin * roundf(frequencyStep / bin);

    if (frequencyStep < bin) {
        frequencyStep = bin;
    }

    setArg(0, static_cast<float>(numFrequencies));
    setArg(1, minFrequency);
    setArg(2, frequencyStep);

    m_scanCount = numFrequencies;
    m_scanActive = true;

    setBusy();

    setResultPointer(0, m_voltagePhasors);
    setResultPointer(1, m_currentPhasors);
    setResultPointer(2, m_impedances);

    m_scannerModule.setScanParameters(numFrequencies,
                                      minFrequency,
                                      frequencyStep);

    if (!m_scannerModule.beginScan(m_voltagePhasors,
                                   m_currentPhasors,
                                   m_impedances)) {
        m_scanActive = false;
        setError(ERROR_NOT_INITIALIZED);
    }
}

void ImpedanceScannerDebugEnvironment::abort()
{
    stopScan();
}

void ImpedanceScannerDebugEnvironment::stopScan()
{
    m_scannerModule.abortScan();

    m_scanActive = false;
    setIdle();
}

void ImpedanceScannerDebugEnvironment::onScanFinished(void *context,
                                                      complexf *,
                                                      complexf *,
                                                      complexf *)
{
    auto *self = static_cast<ImpedanceScannerDebugEnvironment *>(context);

    if (self != nullptr) {
        self->handleScanFinished();
    }
}

void ImpedanceScannerDebugEnvironment::handleScanFinished()
{
    if (!m_scanActive) {
        return;
    }

    m_scanActive = false;
    setDone(ERROR_NONE, m_scanCount);
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_IMPEDANCE_SCANNER
