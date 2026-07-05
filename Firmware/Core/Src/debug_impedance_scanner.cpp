#include "debug_impedance_scanner.hpp"

#include <cmath>

DebugImpedanceScanner ::DebugImpedanceScanner ()
{
}

void DebugImpedanceScanner ::init(UsImpedanceScannerModule *scanner)
{
    m_scanner = scanner;

    if (m_scanner != nullptr) {
        m_scanner->addScanCompleteListenerCallback(this,
                                                   &DebugImpedanceScanner ::onScanFinished);
    }
}

void DebugImpedanceScanner ::poll()
{
    if (m_scanner != nullptr) {
        m_scanner->execute();
    }
}

void DebugImpedanceScanner ::handleCommand(uint16_t localCommand)
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

void DebugImpedanceScanner ::startScan()
{
    if (m_scanner == nullptr) {
        setError(ERROR_NOT_INITIALIZED);
        return;
    }

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
    m_busy = true;

    setBusy();

    setResultPointer(0, m_voltagePhasors);
    setResultPointer(1, m_currentPhasors);
    setResultPointer(2, m_impedances);

    m_scanner->setScanParameters(numFrequencies,
                                 minFrequency,
                                 frequencyStep);

    m_scanner->start(m_voltagePhasors,
                     m_currentPhasors,
                     m_impedances);
}

void DebugImpedanceScanner::abort()
{
    stopScan();
}

void DebugImpedanceScanner::stopScan()
{
    if (m_scanner != nullptr) {
        m_scanner->stop();
    }

    m_busy = false;
    setIdle();
}

void DebugImpedanceScanner ::onScanFinished(void *context,
                                           complexf *,
                                           complexf *,
                                           complexf *)
{
    auto *self = static_cast<DebugImpedanceScanner  *>(context);

    if (self != nullptr) {
        self->handleScanFinished();
    }
}

void DebugImpedanceScanner ::handleScanFinished()
{
    if (!m_busy) {
        return;
    }

    m_busy = false;
    setDone(ERROR_NONE, m_scanCount);
}
