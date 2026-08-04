#ifndef DEBUG_ENVIRONMENT_IMPEDANCE_SCANNER_HPP
#define DEBUG_ENVIRONMENT_IMPEDANCE_SCANNER_HPP

#include <cstdint>

#include "configuration.h"
#include "complex.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "adc_service.hpp"
#include "dac_service.hpp"
#include "us_impedance_scanner_module.hpp"

// Sandbox for the ultrasonic impedance scanner: multitone DAC synthesis with
// raw V/I capture and the scanner module's phasor extraction.
class ImpedanceScannerDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_IMPEDANCE_SCANNER;

    enum Command : uint16_t {
        CMD_SCAN = 1
    };

    ImpedanceScannerDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    void handleCommand(uint16_t localCommand) override;

    bool isBusy() const override
    {
        return m_scanActive;
    }

    void abort() override;

private:
    static void onScanFinished(void *context,
                               complexf *voltagePhasors,
                               complexf *currentPhasors,
                               complexf *impedances);

    void startScan();
    void stopScan();
    void handleScanFinished();

    // Hardware sandbox — exclusively owned by this environment.
    static uint16_t m_adc1Buffer[2 * ADC1_SAMPLES_PER_CHANNEL * ADC1_NUM_CONVERSIONS];
    static uint16_t m_dac1Buffer[2 * DAC1_SAMPLES];
    static uint16_t m_synthesisBuffer[SCANNER_SYNTHESIS_BUFFER_SIZE];
    static uint16_t m_vsensBuffer[SCANNER_ADC_CAPTURE_SIZE];
    static uint16_t m_isensBuffer[SCANNER_ADC_CAPTURE_SIZE];

    static RawAdcChannel m_scannerVsensChannel;
    static RawAdcChannel m_scannerIsensChannel;
    static RawDacChannel m_scannerDacChannel;

    static AdcService m_adc1Service;
    static DacService m_dacService;

    static UsImpedanceScannerModule m_scannerModule;

    complexf m_voltagePhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};
    complexf m_currentPhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};
    complexf m_impedances[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};

    uint16_t m_scanCount = 0;
    bool m_scanActive = false;
};

#endif /* DEBUG_ENVIRONMENT_IMPEDANCE_SCANNER_HPP */
