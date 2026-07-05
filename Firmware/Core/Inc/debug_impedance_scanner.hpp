#ifndef IMPEDANCE_DEBUG_CHANNEL_HPP
#define IMPEDANCE_DEBUG_CHANNEL_HPP

#include <cstdint>

#include "configuration.h"
#include "complex.h"
#include "debug_service.hpp"
#include "us_impedance_scanner_module.hpp"

class DebugImpedanceScanner  : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_IMPEDANCE_SCANNER;

    enum Command : uint16_t {
        CMD_SCAN = 1
    };

    DebugImpedanceScanner ();

    void init(UsImpedanceScannerModule *scanner);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    void handleCommand(uint16_t localCommand) override;

    void poll() override;

    bool isBusy() const override
    {
        return m_busy;
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

    UsImpedanceScannerModule *m_scanner = nullptr;

    complexf m_voltagePhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};
    complexf m_currentPhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};
    complexf m_impedances[BONDER_MODULE_SCAN_MAX_FREQUENCIES] = {};

    uint16_t m_scanCount = 0;
    bool m_busy = false;
};

#endif /* IMPEDANCE_DEBUG_CHANNEL_HPP */
