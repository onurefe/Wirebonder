#pragma once

#include "bonder_module.hpp"

// Start Tach. Cal.: holds Z at ZMOTOR_TACH_CAL_POSITION_MM until settled,
// then averages the tachometer velocity over ZMOTOR_TACH_CAL_SAMPLE_DURATION_S
// and reports it via BonderModule::TachCalReportCallback.
class TachCalProtocol final : public BonderProtocol {
public:
    const Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;

private:
    static const Instruction s_protocol[];
};
