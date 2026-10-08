#pragma once

#include "Protocol/protocol_common.hpp"

// Measures the tachometer's zero-offset residual against the LVDT over a fixed
// window, at a dedicated cal height.
class TachCalProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
