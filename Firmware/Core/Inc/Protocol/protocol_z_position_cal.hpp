#pragma once

#include "Protocol/protocol_common.hpp"

// Finds the Z origin by driving the axis onto it open-loop, and reports where
// the LVDT said it was so the offset can be corrected.
class ZPositionCalProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
