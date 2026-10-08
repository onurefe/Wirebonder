#pragma once

#include "Protocol/protocol_common.hpp"

// Manual mode: the operator drives Z with the mouse buttons -- right toward the
// phase's search height, left back toward its starting height -- and releasing
// either stops in place. The automatic sequence takes over once Z settles at
// the search height.
class ManualBondingProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
