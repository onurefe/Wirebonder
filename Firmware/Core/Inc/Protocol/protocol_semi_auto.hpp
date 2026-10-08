#pragma once

#include "Protocol/protocol_common.hpp"

// Semi-automatic mode: the operator triggers each phase with the right mouse
// button; descent, tail formation (T axis), tear and restoration are automatic.
class SemiAutoBondingProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
