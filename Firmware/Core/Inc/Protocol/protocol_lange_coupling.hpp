#pragma once

#include "Protocol/protocol_common.hpp"

// Lange coupling: like semi-automatic, but the phase-1 loop is formed by the T
// axis alone and the Y reverse stroke happens during the second-bond descent.
class LangeCouplingBondingProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
