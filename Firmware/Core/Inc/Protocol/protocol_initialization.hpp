#pragma once

#include "Protocol/protocol_common.hpp"

// Restores the machine to its idle posture: coil off, clamp closed, and every
// axis back at its reference. Run before a bonding protocol, and by the
// operator's reset, so a protocol always starts from a known state.
class InitializationProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
