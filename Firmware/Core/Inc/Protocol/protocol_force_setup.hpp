#pragma once

#include "Protocol/protocol_common.hpp"

// Force gauge setup: loads the coil against the operator's gauge for as long
// as they hold the button, so the reading can be compared with the commanded
// force.
class ForceSetupProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
