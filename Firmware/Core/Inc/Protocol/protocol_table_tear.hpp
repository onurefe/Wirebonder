#pragma once

#include "Protocol/protocol_common.hpp"

// Table-tear mode: as semi-automatic until the phase-2 tear, which the Y axis
// performs against the table while Z holds the second Z height. The T axis
// never moves.
class TableTearBondingProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
