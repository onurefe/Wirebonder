#include "Protocol/protocol_reset_prologue.hpp"

const BonderProtocol::Instruction ProtocolResetPrologue::s_protocol[] = {
    {Op::SETFORCE, nullptr, 0U, 0U},
    {Op::CLAMPCLOSE, nullptr, 0U, 0U},
    {Op::ZMOVE, &BonderConfig::resetHeight, 0U, 0U},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED, BonderProtocol::WAIT_TIMEOUT_MS},
    {Op::CLRFLAGS, nullptr, 0}
};


const BonderProtocol::Instruction *ProtocolResetPrologue::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t ProtocolResetPrologue::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
