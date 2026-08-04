#include "Protocol/protocol_tach_cal.hpp"

const BonderProtocol::Instruction TachCalProtocol::s_protocol[] = {
    // The VM's reset prologue guarantees pc 0 starts at reset height with
    // every event flag clear; TACHMOVE then holds the dedicated cal height.
    {Op::TACHMOVE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED, WAIT_TIMEOUT_MS},
    {Op::TACHSAMPLE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED, WAIT_TIMEOUT_MS},
    {Op::TACHREPORT, nullptr, 0},
};

const BonderProtocol::Instruction *TachCalProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t TachCalProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
