#include "Protocol/protocol_tach_cal.hpp"

// Tachometer calibration. TACHMOVE holds the dedicated cal height, TACHSAMPLE
// accumulates tachometer velocity against the LVDT's ground truth over its own
// window, and TACHREPORT emits the residual.
//
// The LVDT is the reference here, so its offset has to be right: run the Z
// position calibration first if the axis has not been referenced.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction TachCalProtocol::s_protocol[] = {
    {B::TACHMOVE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},
    {B::TACHSAMPLE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED},
    {B::TACHREPORT, {}, 0},
};

const BonderModule::Instruction *TachCalProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t TachCalProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
