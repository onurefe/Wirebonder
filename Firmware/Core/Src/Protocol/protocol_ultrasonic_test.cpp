#include "Protocol/protocol_ultrasonic_test.hpp"

// Ultrasonic bench test: sweep the transducer, drive it at the operating point
// the sweep produced, and report what was delivered. No motion at all, so it
// runs with the machine wherever it stands.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction UltrasonicTestProtocol::s_protocol[] = {
    {B::SCAN, {&cfg.numOfScannedFrequencies, &cfg.scanStartFrequency,
               &cfg.scanStopFrequency, &cfg.tailAssistPower}, 0},
    {B::WAITFLAGS, {}, B::EVENT_SCAN_COMPLETED},
    {B::PLL, {&tailAssistEnergy, &cfg.maxBondingDuration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_US_POWER_TRANSFERRED},
    {B::USREPORT, {}, 0},
};

const BonderModule::Instruction *UltrasonicTestProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t UltrasonicTestProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
