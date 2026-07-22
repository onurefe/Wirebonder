#include "protocol_ultrasonic_test.hpp"

const BonderProtocol::Instruction UltrasonicTestProtocol::s_protocol[] = {
    {Op::SCAN, &B::firstBondingPower, 0},
    {Op::WAIT, nullptr, EVENT_SCAN_COMPLETED, WAIT_TIMEOUT_MS},
    {Op::PLL, &B::firstBondingEnergy, 0},
    {Op::WAIT, nullptr, EVENT_US_POWER_TRANSFERRED,
                        US_TRANSFER_WAIT_TIMEOUT_MS},
    {Op::USREPORT, nullptr, 0},
};

const BonderProtocol::Instruction *UltrasonicTestProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t UltrasonicTestProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
