#include "protocol_force_setup.hpp"

const BonderProtocol::Instruction ForceSetupProtocol::s_protocol[] = {
    // First-bond force measurement. The VM's reset prologue guarantees pc 0
    // starts at reset height with every event flag clear.
    {Op::WAIT, nullptr, EVENT_LEFT_BUTTON_PRESSED},
    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::MZDOWN, &B::manualLevelingRate, 0},
    {Op::SETFORCE, &B::forceCoilFirstBondForce, 0},
    {Op::TIMER, &B::forceSetupDuration, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},
    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::ZMOVE, &B::resetHeight, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    // Discard presses made during the hold/retraction, then repeat with the
    // second-bond force only after a fresh press.
    {Op::CLRFLAGS, nullptr, EVENT_LEFT_BUTTON_PRESSED},
    {Op::WAIT, nullptr, EVENT_LEFT_BUTTON_PRESSED},
    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::MZDOWN, &B::manualLevelingRate, 0},
    {Op::SETFORCE, &B::forceCoilSecondBondForce, 0},
    {Op::TIMER, &B::forceSetupDuration, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},
    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::ZMOVE, &B::resetHeight, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    {Op::SETFORCE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_FORCE_COIL_SETTLED, WAIT_TIMEOUT_MS},
};

const BonderProtocol::Instruction *ForceSetupProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t ForceSetupProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
