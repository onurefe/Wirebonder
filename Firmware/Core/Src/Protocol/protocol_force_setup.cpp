#include "Protocol/protocol_force_setup.hpp"

const BonderProtocol::Instruction ForceSetupProtocol::s_protocol[] = {
    // The reset prologue leaves the axis parked at reset height with the coil
    // off; nothing moves until the operator asks for it.
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_PRESSED, 0},

    // Load the coil, then send the axis to the workspace minimum. The gauge
    // (or the anvil) stops the descent well before the target in practice, so
    // there is deliberately no wait on Z_POSITION_REACHED here — the setpoint
    // is a direction to push in, not a position to arrive at.
    {Op::SETFORCE, &B::forceSetupTrackingForce, 0},
    {Op::ZMOVE, &B::lowestOvertravel, 0},

    // The operator reads their gauge and releases when done; that release is
    // the protocol's only completion condition.
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_RELEASED, 0},

    // Release the load before retracting so the coil isn't fighting the move.
    {Op::SETFORCE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_FORCE_COIL_SETTLED, WAIT_TIMEOUT_MS},
    {Op::ZMOVE, &B::resetHeight, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED, WAIT_TIMEOUT_MS},
};

const BonderProtocol::Instruction *ForceSetupProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t ForceSetupProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
