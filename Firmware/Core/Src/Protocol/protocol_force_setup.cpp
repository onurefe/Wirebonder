#include "Protocol/protocol_force_setup.hpp"

// Force gauge setup. Nothing moves until the operator asks for it, and the
// descent deliberately has no wait on arrival: the gauge (or the anvil) stops
// it well before the target, so the setpoint is a direction to push in rather
// than a position to reach.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction ForceSetupProtocol::s_protocol[] = {
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonPressed}, 0},

    {B::SETFORCE, {&cfg.forceSetupTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.lowestOvertravel, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},

    // The operator reads their gauge and releases when done; that release is
    // the protocol's only completion condition.
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonReleased}, 0},

    // Release the load before retracting so the coil isn't fighting the move.
    {B::SETFORCE, {&kZero, &cfg.forceCoilForceOffset}, 0},
    {B::WAITFLAGS, {}, B::EVENT_FORCE_COIL_SETTLED},
    {B::ZMOVE, {&cfg.resetHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},
};

const BonderModule::Instruction *ForceSetupProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t ForceSetupProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
