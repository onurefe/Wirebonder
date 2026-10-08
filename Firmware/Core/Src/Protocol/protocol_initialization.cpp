#include "Protocol/protocol_initialization.hpp"

// Restores the machine to its idle posture. Every mechanism a protocol can
// leave displaced is commanded back here -- the coil to zero, the clamp
// closed, and Z, Y and T to their references -- so whatever ran before, or
// failed before, the next protocol starts from the same place.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction InitializationProtocol::s_protocol[] = {
    {B::SETFORCE, {&kZero, &cfg.forceCoilForceOffset}, 0},
    {B::CLAMPCLOSE, {}, 0},
    {B::ZMOVE, {&cfg.resetHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::YMOVE, {&kZero}, 0},
    {B::TMOVE, {&kZero, &kAbsolute}, 0},
    {B::WAITFLAGS, {}, B::EVENT_FORCE_COIL_SETTLED | B::EVENT_CLAMP_SETTLED |
                       B::EVENT_Z_POSITION_REACHED | B::EVENT_Y_MOVE_COMPLETED |
                       B::EVENT_T_MOVE_COMPLETED},
};

const BonderModule::Instruction *InitializationProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t InitializationProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
