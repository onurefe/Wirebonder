#include "Protocol/protocol_z_position_cal.hpp"

// Z position calibration: the measurement every absolute Z number rests on.
//
// Both loops are bypassed and a small fixed drive is applied for a fixed time,
// first upwards and then downwards. The upward phase is what makes the reading
// repeatable: it takes up the drivetrain's slack and leaves the lever's
// friction in the same state every run, so the origin is always approached
// from the same side and the answer does not depend on where the head happened
// to start. The downward phase then runs the head onto the bottom of travel
// and holds it there.
//
// Wherever it ends up is declared BONDER_MODULE_ZAXIS_MIN_POSITION. ZREFERENCE
// reads the LVDT there and reports it; the difference is the correction to the
// sensor's offset, which the robot persists.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction ZPositionCalProtocol::s_protocol[] = {
    {B::OPENZMOVE, {&zCalRetreatDrive, &zCalPhaseTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_OPEN_MOVE_COMPLETED},

    {B::OPENZMOVE, {&zCalApproachDrive, &zCalPhaseTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_OPEN_MOVE_COMPLETED},

    {B::ZREFERENCE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_REFERENCE_MEASURED},
};

const BonderModule::Instruction *ZPositionCalProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t ZPositionCalProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
