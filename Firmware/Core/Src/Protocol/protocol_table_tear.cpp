#include "Protocol/protocol_table_tear.hpp"

// Table-tear mode: identical to semi-automatic until the phase-2 tear, but the
// T axis never moves -- the tail is formed and torn by the Y axis while Z
// holds the second Z height. No ultrasonic power outside the bond
// instructions.
//
// Two things read differently from the pre-refactor table: contact is waited on
// with WAITCONTACTSTATE, which reads the sensor's level and so cannot be
// missed, placed after the WAITFLAGS for the move so a descent's transient
// lever lag has resolved first; and the buttons are waited on with the button
// commands, since nothing raises button flags in the background any more.
// WAITFLAGS carries no timeout -- each command driving a mechanism has its own
// deadline, and a stalled one fails the protocol, which releases the wait.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction TableTearBondingProtocol::s_protocol[] = {
    // ---- Phase 1: first bond ----
    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.firstSearchHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonReleased}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_FORCE_COIL_SETTLED},

    // A short trigger press descends fast enough that the lever falls behind
    // the carriage; hold here (still under tracking force) until it is seated.
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::SETFORCE, {&cfg.forceCoilConstantForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.lowestOvertravel, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_FORCE_COIL_SETTLED},
    {B::WAITCONTACTSTATE, {&kContactSeparated}, WAIT_TIMEOUT_MS},

    {B::TIMER, {&cfg.contactSettlingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED},

    {B::SETFORCE, {&cfg.forceCoilFirstBondForce, &cfg.forceCoilForceOffset}, 0},
    {B::TIMER, {&cfg.contactSettlingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED | B::EVENT_FORCE_COIL_SETTLED},

    {B::SCAN, {&cfg.numOfScannedFrequencies, &cfg.scanStartFrequency,
               &cfg.scanStopFrequency, &cfg.firstBondingPower}, 0},
    {B::WAITFLAGS, {}, B::EVENT_SCAN_COMPLETED},

    {B::PLL, {&cfg.firstBondingEnergy, &cfg.maxBondingDuration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_US_POWER_TRANSFERRED},
    {B::USREPORT, {}, 0},

    {B::SETFORCE, {&cfg.forceCoilConstantForce, &cfg.forceCoilForceOffset}, 0},
    {B::TIMER, {&cfg.coolingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED | B::EVENT_FORCE_COIL_SETTLED},

    // ---- Phase 1: loop formation, no T-axis tail ----
    {B::CLAMPOPEN, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::ZMOVE, {&cfg.kinkHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::YREVERSE, {&cfg.yReverseDisplacement}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},

    {B::ZMOVE, {&cfg.loopHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},

    // ---- Phase 2: second bond ----
    // Clamp stays open through the operator's (unbounded) reaction time, then
    // pulses closed and open (wire tension / anti-slack pulse) once triggered.
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonPressed}, 0},

    {B::CLAMPCLOSE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},
    {B::CLAMPOPEN, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.secondSearchHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::YMOVE, {&cfg.yStepbackPosition}, 0},
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonReleased}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_Y_MOVE_COMPLETED |
                       B::EVENT_FORCE_COIL_SETTLED},

    // Same lever-lag guard as phase 1 (see above).
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::SETFORCE, {&cfg.forceCoilConstantForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.lowestOvertravel, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_FORCE_COIL_SETTLED},
    {B::WAITCONTACTSTATE, {&kContactSeparated}, WAIT_TIMEOUT_MS},

    {B::TIMER, {&cfg.contactSettlingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED},

    {B::SETFORCE, {&cfg.forceCoilSecondBondForce, &cfg.forceCoilForceOffset}, 0},
    {B::TIMER, {&cfg.contactSettlingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED | B::EVENT_FORCE_COIL_SETTLED},

    {B::SCAN, {&cfg.numOfScannedFrequencies, &cfg.scanStartFrequency,
               &cfg.scanStopFrequency, &cfg.secondBondingPower}, 0},
    {B::WAITFLAGS, {}, B::EVENT_SCAN_COMPLETED},

    {B::PLL, {&cfg.secondBondingEnergy, &cfg.maxBondingDuration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_US_POWER_TRANSFERRED},
    {B::USREPORT, {}, 0},

    {B::SETFORCE, {&cfg.forceCoilConstantForce, &cfg.forceCoilForceOffset}, 0},
    {B::TIMER, {&cfg.coolingTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED | B::EVENT_FORCE_COIL_SETTLED},

    // ---- Phase 2: Y-axis tail, tear and simultaneous restore ----
    {B::ZMOVE, {&cfg.secondZHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::YMOVE, {&cfg.yTailPosition}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},

    {B::CLAMPCLOSE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::YMOVE, {&cfg.yTearPosition}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},

    {B::TIMER, {&cfg.tearStabilizationTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED},

    {B::YMOVE, {&kZero}, 0},
    {B::ZMOVE, {&cfg.resetHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED | B::EVENT_Z_POSITION_REACHED},
};

const BonderModule::Instruction *TableTearBondingProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t TableTearBondingProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
