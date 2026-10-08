#include "Protocol/protocol_semi_auto.hpp"

// Semi-automatic mode: the operator triggers each phase with the right mouse
// button; descent, tail formation (T axis), tear and restoration are automatic.
//
// Two things read differently from the pre-refactor table:
//
//   - Contact is waited on with WAITCONTACTSTATE, which reads the sensor's
//     level, rather than as a flag in a combined mask. The old table could
//     only be satisfied by an edge some other party had latched; a level test
//     cannot be missed, and placing it after the WAITFLAGS for the move means
//     a descent's transient lever lag has resolved before it is read.
//   - The buttons are waited on with the button commands, since nothing raises
//     button flags in the background any more.
//
// WAITFLAGS carries no timeout: every command driving a mechanism has its own
// deadline, and a stalled one fails the protocol, which releases the wait.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction SemiAutoBondingProtocol::s_protocol[] = {
    // ---- Phase 1: first bond ----
    // The initialization protocol leaves the machine at reset height with the
    // coil off and the clamp closed, so pc 0 is always entered from idle.
    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.firstSearchHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonReleased}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_FORCE_COIL_SETTLED},

    // A short trigger press descends fast enough that the lever falls behind
    // the carriage: Z reports the search height while the tip is still above
    // it. Hold here (still under tracking force) until the lever is seated.
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::SETFORCE, {&cfg.forceCoilConstantForce, &cfg.forceCoilForceOffset}, 0},
    {B::ZMOVE, {&cfg.lowestOvertravel, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_FORCE_COIL_SETTLED},
    // Touchdown: the carriage has arrived, so a lever still off its seat is
    // the tip carrying the load rather than a descent transient.
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

    // ---- Phase 1: loop formation ----
    {B::CLAMPOPEN, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::ZMOVE, {&cfg.kinkHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::TMOVE, {&cfg.tailDisplacement, &kRelative}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_T_MOVE_COMPLETED},

    {B::YREVERSE, {&cfg.yReverseDisplacement}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},

    {B::ZMOVE, {&cfg.loopHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},

    // ---- Phase 2: second bond ----
    // Clamp stays open through the operator's (unbounded) reaction time, so
    // the wire is never left rigidly fixed for an unpredictable duration;
    // once triggered, it pulses closed then open (K&S-style wire tension /
    // anti-slack pulse, no deliberate hold -- just the solenoid's own settle
    // time) before the second-bond search.
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonPressed}, 0},

    {B::CLAMPCLOSE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},
    {B::CLAMPOPEN, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::YMOVE, {&cfg.yStepbackPosition}, 0},
    {B::ZMOVE, {&cfg.secondSearchHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
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

    // ---- Phase 2: tear and restore (T axis, ultrasonic tail assist) ----
    {B::CLAMPCLOSE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::TMOVE, {&cfg.tearDisplacement, &kRelative}, 0},
    {B::WAITFLAGS, {}, B::EVENT_T_MOVE_COMPLETED},

    {B::ZMOVE, {&cfg.resetHeight, &cfg.zMoveMaxSpeed,
                &cfg.zMoveMaxAcceleration}, 0},
    // The tail assist is hung off a height the carriage passes on its way up
    // rather than off a delay, so it happens at the same place in the travel
    // whatever the Z speed is. The marker is its own variable, so the timing
    // can be moved without disturbing a bonding parameter.
    {B::WAITZPOSITION, {&tailFeedHeight}, WAIT_TIMEOUT_MS},
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},
    {B::SCAN, {&cfg.numOfScannedFrequencies, &cfg.scanStartFrequency,
               &cfg.scanStopFrequency, &cfg.tailAssistPower}, 0},
    {B::WAITFLAGS, {}, B::EVENT_SCAN_COMPLETED},

    {B::PLL, {&tailAssistEnergy, &cfg.maxBondingDuration}, 0},
    // Let the transducer ring up before the wire moves, so the tail is drawn
    // through a tool that is already vibrating.
    {B::TIMER, {&tailVibrationBuildupTime}, 0},
    {B::WAITFLAGS, {}, B::EVENT_TIMER_EXPIRED},
    {B::TMOVE, {&kZero, &kAbsolute}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED | B::EVENT_T_MOVE_COMPLETED |
                       B::EVENT_US_POWER_TRANSFERRED},
    // The lever re-seats as the carriage returns to reset height.
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::YMOVE, {&kZero}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},
};

const BonderModule::Instruction *SemiAutoBondingProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t SemiAutoBondingProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
