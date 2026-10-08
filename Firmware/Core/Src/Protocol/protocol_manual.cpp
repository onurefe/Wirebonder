#include "Protocol/protocol_manual.hpp"

// Manual mode: the operator drives Z with the mouse buttons -- right button
// toward the phase's search height, left button back toward the phase's
// starting height -- and releasing either stops the motion in place. Those two
// hand-driven descents run at the profile's manualZSpeed; every automatic move
// around them keeps the machine settings' speed. The descent hands over to the automatic sequence once Z
// settles at the search height. Y makes a single reverse-to-normal kink stroke
// and there is no second-bond Y stepback.
//
// Two things read differently from the pre-refactor table: contact is waited on
// with WAITCONTACTSTATE, which reads the sensor's level and so cannot be
// missed, placed after the WAITFLAGS for the move so a descent's transient
// lever lag has resolved first; and the buttons are waited on with the button
// commands, since nothing raises button flags in the background any more.
// WAITFLAGS carries no timeout -- each command driving a mechanism has its own
// deadline, and a stalled one fails the protocol, which releases the wait.
//
// MZDRIVE no longer holds the pc by itself: it raises Z_POSITION_REACHED when
// the carriage settles at the lower bound, and the WAITFLAGS after it is what
// waits.

using B = BonderModule;
using namespace protocol;

const BonderModule::Instruction ManualBondingProtocol::s_protocol[] = {
    // ---- Phase 1: first bond ----
    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::WAITFLAGS, {}, B::EVENT_FORCE_COIL_SETTLED},

    // Operator-paced descent between reset height and the search height.
    {B::MZDRIVE, {&cfg.firstSearchHeight, &cfg.resetHeight, &cfg.manualZSpeed,
                  &cfg.manualZStopDistance}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},

    // A fast hand-driven descent can leave the lever hanging behind the
    // carriage; hold here (still under tracking force) until it is seated.
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
    // Clamp stays open through the operator's (unbounded) reaction time, then
    // pulses closed and open (wire tension / anti-slack pulse) once triggered.
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonPressed}, 0},

    {B::CLAMPCLOSE, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},
    {B::CLAMPOPEN, {}, 0},
    {B::WAITFLAGS, {}, B::EVENT_CLAMP_SETTLED},

    {B::SETFORCE, {&cfg.forceCoilTrackingForce, &cfg.forceCoilForceOffset}, 0},
    {B::WAITFLAGS, {}, B::EVENT_FORCE_COIL_SETTLED},

    // MZDRIVE reads the buttons' held state, so the trigger press above would
    // otherwise bleed straight into the descent; make the operator let go.
    {B::WAITMOUSERIGHTBUTTONEVENTS, {&kButtonReleased}, 0},
    {B::MZDRIVE, {&cfg.secondSearchHeight, &cfg.loopHeight, &cfg.manualZSpeed,
                  &cfg.manualZStopDistance}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Z_POSITION_REACHED},

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
    {B::WAITCONTACTSTATE, {&kContactSeated}, WAIT_TIMEOUT_MS},

    {B::YMOVE, {&kZero}, 0},
    {B::WAITFLAGS, {}, B::EVENT_Y_MOVE_COMPLETED},
};

const BonderModule::Instruction *ManualBondingProtocol::getProtocolPtr()
{
    return s_protocol;
}

uint8_t ManualBondingProtocol::getProtocolSize()
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
