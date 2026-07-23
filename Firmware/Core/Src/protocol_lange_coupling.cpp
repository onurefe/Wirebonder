#include "protocol_lange_coupling.hpp"

// Semi-automatic mode: the operator triggers each phase with the right mouse
// button; descent, tail formation (T axis), tear and restoration are automatic.

const BonderProtocol::Instruction LangeCouplingBondingProtocol::s_protocol[] = {
    // ---- Phase 1: first bond ----
    // The VM's reset prologue guarantees pc 0 starts at reset height with
    // every event flag clear, so a stale press can never act as the trigger.
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_PRESSED},

    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::ZMOVE, &B::firstSearchHeight, 0},
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_RELEASED},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::ZMOVE, &B::lowestOvertravel, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_CONTACT_DISCONNECTED |
                        EVENT_FORCE_COIL_SETTLED, WAIT_TIMEOUT_MS},

    {Op::TIMER, &B::contactSettlingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED, WAIT_TIMEOUT_MS},

    {Op::SETFORCE, &B::forceCoilFirstBondForce, 0},
    {Op::TIMER, &B::contactSettlingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    {Op::SCAN, &B::firstBondingPower, 0},
    {Op::WAIT, nullptr, EVENT_SCAN_COMPLETED, WAIT_TIMEOUT_MS},

    {Op::PLL, &B::firstBondingEnergy, 0},
    {Op::WAIT, nullptr, EVENT_US_POWER_TRANSFERRED,
                        US_TRANSFER_WAIT_TIMEOUT_MS},
    {Op::USREPORT, nullptr, 0},

    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::TIMER, &B::coolingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    // ---- Phase 1: loop formation ----
    {Op::CLAMPOPEN, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},

    {Op::CLRFLAGS, nullptr, EVENT_CONTACT_CONNECTED},
    {Op::ZMOVE, &B::kinkHeight, 0},
    {Op::WAIT, nullptr, EVENT_CONTACT_CONNECTED, WAIT_TIMEOUT_MS},

    {Op::TMOVE, &B::tailDisplacement, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_T_MOVE_COMPLETED,
                        WAIT_TIMEOUT_MS},

    {Op::ZMOVE, &B::loopHeight, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED, WAIT_TIMEOUT_MS},

    // ---- Phase 2: second bond ----
    // Clamp stays open through the operator's (unbounded) reaction time, so
    // the wire is never left rigidly fixed for an unpredictable duration;
    // once triggered, it pulses closed then open (K&S-style wire tension/
    // anti-slack pulse, no deliberate hold -- just the solenoid's own
    // settle time) before the second-bond search.
    {Op::CLRFLAGS, nullptr, EVENT_RIGHT_BUTTON_PRESSED |
                            EVENT_RIGHT_BUTTON_RELEASED},
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_PRESSED},

    {Op::CLAMPCLOSE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},
    {Op::CLAMPOPEN, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},

    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::ZMOVE, &B::secondSearchHeight, 0},
    {Op::YMOVE, &B::yStepbackPosition, 0},
    {Op::WAIT, nullptr, EVENT_RIGHT_BUTTON_RELEASED},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_Y_MOVE_COMPLETED |
                        EVENT_FORCE_COIL_SETTLED, WAIT_TIMEOUT_MS},

    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::ZMOVE, &B::lowestOvertravel, 0},
    {Op::YREVERSE, &B::yReverseDisplacement, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_Y_MOVE_COMPLETED |
                        EVENT_CONTACT_DISCONNECTED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    {Op::TIMER, &B::contactSettlingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED, WAIT_TIMEOUT_MS},

    {Op::SETFORCE, &B::forceCoilSecondBondForce, 0},
    {Op::TIMER, &B::contactSettlingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    {Op::SCAN, &B::secondBondingPower, 0},
    {Op::WAIT, nullptr, EVENT_SCAN_COMPLETED, WAIT_TIMEOUT_MS},

    {Op::PLL, &B::secondBondingEnergy, 0},
    {Op::WAIT, nullptr, EVENT_US_POWER_TRANSFERRED,
                        US_TRANSFER_WAIT_TIMEOUT_MS},
    {Op::USREPORT, nullptr, 0},

    {Op::SETFORCE, &B::forceCoilConstantForce, 0},
    {Op::TIMER, &B::coolingTime, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED | EVENT_FORCE_COIL_SETTLED,
                        WAIT_TIMEOUT_MS},

    // ---- Phase 2: tear and restore (T axis, ultrasonic tail assist) ----
    {Op::CLAMPCLOSE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},

    {Op::TMOVE, &B::tearDisplacement, 0},
    {Op::WAIT, nullptr, EVENT_T_MOVE_COMPLETED, WAIT_TIMEOUT_MS},

    {Op::CLRFLAGS, nullptr, EVENT_CONTACT_CONNECTED},
    {Op::ZMOVE, &B::resetHeight, 0},
    {Op::TIMER, &B::tailRestoreDelay, 0},
    {Op::WAIT, nullptr, EVENT_TIMER_EXPIRED, WAIT_TIMEOUT_MS},

    {Op::TMOVE, nullptr, 0},
    {Op::SCAN, &B::tailAssistPower, 0},
    {Op::WAIT, nullptr, EVENT_SCAN_COMPLETED, WAIT_TIMEOUT_MS},
    {Op::PLL, &B::tailAssistEnergy, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_T_MOVE_COMPLETED |
                        EVENT_US_POWER_TRANSFERRED | EVENT_CONTACT_CONNECTED,
                        US_TRANSFER_WAIT_TIMEOUT_MS},
                        
    {Op::YMOVE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_Y_MOVE_COMPLETED, WAIT_TIMEOUT_MS},
};

const BonderModule::Instruction *LangeCouplingBondingProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t LangeCouplingBondingProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
