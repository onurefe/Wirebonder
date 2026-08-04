#include "Protocol/protocol_manual.hpp"

// Manual mode: the operator drives Z with the mouse buttons at the axis's
// normal move speed -- left button toward lowest overtravel, right button
// back toward the phase's starting height -- and releasing either stops the
// motion in place; Y makes a single reverse-to-normal kink stroke and there
// is no second-bond Y stepback.

const BonderProtocol::Instruction ManualBondingProtocol::s_protocol[] = {
    // ---- Phase 1: first bond, operator-controlled descent ----
    {Op::YREVERSE, &B::yReverseDisplacement, 0},
    {Op::WAIT, nullptr, EVENT_Y_MOVE_COMPLETED, WAIT_TIMEOUT_MS},
    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::MZDRIVE, &B::resetHeight,
        EVENT_CONTACT_DISCONNECTED | EVENT_FORCE_COIL_SETTLED},
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
    // ---- Phase 1: loop formation with the single kink stroke ----
    {Op::CLAMPOPEN, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},
    {Op::CLRFLAGS, nullptr, EVENT_CONTACT_CONNECTED},
    {Op::ZMOVE, &B::kinkHeight, 0},
    {Op::WAIT, nullptr, EVENT_CONTACT_CONNECTED, WAIT_TIMEOUT_MS},
    {Op::TMOVE, &B::tailDisplacement, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED | EVENT_T_MOVE_COMPLETED,
                        WAIT_TIMEOUT_MS},
    {Op::YMOVE, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_Y_MOVE_COMPLETED, WAIT_TIMEOUT_MS},
    {Op::ZMOVE, &B::loopHeight, 0},
    {Op::WAIT, nullptr, EVENT_Z_POSITION_REACHED, WAIT_TIMEOUT_MS},
    // ---- Phase 2: second bond, operator-controlled descent ----
    {Op::CLAMPOPEN, nullptr, 0},
    {Op::WAIT, nullptr, EVENT_CLAMP_SETTLED, WAIT_TIMEOUT_MS},
    {Op::SETFORCE, &B::forceCoilTrackingForce, 0},
    {Op::MZDRIVE, &B::loopHeight,
        EVENT_CONTACT_DISCONNECTED | EVENT_FORCE_COIL_SETTLED},
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
};

const BonderModule::Instruction *ManualBondingProtocol::getProtocolPtr() const
{
    return s_protocol;
}

uint8_t ManualBondingProtocol::getProtocolSize() const
{
    return static_cast<uint8_t>(sizeof(s_protocol) / sizeof(s_protocol[0]));
}
