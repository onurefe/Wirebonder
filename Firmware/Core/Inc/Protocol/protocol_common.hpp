#pragma once

#include "bonder_module.hpp"

// =============================================================================
// Shared operands for the protocol tables.
//
// An instruction carries its arguments as raw pointers, so even a literal
// needs an object to point at. These are the literals every table reuses; the
// commands only ever read them. Configuration-bound operands point straight
// into BonderModule::m_config instead, so a table always reads the live value.
// =============================================================================

namespace protocol {

// A protocol's own reference to the live configuration, so tables can read
// `cfg.firstSearchHeight` rather than spelling out the module each time.
inline constexpr BonderConfig &cfg = BonderModule::m_config;

// Height the tail feed is released at on the way up from the tear. Lives in
// the module rather than the profile; see BonderModule::m_tailFeedHeight.
inline constexpr float &tailFeedHeight = BonderModule::m_tailFeedHeight;

// Dwell between starting the tail-assist drive and drawing the tail; see
// BonderModule::m_tailVibrationBuildupTime.
inline constexpr float &tailVibrationBuildupTime =
    BonderModule::m_tailVibrationBuildupTime;

// Energy the tail-assist drive is given, sized to outlast the tail draw; see
// BonderModule::m_tailAssistEnergy.
inline constexpr float &tailAssistEnergy = BonderModule::m_tailAssistEnergy;

// Zero, for the operands that mean "off" (force) or "origin" (an axis).
inline float kZero = 0.0f;

// TMOVE: travel added to wherever the axis sits, versus an absolute move.
inline bool kRelative = true;
inline bool kAbsolute = false;

// Contact sensor. Seated: the lever has caught up with the Z carriage.
// Separated: the tip is loaded and the lever has lifted off it.
inline bool kContactSeated    = true;
inline bool kContactSeparated = false;

// Mouse buttons, for both the edge and the level waits.
inline bool kButtonPressed  = true;
inline bool kButtonReleased = false;

// Deadline for a wait that should not sit forever. The commands driving the
// mechanisms carry their own deadlines now (BONDER_COMMAND_*_TIMEOUT_MS), and
// a stalled one fails the protocol on its own, so WAITFLAGS never needs one --
// only the waits on an external event do.
constexpr uint32_t WAIT_TIMEOUT_MS = 10000U;

} // namespace protocol
