#include "configuration_parameter_catalog.hpp"
#include "configuration.h"
#include <cstddef>

namespace {

constexpr uint8_t modeBit(BondingMode mode)
{
    return static_cast<uint8_t>(1U << static_cast<uint8_t>(mode));
}

constexpr uint8_t MODE_SEMI   = modeBit(BondingMode::SemiAutomatic);
constexpr uint8_t MODE_MANUAL = modeBit(BondingMode::Manual);
constexpr uint8_t MODE_TABLE  = modeBit(BondingMode::TableTear);
constexpr uint8_t MODE_LANGE  = modeBit(BondingMode::LangeCoupling);
constexpr uint8_t MODE_AUTO   = MODE_SEMI | MODE_TABLE | MODE_LANGE;
constexpr uint8_t MODE_T_AXIS = MODE_SEMI | MODE_MANUAL | MODE_LANGE;
constexpr uint8_t MODE_ALL    = MODE_AUTO | MODE_MANUAL;

} // namespace

// The first three groups follow the 4523AD schedule. Firmware-specific
// parameters remain visible in Motion, Timing and U/S Setup groups.
// clang-format off
const ConfigurationParameterCatalog::Descriptor ConfigurationParameterCatalog::kParameters[kParameterCount] = {
    // Display units are what the operator sees and edits: raw * scale. Power,
    // energy and timing are stored in SI (W, J, s) but shown in mW, mJ and ms
    // (scale 1000), so their min/max/step macros are in milli-units. The
    // displayed decimals follow from the step (see displayDecimals()), which
    // also keeps "5000" ms inside the 7-char value field.
    // parameter                  field offset                                      scale    offs  min                                            max                                            step                                          grp int    protocols
    {Parameter::Search1,          offsetof(BonderConfig, firstSearchHeight),        1.0f,    0.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               CONFIGURATION_EDITOR_SEARCH1_STEP,            0, false, MODE_AUTO},
    {Parameter::Power1,           offsetof(BonderConfig, firstBondingPower),        1000.0f, 0.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         CONFIGURATION_EDITOR_POWER1_STEP,             0, false, MODE_ALL},
    {Parameter::Energy1,          offsetof(BonderConfig, firstBondingEnergy),       1000.0f, 0.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       CONFIGURATION_EDITOR_ENERGY1_STEP,            0, false, MODE_ALL},
    {Parameter::Force1Current,    offsetof(BonderConfig, forceCoilFirstBondForce),  1.0f,    0.0f, CONFIGURATION_EDITOR_FORCE_GRAMS_MIN,          CONFIGURATION_EDITOR_FORCE_GRAMS_MAX,          CONFIGURATION_EDITOR_FORCE1_STEP,             0, false, MODE_ALL},

    {Parameter::Stepback,         offsetof(BonderConfig, yStepbackPosition),        1.0f,    0.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_STEPBACK_STEP,           1, false, MODE_AUTO},
    {Parameter::KinkHeight,       offsetof(BonderConfig, kinkHeight),               1.0f,    0.0f, CONFIGURATION_EDITOR_KINK_HEIGHT_MIN,          CONFIGURATION_EDITOR_KINK_HEIGHT_MAX,          CONFIGURATION_EDITOR_KINK_HEIGHT_STEP,        1, false, MODE_ALL},
    {Parameter::Reverse,          offsetof(BonderConfig, yReverseDisplacement),     1.0f,    0.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_REVERSE_STEP,            1, false, MODE_ALL},
    {Parameter::LoopHeight,       offsetof(BonderConfig, loopHeight),               1.0f,    0.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               CONFIGURATION_EDITOR_LOOP_HEIGHT_STEP,        1, false, MODE_ALL},

    {Parameter::Search2,          offsetof(BonderConfig, secondSearchHeight),       1.0f,    0.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               CONFIGURATION_EDITOR_SEARCH2_STEP,            2, false, MODE_AUTO},
    {Parameter::Power2,           offsetof(BonderConfig, secondBondingPower),       1000.0f, 0.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         CONFIGURATION_EDITOR_POWER2_STEP,             2, false, MODE_ALL},
    {Parameter::Energy2,          offsetof(BonderConfig, secondBondingEnergy),      1000.0f, 0.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       CONFIGURATION_EDITOR_ENERGY2_STEP,            2, false, MODE_ALL},
    {Parameter::Force2Current,    offsetof(BonderConfig, forceCoilSecondBondForce), 1.0f,    0.0f, CONFIGURATION_EDITOR_FORCE_GRAMS_MIN,          CONFIGURATION_EDITOR_FORCE_GRAMS_MAX,          CONFIGURATION_EDITOR_FORCE2_STEP,             2, false, MODE_ALL},
    {Parameter::Tail,             offsetof(BonderConfig, tailDisplacement),         1.0f,    0.0f, CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_TAIL_STEP,               2, false, MODE_T_AXIS},
    {Parameter::Tear,             offsetof(BonderConfig, tearDisplacement),         1.0f,    0.0f, CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_TEAR_STEP,               2, false, MODE_T_AXIS},

    {Parameter::ResetHeight,      offsetof(BonderConfig, resetHeight),              1.0f,    0.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               CONFIGURATION_EDITOR_RESET_HEIGHT_STEP,       3, false, MODE_ALL},
    {Parameter::Overtravel,       offsetof(BonderConfig, lowestOvertravel),         1.0f,    0.0f, CONFIGURATION_EDITOR_OVERTRAVEL_MIN,           CONFIGURATION_EDITOR_OVERTRAVEL_MAX,           CONFIGURATION_EDITOR_OVERTRAVEL_STEP,         3, false, MODE_AUTO},
    {Parameter::SecondZHeight,    offsetof(BonderConfig, secondZHeight),            1.0f,    0.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               CONFIGURATION_EDITOR_SECOND_Z_HEIGHT_STEP,    3, false, MODE_TABLE},
    {Parameter::TableTail,        offsetof(BonderConfig, yTailPosition),            1.0f,    0.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_TABLE_TAIL_STEP,         3, false, MODE_TABLE},
    {Parameter::TableTear,        offsetof(BonderConfig, yTearPosition),            1.0f,    0.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   CONFIGURATION_EDITOR_TABLE_TEAR_STEP,         3, false, MODE_TABLE},

    {Parameter::BondTimeout,      offsetof(BonderConfig, maxBondingDuration),       1000.0f, 0.0f, CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MIN, CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MAX, CONFIGURATION_EDITOR_BOND_TIMEOUT_STEP,       4, false, MODE_ALL},
    {Parameter::ContactSettle,    offsetof(BonderConfig, contactSettlingTime),      1000.0f, 0.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               CONFIGURATION_EDITOR_CONTACT_SETTLE_STEP,     4, false, MODE_ALL},
    {Parameter::Cooling,          offsetof(BonderConfig, coolingTime),              1000.0f, 0.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               CONFIGURATION_EDITOR_COOLING_STEP,            4, false, MODE_ALL},
    {Parameter::TailDelay,        offsetof(BonderConfig, tailRestoreDelay),         1000.0f, 0.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               CONFIGURATION_EDITOR_TAIL_DELAY_STEP,         4, false, MODE_T_AXIS},
    {Parameter::TearStabilize,    offsetof(BonderConfig, tearStabilizationTime),    1000.0f, 0.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               CONFIGURATION_EDITOR_TEAR_STABILIZE_STEP,     4, false, MODE_TABLE},

    {Parameter::ConstantCurrent,  offsetof(BonderConfig, forceCoilConstantForce),   1.0f,    0.0f, CONFIGURATION_EDITOR_FORCE_GRAMS_MIN,          CONFIGURATION_EDITOR_FORCE_GRAMS_MAX,          CONFIGURATION_EDITOR_CONSTANT_FORCE_STEP,     5, false, MODE_ALL},
    {Parameter::TrackingCurrent,  offsetof(BonderConfig, forceCoilTrackingForce),   1.0f,    0.0f, CONFIGURATION_EDITOR_FORCE_GRAMS_MIN,          CONFIGURATION_EDITOR_FORCE_GRAMS_MAX,          CONFIGURATION_EDITOR_TRACKING_FORCE_STEP,     5, false, MODE_ALL},
    {Parameter::ScanStart,        offsetof(BonderConfig, scanStartFrequency),       0.001f,  0.0f, CONFIGURATION_EDITOR_SCAN_FREQ_MIN,            CONFIGURATION_EDITOR_SCAN_FREQ_MAX,            CONFIGURATION_EDITOR_SCAN_START_STEP,         5, false, MODE_ALL},
    {Parameter::ScanStop,         offsetof(BonderConfig, scanStopFrequency),        0.001f,  0.0f, CONFIGURATION_EDITOR_SCAN_FREQ_MIN,            CONFIGURATION_EDITOR_SCAN_FREQ_MAX,            CONFIGURATION_EDITOR_SCAN_STOP_STEP,          5, false, MODE_ALL},
    {Parameter::ScanPoints,       offsetof(BonderConfig, numOfScannedFrequencies),  1.0f,    0.0f, CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MIN,       CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MAX,       CONFIGURATION_EDITOR_SCAN_POINTS_STEP,        5, true,  MODE_ALL},
    {Parameter::TailAssistPower,  offsetof(BonderConfig, tailAssistPower),          1000.0f, 0.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         CONFIGURATION_EDITOR_TAIL_ASSIST_POWER_STEP,  5, false, MODE_T_AXIS},
    {Parameter::TailAssistEnergy, offsetof(BonderConfig, tailAssistEnergy),         1000.0f, 0.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       CONFIGURATION_EDITOR_TAIL_ASSIST_ENERGY_STEP, 5, false, MODE_T_AXIS},
};
// clang-format on

uint8_t ConfigurationParameterCatalog::modeBit(BondingMode mode)
{
    return static_cast<uint8_t>(1U << static_cast<uint8_t>(mode));
}

bool ConfigurationParameterCatalog::isAvailable(const Descriptor *descriptor,
                                                BondingMode mode)
{
    return descriptor != nullptr &&
           (descriptor->modeMask & modeBit(mode)) != 0U;
}

uint8_t ConfigurationParameterCatalog::screenParameterCount(uint8_t screen,
                                                            BondingMode mode)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < kParameterCount; ++i) {
        if (kParameters[i].screenIndex == screen &&
            isAvailable(&kParameters[i], mode)) {
            ++count;
        }
    }
    return count;
}

const ConfigurationParameterCatalog::Descriptor *
ConfigurationParameterCatalog::at(uint8_t screen,
                                  uint8_t visibleIndex,
                                  BondingMode mode)
{
    uint8_t index = 0U;
    for (uint8_t i = 0U; i < kParameterCount; ++i) {
        if (kParameters[i].screenIndex != screen ||
            !isAvailable(&kParameters[i], mode)) {
            continue;
        }
        if (index == visibleIndex) return &kParameters[i];
        ++index;
    }
    return nullptr;
}

uint8_t
ConfigurationParameterCatalog::displayDecimals(const Descriptor *descriptor)
{
    if (descriptor == nullptr || descriptor->isInteger) return 0U;

    // Compared against the same float literals the step macros are defined
    // with, so a step of exactly 0.01f lands on 2 rather than falling
    // through to 3 on the representation error. FloatWidget caps at 3.
    const float step = descriptor->stepDisplay < 0.0f
        ? -descriptor->stepDisplay
        : descriptor->stepDisplay;
    if (step >= 1.0f)   return 0U;
    if (step >= 0.1f)   return 1U;
    if (step >= 0.01f)  return 2U;
    return 3U;
}

const ConfigurationParameterCatalog::Descriptor *
ConfigurationParameterCatalog::byOffset(uint16_t offset)
{
    for (uint8_t i = 0U; i < kParameterCount; ++i) {
        if (kParameters[i].offset == offset) return &kParameters[i];
    }
    return nullptr;
}
