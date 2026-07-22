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
    // parameter                       field offset                                                     scale  min                                max                                group int    protocols
    {Parameter::Search1,               offsetof(BonderConfig, firstSearchHeight),                       1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               0, false, MODE_AUTO},
    {Parameter::Power1,                offsetof(BonderConfig, firstBondingPower),                        1.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         0, false, MODE_ALL},
    {Parameter::Energy1,               offsetof(BonderConfig, firstBondingEnergy),                       1.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       0, false, MODE_ALL},
    {Parameter::Force1Current,         offsetof(BonderConfig, forceCoilFirstBondCurrent),                1.0f, CONFIGURATION_EDITOR_FORCE_CURRENT_MIN,        CONFIGURATION_EDITOR_FORCE_CURRENT_MAX,        0, false, MODE_ALL},

    {Parameter::Stepback,              offsetof(BonderConfig, yStepbackPosition),                        1.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   1, false, MODE_AUTO},
    {Parameter::KinkHeight,            offsetof(BonderConfig, kinkHeight),                               1.0f, CONFIGURATION_EDITOR_KINK_HEIGHT_MIN,          CONFIGURATION_EDITOR_KINK_HEIGHT_MAX,          1, false, MODE_ALL},
    {Parameter::Reverse,               offsetof(BonderConfig, yReversePosition),                         1.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   1, false, MODE_ALL},
    {Parameter::LoopHeight,            offsetof(BonderConfig, loopHeight),                               1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               1, false, MODE_ALL},

    {Parameter::Search2,               offsetof(BonderConfig, secondSearchHeight),                       1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               2, false, MODE_AUTO},
    {Parameter::Power2,                offsetof(BonderConfig, secondBondingPower),                       1.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         2, false, MODE_ALL},
    {Parameter::Energy2,               offsetof(BonderConfig, secondBondingEnergy),                      1.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       2, false, MODE_ALL},
    {Parameter::Force2Current,         offsetof(BonderConfig, forceCoilSecondBondCurrent),               1.0f, CONFIGURATION_EDITOR_FORCE_CURRENT_MIN,        CONFIGURATION_EDITOR_FORCE_CURRENT_MAX,        2, false, MODE_ALL},
    {Parameter::Tail,                  offsetof(BonderConfig, tailPosition),                             1.0f, CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MAX,   2, false, MODE_T_AXIS},
    {Parameter::Tear,                  offsetof(BonderConfig, tearPosition),                             1.0f, CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MAX,   2, false, MODE_T_AXIS},

    {Parameter::ResetHeight,           offsetof(BonderConfig, resetHeight),                              1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               3, false, MODE_ALL},
    {Parameter::Overtravel,            offsetof(BonderConfig, lowestOvertravel),                         1.0f, CONFIGURATION_EDITOR_OVERTRAVEL_MIN,           CONFIGURATION_EDITOR_OVERTRAVEL_MAX,           3, false, MODE_AUTO},
    {Parameter::ManualZRate,           offsetof(BonderConfig, manualLevelingRate),                       1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               3, false, MODE_MANUAL},
    {Parameter::SecondZHeight,         offsetof(BonderConfig, secondZHeight),                            1.0f, CONFIGURATION_EDITOR_HEIGHT_MIN,               CONFIGURATION_EDITOR_HEIGHT_MAX,               3, false, MODE_TABLE},
    {Parameter::TableTail,             offsetof(BonderConfig, yTailPosition),                            1.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   3, false, MODE_TABLE},
    {Parameter::TableTear,             offsetof(BonderConfig, yTearPosition),                            1.0f, CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN,   CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX,   3, false, MODE_TABLE},

    {Parameter::BondTimeout,           offsetof(BonderConfig, maxBondingDuration),                       1.0f, CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MIN, CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MAX, 4, false, MODE_ALL},
    {Parameter::ContactSettle,         offsetof(BonderConfig, contactSettlingTime),                      1.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               4, false, MODE_ALL},
    {Parameter::Cooling,               offsetof(BonderConfig, coolingTime),                              1.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               4, false, MODE_ALL},
    {Parameter::TailDelay,             offsetof(BonderConfig, tailRestoreDelay),                         1.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               4, false, MODE_T_AXIS},
    {Parameter::TearStabilize,         offsetof(BonderConfig, tearStabilizationTime),                    1.0f, CONFIGURATION_EDITOR_TIMING_MIN,               CONFIGURATION_EDITOR_TIMING_MAX,               4, false, MODE_TABLE},

    {Parameter::ConstantCurrent,       offsetof(BonderConfig, forceCoilConstantCurrent),                 1.0f, CONFIGURATION_EDITOR_FORCE_CURRENT_MIN,        CONFIGURATION_EDITOR_FORCE_CURRENT_MAX,        5, false, MODE_ALL},
    {Parameter::TrackingCurrent,       offsetof(BonderConfig, forceCoilTrackingCurrent),                 1.0f, CONFIGURATION_EDITOR_FORCE_CURRENT_MIN,        CONFIGURATION_EDITOR_FORCE_CURRENT_MAX,        5, false, MODE_ALL},
    {Parameter::ScanStart,             offsetof(BonderConfig, scanStartFrequency),                       0.001f, CONFIGURATION_EDITOR_SCAN_FREQ_MIN,          CONFIGURATION_EDITOR_SCAN_FREQ_MAX,            5, false, MODE_ALL},
    {Parameter::ScanStop,              offsetof(BonderConfig, scanStopFrequency),                        0.001f, CONFIGURATION_EDITOR_SCAN_FREQ_MIN,          CONFIGURATION_EDITOR_SCAN_FREQ_MAX,            5, false, MODE_ALL},
    {Parameter::ScanPoints,            offsetof(BonderConfig, numOfScannedFrequencies),                  1.0f, CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MIN,       CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MAX,       5, true,  MODE_ALL},
    {Parameter::TailAssistPower,       offsetof(BonderConfig, tailAssistPower),                           1.0f, CONFIGURATION_EDITOR_TARGET_POWER_MIN,         CONFIGURATION_EDITOR_TARGET_POWER_MAX,         5, false, MODE_T_AXIS},
    {Parameter::TailAssistEnergy,      offsetof(BonderConfig, tailAssistEnergy),                          1.0f, CONFIGURATION_EDITOR_BONDING_ENERGY_MIN,       CONFIGURATION_EDITOR_BONDING_ENERGY_MAX,       5, false, MODE_T_AXIS},
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

const ConfigurationParameterCatalog::Descriptor *
ConfigurationParameterCatalog::byOffset(uint16_t offset)
{
    for (uint8_t i = 0U; i < kParameterCount; ++i) {
        if (kParameters[i].offset == offset) return &kParameters[i];
    }
    return nullptr;
}
