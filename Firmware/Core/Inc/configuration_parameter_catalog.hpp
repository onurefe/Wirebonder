#pragma once

#include "bonder_config.hpp"
#include <cstdint>

// Declarative table of the editable BonderConfig fields: screen grouping,
// display scaling and limits, value type, and per-protocol availability.
class ConfigurationParameterCatalog {
public:
    enum class Parameter : uint8_t {
        Search1,
        Power1,
        Energy1,
        Force1Current,
        Stepback,
        KinkHeight,
        Reverse,
        LoopHeight,
        Search2,
        Power2,
        Energy2,
        Force2Current,
        Tail,
        Tear,
        ResetHeight,
        ManualZSpeed,
        ManualZStopDist,
        ZMoveSpeed,
        ZMoveAcceleration,
        Overtravel,
        SecondZHeight,
        TableTail,
        TableTear,
        BondTimeout,
        ContactSettle,
        Cooling,
        TearStabilize,
        ConstantCurrent,
        TrackingCurrent,
        ScanStart,
        ScanStop,
        ScanPoints,
        TailAssistPower,
        TailAssistEnergy
    };

    struct Descriptor {
        Parameter parameter;
        uint16_t offset;
        float scale;          // display = raw * scale + displayOffset
        float displayOffset;  // raw     = (display - displayOffset) / scale
        float minDisplay;
        float maxDisplay;
        float stepDisplay;    // +/- press size, in display units (whole numbers for integer params)
        uint8_t screenIndex;
        bool isInteger;
        uint8_t modeMask;
    };

    static constexpr uint8_t kScreenCount = 6U;
    static constexpr uint8_t kParameterCount = 35U;

    static uint8_t screenParameterCount(uint8_t screen, BondingMode mode);
    static const Descriptor *at(uint8_t screen,
                                uint8_t visibleIndex,
                                BondingMode mode);
    static const Descriptor *byOffset(uint16_t offset);
    static bool isAvailable(const Descriptor *descriptor, BondingMode mode);
    // Digits after the point, derived from the step: showing more than a
    // press can change is noise, showing fewer hides the press entirely.
    static uint8_t displayDecimals(const Descriptor *descriptor);

    // A held key steps by decades so long ranges stay reachable, but the same
    // multiplier on a short range turns a hold into a slam between the limits.
    // These cut the requested multiplier down to the largest decade that still
    // needs at least kMinRepeatsToCrossRange repeats to walk the whole range,
    // so every value takes a comparable amount of holding to traverse
    // regardless of how many steps wide it happens to be.
    static constexpr float kMinRepeatsToCrossRange = 20.0f;
    static uint16_t limitStepScale(uint16_t requested,
                                   float minDisplay,
                                   float maxDisplay,
                                   float stepDisplay);
    static uint16_t limitStepScale(uint16_t requested,
                                   const Descriptor *descriptor);

private:
    static uint8_t modeBit(BondingMode mode);
    static const Descriptor kParameters[kParameterCount];
};
