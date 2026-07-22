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
        Overtravel,
        ManualZRate,
        SecondZHeight,
        TableTail,
        TableTear,
        BondTimeout,
        ContactSettle,
        Cooling,
        TailDelay,
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
        float scale;
        float minDisplay;
        float maxDisplay;
        uint8_t screenIndex;
        bool isInteger;
        uint8_t modeMask;
    };

    static constexpr uint8_t kScreenCount = 6U;
    static constexpr uint8_t kParameterCount = 32U;

    static uint8_t screenParameterCount(uint8_t screen, BondingMode mode);
    static const Descriptor *at(uint8_t screen,
                                uint8_t visibleIndex,
                                BondingMode mode);
    static const Descriptor *byOffset(uint16_t offset);
    static bool isAvailable(const Descriptor *descriptor, BondingMode mode);

private:
    static uint8_t modeBit(BondingMode mode);
    static const Descriptor kParameters[kParameterCount];
};
