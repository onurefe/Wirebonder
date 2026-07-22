#pragma once

#include "configuration.h"
#include <cstdint>

// Bonding protocol selector. Each mode is a different VM sequence table in
// BonderModule; the tables share step implementations.
enum class BondingMode : uint8_t {
    SemiAutomatic = 0,
    Manual        = 1,
    TableTear     = 2,
    LangeCoupling = 3
};

// Standalone configuration struct for BonderModule — lives here so
// ConfigurationEditor and other consumers can include it without
// pulling in the full BonderModule dependency.
struct BonderConfig {
    BondingMode bondingMode             = BondingMode::SemiAutomatic;

    // Force coil currents (A)
    float forceCoilConstantCurrent      = BONDER_MODULE_DEFAULT_FORCE_COIL_CONSTANT_CURRENT;
    float forceCoilTrackingCurrent      = BONDER_MODULE_DEFAULT_FORCE_COIL_TRACKING_CURRENT;
    float forceCoilFirstBondCurrent     = BONDER_MODULE_DEFAULT_FORCE_COIL_FIRST_BOND_CURRENT;
    float forceCoilSecondBondCurrent    = BONDER_MODULE_DEFAULT_FORCE_COIL_SECOND_BOND_CURRENT;

    float resetHeight                   = BONDER_MODULE_DEFAULT_RESET_HEIGHT;
    float loopHeight                    = BONDER_MODULE_DEFAULT_LOOP_HEIGHT;
    float firstSearchHeight             = BONDER_MODULE_DEFAULT_FIRST_SEARCH_HEIGHT;
    float secondSearchHeight            = BONDER_MODULE_DEFAULT_SECOND_SEARCH_HEIGHT;
    float kinkHeight                    = BONDER_MODULE_DEFAULT_KINK_HEIGHT;
    float lowestOvertravel              = BONDER_MODULE_DEFAULT_LOWEST_OVERTRAVEL;

    // Manual-mode Z jog rate (mm/s), applied while a mouse button is held.
    float manualLevelingRate            = BONDER_MODULE_DEFAULT_MANUAL_LEVELING_RATE;

    // Table-tear mode: Z height held while the Y-axis forms and tears the tail.
    float secondZHeight                 = BONDER_MODULE_DEFAULT_SECOND_Z_HEIGHT;

    // Y logical positions are measured from the router origin. The configured
    // T endpoints are centered around zero by BonderModule while preserving
    // the tail-to-tear distance.
    float tailPosition                  = BONDER_MODULE_DEFAULT_TAIL_POSITION;
    float tearPosition                  = BONDER_MODULE_DEFAULT_TEAR_POSITION;
    float yReversePosition              = BONDER_MODULE_DEFAULT_Y_REVERSE_POSITION;
    float yStepbackPosition             = BONDER_MODULE_DEFAULT_Y_STEPBACK_POSITION;

    // Table-tear mode Y positions; the T-axis positions go unused there.
    float yTailPosition                 = BONDER_MODULE_DEFAULT_Y_TAIL_POSITION;
    float yTearPosition                 = BONDER_MODULE_DEFAULT_Y_TEAR_POSITION;

    // Ultrasonic bonding
    float firstBondingPower             = BONDER_MODULE_DEFAULT_FIRST_BONDING_POWER;
    float secondBondingPower            = BONDER_MODULE_DEFAULT_SECOND_BONDING_POWER;
    float firstBondingEnergy            = BONDER_MODULE_DEFAULT_FIRST_BONDING_ENERGY;
    float secondBondingEnergy           = BONDER_MODULE_DEFAULT_SECOND_BONDING_ENERGY;
    float maxBondingDuration            = BONDER_MODULE_DEFAULT_MAX_BONDING_DURATION;

    // Timing (s)
    float coolingTime                   = BONDER_MODULE_DEFAULT_COOLING_TIME;
    float contactSettlingTime           = BONDER_MODULE_DEFAULT_SETTLING_TIME;
    float tailRestoreDelay              = BONDER_MODULE_DEFAULT_TAIL_RESTORE_DELAY;
    float tearStabilizationTime         = BONDER_MODULE_DEFAULT_TEAR_STABILIZATION_TIME;

    // Impedance scan sweep
    uint16_t numOfScannedFrequencies = BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES;
    float    scanStartFrequency      = BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY;
    float    scanStopFrequency       = BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY;

    // Ultrasonic tail assist. Kept at the end so older persisted
    // configurations retain the offsets of every existing field.
    float tailAssistPower             = BONDER_MODULE_DEFAULT_TAIL_ASSIST_POWER;
    float tailAssistEnergy            = BONDER_MODULE_DEFAULT_TAIL_ASSIST_ENERGY;

    // Fixed-duration force measurement used by the Setup protocol. Kept out
    // of the normal configuration editor, but persisted with the profile.
    float forceSetupDuration          = BONDER_MODULE_DEFAULT_FORCE_SETUP_DURATION;
};
