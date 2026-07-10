#pragma once

#include "configuration.h"
#include <cstdint>

// Standalone configuration struct for BonderModule — lives here so UiModule and
// other consumers can include it without pulling in the full BonderModule dependency.
struct BonderConfig {
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

    // Y/T logical positions (mm), measured from the router origin.
    float tailPosition                  = BONDER_MODULE_DEFAULT_TAIL_POSITION;
    float tearPosition                  = BONDER_MODULE_DEFAULT_TEAR_POSITION;
    float yReversePosition              = BONDER_MODULE_DEFAULT_Y_REVERSE_POSITION;
    float yStepbackPosition             = BONDER_MODULE_DEFAULT_Y_STEPBACK_POSITION;

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
    float yRestoreDelay                 = BONDER_MODULE_DEFAULT_Y_RESTORE_DELAY;

    // Impedance scan sweep
    uint16_t numOfScannedFrequencies = BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES;
    float    scanStartFrequency      = BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY;
    float    scanStopFrequency       = BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY;
};
