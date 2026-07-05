#pragma once

#include "configuration.h"
#include <cstdint>

// Standalone configuration struct for BonderModule — lives here so UiModule and
// other consumers can include it without pulling in the full BonderModule dependency.
struct BonderConfig {
    // Force coil currents (A)
    float forceCoilIdleCurrent      = BONDER_MODULE_DEFAULT_FORCE_COIL_IDLE_CURRENT;
    float forceCoilSearchingCurrent = BONDER_MODULE_DEFAULT_FORCE_COIL_SEARCHING_CURRENT;
    float forceCoilSettlingCurrent  = BONDER_MODULE_DEFAULT_FORCE_COIL_SETTLING_CURRENT;
    float forceCoilWeldingCurrent   = BONDER_MODULE_DEFAULT_FORCE_COIL_WELDING_CURRENT;

    // Z-axis heights (mm) — Z increases upward, bond pad contact at z≈0
    float loopHeight                = BONDER_MODULE_DEFAULT_LOOP_HEIGHT;
    float resetHeight               = BONDER_MODULE_DEFAULT_RESET_HEIGHT;
    float searchHeight              = BONDER_MODULE_DEFAULT_SEARCH_HEIGHT;
    float kinkHeight                = BONDER_MODULE_DEFAULT_KINK_HEIGHT;
    float lowestOvertravel          = BONDER_MODULE_DEFAULT_LOWEST_OVERTRAVEL;

    // XY-axis displacements (mm)
    float tailDisplacement          = BONDER_MODULE_DEFAULT_TAIL_DISPLACEMENT;
    float tearDisplacement          = BONDER_MODULE_DEFAULT_TEAR_DISPLACEMENT;
    float yReverseDisplacement      = BONDER_MODULE_DEFAULT_Y_REVERSE_DISPLACEMENT;
    float yStepbackDisplacement     = BONDER_MODULE_DEFAULT_Y_STEPBACK_DISPLACEMENT;

    // Ultrasonic bonding
    float bondingEnergy             = BONDER_MODULE_DEFAULT_BONDING_ENERGY;
    float targetPower               = BONDER_MODULE_DEFAULT_TARGET_POWER;
    float maxBondingDuration        = BONDER_MODULE_DEFAULT_MAX_BONDING_DURATION;

    // Timing (s)
    float settlingTime              = BONDER_MODULE_DEFAULT_SETTLING_TIME;
    float coolingTime               = BONDER_MODULE_DEFAULT_COOLING_TIME;
    float tailRestoreDelay          = BONDER_MODULE_DEFAULT_TAIL_RESTORE_DELAY;
    float yRestoreDelay             = BONDER_MODULE_DEFAULT_Y_RESTORE_DELAY;

    // Impedance scan sweep
    uint16_t numOfScannedFrequencies = BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES;
    float    scanStartFrequency      = BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY;
    float    scanStopFrequency       = BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY;
};
