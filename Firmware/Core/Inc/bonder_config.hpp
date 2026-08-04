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

    // Bonding force (grams). Converted to force-coil current right before
    // it reaches ForceCoilDriverModule (see BonderModule's SETFORCE
    // handling) using the bench current->force fit
    // (force_coil_calibration_fit.py against Core/Inc/force_coil_current_to_gram.txt).
    float forceCoilConstantForce        = BONDER_MODULE_DEFAULT_FORCE_COIL_CONSTANT_FORCE_GRAMS;
    float forceCoilTrackingForce        = BONDER_MODULE_DEFAULT_FORCE_COIL_TRACKING_FORCE_GRAMS;
    float forceCoilFirstBondForce       = BONDER_MODULE_DEFAULT_FORCE_COIL_FIRST_BOND_FORCE_GRAMS;
    float forceCoilSecondBondForce      = BONDER_MODULE_DEFAULT_FORCE_COIL_SECOND_BOND_FORCE_GRAMS;

    float resetHeight                   = BONDER_MODULE_DEFAULT_RESET_HEIGHT;
    float loopHeight                    = BONDER_MODULE_DEFAULT_LOOP_HEIGHT;
    float firstSearchHeight             = BONDER_MODULE_DEFAULT_FIRST_SEARCH_HEIGHT;
    float secondSearchHeight            = BONDER_MODULE_DEFAULT_SECOND_SEARCH_HEIGHT;
    float kinkHeight                    = BONDER_MODULE_DEFAULT_KINK_HEIGHT;
    float lowestOvertravel              = BONDER_MODULE_DEFAULT_LOWEST_OVERTRAVEL;

    // Table-tear mode: Z height held while the Y-axis forms and tears the tail.
    float secondZHeight                 = BONDER_MODULE_DEFAULT_SECOND_Z_HEIGHT;

    // Non-negative T-axis travel added to wherever the axis sits when each
    // move (Opcode::TMOVE) starts -- the tear move's actual endpoint depends
    // on where the tail move left the axis, not a fixed T coordinate.
    float tailDisplacement              = BONDER_MODULE_DEFAULT_TAIL_DISPLACEMENT;
    float tearDisplacement              = BONDER_MODULE_DEFAULT_TEAR_DISPLACEMENT;
    // Magnitude moved in the negative Y direction from wherever the axis
    // currently sits (see Opcode::YREVERSE), not an absolute position.
    float yReverseDisplacement          = BONDER_MODULE_DEFAULT_Y_REVERSE_DISPLACEMENT;
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

    // Machine-wide values mirrored in from MachineSettingsData rather than
    // edited or owned per profile: Robot stamps both on the copy it hands to
    // BonderModule::configure() (see Robot::configureBonderModule()), so
    // whatever a persisted record happens to carry here is always
    // overwritten before the VM can read it. They live in BonderConfig
    // because protocol operands are BonderConfig member pointers and the
    // force correction is applied against m_config.
    float forceSetupTrackingForce     = FORCE_SETUP_TRACKING_FORCE_DEFAULT;
    float forceCoilForceOffset        = FORCE_COIL_FORCE_OFFSET_DEFAULT;
};
