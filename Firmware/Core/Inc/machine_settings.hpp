#pragma once

#include "configuration.h"
#include "eeprom_emulator.hpp"
#include <cstdint>

// Machine-wide hardware tuning values -- one value each, always present,
// independent of which bonding configuration is loaded. Unlike
// ConfigurationManager (many named BonderConfig records), this is a single
// persisted record.
struct MachineSettingsData {
    float clampSolenoidVoltage; // volts, CLAMP_SOLENOID_VOLTAGE_MIN..MAX
    float areaLightLevel;       // percent, 0-100
    float spotlightLevel;       // percent, 0-100
    bool spotlightOn;           // whether the spotlight is enabled
    // Z position-loop velocity clamps, both positive magnitudes in mm/s
    // (ZMOTOR_MAX_*_SPEED_MIN..MAX). Robot::updateZPositionSpeedLimits()
    // negates the downward one when pushing them into the position loop.
    float zMotorMaxUpwardSpeed;
    float zMotorMaxDownwardSpeed;
    // Runtime correction from Start Tach. Cal., subtracted from the raw
    // tachometer reading by DcMotorVelocityControllerModule. Not user-edited
    // via the settings editor; written by Robot::onTachCalReport() and
    // applied every tick by Robot::applyTachometerCalibration().
    float tachometerVelocityOffset;
    // Force held by the Setup protocol (ForceSetupProtocol) while the
    // operator lowers the Z axis onto an external gauge, in grams. Edited on
    // the settings page ("Setup Force").
    float forceSetupTrackingForce;
    // Systematic force-coil error in grams: what the gauge actually read
    // during Setup minus the tracking force that was commanded. Subtracted
    // from every non-zero force command (see BonderModule's SETFORCE
    // handling). Not user-edited on the settings page; written when the
    // operator submits a measurement on the force-entry page. Zero until
    // Setup has been run at least once.
    float forceCoilForceOffset;
};

class MachineSettingsStore {
public:
    explicit MachineSettingsStore(EepromEmulator *eeprom);

    // Loads the persisted record, or seeds and persists defaults if none
    // exists yet.
    bool initialize();
    bool save();

    const MachineSettingsData& data() const { return m_data; }
    MachineSettingsData& mutableData() { return m_data; }

private:
    struct StoredRecord {
        uint32_t magic;
        MachineSettingsData settings;
    };

    // Bumped on every layout change (MST3 added the force-setup tracking
    // force and the force-coil offset); the length check below would already
    // reject older records, this makes it explicit.
    static constexpr uint32_t kRecordMagic = 0x4D535433UL; // "MST3"
    // Reserved, disjoint from the EEPROM emulator's build-reset record
    // (0x0001) and ConfigurationManager's range (0x0200-0x021F).
    static constexpr uint16_t kObjectId = 0x0002U;

    EepromEmulator *m_eeprom;
    MachineSettingsData m_data;
};
