#include "machine_settings.hpp"

MachineSettingsStore::MachineSettingsStore(EepromEmulator *eeprom)
    : m_eeprom(eeprom)
    , m_data{}
{
}

bool MachineSettingsStore::initialize()
{
    if (m_eeprom == nullptr) return false;

    StoredRecord record{};
    uint32_t length = 0U;
    if (m_eeprom->loadObject(kObjectId, &record, sizeof(record), &length) &&
        length == sizeof(record) &&
        record.magic == kRecordMagic) {
        m_data = record.settings;
        return true;
    }

    m_data.clampSolenoidVoltage = CLAMP_SOLENOID_VOLTAGE_DEFAULT;
    m_data.areaLightLevel = AREA_LIGHT_LEVEL_DEFAULT;
    m_data.spotlightLevel = SPOTLIGHT_LEVEL_DEFAULT;
    m_data.spotlightOn = SPOTLIGHT_ON_DEFAULT;
    m_data.zMotorMaxUpwardSpeed = ZMOTOR_MAX_UPWARD_SPEED_DEFAULT;
    m_data.zMotorMaxDownwardSpeed = ZMOTOR_MAX_DOWNWARD_SPEED_DEFAULT;
    m_data.tachometerVelocityOffset = ZMOTOR_TACHOMETER_VELOCITY_OFFSET_DEFAULT;
    m_data.forceSetupTrackingForce = FORCE_SETUP_TRACKING_FORCE_DEFAULT;
    m_data.forceCoilForceOffset = FORCE_COIL_FORCE_OFFSET_DEFAULT;
    return save();
}

bool MachineSettingsStore::save()
{
    if (m_eeprom == nullptr) return false;

    const StoredRecord record{kRecordMagic, m_data};
    return m_eeprom->saveObject(kObjectId, &record, sizeof(record));
}
