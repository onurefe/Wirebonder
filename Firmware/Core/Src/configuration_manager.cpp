#include "configuration_manager.hpp"
#include <cstddef>
#include <cstring>

ConfigurationManager::ConfigurationManager(EepromEmulator *eeprom)
    : m_eeprom(eeprom)
    , m_index{}
    , m_count(0U)
    , m_ready(false)
{
}

bool ConfigurationManager::initialize()
{
    if (m_eeprom == nullptr || !m_eeprom->init()) return false;
    m_count = 0U;
    m_ready = false;

    // New configuration fields are appended to BonderConfig. Accept both
    // earlier prefix-only record sizes; value-initializing StoredRecord below
    // supplies defaults for fields absent from an older record. A later save
    // writes the current full record format.
    constexpr uint32_t kLegacyStoredRecordSize =
        offsetof(StoredRecord, config) +
        offsetof(BonderConfig, tailAssistPower);
    constexpr uint32_t kTailAssistStoredRecordSize =
        offsetof(StoredRecord, config) +
        offsetof(BonderConfig, forceSetupDuration);

    EepromEmulator::ObjectCursor cursor = m_eeprom->beginObjectEnumeration();
    EepromEmulator::ObjectInfo info{};
    while (m_eeprom->getNextObject(cursor, info)) {
        if (info.id < kObjectIdBase ||
            info.id >= kObjectIdBase + kMaxConfigurations ||
            (info.length != sizeof(StoredRecord) &&
             info.length != kTailAssistStoredRecordSize &&
             info.length != kLegacyStoredRecordSize)) {
            continue;
        }
        StoredRecord record{};
        if (!m_eeprom->loadObject(info.id, &record, sizeof(record)) ||
            record.magic != kRecordMagic ||
            !isValidName(record.name)) {
            continue;
        }
        if (m_count >= kMaxConfigurations || indexOf(record.name) >= 0) {
            continue;
        }
        m_index[m_count].objectId = info.id;
        copyName(m_index[m_count].name, record.name);
        ++m_count;
    }

    m_ready = true;
    return true;
}

const char *ConfigurationManager::nameAt(uint16_t index) const
{
    return index < m_count ? m_index[index].name : nullptr;
}

bool ConfigurationManager::exists(const char *name) const
{
    return indexOf(name) >= 0;
}

bool ConfigurationManager::load(const char *name, BonderConfig& config) const
{
    const int32_t index = indexOf(name);
    if (!m_ready || index < 0) return false;

    StoredRecord record{};
    if (!m_eeprom->loadObject(m_index[index].objectId,
                              &record, sizeof(record)) ||
        record.magic != kRecordMagic) {
        return false;
    }
    config = record.config;
    return true;
}

bool ConfigurationManager::add(const char *name, const BonderConfig& config)
{
    if (!m_ready || !isValidName(name) ||
        m_count >= kMaxConfigurations || exists(name)) {
        return false;
    }

    uint16_t objectId;
    if (!findFreeObjectId(objectId)) return false;
    if (!writeRecord(objectId, name, config)) return false;

    m_index[m_count].objectId = objectId;
    copyName(m_index[m_count].name, name);
    ++m_count;
    return true;
}

bool ConfigurationManager::save(const char *name, const BonderConfig& config)
{
    const int32_t index = indexOf(name);
    if (!m_ready || index < 0) return false;
    return writeRecord(m_index[index].objectId, name, config);
}

bool ConfigurationManager::remove(const char *name)
{
    const int32_t index = indexOf(name);
    if (!m_ready || index < 0) return false;
    if (!m_eeprom->logicalErase(m_index[index].objectId)) return false;

    for (uint16_t i = static_cast<uint16_t>(index); i + 1U < m_count; ++i) {
        m_index[i] = m_index[i + 1U];
    }
    --m_count;
    return true;
}

int32_t ConfigurationManager::indexOf(const char *name) const
{
    if (name == nullptr) return -1;
    for (uint16_t i = 0U; i < m_count; ++i) {
        if (std::strncmp(m_index[i].name, name, kNameSize) == 0) {
            return i;
        }
    }
    return -1;
}

bool ConfigurationManager::findFreeObjectId(uint16_t& objectId) const
{
    for (uint16_t slot = 0U; slot < kMaxConfigurations; ++slot) {
        const uint16_t candidate =
            static_cast<uint16_t>(kObjectIdBase + slot);
        bool used = false;
        for (uint16_t i = 0U; i < m_count; ++i) {
            if (m_index[i].objectId == candidate) {
                used = true;
                break;
            }
        }
        if (!used) {
            objectId = candidate;
            return true;
        }
    }
    return false;
}

bool ConfigurationManager::writeRecord(uint16_t objectId,
                                       const char *name,
                                       const BonderConfig& config)
{
    StoredRecord record{};
    record.magic = kRecordMagic;
    copyName(record.name, name);
    record.config = config;
    return m_eeprom->saveObject(objectId, &record, sizeof(record));
}

bool ConfigurationManager::isValidName(const char *name)
{
    if (name == nullptr || name[0] == '\0') return false;
    for (uint8_t i = 0U; i < kNameSize; ++i) {
        if (name[i] == '\0') return true;
        if (name[i] < ' ' || name[i] > '~') return false;
    }
    return false; // Not NUL-terminated within kNameSize.
}

void ConfigurationManager::copyName(char *destination, const char *source)
{
    std::strncpy(destination, source, kNameSize - 1U);
    destination[kNameSize - 1U] = '\0';
}
