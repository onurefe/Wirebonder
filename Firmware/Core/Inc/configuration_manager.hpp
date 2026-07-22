#pragma once

#include "bonder_config.hpp"
#include "configuration.h"
#include "eeprom_emulator.hpp"
#include <cstdint>

// Name-keyed persistent store for bonding configurations — a minimal
// database server over the EEPROM emulator. Pure CRUD: it has no notion of
// an active configuration, protocol defaults, or UI state; callers supply
// complete configurations and refer to them by name.
class ConfigurationManager {
public:
    static constexpr uint8_t kNameSize = ROBOT_CONFIGURATION_NAME_SIZE;
    static constexpr uint16_t kMaxConfigurations = 32U;

    explicit ConfigurationManager(EepromEmulator *eeprom);

    // Builds the in-RAM name index from flash.
    bool initialize();
    bool isReady() const { return m_ready; }

    // --- Listing -------------------------------------------------------
    uint16_t count() const { return m_count; }
    // Name of the index-th stored configuration, nullptr past the end.
    const char *nameAt(uint16_t index) const;
    bool exists(const char *name) const;

    // --- CRUD ----------------------------------------------------------
    bool load(const char *name, BonderConfig& config) const;
    // Fails if the name is taken, invalid, or the store is full.
    bool add(const char *name, const BonderConfig& config);
    // Updates an existing configuration; fails if the name is unknown.
    bool save(const char *name, const BonderConfig& config);
    bool remove(const char *name);

private:
    struct StoredRecord {
        uint32_t magic;
        char name[kNameSize];
        BonderConfig config;
    };

    struct IndexEntry {
        uint16_t objectId;
        char name[kNameSize];
    };

    static constexpr uint32_t kRecordMagic = 0x42434D31UL; // "BCM1"
    // Own object-ID range, disjoint from ConfigurationRepository's records.
    static constexpr uint16_t kObjectIdBase = 0x0200U;

    int32_t indexOf(const char *name) const;
    bool findFreeObjectId(uint16_t& objectId) const;
    bool writeRecord(uint16_t objectId,
                     const char *name,
                     const BonderConfig& config);
    static bool isValidName(const char *name);
    static void copyName(char *destination, const char *source);

    EepromEmulator *m_eeprom;
    IndexEntry m_index[kMaxConfigurations];
    uint16_t m_count;
    bool m_ready;
};
