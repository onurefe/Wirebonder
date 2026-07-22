#pragma once

#include "stm32f4xx_hal.h"
#include "configuration.h"
#include <cstdint>

// Log-structured EEPROM emulation over two interchangeable flash areas.
// New records are appended to the active area. When it fills, the newest
// committed version of every object is copied directly from flash into the
// alternate area. The alternate area is marked active before the old area is
// erased, so a reset at any point leaves at least one complete generation.
class EepromEmulator {
public:
    EepromEmulator();

    struct ObjectInfo {
        uint16_t id;
        uint32_t length;
    };

    struct ObjectCursor {
        int32_t nextEntryIndex;
    };

    bool init();
    bool saveObject(uint16_t objectId, const void *data, uint32_t length);
    bool loadObject(uint16_t objectId,
                    void *data,
                    uint32_t capacity,
                    uint32_t *length = nullptr) const;
    bool getObjectInfo(uint16_t objectId, ObjectInfo& info) const;
    ObjectCursor beginObjectEnumeration() const;
    bool getNextObject(ObjectCursor& cursor, ObjectInfo& info) const;
    // Appends a tombstone. Older versions remain physically present until an
    // area rollover but are immediately inaccessible through loadObject().
    bool logicalErase(uint16_t objectId);
    // Appends a reset barrier. All records before it become inaccessible;
    // physical areas are erased only when normal rollover requires it.
    bool logicalReset();
    // Activates an empty generation in the alternate area, then physically
    // erases the previously active area.
    bool hardReset();

private:
    struct FlashArea {
        uint32_t sector;
        uint32_t address;
    };

    struct AreaHeader {
        uint32_t magic;
        uint32_t generation;
        uint32_t generationInverse;
        uint32_t preparedMarker;
        uint32_t transferMarker;
        uint32_t activeMarker;
    };

    // The ID word is programmed last and therefore commits the record.
    struct Entry {
        uint32_t id;
        uint32_t dataOffset;
        uint32_t length;
        uint32_t checksum;
    };

    static constexpr uint8_t  kAreaCount = 2U;
    static constexpr uint8_t  kAreaA = 0U;
    static constexpr uint8_t  kAreaB = 1U;
    static constexpr uint8_t  kNoArea = 0xFFU;

    static constexpr uint32_t kAreaMagic = 0x45455033UL; // "EEP3"
    static constexpr uint32_t kAreaPrepared = 0x50524550UL; // "PREP"
    static constexpr uint32_t kAreaTransfer = 0x434F5059UL; // "COPY"
    static constexpr uint32_t kAreaActive = 0x41435456UL;    // "ACTV"
    static constexpr uint32_t kNullId = 0xFFFFFFFFUL;
    static constexpr uint32_t kResetId = 0xFFFFFFFEUL;
    static constexpr uint32_t kDeletedLength = 0xFFFFFFFFUL;
    static constexpr uint32_t kTombstoneChecksum = 0x44454C45UL; // "DELE"
    static constexpr uint32_t kResetChecksum = 0x52455345UL;     // "RESE"
    static constexpr uint32_t kAreaSize = EEPROM_EMULATOR_FLASH_AREA_SIZE;
    static constexpr uint32_t kAreaHeaderSize = sizeof(AreaHeader);
    static constexpr uint32_t kEntryStackSize =
        sizeof(Entry) * EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES;
    static constexpr uint32_t kDataStackSize =
        kAreaSize - kAreaHeaderSize - kEntryStackSize;

    static_assert(kAreaHeaderSize + kEntryStackSize < kAreaSize,
                  "EEPROM metadata must fit inside each flash area");

    FlashArea m_flashAreas[kAreaCount];

    uint8_t  m_activeArea;
    uint16_t m_entryStackPtr;
    uint32_t m_dataStackPtr;

    bool format();
    bool compactify();
    bool appendObject(uint8_t areaIndex,
                      uint16_t& entryStackPtr,
                      uint32_t& dataStackPtr,
                      uint16_t objectId,
                      const uint8_t *data,
                      uint32_t length);
    bool appendTombstone(uint8_t areaIndex,
                         uint16_t& entryStackPtr,
                         uint32_t dataStackPtr,
                         uint16_t objectId);
    bool appendResetMarker(uint8_t areaIndex,
                           uint16_t& entryStackPtr,
                           uint32_t dataStackPtr);
    bool appendMarker(uint8_t areaIndex,
                      uint16_t& entryStackPtr,
                      uint32_t dataStackPtr,
                      const Entry& entry);

    void initializeStackPointers();
    bool checkWriteAvailability(uint16_t entryStackPtr,
                                uint32_t dataStackPtr,
                                uint32_t dataLength) const;

    uint32_t getPaddedLength(uint32_t length) const;
    uint32_t calculateChecksum(const uint8_t *data, uint32_t length) const;

    uint32_t entryStackAddress(uint8_t areaIndex) const;
    uint32_t dataStackAddress(uint8_t areaIndex) const;
    uint32_t entryAddress(uint8_t areaIndex, uint16_t entryIndex) const;

    AreaHeader readAreaHeader(uint8_t areaIndex) const;
    bool isAreaActive(uint8_t areaIndex, AreaHeader *header = nullptr) const;
    bool isAreaPrepared(uint8_t areaIndex, uint32_t generation) const;
    bool isGenerationNewer(uint32_t lhs, uint32_t rhs) const;
    bool prepareArea(uint8_t areaIndex, uint32_t generation);
    bool ensureAreaPrepared(uint8_t areaIndex, uint32_t generation);
    bool ensureStandbyAreaPrepared();
    bool beginAreaTransfer(uint8_t areaIndex);
    bool activateArea(uint8_t areaIndex);

    bool findObjectEntry(uint8_t areaIndex,
                         uint16_t entryCount,
                         uint16_t objectId,
                         Entry *entry) const;
    bool hasObjectRecord(uint8_t areaIndex,
                         uint16_t entryCount,
                         uint16_t objectId) const;
    bool hasNewerObjectRecord(uint16_t entryIndex, uint16_t objectId) const;
    bool isEntryErased(const Entry& entry) const;
    bool hasValidMetadata(const Entry& entry) const;
    bool isTombstone(const Entry& entry) const;
    bool isResetMarker(const Entry& entry) const;
    bool isEntryCommitted(uint8_t areaIndex, const Entry& entry) const;
    void readObjectEntry(uint8_t areaIndex, uint16_t entryIndex, Entry *entry) const;
    bool reserveObjectEntry(uint8_t areaIndex, uint16_t entryIndex, const Entry& entry);
    bool commitObjectEntry(uint8_t areaIndex, uint16_t entryIndex, const Entry& entry);

    bool flashWrite(uint32_t address, const uint8_t *data, uint32_t length);
    bool eraseArea(uint8_t areaIndex);
};
