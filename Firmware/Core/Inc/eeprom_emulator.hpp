#pragma once

#include "stm32f4xx_hal.h"
#include "configuration.h"
#include <cstdint>

// ---------------------------------------------------------------------------
// EepromEmulator
//
// Log-structured EEPROM emulation on a single STM32F446RE flash sector.
//
// Sector layout (128 KB, Sector 7 — 0x08060000):
//   [+0x0000–+0x0001]  Sector status (uint16_t: 0xFFFF=empty, 0xAAAA=active)
//   [+0x0002–+0x0007]  Reserved (alignment padding)
//   [+0x0008–+0x3FFF]  Entry stack  (≤ 2048 entries × 8 B = 16 KB)
//   [+0x4000–end    ]  Data  stack  (≈ 112 KB)
//
// Objects are registered with registerObject() before init(). Each write
// appends one Entry {id, dataOffset} to the entry stack and the raw payload
// to the data stack. Reads scan the entry stack backwards for the latest entry.
//
// When either stack is nearly full, saveObject() triggers compactification:
// all registered modules are notified first (so they can decide whether to
// keep their current RAM values or reload from flash), then the sector is
// erased and each object is re-written from its registered RAM buffer.
//
// Power-loss note: a reset during compactification loses all stored data
// (accepted trade-off for single-sector operation).
// ---------------------------------------------------------------------------
class EepromEmulator {
public:
    EepromEmulator();
    
    using CompactificationNotificationCallback = void (*)(void *context);

    bool init();
    bool registerObject(uint16_t objectId, uint8_t *memoryBuffer, uint16_t length, 
        CompactificationNotificationCallback compactificationNotificationCallback, void *context);
    bool saveObject(uint16_t objectId);
    bool loadObject(uint16_t objectId);
    bool reset();

private:
    struct Entry {
        uint32_t id;          // object identifier (uint32_t to pad struct to 8 bytes)
        uint32_t dataOffset;  // byte offset from the data stack base
    };

    // Sector status codes (written to the first 2 bytes of the sector).
    static constexpr uint16_t kSectorEmpty  = 0xFFFFU;  // erased flash default
    static constexpr uint16_t kSectorActive = 0xAAAAU;

    // Sentinels — erased flash reads as 0xFF bytes.
    // kNullId (0xFFFFFFFF) is detectable in Entry.id as "slot never written".
    // kNullIdx (0xFFFF) is returned by getObjectIdx when an ID is not registered.
    static constexpr uint32_t kNullId  = 0xFFFFFFFFU;
    static constexpr uint16_t kNullIdx = 0xFFFFU;

    // Configurable limits (defined in configuration.h)
    static constexpr uint32_t kSector      = EEPROM_EMULATOR_FLASH_SECTOR;
    static constexpr uint32_t kSectorAddr  = EEPROM_EMULATOR_FLASH_SECTOR_ADDR;
    static constexpr uint32_t kSectorSize  = EEPROM_EMULATOR_FLASH_SECTOR_SIZE;

    // Sector layout constants
    static constexpr uint32_t kSectorHdrSize    = 8U;
    static constexpr uint32_t kEntryStackSize   = sizeof(Entry) * EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES;
    static constexpr uint32_t kDataStackSize    = kSectorSize - kSectorHdrSize - kEntryStackSize;
    static constexpr uint32_t kEntryStackAddr   = kSectorAddr + kSectorHdrSize;
    static constexpr uint32_t kDataStackAddr    = kEntryStackAddr + kEntryStackSize;

    // Runtime state
    uint16_t m_objectIds[EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS];
    CompactificationNotificationCallback m_compactificationCallbacks[EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS];
    void *m_contexts[EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS];
    uint8_t *m_objectMemoryBuffers[EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS];
    uint16_t m_objectLengths[EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS];
    uint8_t m_numOfObjects;

    uint16_t m_entryStackPtr;                  // next free entry stack slot
    uint32_t m_dataStackPtr;                   // next free byte offset in the data stack
    
    // Helpers.
    bool compactify();
    
    void sendCompactificationNotification();
    void initializeStackPointers();

    bool checkWriteAvailability(uint16_t dataLength);
    uint16_t readSectorHeader();
    bool findObjectEntry(uint16_t objectId, Entry *entry);
    
    bool writeObjectData(Entry &entry, uint8_t *data, uint16_t length);
    void readObjectData(Entry &entry, uint8_t *data, uint16_t length);

    bool writeObjectEntry(uint16_t idx, Entry *entry);
    void readObjectEntry(uint16_t idx, Entry *entry);
    
    uint16_t getObjectIdx(uint16_t objectId);
    uint32_t getPaddedLength(uint32_t length);

    // Flash primitives
    bool flashWrite(uint32_t addr, uint8_t *data, uint16_t length);
    bool eraseSector();
    bool writeSectorHeader();
};
