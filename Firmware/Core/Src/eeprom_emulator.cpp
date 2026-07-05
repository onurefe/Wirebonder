#include "eeprom_emulator.hpp"
#include <cstring>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

EepromEmulator::EepromEmulator()
    : m_numOfObjects(0U)
    , m_entryStackPtr(0U)
    , m_dataStackPtr(0U)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool EepromEmulator::init()
{
    if (readSectorHeader() == kSectorActive) {
        initializeStackPointers();
        return true;
    } else {
        return reset();
    }
}

bool EepromEmulator::registerObject(uint16_t objectId, uint8_t *memoryBuffer, uint16_t length, 
    CompactificationNotificationCallback compactificationCallback, void *context)
{
    if (m_numOfObjects < EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS) {
        m_objectIds[m_numOfObjects] = objectId;
        m_compactificationCallbacks[m_numOfObjects] = compactificationCallback;
        m_contexts[m_numOfObjects] = context;
        m_objectMemoryBuffers[m_numOfObjects] = memoryBuffer;
        m_objectLengths[m_numOfObjects] = length;
        m_numOfObjects++;
        
        return true;
    }
    
    return false;
}

bool EepromEmulator::saveObject(uint16_t objectId) 
{
    uint16_t object_idx = getObjectIdx(objectId);
    if (object_idx == kNullIdx) {
        return false;
    }

    if (!checkWriteAvailability(m_objectLengths[object_idx])) {
        sendCompactificationNotification();
        if (!compactify()) return false;
    }

    Entry entry = { objectId, m_dataStackPtr};
    if (!writeObjectEntry(m_entryStackPtr, &entry)) {
        return false;
    }

    if (!writeObjectData(entry, m_objectMemoryBuffers[object_idx], m_objectLengths[object_idx])) {
        return false;
    }

    m_entryStackPtr++;
    m_dataStackPtr += getPaddedLength(m_objectLengths[object_idx]);

    return true;
}

bool EepromEmulator::loadObject(uint16_t objectId)
{
    uint16_t object_idx = getObjectIdx(objectId);
    if (object_idx == kNullIdx) {
        return false;
    }

    Entry entry;
    if (!findObjectEntry(objectId, &entry)) {
        return false;
    }

    readObjectData(entry, m_objectMemoryBuffers[object_idx], m_objectLengths[object_idx]);
    return true;
}

void EepromEmulator::sendCompactificationNotification()
{
    for (uint8_t i = 0; i < m_numOfObjects; i++) {
        if (m_compactificationCallbacks[i]) {
            m_compactificationCallbacks[i](m_contexts[i]);
        }
    }
}

bool EepromEmulator::reset()
{
    if (!eraseSector()) return false;
    if (!writeSectorHeader()) return false;

    m_entryStackPtr = 0U;
    m_dataStackPtr  = 0U;

    return true;
}

// ---------------------------------------------------------------------------
// Private-Helpers
// ---------------------------------------------------------------------------
bool EepromEmulator::compactify()
{
    if (!reset()) {
        return false;
    }

    for (uint8_t i = 0; i < m_numOfObjects; i++) {
        Entry entry = {m_objectIds[i], m_dataStackPtr};
        if (!checkWriteAvailability(m_objectLengths[i])) {
            return false;
        }

        if (!writeObjectEntry(m_entryStackPtr, &entry)) {
            return false;
        }

        if (!writeObjectData(entry, m_objectMemoryBuffers[i], m_objectLengths[i])) {
            return false;
        }

        m_entryStackPtr++;
        m_dataStackPtr += getPaddedLength(m_objectLengths[i]);
    }

    return true;
}

uint16_t EepromEmulator::getObjectIdx(uint16_t objectId)
{
    for (uint16_t i = 0; i < m_numOfObjects; i++) {
        if (m_objectIds[i] == objectId) {
            return i;
        }
    }

    return kNullIdx;
}

bool EepromEmulator::checkWriteAvailability(uint16_t dataLength)
{
    uint32_t paddedLength = getPaddedLength(dataLength);
    bool entry_memory_full = (m_entryStackPtr >= EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES - 1U);
    bool data_memory_full  = (m_dataStackPtr + paddedLength > kDataStackSize);

    return !(entry_memory_full || data_memory_full);
}

void EepromEmulator::initializeStackPointers()
{
    m_entryStackPtr = 0U;
    m_dataStackPtr  = 0U;

    for (uint16_t i = 0U; i < EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES; ++i) {
        Entry entry;
        readObjectEntry(i, &entry);

        if (entry.id == kNullId) break;
        
        m_entryStackPtr = i + 1U;  // always advance — the slot is physically occupied in flash

        uint16_t obj_idx = getObjectIdx(static_cast<uint16_t>(entry.id));
        if (obj_idx == kNullIdx) continue;  // length unknown, can't update data pointer

        m_dataStackPtr = entry.dataOffset + getPaddedLength(m_objectLengths[obj_idx]);
    }
}

uint16_t EepromEmulator::readSectorHeader()
{
    uint16_t header;
    std::memcpy(&header, reinterpret_cast<const void *>(kSectorAddr), sizeof(header));
    return header;
}

bool EepromEmulator::findObjectEntry(uint16_t objectId, Entry *entry)
{
    for (int16_t i = static_cast<int16_t>(m_entryStackPtr) - 1; i >= 0; i--) {
        readObjectEntry(static_cast<uint16_t>(i), entry);

        if (entry->id == static_cast<uint32_t>(objectId)) {
            return true;
        }
    }

    return false;
}

bool EepromEmulator::writeObjectData(Entry &entry, uint8_t *data, uint16_t length)
{
    if (length > 0U) {
        uint32_t dataAddr = kDataStackAddr + entry.dataOffset;
        return flashWrite(dataAddr, data, length);
    } else {
        return true;
    }
}

void EepromEmulator::readObjectData(Entry &entry, uint8_t *data, uint16_t length)
{
    std::memcpy(data, 
                reinterpret_cast<const void *>(kDataStackAddr + entry.dataOffset),
                length);
}

bool EepromEmulator::writeObjectEntry(uint16_t idx, Entry *entry)
{
    uint32_t entryAddr = kEntryStackAddr + idx * sizeof(Entry);
    return flashWrite(entryAddr, reinterpret_cast<uint8_t *>(entry), sizeof(Entry));
}

void EepromEmulator::readObjectEntry(uint16_t idx, Entry *entry)
{
    std::memcpy(entry,
                reinterpret_cast<const void *>(kEntryStackAddr + idx * sizeof(Entry)),
                sizeof(Entry));
}

// ---------------------------------------------------------------------------
// Flash primitives — private
// ---------------------------------------------------------------------------
bool EepromEmulator::flashWrite(uint32_t addr, uint8_t *data, uint16_t length)
{
    // addr must be 4-byte aligned; length must be a multiple of 4.
    // Both are guaranteed: Entry is 8 B (two uint32_t fields), payloads are padded
    // via getPaddedLength(), and writeSectorHeader() passes a word-sized buffer.
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;

    for (uint16_t i = 0U; i < length && status == HAL_OK; i += 4U) {
        uint32_t word;
        std::memcpy(&word, data + i, sizeof(word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word);
    }

    HAL_FLASH_Lock();
    return status == HAL_OK;
}

bool EepromEmulator::eraseSector()
{
    FLASH_EraseInitTypeDef eraseInit = {};
    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector       = kSector;
    eraseInit.NbSectors    = 1U;

    uint32_t sectorError = 0U;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status == HAL_OK;
}

bool EepromEmulator::writeSectorHeader()
{
    // Pad the 2-byte status to a full word; upper halfword stays 0xFFFF (erased, no change).
    uint32_t word = 0xFFFF0000U | static_cast<uint32_t>(kSectorActive);
    return flashWrite(kSectorAddr, reinterpret_cast<uint8_t *>(&word), sizeof(word));
}

uint32_t EepromEmulator::getPaddedLength(uint32_t length)
{
    return (length + 0b111U) & ~0b111U;
}
