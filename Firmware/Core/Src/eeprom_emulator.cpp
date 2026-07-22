#include "eeprom_emulator.hpp"
#include <cstddef>
#include <cstring>

namespace {

#if EEPROM_RESET_ON_NEW_BUILD

struct EepromBuildResetRecord {
    uint32_t magic;
    uint32_t buildSignature;
};

// Reserved for EEPROM-emulator metadata and disjoint from configuration
// records, which begin at 0x0200.
constexpr uint16_t kBuildResetRecordObjectId = 0x0001U;
constexpr uint32_t kBuildResetRecordMagic = 0x42525331UL; // "BRS1"

constexpr uint32_t hashBuildTimestamp(const char *text)
{
    // FNV-1a keeps the persistent marker compact while remaining deterministic
    // for every copy of the same firmware image.
    uint32_t hash = 2166136261UL;
    while (*text != '\0') {
        hash ^= static_cast<uint8_t>(*text++);
        hash *= 16777619UL;
    }
    return hash;
}

constexpr uint32_t kBuildSignature =
    hashBuildTimestamp(__DATE__ " " __TIME__);

#endif

} // namespace

EepromEmulator::EepromEmulator()
    : m_flashAreas{
          {EEPROM_EMULATOR_FLASH_AREA_A_SECTOR, EEPROM_EMULATOR_FLASH_AREA_A_ADDR},
          {EEPROM_EMULATOR_FLASH_AREA_B_SECTOR, EEPROM_EMULATOR_FLASH_AREA_B_ADDR}}
    , m_activeArea(kNoArea)
    , m_entryStackPtr(0U)
    , m_dataStackPtr(0U)
{
}

bool EepromEmulator::init()
{
    AreaHeader headerA;
    AreaHeader headerB;
    const bool areaAActive = isAreaActive(kAreaA, &headerA);
    const bool areaBActive = isAreaActive(kAreaB, &headerB);

    if (!areaAActive && !areaBActive) {
        if (!format()) {
            return false;
        }
    } else {
        if (areaAActive && areaBActive) {
            m_activeArea =
                isGenerationNewer(headerB.generation, headerA.generation)
                    ? kAreaB
                    : kAreaA;
        } else {
            m_activeArea = areaAActive ? kAreaA : kAreaB;
        }

        initializeStackPointers();
        if (!ensureStandbyAreaPrepared()) {
            return false;
        }
    }

#if EEPROM_RESET_ON_NEW_BUILD
    EepromBuildResetRecord storedRecord{};
    uint32_t storedLength = 0U;
    const bool alreadyResetForThisBuild =
        loadObject(kBuildResetRecordObjectId,
                   &storedRecord,
                   sizeof(storedRecord),
                   &storedLength) &&
        storedLength == sizeof(storedRecord) &&
        storedRecord.magic == kBuildResetRecordMagic &&
        storedRecord.buildSignature == kBuildSignature;

    if (!alreadyResetForThisBuild) {
        // Write the marker after the reset barrier. If power fails between
        // these operations, the next initialization safely repeats the reset.
        if (!logicalReset()) {
            return false;
        }
        const EepromBuildResetRecord currentRecord{
            kBuildResetRecordMagic,
            kBuildSignature
        };
        if (!saveObject(kBuildResetRecordObjectId,
                        &currentRecord,
                        sizeof(currentRecord))) {
            return false;
        }
    }
#endif

    return true;
}

bool EepromEmulator::saveObject(uint16_t objectId,
                                const void *data,
                                uint32_t length)
{
    if (m_activeArea == kNoArea || (length > 0U && data == nullptr)) {
        return false;
    }

    if (!checkWriteAvailability(m_entryStackPtr, m_dataStackPtr, length)) {
        if (!compactify() ||
            !checkWriteAvailability(m_entryStackPtr, m_dataStackPtr, length)) {
            return false;
        }
    }

    if (!appendObject(m_activeArea,
                      m_entryStackPtr,
                      m_dataStackPtr,
                      objectId,
                      static_cast<const uint8_t *>(data),
                      length)) {
        // A failed flash operation may have consumed part of a record. Rescan
        // so a later save never attempts to program that location again.
        initializeStackPointers();
        return false;
    }

    return true;
}

bool EepromEmulator::loadObject(uint16_t objectId,
                                void *data,
                                uint32_t capacity,
                                uint32_t *length) const
{
    if (m_activeArea == kNoArea) {
        return false;
    }

    Entry entry;
    if (!findObjectEntry(m_activeArea, m_entryStackPtr, objectId, &entry) ||
        entry.length > capacity ||
        (entry.length > 0U && data == nullptr)) {
        return false;
    }

    if (entry.length > 0U) {
        std::memcpy(data,
                    reinterpret_cast<const void *>(dataStackAddress(m_activeArea) +
                                                   entry.dataOffset),
                    entry.length);
    }
    if (length != nullptr) {
        *length = entry.length;
    }
    return true;
}

bool EepromEmulator::getObjectInfo(uint16_t objectId, ObjectInfo& info) const
{
    if (m_activeArea == kNoArea) {
        return false;
    }

    Entry entry;
    if (!findObjectEntry(m_activeArea, m_entryStackPtr, objectId, &entry)) {
        return false;
    }

    info.id = objectId;
    info.length = entry.length;
    return true;
}

EepromEmulator::ObjectCursor EepromEmulator::beginObjectEnumeration() const
{
    return {static_cast<int32_t>(m_entryStackPtr) - 1};
}

bool EepromEmulator::getNextObject(ObjectCursor& cursor, ObjectInfo& info) const
{
    while (cursor.nextEntryIndex >= 0) {
        const uint16_t entryIndex = static_cast<uint16_t>(cursor.nextEntryIndex--);
        Entry entry;
        readObjectEntry(m_activeArea, entryIndex, &entry);

        if (isResetMarker(entry)) {
            cursor.nextEntryIndex = -1;
            return false;
        }
        if (!isEntryCommitted(m_activeArea, entry) ||
            isTombstone(entry) ||
            entry.id > 0xFFFFU) {
            continue;
        }

        const uint16_t objectId = static_cast<uint16_t>(entry.id);
        if (hasNewerObjectRecord(entryIndex, objectId)) {
            continue;
        }

        info.id = objectId;
        info.length = entry.length;
        return true;
    }
    return false;
}

bool EepromEmulator::logicalErase(uint16_t objectId)
{
    if (m_activeArea == kNoArea) {
        return false;
    }

    if (!checkWriteAvailability(m_entryStackPtr, m_dataStackPtr, 0U)) {
        if (!compactify() ||
            !checkWriteAvailability(m_entryStackPtr, m_dataStackPtr, 0U)) {
            return false;
        }
    }

    if (!appendTombstone(m_activeArea,
                         m_entryStackPtr,
                         m_dataStackPtr,
                         objectId)) {
        initializeStackPointers();
        return false;
    }
    return true;
}

bool EepromEmulator::logicalReset()
{
    if (m_activeArea == kNoArea) {
        return format();
    }

    if (!checkWriteAvailability(m_entryStackPtr, m_dataStackPtr, 0U)) {
        // There is no need to copy objects that the reset will hide. Start an
        // empty generation directly, using the normal atomic area handover.
        return hardReset();
    }

    if (!appendResetMarker(m_activeArea, m_entryStackPtr, m_dataStackPtr)) {
        initializeStackPointers();
        return false;
    }
    return true;
}

bool EepromEmulator::format()
{
    // Physical formatting is needed only when neither area contains a valid
    // generation, such as first boot with erased or legacy-format flash.
    if (!prepareArea(kAreaA, 1U) || !activateArea(kAreaA)) {
        return false;
    }

    m_activeArea = kAreaA;
    m_entryStackPtr = 0U;
    m_dataStackPtr = 0U;
    return prepareArea(kAreaB, 2U);
}

bool EepromEmulator::compactify()
{
    if (m_activeArea == kNoArea) {
        return false;
    }

    const uint8_t sourceArea = m_activeArea;
    const uint8_t destinationArea = sourceArea == kAreaA ? kAreaB : kAreaA;
    const AreaHeader sourceHeader = readAreaHeader(sourceArea);
    const uint32_t nextGeneration = sourceHeader.generation + 1U;

    if (!ensureAreaPrepared(destinationArea, nextGeneration) ||
        !beginAreaTransfer(destinationArea)) {
        return false;
    }

    uint16_t destinationEntryPtr = 0U;
    uint32_t destinationDataPtr = 0U;

    // Scan newest-to-oldest. The first committed occurrence of an ID is its
    // latest version. Already-copied IDs are skipped, requiring no RAM cache.
    for (int32_t i = static_cast<int32_t>(m_entryStackPtr) - 1; i >= 0; --i) {
        Entry sourceEntry;
        readObjectEntry(sourceArea, static_cast<uint16_t>(i), &sourceEntry);
        if (isResetMarker(sourceEntry)) {
            break;
        }
        if (!isEntryCommitted(sourceArea, sourceEntry)) {
            continue;
        }

        const uint16_t objectId = static_cast<uint16_t>(sourceEntry.id);
        if (hasObjectRecord(destinationArea, destinationEntryPtr, objectId)) {
            continue;
        }

        if (isTombstone(sourceEntry)) {
            if (!appendTombstone(destinationArea,
                                 destinationEntryPtr,
                                 destinationDataPtr,
                                 objectId)) {
                return false;
            }
        } else {
            const uint8_t *sourceData = reinterpret_cast<const uint8_t *>(
                dataStackAddress(sourceArea) + sourceEntry.dataOffset);
            if (!appendObject(destinationArea,
                              destinationEntryPtr,
                              destinationDataPtr,
                              objectId,
                              sourceData,
                              sourceEntry.length)) {
                return false;
            }
        }
    }

    // This single active-marker update is the commit point. Until it succeeds,
    // the source is the only active generation.
    if (!activateArea(destinationArea)) {
        return false;
    }

    m_activeArea = destinationArea;
    m_entryStackPtr = destinationEntryPtr;
    m_dataStackPtr = destinationDataPtr;

    // Failure to prepare the next standby is harmless to the committed data.
    // Initialization or the next rollover will retry before using that area.
    (void)prepareArea(sourceArea, nextGeneration + 1U);
    return true;
}

bool EepromEmulator::hardReset()
{
    if (m_activeArea == kNoArea) {
        return format();
    }

    const uint8_t sourceArea = m_activeArea;
    const uint8_t destinationArea = sourceArea == kAreaA ? kAreaB : kAreaA;
    const AreaHeader sourceHeader = readAreaHeader(sourceArea);

    const uint32_t nextGeneration = sourceHeader.generation + 1U;
    if (!ensureAreaPrepared(destinationArea, nextGeneration) ||
        !activateArea(destinationArea)) {
        return false;
    }

    m_activeArea = destinationArea;
    m_entryStackPtr = 0U;
    m_dataStackPtr = 0U;
    return prepareArea(sourceArea, nextGeneration + 1U);
}

bool EepromEmulator::appendObject(uint8_t areaIndex,
                                  uint16_t& entryStackPtr,
                                  uint32_t& dataStackPtr,
                                  uint16_t objectId,
                                  const uint8_t *data,
                                  uint32_t length)
{
    if ((length > 0U && data == nullptr) ||
        !checkWriteAvailability(entryStackPtr, dataStackPtr, length)) {
        return false;
    }

    Entry entry = {
        static_cast<uint32_t>(objectId),
        dataStackPtr,
        length,
        calculateChecksum(data, length)};

    // Reserve offset and length before touching payload flash. After a reset,
    // an interrupted record can therefore be skipped safely.
    if (!reserveObjectEntry(areaIndex, entryStackPtr, entry)) {
        return false;
    }
    if (length > 0U &&
        !flashWrite(dataStackAddress(areaIndex) + entry.dataOffset, data, length)) {
        return false;
    }
    if (!commitObjectEntry(areaIndex, entryStackPtr, entry)) {
        return false;
    }

    ++entryStackPtr;
    dataStackPtr += getPaddedLength(length);
    return true;
}

bool EepromEmulator::appendTombstone(uint8_t areaIndex,
                                     uint16_t& entryStackPtr,
                                     uint32_t dataStackPtr,
                                     uint16_t objectId)
{
    const Entry entry = {
        static_cast<uint32_t>(objectId),
        dataStackPtr,
        kDeletedLength,
        kTombstoneChecksum};
    return appendMarker(areaIndex, entryStackPtr, dataStackPtr, entry);
}

bool EepromEmulator::appendResetMarker(uint8_t areaIndex,
                                       uint16_t& entryStackPtr,
                                       uint32_t dataStackPtr)
{
    const Entry entry = {kResetId, dataStackPtr, 0U, kResetChecksum};
    return appendMarker(areaIndex, entryStackPtr, dataStackPtr, entry);
}

bool EepromEmulator::appendMarker(uint8_t areaIndex,
                                  uint16_t& entryStackPtr,
                                  uint32_t dataStackPtr,
                                  const Entry& entry)
{
    if (!checkWriteAvailability(entryStackPtr, dataStackPtr, 0U) ||
        !reserveObjectEntry(areaIndex, entryStackPtr, entry) ||
        !commitObjectEntry(areaIndex, entryStackPtr, entry)) {
        return false;
    }

    ++entryStackPtr;
    return true;
}

void EepromEmulator::initializeStackPointers()
{
    m_entryStackPtr = 0U;
    m_dataStackPtr = 0U;
    if (m_activeArea == kNoArea) {
        return;
    }

    for (uint16_t i = 0U; i < EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES; ++i) {
        Entry entry;
        readObjectEntry(m_activeArea, i, &entry);
        if (isEntryErased(entry)) {
            break;
        }

        // Dirty/incomplete entries consume their physical entry slot. If
        // offset and length were committed, reserve their data extent too.
        m_entryStackPtr = i + 1U;
        if (hasValidMetadata(entry)) {
            const uint32_t end = entry.dataOffset + getPaddedLength(entry.length);
            if (end > m_dataStackPtr) {
                m_dataStackPtr = end;
            }
        }
    }
}

bool EepromEmulator::checkWriteAvailability(uint16_t entryStackPtr,
                                            uint32_t dataStackPtr,
                                            uint32_t dataLength) const
{
    if (entryStackPtr >= EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES ||
        dataStackPtr > kDataStackSize) {
        return false;
    }

    const uint32_t paddedLength = getPaddedLength(dataLength);
    return paddedLength <= (kDataStackSize - dataStackPtr);
}

uint32_t EepromEmulator::getPaddedLength(uint32_t length) const
{
    return (length + 7U) & ~7U;
}

uint32_t EepromEmulator::calculateChecksum(const uint8_t *data, uint32_t length) const
{
    uint32_t checksum = 2166136261UL;
    for (uint32_t i = 0U; i < length; ++i) {
        checksum ^= data[i];
        checksum *= 16777619UL;
    }
    return checksum;
}

uint32_t EepromEmulator::entryStackAddress(uint8_t areaIndex) const
{
    return m_flashAreas[areaIndex].address + kAreaHeaderSize;
}

uint32_t EepromEmulator::dataStackAddress(uint8_t areaIndex) const
{
    return entryStackAddress(areaIndex) + kEntryStackSize;
}

uint32_t EepromEmulator::entryAddress(uint8_t areaIndex, uint16_t entryIndex) const
{
    return entryStackAddress(areaIndex) + entryIndex * sizeof(Entry);
}

EepromEmulator::AreaHeader EepromEmulator::readAreaHeader(uint8_t areaIndex) const
{
    AreaHeader header;
    std::memcpy(&header,
                reinterpret_cast<const void *>(m_flashAreas[areaIndex].address),
                sizeof(header));
    return header;
}

bool EepromEmulator::isAreaActive(uint8_t areaIndex, AreaHeader *header) const
{
    const AreaHeader value = readAreaHeader(areaIndex);
    if (header != nullptr) {
        *header = value;
    }
    return value.magic == kAreaMagic &&
           value.generationInverse == ~value.generation &&
           value.preparedMarker == kAreaPrepared &&
           value.activeMarker == kAreaActive;
}

bool EepromEmulator::isAreaPrepared(uint8_t areaIndex, uint32_t generation) const
{
    const AreaHeader header = readAreaHeader(areaIndex);
    return header.magic == kAreaMagic &&
           header.generation == generation &&
           header.generationInverse == ~generation &&
           header.preparedMarker == kAreaPrepared &&
           header.transferMarker == 0xFFFFFFFFUL &&
           header.activeMarker == 0xFFFFFFFFUL;
}

bool EepromEmulator::isGenerationNewer(uint32_t lhs, uint32_t rhs) const
{
    return static_cast<int32_t>(lhs - rhs) > 0;
}

bool EepromEmulator::prepareArea(uint8_t areaIndex, uint32_t generation)
{
    if (!eraseArea(areaIndex)) {
        return false;
    }

    const uint32_t metadata[] = {
        kAreaMagic, generation, ~generation, kAreaPrepared};
    if (!flashWrite(m_flashAreas[areaIndex].address,
                    reinterpret_cast<const uint8_t *>(metadata),
                    sizeof(metadata))) {
        return false;
    }

    return true;
}

bool EepromEmulator::ensureAreaPrepared(uint8_t areaIndex, uint32_t generation)
{
    return isAreaPrepared(areaIndex, generation) ||
           prepareArea(areaIndex, generation);
}

bool EepromEmulator::ensureStandbyAreaPrepared()
{
    if (m_activeArea == kNoArea) {
        return false;
    }

    const uint8_t standbyArea = m_activeArea == kAreaA ? kAreaB : kAreaA;
    const AreaHeader activeHeader = readAreaHeader(m_activeArea);
    return ensureAreaPrepared(standbyArea, activeHeader.generation + 1U);
}

bool EepromEmulator::beginAreaTransfer(uint8_t areaIndex)
{
    return flashWrite(m_flashAreas[areaIndex].address +
                          static_cast<uint32_t>(offsetof(AreaHeader, transferMarker)),
                      reinterpret_cast<const uint8_t *>(&kAreaTransfer),
                      sizeof(kAreaTransfer));
}

bool EepromEmulator::activateArea(uint8_t areaIndex)
{
    return flashWrite(m_flashAreas[areaIndex].address +
                          static_cast<uint32_t>(offsetof(AreaHeader, activeMarker)),
                      reinterpret_cast<const uint8_t *>(&kAreaActive),
                      sizeof(kAreaActive));
}

bool EepromEmulator::findObjectEntry(uint8_t areaIndex,
                                     uint16_t entryCount,
                                     uint16_t objectId,
                                     Entry *entry) const
{
    for (int32_t i = static_cast<int32_t>(entryCount) - 1; i >= 0; --i) {
        readObjectEntry(areaIndex, static_cast<uint16_t>(i), entry);
        if (isResetMarker(*entry)) {
            return false;
        }
        if (entry->id == static_cast<uint32_t>(objectId)) {
            if (isTombstone(*entry)) {
                return false;
            }
            if (isEntryCommitted(areaIndex, *entry)) {
                return true;
            }
        }
    }
    return false;
}

bool EepromEmulator::hasObjectRecord(uint8_t areaIndex,
                                     uint16_t entryCount,
                                     uint16_t objectId) const
{
    for (int32_t i = static_cast<int32_t>(entryCount) - 1; i >= 0; --i) {
        Entry entry;
        readObjectEntry(areaIndex, static_cast<uint16_t>(i), &entry);
        if (isResetMarker(entry)) {
            return false;
        }
        if (entry.id == static_cast<uint32_t>(objectId) &&
            isEntryCommitted(areaIndex, entry)) {
            return true;
        }
    }
    return false;
}

bool EepromEmulator::hasNewerObjectRecord(uint16_t entryIndex,
                                          uint16_t objectId) const
{
    for (uint16_t i = static_cast<uint16_t>(entryIndex + 1U);
         i < m_entryStackPtr;
         ++i) {
        Entry entry;
        readObjectEntry(m_activeArea, i, &entry);
        if (isResetMarker(entry)) {
            return true;
        }
        if (entry.id == static_cast<uint32_t>(objectId) &&
            isEntryCommitted(m_activeArea, entry)) {
            return true;
        }
    }
    return false;
}

bool EepromEmulator::isEntryErased(const Entry& entry) const
{
    return entry.id == 0xFFFFFFFFUL &&
           entry.dataOffset == 0xFFFFFFFFUL &&
           entry.length == 0xFFFFFFFFUL &&
           entry.checksum == 0xFFFFFFFFUL;
}

bool EepromEmulator::hasValidMetadata(const Entry& entry) const
{
    if ((entry.dataOffset & 7U) != 0U || entry.dataOffset > kDataStackSize) {
        return false;
    }
    const uint32_t paddedLength = getPaddedLength(entry.length);
    return paddedLength >= entry.length &&
           paddedLength <= (kDataStackSize - entry.dataOffset);
}

bool EepromEmulator::isTombstone(const Entry& entry) const
{
    return entry.id <= 0xFFFFU &&
           entry.length == kDeletedLength &&
           entry.checksum == kTombstoneChecksum &&
           (entry.dataOffset & 7U) == 0U &&
           entry.dataOffset <= kDataStackSize;
}

bool EepromEmulator::isResetMarker(const Entry& entry) const
{
    return entry.id == kResetId &&
           entry.length == 0U &&
           entry.checksum == kResetChecksum &&
           (entry.dataOffset & 7U) == 0U &&
           entry.dataOffset <= kDataStackSize;
}

bool EepromEmulator::isEntryCommitted(uint8_t areaIndex, const Entry& entry) const
{
    if (isResetMarker(entry) || isTombstone(entry)) {
        return true;
    }
    if (entry.id > 0xFFFFU || !hasValidMetadata(entry)) {
        return false;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(
        dataStackAddress(areaIndex) + entry.dataOffset);
    return calculateChecksum(data, entry.length) == entry.checksum;
}

void EepromEmulator::readObjectEntry(uint8_t areaIndex,
                                     uint16_t entryIndex,
                                     Entry *entry) const
{
    std::memcpy(entry,
                reinterpret_cast<const void *>(entryAddress(areaIndex, entryIndex)),
                sizeof(*entry));
}

bool EepromEmulator::reserveObjectEntry(uint8_t areaIndex,
                                        uint16_t entryIndex,
                                        const Entry& entry)
{
    const uint32_t metadata[] = {entry.dataOffset, entry.length};
    return flashWrite(entryAddress(areaIndex, entryIndex) + sizeof(entry.id),
                      reinterpret_cast<const uint8_t *>(metadata),
                      sizeof(metadata));
}

bool EepromEmulator::commitObjectEntry(uint8_t areaIndex,
                                       uint16_t entryIndex,
                                       const Entry& entry)
{
    const uint32_t address = entryAddress(areaIndex, entryIndex);
    if (!flashWrite(address + static_cast<uint32_t>(offsetof(Entry, checksum)),
                    reinterpret_cast<const uint8_t *>(&entry.checksum),
                    sizeof(entry.checksum))) {
        return false;
    }
    return flashWrite(address + static_cast<uint32_t>(offsetof(Entry, id)),
                      reinterpret_cast<const uint8_t *>(&entry.id),
                      sizeof(entry.id));
}

bool EepromEmulator::flashWrite(uint32_t address,
                                const uint8_t *data,
                                uint32_t length)
{
    if ((address & 3U) != 0U || (length > 0U && data == nullptr)) {
        return false;
    }

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_OK;

    for (uint32_t offset = 0U; offset < length && status == HAL_OK; offset += 4U) {
        uint32_t word = 0xFFFFFFFFUL;
        const uint32_t remaining = length - offset;
        const uint32_t chunk = remaining < sizeof(word) ? remaining : sizeof(word);
        std::memcpy(&word, data + offset, chunk);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word);
    }

    HAL_FLASH_Lock();
    return status == HAL_OK;
}

bool EepromEmulator::eraseArea(uint8_t areaIndex)
{
    FLASH_EraseInitTypeDef eraseInit = {};
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = m_flashAreas[areaIndex].sector;
    eraseInit.NbSectors = 1U;

    uint32_t sectorError = 0U;
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();
    return status == HAL_OK;
}
