#include "ui_module.hpp"
#include "configuration.h"
#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstring>
#include <cstddef>

static void fmtFloat(char *buffer, size_t bufferSize, float value);

// =============================================================================
// Static data
// =============================================================================

// clang-format off
const char *UiModule::kGroupNames[UiModule::kNumGroups] = {
    "Force", "Heights", "Positions", "Bonding", "Timing", "Scan"
};

const UiModule::ParamDescriptor UiModule::kParams[UiModule::kParamCount] = {
    //  name              unit    offset                                                          scale    min                              max                              grp  int?
    // Group 0: Force-coil currents
    {"Constant I", "A", offsetof(BonderConfig, forceCoilConstantCurrent), 1.0f, UI_MODULE_FORCE_CURRENT_MIN, UI_MODULE_FORCE_CURRENT_MAX, 0, false},
    {"Tracking I", "A", offsetof(BonderConfig, forceCoilTrackingCurrent), 1.0f, UI_MODULE_FORCE_CURRENT_MIN, UI_MODULE_FORCE_CURRENT_MAX, 0, false},
    {"Bond 1 I", "A", offsetof(BonderConfig, forceCoilFirstBondCurrent), 1.0f, UI_MODULE_FORCE_CURRENT_MIN, UI_MODULE_FORCE_CURRENT_MAX, 0, false},
    {"Bond 2 I", "A", offsetof(BonderConfig, forceCoilSecondBondCurrent), 1.0f, UI_MODULE_FORCE_CURRENT_MIN, UI_MODULE_FORCE_CURRENT_MAX, 0, false},

    // Group 1: Z-axis heights
    {"Reset Height", "mm", offsetof(BonderConfig, resetHeight), 1.0f, UI_MODULE_HEIGHT_MIN, UI_MODULE_HEIGHT_MAX, 1, false},
    {"Loop Height", "mm", offsetof(BonderConfig, loopHeight), 1.0f, UI_MODULE_HEIGHT_MIN, UI_MODULE_HEIGHT_MAX, 1, false},
    {"Search 1", "mm", offsetof(BonderConfig, firstSearchHeight), 1.0f, UI_MODULE_HEIGHT_MIN, UI_MODULE_HEIGHT_MAX, 1, false},
    {"Search 2", "mm", offsetof(BonderConfig, secondSearchHeight), 1.0f, UI_MODULE_HEIGHT_MIN, UI_MODULE_HEIGHT_MAX, 1, false},
    {"Kink Height", "mm", offsetof(BonderConfig, kinkHeight), 1.0f, UI_MODULE_KINK_HEIGHT_MIN, UI_MODULE_KINK_HEIGHT_MAX, 1, false},
    {"Overtravel", "mm", offsetof(BonderConfig, lowestOvertravel), 1.0f, UI_MODULE_OVERTRAVEL_MIN, UI_MODULE_OVERTRAVEL_MAX, 1, false},

    // Group 2: Y/T logical positions
    {"Tail Position", "mm", offsetof(BonderConfig, tailPosition), 1.0f, UI_MODULE_LARGE_DISPLACEMENT_MIN, UI_MODULE_LARGE_DISPLACEMENT_MAX, 2, false},
    {"Tear Position", "mm", offsetof(BonderConfig, tearPosition), 1.0f, UI_MODULE_LARGE_DISPLACEMENT_MIN, UI_MODULE_LARGE_DISPLACEMENT_MAX, 2, false},
    {"Y Reverse Pos", "mm", offsetof(BonderConfig, yReversePosition), 1.0f, UI_MODULE_SMALL_DISPLACEMENT_MIN, UI_MODULE_SMALL_DISPLACEMENT_MAX, 2, false},
    {"Y Stepback Pos", "mm", offsetof(BonderConfig, yStepbackPosition), 1.0f, UI_MODULE_SMALL_DISPLACEMENT_MIN, UI_MODULE_SMALL_DISPLACEMENT_MAX, 2, false},

    // Group 3: Ultrasonic bonding
    { "Bond 1 Power", "", offsetof(BonderConfig, firstBondingPower), 1.0f, UI_MODULE_TARGET_POWER_MIN, UI_MODULE_TARGET_POWER_MAX, 3, false},
    { "Bond 2 Power", "", offsetof(BonderConfig, secondBondingPower), 1.0f, UI_MODULE_TARGET_POWER_MIN, UI_MODULE_TARGET_POWER_MAX, 3, false},
    { "Bond 1 Energy", "J", offsetof(BonderConfig, firstBondingEnergy), 1.0f, UI_MODULE_BONDING_ENERGY_MIN, UI_MODULE_BONDING_ENERGY_MAX, 3, false},
    { "Bond 2 Energy", "J", offsetof(BonderConfig, secondBondingEnergy), 1.0f, UI_MODULE_BONDING_ENERGY_MIN, UI_MODULE_BONDING_ENERGY_MAX, 3, false},
    { "Max Duration", "s", offsetof(BonderConfig, maxBondingDuration), 1.0f, UI_MODULE_MAX_BONDING_DURATION_MIN, UI_MODULE_MAX_BONDING_DURATION_MAX, 3, false},

    // Group 4: Timing
    { "Contact Settle", "s", offsetof(BonderConfig, contactSettlingTime), 1.0f, UI_MODULE_TIMING_MIN, UI_MODULE_TIMING_MAX, 4, false},
    { "Cooling Time", "s", offsetof(BonderConfig, coolingTime), 1.0f, UI_MODULE_TIMING_MIN, UI_MODULE_TIMING_MAX, 4, false},
    { "Tail Delay", "s", offsetof(BonderConfig, tailRestoreDelay), 1.0f, UI_MODULE_TIMING_MIN, UI_MODULE_TIMING_MAX, 4, false},
    { "Y Delay", "s", offsetof(BonderConfig, yRestoreDelay), 1.0f, UI_MODULE_TIMING_MIN, UI_MODULE_TIMING_MAX, 4, false},

    // Group 5: Impedance scan
    { "Start Freq", "kHz", offsetof(BonderConfig, scanStartFrequency), 0.001f, UI_MODULE_SCAN_FREQ_MIN, UI_MODULE_SCAN_FREQ_MAX, 5, false},
    { "Stop Freq", "kHz", offsetof(BonderConfig, scanStopFrequency), 0.001f, UI_MODULE_SCAN_FREQ_MIN, UI_MODULE_SCAN_FREQ_MAX, 5, false},
    { "Num Freqs", "",    offsetof(BonderConfig, numOfScannedFrequencies), 1.0f, UI_MODULE_SCAN_NUM_FREQS_MIN, UI_MODULE_SCAN_NUM_FREQS_MAX, 5, true},
};
// clang-format on


// =============================================================================
// UiModule
// =============================================================================
UiModule::UiModule(LcdModule *lcd, const Buttons& btns)
    : m_lcd(lcd)
    , m_btns(btns)
    , m_config()
    , m_mode(Mode::NORMAL)
    , m_groupIndex(0U)
    , m_paramIndex(0U)
    , m_stepIndex(2U)           // default step = 0.1
    , m_displayOutOfDate(false)
    , m_statusClearTick(0U)
    , m_statusMsg{}
    , m_configChangedCallback(nullptr)
    , m_configChangedCtx(nullptr)
    , m_saveCallback(nullptr)
    , m_loadCallback(nullptr)
    , m_persistenceCtx(nullptr)
    , m_state(ServiceState::READY)
{
}

void UiModule::setConfigChangedListenerCallback(void *ctx, ConfigChangedCallback cb)
{
    m_configChangedCtx      = ctx;
    m_configChangedCallback = cb;
}

void UiModule::setPersistenceControllerCallbacks(void *ctx, SaveCallback save, LoadCallback load)
{
    m_persistenceCtx = ctx;
    m_saveCallback   = save;
    m_loadCallback   = load;
}

void UiModule::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    auto reg = [](ButtonChannel *ch, void *ctx, ButtonChannel::PressCallback cb) {
        if (ch) ch->addPressListenerCallback(ctx, cb);
    };

    reg(m_btns.up,          this, onUp_);
    reg(m_btns.down,        this, onDown_);
    reg(m_btns.left,        this, onLeft_);
    reg(m_btns.right,       this, onRight_);
    reg(m_btns.plus,        this, onPlus_);
    reg(m_btns.minus,       this, onMinus_);
    reg(m_btns.save,        this, onSave_);
    reg(m_btns.load,        this, onLoad_);
    reg(m_btns.tailPlus,    this, onTailPlus_);
    reg(m_btns.tailMinus,   this, onTailMinus_);
    reg(m_btns.loopPlus,    this, onLoopPlus_);
    reg(m_btns.loopMinus,   this, onLoopMinus_);
    reg(m_btns.searchPlus,  this, onSearchPlus_);
    reg(m_btns.searchMinus, this, onSearchMinus_);
    reg(m_btns.stepPlus,     this, onStepPlus_);
    reg(m_btns.stepMinus,    this, onStepMinus_);
    reg(m_btns.factoryReset, this, onFactoryReset_);
    reg(m_btns.enter,        this, onEnter_);

    m_displayOutOfDate = true;
    m_state = ServiceState::OPERATING;
}

void UiModule::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    m_displayOutOfDate = false;
    m_statusMsg[0] = '\0';
    m_mode = Mode::NORMAL;
    m_state = ServiceState::READY;
}

void UiModule::execute()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    // Expire timed status messages.
    if (m_statusMsg[0] != '\0') {
        if ((HAL_GetTick() - m_statusClearTick) >= 2000U) {
            m_statusMsg[0] = '\0';
            m_displayOutOfDate = true;
        }
    }

    // Refresh display only when the LCD has drained all previous commands.
    if (m_displayOutOfDate && m_lcd->isIdle()) {
        refreshDisplay();
        m_displayOutOfDate = false;
    }
}

// =============================================================================
// Display
// =============================================================================

void UiModule::refreshDisplay()
{
    char line[21];
    for (uint8_t row = 0U; row < 4U; ++row) {
        buildRow(line, row);
        m_lcd->setCursor(0U, row);
        m_lcd->printString(line);
    }
}

void UiModule::buildRow(char *line, uint8_t row) const
{
    if (m_mode == Mode::CONFIRMING_FACTORY_RESET) {
        buildConfirmRow(line, row);
        return;
    }

    switch (row) {
    case 0U:  buildGroupRow(line);     break;
    case 1U:  buildParamNameRow(line); break;
    case 2U:  buildValueRow(line);     break;
    default:  buildStatusRow(line);    break;
    }
}

void UiModule::buildConfirmRow(char *line, uint8_t row) const
{
    switch (row) {
    case 0U:  snprintf(line, 21U, "%-20s", "!! FACTORY RESET !!"); break;
    case 1U:  snprintf(line, 21U, "%-20s", "");                    break;
    case 2U:  snprintf(line, 21U, "%-20s", "ENTER: confirm");      break;
    default:  snprintf(line, 21U, "%-20s", "RESET: cancel");       break;
    }
}

void UiModule::buildGroupRow(char *line) const
{
    // "Heights              2/6"  (group name left, index right)
    snprintf(line, 21U, "%-17s%hhu/%hhu",
             kGroupNames[m_groupIndex],
             static_cast<unsigned char>(m_groupIndex + 1U),
             static_cast<unsigned char>(kNumGroups));
}

void UiModule::buildParamNameRow(char *line) const
{
    // ">Search Height      "
    const ParamDescriptor *d = currentDescriptor();
    snprintf(line, 21U, ">%-19s", d ? d->name : "");
}

void UiModule::buildValueRow(char *line) const
{
    // "  1.500 mm          "
    const ParamDescriptor *d = currentDescriptor();
    if (d == nullptr) {
        snprintf(line, 21U, "%-20s", "");
        return;
    }

    char valStr[12];
    const uint8_t *base = reinterpret_cast<const uint8_t*>(&m_config);
    if (d->isInteger) {
        uint16_t v;
        std::memcpy(&v, base + d->offset, sizeof(v));
        snprintf(valStr, sizeof(valStr), "%hu", static_cast<unsigned short>(v));
    } else {
        float v;
        std::memcpy(&v, base + d->offset, sizeof(v));
        fmtFloat(valStr, sizeof(valStr), v * d->scale);
    }
    snprintf(line, 21U, "  %-9.9s%-9.9s", valStr, d->unit);
}

void UiModule::buildStatusRow(char *line) const
{
    // "step:0.100          " or timed status message
    if (m_statusMsg[0] != '\0') {
        snprintf(line, 21U, "%-20s", m_statusMsg);
    } else {
        char stepStr[8];
        fmtFloat(stepStr, sizeof(stepStr), kStepSizes[m_stepIndex]);
        snprintf(line, 21U, "step:%-15s", stepStr);
    }
}

void UiModule::setStatus(const char *msg)
{
    snprintf(m_statusMsg, sizeof(m_statusMsg), "%s", msg);
    m_statusClearTick = HAL_GetTick();
    m_displayOutOfDate    = true;
}

// =============================================================================
// Navigation
// =============================================================================

uint8_t UiModule::groupParamCount(uint8_t group) const
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < kParamCount; ++i) {
        if (kParams[i].groupIndex == group) ++count;
    }
    return count;
}

const UiModule::ParamDescriptor *UiModule::currentDescriptor() const
{
    uint8_t idx = 0U;
    for (uint8_t i = 0U; i < kParamCount; ++i) {
        if (kParams[i].groupIndex != m_groupIndex) continue;
        if (idx == m_paramIndex) return &kParams[i];
        ++idx;
    }
    return nullptr;
}

const UiModule::ParamDescriptor *UiModule::descriptorByOffset(uint16_t offset) const
{
    for (uint8_t i = 0U; i < kParamCount; ++i) {
        if (kParams[i].offset == offset) return &kParams[i];
    }
    return nullptr;
}

void UiModule::jumpToParam(uint16_t offset)
{
    for (uint8_t i = 0U; i < kParamCount; ++i) {
        if (kParams[i].offset != offset) continue;
        m_groupIndex = kParams[i].groupIndex;
        m_paramIndex = 0U;
        for (uint8_t j = 0U; j < i; ++j) {
            if (kParams[j].groupIndex == m_groupIndex) ++m_paramIndex;
        }
        return;
    }
}

void UiModule::navigate(int paramDelta, int groupDelta)
{
    if (m_mode != Mode::NORMAL) return;

    if (groupDelta != 0) {
        m_groupIndex = static_cast<uint8_t>(
            (static_cast<int>(m_groupIndex) + kNumGroups + groupDelta) % kNumGroups);
        m_paramIndex = 0U;
    }
    if (paramDelta != 0) {
        uint8_t count = groupParamCount(m_groupIndex);
        if (count > 0U) {
            m_paramIndex = static_cast<uint8_t>(
                (static_cast<int>(m_paramIndex) + count + paramDelta) % count);
        }
    }
    m_displayOutOfDate = true;
}

// =============================================================================
// Value adjustment
// =============================================================================

void UiModule::adjustParam(uint16_t offset, float delta)
{
    const ParamDescriptor *d = descriptorByOffset(offset);
    if (d == nullptr) return;

    if (d->isInteger) adjustIntegerParam(d, delta);
    else              adjustFloatParam(d, delta);

    if (m_configChangedCallback) {
        m_configChangedCallback(m_configChangedCtx, m_config);
    }
    m_displayOutOfDate = true;
}

void UiModule::adjustIntegerParam(const ParamDescriptor *d, float delta)
{
    uint8_t  *base = reinterpret_cast<uint8_t*>(&m_config);
    uint16_t  current;
    std::memcpy(&current, base + d->offset, sizeof(current));

    float dv = static_cast<float>(current) + delta;
    if (dv < d->minDisplay) dv = d->minDisplay;
    if (dv > d->maxDisplay) dv = d->maxDisplay;

    current = static_cast<uint16_t>(dv + 0.5f);
    std::memcpy(base + d->offset, &current, sizeof(current));
}

void UiModule::adjustFloatParam(const ParamDescriptor *d, float delta)
{
    uint8_t *base = reinterpret_cast<uint8_t*>(&m_config);
    float    current;
    std::memcpy(&current, base + d->offset, sizeof(current));

    float dv = current * d->scale + delta;
    if (dv < d->minDisplay) dv = d->minDisplay;
    if (dv > d->maxDisplay) dv = d->maxDisplay;

    current = dv / d->scale;
    std::memcpy(base + d->offset, &current, sizeof(current));
}

void UiModule::adjustCurrent(float sign)
{
    if (m_mode != Mode::NORMAL) return;

    const ParamDescriptor *d = currentDescriptor();
    if (d) adjustParam(d->offset, sign * kStepSizes[m_stepIndex]);
}

void UiModule::adjustHotkey(uint16_t offset, float sign)
{
    jumpToParam(offset);
    adjustParam(offset, sign * kStepSizes[m_stepIndex]);
}

// =============================================================================
// Persistence
// =============================================================================

void UiModule::doSave()
{
    if (m_mode != Mode::NORMAL) return;

    if (m_saveCallback) m_saveCallback(m_persistenceCtx, m_config);
    setStatus("Saved!");
}

void UiModule::doLoad()
{
    if (m_mode != Mode::NORMAL) return;

    if (m_loadCallback) m_loadCallback(m_persistenceCtx, m_config);
    if (m_configChangedCallback) m_configChangedCallback(m_configChangedCtx, m_config);
    setStatus("Loaded!");
}

// =============================================================================
// Static button callbacks
// =============================================================================

void UiModule::onUp_(void *ctx)    { static_cast<UiModule*>(ctx)->navigate(-1, 0); }
void UiModule::onDown_(void *ctx)  { static_cast<UiModule*>(ctx)->navigate(+1, 0); }
void UiModule::onLeft_(void *ctx)  { static_cast<UiModule*>(ctx)->navigate(0, -1); }
void UiModule::onRight_(void *ctx) { static_cast<UiModule*>(ctx)->navigate(0, +1); }

void UiModule::onPlus_(void *ctx)  { static_cast<UiModule*>(ctx)->adjustCurrent(+1.0f); }
void UiModule::onMinus_(void *ctx) { static_cast<UiModule*>(ctx)->adjustCurrent(-1.0f); }

void UiModule::onSave_(void *ctx)  { static_cast<UiModule*>(ctx)->doSave(); }
void UiModule::onLoad_(void *ctx)  { static_cast<UiModule*>(ctx)->doLoad(); }

void UiModule::onTailPlus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, tailPosition), +1.0f); }
void UiModule::onTailMinus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, tailPosition), -1.0f); }

void UiModule::onLoopPlus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, loopHeight), +1.0f); }
void UiModule::onLoopMinus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, loopHeight), -1.0f); }

void UiModule::onSearchPlus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, firstSearchHeight), +1.0f); }
void UiModule::onSearchMinus_(void *ctx)
{ static_cast<UiModule*>(ctx)->adjustHotkey(offsetof(BonderConfig, firstSearchHeight), -1.0f); }

void UiModule::onStepPlus_(void *ctx)
{
    UiModule *self = static_cast<UiModule*>(ctx);
    self->m_stepIndex = static_cast<uint8_t>((self->m_stepIndex + 1U) % kNumStepSizes);
    self->m_displayOutOfDate = true;
}
void UiModule::onStepMinus_(void *ctx)
{
    UiModule *self = static_cast<UiModule*>(ctx);
    self->m_stepIndex = static_cast<uint8_t>(
        (self->m_stepIndex + kNumStepSizes - 1U) % kNumStepSizes);
    self->m_displayOutOfDate = true;
}

void UiModule::onFactoryReset_(void *ctx)
{
    UiModule *self = static_cast<UiModule*>(ctx);
    if (self->m_mode == Mode::CONFIRMING_FACTORY_RESET) {
        self->m_mode = Mode::NORMAL;
    } else {
        self->m_mode = Mode::CONFIRMING_FACTORY_RESET;
    }
    self->m_displayOutOfDate = true;
}

void UiModule::onEnter_(void *ctx)
{
    UiModule *self = static_cast<UiModule*>(ctx);
    if (self->m_mode == Mode::CONFIRMING_FACTORY_RESET) {
        self->doFactoryReset();
    }
}

void UiModule::doFactoryReset()
{
    m_config = BonderConfig{};
    m_mode   = Mode::NORMAL;

    if (m_configChangedCallback) {
        m_configChangedCallback(m_configChangedCtx, m_config);
    }
    setStatus("Factory reset done");
}

// =============================================================================
// Helpers
// =============================================================================

static void fmtFloat(char *buffer, size_t bufferSize, float value)
{
    if (buffer == nullptr || bufferSize == 0U) {
        return;
    }

    bool isNegative = (value < 0.0f);

    if (isNegative) {
        value = -value;
    }

    // Convert to thousandths and round.
    uint32_t scaledValue = static_cast<uint32_t>(value * 1000.0f + 0.5f);

    uint32_t integerPart = scaledValue / 1000U;
    uint32_t fractionalPart = scaledValue % 1000U;

    if (isNegative) {
        snprintf(buffer,
                 bufferSize,
                 "-%lu.%03lu",
                 static_cast<unsigned long>(integerPart),
                 static_cast<unsigned long>(fractionalPart));
    } else {
        snprintf(buffer,
                 bufferSize,
                 "%lu.%03lu",
                 static_cast<unsigned long>(integerPart),
                 static_cast<unsigned long>(fractionalPart));
    }
}
