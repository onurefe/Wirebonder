#include "menu.hpp"
#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstring>

namespace {

// Column 0 is reserved for the PageRenderer pointer on rows the operator
// can select.
constexpr uint8_t kValueColumn = 13U;
constexpr uint8_t kValueWidth = 7U;
constexpr uint8_t kNameColumn = 1U;
constexpr uint8_t kNameWidth = 12U;
constexpr uint8_t kFloatDecimals = 3U;

constexpr char kNameCharacters[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";

// Step multiplier by hold duration. Highest threshold first; a hold shorter
// than every entry steps by one. Adding a decade is one row.
struct RepeatStage {
    uint32_t heldMs;
    uint16_t scale;
};
constexpr RepeatStage kRepeatStages[] = {
    {KEYPAD_BTN_REPEAT_X100_MS, 100U},
    {KEYPAD_BTN_REPEAT_X10_MS,   10U},
};

char bondingModeCode(BondingMode mode)
{
    switch (mode) {
    case BondingMode::Manual:        return 'M';
    case BondingMode::TableTear:     return 'T';
    case BondingMode::LangeCoupling: return 'L';
    case BondingMode::SemiAutomatic:
    default:                         return 'S';
    }
}

const char *parameterName(ConfigurationParameterCatalog::Parameter parameter)
{
    using Parameter = ConfigurationParameterCatalog::Parameter;
    switch (parameter) {
    case Parameter::Search1:          return "Search 1";
    case Parameter::Power1:           return "Power 1";
    case Parameter::Energy1:          return "Energy 1";
    case Parameter::Force1Current:    return "Force 1 G";
    case Parameter::Stepback:         return "Stepback";
    case Parameter::KinkHeight:       return "Kink Height";
    case Parameter::Reverse:          return "Reverse";
    case Parameter::LoopHeight:       return "Loop Height";
    case Parameter::Search2:          return "Search 2";
    case Parameter::Power2:           return "Power 2";
    case Parameter::Energy2:          return "Energy 2";
    case Parameter::Force2Current:    return "Force 2 G";
    case Parameter::Tail:             return "Tail";
    case Parameter::Tear:             return "Tear";
    case Parameter::ResetHeight:      return "Reset Height";
    case Parameter::Overtravel:       return "Overtravel";
    case Parameter::SecondZHeight:    return "Second Z Hgt";
    case Parameter::TableTail:        return "Table Tail";
    case Parameter::TableTear:        return "Table Tear";
    case Parameter::BondTimeout:      return "Bond Timeout";
    case Parameter::ContactSettle:    return "Contact Stl";
    case Parameter::Cooling:          return "Cooling";
    case Parameter::TailDelay:        return "Tail Delay";
    case Parameter::TearStabilize:    return "Tear Stabil";
    case Parameter::ConstantCurrent:  return "Constant G";
    case Parameter::TrackingCurrent:  return "Tracking G";
    case Parameter::ScanStart:        return "Scan Start";
    case Parameter::ScanStop:         return "Scan Stop";
    case Parameter::ScanPoints:       return "Scan Points";
    case Parameter::TailAssistPower:  return "Tail Power";
    case Parameter::TailAssistEnergy: return "Tail Energy";
    default:                          return "";
    }
}

} // namespace

Menu::Menu(PageRenderer *renderer,
           const NavigationButtons& navigationButtons,
           const ConfigurationButtons& configurationButtons)
    : m_renderer(renderer)
    , m_buttons(navigationButtons)
    , m_configurationButtons(configurationButtons)
    , m_requestCallback(nullptr)
    , m_requestCallbackCtx(nullptr)
    , m_pointerRow(PageRenderer::kNoPointer)
    , m_newName{}
    , m_nameCursor(0U)
    , m_deleteName{}
    , m_deleteYes(false)
    , m_messageKind(MessageKind::None)
    , m_messageStartTick(0U)
    , m_messageDurationMs(0U)
    , m_pageBeforeMessage(nullptr)
    , m_pointerRowBeforeMessage(PageRenderer::kNoPointer)
    , m_windowStartBeforeMessage(0U)
    , m_configuration{}
    , m_hasConfiguration(false)
    , m_parameterScreen(0U)
    , m_configurationName{}
    , m_parameterPage(1U + kParameterRowCount)
    , m_parameterHeader(0U, 0U, 16U)
    , m_parameterScreenIndicator(0U, 16U, 4U)
    , m_parameterNames{
          {1U, kNameColumn, kNameWidth},
          {2U, kNameColumn, kNameWidth},
          {3U, kNameColumn, kNameWidth},
          {4U, kNameColumn, kNameWidth},
          {5U, kNameColumn, kNameWidth},
          {6U, kNameColumn, kNameWidth},
          {7U, kNameColumn, kNameWidth}}
    , m_parameterFloatValues{
          {1U, kValueColumn, kValueWidth, kFloatDecimals},
          {2U, kValueColumn, kValueWidth, kFloatDecimals},
          {3U, kValueColumn, kValueWidth, kFloatDecimals},
          {4U, kValueColumn, kValueWidth, kFloatDecimals},
          {5U, kValueColumn, kValueWidth, kFloatDecimals},
          {6U, kValueColumn, kValueWidth, kFloatDecimals},
          {7U, kValueColumn, kValueWidth, kFloatDecimals}}
    , m_parameterIntegerValues{
          {1U, kValueColumn, kValueWidth},
          {2U, kValueColumn, kValueWidth},
          {3U, kValueColumn, kValueWidth},
          {4U, kValueColumn, kValueWidth},
          {5U, kValueColumn, kValueWidth},
          {6U, kValueColumn, kValueWidth},
          {7U, kValueColumn, kValueWidth}}
    , m_configurationSelectPage(1U + kConfigurationRowCount)
    , m_configurationSelectHeader(0U, 0U, 20U)
    , m_configurationNames{
          {1U, 1U, 19U},
          {2U, 1U, 19U},
          {3U, 1U, 19U},
          {4U, 1U, 19U},
          {5U, 1U, 19U},
          {6U, 1U, 19U},
          {7U, 1U, 19U},
          {8U, 1U, 19U}}
    , m_nameEntryPage(4U)
    , m_nameEntryTitle(0U, 0U, 20U, "ADD CONFIGURATION")
    , m_nameEntryMode(1U, 0U, 20U)
    , m_nameEntryName(2U, 0U, 20U)
    , m_nameEntryCursor(3U, 0U, 20U)
    , m_saveAsPage(3U)
    , m_saveAsTitle(0U, 0U, 20U, "SAVE AS")
    , m_saveAsName(1U, 0U, 20U)
    , m_saveAsCursor(2U, 0U, 20U)
    , m_settingsPage(1U + kSettingsRowCount)
    , m_settingsHeader(0U, 0U, 20U, "SETTINGS")
    , m_settingsNames{
          {1U, kNameColumn, kNameWidth, "Clamp Volt"},
          {2U, kNameColumn, kNameWidth, "Area Light"},
          {3U, kNameColumn, kNameWidth, "Spotlight"},
          {4U, kNameColumn, kNameWidth, "Up Speed"},
          {5U, kNameColumn, kNameWidth, "Down Speed"},
          {6U, kNameColumn, kNameWidth, "Setup Force"},
          {7U, kNameColumn, kNameWidth, "Spot On"},
          {8U, kNameColumn, 19U, "Start Tach. Cal."}}
    , m_settingsValues{
          {1U, kValueColumn, kValueWidth, 1U},
          {2U, kValueColumn, kValueWidth, 0U},
          {3U, kValueColumn, kValueWidth, 0U},
          {4U, kValueColumn, kValueWidth, 1U},
          {5U, kValueColumn, kValueWidth, 1U},
          {6U, kValueColumn, kValueWidth, 1U}}
    , m_settingsSpotlightOnValue(kSettingsSpotOnRow, kValueColumn, kValueWidth)
    , m_forceMeasurementPage(4U)
    , m_forceMeasurementTitle(0U, 0U, 20U, "FORCE SETUP")
    , m_forceMeasurementName(1U, 0U, 12U, "Measured Gr")
    , m_forceMeasurementValue(1U, kValueColumn, kValueWidth, 1U)
    , m_forceMeasurementHint(3U, 0U, 20U, "ENTER=save ESC=exit")
    , m_pageBeforeForceMeasurement(nullptr)
    , m_deleteConfirmPage(4U)
    , m_deleteTarget(0U, 0U, 20U)
    , m_deleteQuestion(1U, 0U, 20U, "Are you sure?")
    , m_deleteAnswers(3U, 0U, 20U)
    , m_messagePage(4U)
    , m_messageRows{
          {0U, 0U, 20U},
          {1U, 0U, 20U},
          {2U, 0U, 20U},
          {3U, 0U, 20U}}
{
    m_parameterPage.addWidget(&m_parameterHeader);
    m_parameterPage.addWidget(&m_parameterScreenIndicator);
    for (uint8_t row = 0U; row < kParameterRowCount; ++row) {
        m_parameterPage.addWidget(&m_parameterNames[row]);
        m_parameterPage.addWidget(&m_parameterFloatValues[row]);
        m_parameterPage.addWidget(&m_parameterIntegerValues[row]);
        m_parameterFloatValues[row].setVisible(false);
        m_parameterIntegerValues[row].setVisible(false);
    }

    m_configurationSelectPage.addWidget(&m_configurationSelectHeader);
    for (uint8_t row = 0U; row < kConfigurationRowCount; ++row) {
        m_configurationSelectPage.addWidget(&m_configurationNames[row]);
    }

    m_nameEntryPage.addWidget(&m_nameEntryTitle);
    m_nameEntryPage.addWidget(&m_nameEntryMode);
    m_nameEntryPage.addWidget(&m_nameEntryName);
    m_nameEntryPage.addWidget(&m_nameEntryCursor);

    m_saveAsPage.addWidget(&m_saveAsTitle);
    m_saveAsPage.addWidget(&m_saveAsName);
    m_saveAsPage.addWidget(&m_saveAsCursor);

    m_settingsPage.addWidget(&m_settingsHeader);
    for (uint8_t row = 0U; row < kSettingsLevelRowCount; ++row) {
        m_settingsPage.addWidget(&m_settingsNames[row]);
        m_settingsPage.addWidget(&m_settingsValues[row]);
    }
    m_settingsPage.addWidget(&m_settingsNames[kSettingsLevelRowCount]);
    m_settingsPage.addWidget(&m_settingsSpotlightOnValue);
    // kSettingsTachCalRow is the 1-based row number (m_pointerRow's
    // convention, matched in handleEnter()); the array index is 0-based, so
    // the last element is kSettingsRowCount - 1.
    m_settingsPage.addWidget(&m_settingsNames[kSettingsRowCount - 1U]);

    m_forceMeasurementPage.addWidget(&m_forceMeasurementTitle);
    m_forceMeasurementPage.addWidget(&m_forceMeasurementName);
    m_forceMeasurementPage.addWidget(&m_forceMeasurementValue);
    m_forceMeasurementPage.addWidget(&m_forceMeasurementHint);

    m_deleteConfirmPage.addWidget(&m_deleteTarget);
    m_deleteConfirmPage.addWidget(&m_deleteQuestion);
    m_deleteConfirmPage.addWidget(&m_deleteAnswers);

    for (uint8_t row = 0U; row < kMessageRowCount; ++row) {
        m_messagePage.addWidget(&m_messageRows[row]);
    }

    if (renderer != nullptr) {
        renderer->registerPage(&m_parameterPage);
        renderer->registerPage(&m_configurationSelectPage);
        renderer->registerPage(&m_nameEntryPage);
        renderer->registerPage(&m_saveAsPage);
        renderer->registerPage(&m_settingsPage);
        renderer->registerPage(&m_forceMeasurementPage);
        renderer->registerPage(&m_deleteConfirmPage);
        renderer->registerPage(&m_messagePage);
    }

    // A null repeat callback leaves the button press-only. Save, Load, Enter,
    // Add and Escape/Delete are deliberately in that group: one hold must
    // never save, load, delete or confirm more than once.
    struct ButtonBinding {
        ButtonChannel *button;
        ButtonChannel::PressCallback press;
        ButtonChannel::ProlongedPressCallback repeat;
    };
    const ButtonBinding bindings[] = {
        {m_buttons.up, onButton_<&Menu::handleUp>,
         onRepeatButton_<&Menu::handleUp>},
        {m_buttons.down, onButton_<&Menu::handleDown>,
         onRepeatButton_<&Menu::handleDown>},
        {m_buttons.left, onButton_<&Menu::handleLeft>,
         onRepeatButton_<&Menu::handleLeft>},
        {m_buttons.right, onButton_<&Menu::handleRight>,
         onRepeatButton_<&Menu::handleRight>},
        {m_configurationButtons.plus, onButton_<&Menu::handlePlus>,
         onRepeatButton_<&Menu::handlePlus>},
        {m_configurationButtons.minus, onButton_<&Menu::handleMinus>,
         onRepeatButton_<&Menu::handleMinus>},
        {m_configurationButtons.save, onButton_<&Menu::handleSave>, nullptr},
        {m_configurationButtons.load, onButton_<&Menu::handleLoad>, nullptr},
        {m_configurationButtons.enter, onButton_<&Menu::handleEnter>, nullptr},
        {m_configurationButtons.add, onButton_<&Menu::handleAdd>, nullptr},
        {m_configurationButtons.escapeDelete,
         onButton_<&Menu::handleEscapeDelete>, nullptr},
        {m_configurationButtons.tailPlus,
         onHotkeyButton_<Hotkey::Tail, 1>,
         onRepeatHotkeyButton_<Hotkey::Tail, 1>},
        {m_configurationButtons.tailMinus,
         onHotkeyButton_<Hotkey::Tail, -1>,
         onRepeatHotkeyButton_<Hotkey::Tail, -1>},
        {m_configurationButtons.loopPlus,
         onHotkeyButton_<Hotkey::Loop, 1>,
         onRepeatHotkeyButton_<Hotkey::Loop, 1>},
        {m_configurationButtons.loopMinus,
         onHotkeyButton_<Hotkey::Loop, -1>,
         onRepeatHotkeyButton_<Hotkey::Loop, -1>},
        {m_configurationButtons.searchPlus,
         onHotkeyButton_<Hotkey::Search, 1>,
         onRepeatHotkeyButton_<Hotkey::Search, 1>},
        {m_configurationButtons.searchMinus,
         onHotkeyButton_<Hotkey::Search, -1>,
         onRepeatHotkeyButton_<Hotkey::Search, -1>},
        {m_configurationButtons.stepPlus,
         onHotkeyButton_<Hotkey::Step, 1>,
         onRepeatHotkeyButton_<Hotkey::Step, 1>},
        {m_configurationButtons.stepMinus,
         onHotkeyButton_<Hotkey::Step, -1>,
         onRepeatHotkeyButton_<Hotkey::Step, -1>},
    };
    for (const ButtonBinding& binding : bindings) {
        if (binding.button == nullptr) continue;
        binding.button->addPressListenerCallback(this, binding.press);
        if (binding.repeat != nullptr) {
            binding.button->addProlongedPressListenerCallback(this, binding.repeat);
        }
    }
}

void Menu::setRequestListenerCallback(void *ctx, RequestCallback callback)
{
    m_requestCallbackCtx = ctx;
    m_requestCallback = callback;
}

void Menu::fireRequest(const Request& request)
{
    if (m_requestCallback != nullptr) {
        m_requestCallback(m_requestCallbackCtx, request);
    }
}

// Dismisses an active message and reports whether the press was consumed
// doing so. Errors are latched: the press is swallowed but the message
// stays, locking the keypad until the machine is reset.
uint16_t Menu::stepScaleForHold(uint32_t heldMs)
{
    for (const RepeatStage& stage : kRepeatStages) {
        if (heldMs >= stage.heldMs) return stage.scale;
    }
    return 1U;
}

bool Menu::acknowledgeMessage()
{
    if (!isMessageActive()) return false;
    if (m_messageKind != MessageKind::Error) {
        dismissMessage();
    }
    return true;
}

void Menu::handleUp(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        changeNameCharacter(1);
    } else if (page == &m_deleteConfirmPage) {
        chooseDeleteAnswer(true);
    } else if (page == &m_forceMeasurementPage) {
        adjustForceMeasurement(1, input);
    } else {
        movePointer(-1);
    }
}

void Menu::handleDown(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        changeNameCharacter(-1);
    } else if (page == &m_deleteConfirmPage) {
        chooseDeleteAnswer(false);
    } else if (page == &m_forceMeasurementPage) {
        adjustForceMeasurement(-1, input);
    } else {
        movePointer(1);
    }
}

// Left/Right repeat only where they walk the name cursor. On the parameter
// page they flip screens and reset the pointer row, and on the selector they
// reload every stored configuration from EEPROM — neither belongs on a key
// the operator is holding down. The first press still works everywhere.
bool Menu::declineCursorRepeat(const InputEvent& input, const Page *page) const
{
    return input.repeat && page != &m_nameEntryPage && page != &m_saveAsPage;
}

void Menu::handleLeft(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (declineCursorRepeat(input, page)) return;

    if (page == &m_parameterPage) {
        changeParameterScreen(-1);
    } else if (page == &m_configurationSelectPage) {
        fireRequest({RequestType::SelectionProtocolChanged,
                     -1, 0U, 0U, Hotkey::Tail, nullptr});
        resetNavigation();
    } else if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        moveNameCursor(-1);
    } else if (page == &m_deleteConfirmPage) {
        chooseDeleteAnswer(true);
    }
}

void Menu::handleRight(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (declineCursorRepeat(input, page)) return;

    if (page == &m_parameterPage) {
        changeParameterScreen(1);
    } else if (page == &m_configurationSelectPage) {
        fireRequest({RequestType::SelectionProtocolChanged,
                     1, 0U, 0U, Hotkey::Tail, nullptr});
        resetNavigation();
    } else if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        moveNameCursor(1);
    } else if (page == &m_deleteConfirmPage) {
        chooseDeleteAnswer(false);
    }
}

void Menu::handlePlus(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_parameterPage) {
        adjustParameter(1, input);
    } else if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        changeNameCharacter(1);
    } else if (page == &m_settingsPage) {
        adjustSetting(1, input);
    } else if (page == &m_forceMeasurementPage) {
        adjustForceMeasurement(1, input);
    }
}

void Menu::handleMinus(const InputEvent& input)
{
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_parameterPage) {
        adjustParameter(-1, input);
    } else if (page == &m_nameEntryPage || page == &m_saveAsPage) {
        changeNameCharacter(-1);
    } else if (page == &m_settingsPage) {
        adjustSetting(-1, input);
    } else if (page == &m_forceMeasurementPage) {
        adjustForceMeasurement(-1, input);
    }
}

void Menu::handleSave(const InputEvent& input)
{
    (void)input;  // press-only button
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_parameterPage) {
        fireRequest({RequestType::SaveConfiguration,
                     0, 0U, 0U, Hotkey::Tail, nullptr});
    } else if (page == &m_configurationSelectPage) {
        beginSettings();
    } else if (page == &m_settingsPage) {
        fireRequest({RequestType::SaveSettings,
                     0, 0U, 0U, Hotkey::Tail, nullptr});
    } else if (page == &m_forceMeasurementPage) {
        submitForceMeasurement();
    }
}

void Menu::handleLoad(const InputEvent& input)
{
    (void)input;  // press-only button
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_parameterPage) {
        beginSelection();
    } else if (page == &m_configurationSelectPage) {
        loadPointedConfiguration();
    }
}

void Menu::handleEnter(const InputEvent& input)
{
    (void)input;  // press-only button
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_configurationSelectPage) {
        loadPointedConfiguration();
    } else if (page == &m_nameEntryPage) {
        submitNewConfiguration();
    } else if (page == &m_saveAsPage) {
        submitSaveAs();
    } else if (page == &m_deleteConfirmPage) {
        submitDeleteConfirmation();
    } else if (page == &m_forceMeasurementPage) {
        submitForceMeasurement();
    } else if (page == &m_settingsPage && m_pointerRow == kSettingsSpotOnRow) {
        fireRequest({RequestType::ToggleSpotlight,
                     0, 0U, 0U, Hotkey::Tail, nullptr});
    } else if (page == &m_settingsPage && m_pointerRow == kSettingsTachCalRow) {
        fireRequest({RequestType::StartTachCal,
                     0, 0U, 0U, Hotkey::Tail, nullptr});
    }
}

void Menu::handleAdd(const InputEvent& input)
{
    (void)input;  // press-only button
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_configurationSelectPage) {
        beginNameEditor();
    }
}

void Menu::handleEscapeDelete(const InputEvent& input)
{
    (void)input;  // press-only button
    const Page *page = m_renderer != nullptr
        ? m_renderer->activePage() : nullptr;
    if (page == &m_parameterPage) {
        beginSelection();
    } else if (page == &m_configurationSelectPage) {
        beginDeleteConfirmation();
    } else if (page == &m_nameEntryPage || page == &m_deleteConfirmPage) {
        showPage(&m_configurationSelectPage);
    } else if (page == &m_saveAsPage) {
        showPage(&m_parameterPage);
    } else if (page == &m_settingsPage) {
        showPage(&m_configurationSelectPage);
    } else if (page == &m_forceMeasurementPage) {
        leaveForceMeasurement();
    }
}

void Menu::adjustParameter(int8_t sign, const InputEvent& input)
{
    if (m_renderer == nullptr ||
        m_renderer->activePage() != &m_parameterPage ||
        m_pointerRow == PageRenderer::kNoPointer) return;
    fireRequest({RequestType::AdjustParameter, sign, m_parameterScreen,
                 static_cast<uint8_t>(m_pointerRow - 1U),
                 Hotkey::Tail, nullptr, 0.0f,
                 input.repeat, input.stepScale});
}

void Menu::adjustHotkey(Hotkey hotkey, int8_t sign, const InputEvent& input)
{
    if (m_renderer == nullptr ||
        m_renderer->activePage() != &m_parameterPage) return;
    fireRequest({RequestType::AdjustHotkey, sign, 0U, 0U, hotkey, nullptr, 0.0f,
                 input.repeat, input.stepScale});
}

void Menu::beginSelection()
{
    fireRequest({RequestType::SelectionStarted,
                 0, 0U, 0U, Hotkey::Tail, nullptr});
    showPage(&m_configurationSelectPage);
}

void Menu::beginNameEditor()
{
    std::memset(m_newName, ' ', kEditableNameLength);
    constexpr char initialName[] = "NEWCONFIG";
    std::memcpy(m_newName, initialName, sizeof(initialName) - 1U);
    m_newName[kEditableNameLength] = '\0';
    m_nameCursor = 0U;
    setNameEntryText(m_newName);
    setNameEntryCursor(m_nameCursor);
    showPage(&m_nameEntryPage);
}

void Menu::beginSaveAs()
{
    std::memset(m_newName, ' ', kEditableNameLength);
    constexpr char initialName[] = "NEWCONFIG";
    std::memcpy(m_newName, initialName, sizeof(initialName) - 1U);
    m_newName[kEditableNameLength] = '\0';
    m_nameCursor = 0U;
    setSaveAsText(m_newName);
    setSaveAsCursor(m_nameCursor);
    showPage(&m_saveAsPage);
}

void Menu::promptSaveAsName()
{
    beginSaveAs();
}

void Menu::beginSettings()
{
    showPage(&m_settingsPage);
}

void Menu::adjustSetting(int8_t sign, const InputEvent& input)
{
    if (m_renderer == nullptr ||
        m_renderer->activePage() != &m_settingsPage ||
        m_pointerRow == PageRenderer::kNoPointer ||
        m_pointerRow > kSettingsLevelRowCount) return;  // action rows: Enter only
    fireRequest({RequestType::AdjustSetting, sign, 0U,
                 static_cast<uint8_t>(m_pointerRow - 1U),
                 Hotkey::Tail, nullptr, 0.0f,
                 input.repeat, input.stepScale});
}

void Menu::promptForceMeasurement(float initialGrams)
{
    if (m_renderer == nullptr) return;

    // The prompt is raised right after the Setup protocol reports completion,
    // so the notification for it is usually still up and activePage() is the
    // message page. What has to come back on submit/Escape is the page the
    // message will restore to, not the message itself.
    m_pageBeforeForceMeasurement = isMessageActive()
        ? m_pageBeforeMessage
        : m_renderer->activePage();

    m_forceMeasurementValue.setValue(initialGrams);
    showPage(&m_forceMeasurementPage);
}

// The measured force never leaves the menu until it is submitted, so the hold
// scaling is applied here rather than travelling through a Request.
void Menu::adjustForceMeasurement(int8_t sign, const InputEvent& input)
{
    float value = m_forceMeasurementValue.value() +
        static_cast<float>(sign) * static_cast<float>(input.stepScale) *
        FORCE_SETUP_MEASURED_FORCE_STEP;
    if (value < FORCE_SETUP_MEASURED_FORCE_MIN) {
        value = FORCE_SETUP_MEASURED_FORCE_MIN;
    }
    if (value > FORCE_SETUP_MEASURED_FORCE_MAX) {
        value = FORCE_SETUP_MEASURED_FORCE_MAX;
    }
    m_forceMeasurementValue.setValue(value);
}

void Menu::submitForceMeasurement()
{
    Request request{RequestType::SaveForceMeasurement,
                    0, 0U, 0U, Hotkey::Tail, nullptr,
                    m_forceMeasurementValue.value()};
    fireRequest(request);
    leaveForceMeasurement();
}

void Menu::leaveForceMeasurement()
{
    Page *restore = m_pageBeforeForceMeasurement != nullptr
        ? m_pageBeforeForceMeasurement
        : &m_configurationSelectPage;
    m_pageBeforeForceMeasurement = nullptr;
    showPage(restore);
}

void Menu::beginDeleteConfirmation()
{
    const char *name = pointedConfigurationName();
    if (name == nullptr) return;
    snprintf(m_deleteName, sizeof(m_deleteName), "%s", name);
    m_deleteYes = false;
    setDeleteTarget(m_deleteName);
    setDeleteSelection(m_deleteYes);
    showPage(&m_deleteConfirmPage);
}

void Menu::moveNameCursor(int delta)
{
    int cursor = m_nameCursor + delta;
    if (cursor < 0) cursor = 0;
    if (cursor >= kEditableNameLength) cursor = kEditableNameLength - 1;
    m_nameCursor = static_cast<uint8_t>(cursor);
    if (m_renderer != nullptr && m_renderer->activePage() == &m_saveAsPage) {
        setSaveAsCursor(m_nameCursor);
    } else {
        setNameEntryCursor(m_nameCursor);
    }
}

void Menu::changeNameCharacter(int delta)
{
    constexpr int characterCount =
        static_cast<int>(sizeof(kNameCharacters)) - 1;
    int current = 0;
    for (int i = 0; i < characterCount; ++i) {
        if (kNameCharacters[i] == m_newName[m_nameCursor]) {
            current = i;
            break;
        }
    }
    current = (current + characterCount + delta) % characterCount;
    m_newName[m_nameCursor] = kNameCharacters[current];
    if (m_renderer != nullptr && m_renderer->activePage() == &m_saveAsPage) {
        setSaveAsText(m_newName);
    } else {
        setNameEntryText(m_newName);
    }
}

void Menu::chooseDeleteAnswer(bool yes)
{
    m_deleteYes = yes;
    setDeleteSelection(m_deleteYes);
}

const char *Menu::pointedConfigurationName() const
{
    if (m_pointerRow == PageRenderer::kNoPointer ||
        m_pointerRow < 1U ||
        m_pointerRow > kConfigurationRowCount) return nullptr;
    const char *name = m_configurationNames[m_pointerRow - 1U].text();
    return name[0] != '\0' ? name : nullptr;
}

void Menu::loadPointedConfiguration()
{
    const char *name = pointedConfigurationName();
    if (name == nullptr) return;
    fireRequest({RequestType::LoadConfiguration,
                 0, 0U, 0U, Hotkey::Tail, name});
    showPage(&m_parameterPage);
}

void Menu::submitNewConfiguration()
{
    // Trim trailing spaces before handing the name over.
    int end = kEditableNameLength - 1;
    while (end >= 0 && m_newName[end] == ' ') --end;
    m_newName[end + 1] = '\0';
    if (m_newName[0] == '\0') return;

    fireRequest({RequestType::AddConfiguration,
                 0, 0U, 0U, Hotkey::Tail, m_newName});
    showPage(&m_parameterPage);
}

void Menu::submitSaveAs()
{
    // Trim trailing spaces before handing the name over.
    int end = kEditableNameLength - 1;
    while (end >= 0 && m_newName[end] == ' ') --end;
    m_newName[end + 1] = '\0';
    if (m_newName[0] == '\0') return;

    fireRequest({RequestType::SaveConfigurationAs,
                 0, 0U, 0U, Hotkey::Tail, m_newName});
    showPage(&m_parameterPage);
}

void Menu::submitDeleteConfirmation()
{
    if (m_deleteYes) {
        fireRequest({RequestType::DeleteConfiguration,
                     0, 0U, 0U, Hotkey::Tail, m_deleteName});
    }
    showPage(&m_configurationSelectPage);
}

void Menu::execute()
{
    if (m_messageKind == MessageKind::Notification &&
        (HAL_GetTick() - m_messageStartTick) >= m_messageDurationMs) {
        dismissMessage();
    }
}

void Menu::setErrorMessage(const char *message)
{
    showMessagePage(message, MessageKind::Error);
}

void Menu::setWarningMessage(const char *message)
{
    showMessagePage(message, MessageKind::Warning);
}

void Menu::setNotificationMessage(const char *message, uint32_t durationMs)
{
    m_messageDurationMs = durationMs;
    showMessagePage(message, MessageKind::Notification);
}

void Menu::setReportMessage(const char *const rows[kMessageRowCount],
                            uint32_t durationMs)
{
    if (m_renderer == nullptr || rows == nullptr) return;
    // A latched error owns the screen; the first one reported is the root
    // cause and nothing may replace it.
    if (m_messageKind == MessageKind::Error) return;
    for (uint8_t row = 0U; row < kMessageRowCount; ++row) {
        m_messageRows[row].setText(rows[row] != nullptr ? rows[row] : "");
    }
    m_messageDurationMs = durationMs;
    presentMessagePage(MessageKind::Notification);
}

void Menu::showMessagePage(const char *message, MessageKind kind)
{
    if (m_renderer == nullptr || message == nullptr) return;
    // A latched error owns the screen; the first one reported is the root
    // cause and nothing may replace it.
    if (m_messageKind == MessageKind::Error) return;

    if (kind == MessageKind::Error) {
        m_messageRows[0].setText("ERROR");
        layoutMessageRows(message, 1U);
    } else if (kind == MessageKind::Warning) {
        m_messageRows[0].setText("WARNING");
        layoutMessageRows(message, 1U);
    } else {
        layoutMessageRows(message, 0U);
    }
    presentMessagePage(kind);
}

void Menu::layoutMessageRows(const char *message, uint8_t startRow)
{
    uint8_t row = startRow;
    const char *line = message;
    while (line != nullptr && row < kMessageRowCount) {
        const char *lineEnd = strchr(line, '\n');
        if (lineEnd == nullptr) {
            m_messageRows[row].setText(line);
            line = nullptr;
        } else {
            // setText needs a terminated string, so copy the line out;
            // anything past the row width is dropped by the widget.
            char buffer[TextWidget::kMaxTextLength + 1U];
            size_t length = static_cast<size_t>(lineEnd - line);
            if (length > TextWidget::kMaxTextLength) {
                length = TextWidget::kMaxTextLength;
            }
            memcpy(buffer, line, length);
            buffer[length] = '\0';
            m_messageRows[row].setText(buffer);
            line = lineEnd + 1;
        }
        ++row;
    }
    for (; row < kMessageRowCount; ++row) {
        m_messageRows[row].setText("");
    }
}

void Menu::presentMessagePage(MessageKind kind)
{
    // A follow-up message must not adopt the message page as the page to
    // restore; keep the one saved when the first message appeared.
    if (m_messageKind == MessageKind::None) {
        m_pageBeforeMessage = m_renderer->activePage();
        m_pointerRowBeforeMessage = m_pointerRow;
        m_windowStartBeforeMessage = m_renderer->windowStart();
    }
    m_messageKind = kind;
    m_messageStartTick = HAL_GetTick();

    m_renderer->setActivePage(&m_messagePage);
    m_pointerRow = PageRenderer::kNoPointer;
}

void Menu::dismissMessage()
{
    if (m_renderer == nullptr || m_messageKind == MessageKind::None) return;
    m_messageKind = MessageKind::None;

    m_renderer->setActivePage(m_pageBeforeMessage);
    if (m_pointerRowBeforeMessage == PageRenderer::kNoPointer) {
        // No saved position (page changed underneath the message, or it had
        // none); derive a fresh one from the restored page.
        resetNavigation();
    } else {
        m_pointerRow = m_pointerRowBeforeMessage;
        m_renderer->setPointerRow(m_pointerRow);
        m_renderer->setWindowStart(m_windowStartBeforeMessage);
    }
    m_pageBeforeMessage = nullptr;
}

void Menu::showPage(Page *page)
{
    if (m_renderer == nullptr) return;

    // While a message is up, the requested page becomes what the message
    // restores to instead of replacing it on screen.
    if (isMessageActive() && page != &m_messagePage) {
        m_pageBeforeMessage = page;
        m_pointerRowBeforeMessage = PageRenderer::kNoPointer;
        m_windowStartBeforeMessage = 0U;
        return;
    }

    m_renderer->setActivePage(page);
    resetNavigation();
}

// Rows the pointer may visit on the active page; they start at page row 1,
// below the header.
uint8_t Menu::selectableRowCount() const
{
    if (m_renderer == nullptr) return 0U;
    const Page *page = m_renderer->activePage();

    if (page == &m_parameterPage) {
        if (!m_hasConfiguration) return 0U;
        return ConfigurationParameterCatalog::screenParameterCount(
            m_parameterScreen, m_configuration.bondingMode);
    }
    if (page == &m_configurationSelectPage) {
        uint8_t count = 0U;
        while (count < kConfigurationRowCount &&
               m_configurationNames[count].text()[0] != '\0') {
            ++count;
        }
        return count;
    }
    if (page == &m_settingsPage) {
        return kSettingsRowCount;
    }
    return 0U;
}

void Menu::resetNavigation()
{
    if (m_renderer == nullptr) return;
    m_pointerRow = selectableRowCount() > 0U
        ? 1U
        : PageRenderer::kNoPointer;
    m_renderer->setWindowStart(0U);
    m_renderer->setPointerRow(m_pointerRow);
}

void Menu::movePointer(int delta)
{
    if (m_renderer == nullptr) return;
    const uint8_t count = selectableRowCount();
    if (count == 0U) {
        m_pointerRow = PageRenderer::kNoPointer;
        m_renderer->setPointerRow(m_pointerRow);
        return;
    }

    int row = m_pointerRow == PageRenderer::kNoPointer
        ? 1
        : m_pointerRow + delta;
    if (row < 1) row = 1;
    if (row > count) row = count;

    m_pointerRow = static_cast<uint8_t>(row);
    m_renderer->setPointerRow(m_pointerRow);
    scrollWindowToPointer();
}

void Menu::changeParameterScreen(int delta)
{
    if (m_renderer == nullptr ||
        m_renderer->activePage() != &m_parameterPage) return;

    int screen = m_parameterScreen + delta;
    const int screenCount = ConfigurationParameterCatalog::kScreenCount;
    while (screen < 0) screen += screenCount;
    while (screen >= screenCount) screen -= screenCount;

    setParameterScreen(static_cast<uint8_t>(screen));
    resetNavigation();
}

void Menu::scrollWindowToPointer()
{
    if (m_pointerRow == PageRenderer::kNoPointer) return;

    // Recomputed from pointerRow alone (not the previous windowStart) so the
    // header row reappears once the pointer scrolls back within the first
    // window, regardless of how far down it had previously scrolled.
    const uint8_t windowStart = m_pointerRow < PageRenderer::kVisibleRows
        ? 0U
        : static_cast<uint8_t>(
              m_pointerRow - (PageRenderer::kVisibleRows - 1U));
    m_renderer->setWindowStart(windowStart);
}

void Menu::setConfiguration(const BonderConfig& configuration)
{
    m_configuration = configuration;
    m_hasConfiguration = true;
    refillParameterPage();
}

void Menu::setParameterScreen(uint8_t screen)
{
    if (screen >= ConfigurationParameterCatalog::kScreenCount) return;
    m_parameterScreen = screen;
    refillParameterPage();
}

void Menu::setActiveConfigurationName(const char *name)
{
    snprintf(m_configurationName, sizeof(m_configurationName), "%s",
             name != nullptr ? name : "");
    rebuildParameterHeader();
}

void Menu::rebuildParameterHeader()
{
    char text[21];
    snprintf(text, sizeof(text), "Mod:%c %-10.10s",
             bondingModeCode(m_configuration.bondingMode),
             m_configurationName);
    m_parameterHeader.setText(text);
}

void Menu::refillParameterPage()
{
    if (!m_hasConfiguration) return;

    rebuildParameterHeader();

    char indicator[8];
    snprintf(indicator, sizeof(indicator), "%2u/%u",
             static_cast<unsigned>(m_parameterScreen + 1U),
             static_cast<unsigned>(
                 ConfigurationParameterCatalog::kScreenCount));
    m_parameterScreenIndicator.setText(indicator);

    const uint8_t *base =
        reinterpret_cast<const uint8_t *>(&m_configuration);
    for (uint8_t row = 0U; row < kParameterRowCount; ++row) {
        const ConfigurationParameterCatalog::Descriptor *descriptor =
            ConfigurationParameterCatalog::at(
                m_parameterScreen, row, m_configuration.bondingMode);
        if (descriptor == nullptr) {
            clearParameterRow(row);
            continue;
        }
        const char *name = parameterName(descriptor->parameter);
        if (descriptor->isInteger) {
            uint16_t value;
            std::memcpy(&value, base + descriptor->offset, sizeof(value));
            setIntegerParameterRow(row, name, value);
        } else {
            float value;
            std::memcpy(&value, base + descriptor->offset, sizeof(value));
            setFloatParameterRow(row, name,
                value * descriptor->scale + descriptor->displayOffset,
                ConfigurationParameterCatalog::displayDecimals(descriptor));
        }
    }

    // A mode or screen change can shrink the selectable list; keep the
    // pointer inside it.
    if (m_renderer != nullptr &&
        m_renderer->activePage() == &m_parameterPage) {
        movePointer(0);
    }
}

void Menu::setFloatParameterRow(uint8_t row, const char *name, float value,
                                uint8_t decimals)
{
    if (row >= kParameterRowCount) return;
    m_parameterNames[row].setText(name);
    m_parameterFloatValues[row].setDecimals(decimals);
    m_parameterFloatValues[row].setValue(value);
    m_parameterFloatValues[row].setVisible(true);
    m_parameterIntegerValues[row].setVisible(false);
}

void Menu::setIntegerParameterRow(uint8_t row, const char *name, int32_t value)
{
    if (row >= kParameterRowCount) return;
    m_parameterNames[row].setText(name);
    m_parameterIntegerValues[row].setValue(value);
    m_parameterIntegerValues[row].setVisible(true);
    m_parameterFloatValues[row].setVisible(false);
}

void Menu::clearParameterRow(uint8_t row)
{
    if (row >= kParameterRowCount) return;
    m_parameterNames[row].setText("");
    m_parameterFloatValues[row].setVisible(false);
    m_parameterIntegerValues[row].setVisible(false);
}

void Menu::setConfigurationSelectHeader(const char *modeName,
                                        uint8_t modeIndex,
                                        uint8_t modeCount)
{
    char text[28];
    snprintf(text, sizeof(text), "Mode: %-10.10s %u/%u",
             modeName != nullptr ? modeName : "",
             static_cast<unsigned>(modeIndex),
             static_cast<unsigned>(modeCount));
    m_configurationSelectHeader.setText(text);
}

void Menu::setConfigurationRow(uint8_t row, const char *name)
{
    if (row >= kConfigurationRowCount) return;
    m_configurationNames[row].setText(name);
}

void Menu::clearConfigurationRow(uint8_t row)
{
    if (row >= kConfigurationRowCount) return;
    m_configurationNames[row].setText("");
}

void Menu::setNameEntryMode(const char *modeName)
{
    char text[21];
    snprintf(text, sizeof(text), "Mode: %-14.14s",
             modeName != nullptr ? modeName : "");
    m_nameEntryMode.setText(text);
}

void Menu::setNameEntryText(const char *name)
{
    char text[21];
    snprintf(text, sizeof(text), "[%-10.10s]",
             name != nullptr ? name : "");
    m_nameEntryName.setText(text);
}

void Menu::setNameEntryCursor(uint8_t position)
{
    // The caret sits under the bracketed name; column 0 holds '['.
    char text[21];
    std::memset(text, ' ', 20U);
    text[20] = '\0';
    if (position < 10U) {
        text[1U + position] = '^';
    }
    m_nameEntryCursor.setText(text);
}

void Menu::setSaveAsText(const char *name)
{
    char text[21];
    snprintf(text, sizeof(text), "[%-10.10s]",
             name != nullptr ? name : "");
    m_saveAsName.setText(text);
}

void Menu::setSaveAsCursor(uint8_t position)
{
    // The caret sits under the bracketed name; column 0 holds '['.
    char text[21];
    std::memset(text, ' ', 20U);
    text[20] = '\0';
    if (position < 10U) {
        text[1U + position] = '^';
    }
    m_saveAsCursor.setText(text);
}

void Menu::setSettingsValues(const MachineSettingsData& data)
{
    m_settingsValues[0].setValue(data.clampSolenoidVoltage);
    m_settingsValues[1].setValue(data.areaLightLevel);
    m_settingsValues[2].setValue(data.spotlightLevel);
    m_settingsValues[3].setValue(data.zMotorMaxUpwardSpeed);
    m_settingsValues[4].setValue(data.zMotorMaxDownwardSpeed);
    m_settingsValues[5].setValue(data.forceSetupTrackingForce);
    m_settingsSpotlightOnValue.setText(data.spotlightOn ? "On" : "Off");
}

void Menu::setDeleteTarget(const char *name)
{
    char text[21];
    snprintf(text, sizeof(text), "Deleting: %-10.10s",
             name != nullptr ? name : "");
    m_deleteTarget.setText(text);
}

void Menu::setDeleteSelection(bool yes)
{
    m_deleteAnswers.setText(yes
        ? " Y*             N"
        : " Y              N*");
}

void Menu::setMessageRow(uint8_t row, const char *text)
{
    if (row >= kMessageRowCount) return;
    m_messageRows[row].setText(text);
}
