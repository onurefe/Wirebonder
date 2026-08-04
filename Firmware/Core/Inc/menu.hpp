#pragma once

#include "bonder_config.hpp"
#include "configuration_parameter_catalog.hpp"
#include "control_panel_service.hpp"
#include "machine_settings.hpp"
#include "page_renderer.hpp"
#include "widgets.hpp"
#include <cstdint>

// The operator-facing view layer: owns every UI page and widget shown on the
// LCD and exposes plain content setters. It holds no workflow state and knows
// nothing about configuration management — higher-level code decides what the
// values are, which page is active and where the pointer/window sit.
class Menu {
public:
    // Largest parameter group in the catalog (U/S Setup has 7 entries).
    static constexpr uint8_t kParameterRowCount = 7U;
    static constexpr uint8_t kConfigurationRowCount = 8U;
    static constexpr uint8_t kMessageRowCount = 4U;

    struct NavigationButtons {
        ButtonChannel *up;
        ButtonChannel *down;
        ButtonChannel *left;
        ButtonChannel *right;
    };

    // Left-panel buttons that edit or manage configurations.
    struct ConfigurationButtons {
        ButtonChannel *plus;
        ButtonChannel *minus;
        ButtonChannel *save;
        ButtonChannel *load;
        ButtonChannel *enter;
        ButtonChannel *add;
        ButtonChannel *escapeDelete;
        ButtonChannel *tailPlus;
        ButtonChannel *tailMinus;
        ButtonChannel *loopPlus;
        ButtonChannel *loopMinus;
        ButtonChannel *searchPlus;
        ButtonChannel *searchMinus;
        ButtonChannel *stepPlus;
        ButtonChannel *stepMinus;
    };

    enum class Hotkey : uint8_t { Tail, Loop, Search, Step };

    // User requests the menu cannot satisfy on its own. The upper layer
    // reacts (adjusts the config, talks to storage, refills menu content);
    // the menu updates the visible area right after the callback returns.
    enum class RequestType : uint8_t {
        AdjustParameter,          // screen, parameterIndex, sign
        AdjustHotkey,             // hotkey, sign
        SaveConfiguration,        // persist the active configuration
        SaveConfigurationAs,      // name — persist current edits under a new name
        SelectionStarted,         // fill the configuration list now
        SelectionProtocolChanged, // sign = direction; refill the list
        LoadConfiguration,        // name
        AddConfiguration,         // name
        DeleteConfiguration,      // name
        AdjustSetting,            // parameterIndex = numeric row, sign
        SaveSettings,             // persist machine-wide settings
        ToggleSpotlight,          // Enter pressed on the "Spot On" row
        StartTachCal,             // Enter pressed on the "Start Tach. Cal." row
        SaveForceMeasurement      // value = grams read off the operator's gauge
    };

    // What produced the current handler call. A press is {false, 1}; a repeat
    // carries the step multiplier for the stage the hold has reached, so
    // handlers need no memory of their own. Handlers that neither decline
    // repeats nor scale their action ignore it.
    struct InputEvent {
        bool repeat;
        uint16_t stepScale;
    };
    static constexpr InputEvent kPressInput{false, 1U};

    struct Request {
        RequestType type;
        int8_t sign;
        uint8_t screen;
        uint8_t parameterIndex;
        Hotkey hotkey;
        // Valid only during the callback; points at a menu-owned buffer.
        const char *name;
        // Numeric payload; only SaveForceMeasurement uses it.
        float value;
        // Repeat context. Defaulted so the positional initializers elsewhere
        // in the menu keep describing an ordinary single-step press.
        bool isRepeat = false;
        uint16_t stepScale = 1U;
    };
    using RequestCallback = void (*)(void *ctx, const Request& request);

    Menu(PageRenderer *renderer,
         const NavigationButtons& navigationButtons,
         const ConfigurationButtons& configurationButtons);

    void setRequestListenerCallback(void *ctx, RequestCallback callback);

    // Drives message timeouts; call from the main loop.
    void execute();

    // Activates a page and resets pointer/window. Navigation stays owned by
    // the menu: up/down move the '*' pointer (the visible window follows it),
    // left/right step through the parameter screens.
    void showPage(Page *page);
    // Pointer position in page rows; row 1 is the first selectable row, so
    // the selected parameter index is pointerRow() - 1. Returns
    // PageRenderer::kNoPointer when nothing is selectable.
    uint8_t pointerRow() const { return m_pointerRow; }

    // Pages owned by the menu. Widget rows start at page row 1 on pages
    // with a header.
    Page *parameterPage() { return &m_parameterPage; }
    Page *configurationSelectPage() { return &m_configurationSelectPage; }
    Page *nameEntryPage() { return &m_nameEntryPage; }
    Page *deleteConfirmPage() { return &m_deleteConfirmPage; }
    Page *messagePage() { return &m_messagePage; }

    // --- Parameter page -----------------------------------------------
    // Presents the given configuration: fills every parameter widget of the
    // current screen from the catalog (names, scaling, int/float, per-mode
    // visibility). Call again whenever a value or the protocol changes.
    void setConfiguration(const BonderConfig& configuration);
    // Selects which catalog screen (parameter group) the page shows.
    void setParameterScreen(uint8_t screen);
    uint8_t parameterScreen() const { return m_parameterScreen; }
    // Name of the active configuration shown in the header.
    void setActiveConfigurationName(const char *name);

    // --- Configuration selection page ---------------------------------
    void setConfigurationSelectHeader(const char *modeName,
                                      uint8_t modeIndex,
                                      uint8_t modeCount);
    void setConfigurationRow(uint8_t row, const char *name);
    void clearConfigurationRow(uint8_t row);

    // --- Name entry page ----------------------------------------------
    void setNameEntryMode(const char *modeName);
    void setNameEntryText(const char *name);
    void setNameEntryCursor(uint8_t position);

    // --- Save-as page ---------------------------------------------------
    // Opens a dedicated page to persist the current parameter-page edits
    // under a new name (e.g. the active configuration is protected and
    // cannot be saved over). Only reachable from the parameter page, so
    // both submit and Escape always return there.
    void promptSaveAsName();

    // --- Settings page ---------------------------------------------------
    // Machine-wide hardware tuning values (clamp voltage, area light,
    // spotlight level, spotlight on/off), independent of which bonding
    // configuration is loaded. Reached by pressing Save while on the
    // configuration selector page.
    // First kSettingsLevelRowCount rows are numeric (FloatWidget), edited
    // with +/-. The two rows below them are action rows driven by Enter:
    // "Spot On" fires RequestType::ToggleSpotlight (its ON/OFF TextWidget
    // only reports the resulting state) and "Start Tach. Cal." fires
    // RequestType::StartTachCal. +/- does nothing on either.
    static constexpr uint8_t kSettingsLevelRowCount = 6U;
    static constexpr uint8_t kSettingsRowCount = 8U;
    // 1-based, matching m_pointerRow's convention on the settings page (row 0
    // is the header; selectable rows are numbered 1..kSettingsRowCount).
    static constexpr uint8_t kSettingsSpotOnRow = 7U;
    static constexpr uint8_t kSettingsTachCalRow = 8U;
    void setSettingsValues(const MachineSettingsData& data);

    // --- Force measurement entry ----------------------------------------
    // Opened once the Setup protocol finishes, so the operator can type in
    // what their gauge read while the setup tracking force was held. +/- and
    // up/down adjust the value; Enter or Save submits it as
    // RequestType::SaveForceMeasurement; Escape abandons it. Either way the
    // page that was showing when the prompt opened comes back.
    void promptForceMeasurement(float initialGrams);
    Page *forceMeasurementPage() { return &m_forceMeasurementPage; }

    // --- Delete confirmation page -------------------------------------
    void setDeleteTarget(const char *name);
    void setDeleteSelection(bool yes);

    // --- Messages ------------------------------------------------------
    // All overlay the message page on top of the current page and restore
    // it (with pointer and scroll position) when the message ends. Three
    // severities:
    //  - Error: latched. Swallows every keypad press without dismissing;
    //    the operator must reset the machine. Once shown, no later message
    //    of any severity can replace it.
    //  - Warning: stays until the operator presses a button (the press is
    //    swallowed); the keypad is otherwise unaffected.
    //  - Notification: dismissed by a button press or a timeout.
    // A '\n' in the message starts a new row; rows past the message page
    // (and text past a row's width) are dropped.
    void setErrorMessage(const char *message);
    void setWarningMessage(const char *message);
    void setNotificationMessage(const char *message,
                                uint32_t durationMs = 5000U);
    // Multi-row notification (e.g. measurement reports); nullptr rows blank.
    void setReportMessage(const char *const rows[kMessageRowCount],
                          uint32_t durationMs = 10000U);
    // Ends the current message immediately. Navigation buttons call this
    // themselves; forward other button presses here so any key dismisses.
    void dismissMessage();
    bool isMessageActive() const { return m_messageKind != MessageKind::None; }

    // Free-form access for multi-row reports shown via showPage().
    void setMessageRow(uint8_t row, const char *text);

private:
    enum class MessageKind : uint8_t { None, Notification, Warning, Error };

    static constexpr uint8_t kEditableNameLength = 10U;

    void showMessagePage(const char *message, MessageKind kind);
    // Lays the message out over the rows from startRow on, breaking at
    // '\n', and blanks the rows that are left over.
    void layoutMessageRows(const char *message, uint8_t startRow);
    void presentMessagePage(MessageKind kind);
    void fireRequest(const Request& request);

    void handleUp(const InputEvent& input);
    void handleDown(const InputEvent& input);
    void handleLeft(const InputEvent& input);
    void handleRight(const InputEvent& input);
    void handleSave(const InputEvent& input);
    void handleLoad(const InputEvent& input);
    void handlePlus(const InputEvent& input);
    void handleMinus(const InputEvent& input);
    void handleEnter(const InputEvent& input);
    void handleAdd(const InputEvent& input);
    void handleEscapeDelete(const InputEvent& input);
    void adjustHotkey(Hotkey hotkey, int8_t sign, const InputEvent& input);
    void adjustParameter(int8_t sign, const InputEvent& input);
    // True when Left/Right must ignore this event: repeats only walk the name
    // cursor, never flip parameter screens or change the selected protocol.
    bool declineCursorRepeat(const InputEvent& input, const Page *page) const;

    void beginSelection();
    void beginNameEditor();
    void beginSaveAs();
    void beginSettings();
    void beginDeleteConfirmation();
    void moveNameCursor(int delta);
    void changeNameCharacter(int delta);
    void setSaveAsText(const char *name);
    void setSaveAsCursor(uint8_t position);
    void adjustSetting(int8_t sign, const InputEvent& input);
    void adjustForceMeasurement(int8_t sign, const InputEvent& input);
    void submitForceMeasurement();
    void leaveForceMeasurement();
    void chooseDeleteAnswer(bool yes);
    void loadPointedConfiguration();
    void submitNewConfiguration();
    void submitSaveAs();
    void submitDeleteConfirmation();
    const char *pointedConfigurationName() const;

    uint8_t selectableRowCount() const;
    void resetNavigation();
    void movePointer(int delta);
    void changeParameterScreen(int delta);
    void scrollWindowToPointer();

    // Button trampolines. Every press first acknowledges an active message
    // (the press is swallowed), then runs the bound handler.
    bool acknowledgeMessage();

    // Step multiplier for a hold of the given duration, from the decade
    // schedule in menu.cpp.
    static uint16_t stepScaleForHold(uint32_t heldMs);

    template <void (Menu::*Handler)(const InputEvent&)>
    static void onButton_(void *ctx)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (!self->acknowledgeMessage()) (self->*Handler)(kPressInput);
    }

    template <Hotkey hotkey, int8_t sign>
    static void onHotkeyButton_(void *ctx)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (!self->acknowledgeMessage()) self->adjustHotkey(hotkey, sign, kPressInput);
    }

    // Repeat trampolines. A held key must never dismiss a message, nor act on
    // the page hidden behind one, so an active message ends the repeat rather
    // than acknowledging it. The stage comes from the hold duration the button
    // channel reports, which is why the menu keeps no repeat state — presses
    // arrive from the I2C interrupt and repeats from the main loop.
    template <void (Menu::*Handler)(const InputEvent&)>
    static void onRepeatButton_(void *ctx, uint32_t heldMs)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (self->isMessageActive()) return;
        (self->*Handler)(InputEvent{true, stepScaleForHold(heldMs)});
    }

    template <Hotkey hotkey, int8_t sign>
    static void onRepeatHotkeyButton_(void *ctx, uint32_t heldMs)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (self->isMessageActive()) return;
        self->adjustHotkey(hotkey, sign, InputEvent{true, stepScaleForHold(heldMs)});
    }

    void refillParameterPage();
    void rebuildParameterHeader();
    void setFloatParameterRow(uint8_t row, const char *name, float value,
                              uint8_t decimals);
    void setIntegerParameterRow(uint8_t row, const char *name, int32_t value);
    void clearParameterRow(uint8_t row);

    PageRenderer *m_renderer;
    NavigationButtons m_buttons;
    ConfigurationButtons m_configurationButtons;
    RequestCallback m_requestCallback;
    void *m_requestCallbackCtx;
    uint8_t m_pointerRow;

    char m_newName[kEditableNameLength + 1U];
    uint8_t m_nameCursor;
    char m_deleteName[kEditableNameLength + 1U];
    bool m_deleteYes;

    MessageKind m_messageKind;
    uint32_t m_messageStartTick;
    uint32_t m_messageDurationMs;
    Page *m_pageBeforeMessage;
    uint8_t m_pointerRowBeforeMessage;
    uint8_t m_windowStartBeforeMessage;

    BonderConfig m_configuration;
    bool m_hasConfiguration;
    uint8_t m_parameterScreen;
    char m_configurationName[11];

    Page m_parameterPage;
    TextWidget m_parameterHeader;
    TextWidget m_parameterScreenIndicator;
    TextWidget m_parameterNames[kParameterRowCount];
    FloatWidget m_parameterFloatValues[kParameterRowCount];
    IntegerWidget m_parameterIntegerValues[kParameterRowCount];

    Page m_configurationSelectPage;
    TextWidget m_configurationSelectHeader;
    TextWidget m_configurationNames[kConfigurationRowCount];

    Page m_nameEntryPage;
    TextWidget m_nameEntryTitle;
    TextWidget m_nameEntryMode;
    TextWidget m_nameEntryName;
    TextWidget m_nameEntryCursor;

    Page m_saveAsPage;
    TextWidget m_saveAsTitle;
    TextWidget m_saveAsName;
    TextWidget m_saveAsCursor;

    Page m_settingsPage;
    TextWidget m_settingsHeader;
    TextWidget m_settingsNames[kSettingsRowCount];
    FloatWidget m_settingsValues[kSettingsLevelRowCount];
    TextWidget m_settingsSpotlightOnValue;

    Page m_forceMeasurementPage;
    TextWidget m_forceMeasurementTitle;
    TextWidget m_forceMeasurementName;
    FloatWidget m_forceMeasurementValue;
    TextWidget m_forceMeasurementHint;
    // Page to restore when the prompt is submitted or abandoned.
    Page *m_pageBeforeForceMeasurement;

    Page m_deleteConfirmPage;
    TextWidget m_deleteTarget;
    TextWidget m_deleteQuestion;
    TextWidget m_deleteAnswers;

    Page m_messagePage;
    TextWidget m_messageRows[kMessageRowCount];
};
