#pragma once

#include "bonder_config.hpp"
#include "configuration_parameter_catalog.hpp"
#include "control_panel_service.hpp"
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
        SelectionStarted,         // fill the configuration list now
        SelectionProtocolChanged, // sign = direction; refill the list
        LoadConfiguration,        // name
        AddConfiguration,         // name
        DeleteConfiguration       // name
    };

    struct Request {
        RequestType type;
        int8_t sign;
        uint8_t screen;
        uint8_t parameterIndex;
        Hotkey hotkey;
        // Valid only during the callback; points at a menu-owned buffer.
        const char *name;
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
    void presentMessagePage(MessageKind kind);
    void fireRequest(const Request& request);

    void handleUp();
    void handleDown();
    void handleLeft();
    void handleRight();
    void handlePlus();
    void handleMinus();
    void handleSave();
    void handleLoad();
    void handleEnter();
    void handleAdd();
    void handleEscapeDelete();
    void adjustHotkey(Hotkey hotkey, int8_t sign);
    void adjustParameter(int8_t sign);

    void beginSelection();
    void beginNameEditor();
    void beginDeleteConfirmation();
    void moveNameCursor(int delta);
    void changeNameCharacter(int delta);
    void chooseDeleteAnswer(bool yes);
    void loadPointedConfiguration();
    void submitNewConfiguration();
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

    template <void (Menu::*Handler)()>
    static void onButton_(void *ctx)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (!self->acknowledgeMessage()) (self->*Handler)();
    }

    template <Hotkey hotkey, int8_t sign>
    static void onHotkeyButton_(void *ctx)
    {
        Menu *self = static_cast<Menu *>(ctx);
        if (!self->acknowledgeMessage()) self->adjustHotkey(hotkey, sign);
    }

    void refillParameterPage();
    void rebuildParameterHeader();
    void setFloatParameterRow(uint8_t row, const char *name, float value);
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

    Page m_deleteConfirmPage;
    TextWidget m_deleteTarget;
    TextWidget m_deleteQuestion;
    TextWidget m_deleteAnswers;

    Page m_messagePage;
    TextWidget m_messageRows[kMessageRowCount];
};
