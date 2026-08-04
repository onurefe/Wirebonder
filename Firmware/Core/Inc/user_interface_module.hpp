#pragma once

#include "configuration_manager.hpp"
#include "control_panel_service.hpp"
#include "lcd_controller_module.hpp"
#include "machine_settings.hpp"
#include "menu.hpp"
#include "page_renderer.hpp"
#include "pin_monitor_service.hpp"
#include "process.hpp"
#include <cstdint>

// Operator-facing boundary. Applies the user's configuration requests coming
// from the Menu against the ConfigurationManager, mutates the RAM-level
// active configuration owned by Robot and injected by pointer, and relays
// every other physical control (mouse, control panel) to the Robot. The
// Robot stays responsible for arbitrating these intents against machine
// state.
class UserInterfaceModule : public Process {
public:
    enum class Event : uint8_t {
        ActiveConfigurationChanged,
        ConfigurationConfirmed,
        ConfigurationSelectionStarted,
        StartTachCalRequested,
        MachineSettingsChanged
    };
    using EventCallback = void (*)(void *ctx, Event event);

    enum class MouseButtonEvent : uint8_t {
        RightPressed,
        RightReleased,
        LeftPressed,
        LeftReleased
    };
    using MouseButtonCallback = void (*)(void *ctx, MouseButtonEvent event);

    enum class ControlPanelButtonEvent : uint8_t {
        SetupPressed,
        TestPressed,
        ResetPressed,
        ClampOpenPressed,
        LightPressed
    };
    using ControlPanelButtonCallback =
        void (*)(void *ctx, ControlPanelButtonEvent event);

    struct ControlPanelButtons {
        ButtonChannel *setup;
        ButtonChannel *test;
        ButtonChannel *reset;
        ButtonChannel *clampOpen;
        ButtonChannel *light;
        // Switches the working configuration to manual bonding; handled
        // here rather than relayed, since it is a configuration change.
        ButtonChannel *manual;
    };

    UserInterfaceModule(
        LcdControllerModule *lcdController,
        ConfigurationManager *configurationManager,
        BonderConfig *activeConfiguration,
        MachineSettingsStore *machineSettingsStore,
        const Menu::NavigationButtons& navigationButtons,
        const Menu::ConfigurationButtons& configurationButtons,
        LedChannel *manualModeLed,
        PinMonitorChannel *mouseRightButtonChannel,
        PinMonitorChannel *mouseLeftButtonChannel,
        const ControlPanelButtons& controlPanelButtons);

    void setEventListenerCallback(void *ctx, EventCallback callback);
    void setMouseButtonListenerCallback(void *ctx,
                                        MouseButtonCallback callback);
    void setControlPanelButtonListenerCallback(
        void *ctx,
        ControlPanelButtonCallback callback);

    // Robot-owned RAM-level working configuration. Persisted only when the
    // operator presses save.
    const BonderConfig& activeConfiguration() const {
        return *m_activeConfiguration;
    }

    // --- Operator feedback --------------------------------------------
    // Three severities (see Menu): notifications time out on their own,
    // warnings stay until a button press, errors latch the keypad until
    // the machine is reset. A '\n' in the message starts a new display row;
    // rows past the message page are dropped.
    void notifyUser(const char *message);
    void warnUser(const char *message);
    void raiseError(const char *message);

    // Opens the force-entry page after a completed Setup run so the operator
    // can enter what their gauge read while the setup tracking force was
    // held. Submitting stores the difference as the force-coil offset;
    // Escape leaves the stored offset untouched.
    void promptForceMeasurement();

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    void fireEvent(Event event);
    void fireControlPanelButtonEvent(ControlPanelButtonEvent event);

    static void onMenuRequest_(void *ctx, const Menu::Request& request);
    void handleMenuRequest(const Menu::Request& request);
    // stepScale multiplies the catalog step; it is 1 for a single press and
    // larger while the operator holds the key down.
    void adjustParameter(uint8_t screen, uint8_t parameterIndex, int8_t sign,
                         uint16_t stepScale);
    void adjustHotkey(Menu::Hotkey hotkey, int8_t sign, uint16_t stepScale,
                      bool isRepeat);
    void adjustByDescriptor(
        const ConfigurationParameterCatalog::Descriptor *descriptor,
        float delta);
    void saveConfiguration();
    void saveConfigurationAs(const char *name);
    void beginSelection();
    void changeSelectionProtocol(int8_t direction);
    void loadConfiguration(const char *name);
    void addConfiguration(const char *name);
    void deleteConfiguration(const char *name);
    void fillConfigurationList();
    bool ensureStartupConfiguration();
    void publishActiveConfiguration(bool confirmed);
    void adjustSetting(uint8_t index, int8_t sign, uint16_t stepScale);
    void saveForceMeasurement(float measuredGrams);
    void toggleSpotlight();
    void saveSettings();
    void publishMachineSettings();

    static void onMouseRightButtonStateChanged(
        void *ctx, PinMonitorChannel::PinState state);
    static void onMouseLeftButtonStateChanged(
        void *ctx, PinMonitorChannel::PinState state);
    static void onSetupPressed(void *ctx);
    static void onTestPressed(void *ctx);
    static void onResetPressed(void *ctx);
    static void onClampOpenPressed(void *ctx);
    static void onLightPressed(void *ctx);
    static void onManualPressed(void *ctx);

    ConfigurationManager *m_configurationManager;
    MachineSettingsStore *m_machineSettingsStore;
    PageRenderer m_pageRenderer;
    Menu m_menu;
    LedChannel *m_manualModeLed;
    PinMonitorChannel *m_mouseRightButtonChannel;
    PinMonitorChannel *m_mouseLeftButtonChannel;
    ControlPanelButtons m_controlPanelButtons;

    BonderConfig *m_activeConfiguration;
    char m_activeConfigurationName[ConfigurationManager::kNameSize];
    BondingMode m_selectionProtocol;

    EventCallback m_eventCallback;
    void *m_eventCallbackCtx;
    MouseButtonCallback m_mouseButtonCallback;
    void *m_mouseButtonCallbackCtx;
    ControlPanelButtonCallback m_controlPanelButtonCallback;
    void *m_controlPanelButtonCallbackCtx;
};
