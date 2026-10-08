#include "user_interface_module.hpp"
#include "configuration.h"
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint8_t kProtocolCount = 4U;
// Bootstrap seed name created by ensureStartupConfiguration() when the
// store is empty. Protected from being silently overwritten by Save.
constexpr char kDefaultConfigurationName[] = "DEFAULT";

struct SettingLimits {
    float min;
    float max;
    float step;
};

// Indexed by settings-page row (0 = clamp voltage, 1 = area light,
// 2 = spotlight level, 3 = Z up speed, 4 = Z down speed); mirrors
// Menu::kSettingsLevelRowCount, in the same order as the field table in
// adjustSetting(). The row after these (spotlight on/off) is a bool,
// handled separately there.
constexpr SettingLimits kSettingLimits[Menu::kSettingsLevelRowCount] = {
    {CLAMP_SOLENOID_VOLTAGE_MIN, CLAMP_SOLENOID_VOLTAGE_MAX,
     CLAMP_SOLENOID_VOLTAGE_STEP},
    {0.0f, 100.0f, AREA_LIGHT_LEVEL_STEP},
    {0.0f, 100.0f, SPOTLIGHT_LEVEL_STEP},
    {ZMOTOR_MAX_UPWARD_SPEED_MIN, ZMOTOR_MAX_UPWARD_SPEED_MAX,
     ZMOTOR_MAX_UPWARD_SPEED_STEP},
    {ZMOTOR_MAX_DOWNWARD_SPEED_MIN, ZMOTOR_MAX_DOWNWARD_SPEED_MAX,
     ZMOTOR_MAX_DOWNWARD_SPEED_STEP},
    {FORCE_SETUP_TRACKING_FORCE_MIN, FORCE_SETUP_TRACKING_FORCE_MAX,
     FORCE_SETUP_TRACKING_FORCE_STEP},
};

const char *bondingModeName(BondingMode mode)
{
    switch (mode) {
    case BondingMode::Manual:        return "Manual";
    case BondingMode::TableTear:     return "Table Tear";
    case BondingMode::LangeCoupling: return "Lange Cplr";
    case BondingMode::SemiAutomatic:
    default:                         return "Semi Auto";
    }
}

uint16_t hotkeyParameterOffset(Menu::Hotkey hotkey, BondingMode mode)
{
    switch (hotkey) {
    case Menu::Hotkey::Tail:
        return mode == BondingMode::TableTear
            ? offsetof(BonderConfig, yTailPosition)
            : offsetof(BonderConfig, tailDisplacement);
    case Menu::Hotkey::Loop:
        return offsetof(BonderConfig, loopHeight);
    case Menu::Hotkey::Search:
        return offsetof(BonderConfig, firstSearchHeight);
    case Menu::Hotkey::Step:
    default:
        return offsetof(BonderConfig, yStepbackPosition);
    }
}

} // namespace

UserInterfaceModule::UserInterfaceModule(
    LcdControllerModule *lcdController,
    ConfigurationManager *configurationManager,
    BonderConfig *activeConfiguration,
    MachineSettingsStore *machineSettingsStore,
    const Menu::NavigationButtons& navigationButtons,
    const Menu::ConfigurationButtons& configurationButtons,
    LedChannel *manualModeLed,
    PinMonitorChannel *mouseRightButtonChannel,
    PinMonitorChannel *mouseLeftButtonChannel,
    const ControlPanelButtons& controlPanelButtons)
    : m_configurationManager(configurationManager)
    , m_machineSettingsStore(machineSettingsStore)
    , m_pageRenderer(lcdController)
    , m_menu(&m_pageRenderer, navigationButtons, configurationButtons)
    , m_manualModeLed(manualModeLed)
    , m_mouseRightButtonChannel(mouseRightButtonChannel)
    , m_mouseLeftButtonChannel(mouseLeftButtonChannel)
    , m_controlPanelButtons(controlPanelButtons)
    , m_activeConfiguration(activeConfiguration)
    , m_activeConfigurationName{}
    , m_selectionProtocol(BondingMode::SemiAutomatic)
{
    m_menu.addRequestListenerCallback(this, onMenuRequest_);
}

bool UserInterfaceModule::addEventListenerCallback(void *ctx, EventCallback callback)
{
    return m_eventCallbacks.add(ctx, callback);
}

bool UserInterfaceModule::removeEventListenerCallback(void *ctx, EventCallback callback)
{
    return m_eventCallbacks.remove(ctx, callback);
}

bool UserInterfaceModule::addMouseButtonListenerCallback(void *ctx, MouseButtonCallback callback)
{
    return m_mouseButtonCallbacks.add(ctx, callback);
}

bool UserInterfaceModule::removeMouseButtonListenerCallback(void *ctx, MouseButtonCallback callback)
{
    return m_mouseButtonCallbacks.remove(ctx, callback);
}

bool UserInterfaceModule::addControlPanelButtonListenerCallback(
    void *ctx, ControlPanelButtonCallback callback)
{
    return m_controlPanelButtonCallbacks.add(ctx, callback);
}

bool UserInterfaceModule::removeControlPanelButtonListenerCallback(
    void *ctx, ControlPanelButtonCallback callback)
{
    return m_controlPanelButtonCallbacks.remove(ctx, callback);
}

void UserInterfaceModule::fireEvent(Event event)
{
    {
        m_eventCallbacks.invoke(event);
    }
}

void UserInterfaceModule::fireControlPanelButtonEvent(
    ControlPanelButtonEvent event)
{
    {
        m_controlPanelButtonCallbacks.invoke(event);
    }
}

// -----------------------------------------------------------------------------
// Process lifecycle
// -----------------------------------------------------------------------------

void UserInterfaceModule::onStart()
{
    if (m_configurationManager == nullptr ||
        m_activeConfiguration == nullptr ||
        m_machineSettingsStore == nullptr ||
        m_mouseRightButtonChannel == nullptr ||
        m_mouseLeftButtonChannel == nullptr ||
        m_controlPanelButtons.setup == nullptr ||
        m_controlPanelButtons.test == nullptr ||
        m_controlPanelButtons.reset == nullptr ||
        m_controlPanelButtons.clampOpen == nullptr ||
        m_controlPanelButtons.light == nullptr ||
        m_controlPanelButtons.manual == nullptr) {
        setProcessError();
        return;
    }
    if (!m_configurationManager->isReady() &&
        !m_configurationManager->initialize()) {
        setProcessError();
        return;
    }
    if (!ensureStartupConfiguration()) {
        setProcessError();
        return;
    }
    if (!m_machineSettingsStore->initialize()) {
        setProcessError();
        return;
    }

    // A full listener registry would leave a control silently dead, which is
    // worse than refusing to start. m_menu binds its own hotkeys in its
    // constructor and reports the same condition through buttonsBound().
    bool registered = m_menu.buttonsBound();

    registered &= m_mouseRightButtonChannel->addStateListenerCallback(
        this, &UserInterfaceModule::onMouseRightButtonStateChanged);
    registered &= m_mouseLeftButtonChannel->addStateListenerCallback(
        this, &UserInterfaceModule::onMouseLeftButtonStateChanged);
    registered &= m_controlPanelButtons.setup->addPressListenerCallback(
        this, &UserInterfaceModule::onSetupPressed);
    registered &= m_controlPanelButtons.test->addPressListenerCallback(
        this, &UserInterfaceModule::onTestPressed);
    registered &= m_controlPanelButtons.reset->addPressListenerCallback(
        this, &UserInterfaceModule::onResetPressed);
    registered &= m_controlPanelButtons.clampOpen->addPressListenerCallback(
        this, &UserInterfaceModule::onClampOpenPressed);
    registered &= m_controlPanelButtons.light->addPressListenerCallback(
        this, &UserInterfaceModule::onLightPressed);
    registered &= m_controlPanelButtons.manual->addPressListenerCallback(
        this, &UserInterfaceModule::onManualPressed);

    if (!registered) {
        setProcessError();
        return;
    }

    m_pageRenderer.start();
    publishActiveConfiguration(true);
    publishMachineSettings();

    // Land on the selector rather than the parameter page. Unlike an
    // operator-triggered SelectionStarted, this must not fire
    // Event::ConfigurationSelectionStarted: that would flip
    // m_configurationConfirmed back off in Robot and defeat the
    // just-published confirmation above. The active configuration is
    // already loaded and confirmed; this is only what's on screen.
    m_selectionProtocol = m_activeConfiguration->bondingMode;
    fillConfigurationList();
    m_menu.showPage(m_menu.configurationSelectPage());
}

void UserInterfaceModule::onStop()
{
    m_pageRenderer.stop();
    m_menu.unbindButtons();
    if (m_mouseRightButtonChannel != nullptr) {
        m_mouseRightButtonChannel->removeStateListenerCallback(this, &UserInterfaceModule::onMouseRightButtonStateChanged);
    }
    if (m_mouseLeftButtonChannel != nullptr) {
        m_mouseLeftButtonChannel->removeStateListenerCallback(this, &UserInterfaceModule::onMouseLeftButtonStateChanged);
    }

    auto remove = [this](ButtonChannel *button,
                         ButtonChannel::PressCallback callback) {
        if (button != nullptr) {
            button->removePressListenerCallback(this, callback);
        }
    };
    remove(m_controlPanelButtons.setup,
           &UserInterfaceModule::onSetupPressed);
    remove(m_controlPanelButtons.test,
           &UserInterfaceModule::onTestPressed);
    remove(m_controlPanelButtons.reset,
           &UserInterfaceModule::onResetPressed);
    remove(m_controlPanelButtons.clampOpen,
           &UserInterfaceModule::onClampOpenPressed);
    remove(m_controlPanelButtons.light,
           &UserInterfaceModule::onLightPressed);
    remove(m_controlPanelButtons.manual,
           &UserInterfaceModule::onManualPressed);
}

void UserInterfaceModule::onExecute()
{
    m_menu.execute();
    m_pageRenderer.execute();
}

// -----------------------------------------------------------------------------
// Configuration handling
// -----------------------------------------------------------------------------

// Guarantees at least one stored configuration and loads the first one as
// the RAM working copy.
bool UserInterfaceModule::ensureStartupConfiguration()
{
    if (m_configurationManager->count() == 0U) {
        BonderConfig defaults{};
        if (!m_configurationManager->add(kDefaultConfigurationName, defaults)) return false;
    }
    const char *name = m_configurationManager->nameAt(0U);
    if (name == nullptr ||
        !m_configurationManager->load(name, *m_activeConfiguration)) {
        return false;
    }
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    return true;
}

// Pushes the working copy to the menu, the manual-mode LED and the robot.
void UserInterfaceModule::publishActiveConfiguration(bool confirmed)
{
    m_menu.setConfiguration(*m_activeConfiguration);
    m_menu.setActiveConfigurationName(m_activeConfigurationName);
    if (m_manualModeLed != nullptr) {
        m_manualModeLed->set(
            m_activeConfiguration->bondingMode == BondingMode::Manual);
    }
    fireEvent(Event::ActiveConfigurationChanged);
    if (confirmed) {
        fireEvent(Event::ConfigurationConfirmed);
    }
}

void UserInterfaceModule::onMenuRequest_(void *ctx,
                                         const Menu::Request& request)
{
    static_cast<UserInterfaceModule *>(ctx)->handleMenuRequest(request);
}

void UserInterfaceModule::handleMenuRequest(const Menu::Request& request)
{
    switch (request.type) {
    case Menu::RequestType::AdjustParameter:
        adjustParameter(request.screen, request.parameterIndex, request.sign,
                        request.stepScale);
        break;
    case Menu::RequestType::AdjustHotkey:
        adjustHotkey(request.hotkey, request.sign, request.stepScale,
                     request.isRepeat);
        break;
    case Menu::RequestType::SaveConfiguration:
        saveConfiguration();
        break;
    case Menu::RequestType::SaveConfigurationAs:
        saveConfigurationAs(request.name);
        break;
    case Menu::RequestType::SelectionStarted:
        beginSelection();
        break;
    case Menu::RequestType::SelectionProtocolChanged:
        changeSelectionProtocol(request.sign);
        break;
    case Menu::RequestType::LoadConfiguration:
        loadConfiguration(request.name);
        break;
    case Menu::RequestType::AddConfiguration:
        addConfiguration(request.name);
        break;
    case Menu::RequestType::DeleteConfiguration:
        deleteConfiguration(request.name);
        break;
    case Menu::RequestType::AdjustSetting:
        adjustSetting(request.parameterIndex, request.sign, request.stepScale);
        break;
    case Menu::RequestType::SaveSettings:
        saveSettings();
        break;
    case Menu::RequestType::ToggleSpotlight:
        toggleSpotlight();
        break;
    case Menu::RequestType::StartZPositionCal:
        fireEvent(Event::StartZPositionCalRequested);
        break;

    case Menu::RequestType::StartTachCal:
        fireEvent(Event::StartTachCalRequested);
        break;
    case Menu::RequestType::SaveForceMeasurement:
        saveForceMeasurement(request.value);
        break;
    }
}

// stepScale is what the hold duration asked for; the catalog cuts it down to
// what this parameter's range can absorb, so a short-range value cannot be
// slammed between its limits by holding the key.
void UserInterfaceModule::adjustByDescriptor(
    const ConfigurationParameterCatalog::Descriptor *descriptor,
    int8_t sign,
    uint16_t stepScale)
{
    if (descriptor == nullptr) return;

    const uint16_t scale =
        ConfigurationParameterCatalog::limitStepScale(stepScale, descriptor);
    const float delta = static_cast<float>(sign) * static_cast<float>(scale) *
                        descriptor->stepDisplay;

    uint8_t *base = reinterpret_cast<uint8_t *>(m_activeConfiguration);

    if (descriptor->isInteger) {
        uint16_t value;
        std::memcpy(&value, base + descriptor->offset, sizeof(value));
        float changed = static_cast<float>(value) + delta;
        if (changed < descriptor->minDisplay) changed = descriptor->minDisplay;
        if (changed > descriptor->maxDisplay) changed = descriptor->maxDisplay;
        value = static_cast<uint16_t>(changed + 0.5f);
        std::memcpy(base + descriptor->offset, &value, sizeof(value));
    } else {
        float value;
        std::memcpy(&value, base + descriptor->offset, sizeof(value));
        float changed = value * descriptor->scale + descriptor->displayOffset + delta;
        if (changed < descriptor->minDisplay) changed = descriptor->minDisplay;
        if (changed > descriptor->maxDisplay) changed = descriptor->maxDisplay;
        value = (changed - descriptor->displayOffset) / descriptor->scale;
        std::memcpy(base + descriptor->offset, &value, sizeof(value));
    }

    publishActiveConfiguration(false);
}

// stepScale is the multiplier the menu derived from how long the key has been
// held; it is 1 for an ordinary press.
void UserInterfaceModule::adjustParameter(uint8_t screen,
                                          uint8_t parameterIndex,
                                          int8_t sign,
                                          uint16_t stepScale)
{
    const ConfigurationParameterCatalog::Descriptor *descriptor =
        ConfigurationParameterCatalog::at(
            screen, parameterIndex, m_activeConfiguration->bondingMode);
    if (descriptor == nullptr) return;
    adjustByDescriptor(descriptor, sign, stepScale);
}

void UserInterfaceModule::adjustHotkey(Menu::Hotkey hotkey, int8_t sign,
                                       uint16_t stepScale, bool isRepeat)
{
    const uint16_t offset =
        hotkeyParameterOffset(hotkey, m_activeConfiguration->bondingMode);
    const ConfigurationParameterCatalog::Descriptor *descriptor =
        ConfigurationParameterCatalog::byOffset(offset);
    if (!ConfigurationParameterCatalog::isAvailable(
            descriptor, m_activeConfiguration->bondingMode)) {
        // Only the press reports it. Raising the message on every repeat would
        // loop: the message suppresses repeats, times out, then the next
        // repeat raises it again.
        if (!isRepeat) m_menu.setNotificationMessage("Not used by protocol");
        return;
    }
    adjustByDescriptor(descriptor, sign, stepScale);
}

void UserInterfaceModule::saveConfiguration()
{
    if (std::strcmp(m_activeConfigurationName, kDefaultConfigurationName) == 0) {
        m_menu.promptSaveAsName();
        return;
    }
    if (m_configurationManager->save(m_activeConfigurationName,
                                     *m_activeConfiguration)) {
        m_menu.setNotificationMessage("Configuration saved");
    } else {
        m_menu.setWarningMessage("Save failed");
    }
}

// Persists the current parameter-page edits under a new name, unlike
// addConfiguration() which creates a fresh blank record.
void UserInterfaceModule::saveConfigurationAs(const char *name)
{
    if (name == nullptr ||
        !m_configurationManager->add(name, *m_activeConfiguration)) {
        m_menu.setWarningMessage("Save failed");
        return;
    }
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    publishActiveConfiguration(true);
    m_menu.setNotificationMessage("Configuration saved");
}

void UserInterfaceModule::adjustSetting(uint8_t index, int8_t sign,
                                        uint16_t stepScale)
{
    // Only the numeric rows are +/- editable; the action rows below them are
    // filtered out by Menu::adjustSetting() before they get here.
    if (index >= Menu::kSettingsLevelRowCount) return;

    MachineSettingsData& data = m_machineSettingsStore->mutableData();

    // Row order must match kSettingLimits and Menu's m_settingsValues.
    float *const fields[Menu::kSettingsLevelRowCount] = {
        &data.clampSolenoidVoltage,
        &data.areaLightLevel,
        &data.spotlightLevel,
        &data.zMotorMaxUpwardSpeed,
        &data.zMotorMaxDownwardSpeed,
        &data.forceSetupTrackingForce};

    float *field = fields[index];
    const SettingLimits& limits = kSettingLimits[index];

    const uint16_t scale = ConfigurationParameterCatalog::limitStepScale(
        stepScale, limits.min, limits.max, limits.step);

    float value = *field +
        static_cast<float>(sign) * static_cast<float>(scale) * limits.step;
    if (value < limits.min) value = limits.min;
    if (value > limits.max) value = limits.max;
    *field = value;

    publishMachineSettings();
}

void UserInterfaceModule::promptForceMeasurement()
{
    // Seed the field with the force that was actually commanded, so leaving
    // it untouched means "the gauge agreed" — a zero offset.
    m_menu.promptForceMeasurement(
        m_machineSettingsStore->data().forceSetupTrackingForce);
}

void UserInterfaceModule::saveForceMeasurement(float measuredGrams)
{
    MachineSettingsData& data = m_machineSettingsStore->mutableData();

    // What the machine actually delivers minus what it was told to deliver;
    // BonderModule subtracts this from every non-zero force command.
    data.forceCoilForceOffset =
        measuredGrams - data.forceSetupTrackingForce;

    if (m_machineSettingsStore->save()) {
        m_menu.setNotificationMessage("Force offset saved");
    } else {
        m_menu.setWarningMessage("Save failed");
    }
    publishMachineSettings();
}

void UserInterfaceModule::toggleSpotlight()
{
    MachineSettingsData& data = m_machineSettingsStore->mutableData();
    data.spotlightOn = !data.spotlightOn;
    publishMachineSettings();
}

void UserInterfaceModule::saveSettings()
{
    if (m_machineSettingsStore->save()) {
        m_menu.setNotificationMessage("Settings saved");
    } else {
        m_menu.setWarningMessage("Save failed");
    }
}

void UserInterfaceModule::publishMachineSettings()
{
    m_menu.setSettingsValues(m_machineSettingsStore->data());
    fireEvent(Event::MachineSettingsChanged);
}

void UserInterfaceModule::beginSelection()
{
    m_selectionProtocol = m_activeConfiguration->bondingMode;
    fillConfigurationList();
    fireEvent(Event::ConfigurationSelectionStarted);
}

void UserInterfaceModule::changeSelectionProtocol(int8_t direction)
{
    int protocol = static_cast<int>(m_selectionProtocol) + direction;
    while (protocol < 0) protocol += kProtocolCount;
    while (protocol >= kProtocolCount) protocol -= kProtocolCount;
    m_selectionProtocol = static_cast<BondingMode>(protocol);
    fillConfigurationList();
}

void UserInterfaceModule::fillConfigurationList()
{
    uint8_t row = 0U;
    for (uint16_t i = 0U; i < m_configurationManager->count() &&
                          row < Menu::kConfigurationRowCount; ++i) {
        const char *name = m_configurationManager->nameAt(i);
        BonderConfig config{};
        if (name == nullptr ||
            !m_configurationManager->load(name, config) ||
            config.bondingMode != m_selectionProtocol) {
            continue;
        }
        m_menu.setConfigurationRow(row++, name);
    }
    for (; row < Menu::kConfigurationRowCount; ++row) {
        m_menu.clearConfigurationRow(row);
    }
    m_menu.setConfigurationSelectHeader(
        bondingModeName(m_selectionProtocol),
        static_cast<uint8_t>(static_cast<uint8_t>(m_selectionProtocol) + 1U),
        kProtocolCount);
}

void UserInterfaceModule::loadConfiguration(const char *name)
{
    BonderConfig config{};
    if (name == nullptr || !m_configurationManager->load(name, config)) {
        m_menu.setWarningMessage("Load failed");
        return;
    }
    *m_activeConfiguration = config;
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    publishActiveConfiguration(true);
}

void UserInterfaceModule::addConfiguration(const char *name)
{
    BonderConfig config{};
    config.bondingMode = m_selectionProtocol;
    if (name == nullptr || !m_configurationManager->add(name, config)) {
        m_menu.setWarningMessage("Add failed");
        return;
    }
    *m_activeConfiguration = config;
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    publishActiveConfiguration(true);
    m_menu.setNotificationMessage("Configuration added");
}

void UserInterfaceModule::deleteConfiguration(const char *name)
{
    if (name != nullptr &&
        std::strcmp(name, kDefaultConfigurationName) == 0) {
        m_menu.setWarningMessage("Cannot delete DEFAULT");
        return;
    }
    if (name == nullptr || !m_configurationManager->remove(name)) {
        m_menu.setWarningMessage("Delete failed");
    }
    // The RAM working copy stays valid even when its record was deleted.
    fillConfigurationList();
}

// -----------------------------------------------------------------------------
// Operator feedback
// -----------------------------------------------------------------------------

void UserInterfaceModule::notifyUser(const char *message)
{
    m_menu.setNotificationMessage(message);
}

void UserInterfaceModule::warnUser(const char *message)
{
    m_menu.setWarningMessage(message);
}

void UserInterfaceModule::raiseError(const char *message)
{
    m_menu.setErrorMessage(message);
}

// -----------------------------------------------------------------------------
// Mouse and control panel forwarding
// -----------------------------------------------------------------------------

void UserInterfaceModule::onMouseRightButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    UserInterfaceModule *self = static_cast<UserInterfaceModule *>(ctx);
    self->m_mouseButtonCallbacks.invoke(
        state == PinMonitorChannel::PinState::ACTIVE
            ? MouseButtonEvent::RightPressed
            : MouseButtonEvent::RightReleased);
}

void UserInterfaceModule::onMouseLeftButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    UserInterfaceModule *self = static_cast<UserInterfaceModule *>(ctx);
    self->m_mouseButtonCallbacks.invoke(
        state == PinMonitorChannel::PinState::ACTIVE
            ? MouseButtonEvent::LeftPressed
            : MouseButtonEvent::LeftReleased);
}

void UserInterfaceModule::onSetupPressed(void *ctx)
{
    static_cast<UserInterfaceModule *>(ctx)->fireControlPanelButtonEvent(
        ControlPanelButtonEvent::SetupPressed);
}

void UserInterfaceModule::onTestPressed(void *ctx)
{
    static_cast<UserInterfaceModule *>(ctx)->fireControlPanelButtonEvent(
        ControlPanelButtonEvent::TestPressed);
}

void UserInterfaceModule::onResetPressed(void *ctx)
{
    static_cast<UserInterfaceModule *>(ctx)->fireControlPanelButtonEvent(
        ControlPanelButtonEvent::ResetPressed);
}

void UserInterfaceModule::onClampOpenPressed(void *ctx)
{
    static_cast<UserInterfaceModule *>(ctx)->fireControlPanelButtonEvent(
        ControlPanelButtonEvent::ClampOpenPressed);
}

void UserInterfaceModule::onLightPressed(void *ctx)
{
    static_cast<UserInterfaceModule *>(ctx)->fireControlPanelButtonEvent(
        ControlPanelButtonEvent::LightPressed);
}

// Puts the working configuration into manual bonding mode. Publishing lights
// the manual LED and hands the new mode to the Robot, which picks the
// protocol from it at the next start.
void UserInterfaceModule::onManualPressed(void *ctx)
{
    UserInterfaceModule *self = static_cast<UserInterfaceModule *>(ctx);
    if (self->m_activeConfiguration->bondingMode == BondingMode::Manual) {
        return;
    }

    self->m_activeConfiguration->bondingMode = BondingMode::Manual;
    self->publishActiveConfiguration(true);
}
