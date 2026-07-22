#include "user_interface_module.hpp"
#include "configuration.h"
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace {

constexpr float kEditIncrement = 0.01f;
constexpr uint8_t kProtocolCount = 4U;

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
            : offsetof(BonderConfig, tailPosition);
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
    const Menu::NavigationButtons& navigationButtons,
    const Menu::ConfigurationButtons& configurationButtons,
    LedChannel *manualModeLed,
    PinMonitorChannel *mouseRightButtonChannel,
    PinMonitorChannel *mouseLeftButtonChannel,
    const ControlPanelButtons& controlPanelButtons)
    : m_configurationManager(configurationManager)
    , m_pageRenderer(lcdController)
    , m_menu(&m_pageRenderer, navigationButtons, configurationButtons)
    , m_manualModeLed(manualModeLed)
    , m_mouseRightButtonChannel(mouseRightButtonChannel)
    , m_mouseLeftButtonChannel(mouseLeftButtonChannel)
    , m_controlPanelButtons(controlPanelButtons)
    , m_activeConfiguration{}
    , m_activeConfigurationName{}
    , m_selectionProtocol(BondingMode::SemiAutomatic)
    , m_eventCallback(nullptr)
    , m_eventCallbackCtx(nullptr)
    , m_mouseButtonCallback(nullptr)
    , m_mouseButtonCallbackCtx(nullptr)
    , m_controlPanelButtonCallback(nullptr)
    , m_controlPanelButtonCallbackCtx(nullptr)
{
    m_menu.setRequestListenerCallback(this, onMenuRequest_);
}

void UserInterfaceModule::setEventListenerCallback(void *ctx,
                                                  EventCallback callback)
{
    m_eventCallbackCtx = ctx;
    m_eventCallback = callback;
}

void UserInterfaceModule::setMouseButtonListenerCallback(
    void *ctx, MouseButtonCallback callback)
{
    m_mouseButtonCallbackCtx = ctx;
    m_mouseButtonCallback = callback;
}

void UserInterfaceModule::setControlPanelButtonListenerCallback(
    void *ctx, ControlPanelButtonCallback callback)
{
    m_controlPanelButtonCallbackCtx = ctx;
    m_controlPanelButtonCallback = callback;
}

void UserInterfaceModule::fireEvent(Event event)
{
    if (m_eventCallback != nullptr) {
        m_eventCallback(m_eventCallbackCtx, event);
    }
}

void UserInterfaceModule::fireControlPanelButtonEvent(
    ControlPanelButtonEvent event)
{
    if (m_controlPanelButtonCallback != nullptr) {
        m_controlPanelButtonCallback(m_controlPanelButtonCallbackCtx, event);
    }
}

// -----------------------------------------------------------------------------
// Process lifecycle
// -----------------------------------------------------------------------------

void UserInterfaceModule::onStart()
{
    if (m_configurationManager == nullptr ||
        m_mouseRightButtonChannel == nullptr ||
        m_mouseLeftButtonChannel == nullptr ||
        m_controlPanelButtons.setup == nullptr ||
        m_controlPanelButtons.test == nullptr ||
        m_controlPanelButtons.reset == nullptr ||
        m_controlPanelButtons.clampOpen == nullptr ||
        m_controlPanelButtons.light == nullptr) {
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

    m_mouseRightButtonChannel->addStateListenerCallback(
        this, &UserInterfaceModule::onMouseRightButtonStateChanged);
    m_mouseLeftButtonChannel->addStateListenerCallback(
        this, &UserInterfaceModule::onMouseLeftButtonStateChanged);
    m_controlPanelButtons.setup->addPressListenerCallback(
        this, &UserInterfaceModule::onSetupPressed);
    m_controlPanelButtons.test->addPressListenerCallback(
        this, &UserInterfaceModule::onTestPressed);
    m_controlPanelButtons.reset->addPressListenerCallback(
        this, &UserInterfaceModule::onResetPressed);
    m_controlPanelButtons.clampOpen->addPressListenerCallback(
        this, &UserInterfaceModule::onClampOpenPressed);
    m_controlPanelButtons.light->addPressListenerCallback(
        this, &UserInterfaceModule::onLightPressed);

    m_pageRenderer.start();
    publishActiveConfiguration(false);
    m_menu.showPage(m_menu.parameterPage());
}

void UserInterfaceModule::onStop()
{
    m_pageRenderer.stop();
    if (m_mouseRightButtonChannel != nullptr) {
        m_mouseRightButtonChannel->addStateListenerCallback(nullptr, nullptr);
    }
    if (m_mouseLeftButtonChannel != nullptr) {
        m_mouseLeftButtonChannel->addStateListenerCallback(nullptr, nullptr);
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
        if (!m_configurationManager->add("DEFAULT", defaults)) return false;
    }
    const char *name = m_configurationManager->nameAt(0U);
    if (name == nullptr ||
        !m_configurationManager->load(name, m_activeConfiguration)) {
        return false;
    }
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    return true;
}

// Pushes the working copy to the menu, the manual-mode LED and the robot.
void UserInterfaceModule::publishActiveConfiguration(bool confirmed)
{
    m_menu.setConfiguration(m_activeConfiguration);
    m_menu.setActiveConfigurationName(m_activeConfigurationName);
    if (m_manualModeLed != nullptr) {
        m_manualModeLed->set(
            m_activeConfiguration.bondingMode == BondingMode::Manual);
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
        adjustParameter(request.screen, request.parameterIndex, request.sign);
        break;
    case Menu::RequestType::AdjustHotkey:
        adjustHotkey(request.hotkey, request.sign);
        break;
    case Menu::RequestType::SaveConfiguration:
        saveConfiguration();
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
    }
}

void UserInterfaceModule::adjustByDescriptor(
    const ConfigurationParameterCatalog::Descriptor *descriptor,
    float delta)
{
    if (descriptor == nullptr) return;
    uint8_t *base = reinterpret_cast<uint8_t *>(&m_activeConfiguration);

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
        float changed = value * descriptor->scale + delta;
        if (changed < descriptor->minDisplay) changed = descriptor->minDisplay;
        if (changed > descriptor->maxDisplay) changed = descriptor->maxDisplay;
        value = changed / descriptor->scale;
        std::memcpy(base + descriptor->offset, &value, sizeof(value));
    }

    publishActiveConfiguration(false);
}

void UserInterfaceModule::adjustParameter(uint8_t screen,
                                          uint8_t parameterIndex,
                                          int8_t sign)
{
    const ConfigurationParameterCatalog::Descriptor *descriptor =
        ConfigurationParameterCatalog::at(
            screen, parameterIndex, m_activeConfiguration.bondingMode);
    if (descriptor == nullptr) return;
    adjustByDescriptor(
        descriptor,
        static_cast<float>(sign) *
            (descriptor->isInteger ? 1.0f : kEditIncrement));
}

void UserInterfaceModule::adjustHotkey(Menu::Hotkey hotkey, int8_t sign)
{
    const uint16_t offset =
        hotkeyParameterOffset(hotkey, m_activeConfiguration.bondingMode);
    const ConfigurationParameterCatalog::Descriptor *descriptor =
        ConfigurationParameterCatalog::byOffset(offset);
    if (!ConfigurationParameterCatalog::isAvailable(
            descriptor, m_activeConfiguration.bondingMode)) {
        m_menu.setNotificationMessage("Not used by protocol");
        return;
    }
    adjustByDescriptor(descriptor, static_cast<float>(sign) * kEditIncrement);
}

void UserInterfaceModule::saveConfiguration()
{
    if (m_configurationManager->save(m_activeConfigurationName,
                                     m_activeConfiguration)) {
        m_menu.setNotificationMessage("Configuration saved");
    } else {
        m_menu.setWarningMessage("Save failed");
    }
}

void UserInterfaceModule::beginSelection()
{
    m_selectionProtocol = m_activeConfiguration.bondingMode;
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
    m_activeConfiguration = config;
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
    m_activeConfiguration = config;
    snprintf(m_activeConfigurationName, sizeof(m_activeConfigurationName),
             "%s", name);
    publishActiveConfiguration(true);
    m_menu.setNotificationMessage("Configuration added");
}

void UserInterfaceModule::deleteConfiguration(const char *name)
{
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

void UserInterfaceModule::ultrasonicInfo(float resonanceFrequency,
                                         float qualityFactor,
                                         float transferredPower,
                                         float bondingDuration)
{
    char rows[4][21];
    const uint32_t frequencyHz =
        static_cast<uint32_t>(resonanceFrequency + 0.5f);
    const uint32_t qualityTenths =
        static_cast<uint32_t>(qualityFactor * 10.0f + 0.5f);
    const uint32_t powerTenths =
        static_cast<uint32_t>(transferredPower * 10.0f + 0.5f);
    const uint32_t durationMilliseconds =
        static_cast<uint32_t>(bondingDuration * 1000.0f + 0.5f);

    snprintf(rows[0], sizeof(rows[0]), "Freq:%lu.%03lu kHz",
             static_cast<unsigned long>(frequencyHz / 1000U),
             static_cast<unsigned long>(frequencyHz % 1000U));
    snprintf(rows[1], sizeof(rows[1]), "Q:%lu.%lu",
             static_cast<unsigned long>(qualityTenths / 10U),
             static_cast<unsigned long>(qualityTenths % 10U));
    snprintf(rows[2], sizeof(rows[2]), "Power:%lu.%lu W",
             static_cast<unsigned long>(powerTenths / 10U),
             static_cast<unsigned long>(powerTenths % 10U));
    snprintf(rows[3], sizeof(rows[3]), "Time:%lu.%03lu s",
             static_cast<unsigned long>(durationMilliseconds / 1000U),
             static_cast<unsigned long>(durationMilliseconds % 1000U));

    const char *reportRows[Menu::kMessageRowCount] = {
        rows[0], rows[1], rows[2], rows[3]
    };
    
    m_menu.setReportMessage(
        reportRows, USER_INTERFACE_MODULE_REPORT_DURATION_MS);
}

// -----------------------------------------------------------------------------
// Mouse and control panel forwarding
// -----------------------------------------------------------------------------

void UserInterfaceModule::onMouseRightButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    UserInterfaceModule *self = static_cast<UserInterfaceModule *>(ctx);
    if (self->m_mouseButtonCallback == nullptr) return;
    self->m_mouseButtonCallback(
        self->m_mouseButtonCallbackCtx,
        state == PinMonitorChannel::PinState::ACTIVE
            ? MouseButtonEvent::RightPressed
            : MouseButtonEvent::RightReleased);
}

void UserInterfaceModule::onMouseLeftButtonStateChanged(
    void *ctx, PinMonitorChannel::PinState state)
{
    UserInterfaceModule *self = static_cast<UserInterfaceModule *>(ctx);
    if (self->m_mouseButtonCallback == nullptr) return;
    self->m_mouseButtonCallback(
        self->m_mouseButtonCallbackCtx,
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
