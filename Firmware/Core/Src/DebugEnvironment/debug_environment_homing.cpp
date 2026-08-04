#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_HOMING

#include "DebugEnvironment/debug_environment_homing.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim6;

// -----------------------------------------------------------------------------
// Static member definitions — the Robot's Y-axis homing plant, owned here
// outright.
// -----------------------------------------------------------------------------

FastIO HomingDebugEnvironment::m_stepperEnablePin(STEPPER_EN_GPIO_Port,    STEPPER_EN_Pin,    FALSE);
FastIO HomingDebugEnvironment::m_stepperResetPin (STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin, FALSE);

FastIO HomingDebugEnvironment::m_yAxisStepPin(STEPPER_Y_STEP_GPIO_Port, STEPPER_Y_STEP_Pin, FALSE);
FastIO HomingDebugEnvironment::m_yAxisDirPin (STEPPER_Y_DIR_GPIO_Port,  STEPPER_Y_DIR_Pin,  FALSE);

// Limit switch is behind an active-low optocoupler front-end.
FastIO HomingDebugEnvironment::m_yAxisLimitSwitchPin(
    CONTACT_SENSORS_YLIM_GPIO_Port, CONTACT_SENSORS_YLIM_Pin, FALSE);

TimerExpireService HomingDebugEnvironment::m_timerExpireService;
Timer HomingDebugEnvironment::m_pinMonitorCriticalTimer;
Timer HomingDebugEnvironment::m_pinMonitorNormalTimer;

PinMonitorService HomingDebugEnvironment::m_pinMonitorService(
    &HomingDebugEnvironment::m_pinMonitorCriticalTimer,
    &HomingDebugEnvironment::m_pinMonitorNormalTimer);

PinMonitorChannel HomingDebugEnvironment::m_yAxisLimitSwitchChannel(
    &HomingDebugEnvironment::m_yAxisLimitSwitchPin,
    PinMonitorChannel::Level::LOW);

StepperService HomingDebugEnvironment::m_stepperService(
    &htim6,
    &HomingDebugEnvironment::m_stepperEnablePin,
    &HomingDebugEnvironment::m_stepperResetPin);

StepperChannel HomingDebugEnvironment::m_yAxisStepperChannel(
    &HomingDebugEnvironment::m_yAxisStepPin,
    &HomingDebugEnvironment::m_yAxisDirPin);

RouterChannel HomingDebugEnvironment::m_yAxisRouterChannel(
    &HomingDebugEnvironment::m_yAxisStepperChannel,
    ROBOT_Y_AXIS_MAX_VELOCITY,
    ROBOT_Y_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_Y_AXIS_STEPS_PER_MM);

StepperRouterService HomingDebugEnvironment::m_routerService;

HomingModule HomingDebugEnvironment::m_homingModule(
    &HomingDebugEnvironment::m_yAxisRouterChannel,
    &HomingDebugEnvironment::m_yAxisLimitSwitchChannel);

HomingDebugEnvironment::HomingDebugEnvironment()
{
    m_timerExpireService.addTimer(&m_pinMonitorCriticalTimer, true);
    m_timerExpireService.addTimer(&m_pinMonitorNormalTimer,   false);

    m_pinMonitorService.addChannel(&m_yAxisLimitSwitchChannel, true,
                                   PIN_MONITOR_BLIND_REGION_MS);

    m_stepperService.addChannel(&m_yAxisStepperChannel);
    m_routerService.addChannel(&m_yAxisRouterChannel);

    m_homingModule.addEventListenerCallback(
        this, &HomingDebugEnvironment::onHomingEvent);

    addProcess(&m_timerExpireService);
    addProcess(&m_pinMonitorService);
    addProcess(&m_stepperService);
    addProcess(&m_routerService);
    addProcess(&m_homingModule);
}

void HomingDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_HOME:
        startHoming();
        break;

    case CMD_CENTER:
        startCentering();
        break;

    case CMD_STOP:
        stopOperation();
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void HomingDebugEnvironment::onPoll()
{
    m_result.limitActive =
        (m_yAxisLimitSwitchChannel.samplePinState() ==
         PinMonitorChannel::PinState::ACTIVE) ? 1U : 0U;

    switch (m_activeOperation) {
    case Operation::None:
        return;

    case Operation::Homing:
        if (!m_homingEventPending) {
            return;
        }
        m_homingEventPending = false;
        m_activeOperation = Operation::None;
        if (m_lastHomingEvent == HomingModule::Event::Completed) {
            setDone(RESULT_HOME_COMPLETE, m_result.count);
        } else {
            setDone(RESULT_HOME_FAILED, m_result.count);
        }
        return;

    case Operation::Centering:
        if (m_yAxisRouterChannel.isBusy()) {
            return;
        }
        m_activeOperation = Operation::None;
        setDone(RESULT_CENTER_COMPLETE, m_result.count);
        return;
    }
}

void HomingDebugEnvironment::abort()
{
    stopOperation();
}

void HomingDebugEnvironment::startHoming()
{
    m_homingEventPending = false;

    if (!m_homingModule.home()) {
        setError(ERROR_HOMING_REJECTED);
        return;
    }

    m_activeOperation = Operation::Homing;
    setBusy();
    setResultPointer(0, &m_result);
}

void HomingDebugEnvironment::startCentering()
{
    if (!m_homingModule.isHomed()) {
        setError(ERROR_NOT_HOMED);
        return;
    }

    if (m_yAxisRouterChannel.isBusy()) {
        setError(ERROR_BUSY);
        return;
    }

    m_activeOperation = Operation::Centering;
    setBusy();
    setResultPointer(0, &m_result);
    m_yAxisRouterChannel.moveTo(0.5f * ROBOT_Y_AXIS_WORKSPACE_SIZE_MM);
}

void HomingDebugEnvironment::stopOperation()
{
    if (m_activeOperation == Operation::Homing) {
        m_homingModule.abortHoming();
    } else {
        m_yAxisRouterChannel.stop();
    }

    m_homingEventPending = false;
    m_activeOperation = Operation::None;
    setResultPointer(0, &m_result);
    setDone(RESULT_STOPPED, m_result.count);
}

void HomingDebugEnvironment::logEvent(HomingModule::Event event)
{
    const uint32_t slot = m_result.count % HOMING_DEBUG_LOG_DEPTH;
    m_result.entries[slot].tickMs = HAL_GetTick();
    m_result.entries[slot].event = static_cast<uint32_t>(event);
    m_result.count = m_result.count + 1U;
}

void HomingDebugEnvironment::onHomingEvent(void *context,
                                           HomingModule::Event event)
{
    auto *self = static_cast<HomingDebugEnvironment *>(context);
    if (self == nullptr) {
        return;
    }

    self->logEvent(event);

    // Progress events only feed the log; the operation finishes on the
    // terminal events.
    if (event == HomingModule::Event::Completed ||
        event == HomingModule::Event::Failed) {
        self->m_lastHomingEvent = event;
        self->m_homingEventPending = true;
    }
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_HOMING
