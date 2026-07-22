#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_STEPPER_ROUTER

#include "debug_environment_stepper_router.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim6;

// -----------------------------------------------------------------------------
// Static member definitions — same wiring the Robot uses for the stepper
// axes, but owned here outright.
// -----------------------------------------------------------------------------

FastIO StepperRouterDebugEnvironment::m_stepperEnablePin(STEPPER_EN_GPIO_Port,    STEPPER_EN_Pin,    FALSE);
FastIO StepperRouterDebugEnvironment::m_stepperResetPin (STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin, FALSE);

FastIO StepperRouterDebugEnvironment::m_yAxisStepPin(STEPPER_Y_STEP_GPIO_Port,    STEPPER_Y_STEP_Pin,    FALSE);
FastIO StepperRouterDebugEnvironment::m_yAxisDirPin (STEPPER_Y_DIR_GPIO_Port,     STEPPER_Y_DIR_Pin,     FALSE);
FastIO StepperRouterDebugEnvironment::m_tAxisStepPin(STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin, FALSE);
FastIO StepperRouterDebugEnvironment::m_tAxisDirPin (STEPPER_TEAR_DIR_GPIO_Port,  STEPPER_TEAR_DIR_Pin,  FALSE);

StepperService StepperRouterDebugEnvironment::m_stepperService(
    &htim6,
    &StepperRouterDebugEnvironment::m_stepperEnablePin,
    &StepperRouterDebugEnvironment::m_stepperResetPin);

StepperChannel StepperRouterDebugEnvironment::m_yAxisStepperChannel(
    &StepperRouterDebugEnvironment::m_yAxisStepPin,
    &StepperRouterDebugEnvironment::m_yAxisDirPin);

StepperChannel StepperRouterDebugEnvironment::m_tAxisStepperChannel(
    &StepperRouterDebugEnvironment::m_tAxisStepPin,
    &StepperRouterDebugEnvironment::m_tAxisDirPin);

RouterChannel StepperRouterDebugEnvironment::m_yAxisRouterChannel(
    &StepperRouterDebugEnvironment::m_yAxisStepperChannel,
    ROBOT_Y_AXIS_MAX_VELOCITY,
    ROBOT_Y_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_Y_AXIS_STEPS_PER_MM);

RouterChannel StepperRouterDebugEnvironment::m_tAxisRouterChannel(
    &StepperRouterDebugEnvironment::m_tAxisStepperChannel,
    ROBOT_T_AXIS_MAX_VELOCITY,
    ROBOT_T_AXIS_MAX_ACCELERATION,
    ROUTER_MODULE_T_AXIS_STEPS_PER_MM);

StepperRouterService StepperRouterDebugEnvironment::m_routerService;

StepperRouterDebugEnvironment::StepperRouterDebugEnvironment()
{
    m_stepperService.addChannel(&m_yAxisStepperChannel);
    m_stepperService.addChannel(&m_tAxisStepperChannel);

    m_routerService.addChannel(&m_yAxisRouterChannel);
    m_routerService.addChannel(&m_tAxisRouterChannel);

    addProcess(&m_stepperService);
    addProcess(&m_routerService);
}

void StepperRouterDebugEnvironment::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_MOVE:
        startMove();
        break;

    case CMD_STOP:
        stopMove(RESULT_STOPPED);
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void StepperRouterDebugEnvironment::onPoll()
{
    if (!m_moveActive) {
        return;
    }

    if (m_activeRouter == nullptr || !m_activeRouter->isBusy()) {
        m_moveActive = false;
        m_activeRouter = nullptr;
        setDone(RESULT_MOVE_COMPLETE, m_activeAxis);
    }
}

void StepperRouterDebugEnvironment::abort()
{
    stopMove(RESULT_STOPPED);
}

RouterChannel *StepperRouterDebugEnvironment::routerForAxis(uint32_t axis) const
{
    if (axis == AXIS_Y) {
        return &m_yAxisRouterChannel;
    }

    if (axis == AXIS_T) {
        return &m_tAxisRouterChannel;
    }

    return nullptr;
}

void StepperRouterDebugEnvironment::startMove()
{
    const uint32_t axis = static_cast<uint32_t>(arg(0));
    const float targetPositionMm = arg(1);
    RouterChannel *router = routerForAxis(axis);

    if (router == nullptr) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if (m_moveActive || router->isBusy()) {
        setError(ERROR_BUSY);
        return;
    }

    m_activeAxis = axis;
    m_activeRouter = router;
    m_moveActive = true;

    setBusy();
    router->moveTo(targetPositionMm);
}

void StepperRouterDebugEnvironment::stopMove(uint32_t resultCode)
{
    if (m_activeRouter != nullptr) {
        m_activeRouter->stop();
    }

    m_moveActive = false;
    m_activeRouter = nullptr;
    setDone(resultCode, m_activeAxis);
}

#endif // FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_STEPPER_ROUTER
