#include "debug_stepper_router.hpp"

DebugStepperRouter::DebugStepperRouter()
{
}

void DebugStepperRouter::init(RouterChannel *yAxisRouter, RouterChannel *tAxisRouter)
{
    m_yAxisRouter = yAxisRouter;
    m_tAxisRouter = tAxisRouter;
}

void DebugStepperRouter::handleCommand(uint16_t localCommand)
{
    switch (localCommand) {
    case CMD_MOVE:
        move();
        break;

    case CMD_STOP:
        stop(RESULT_STOPPED);
        break;

    default:
        setError(ERROR_UNSUPPORTED_COMMAND);
        break;
    }
}

void DebugStepperRouter::poll()
{
    if (!m_busy) {
        return;
    }

    if (m_activeRouter == nullptr || !m_activeRouter->isBusy()) {
        m_busy = false;
        m_activeRouter = nullptr;
        setDone(RESULT_MOVE_COMPLETE, m_activeAxis);
    }
}

void DebugStepperRouter::abort()
{
    stop(RESULT_STOPPED);
}

RouterChannel *DebugStepperRouter::routerForAxis(uint32_t axis) const
{
    if (axis == AXIS_Y) {
        return m_yAxisRouter;
    }

    if (axis == AXIS_T) {
        return m_tAxisRouter;
    }

    return nullptr;
}

void DebugStepperRouter::move()
{
    const uint32_t axis = static_cast<uint32_t>(arg(0));
    const float targetPositionMm = arg(1);
    RouterChannel *router = routerForAxis(axis);

    if (router == nullptr) {
        setError(ERROR_INVALID_ARGUMENT);
        return;
    }

    if (m_busy || router->isBusy()) {
        setError(DEBUG_SERVICE_ERROR_BUSY);
        return;
    }

    m_activeAxis = axis;
    m_activeRouter = router;
    m_busy = true;

    setBusy();
    router->moveTo(targetPositionMm);
}

void DebugStepperRouter::stop(uint32_t resultCode)
{
    if (m_activeRouter != nullptr) {
        m_activeRouter->stop();
    }

    m_busy = false;
    m_activeRouter = nullptr;
    setDone(resultCode, m_activeAxis);
}
