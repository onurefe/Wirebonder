#ifndef DEBUG_ENVIRONMENT_STEPPER_ROUTER_HPP
#define DEBUG_ENVIRONMENT_STEPPER_ROUTER_HPP

#include <cstdint>

#include "configuration.h"
#include "DebugEnvironment/debug_environment.hpp"
#include "fast_io.hpp"
#include "stepper_service.hpp"
#include "stepper_router_service.hpp"

// Sandbox for the Y/T stepper axes: StepperService + router channels, driven
// by absolute move commands. No homing — positions are relative to power-on.
class StepperRouterDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_STEPPER_ROUTER;

    enum Command : uint16_t {
        CMD_MOVE = 1,
        CMD_STOP = 2
    };

    enum Axis : uint32_t {
        AXIS_Y = 0,
        AXIS_T = 1
    };

    enum ResultCode : uint32_t {
        RESULT_MOVE_COMPLETE = 0,
        RESULT_STOPPED = 1
    };

    StepperRouterDebugEnvironment();

protected:
    uint16_t environmentId() const override
    {
        return EnvironmentId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;

    void onPoll() override;

    bool isBusy() const override
    {
        return m_moveActive;
    }

    void abort() override;

private:
    RouterChannel *routerForAxis(uint32_t axis) const;

    void startMove();
    void stopMove(uint32_t resultCode);

    // Hardware sandbox — exclusively owned by this environment.
    static FastIO m_stepperEnablePin;
    static FastIO m_stepperResetPin;
    static FastIO m_yAxisStepPin;
    static FastIO m_yAxisDirPin;
    static FastIO m_tAxisStepPin;
    static FastIO m_tAxisDirPin;

    static StepperService m_stepperService;
    static StepperChannel m_yAxisStepperChannel;
    static StepperChannel m_tAxisStepperChannel;
    static RouterChannel m_yAxisRouterChannel;
    static RouterChannel m_tAxisRouterChannel;
    static StepperRouterService m_routerService;

    RouterChannel *m_activeRouter = nullptr;
    uint32_t m_activeAxis = AXIS_Y;
    bool m_moveActive = false;
};

#endif /* DEBUG_ENVIRONMENT_STEPPER_ROUTER_HPP */
