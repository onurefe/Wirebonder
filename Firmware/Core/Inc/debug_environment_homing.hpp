#ifndef DEBUG_ENVIRONMENT_HOMING_HPP
#define DEBUG_ENVIRONMENT_HOMING_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_environment.hpp"
#include "fast_io.hpp"
#include "homing_module.hpp"
#include "pin_monitor_service.hpp"
#include "stepper_service.hpp"
#include "stepper_router_service.hpp"
#include "timer_expire_service.hpp"

#define HOMING_DEBUG_LOG_DEPTH 32

struct HomingDebugEventRecord {
    volatile uint32_t tickMs;

    // HomingModule::Event value.
    volatile uint32_t event;
};

struct HomingDebugResult {
    volatile uint32_t count;
    HomingDebugEventRecord entries[HOMING_DEBUG_LOG_DEPTH];

    // Live limit-switch sample, refreshed every poll (1 = ACTIVE).
    volatile uint32_t limitActive;
};

// Sandbox for Y-axis homing bring-up: StepperService + router channel +
// PinMonitorService with the limit switch, driving the real HomingModule.
// HomingModule events are logged into a GDB-readable ring; commands home the
// axis or move it to the workspace center.
class HomingDebugEnvironment : public DebugEnvironment {
public:
    static constexpr uint16_t EnvironmentId = DEBUG_ENVIRONMENT_ID_HOMING;

    enum Command : uint16_t {
        CMD_HOME = 1,
        CMD_CENTER = 2,
        CMD_STOP = 3
    };

    enum ResultCode : uint32_t {
        RESULT_HOME_COMPLETE = 0,
        RESULT_CENTER_COMPLETE = 1,
        RESULT_STOPPED = 2,

        // Reported through DONE (not ERROR) so the event-log result pointer
        // survives; setError would clear it before the host reads the log.
        RESULT_HOME_FAILED = 3
    };

    enum EnvironmentError : uint32_t {
        ERROR_NOT_HOMED = 201,
        ERROR_HOMING_REJECTED = 202
    };

    HomingDebugEnvironment();

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
        return m_activeOperation != Operation::None;
    }

    void abort() override;

private:
    enum class Operation : uint8_t {
        None,
        Homing,
        Centering
    };

    static void onHomingEvent(void *context, HomingModule::Event event);

    void startHoming();
    void startCentering();
    void stopOperation();
    void logEvent(HomingModule::Event event);

    // Hardware sandbox — exclusively owned by this environment.
    static FastIO m_stepperEnablePin;
    static FastIO m_stepperResetPin;
    static FastIO m_yAxisStepPin;
    static FastIO m_yAxisDirPin;
    static FastIO m_yAxisLimitSwitchPin;

    static TimerExpireService m_timerExpireService;
    static Timer m_pinMonitorCriticalTimer;
    static Timer m_pinMonitorNormalTimer;

    static PinMonitorService m_pinMonitorService;
    static PinMonitorChannel m_yAxisLimitSwitchChannel;

    static StepperService m_stepperService;
    static StepperChannel m_yAxisStepperChannel;
    static RouterChannel m_yAxisRouterChannel;
    static StepperRouterService m_routerService;

    static HomingModule m_homingModule;

    HomingDebugResult m_result = {};
    Operation m_activeOperation = Operation::None;
    bool m_homingEventPending = false;
    HomingModule::Event m_lastHomingEvent = HomingModule::Event::Completed;
};

#endif /* DEBUG_ENVIRONMENT_HOMING_HPP */
