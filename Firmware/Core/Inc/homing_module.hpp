#ifndef HOMING_MODULE_HPP
#define HOMING_MODULE_HPP

#include <atomic>
#include <cstdint>

#include "pin_monitor_service.hpp"
#include "stepper_router_service.hpp"
#include "process.hpp"

class HomingModule : public Process {
public:
    enum class Event : uint8_t {
        Completed,
        Failed,

        // Progress notifications, emitted on state entry. Listeners that
        // only care about the outcome should ignore these.
        ClearingLimit,
        SeekingLimit,
        BackingOff
    };

    using EventCallback = void (*)(void *context, Event event);

    HomingModule(RouterChannel *yAxisRouter,
                 PinMonitorChannel *yAxisLimitSwitch);

    void addEventListenerCallback(void *context, EventCallback callback);

    bool home();
    void abortHoming();

    bool isHomed() const;
    bool hasFailed() const;

private:
    enum class HomingState : uint8_t {
        Idle,
        ClearingLimit,
        SeekingLimit,
        BackingOff,
        Homed,
        Failed
    };

    void onStart() override;
    void onStop() override;
    void onExecute() override;

    void startSeek();
    void fail();
    void notify(Event event);

    static void onLimitSwitchStateChanged(
        void *context,
        PinMonitorChannel::PinState state);

    RouterChannel *m_yAxisRouter;
    PinMonitorChannel *m_yAxisLimitSwitch;
    EventCallback m_eventCallback;
    void *m_eventCallbackContext;
    std::atomic<bool> m_limitActive;
    HomingState m_homingState;
};

#endif /* HOMING_MODULE_HPP */
