#ifndef DC_ROUTER_MODULE_HPP
#define DC_ROUTER_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "timer_expire_service.hpp"
#include "dc_motor_position_controller_module.hpp"

class DcRouterModule {
public:
    enum class Event {
        SETPOINT_ACHIEVED = 0,
        UNABLE_TO_REACH_SETPOINT = 1
    };

    using EventOccurredCallback = void (*)(void *context, Event event);

    DcRouterModule(DcMotorPositionControllerModule *positionController,
        Timer *settlingTimer,
        float maxVelocity,
        float maxAcceleration);

    void start(float initialPosition);
    void stop();
    void beginMove(float targetPosition);

    float getPosition() const;
    void addEventListenerCallback(EventOccurredCallback callback, void *context);

private:
    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static bool setpointSourceCallback(void *context, float *positionSetpoint);
    static void settlingTimerCallback(void *context, Timer *timer);

    // -----------------------------------------------------------------------
    // Event-Handlers
    // -----------------------------------------------------------------------
    // Pulled by the position controller once per control tick: advances the
    // planner and writes the instantaneous position setpoint. Returns true
    // while operating (i.e. this router is the active setpoint controller).
    bool onProvideSetpoint(float *positionSetpoint);
    void onSettlingTimerExpireServiced();

    // -----------------------------------------------------------------------
    // Motion-Planning
    // -----------------------------------------------------------------------
    struct TrapezoidProfile {
        float direction;
        float distance;
        float accelTime;
        float cruiseTime;
        float totalTime;
        float peakVelocity;
    };

    void planTrapezoidProfile();
    float plannedPosition(float time, bool *segmentCompleted) const;

    // -----------------------------------------------------------------------
    // Move-Lifecycle
    // -----------------------------------------------------------------------
    void completeMove();
    void abortMove();

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();
    void publishEvent(Event event);
    bool isReady() const;
    bool isOperating() const;
    bool setpointIsReached() const;

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    DcMotorPositionControllerModule *m_positionController;
    Timer                           *m_settlingTimer;

    EventOccurredCallback m_callback;
    void *m_callbackContext;

    ServiceState m_state;

    TrapezoidProfile m_profile;

    float m_maxVelocity;
    float m_maxAcceleration;

    float m_startingPosition;
    float m_destinationPosition;

    bool     m_moveInProgress;
    uint32_t m_segmentCounter;
};

#endif
