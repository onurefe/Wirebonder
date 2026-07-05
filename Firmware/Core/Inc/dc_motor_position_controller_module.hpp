#ifndef DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP
#define DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "lvdt_module.hpp"
#include "pid_controller.hpp"
#include "dc_motor_velocity_controller_module.hpp"

class DcMotorPositionControllerModule {
public:
    // Returns true when the controller is active; the position setpoint is
    // written through the pointer. The first active controller wins.
    using SetpointCallback = bool (*)(void *context, float *positionSetpoint);

    DcMotorPositionControllerModule(LvdtSensorModule *lvdtSensor,
        DcMotorVelocityControllerModule *velocityController);

    void start();
    void stop();

    void restartControlLoop();

    bool addPositionSetpointControllerCallback(void *context, SetpointCallback callback);

    float getPosition() const;
    float getVelocity() const;
    bool isOperating() const;

private:
    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static void lvdtCallback(void *context, float position);
    static bool controlUpdateCallback(void *context, float *targetVelocity);

    // -----------------------------------------------------------------------
    // Peripheral-Event-Handlers
    // -----------------------------------------------------------------------
    void onLvdtMeasured(float position);
    // Pulled by the velocity controller once per inner-loop tick; pulls the
    // position setpoint from the setpoint controllers and runs the position PID
    // against the latest LVDT sample, writing the velocity setpoint. Returns
    // true while operating (i.e. this module is an active velocity controller).
    bool onControlUpdate(float *targetVelocity);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();
    bool isReady() const;

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    LvdtSensorModule                *m_lvdtSensorModule;
    DcMotorVelocityControllerModule *m_velocityController;

    PidController m_positionPid;

    ServiceState m_state;

    struct SetpointControllerRegistration {
        SetpointCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxSetpointControllerCallbacks = 4U;

    SetpointControllerRegistration m_setpointControllerCallbacks[kMaxSetpointControllerCallbacks];
    uint8_t m_setpointControllerCallbackCount;

    float m_positionMeasurement;
};

#endif
