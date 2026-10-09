#ifndef DC_MOTOR_VELOCITY_CONTROLLER_MODULE_HPP
#define DC_MOTOR_VELOCITY_CONTROLLER_MODULE_HPP

#include "callback_list.hpp"
#include "configuration.h"
#include "generic.h"
#include "pwm_service.hpp"
#include "process.hpp"
#include "lvdt_module.hpp"
#include "pid_controller.hpp"
#include "zaxis_kalman_filter.hpp"

// The Z axis's inner loop. There is no tachometer: every LVDT measurement
// clocks one control tick, in which a Kalman filter turns the position into a
// velocity estimate, the velocity PID drives the PWM from it, and the voltage
// actually applied is fed back into the filter's motor model.
//
// Because the LVDT is this loop's only sensor, the module owns it while
// control is enabled and relays each measurement to its position listeners
// (the position loop) before asking its controller for a target velocity, so
// that controller always works from this tick's position.
class DcMotorVelocityControllerModule : public Process {
public:
    using VelocityListenerCallback = void (*)(void *context, float estimatedVelocity);
    using PositionListenerCallback = void (*)(void *context, float position, float magA, float magB);
    // Returns true when the controller is active; the target velocity is
    // written through the pointer. The first active controller wins.
    using VelocityControllerCallback = bool (*)(void *context, float *targetVelocity);

    DcMotorVelocityControllerModule(LvdtSensorModule *lvdtSensor,
        PwmRampChannel *pwmChannel);

    bool enableControl();
    void disableControl();

    // Velocity listeners fire at the end of each tick, once the drive for the
    // coming period has been set, so getAppliedVoltage() then matches it.
    bool addVelocityListenerCallback(void *context, VelocityListenerCallback cb);
    bool removeVelocityListenerCallback(void *context, VelocityListenerCallback cb);
    bool addPositionListenerCallback(void *context, PositionListenerCallback cb);
    bool removePositionListenerCallback(void *context, PositionListenerCallback cb);
    bool addVelocityControllerCallback(void *context, VelocityControllerCallback cb);
    bool removeVelocityControllerCallback(void *context, VelocityControllerCallback cb);

    // The raw LVDT position of the current tick, the estimated velocity, the
    // estimated input-referred disturbance (V) and the voltage applied after
    // duty clamping, all in the up-positive convention.
    float getPosition() const;
    float getVelocity() const;
    float getDisturbance() const;
    float getAppliedVoltage() const;
    bool isControlEnabled() const;
    void enablePidBypass();
    void disablePidBypass();

    // The speed the motion is planned at this tick (mm/s, up positive),
    // without any correction on top: a profile's own velocity, zero while
    // holding. Friction compensation fades in on this, so a
    // position correction at a hold cannot switch them on. The controller
    // sets it from inside its callback each tick; it reads zero otherwise.
    void setPlannedVelocity(float velocity);

    // Switch the drive off and keep the PID reset while the controller
    // judges the head to be holding (see ZMOTOR_HOLD_DEADBAND). Set from
    // inside the controller callback each tick, like the planned velocity;
    // it reads false otherwise. The estimator keeps running throughout.
    void setHolding(bool holding);

private:
    enum class ControlState : uint8_t { Disabled, Enabled };

    void onStart() override;
    void onStop() override;

    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static void lvdtCallback(void *context, float position, float magA, float magB);
    static bool pwmUpdateCallback(void *context, float *value);

    // -----------------------------------------------------------------------
    // Peripheral-Event-Handlers
    // -----------------------------------------------------------------------
    void onLvdtMeasured(float position, float magA, float magB);
    bool onPwmUpdate(float *value);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();
    float computeTargetDuty(float velocityControlOutput) const;
    static float clampDuty(float duty);
    static float dutyToVoltage(float duty);

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    LvdtSensorModule *m_lvdtSensorModule;
    PwmRampChannel   *m_pwmChannel;

    PidController m_velocityPid;
    ZAxisKalmanFilter m_estimator;

    ControlState m_controlState;
    // False until the first measurement after enableControl() has seeded the
    // filter at the carriage's actual position.
    bool m_hasEstimate;

    // Listeners (observers of each tick) and the single controller that
    // supplies the target velocity each tick — mirrors the
    // IQDemodulatorChannel measurement-listener / controller pattern.
    ListenerList<float> m_velocityListenerCallbacks;
    ListenerList<float, float, float> m_positionListenerCallbacks;
    ArbiterList<float *> m_velocityControllerCallbacks;

    float m_positionMeasurement;
    float m_velocityEstimate;
    float m_appliedVoltage;
    float m_plannedVelocity;
    bool m_holding;
    bool m_wasHolding;

    float m_targetDuty;
};

#endif
