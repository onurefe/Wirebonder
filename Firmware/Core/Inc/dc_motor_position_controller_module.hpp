#ifndef DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP
#define DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP

#include "callback_list.hpp"
#include "configuration.h"
#include "generic.h"
#include "dc_motor_velocity_controller_module.hpp"
#include "process.hpp"
#include "lvdt_module.hpp"

class DcMotorPositionControllerModule : public Process {
public:
    // A provider supplies where the carriage should be and, optionally, how
    // fast it is being asked to travel there. The velocity is fed forward past
    // the position term, so a provider walking a profile carries its own
    // motion and leaves the loop only the residual error to correct. Leave it
    // at zero for a plain step setpoint.
    using SetpointCallback = bool (*)(void *context,
                                      float *positionSetpoint,
                                      float *velocityFeedforward);
    using VelocityListenerCallback = void (*)(void *context, float velocity);
    using PositionListenerCallback = void (*)(void *context, float position);

    enum class Event : uint8_t {
        // Fired on every control sample whose position error is within
        // DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR -- level-triggered, not
        // one-shot. It reports on whichever provider won the setpoint
        // arbiter, so it says "the loop is tracking its input", not "your
        // move finished". A caller that needs the latter tests its own
        // setpoint against getPosition() (see BonderCommandZMove).
        SetpointReached
    };

    using EventCallback = void (*)(void *context, Event event);

    DcMotorPositionControllerModule(LvdtSensorModule *lvdtSensor,
        DcMotorVelocityControllerModule *velocityController);

    bool enableControl();
    void disableControl();

    void restartControlLoop();
    bool addPositionSetpointControllerCallback(void *context,
        SetpointCallback callback);
    bool removePositionSetpointControllerCallback(void *context,
        SetpointCallback callback);
    bool addEventListenerCallback(void *context, EventCallback callback);
    bool removeEventListenerCallback(void *context, EventCallback callback);

    // Continuous push notifications (proxied from the wrapped velocity
    // controller / LVDT), for consumers that only hold a
    // DcMotorPositionControllerModule* and need every measurement rather
    // than polling getVelocity()/getPosition() at an unrelated tick rate.
    bool addVelocityListenerCallback(void *context, VelocityListenerCallback callback);
    bool removeVelocityListenerCallback(void *context, VelocityListenerCallback callback);
    bool addPositionListenerCallback(void *context, PositionListenerCallback callback);
    bool removePositionListenerCallback(void *context, PositionListenerCallback callback);

    float execute(float positionSetpoint, float velocityFeedforward = 0.0f);

    float getPosition() const;
    float getVelocity() const;
    float getLvdtMagnitudeA() const;
    float getLvdtMagnitudeB() const;
    bool isControlEnabled() const;
    void enableBypass();
    void disableBypass();
    void enableDriveBypass();
    void disableDriveBypass();

    // Velocity-setpoint clamp, in mm/s and signed (Z increases upward, so
    // minVelocity is negative). Seeded from the ZMOTOR_MAX_*_SPEED_DEFAULT
    // macros and re-pushed by Robot::updateZPositionSpeedLimits() on every
    // protocol selection (manual mode runs at its configuration's own speed,
    // everything else at the SETTINGS UP/DOWN SPEED) and on a settings edit.
    void setOutputLimits(float minVelocity, float maxVelocity);
    // Read-back, so a caller that narrows the clamp for a while can put back
    // exactly what it found (see BonderCommandMzDrive).
    float getOutputMin() const { return m_outputMin; }
    float getOutputMax() const { return m_outputMax; }

private:
    enum class ControlState : uint8_t { Disabled, Enabled };

    void onStart() override;
    void onStop() override;

    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static void lvdtCallback(void *context, float position, float magA, float magB);
    static bool controlUpdateCallback(void *context, float *targetVelocity);
    static void velocityMeasuredCallback(void *context, float velocity);

    // -----------------------------------------------------------------------
    // Peripheral-Event-Handlers
    // -----------------------------------------------------------------------
    void onLvdtMeasured(float position, float magA, float magB);
    bool onControlUpdate(float *targetVelocity);
    void onVelocityMeasured(float velocity);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    LvdtSensorModule *m_lvdtSensorModule;
    DcMotorVelocityControllerModule *m_velocityController;

    ControlState m_controlState;
    bool m_bypassEnabled;
    bool m_hasPositionMeasurement;

    ArbiterList<float *, float *> m_setpointControllerCallbacks;
    ListenerList<float>  m_velocityListenerCallbacks;
    ListenerList<float>  m_positionListenerCallbacks;
    ListenerList<Event>  m_eventCallbacks;

    float m_positionMeasurement;
    float m_lvdtMagnitudeA;
    float m_lvdtMagnitudeB;
    float m_profiledPositionSetpoint;
    float m_profiledPositionVelocity;
    float m_previousPositionError;
    float m_filteredPositionErrorRate;
    uint16_t m_positionStableSampleCount;
    bool m_previousPositionErrorValid;
    bool m_positionErrorRateValid;
    bool m_profiledPositionSetpointValid;

    float m_outputMin;
    float m_outputMax;

    float clampOutput(float rawOutput) const;
};

#endif
