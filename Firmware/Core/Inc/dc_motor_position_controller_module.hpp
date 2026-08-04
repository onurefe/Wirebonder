#ifndef DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP
#define DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "dc_motor_velocity_controller_module.hpp"
#include "process.hpp"
#include "lvdt_module.hpp"

class DcMotorPositionControllerModule : public Process {
public:
    using SetpointCallback = bool (*)(void *context, float *positionSetpoint);
    using VelocityListenerCallback = void (*)(void *context, float velocity);
    using PositionListenerCallback = void (*)(void *context, float position);

    enum class Event : uint8_t {
        // Fired after the position error remains within tolerance and its
        // EMA-filtered rate remains below the configured mm/s limit for
        // consecutive control samples. Level-triggered afterward so a
        // listener latching a flag never misses it.
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
    void addEventListenerCallback(void *context, EventCallback callback);

    // Continuous push notifications (proxied from the wrapped velocity
    // controller / LVDT), for consumers that only hold a
    // DcMotorPositionControllerModule* and need every measurement rather
    // than polling getVelocity()/getPosition() at an unrelated tick rate.
    bool addVelocityListenerCallback(void *context, VelocityListenerCallback callback);
    bool addPositionListenerCallback(void *context, PositionListenerCallback callback);

    float execute(float positionSetpoint);

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
    // macros and re-pushed by Robot::updateZPositionSpeedLimits() whenever
    // the operator edits UP SPEED / DOWN SPEED.
    void setOutputLimits(float minVelocity, float maxVelocity);

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
    void resetPositionSettling();
    void resetPositionProfile();
    float advancePositionProfile(float targetPosition);

    struct SetpointControllerRegistration {
        SetpointCallback callback;
        void *context;
    };

    struct VelocityListenerRegistration {
        VelocityListenerCallback callback;
        void *context;
    };

    struct PositionListenerRegistration {
        PositionListenerCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxSetpointControllerCallbacks = 4U;
    static constexpr uint8_t kMaxVelocityListenerCallbacks = 4U;
    static constexpr uint8_t kMaxPositionListenerCallbacks = 4U;

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    LvdtSensorModule *m_lvdtSensorModule;
    DcMotorVelocityControllerModule *m_velocityController;

    ControlState m_controlState;
    bool m_bypassEnabled;
    bool m_hasPositionMeasurement;

    SetpointControllerRegistration m_setpointControllerCallbacks[kMaxSetpointControllerCallbacks];
    uint8_t m_setpointControllerCallbackCount;

    VelocityListenerRegistration m_velocityListenerCallbacks[kMaxVelocityListenerCallbacks];
    uint8_t m_velocityListenerCallbackCount;

    PositionListenerRegistration m_positionListenerCallbacks[kMaxPositionListenerCallbacks];
    uint8_t m_positionListenerCallbackCount;

    EventCallback m_eventCallback;
    void         *m_eventCallbackContext;

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
    // SetpointReached is one-shot: latched once fired and re-armed only by
    // resetPositionSettling() (new setpoint, restart, enable, bypass change).
    bool m_setpointReachedNotified;

    float m_outputMin;
    float m_outputMax;

    float clampOutput(float rawOutput) const;
};

#endif
