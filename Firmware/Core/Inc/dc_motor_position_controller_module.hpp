#ifndef DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP
#define DC_MOTOR_POSITION_CONTROLLER_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "dc_motor_velocity_controller_module.hpp"
#include "lvdt_module.hpp"

class DcMotorPositionControllerModule {
public:
    using SetpointCallback = bool (*)(void *context, float *positionSetpoint);

    DcMotorPositionControllerModule(LvdtSensorModule *lvdtSensor,
        DcMotorVelocityControllerModule *velocityController);

    void start();
    void stop();

    void restartControlLoop();
    bool addPositionSetpointControllerCallback(void *context,
        SetpointCallback callback);

    float execute(float positionSetpoint);

    float getPosition() const;
    float getVelocity() const;
    float getLvdtMagnitudeA() const;
    float getLvdtMagnitudeB() const;
    bool isOperating() const;
    void enableBypass();
    void disableBypass();
    void enableDriveBypass();
    void disableDriveBypass();

private:
    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static void lvdtCallback(void *context, float position, float magA, float magB);
    static bool controlUpdateCallback(void *context, float *targetVelocity);

    // -----------------------------------------------------------------------
    // Peripheral-Event-Handlers
    // -----------------------------------------------------------------------
    void onLvdtMeasured(float position, float magA, float magB);
    bool onControlUpdate(float *targetVelocity);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();
    bool isReady() const;

    struct SetpointControllerRegistration {
        SetpointCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxSetpointControllerCallbacks = 4U;

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    LvdtSensorModule *m_lvdtSensorModule;
    DcMotorVelocityControllerModule *m_velocityController;

    ServiceState m_state;
    bool m_bypassEnabled;
    bool m_hasPositionMeasurement;

    SetpointControllerRegistration m_setpointControllerCallbacks[kMaxSetpointControllerCallbacks];
    uint8_t m_setpointControllerCallbackCount;

    float m_positionMeasurement;
    float m_lvdtMagnitudeA;
    float m_lvdtMagnitudeB;

    float clampOutput(float rawOutput) const;
};

#endif
