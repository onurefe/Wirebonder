#ifndef DC_MOTOR_VELOCITY_CONTROLLER_MODULE_HPP
#define DC_MOTOR_VELOCITY_CONTROLLER_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "pwm_service.hpp"
#include "process.hpp"
#include "adc_service.hpp"
#include "pid_controller.hpp"

class DcMotorVelocityControllerModule : public Process {
public:
    using VelocityListenerCallback = void (*)(void *context, float measuredVelocity);
    // Returns true when the controller is active; the target velocity is
    // written through the pointer. The first active controller wins.
    using VelocityControllerCallback = bool (*)(void *context, float *targetVelocity);

    DcMotorVelocityControllerModule(AnalogChannel *tachometerChannel,
        PwmRampChannel *pwmChannel);

    bool enableControl();
    void disableControl();

    bool addVelocityListenerCallback(void *context, VelocityListenerCallback cb);
    bool addVelocityControllerCallback(void *context, VelocityControllerCallback cb);

    float getVelocity() const;
    bool isControlEnabled() const;
    void enablePidBypass();
    void disablePidBypass();

private:
    enum class ControlState : uint8_t { Disabled, Enabled };

    void onStart() override;
    void onStop() override;

    struct VelocityListenerRegistration {
        VelocityListenerCallback callback;
        void *context;
    };

    struct VelocityControllerRegistration {
        VelocityControllerCallback callback;
        void *context;
    };

    static constexpr uint8_t kMaxVelocityListenerCallbacks = 4U;
    static constexpr uint8_t kMaxVelocityControllerCallbacks = 4U;

    // -----------------------------------------------------------------------
    // Peripheral-Bridge-Callbacks
    // -----------------------------------------------------------------------
    static void tachometerCallback(void *context, float velocity);
    static bool pwmUpdateCallback(void *context, float *value);

    // -----------------------------------------------------------------------
    // Peripheral-Event-Handlers
    // -----------------------------------------------------------------------
    void onTachometerMeasured(float velocity);
    bool onPwmUpdate(float *value);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void registerPeripheralCallbacks();
    float computeTargetDuty(float velocityControlOutput) const;
    static float clampDuty(float duty);

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    AnalogChannel  *m_tachometerChannel;
    PwmRampChannel *m_pwmChannel;

    PidController m_velocityPid;

    ControlState m_controlState;

    // Listeners (observers of the measured velocity) and the single
    // controller that supplies the target velocity each tick — mirrors the
    // IQDemodulatorChannel measurement-listener / controller pattern.
    VelocityListenerRegistration m_velocityListenerCallbacks[kMaxVelocityListenerCallbacks];
    uint8_t m_velocityListenerCallbackCount;

    VelocityControllerRegistration m_velocityControllerCallbacks[kMaxVelocityControllerCallbacks];
    uint8_t m_velocityControllerCallbackCount;

    float m_velocityMeasurement;
    float m_targetDuty;
};

#endif
