#ifndef FORCE_COIL_MODULE_HPP
#define FORCE_COIL_MODULE_HPP

#include "callback_list.hpp"
#include "configuration.h"
#include "generic.h"
#include "pid_controller.hpp"
#include "adc_service.hpp"
#include "pwm_service.hpp"
#include "process.hpp"

// -----------------------------------------------------------------------
class ForceCoilDriverModule : public Process {
    public:
        using CurrentListenerCallback = void (*)(void *context, float measuredCurrent);

        ForceCoilDriverModule(AnalogChannel *iSensChannel,
                        PwmRampChannel *iDriveChannel);
        
        typedef enum {
            SetpointAchieved = 0,
            UnableToSetCurrent = 1
        } Event;

        using ForceCoilCallback = void (*)(void *context, Event event);
        bool enableControl();
        void disableControl();
        bool isControlEnabled() const;
        void setCurrentSetpoint(float currentSetpoint);
        bool addEventListenerCallback(void *context, ForceCoilCallback callback);
        bool removeEventListenerCallback(void *context, ForceCoilCallback callback);
        bool addCurrentListenerCallback(void *context, CurrentListenerCallback callback);
        bool removeCurrentListenerCallback(void *context, CurrentListenerCallback callback);
        void enablePidBypass();
        void disablePidBypass();

    private:
        enum class ControlState : uint8_t { Disabled, Enabled };

        void onStart() override;
        void onStop() override;

        void onCurrentMeasured(float measuredCurrent);
        bool onPwmUpdate(float *value);
        void advanceSetpointRamp();

        static void iSensCallback(void *context, float value);
        static bool iDriveCallback(void *context, float *value);

        AnalogChannel *m_iSensChannel;
        PwmRampChannel *m_iDriveChannel;
        PidController m_pidCtrl;

        ControlState m_controlState;
        ListenerList<Event> m_eventCallbacks;
        ListenerList<float> m_currentListenerCallbacks;

        float m_currentSetpoint;   // ramped value driving the PID this tick
        float m_targetSetpoint;    // ultimate requested value (see setCurrentSetpoint)
        float m_targetDuty;
        bool m_newSetpoint;
};

#endif /* FORCE_COIL_MODULE_HPP */
