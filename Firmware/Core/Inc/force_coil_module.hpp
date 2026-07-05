#ifndef FORCE_COIL_MODULE_HPP
#define FORCE_COIL_MODULE_HPP

#include "configuration.h"
#include "generic.h"
#include "pid_controller.hpp"
#include "adc_service.hpp"
#include "pwm_service.hpp"

// -----------------------------------------------------------------------
class ForceCoilDriverModule {
    public:
        using CurrentListenerCallback = void (*)(void *context, float measuredCurrent);

        ForceCoilDriverModule(AnalogChannel *iSensChannel,
                        PwmRampChannel *iDriveChannel);
        
        typedef enum {
            SetpointAchieved = 0,
            UnableToSetCurrent = 1
        } Event;

        using ForceCoilCallback = void (*)(Event event);
        void start(void);
        void stop(void);
        void setCurrentSetpoint(float currentSetpoint);
        void addEventListenerCallback(ForceCoilCallback callback);
        bool addCurrentListenerCallback(void *context, CurrentListenerCallback callback);
        void enablePidBypass();
        void disablePidBypass();

    private:
        struct CurrentListenerRegistration {
            CurrentListenerCallback callback;
            void *context;
        };

        static constexpr uint8_t kMaxCurrentListenerCallbacks = 4U;

        void onCurrentMeasured(float measuredCurrent);
        bool onPwmUpdate(float *value);

        static void iSensCallback(void *context, float value);
        static bool iDriveCallback(void *context, float *value);

        AnalogChannel *m_iSensChannel;
        PwmRampChannel *m_iDriveChannel;
        PidController m_pidCtrl;

        State_t m_state;
        ForceCoilCallback m_callback;
        CurrentListenerRegistration m_currentListenerCallbacks[kMaxCurrentListenerCallbacks];
        uint8_t m_currentListenerCallbackCount;

        float m_currentSetpoint;
        float m_targetDuty;
        bool m_newSetpoint;
};

#endif /* FORCE_COIL_MODULE_HPP */
