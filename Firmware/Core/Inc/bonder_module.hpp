#pragma once

#include <cstdint>
#include "generic.h"
#include "bonder_config.hpp"
#include "configuration.h"
#include "complex.h"
#include "timer_expire_service.hpp"
#include "solenoid_service.hpp"
#include "fast_io.hpp"
#include "pin_monitor_service.hpp"
#include "stepper_router_service.hpp"
#include "dc_motor_position_controller_module.hpp"
#include "force_coil_module.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"

class BonderModule {
public:
    enum class State { Uninit, Ready, Operating };

    enum class Error {
        InsufficientBondingPower,
        UnableToSetForceCoilCurrent,
        UnableToSetPosition
    };

    using Config = BonderConfig;

    using BonderStateChangedCallback = void (*)(bool isIdle);
    using BonderErrorCallback        = void (*)(Error error);

    BonderModule(DcMotorPositionControllerModule *zMotorController,
                 ForceCoilDriverModule *forceCoilDriver,
                 RouterChannel *yAxisRouter,
                 RouterChannel *tAxisRouter,
                 PllModule *pll,
                 UsImpedanceScannerModule *impedanceScanner,
                 SolenoidChannel *clampSolenoid,
                 PinMonitorChannel *contactSensorMonitor,
                 PinMonitorChannel *mouseRightButtonMonitor,
                 Timer *timer);

    void configure(const Config& config);
    const Config& getConfig() const;
    bool isIdle() const { return m_stepIndex == 0U; }

    void addEventListenerCallbacks(BonderStateChangedCallback stateCb, BonderErrorCallback errorCb);
    void init();
    void start();
    void stop();
    void execute();

private:
    // =========================================================================
    // Step-sequence VM
    // =========================================================================

    enum class StepStatus { Running, Done, Error };
    using StepFn = StepStatus (BonderModule::*)();

    static const StepFn  kBondSequence[];
    static const uint8_t kSequenceLen;

    // Steps — first bond
    StepStatus stepWaitForTrigger();
    StepStatus stepMoveToSearchHeight();
    StepStatus stepSearch();
    StepStatus stepSettle();
    StepStatus stepScanImpedance();
    StepStatus stepWeld();
    StepStatus stepFirstBondCool();

    // Steps — loop formation
    StepStatus stepFormLoopTAndZ();
    StepStatus stepFormLoopYReverse();
    StepStatus stepFormLoopZ();

    // Steps — second bond preparation
    StepStatus stepWaitSecondTrigger();
    StepStatus stepPrepCloseClamp();
    StepStatus stepPrepStepBack();
    StepStatus stepPrepOpenClamp();

    // Steps — second bond cooling (search/settle/scan/weld reuse steps above)
    StepStatus stepSecondBondCool();

    // Steps — tail restore
    StepStatus stepTearTMove();
    StepStatus stepRestoreZWaitContact();
    StepStatus stepRestoreScanImpedance();
    StepStatus stepRestoreTMoveWithUs();
    StepStatus stepRestoreYMove();

    void emergencyStop();

    // =========================================================================
    // Configuration & runtime state
    // =========================================================================

    Config  m_config;
    float   m_centerFrequency;
    float   m_driveAmplitude;

    State   m_systemState;
    uint8_t m_stepIndex;    // position in kBondSequence[]
    bool    m_stepStarted;  // one-shot init guard per step

    // =========================================================================
    // Hardware event flags  (set by ISR callbacks, polled by step functions)
    // =========================================================================

    bool m_tMoveCompleted;
    bool m_yMoveCompleted;
    bool m_forceCoilCurrentSettled;
    bool m_contactPinConnected;
    bool m_contactPinDisconnected;
    bool m_timerExpired;
    bool m_impedanceScanningCompleted;
    bool m_usPowerTransferred;
    bool m_clampOpened;
    bool m_clampClosed;
    bool m_rightButtonPressed;
    bool m_rightButtonReleased;

    // Error flags (checked inside steps → StepStatus::Error)
    bool m_positionError;
    bool m_forceCoilError;
    bool m_usPowerError;

    // =========================================================================
    // Callbacks
    // =========================================================================

    BonderStateChangedCallback m_stateChangedCallback;
    BonderErrorCallback        m_errorCallback;

    // =========================================================================
    // Injected dependencies
    // =========================================================================

    DcMotorPositionControllerModule *m_zMotorControllerModule;
    ForceCoilDriverModule   *m_forceCoilControllerModule;
    RouterChannel            *m_yAxisRouter;
    RouterChannel            *m_tAxisRouter;
    PllModule                *m_pllModule;
    UsImpedanceScannerModule *m_impedanceScannerModule;
    SolenoidChannel          *m_clampSolenoid;
    PinMonitorChannel        *m_contactSensorMonitor;
    PinMonitorChannel        *m_mouseRightButtonMonitor;
    Timer                    *m_timer;

    // =========================================================================
    // Impedance scan buffers
    // =========================================================================

    complexf m_voltagePhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_currentPhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_impedances[BONDER_MODULE_SCAN_MAX_FREQUENCIES];

    // =========================================================================
    // Helpers
    // =========================================================================

    void    clearFlags();
    void    setZMotorPosition(float position);
    bool    zMotorPositionReached() const;
    void    computeOperatingPoint();
    float   amplitudeForTargetPower(float realAdmittance) const;
    uint8_t findResonanceIndex() const;
    float   calculateCenterFrequency(uint8_t resonanceIndex) const;
    float   calculateDriveAmplitude(uint8_t resonanceIndex) const;

    // =========================================================================
    // Static ISR callbacks
    // =========================================================================

    static BonderModule *s_instance;

    static void onSolenoidChanged(void *context, SolenoidChannel::State state);
    static void onContactSensorTransition(void *context, PinMonitorChannel::Transition transition);
    static void onMouseRightButtonTransition(void *context, PinMonitorChannel::Transition transition);
    static void onTimerDone(void *context, Timer *t);
    static void onYAxisRouterDone(void *context, RouterChannel *channel);
    static void onTAxisRouterDone(void *context, RouterChannel *channel);
    static void onPllEvent(void *context, PllModule::Event event);
    static void onForceCoilEvent(ForceCoilDriverModule::Event eventId);
    static bool onZMotorPositionSetpoint(void *context, float *positionSetpoint);
    static void onImpedanceScanned(void *context, complexf *v, complexf *c, complexf *i);

    float m_zMotorPositionSetpoint;
    bool  m_zMotorSetpointActive;
};
