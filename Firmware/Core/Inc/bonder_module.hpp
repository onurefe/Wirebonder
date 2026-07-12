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
        // PLL reached its duration limit before delivering the requested energy.
        InsufficientBondingPower,
        // Force-coil current control could not reach its requested setpoint.
        UnableToSetForceCoilCurrent,
        // Z-axis position control is unavailable while a move is required.
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
                 DirectSolenoidChannel *clampSolenoid,
                 PinMonitorChannel *contactSensorMonitor,
                 PinMonitorChannel *mouseRightButtonMonitor,
                 Timer *timer);

    void configure(const Config& config);
    const Config& getConfig() const;
    // True when the VM is at its first sequence step.
    bool isIdle() const { return m_stepIndex == 0U; }

    void addEventListenerCallbacks(BonderStateChangedCallback stateCb, BonderErrorCallback errorCb);
    void init();
    void start();
    void stop();
    void execute();

private:
    // =========================================================================
    // Bonding-sequence VM
    // =========================================================================

    enum class StepStatus { Running, Done, Error };
    enum class BondingPhase { Phase1, Phase2 };
    using StepFn = StepStatus (BonderModule::*)(BondingPhase phase);
    struct Step {
        StepFn fn;
        BondingPhase phase;
    };

    static const Step  kBondSequence[];
    static const uint8_t kSequenceLen;

    // Shared first- and second-bond steps.

    /*
     * Entry: no action. Phase 2 first ensures that the clamp is closed.
     * Complete when: the semi-automatic button is pressed.
     */
    StepStatus stepWaitForSemiAutoButton(BondingPhase phase);

    /*
     * Entry: apply tracking current and command the phase search height;
     *        phase 2 also commands Y stepback. Phase 2 holds the entry
     *        actions until the clamp is confirmed open.
     * Complete when: Z and, for phase 2, Y are settled; tracking current is
     *                settled; and the operator has released the button.
     */
    StepStatus stepMovingToSearchHeight(BondingPhase phase);

    /*
     * Entry: apply constant current and command the lowest overtravel height.
     * Complete when: Z is settled, the contact pin is disconnected, and the
     *                force-coil current is settled.
     */
    StepStatus stepMovingToLowestOvertravel(BondingPhase phase);

    /*
     * Entry: start the position-settling timer.
     * Complete when: the timer expires.
     */
    StepStatus stepWaitForZMotorPositionSettlement(BondingPhase phase);

    /*
     * Entry: apply the phase bonding current and start the force-settling
     *        timer.
     * Complete when: the timer expires and the force-coil current is settled.
     */
    StepStatus stepWaitForBondingForceSettlement(BondingPhase phase);

    /*
     * Entry: start the impedance scan.
     * Complete when: the scan completes and its operating point is calculated
     *                for the phase bonding power.
     */
    StepStatus stepScanImpedance(BondingPhase phase);

    /*
     * Entry: start PLL-controlled ultrasonic bonding with the phase energy.
     * Complete when: the PLL reports that the requested energy was delivered.
     */
    StepStatus stepBond(BondingPhase phase);

    /*
     * Entry: apply constant current and start the cooling timer.
     * Complete when: the timer expires and the force-coil current is settled.
     */
    StepStatus stepCool(BondingPhase phase);

    // First-bond loop-formation steps.

    /*
     * Entry: Ensure that the clamp is opened and then command Z to kink height.
     * While waiting: after the contact pin connects, command the T-axis tail
     *                move once.
     * Complete when: Z and T are settled and the contact pin is connected.
     */
    StepStatus stepMoveToKinkHeight(BondingPhase phase);

    /*
     * Entry: command the Y-axis reverse displacement.
     * Complete when: the Y-axis move completes.
     */
    StepStatus stepYReverse(BondingPhase phase);

    /*
     * Entry: command Z to loop height.
     * Complete when: Z is settled.
     */
    StepStatus stepMoveToLoopHeight(BondingPhase phase);

    // Second-bond completion and tail restoration.

    /*
     * Entry: Ensure that the clamp is closed, and command the T-axis tear displacement.
     * Complete when: the T-axis move completes.
     */
    StepStatus stepTearTMove(BondingPhase phase);

    /*
     * Entry: command Z to reset height and start the tail-restore delay.
     * While waiting: after the delay, return T to its initial position and
     *                start ultrasonic power using the last scan result.
     * Complete when: Z and T are settled, ultrasonic power completes, and the
     *                contact pin is connected.
     */
    StepStatus stepMoveToResetHeight(BondingPhase phase);

    /*
     * Entry: return Y to its initial position.
     * Complete when: the Y-axis move completes.
     */
    StepStatus stepRestoreYMove(BondingPhase phase);

    void emergencyStop();

    // =========================================================================
    // Configuration and VM runtime state
    // =========================================================================

    Config  m_config;
    float   m_centerFrequency;
    float   m_driveAmplitude;

    State   m_systemState;
    uint8_t m_stepIndex;    // position in kBondSequence[]
    bool    m_stepStarted;  // one-shot init guard per step

    // =========================================================================
    // Hardware event flags (set by callback bridges, polled by VM steps)
    // =========================================================================

    bool m_tMoveCompleted;
    bool m_yMoveCompleted;
    bool m_forceCoilCurrentSettled;
    bool m_contactPinConnected;
    bool m_contactPinDisconnected;
    bool m_timerExpired;
    bool m_impedanceScanningCompleted;
    bool m_usPowerTransferred;
    bool m_rightButtonPressed;
    bool m_rightButtonReleased;

    // Errors reported by dependencies and consumed by the VM.
    bool m_positionError;
    bool m_forceCoilError;
    bool m_usPowerError;

    // Deferred step actions, issued once after their triggering event.
    bool m_tailMoveStarted;
    bool m_resetRestoreStarted;

    // =========================================================================
    // External notifications
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
    DirectSolenoidChannel    *m_clampSolenoid;
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
    // VM utilities
    // =========================================================================

    void    clearFlags();
    void    setZMotorPosition(float position);
    bool    zMotorPositionReached();
    void    computeOperatingPoint(float targetPower);
    float   amplitudeForTargetPower(float realAdmittance, float targetPower);
    uint8_t findResonanceIndex();
    float   calculateCenterFrequency(uint8_t resonanceIndex);
    float   calculateDriveAmplitude(uint8_t resonanceIndex, float targetPower);

    // =========================================================================
    // Peripheral callback bridges
    // =========================================================================

    static BonderModule *s_instance;

    static void onContactSensorStateChanged(void *context, PinMonitorChannel::PinState state);
    static void onMouseRightButtonStateChanged(void *context, PinMonitorChannel::PinState state);
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
