#pragma once

#include <atomic>
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
#include "process.hpp"

class BonderProtocol;

class BonderModule : public Process {
public:
    enum class Error {
        // PLL reached its duration limit before delivering the requested energy.
        InsufficientBondingPower,
        // Force-coil current control could not reach its requested setpoint.
        UnableToSetForceCoilCurrent,
        // Z-axis position control is unavailable while a move is required.
        UnableToSetPosition,
        // A protocol step did not produce its completion events in time.
        ProtocolTimeout
    };

    using Config = BonderConfig;

    using BonderStateChangedCallback = void (*)(void *context, bool isIdle);
    using BonderErrorCallback        = void (*)(void *context, Error error);

    struct UltrasonicReport {
        float resonanceFrequency;
        float qualityFactor;
        float transferredPower;
        float bondingDuration;
    };
    using UltrasonicReportCallback =
        void (*)(void *context, const UltrasonicReport& report);

    // Reports the tachometer's zero-offset residual measured over
    // TACHSAMPLE's window (Start Tach. Cal.): the tachometer's own average
    // velocity minus the ground-truth average velocity derived from the
    // independent LVDT position delta over the same window (so real motion
    // during an imperfect hold isn't misattributed to sensor offset).
    // Consumed by Robot to update/persist the velocity controller's
    // zero-offset correction.
    using TachCalReportCallback = void (*)(void *context, float offsetResidual);

    // Hardware event flags as a bitmask. Events latch when their callback
    // fires and stay latched until consumed by a WAIT/MZDRIVE mask, cleared by
    // CLRFLAGS, or re-armed by the command that produces them.
    enum EventFlag : uint32_t {
        EVENT_T_MOVE_COMPLETED      = 1u << 0,
        EVENT_Y_MOVE_COMPLETED      = 1u << 1,
        EVENT_FORCE_COIL_SETTLED    = 1u << 2,
        EVENT_CONTACT_CONNECTED     = 1u << 3,
        EVENT_CONTACT_DISCONNECTED  = 1u << 4,
        EVENT_TIMER_EXPIRED         = 1u << 5,
        EVENT_SCAN_COMPLETED        = 1u << 6,
        EVENT_US_POWER_TRANSFERRED  = 1u << 7,
        EVENT_RIGHT_BUTTON_PRESSED  = 1u << 8,
        EVENT_RIGHT_BUTTON_RELEASED = 1u << 9,
        EVENT_POSITION_ERROR        = 1u << 10,
        EVENT_FORCE_COIL_ERROR      = 1u << 11,
        EVENT_US_POWER_ERROR        = 1u << 12,
        EVENT_Z_POSITION_REACHED    = 1u << 13,
        EVENT_CLAMP_SETTLED         = 1u << 14,
        EVENT_WAIT_TIMEOUT          = 1u << 15,
        EVENT_LEFT_BUTTON_PRESSED   = 1u << 16,
        EVENT_LEFT_BUTTON_RELEASED  = 1u << 17,
        // Set by notifyClampToggle() when the operator asks a running
        // ClampOpenProtocol to reverse direction (open->close or vice versa).
        EVENT_CLAMP_TOGGLE_REQUESTED = 1u << 18
    };

    // =========================================================================
    // Protocol VM
    //
    // A protocol is a flat array of instructions. Commands issue an action and
    // complete immediately; WAIT and the manual Z instructions block. WAIT
    // and MZDRIVE consume their masked flags on completion. Commands that
    // produce a completion event re-arm (clear) that event when issued, so a
    // stale latch can never satisfy the matching WAIT.
    //
    // Motion protocols do not start at pc 0 directly: beginBonding() first
    // runs a built-in reset prologue (force coil to zero, clamp closed, Z to
    // resetHeight, wait until reached) and clears every event flag. pc 0 is
    // therefore always entered from the idle posture — protocols never need
    // to re-establish or wait for it themselves.
    // =========================================================================

    enum class Opcode : uint8_t {
        ZMOVE = 0,   // arg: height (mm); re-arms Z_POSITION_REACHED
        YMOVE,       // arg: position (mm); re-arms Y_MOVE_COMPLETED
        YREVERSE,    // arg: displacement (mm); moves to
                     // currentYPosition - displacement; re-arms Y_MOVE_COMPLETED
        TMOVE,       // arg: position (mm); re-arms T_MOVE_COMPLETED
        TIMER,       // arg: duration (s); re-arms TIMER_EXPIRED
        WAIT,        // mask: flags that must all be set; timeoutMs == 0 waits
                     // indefinitely; consumed on completion
        CLRFLAGS,    // mask: flags to clear unconditionally
        CLAMPOPEN,   // re-arms CLAMP_SETTLED (set immediately if already open)
        CLAMPCLOSE,  // re-arms CLAMP_SETTLED (set immediately if already closed)
        SCAN,        // arg: target power (W); re-arms SCAN_COMPLETED; the
                     // operating point is computed when the scan finishes
        PLL,         // arg: energy (J); drives the last computed operating
                     // point; re-arms US_POWER_TRANSFERRED
        SETFORCE,    // arg: force (g); re-arms FORCE_COIL_SETTLED
        USREPORT,    // emits the latest scan/PLL result
        MZDRIVE,     // arg: raise target (mm); left button drives toward
                     // lowest overtravel, right button drives toward arg, at
                     // the axis's normal move speed; releasing both stops in
                     // place, re-pressing resumes; blocks until Z is at
                     // lowest overtravel and mask flags are set; consumes mask
        MZSETUP,     // arg: raise target (mm); left button drives toward
                     // lowest overtravel, right button drives toward arg, at
                     // the axis's normal move speed, same as MZDRIVE, but
                     // completes as soon as the left button is released
                     // (regardless of position); does not consume mask
        TACHMOVE,    // moves to ZMOTOR_TACH_CAL_POSITION_MM; re-arms Z_POSITION_REACHED
        TACHSAMPLE,  // arms a ZMOTOR_TACH_CAL_SAMPLE_DURATION_S timer (re-arms
                     // TIMER_EXPIRED, same as TIMER), starts accumulating
                     // tachometer velocity pushed from the velocity
                     // controller, and snapshots the current Z position as
                     // the window's start
        TACHREPORT   // emits the TACHSAMPLE offset residual (tachometer
                     // average minus the LVDT-derived true average velocity)
                     // via TachCalReportCallback
    };

    // Operands are bound to configuration fields so protocols always read the
    // live runtime configuration; nullptr reads as 0.0f (axis origin).
    using ConfigField = float BonderConfig::*;

    struct Instruction {
        Opcode      opcode;
        ConfigField arg;
        uint32_t    mask;
        uint32_t    timeoutMs;
    };

    // Emitted once per completed (or failed) instruction while a telemetry
    // listener is registered. The scan fields are populated only on records
    // whose mask consumed EVENT_SCAN_COMPLETED; impedances is nullptr
    // otherwise.
    struct Telemetry {
        uint8_t  pc;
        uint8_t  opcode;
        bool     succeeded;
        uint32_t mask;
        uint32_t eventFlags;
        float    argValue;
        float    zPosition;
        float    zSetpoint;
        float    yPosition;
        float    tPosition;
        uint8_t  clampState;
        float    transferredEnergy;
        uint16_t scanCount;
        float    centerFrequency;
        float    driveAmplitude;
        const complexf *impedances;
    };

    using TelemetryCallback = void (*)(void *context, const Telemetry &telemetry);

    // Read-only snapshot of where the VM currently is, for debug observation.
    // opcode/mask describe the instruction at pc and are valid while running.
    struct VmStatus {
        bool     running;
        uint8_t  pc;
        uint8_t  opcode;
        uint32_t mask;
        uint32_t eventFlags;
    };

    BonderModule(DcMotorPositionControllerModule *zMotorController,
                 ForceCoilDriverModule *forceCoilDriver,
                 RouterChannel *yAxisRouter,
                 RouterChannel *tAxisRouter,
                 PllModule *pll,
                 UsImpedanceScannerModule *impedanceScanner,
                 SolenoidChannel *clampSolenoid,
                 PinMonitorChannel *contactSensorMonitor,
                 Timer *timer);

    void configure(const Config& config);
    const Config& getConfig() const;

    // Selects the protocol program the VM runs from Idle. The module is
    // protocol-agnostic; the caller (Robot) maps the configured bonding mode
    // to a program. Must be idle.
    bool setProtocol(const BonderProtocol& protocol);
    bool isActive() const { return m_activityState == ActivityState::Running; }
    bool isIdle() const { return !isActive(); }

    void addEventListenerCallbacks(
        void *context, BonderStateChangedCallback stateCb, BonderErrorCallback errorCb);
    void setUltrasonicReportListenerCallback(
        void *context, UltrasonicReportCallback callback);
    void setTachCalReportListenerCallback(
        void *context, TachCalReportCallback callback);
    void setTelemetryListenerCallback(void *context, TelemetryCallback callback);

    VmStatus getVmStatus() const;

    void notifyRightButton(bool pressed);
    void notifyLeftButton(bool pressed);

    void notifyClampToggle();

    bool engage();
    void disengage();

private:
    enum class ActivityState : uint8_t { Idle, Running };
    enum class InstrStatus { Running, Done };

    void onStart() override;
    void onStop() override;
    void onExecute() override;

    void executeVM();
    InstrStatus executeInstruction(const Instruction& instr);
    InstrStatus executeMzDrive(const Instruction& instr);
    InstrStatus executeMzSetup(const Instruction& instr);
    InstrStatus executeClampCommand(SolenoidChannel::State target);

    void emergencyStop();
    // Enables/disables the force-coil + Z-motor control loops to match
    // `enabled`, tracking m_motionControlEnabled; no-op if already there.
    // Only the enable direction can fail (hardware refuses to arm).
    bool setMotionControlEnabled(bool enabled);
    // Switches control loops to what the (already pointed-to) program needs
    // and resets VM position to its pc 0 (reset prologue first, if the
    // program needs motion control).
    bool activateProgram(bool requiresMotionControl);
    // Aborts the VM to Idle on error.
    void failVm(Error error);

    // =========================================================================
    // Configuration and VM runtime state
    // =========================================================================

    Config  m_config;
    float   m_centerFrequency;
    float   m_driveAmplitude;
    float   m_scanTargetPower;
    float   m_qualityFactor;

    // Reset prologue executed before every motion protocol; guarantees pc 0
    // is entered at reset height with force off and the clamp closed.
    static const Instruction kResetPrologue[5];
    static constexpr uint8_t kResetPrologueLength = 5U;

    ActivityState m_activityState;
    const Instruction *m_program;   // active protocol, set via setProtocol()
    uint8_t m_programLength;
    bool    m_protocolRequiresMotionControl;
    // Control loops actually enabled at engage(); tracked separately from the
    // protocol's requirement so a swap to a motion-free protocol while
    // engaged still disables the loops when the VM idles.
    bool    m_motionControlEnabled;
    uint8_t m_pc;                   // position in m_program[] (or prologue)
    bool    m_instrStarted;         // one-shot entry guard for blocking ops
    uint32_t m_waitStartTick;

    // =========================================================================
    // Hardware event flags (set by callback bridges, consumed by the VM)
    //
    // Producers run in mixed contexts (main loop, control-update chain, timer
    // and pin-monitor ISRs) while the VM consumes from the main loop, so the
    // latch is an atomic bitmask: fetch_or/fetch_and compile to LDREX/STREX
    // on Cortex-M4 and no set can be lost to a concurrent clear.
    // =========================================================================

    std::atomic<uint32_t> m_eventFlags;

    static constexpr uint32_t kErrorFlagsMask =
        EVENT_POSITION_ERROR | EVENT_FORCE_COIL_ERROR | EVENT_US_POWER_ERROR |
        EVENT_WAIT_TIMEOUT;

    // Clamp state a CLAMPOPEN/CLAMPCLOSE is driving towards; the solenoid
    // state callback raises CLAMP_SETTLED when this state is reached.
    SolenoidChannel::State m_clampCommandTarget;

    // Manual Z-drive button state. Held states are written by the
    // user-interface relay (possibly from monitor sampling context) and read by
    // the VM in the main loop.
    std::atomic<bool> m_raiseButtonHeld;
    std::atomic<bool> m_lowerButtonHeld;

    // Button-driven state shared by MZDRIVE/MZSETUP: which direction's target
    // is currently commanded, so a held-state change is only issued once
    // (not every tick).
    enum class ManualDriveDir : uint8_t { None, Lower, Raise };
    ManualDriveDir m_manualDriveDir;

    // =========================================================================
    // External notifications
    // =========================================================================

    BonderStateChangedCallback m_stateChangedCallback;
    BonderErrorCallback        m_errorCallback;
    void                      *m_eventCallbackContext;
    UltrasonicReportCallback   m_ultrasonicReportCallback;
    void                      *m_ultrasonicReportCallbackContext;
    TachCalReportCallback      m_tachCalReportCallback;
    void                      *m_tachCalReportCallbackContext;
    TelemetryCallback          m_telemetryCallback;
    void                      *m_telemetryCallbackContext;

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
    Timer                    *m_timer;

    // =========================================================================
    // Impedance scan buffers
    // =========================================================================

    complexf m_voltagePhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_currentPhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_impedances[BONDER_MODULE_SCAN_MAX_FREQUENCIES];

    // =========================================================================
    // Tach-cal sampling (TACHSAMPLE/TACHREPORT) — accumulated from velocity
    // measurements pushed by the velocity controller (via
    // DcMotorPositionControllerModule's proxy), not polled, so every real
    // tachometer sample is counted exactly once regardless of this module's
    // own tick rate.
    // =========================================================================

    bool     m_tachSamplingActive;
    float    m_tachVelocitySum;
    uint32_t m_tachSampleCount;
    // Z position (independent LVDT measurement, not the tachometer) at the
    // instant TACHSAMPLE armed — compared against the position at TACHREPORT
    // to separate real motion (imperfect position hold during the window)
    // from genuine tachometer zero-offset error.
    float    m_tachPositionAtSampleStart;

    // =========================================================================
    // VM utilities
    // =========================================================================

    float    resolveArg(const Instruction& instr) const;
    uint32_t collectEventFlags() const;
    bool     flagsSatisfied(uint32_t mask) const;
    void     setEventFlags(uint32_t mask);
    void     clearFlagsMask(uint32_t mask);
    void     clearAllFlags();
    void     fireTelemetry(const Instruction& instr, bool succeeded);
    void     setZMotorPosition(float position);
    void     computeOperatingPoint(float targetPower);
    float    amplitudeForTargetPower(float realAdmittance, float targetPower);
    float    correctedForceGrams(float grams) const;
    static float forceGramsToAmps(float grams);
    uint8_t  findResonanceIndex();
    float    calculateCenterFrequency(uint8_t resonanceIndex);
    float    calculateDriveAmplitude(uint8_t resonanceIndex, float targetPower);

    // =========================================================================
    // Peripheral callback bridges
    // =========================================================================

    static BonderModule *s_instance;

    static void onContactSensorStateChanged(void *context, PinMonitorChannel::PinState state);
    static void onTimerDone(void *context, Timer *t);
    static void onYAxisRouterDone(void *context, RouterChannel *channel);
    static void onTAxisRouterDone(void *context, RouterChannel *channel);
    static void onPllEvent(void *context, PllModule::Event event);
    static void onForceCoilEvent(ForceCoilDriverModule::Event eventId);
    static bool onZMotorPositionSetpoint(void *context, float *positionSetpoint);
    static void onZMotorEvent(void *context, DcMotorPositionControllerModule::Event event);
    static void onClampStateChanged(void *context, SolenoidChannel::State state);
    static void onImpedanceScanned(void *context, complexf *v, complexf *c, complexf *i);
    static void onZVelocityMeasured(void *context, float velocity);
    void handleZVelocityMeasured(float velocity);

    float m_zMotorPositionSetpoint;
    bool  m_zMotorSetpointActive;
};

// Base interface for flat instruction programs executed by BonderModule's VM.
// The aliases keep concrete protocol tables compact and tied to the VM's
// canonical opcode and event definitions.
class BonderProtocol {
public:
    using Instruction = BonderModule::Instruction;
    using Op = BonderModule::Opcode;
    using B = BonderConfig;

    static constexpr uint32_t WAIT_TIMEOUT_MS = 10000U;
    static constexpr uint32_t US_TRANSFER_WAIT_TIMEOUT_MS = 65000U;

    static constexpr uint32_t EVENT_T_MOVE_COMPLETED      = BonderModule::EVENT_T_MOVE_COMPLETED;
    static constexpr uint32_t EVENT_Y_MOVE_COMPLETED      = BonderModule::EVENT_Y_MOVE_COMPLETED;
    static constexpr uint32_t EVENT_FORCE_COIL_SETTLED    = BonderModule::EVENT_FORCE_COIL_SETTLED;
    static constexpr uint32_t EVENT_CONTACT_CONNECTED     = BonderModule::EVENT_CONTACT_CONNECTED;
    static constexpr uint32_t EVENT_CONTACT_DISCONNECTED  = BonderModule::EVENT_CONTACT_DISCONNECTED;
    static constexpr uint32_t EVENT_TIMER_EXPIRED         = BonderModule::EVENT_TIMER_EXPIRED;
    static constexpr uint32_t EVENT_SCAN_COMPLETED        = BonderModule::EVENT_SCAN_COMPLETED;
    static constexpr uint32_t EVENT_US_POWER_TRANSFERRED  = BonderModule::EVENT_US_POWER_TRANSFERRED;
    static constexpr uint32_t EVENT_RIGHT_BUTTON_PRESSED  = BonderModule::EVENT_RIGHT_BUTTON_PRESSED;
    static constexpr uint32_t EVENT_RIGHT_BUTTON_RELEASED = BonderModule::EVENT_RIGHT_BUTTON_RELEASED;
    static constexpr uint32_t EVENT_LEFT_BUTTON_PRESSED   = BonderModule::EVENT_LEFT_BUTTON_PRESSED;
    static constexpr uint32_t EVENT_LEFT_BUTTON_RELEASED   = BonderModule::EVENT_LEFT_BUTTON_RELEASED;
    static constexpr uint32_t EVENT_Z_POSITION_REACHED    = BonderModule::EVENT_Z_POSITION_REACHED;
    static constexpr uint32_t EVENT_CLAMP_SETTLED         = BonderModule::EVENT_CLAMP_SETTLED;
    static constexpr uint32_t EVENT_CLAMP_TOGGLE_REQUESTED = BonderModule::EVENT_CLAMP_TOGGLE_REQUESTED;

    virtual ~BonderProtocol() = default;
    virtual const Instruction *getProtocolPtr() const = 0;
    virtual uint8_t getProtocolSize() const = 0;
    virtual bool requiresMotionControl() const { return true; }
};
