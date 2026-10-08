#pragma once

#include <atomic>
#include <cstdint>
#include "generic.h"
#include "configuration.h"
#include "complex.h"
#include "bonder_vm_resources.hpp"
#include "timer_expire_service.hpp"
#include "solenoid_service.hpp"
#include "fast_io.hpp"
#include "pin_monitor_service.hpp"
#include "stepper_router_service.hpp"
#include "dc_motor_position_controller_module.hpp"
#include "force_coil_module.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"

// =============================================================================
// BonderCommand
//
// One command owns one action on a small set of resources: it claims what it
// needs in start(), drives it to completion in execute(), and releases it in
// stop(). Nothing else is shared -- every value a command needs arrives in its
// Args and every value it produces leaves through its event callback (or a
// getter on the command itself). Sequencing, flag latching and error policy
// belong to whatever runs the commands, not to the commands.
// =============================================================================

class BonderCommand {
    public:
    BonderCommand()
        : m_resources(nullptr)
        , m_callbackContext(nullptr)
        , m_callback(nullptr)
        , m_stoppable(false)
        , m_timeoutMs(0U)
        , m_startTick(0U)
    {
    }
    enum class InstrStatus { Running, Error, Done };

    using EventOccurredCallback = void (*)(void *context, uint8_t eventId, void *eventParams);

    virtual ~BonderCommand() = default;

    virtual void init(BonderVMResources *resources, EventOccurredCallback callback, void *callbackContext) {
        m_resources = resources;
        m_callback = callback;
        m_callbackContext = callbackContext;
    }

    virtual bool start(void *args) {
        (void)args;
        return true;
    }

    virtual InstrStatus execute() {
        return InstrStatus::Done;
    }

    virtual bool stop() {
        return true;
    }

    // Whether the VM may cut this command short when a protocol is stopped
    // rather than let it run to its own completion. Off by default: a command
    // driving a mechanism has to finish what it started. A command that only
    // waits -- above all one waiting on the operator, which may never come --
    // sets it, so a stop is not held hostage.
    bool isStoppable() const { return m_stoppable; }

    protected:
    void notify(uint8_t eventId, void *eventParams = nullptr) {
        if (m_callback != nullptr) {
            m_callback(m_callbackContext, eventId, eventParams);
        }
    }

    // Deadline, in milliseconds on the 1 kHz TimerExpireService tick. Armed in
    // start(); zero waits forever. A command that cannot be dropped when a
    // protocol is stopped arms one, so a stalled mechanism fails into the
    // error path instead of holding the VM open.
    void armTimeout(uint32_t timeoutMs) {
        m_timeoutMs = timeoutMs;
        m_startTick = TimerExpireService::getTicks();
    }

    bool hasTimedOut() const {
        return (m_timeoutMs != 0U) &&
               ((TimerExpireService::getTicks() - m_startTick) >= m_timeoutMs);
    }

    BonderVMResources *m_resources;

    void *m_callbackContext;
    EventOccurredCallback m_callback;
    bool m_stoppable;

    private:
    uint32_t m_timeoutMs;
    uint32_t m_startTick;
};

// -----------------------------------------------------------------------------
// Z axis: position controller
// -----------------------------------------------------------------------------

// Drives the Z carriage to a height on a trapezoidal profile: the setpoint is
// walked from the control loop at the axis's own velocity limit, ramping up
// and decelerating into the destination over a fixed distance of travel, and
// the command completes when the carriage has caught up with it.
class BonderCommandZMove: public BonderCommand {
    public:
    typedef struct {
        float zPosition;
        // The profile the move runs at. The position loop's own velocity
        // clamp still applies on top, as the machine-level limit.
        float maxSpeed;          // mm/s
        float maxAcceleration;   // mm/s^2
    } Args;

    enum EventId: uint8_t {
        SetpointReached = 0,
        PositionError = 1,
        ControllerInitializationError = 2
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    // The carriage has caught up with the destination. Decided here rather
    // than taken from the position loop's SetpointReached, which reports on
    // whichever provider won the arbiter and repeats every tick.
    bool isMotionCompleted() const;
    // One control tick of the profile.
    void advanceProfile();

    float m_destination;
    float m_speed;
    float m_acceleration;

    // Walked towards m_destination by the control loop, which owns both of
    // these once the command is running.
    float m_setpoint;
    float m_velocity;

    static bool onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                         float *velocityFeedforward);

};

// Open-loop Z motion: a fixed drive for a fixed time, with both loops
// bypassed so the value goes straight through as a drive voltage. Nothing is
// controlled and nothing is measured -- it exists so a calibration can put the
// head somewhere known without depending on the very feedback it is about to
// calibrate. Stalling against a stop is inert: no integrator to wind up and no
// position error to amplify.
class BonderCommandOpenZMove: public BonderCommand {
    public:
    typedef struct {
        // Signed drive (volts): positive is the Z-increasing direction.
        float drive;
        // How long to apply it (seconds).
        float duration;
    } Args;

    enum EventId: uint8_t {
        MoveCompleted = 0,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    static bool onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                         float *velocityFeedforward);

    float m_drive;
};

// Declares wherever the head is now to be the Z origin, and reports it so the
// LVDT's offset can be corrected. Measures only -- getting the head there is
// the preceding open-loop moves' job.
class BonderCommandZReference: public BonderCommand {
    public:
    enum EventId: uint8_t {
        // eventParams: const float * (position the head was found at)
        OriginMeasured = 0,
    };

    bool start(void *args) override;
    InstrStatus execute() override;

    private:
    float m_measuredPosition;
};

// Operator-driven Z motion between two heights: the right mouse button drives
// towards heights.lower, the left towards heights.upper, releasing both stops
// in place. Completes once the carriage settles at the lower bound. The
// buttons are read straight off their monitors each tick, so the command holds
// the Z loop and both button monitors and needs nothing pushed into it.
//
// A held button produces a trapezoidal move at the speed given in the
// arguments: the position setpoint is walked from the control loop itself, so
// the speed is what was asked for rather than a by-product of clamping the
// loop's output. The ramp is specified as a distance rather than a time, so
// it occupies the same travel whatever speed the operator has chosen
// (a = v^2 / 2d). Releasing decelerates over that same distance, as does
// arriving at either bound; reversing runs the ramp through zero.
class BonderCommandMzDrive: public BonderCommand {
    public:
    // Driven by the operator, so it may never reach its lower bound.
    BonderCommandMzDrive() { m_stoppable = true; }

    typedef struct {
        float lowerHeight;
        float upperHeight;
        // Speed the operator drives at (mm/s, both directions).
        float maxSpeed;
        // Travel spent reaching that speed, and giving it back up (mm). A
        // distance rather than a rate, so the stop feels the same at any
        // speed setting.
        float maxStopDistance;
    } Args;

    enum EventId: uint8_t {
        LowerBoundReached = 0,
        PositionError     = 1,
        ControllerInitializationError = 2,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    enum class Direction: uint8_t { None, Lower, Raise };

    float m_lowerHeight;
    float m_upperHeight;
    float m_driveSpeed;
    // Derived from the drive speed and the acceleration distance (mm/s^2).
    float m_acceleration;

    // Walked by the control loop, in carriage coordinates, at m_velocity.
    // Both are written only from the setpoint callback once running.
    float m_setpoint;
    float m_velocity;
    // Decided in execute() from the buttons and read by the control loop.
    std::atomic<Direction> m_direction;

    // Set by the control loop once the profile has walked the setpoint onto
    // the descent target; cleared again if it walks back up. Arriving there is
    // what completes the command, not merely starting there.
    std::atomic<bool> m_destinationCommanded;

    // One control tick of the profile, in the order it happens: what the
    // buttons ask for, held back by what can still be stopped in time, slewed
    // into the current velocity, integrated, then kept within reach of the
    // carriage.
    void advanceProfile();
    float requestedVelocity() const;
    float approachLimited(float velocity) const;
    void advanceVelocity(float targetVelocity);
    void advanceSetpoint();
    void limitSetpointLead();

    // The route has run out and the carriage has caught up with it.
    bool isMotionCompleted() const;

    static bool onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                         float *velocityFeedforward);
};

// Waits until the Z carriage reaches a height, whichever way it is travelling:
// the direction is taken at start() from where the carriage is relative to the
// target, so the test is a level comparison that cannot be missed between
// polls the way a crossing could. A carriage already at or past the height
// completes immediately.
//
// Observes only -- the move itself belongs to whatever ZMOVE or MZDRIVE is in
// flight -- so it is the way to hang an action off a height rather than off a
// delay.
class BonderCommandWaitZPosition: public BonderCommand {
    public:
    BonderCommandWaitZPosition() { m_stoppable = true; }

    typedef struct {
        // Height (mm above the table), as ZMOVE takes.
        float zPosition;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        PositionReached = 0,
        TimedOut        = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    float m_targetHeight;
    // Whether the carriage was below the target when the wait started, i.e.
    // which side of it counts as "reached".
    bool m_awaitingAscent;
};

// -----------------------------------------------------------------------------
// Y and T axes: stepper routers
// -----------------------------------------------------------------------------

// Moves the Y axis to an absolute position.
class BonderCommandYMove: public BonderCommand {
    public:
    typedef struct {
        float yPosition;
    } Args;

    enum EventId: uint8_t {
        MoveCompleted = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_moveCompleted;
    static void onRouterDone(void *context, RouterChannel *channel);
};

// Moves the Y axis by a displacement in the negative direction, from wherever
// the axis currently sits.
class BonderCommandYReverse: public BonderCommand {
    public:
    typedef struct {
        float displacement;
    } Args;

    enum EventId: uint8_t {
        MoveCompleted = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_moveCompleted;
    static void onRouterDone(void *context, RouterChannel *channel);
};

// Moves the T axis. Tail and tear travels are relative to wherever the axis
// currently sits; homing the axis is an absolute move to the origin.
class BonderCommandTMove: public BonderCommand {
    public:
    typedef struct {
        float position;
        bool  relative;
    } Args;

    enum EventId: uint8_t {
        MoveCompleted = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_moveCompleted;
    static void onRouterDone(void *context, RouterChannel *channel);
};

// -----------------------------------------------------------------------------
// Timing and sensing
// -----------------------------------------------------------------------------

// Holds for a duration.
class BonderCommandTimer: public BonderCommand {
    public:
    // A dwell holds nothing but the timer; ending it early only shortens it.
    BonderCommandTimer() { m_stoppable = true; }

    typedef struct {
        float durationSeconds;
    } Args;

    enum EventId: uint8_t {
        Expired = 0,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_expired;
    static void onTimerDone(void *context, Timer *timer);
};

// Waits for a contact-sensor edge. Edge-triggered: the state the sensor is
// already in does not satisfy the wait, only a transition into it does.
// timeoutMs == 0 waits indefinitely, otherwise a lapsed wait ends in Error.
class BonderCommandWaitContactEvents: public BonderCommand {
    public:
    BonderCommandWaitContactEvents() { m_stoppable = true; }

    typedef struct {
        // true: wait for the connecting edge (sensor goes active),
        // false: wait for the disconnecting edge.
        bool isRisingEdge;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the edge that satisfied the wait)
        EventOccurred = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_isRisingEdge;
    bool m_occurred;
    static void onContactSensorStateChanged(void *context, PinMonitorChannel::PinState state);
};

// Waits until the contact sensor *reads* a state. Level-triggered, so it
// completes immediately when the sensor already sits there -- a descent faster
// than the lever can follow leaves no edge to wait for.
class BonderCommandWaitContactState: public BonderCommand {
    public:
    BonderCommandWaitContactState() { m_stoppable = true; }

    typedef struct {
        // true: wait until the sensor reads connected, false: disconnected.
        bool state;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the state that satisfied the wait)
        StateReached = 0,
        TimedOut     = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_awaitedState;
};

// Waits for a left mouse-button edge. Edge-triggered, like
// BonderCommandWaitContactEvents: a button already down does not satisfy a
// wait for the press.
class BonderCommandWaitMouseLeftButtonEvents: public BonderCommand {
    public:
    BonderCommandWaitMouseLeftButtonEvents() { m_stoppable = true; }

    typedef struct {
        // true: wait for the press, false: wait for the release.
        bool isRisingEdge;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the edge that satisfied the wait)
        EventOccurred = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_isRisingEdge;
    bool m_occurred;
    static void onButtonStateChanged(void *context, PinMonitorChannel::PinState state);
};

// Waits until the left mouse button *reads* a state. Level-triggered, so a
// button already held satisfies the wait for pressed immediately.
class BonderCommandWaitMouseLeftButtonState: public BonderCommand {
    public:
    BonderCommandWaitMouseLeftButtonState() { m_stoppable = true; }

    typedef struct {
        // true: wait until the button reads pressed, false: released.
        bool state;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the state that satisfied the wait)
        StateReached = 0,
        TimedOut     = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_awaitedState;
};

// Right mouse button, edge-triggered. Same contract as its left twin.
class BonderCommandWaitMouseRightButtonEvents: public BonderCommand {
    public:
    BonderCommandWaitMouseRightButtonEvents() { m_stoppable = true; }

    typedef struct {
        // true: wait for the press, false: wait for the release.
        bool isRisingEdge;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the edge that satisfied the wait)
        EventOccurred = 0,
        TimedOut      = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_isRisingEdge;
    bool m_occurred;
    static void onButtonStateChanged(void *context, PinMonitorChannel::PinState state);
};

// Right mouse button, level-triggered. Same contract as its left twin.
class BonderCommandWaitMouseRightButtonState: public BonderCommand {
    public:
    BonderCommandWaitMouseRightButtonState() { m_stoppable = true; }

    typedef struct {
        // true: wait until the button reads pressed, false: released.
        bool state;
        // Milliseconds, measured on the 1 kHz TimerExpireService tick.
        uint32_t timeoutMs;
    } Args;

    enum EventId: uint8_t {
        // eventParams: const bool * (the state that satisfied the wait)
        StateReached = 0,
        TimedOut     = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_awaitedState;
};

// -----------------------------------------------------------------------------
// Clamp solenoid
// -----------------------------------------------------------------------------

// Energizes the clamp solenoid and completes once it has settled open.
class BonderCommandClampOpen: public BonderCommand {
    public:
    enum EventId: uint8_t {
        ClampSettled = 0,
        TimedOut     = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_settled;
    static void onClampStateChanged(void *context, SolenoidChannel::State state);
};

// Deenergizes the clamp solenoid and completes once it has settled closed.
class BonderCommandClampClose: public BonderCommand {
    public:
    enum EventId: uint8_t {
        ClampSettled = 0,
        TimedOut     = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    bool m_settled;
    static void onClampStateChanged(void *context, SolenoidChannel::State state);
};

// -----------------------------------------------------------------------------
// Ultrasonics
// -----------------------------------------------------------------------------

// Sweeps the transducer's impedance and derives the operating point the PLL
// should be driven at. The scan buffers and the fit belong to the command;
// the result leaves through getResult(), so no other command reads its state.
class BonderCommandScan: public BonderCommand {
    public:
    typedef struct {
        uint16_t numFrequencies;
        float    startFrequency;
        float    stopFrequency;
        float    targetPower;
    } Args;

    typedef struct {
        float centerFrequency;
        float driveAmplitude;
        float qualityFactor;
        // Frequency-PID tuning derived from the fitted transducer; only
        // meaningful when pidTuningValid, and applied by the PLL command.
        bool  pidTuningValid;
        float pidGain;
        float pidIntegralTc;
        float pidDerivativeTc;
        const complexf *impedances;
        uint16_t numFrequencies;
    } Result;

    enum EventId: uint8_t {
        // eventParams: const Result *
        ScanCompleted = 0,
        ScanFailed    = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    const Result& getResult() const { return m_result; }

    private:
    void computeOperatingPoint();
    uint8_t findResonanceIndex() const;
    float calculateCenterFrequency(uint8_t resonanceIndex) const;
    float calculateDriveAmplitude(uint8_t resonanceIndex) const;
    static float amplitudeForTargetPower(float realAdmittance, float targetPower);

    static void onImpedanceScanned(void *context, complexf *v, complexf *c, complexf *i);

    uint16_t m_numFrequencies;
    float    m_startFrequency;
    float    m_frequencyStep;
    float    m_targetPower;
    bool     m_scanCompleted;
    bool     m_scanFailed;
    Result   m_result;

    complexf m_voltagePhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_currentPhasors[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
    complexf m_impedances[BONDER_MODULE_SCAN_MAX_FREQUENCIES];
};

// Drives the transducer at the operating point until the requested energy has
// been delivered, and retunes the frequency loop first when the scan produced
// a trusted tuning.
class BonderCommandPll: public BonderCommand {
    public:
    typedef struct {
        float centerFrequency;
        float driveAmplitude;
        float energyJoules;
        float maxDurationSeconds;
        bool  applyPidTuning;
        float pidGain;
        float pidIntegralTc;
        float pidDerivativeTc;
    } Args;

    enum EventId: uint8_t {
        PowerTransferred = 0,
        PowerError       = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    static void onPllEvent(void *context, PllModule::Event event);

    bool m_transferred;
    bool m_failed;
};

// Emits the ultrasonic result of the bond that just ran. Reads the PLL's
// delivered power and duration; the transducer figures come from the scan
// that produced them.
class BonderCommandUsReport: public BonderCommand {
    public:
    typedef struct {
        float resonanceFrequency;
        float qualityFactor;
    } Args;

    typedef struct {
        float resonanceFrequency;
        float qualityFactor;
        float transferredPower;
        float bondingDuration;
    } Report;

    enum EventId: uint8_t {
        // eventParams: const Report *
        ReportReady = 0,
    };

    bool start(void *args) override;
    InstrStatus execute() override;

    private:
    Report m_report;
};

// -----------------------------------------------------------------------------
// Force coil
// -----------------------------------------------------------------------------

// Commands a bonding force and completes once the current loop has settled
// there. Grams are converted to amps here -- the one place that is specific
// to force.
class BonderCommandSetForce: public BonderCommand {
    public:
    typedef struct {
        float forceGrams;
        // Setup-measured calibration offset (gauge reading minus commanded).
        float forceOffsetGrams;
    } Args;

    enum EventId: uint8_t {
        Settled = 0,
        Error   = 1,
    };

    bool start(void *args) override;
    InstrStatus execute() override;
    bool stop() override;

    private:
    static float correctedForceGrams(float grams, float offset);
    static float forceGramsToAmps(float grams);
    static void onForceCoilEvent(void *context, ForceCoilDriverModule::Event event);

    bool m_settled;
    bool m_failed;
};
