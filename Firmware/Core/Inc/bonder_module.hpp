#pragma once

#include "callback_list.hpp"
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
#include "bonder_vm_resources.hpp"
#include "bonder_commands.hpp"

class BonderModule: public Process {
    public:
    BonderModule(DcMotorPositionControllerModule *zMotorController,
                 ForceCoilDriverModule *forceCoilDriver,
                 RouterChannel *yAxisRouter,
                 RouterChannel *tAxisRouter,
                 PllModule *pll,
                 UsImpedanceScannerModule *impedanceScanner,
                 SolenoidChannel *clampSolenoid,
                 PinMonitorChannel *contactSensorMonitor,
                 PinMonitorChannel *leftMouseButtonMonitor,
                 PinMonitorChannel *rightMouseButtonMonitor,
                 Timer *timer);

    enum Opcode : uint8_t {
        ZMOVE = 0,   // arg: height (mm); re-arms Z_POSITION_REACHED
        YMOVE,       // arg: position (mm); re-arms Y_MOVE_COMPLETED
        YREVERSE,    // arg: displacement (mm); moves to
                     // currentYPosition - displacement; re-arms Y_MOVE_COMPLETED
        TMOVE,       // arg: position (mm); re-arms T_MOVE_COMPLETED
        TIMER,       // arg: duration (s); re-arms TIMER_EXPIRED
        WAITFLAGS,   // bonderVmArg: mask of flags that must all be set;
                     // consumed on completion. Instruction-only: the VM tests
                     // its own latch, so no command backs this opcode
        WAITCONTACTEVENTS, // arg0: awaited edge (true = connecting);
                     // bonderVmArg: timeout. Edge-triggered, so the state the
                     // sensor already sits in does not satisfy it
        WAITCONTACTSTATE,  // arg0: awaited state (true = connected);
                     // bonderVmArg: timeout. Blocks until the contact sensor
                     // *reads* that state, i.e. the lever has caught up with
                     // the Z carriage; unlike WAITCONTACTEVENTS this is level-
                     // not edge-triggered, so it completes immediately when
                     // the lever is already seated
        WAITZPOSITION, // arg0: height (mm); bonderVmArg: timeout. Blocks
                     // until the carriage reaches that height from wherever
                     // it started, so an action can be hung off a height
                     // rather than off a delay. Observes only -- the move
                     // belongs to the ZMOVE or MZDRIVE in flight
        WAITMOUSELEFTBUTTONEVENTS,  // arg0: awaited edge (true = press);
                     // bonderVmArg: timeout. Edge-triggered
        WAITMOUSELEFTBUTTONSTATE,   // arg0: awaited state (true = pressed);
                     // bonderVmArg: timeout. Level-triggered, so a button
                     // already held satisfies it immediately
        WAITMOUSERIGHTBUTTONEVENTS, // right button, edge-triggered
        WAITMOUSERIGHTBUTTONSTATE,  // right button, level-triggered
        CLRFLAGS,    // bonderVmArg: mask of flags to clear unconditionally.
                     // Instruction-only, like WAITFLAGS
        CLAMPOPEN,   // re-arms CLAMP_SETTLED (set immediately if already open)
        CLAMPCLOSE,  // re-arms CLAMP_SETTLED (set immediately if already closed)
        SCAN,        // arg: target power (W); re-arms SCAN_COMPLETED; the
                     // operating point is computed when the scan finishes
        PLL,         // arg: energy (J); drives the last computed operating
                     // point; re-arms US_POWER_TRANSFERRED
        SETFORCE,    // arg: force (g); re-arms FORCE_COIL_SETTLED
        USREPORT,    // emits the latest scan/PLL result
        MZDRIVE,     // arg0/arg1: the heights the operator drives between;
                     // arg2: the speed they drive at (0 keeps the machine's
                     // own limits). Right button drives toward arg0, left
                     // toward arg1; releasing both stops in place and
                     // re-pressing resumes. Raises Z_POSITION_REACHED once
                     // the carriage settles at arg0
        NUM_COMMANDS    // JUST FOR COUNTING NUMBER OF COMMANDS.
    };
    
    enum EventFlag : uint32_t {
        EVENT_T_MOVE_COMPLETED          = 1u << 0,
        EVENT_Y_MOVE_COMPLETED          = 1u << 1,
        EVENT_FORCE_COIL_SETTLED        = 1u << 2,
        EVENT_CONTACT_CONNECTED         = 1u << 3,
        EVENT_CONTACT_DISCONNECTED      = 1u << 4,
        EVENT_TIMER_EXPIRED             = 1u << 5,
        EVENT_SCAN_COMPLETED            = 1u << 6,
        EVENT_SCAN_ERROR                = 1u << 7,
        EVENT_US_POWER_TRANSFERRED      = 1u << 8,
        EVENT_RIGHT_BUTTON_PRESSED      = 1u << 9,
        EVENT_RIGHT_BUTTON_RELEASED     = 1u << 10,
        EVENT_POSITION_ERROR            = 1u << 11,
        // The Z position loop was not running when a move went to service it.
        // Kept apart from EVENT_POSITION_ERROR, which means the loop ran and
        // the carriage did not arrive: one is a control chain that never
        // started, the other a move that failed. They call for opposite
        // investigations, so they are not worth merging.
        EVENT_Z_CONTROLLER_INIT_ERROR   = 1u << 12,
        EVENT_FORCE_COIL_ERROR          = 1u << 13,
        EVENT_US_POWER_ERROR            = 1u << 14,
        EVENT_Z_POSITION_REACHED        = 1u << 15,
        EVENT_CLAMP_SETTLED             = 1u << 16,
        EVENT_WAIT_TIMEOUT              = 1u << 17,
        EVENT_LEFT_BUTTON_PRESSED       = 1u << 18,
        EVENT_LEFT_BUTTON_RELEASED      = 1u << 19,
        EVENT_CLAMP_TOGGLE_REQUESTED    = 1u << 20,
        EVENT_US_REPORT_READY           = 1u << 21,
        // A Y or T move did not complete within its deadline. Kept apart from
        // EVENT_WAIT_TIMEOUT so a stalled axis is distinguishable from a wait
        // that simply lapsed.
        EVENT_AXIS_MOVE_ERROR           = 1u << 23,
        // A WAITZPOSITION saw its height go by. Kept apart from
        // EVENT_Z_POSITION_REACHED, which means a commanded move has settled.
        EVENT_Z_HEIGHT_REACHED          = 1u << 24
    };
    
    enum class Error {
        // The transducer could not be driven to the requested power/energy.
        InsufficientBondingPower,
        // Force-coil current control could not reach its requested setpoint.
        UnableToSetForceCoilCurrent,
        // Z-axis position control ran, but the carriage did not reach its
        // setpoint in time.
        UnableToSetPosition,
        // The Z position loop was not running when a move needed it, so the
        // axis was never commanded at all.
        UnableToStartPositionControl,
        // A Y or T move did not complete in time.
        UnableToMoveAxis,
        // A wait did not see its completion within its timeout.
        ProtocolTimeout
    };

    struct UltrasonicReport {
        float resonanceFrequency;
        float qualityFactor;
        float transferredPower;
        float bondingDuration;
    };

    // isIdle: whether a protocol is running, not whether the module is engaged.
    using BonderStateChangedCallback = void (*)(void *context, bool isIdle);
    using BonderErrorCallback        = void (*)(void *context, Error error);
    using UltrasonicReportCallback   =
        void (*)(void *context, const UltrasonicReport &report);

    static const uint8_t BONDER_INSTRUCTION_MAX_ARGS = 4;

    struct Instruction {
        Opcode      opcode;
        void        *args[BONDER_INSTRUCTION_MAX_ARGS];
        // Opcode-specific scalar operand. A wait's timeout in milliseconds
        // most of the time; the flag mask for WAITFLAGS and CLRFLAGS.
        uint32_t    bonderVmArg;
    };

    // Producers run in mixed contexts (main loop, control-update chain, timer
    // and pin-monitor ISRs) while the VM consumes from the main loop, so the
    // latch is atomic: fetch_or/fetch_and compile to LDREX/STREX on Cortex-M4
    // and no set can be lost to a concurrent clear.
    std::atomic<uint32_t> m_events{0U};

    // Live machine configuration. Instruction operands are raw pointers, so a
    // protocol binds an operand to a field of this and always reads the
    // current value. Static because the address has to be a compile-time
    // constant for a protocol table to live in flash -- and because the module
    // is a singleton in practice, like the command instances below.
    static BonderConfig m_config;

    /* Height at which the tail feed is released on the way up from the tear.
       Not part of the profile: configure() derives it from the reset height,
       and it is a plain variable so the value can be moved while the timing
       is being dialled in. A protocol binds its WAITZPOSITION operand here. */
    static float m_tailFeedHeight;

    /* Dwell (s) between starting the tail-assist drive and drawing the tail,
       so the transducer has rung up and the wire is pulled through a tool
       that is already vibrating. Same treatment as the height above: module
       state, not a profile parameter, while the timing is being dialled in. */
    static float m_tailVibrationBuildupTime;

    /* Energy (J) the tail-assist drive is given. Derived, not configured:
       configure() sizes it so the drive outlasts the T axis's restore move
       (see BONDER_MODULE_TAIL_ASSIST_ENERGY_SAFETY_FACTOR). */
    static float m_tailAssistEnergy;


    void setEventFlags(uint32_t mask);
    void clearFlagsMask(uint32_t mask);
    uint32_t getEventFlags() const;

    void configure(const BonderConfig &config);
    const BonderConfig &getConfig() const { return m_config; }

    // Loads a protocol and runs it from pc 0. Must not be called while one is
    // running. Independent of engagement -- the protocol is walked only while
    // the module is engaged, so either may come first.
    bool startProtocol(const Instruction *protocol, uint8_t protocolLength);

    // Ends the running protocol where it stands. The pc stops advancing and
    // the commands already in flight are left to finish; the module goes idle
    // (isRunning() false, listeners told) once the last of them is done. The
    // module stays engaged.
    void stopProtocol();

    // Switches on the mechanisms the module holds continuously; disengage()
    // switches them back off. Programs are run on an engaged module, but
    // engaging needs no protocol of its own. Distinct from the Process
    // lifecycle: a disengaged module is still operating, it just holds no
    // actuator.
    bool engage();
    void disengage();

    bool isEngaged() const { return m_engaged; }
    bool isRunning() const { return m_running; }

    // Separate registries, so a listener interested in only one of these does
    // not have to register a stub for the others.
    bool addStateChangedListenerCallback(void *context, BonderStateChangedCallback cb);
    bool removeStateChangedListenerCallback(void *context, BonderStateChangedCallback cb);
    bool addErrorListenerCallback(void *context, BonderErrorCallback cb);
    bool removeErrorListenerCallback(void *context, BonderErrorCallback cb);
    bool addUltrasonicReportListenerCallback(void *context, UltrasonicReportCallback cb);
    bool removeUltrasonicReportListenerCallback(void *context, UltrasonicReportCallback cb);

    private:
    void onStart() override;
    void onExecute() override;
    void onStop() override;

    // Services every command that has been started and has not finished yet,
    // so instructions the pc has already walked past keep running in the
    // background.
    void executeCommands();
    // Advances the pc by one instruction per call, except on a wait, which
    // holds it until satisfied. The protocol ends when the pc has run off the
    // end and every command it started has finished.
    void executeProtocol();
    // Starts the instruction at the pc and marks its command in flight; a
    // non-wait advances the pc in the same call.
    void beginInstruction(const Instruction &instruction);
    void advanceProgramCounter();
    // Whether the wait holding the pc may release it.
    bool isWaitSatisfied(const Instruction &instruction);
    // Tests the mask against the latch and clears it once satisfied.
    bool consumeEventFlags(uint32_t mask);
    // Whether the command backing a wait has finished; a failed one faults
    // the VM instead of releasing the pc.
    bool isCommandFinished(uint8_t index);
    bool isAnyCommandActive() const;
    // Only these hold the pc; everything else is fire-and-forget, and a
    // protocol that needs to sit on a command's completion follows it with a
    // WAITFLAGS on the flag that command raises.
    static bool isWaitOpcode(Opcode opcode);
    // The flags an opcode's command can raise. Dropped when the instruction
    // is issued, so a latch left over from an earlier one cannot satisfy the
    // wait that follows it.
    static uint32_t producedEventFlags(Opcode opcode);

    // Single place a protocol's idle/busy transition is announced from.
    void setRunning(bool running);
    void publishError(Error error);

    // Every failure the module reports goes through here: read the latched
    // error flags on the main loop, report the first one, and end the protocol.
    void checkErrorFlags();
    void failProtocol(Error error);
    static Error errorFor(uint32_t errorFlags);

    // Flags that mean the protocol cannot go on. Raised by the command that
    // failed (possibly from an ISR) and consumed by checkErrorFlags().
    static constexpr uint32_t kErrorFlagsMask =
        EVENT_POSITION_ERROR | EVENT_Z_CONTROLLER_INIT_ERROR |
        EVENT_AXIS_MOVE_ERROR | EVENT_FORCE_COIL_ERROR | EVENT_US_POWER_ERROR |
        EVENT_SCAN_ERROR | EVENT_WAIT_TIMEOUT;

    // Enable/disable the continuously held mechanisms (the Z position loop and
    // the force coil). Rolls back on a partial failure.
    bool activateResources();
    void deactivateResources();
    // Releases whatever the in-flight commands still hold.
    void stopActiveCommands();
    // Drops only the in-flight commands that may be cut short, leaving the
    // rest to finish. Used when a protocol is stopped.
    void stopStoppableCommands();

    void registerCommands();
    void registerCommand(Opcode opcode, BonderCommand &command,
                         BonderCommand::EventOccurredCallback handler);
    void startInstruction(const Instruction &instruction);

    // Each dispatcher unpacks the instruction's raw argument slots into its
    // command's Args and starts it. Values a command needs but the protocol
    // does not carry (the operating point a scan produced) are routed here,
    // so no command reads another's state.
    void startInstructionZMove(void **args);
    void startInstructionYMove(void **args);
    void startInstructionYReverse(void **args);
    void startInstructionTMove(void **args);
    void startInstructionTimer(void **args);
    void startInstructionWaitZPosition(void **args, uint32_t timeoutMs);
    void startInstructionWaitContactEvents(void **args, uint32_t timeoutMs);
    void startInstructionWaitContactState(void **args, uint32_t timeoutMs);
    void startInstructionWaitMouseLeftButtonEvents(void **args, uint32_t timeoutMs);
    void startInstructionWaitMouseLeftButtonState(void **args, uint32_t timeoutMs);
    void startInstructionWaitMouseRightButtonEvents(void **args, uint32_t timeoutMs);
    void startInstructionWaitMouseRightButtonState(void **args, uint32_t timeoutMs);
    void startInstructionClampOpen(void **args);
    void startInstructionClampClose(void **args);
    void startInstructionScan(void **args);
    void startInstructionPll(void **args);
    void startInstructionSetForce(void **args);
    void startInstructionUsReport(void **args);
    void startInstructionMzDrive(void **args);

    // Everything the commands drive, in one bag they can be handed by init().
    BonderVMResources m_resources;

    ListenerList<bool>                    m_stateChangedCallbacks;
    ListenerList<Error>                   m_errorCallbacks;
    ListenerList<const UltrasonicReport &> m_ultrasonicReportCallbacks;

    BonderCommand *m_commandList[Opcode::NUM_COMMANDS] = {};

    // Which commands have been started and not yet reported completion, and
    // what they last reported. Indexed by opcode, like m_commandList.
    bool m_commandActive[Opcode::NUM_COMMANDS] = {};
    BonderCommand::InstrStatus m_commandStatus[Opcode::NUM_COMMANDS] = {};

    const Instruction *m_protocol = nullptr;
    uint8_t m_protocolLength = 0U;
    uint8_t m_pc = 0U;
    // Entry guard for the instruction at the pc, so a blocking one is started
    // once and then only polled.
    bool m_instructionStarted = false;
    bool m_running = false;
    // Set by stopProtocol(): the pc is frozen and the module is waiting for
    // the in-flight commands to finish before it reports idle.
    bool m_stopping = false;
    // Whether the continuously held mechanisms are switched on. Tracked apart
    // from m_running: a protocol that runs to its end leaves the module engaged
    // until the caller disengages it.
    bool m_engaged = false;

    static BonderCommandZMove             m_CmdZMove;
    static BonderCommandYMove             m_CmdYMove;
    static BonderCommandYReverse          m_CmdYReverse;
    static BonderCommandTMove             m_CmdTMove;
    static BonderCommandTimer             m_CmdTimer;
    static BonderCommandWaitZPosition     m_CmdWaitZPosition;
    static BonderCommandWaitContactEvents m_CmdWaitContactEvents;
    static BonderCommandWaitContactState  m_CmdWaitContactState;
    static BonderCommandWaitMouseLeftButtonEvents  m_CmdWaitMouseLeftButtonEvents;
    static BonderCommandWaitMouseLeftButtonState   m_CmdWaitMouseLeftButtonState;
    static BonderCommandWaitMouseRightButtonEvents m_CmdWaitMouseRightButtonEvents;
    static BonderCommandWaitMouseRightButtonState  m_CmdWaitMouseRightButtonState;
    static BonderCommandClampOpen         m_CmdClampOpen;
    static BonderCommandClampClose        m_CmdClampClose;
    static BonderCommandScan              m_CmdScan;
    static BonderCommandPll               m_CmdPll;
    static BonderCommandSetForce          m_CmdSetForce;
    static BonderCommandUsReport          m_CmdUsReport;
    static BonderCommandMzDrive           m_CmdMzDrive;

    static void onZMoveCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onYMoveCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onYReverseCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onTMoveCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onTimerCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitZPositionCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitContactEventsCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitContactStateCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitMouseLeftButtonEventsCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitMouseLeftButtonStateCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitMouseRightButtonEventsCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onWaitMouseRightButtonStateCmdEvent(void *context, uint8_t eventId, void *eventParams);

    static void onClampOpenCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onClampCloseCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onScanCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onPllCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onSetForceCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onUsReportCmdEvent(void *context, uint8_t eventId, void *eventParams);
    static void onMzDriveCmdEvent(void *context, uint8_t eventId, void *eventParams);
};