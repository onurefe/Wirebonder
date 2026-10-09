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
#include "bonder_module.hpp"

BonderModule::BonderModule(DcMotorPositionControllerModule *zMotorController,
                           ForceCoilDriverModule *forceCoilDriver,
                           RouterChannel *yAxisRouter,
                           RouterChannel *tAxisRouter,
                           PllModule *pll,
                           UsImpedanceScannerModule *impedanceScanner,
                           SolenoidChannel *clampSolenoid,
                           PinMonitorChannel *contactSensorMonitor,
                           PinMonitorChannel *leftMouseButtonMonitor,
                           PinMonitorChannel *rightMouseButtonMonitor,
                           Timer *timer)
    : m_resources(zMotorController,
                  forceCoilDriver,
                  yAxisRouter,
                  tAxisRouter,
                  pll,
                  impedanceScanner,
                  clampSolenoid,
                  contactSensorMonitor,
                  leftMouseButtonMonitor,
                  rightMouseButtonMonitor,
                  timer)
{
}

BonderConfig BonderModule::m_config = {};
float BonderModule::m_tailFeedHeight = 0.0f;
float BonderModule::m_tailVibrationBuildupTime = BONDER_MODULE_TAIL_VIBRATION_BUILDUP_TIME;
float BonderModule::m_tailAssistEnergy = 0.0f;

BonderCommandZMove             BonderModule::m_CmdZMove;
BonderCommandYMove             BonderModule::m_CmdYMove;
BonderCommandYReverse          BonderModule::m_CmdYReverse;
BonderCommandTMove             BonderModule::m_CmdTMove;
BonderCommandTimer             BonderModule::m_CmdTimer;
BonderCommandWaitZPosition     BonderModule::m_CmdWaitZPosition;
BonderCommandWaitContactEvents BonderModule::m_CmdWaitContactEvents;
BonderCommandWaitContactState  BonderModule::m_CmdWaitContactState;
BonderCommandWaitMouseLeftButtonEvents  BonderModule::m_CmdWaitMouseLeftButtonEvents;
BonderCommandWaitMouseLeftButtonState   BonderModule::m_CmdWaitMouseLeftButtonState;
BonderCommandWaitMouseRightButtonEvents BonderModule::m_CmdWaitMouseRightButtonEvents;
BonderCommandWaitMouseRightButtonState  BonderModule::m_CmdWaitMouseRightButtonState;
BonderCommandClampOpen         BonderModule::m_CmdClampOpen;
BonderCommandClampClose        BonderModule::m_CmdClampClose;
BonderCommandScan              BonderModule::m_CmdScan;
BonderCommandPll               BonderModule::m_CmdPll;
BonderCommandSetForce          BonderModule::m_CmdSetForce;
BonderCommandUsReport          BonderModule::m_CmdUsReport;
BonderCommandMzDrive           BonderModule::m_CmdMzDrive;

// =============================================================================
// Reporting
// =============================================================================

bool BonderModule::addStateChangedListenerCallback(void *context, BonderStateChangedCallback cb)
{
    return m_stateChangedCallbacks.add(context, cb);
}

bool BonderModule::removeStateChangedListenerCallback(void *context, BonderStateChangedCallback cb)
{
    return m_stateChangedCallbacks.remove(context, cb);
}

bool BonderModule::addErrorListenerCallback(void *context, BonderErrorCallback cb)
{
    return m_errorCallbacks.add(context, cb);
}

bool BonderModule::removeErrorListenerCallback(void *context, BonderErrorCallback cb)
{
    return m_errorCallbacks.remove(context, cb);
}

bool BonderModule::addUltrasonicReportListenerCallback(void *context, UltrasonicReportCallback cb)
{
    return m_ultrasonicReportCallbacks.add(context, cb);
}

bool BonderModule::removeUltrasonicReportListenerCallback(void *context, UltrasonicReportCallback cb)
{
    return m_ultrasonicReportCallbacks.remove(context, cb);
}

void BonderModule::setRunning(bool running)
{
    if (m_running == running) {
        return;
    }

    m_running = running;
    m_stateChangedCallbacks.invoke(!running);
}

void BonderModule::publishError(Error error)
{
    m_errorCallbacks.invoke(error);
}

void BonderModule::configure(const BonderConfig &config)
{
    m_config = config;

    if (m_config.numOfScannedFrequencies == 0U) {
        m_config.numOfScannedFrequencies = 1U;
    }
    if (m_config.numOfScannedFrequencies > BONDER_MODULE_SCAN_MAX_FREQUENCIES) {
        m_config.numOfScannedFrequencies = BONDER_MODULE_SCAN_MAX_FREQUENCIES;
    }

    // A fraction of the way up from the bottom of the window, not of the
    // height's raw value: heights are raw LVDT readings, whose zero is
    // wherever the sensor's electrical centre happens to sit.
    m_tailFeedHeight = BONDER_MODULE_ZAXIS_MIN_POSITION +
        (m_config.resetHeight - BONDER_MODULE_ZAXIS_MIN_POSITION) *
        BONDER_MODULE_TAIL_FEED_HEIGHT_FRACTION;

    /* The assist has to keep the tool ringing for the whole of the tail draw.
       That is the T axis returning to its origin from tail + tear, so the
       axis is asked how long its own profile takes and the energy follows
       from the power. The buildup dwell is part of the drive too. */
    if (m_resources.m_tAxisRouter != nullptr) {
        const float restoreTravel =
            m_config.tailDisplacement + m_config.tearDisplacement;
        const float assistDuration =
            m_tailVibrationBuildupTime +
            m_resources.m_tAxisRouter->estimateMoveDuration(restoreTravel);

        m_tailAssistEnergy = m_config.tailAssistPower * assistDuration *
                             BONDER_MODULE_TAIL_ASSIST_ENERGY_SAFETY_FACTOR;
    }
}

void BonderModule::onStart()
{
    if (m_resources.m_zMotorController == nullptr ||
        m_resources.m_forceCoilDriver == nullptr ||
        m_resources.m_yAxisRouter == nullptr ||
        m_resources.m_tAxisRouter == nullptr ||
        m_resources.m_pll == nullptr ||
        m_resources.m_usImpedanceScanner == nullptr ||
        m_resources.m_clampSolenoid == nullptr ||
        m_resources.m_contactSensorMonitor == nullptr ||
        m_resources.m_leftMouseButtonMonitor == nullptr ||
        m_resources.m_rightMouseButtonMonitor == nullptr ||
        m_resources.m_timer == nullptr) {
        setProcessError();
        return;
    }

    registerCommands();

    m_pc = 0U;
    m_instructionStarted = false;
    m_events.store(0U);
    setRunning(false);
}

void BonderModule::onStop()
{
    disengage();
}

void BonderModule::onExecute()
{
    executeCommands();
    checkErrorFlags();
    executeProtocol();
}

// =============================================================================
// VM
// =============================================================================

// Loads a protocol and runs it from pc 0. The protocol is walked only while the
// module is engaged, so it may be loaded before or after engaging.
bool BonderModule::startProtocol(const Instruction *protocol, uint8_t protocolLength)
{
    // The running protocol cannot be swapped underneath the pc.
    if (m_running) {
        return false;
    }

    m_protocol = protocol;
    m_protocolLength = protocolLength;
    m_pc = 0U;
    m_instructionStarted = false;
    m_stopping = false;
    setRunning((protocol != nullptr) && (protocolLength > 0U));

    return m_running;
}

// Asks the running protocol to end where it is. This is also the way out of a
// pc parked on a command that failed, so recovering from an error never needs
// the module disengaged. The pc stops advancing and
// nothing is torn out from under a move or a bond: the commands in flight are
// left to finish, and the module goes idle once the last of them is done. The
// stoppable ones -- the waits that may never be satisfied, because they are
// waiting on the operator -- are dropped straight away, so they cannot hold a
// stop open indefinitely.
void BonderModule::stopProtocol()
{
    if (!m_running) {
        return;
    }

    m_stopping = true;
    stopStoppableCommands();
}

// =============================================================================
// Engagement
//
// Engaging switches on the mechanisms this module drives continuously and
// rewinds the protocol; disengaging switches them back off. This is not the
// Process lifecycle -- a disengaged module is still operating, it just holds
// no actuator.
// =============================================================================

bool BonderModule::engage()
{
    if (!isOperating() || m_running) {
        return false;
    }

    if (!m_engaged) {
        if (!activateResources()) {
            return false;
        }
        m_engaged = true;
    }

    // Nothing latched carries into the engagement; whatever protocol is run on
    // it starts from a clear latch.
    m_events.store(0U);

    return true;
}

void BonderModule::disengage()
{
    stopActiveCommands();

    if (m_engaged) {
        deactivateResources();
        m_engaged = false;
    }

    m_instructionStarted = false;
    m_pc = 0U;
    m_stopping = false;
    setRunning(false);
}

// The closed loops are the only resources held for as long as the module is
// engaged: they regulate every tick, whether or not an instruction is using
// them. The routers, clamp solenoid, pin monitors and timer are actuated by
// individual commands, which claim and release them in their own start/stop.
bool BonderModule::activateResources()
{
    if (!m_resources.m_forceCoilDriver->enableControl()) {
        return false;
    }

    if (!m_resources.m_zMotorController->enableControl()) {
        m_resources.m_forceCoilDriver->disableControl();
        return false;
    }

    return true;
}

void BonderModule::deactivateResources()
{
    m_resources.m_zMotorController->disableControl();
    m_resources.m_forceCoilDriver->disableControl();
}

void BonderModule::stopStoppableCommands()
{
    for (uint8_t i = 0U; i < Opcode::NUM_COMMANDS; i++) {
        if (m_commandActive[i] && m_commandList[i] != nullptr &&
            m_commandList[i]->isStoppable()) {
            m_commandList[i]->stop();
            m_commandActive[i] = false;
        }
    }
}

void BonderModule::stopActiveCommands()
{
    for (uint8_t i = 0U; i < Opcode::NUM_COMMANDS; i++) {
        if (m_commandActive[i] && m_commandList[i] != nullptr) {
            m_commandList[i]->stop();
        }
        m_commandActive[i] = false;
    }
}

bool BonderModule::isWaitOpcode(Opcode opcode)
{
    return (opcode == Opcode::WAITFLAGS) ||
           (opcode == Opcode::WAITZPOSITION) ||
           (opcode == Opcode::WAITCONTACTEVENTS) ||
           (opcode == Opcode::WAITCONTACTSTATE) ||
           (opcode == Opcode::WAITMOUSELEFTBUTTONEVENTS) ||
           (opcode == Opcode::WAITMOUSELEFTBUTTONSTATE) ||
           (opcode == Opcode::WAITMOUSERIGHTBUTTONEVENTS) ||
           (opcode == Opcode::WAITMOUSERIGHTBUTTONSTATE);
}

// Mirrors the on*CmdEvent handlers: every flag a handler can set for this
// opcode has to be listed, or a stale one survives the re-arm.
uint32_t BonderModule::producedEventFlags(Opcode opcode)
{
    switch (opcode) {
    case Opcode::ZMOVE:
    case Opcode::MZDRIVE:
        return EVENT_Z_POSITION_REACHED | EVENT_POSITION_ERROR | EVENT_Z_CONTROLLER_INIT_ERROR;

    case Opcode::YMOVE:
    case Opcode::YREVERSE:
        return EVENT_Y_MOVE_COMPLETED | EVENT_AXIS_MOVE_ERROR;

    case Opcode::TMOVE:
        return EVENT_T_MOVE_COMPLETED | EVENT_AXIS_MOVE_ERROR;

    case Opcode::TIMER:
        return EVENT_TIMER_EXPIRED | EVENT_WAIT_TIMEOUT;

    case Opcode::CLAMPOPEN:
    case Opcode::CLAMPCLOSE:
        return EVENT_CLAMP_SETTLED | EVENT_WAIT_TIMEOUT;

    case Opcode::SCAN:
        return EVENT_SCAN_COMPLETED | EVENT_SCAN_ERROR;

    case Opcode::WAITZPOSITION:
        return EVENT_Z_HEIGHT_REACHED | EVENT_WAIT_TIMEOUT;

    // A wait raises the flag for whichever edge/state it was given, so both of
    // the pair are dropped when it is issued, along with its timeout.
    case Opcode::WAITCONTACTEVENTS:
    case Opcode::WAITCONTACTSTATE:
        return EVENT_CONTACT_CONNECTED | EVENT_CONTACT_DISCONNECTED |
               EVENT_WAIT_TIMEOUT;

    case Opcode::PLL:
        return EVENT_US_POWER_TRANSFERRED | EVENT_US_POWER_ERROR;

    case Opcode::SETFORCE:
        return EVENT_FORCE_COIL_SETTLED | EVENT_FORCE_COIL_ERROR;

    case Opcode::USREPORT:
        return EVENT_US_REPORT_READY;

    case Opcode::WAITMOUSELEFTBUTTONEVENTS:
    case Opcode::WAITMOUSELEFTBUTTONSTATE:
        return EVENT_LEFT_BUTTON_PRESSED | EVENT_LEFT_BUTTON_RELEASED |
               EVENT_WAIT_TIMEOUT;

    case Opcode::WAITMOUSERIGHTBUTTONEVENTS:
    case Opcode::WAITMOUSERIGHTBUTTONSTATE:
        return EVENT_RIGHT_BUTTON_PRESSED | EVENT_RIGHT_BUTTON_RELEASED |
               EVENT_WAIT_TIMEOUT;

    // WAITFLAGS and CLRFLAGS act on the latch itself and raise nothing.
    default:
        return 0U;
    }
}

void BonderModule::executeCommands()
{
    for (uint8_t i = 0U; i < Opcode::NUM_COMMANDS; i++) {
        if (!m_commandActive[i] || m_commandList[i] == nullptr) {
            continue;
        }

        const BonderCommand::InstrStatus status = m_commandList[i]->execute();
        m_commandStatus[i] = status;

        // Done and Error both leave the command idle; it released its own
        // resources on the way out.
        if (status != BonderCommand::InstrStatus::Running) {
            m_commandActive[i] = false;
        }
    }
}

// Errors reach the VM as latched flags rather than as direct calls: a command
// can fail from an ISR (the force-coil and PLL events come off the control
// chain), so the flag is set there and read here, on the main loop, where
// listeners can safely be invoked.
void BonderModule::checkErrorFlags()
{
    const uint32_t errorFlags = getEventFlags() & kErrorFlagsMask;

    if (errorFlags == 0U) {
        return;
    }

    // Consumed, so one failure is reported once.
    clearFlagsMask(errorFlags);
    failProtocol(errorFor(errorFlags));
}

BonderModule::Error BonderModule::errorFor(uint32_t errorFlags)
{
    /* Ahead of EVENT_POSITION_ERROR: a loop that never started explains a
       move that never arrived, so when both are set the specific cause is the
       one worth reporting. */
    if ((errorFlags & EVENT_Z_CONTROLLER_INIT_ERROR) != 0U) {
        return Error::UnableToStartPositionControl;
    }
    if ((errorFlags & EVENT_POSITION_ERROR) != 0U) {
        return Error::UnableToSetPosition;
    }
    if ((errorFlags & EVENT_AXIS_MOVE_ERROR) != 0U) {
        return Error::UnableToMoveAxis;
    }
    if ((errorFlags & EVENT_FORCE_COIL_ERROR) != 0U) {
        return Error::UnableToSetForceCoilCurrent;
    }
    if ((errorFlags & (EVENT_US_POWER_ERROR | EVENT_SCAN_ERROR)) != 0U) {
        return Error::InsufficientBondingPower;
    }

    return Error::ProtocolTimeout;
}

// A failure ends the protocol the same way stopProtocol() does -- the pc stops
// where it stands and the commands in flight wind down -- and tells the upper
// layer what went wrong. Recovery is the caller's: it runs the initialization
// protocol, which restores the axes, the force coil and the clamp.
void BonderModule::failProtocol(Error error)
{
    if (m_running) {
        m_stopping = true;
        stopStoppableCommands();
    }

    publishError(error);
}

// One instruction per call: a non-wait is started and the pc moves on, a wait
// is armed on the call that reaches it and polled on the calls after.
void BonderModule::executeProtocol()
{
    // A protocol only walks on an engaged module: its commands drive mechanisms
    // that engagement switches on.
    if (!m_engaged || !m_running || m_protocol == nullptr) {
        return;
    }

    // A stop request halts the pc where it stands; running off the end halts
    // it too. Either way the protocol is over once the commands it left
    // running have finished.
    if (m_stopping || m_pc >= m_protocolLength) {
        if (isAnyCommandActive()) {
            return;
        }

        m_stopping = false;
        m_instructionStarted = false;
        m_pc = 0U;
        setRunning(false);
        return;
    }

    const Instruction &instruction = m_protocol[m_pc];

    if (!m_instructionStarted) {
        beginInstruction(instruction);
    } else if (isWaitSatisfied(instruction)) {
        advanceProgramCounter();
    }
}

void BonderModule::beginInstruction(const Instruction &instruction)
{
    const uint8_t index = static_cast<uint8_t>(instruction.opcode);

    // Re-arm before issuing: whatever this instruction is about to raise must
    // not already be latched from an earlier one.
    clearFlagsMask(producedEventFlags(instruction.opcode));

    startInstruction(instruction);

    if (m_commandList[index] != nullptr) {
        m_commandActive[index] = true;
        m_commandStatus[index] = BonderCommand::InstrStatus::Running;
    }

    if (isWaitOpcode(instruction.opcode)) {
        m_instructionStarted = true;
    } else {
        // Everything but a wait is fire-and-forget: the command keeps running
        // in the background while the pc moves on.
        advanceProgramCounter();
    }
}

void BonderModule::advanceProgramCounter()
{
    m_instructionStarted = false;
    m_pc++;
}

bool BonderModule::isWaitSatisfied(const Instruction &instruction)
{
    if (instruction.opcode == Opcode::WAITFLAGS) {
        return consumeEventFlags(instruction.bonderVmArg);
    }

    return isCommandFinished(static_cast<uint8_t>(instruction.opcode));
}

bool BonderModule::consumeEventFlags(uint32_t mask)
{
    if ((getEventFlags() & mask) != mask) {
        return false;
    }

    // Consumed, so the same latch cannot satisfy a later wait.
    clearFlagsMask(mask);

    return true;
}

bool BonderModule::isCommandFinished(uint8_t index)
{
    if (m_commandActive[index]) {
        return false;
    }

    // A failed command is expected to have raised its error flag on the way
    // out, which checkErrorFlags() turns into a report and an orderly stop;
    // holding the pc here only keeps it from walking on in the meantime. One
    // that failed without raising anything leaves the pc parked, and
    // stopProtocol() is enough to clear that: the command is already inactive,
    // so the wind-down finds nothing to wait for and the module goes idle.
    if (m_commandStatus[index] == BonderCommand::InstrStatus::Error) {
        return false;
    }

    return true;
}

bool BonderModule::isAnyCommandActive() const
{
    for (uint8_t i = 0U; i < Opcode::NUM_COMMANDS; i++) {
        if (m_commandActive[i]) {
            return true;
        }
    }

    return false;
}

// Table entry and initialization in one call, so a command cannot be added to
// the one and forgotten in the other -- a command left uninitialized has no
// resources and faults the first time an instruction reaches it.
void BonderModule::registerCommand(Opcode opcode, BonderCommand &command,
                                   BonderCommand::EventOccurredCallback handler)
{
    m_commandList[opcode] = &command;
    command.init(&m_resources, handler, this);
}

void BonderModule::registerCommands()
{
    registerCommand(Opcode::ZMOVE, m_CmdZMove, &BonderModule::onZMoveCmdEvent);
    registerCommand(Opcode::YMOVE, m_CmdYMove, &BonderModule::onYMoveCmdEvent);
    registerCommand(Opcode::YREVERSE, m_CmdYReverse, &BonderModule::onYReverseCmdEvent);
    registerCommand(Opcode::TMOVE, m_CmdTMove, &BonderModule::onTMoveCmdEvent);
    registerCommand(Opcode::TIMER, m_CmdTimer, &BonderModule::onTimerCmdEvent);
    registerCommand(Opcode::WAITZPOSITION, m_CmdWaitZPosition,
                    &BonderModule::onWaitZPositionCmdEvent);
    registerCommand(Opcode::WAITCONTACTEVENTS, m_CmdWaitContactEvents,
                    &BonderModule::onWaitContactEventsCmdEvent);
    registerCommand(Opcode::WAITCONTACTSTATE, m_CmdWaitContactState,
                    &BonderModule::onWaitContactStateCmdEvent);
    registerCommand(Opcode::WAITMOUSELEFTBUTTONEVENTS, m_CmdWaitMouseLeftButtonEvents,
                    &BonderModule::onWaitMouseLeftButtonEventsCmdEvent);
    registerCommand(Opcode::WAITMOUSELEFTBUTTONSTATE, m_CmdWaitMouseLeftButtonState,
                    &BonderModule::onWaitMouseLeftButtonStateCmdEvent);
    registerCommand(Opcode::WAITMOUSERIGHTBUTTONEVENTS, m_CmdWaitMouseRightButtonEvents,
                    &BonderModule::onWaitMouseRightButtonEventsCmdEvent);
    registerCommand(Opcode::WAITMOUSERIGHTBUTTONSTATE, m_CmdWaitMouseRightButtonState,
                    &BonderModule::onWaitMouseRightButtonStateCmdEvent);
    registerCommand(Opcode::CLAMPOPEN, m_CmdClampOpen, &BonderModule::onClampOpenCmdEvent);
    registerCommand(Opcode::CLAMPCLOSE, m_CmdClampClose, &BonderModule::onClampCloseCmdEvent);
    registerCommand(Opcode::SCAN, m_CmdScan, &BonderModule::onScanCmdEvent);
    registerCommand(Opcode::PLL, m_CmdPll, &BonderModule::onPllCmdEvent);
    registerCommand(Opcode::SETFORCE, m_CmdSetForce, &BonderModule::onSetForceCmdEvent);
    registerCommand(Opcode::USREPORT, m_CmdUsReport, &BonderModule::onUsReportCmdEvent);
    registerCommand(Opcode::MZDRIVE, m_CmdMzDrive, &BonderModule::onMzDriveCmdEvent);

    // WAITFLAGS and CLRFLAGS act on the VM's own flag latch, so no command
    // backs them; their table entries stay null.
}

void BonderModule::startInstruction(const Instruction &instruction)
{
    void **args = const_cast<void **>(instruction.args);

    switch (instruction.opcode)
    {
    case BonderModule::Opcode::ZMOVE:
        startInstructionZMove(args);
        break;

    case BonderModule::Opcode::YMOVE:
        startInstructionYMove(args);
        break;

    case BonderModule::Opcode::YREVERSE:
        startInstructionYReverse(args);
        break;

    case BonderModule::Opcode::TMOVE:
        startInstructionTMove(args);
        break;

    case BonderModule::Opcode::TIMER:
        startInstructionTimer(args);
        break;

    case BonderModule::Opcode::WAITFLAGS:
        // Instruction-only: the mask is tested against the VM's latch while
        // the instruction runs, so there is nothing to start here.
        break;

    case BonderModule::Opcode::WAITMOUSELEFTBUTTONEVENTS:
        startInstructionWaitMouseLeftButtonEvents(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITMOUSELEFTBUTTONSTATE:
        startInstructionWaitMouseLeftButtonState(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITMOUSERIGHTBUTTONEVENTS:
        startInstructionWaitMouseRightButtonEvents(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITMOUSERIGHTBUTTONSTATE:
        startInstructionWaitMouseRightButtonState(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::CLRFLAGS:
        // Instruction-only, and instantaneous.
        clearFlagsMask(instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITZPOSITION:
        startInstructionWaitZPosition(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITCONTACTEVENTS:
        startInstructionWaitContactEvents(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::WAITCONTACTSTATE:
        startInstructionWaitContactState(args, instruction.bonderVmArg);
        break;

    case BonderModule::Opcode::CLAMPOPEN:
        startInstructionClampOpen(args);
        break;

    case BonderModule::Opcode::CLAMPCLOSE:
        startInstructionClampClose(args);
        break;

    case BonderModule::Opcode::SCAN:
        startInstructionScan(args);
        break;

    case BonderModule::Opcode::PLL:
        startInstructionPll(args);
        break;

    case BonderModule::Opcode::SETFORCE:
        startInstructionSetForce(args);
        break;

    case BonderModule::Opcode::USREPORT:
        startInstructionUsReport(args);
        break;

    case BonderModule::Opcode::MZDRIVE:
        startInstructionMzDrive(args);
        break;

    default:
        break;
    }
}

// args[0]: height (mm), args[1]: max speed (mm/s),
// args[2]: max acceleration (mm/s^2)
void BonderModule::startInstructionZMove(void **args)
{
    BonderCommandZMove::Args ordered_args;
    ordered_args.zPosition = *((float *)args[0]);
    ordered_args.maxSpeed = *((float *)args[1]);
    ordered_args.maxAcceleration = *((float *)args[2]);
    m_CmdZMove.start(&ordered_args);
}

// args[0]: absolute position (mm)
void BonderModule::startInstructionYMove(void **args)
{
    BonderCommandYMove::Args ordered_args;
    ordered_args.yPosition = *((float *)args[0]);
    m_CmdYMove.start(&ordered_args);
}

// args[0]: displacement (mm), travelled in the negative direction
void BonderModule::startInstructionYReverse(void **args)
{
    BonderCommandYReverse::Args ordered_args;
    ordered_args.displacement = *((float *)args[0]);
    m_CmdYReverse.start(&ordered_args);
}

// args[0]: position (mm), args[1]: relative flag. Tail and tear travels are
// relative to wherever the axis sits; homing is an absolute move to origin.
void BonderModule::startInstructionTMove(void **args)
{
    BonderCommandTMove::Args ordered_args;
    ordered_args.position = *((float *)args[0]);
    ordered_args.relative = *((bool *)args[1]);
    m_CmdTMove.start(&ordered_args);
}

// args[0]: duration (s)
void BonderModule::startInstructionTimer(void **args)
{
    BonderCommandTimer::Args ordered_args;
    ordered_args.durationSeconds = *((float *)args[0]);
    m_CmdTimer.start(&ordered_args);
}

// args[0]: height (mm); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitZPosition(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitZPosition::Args ordered_args;
    ordered_args.zPosition = *((float *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitZPosition.start(&ordered_args);
}

// args[0]: awaited edge (true = connecting); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitContactEvents(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitContactEvents::Args ordered_args;
    ordered_args.isRisingEdge = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitContactEvents.start(&ordered_args);
}

// args[0]: awaited state (true = connected); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitContactState(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitContactState::Args ordered_args;
    ordered_args.state = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitContactState.start(&ordered_args);
}

// args[0]: awaited edge (true = press); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitMouseLeftButtonEvents(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitMouseLeftButtonEvents::Args ordered_args;
    ordered_args.isRisingEdge = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitMouseLeftButtonEvents.start(&ordered_args);
}

// args[0]: awaited state (true = pressed); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitMouseLeftButtonState(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitMouseLeftButtonState::Args ordered_args;
    ordered_args.state = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitMouseLeftButtonState.start(&ordered_args);
}

// args[0]: awaited edge (true = press); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitMouseRightButtonEvents(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitMouseRightButtonEvents::Args ordered_args;
    ordered_args.isRisingEdge = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitMouseRightButtonEvents.start(&ordered_args);
}

// args[0]: awaited state (true = pressed); timeout in ms, 0 = forever
void BonderModule::startInstructionWaitMouseRightButtonState(void **args, uint32_t timeoutMs)
{
    BonderCommandWaitMouseRightButtonState::Args ordered_args;
    ordered_args.state = *((bool *)args[0]);
    ordered_args.timeoutMs = timeoutMs;
    m_CmdWaitMouseRightButtonState.start(&ordered_args);
}

void BonderModule::startInstructionClampOpen(void **args)
{
    (void)args;
    m_CmdClampOpen.start(nullptr);
}

void BonderModule::startInstructionClampClose(void **args)
{
    (void)args;
    m_CmdClampClose.start(nullptr);
}

// args[0]: number of frequencies, args[1]: start frequency (Hz),
// args[2]: stop frequency (Hz), args[3]: target power (W)
void BonderModule::startInstructionScan(void **args)
{
    BonderCommandScan::Args ordered_args;
    ordered_args.numFrequencies = *((uint16_t *)args[0]);
    ordered_args.startFrequency = *((float *)args[1]);
    ordered_args.stopFrequency = *((float *)args[2]);
    ordered_args.targetPower = *((float *)args[3]);
    m_CmdScan.start(&ordered_args);
}

// args[0]: energy (J), args[1]: max bonding duration (s). The operating point
// and the frequency-loop tuning come from the scan that produced them.
void BonderModule::startInstructionPll(void **args)
{
    const BonderCommandScan::Result &scan = m_CmdScan.getResult();

    BonderCommandPll::Args ordered_args;
    ordered_args.centerFrequency = scan.centerFrequency;
    ordered_args.driveAmplitude = scan.driveAmplitude;
    ordered_args.energyJoules = *((float *)args[0]);
    ordered_args.maxDurationSeconds = *((float *)args[1]);
    ordered_args.applyPidTuning = scan.pidTuningValid;
    ordered_args.pidGain = scan.pidGain;
    ordered_args.pidIntegralTc = scan.pidIntegralTc;
    ordered_args.pidDerivativeTc = scan.pidDerivativeTc;
    m_CmdPll.start(&ordered_args);
}

// args[0]: force (g), args[1]: Setup-measured calibration offset (g)
void BonderModule::startInstructionSetForce(void **args)
{
    BonderCommandSetForce::Args ordered_args;
    ordered_args.forceGrams = *((float *)args[0]);
    ordered_args.forceOffsetGrams = *((float *)args[1]);
    m_CmdSetForce.start(&ordered_args);
}

// The transducer figures come from the scan; the delivered power and duration
// are read off the PLL by the command itself.
void BonderModule::startInstructionUsReport(void **args)
{
    (void)args;

    const BonderCommandScan::Result &scan = m_CmdScan.getResult();

    BonderCommandUsReport::Args ordered_args;
    ordered_args.resonanceFrequency = scan.centerFrequency;
    ordered_args.qualityFactor = scan.qualityFactor;
    m_CmdUsReport.start(&ordered_args);
}

// args[0]: lower height (mm), args[1]: upper height (mm),
// args[2]: drive speed (mm/s), args[3]: stop distance (mm)
void BonderModule::startInstructionMzDrive(void **args)
{
    BonderCommandMzDrive::Args ordered_args;
    ordered_args.lowerHeight = *((float *)args[0]);
    ordered_args.upperHeight = *((float *)args[1]);
    ordered_args.maxSpeed = *((float *)args[2]);
    ordered_args.maxStopDistance = *((float *)args[3]);
    m_CmdMzDrive.start(&ordered_args);
}


void BonderModule::setEventFlags(uint32_t mask)
{
    m_events.fetch_or(mask);
}

void BonderModule::clearFlagsMask(uint32_t mask)
{
    m_events.fetch_and(~mask);
}

uint32_t BonderModule::getEventFlags() const
{
    return m_events.load();
}

/**
 *  Callbacks.
 */
void BonderModule::onZMoveCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandZMove::EventId::SetpointReached) {
        self->setEventFlags(EVENT_Z_POSITION_REACHED);
    } else if (eventId == BonderCommandZMove::EventId::PositionError) {
        self->setEventFlags(EVENT_POSITION_ERROR);
    } else if (eventId ==
               BonderCommandZMove::EventId::ControllerInitializationError) {
        self->setEventFlags(EVENT_Z_CONTROLLER_INIT_ERROR);
    } else if (eventId == BonderCommandZMove::EventId::ControllerInitializationError) {
        self->setEventFlags(EVENT_Z_CONTROLLER_INIT_ERROR);
    }
}

void BonderModule::onYMoveCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandYMove::EventId::MoveCompleted) {
        self->setEventFlags(EVENT_Y_MOVE_COMPLETED);
    } else if (eventId == BonderCommandYMove::EventId::TimedOut) {
        self->setEventFlags(EVENT_AXIS_MOVE_ERROR);
    }
}

void BonderModule::onYReverseCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandYReverse::EventId::MoveCompleted) {
        self->setEventFlags(EVENT_Y_MOVE_COMPLETED);
    } else if (eventId == BonderCommandYReverse::EventId::TimedOut) {
        self->setEventFlags(EVENT_AXIS_MOVE_ERROR);
    }
}

void BonderModule::onTMoveCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandTMove::EventId::MoveCompleted) {
        self->setEventFlags(EVENT_T_MOVE_COMPLETED);
    } else if (eventId == BonderCommandTMove::EventId::TimedOut) {
        self->setEventFlags(EVENT_AXIS_MOVE_ERROR);
    }
}

void BonderModule::onTimerCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandTimer::EventId::Expired) {
        self->setEventFlags(EVENT_TIMER_EXPIRED);   
    }
}

void BonderModule::onWaitZPositionCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    (void)eventParams;

    if (eventId == BonderCommandWaitZPosition::EventId::PositionReached) {
        self->setEventFlags(EVENT_Z_HEIGHT_REACHED);
    } else if (eventId == BonderCommandWaitZPosition::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
    }
}

void BonderModule::onWaitContactEventsCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitContactEvents::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitContactEvents::EventId::EventOccurred ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_CONTACT_CONNECTED
                            : EVENT_CONTACT_DISCONNECTED);
}

void BonderModule::onWaitContactStateCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitContactState::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitContactState::EventId::StateReached ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_CONTACT_CONNECTED
                            : EVENT_CONTACT_DISCONNECTED);
}

// A wait reports the edge/state that satisfied it, so the completion maps onto
// the matching button flag.
void BonderModule::onWaitMouseLeftButtonEventsCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitMouseLeftButtonEvents::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitMouseLeftButtonEvents::EventId::EventOccurred ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_LEFT_BUTTON_PRESSED
                            : EVENT_LEFT_BUTTON_RELEASED);
}

void BonderModule::onWaitMouseLeftButtonStateCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitMouseLeftButtonState::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitMouseLeftButtonState::EventId::StateReached ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_LEFT_BUTTON_PRESSED
                            : EVENT_LEFT_BUTTON_RELEASED);
}

void BonderModule::onWaitMouseRightButtonEventsCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitMouseRightButtonEvents::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitMouseRightButtonEvents::EventId::EventOccurred ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_RIGHT_BUTTON_PRESSED
                            : EVENT_RIGHT_BUTTON_RELEASED);
}

void BonderModule::onWaitMouseRightButtonStateCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandWaitMouseRightButtonState::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
        return;
    }

    if (eventId != BonderCommandWaitMouseRightButtonState::EventId::StateReached ||
        eventParams == nullptr) {
        return;
    }

    self->setEventFlags(*static_cast<const bool *>(eventParams)
                            ? EVENT_RIGHT_BUTTON_PRESSED
                            : EVENT_RIGHT_BUTTON_RELEASED);
}

void BonderModule::onClampOpenCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandClampOpen::EventId::ClampSettled) {
        self->setEventFlags(EVENT_CLAMP_SETTLED);
    } else if (eventId == BonderCommandClampOpen::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
    }
}

void BonderModule::onClampCloseCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandClampClose::EventId::ClampSettled) {
        self->setEventFlags(EVENT_CLAMP_SETTLED);
    } else if (eventId == BonderCommandClampClose::EventId::TimedOut) {
        self->setEventFlags(EVENT_WAIT_TIMEOUT);
    }
}

void BonderModule::onScanCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandScan::EventId::ScanCompleted) {
        self->setEventFlags(EVENT_SCAN_COMPLETED);
    } else if (eventId == BonderCommandScan::EventId::ScanFailed) {
        self->setEventFlags(EVENT_SCAN_ERROR);
    }    
}

void BonderModule::onPllCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (eventId == BonderCommandPll::EventId::PowerTransferred) {
        self->setEventFlags(EVENT_US_POWER_TRANSFERRED);
    } else if (eventId == BonderCommandPll::EventId::PowerError) {
        self->setEventFlags(EVENT_US_POWER_ERROR);
    }
}

void BonderModule::onSetForceCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    if (eventId == BonderCommandSetForce::EventId::Settled) {
        self->setEventFlags(EVENT_FORCE_COIL_SETTLED);
    } else if (eventId == BonderCommandSetForce::EventId::Error) {
        self->setEventFlags(EVENT_FORCE_COIL_ERROR);
    }
}

void BonderModule::onUsReportCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    
    if (eventId != BonderCommandUsReport::EventId::ReportReady ||
        eventParams == nullptr) {
        return;
    }

    const BonderCommandUsReport::Report *report =
        static_cast<const BonderCommandUsReport::Report *>(eventParams);

    const UltrasonicReport published{
        report->resonanceFrequency,
        report->qualityFactor,
        report->transferredPower,
        report->bondingDuration
    };

    self->setEventFlags(EVENT_US_REPORT_READY);
    self->m_ultrasonicReportCallbacks.invoke(published);
}

void BonderModule::onMzDriveCmdEvent(void *context, uint8_t eventId, void *eventParams)
{
    BonderModule *self = static_cast<BonderModule *>(context);

    // The lower bound is a commanded Z destination, so settling there is the
    // same event a ZMOVE raises.
    if (eventId == BonderCommandMzDrive::EventId::LowerBoundReached) {
        self->setEventFlags(EVENT_Z_POSITION_REACHED);
    } else if (eventId == BonderCommandMzDrive::EventId::PositionError) {
        self->setEventFlags(EVENT_POSITION_ERROR);
    } else if (eventId ==
               BonderCommandMzDrive::EventId::ControllerInitializationError) {
        self->setEventFlags(EVENT_Z_CONTROLLER_INIT_ERROR);
    }
}

