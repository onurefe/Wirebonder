#include "bonder_module.hpp"
#include "transducer_analyzer.hpp"
#include <cmath>

// =============================================================================
// Static definitions
// =============================================================================

BonderModule *BonderModule::s_instance = nullptr;

// Drives the machine to the idle posture and holds the VM until Z has
// actually reached reset height; only then does protocol pc 0 execute.
const BonderModule::Instruction BonderModule::kResetPrologue[4] = {
    {Opcode::SETFORCE, nullptr, 0U, 0U},
    {Opcode::CLAMPCLOSE, nullptr, 0U, 0U},
    {Opcode::ZMOVE, &BonderConfig::resetHeight, 0U, 0U},
    {Opcode::WAIT, nullptr, EVENT_Z_POSITION_REACHED,
     BonderProtocol::WAIT_TIMEOUT_MS},
};

// =============================================================================
// Public interface
// =============================================================================

BonderModule::BonderModule(DcMotorPositionControllerModule *zMotorController,
                           ForceCoilDriverModule *forceCoilDriver,
                           RouterChannel *yAxisRouter,
                           RouterChannel *tAxisRouter,
                           PllModule *pll,
                           UsImpedanceScannerModule *impedanceScanner,
                           DirectSolenoidChannel *clampSolenoid,
                           PinMonitorChannel *contactSensorMonitor,
                           Timer *timer)
    : m_config{}
    , m_centerFrequency(0.0f)
    , m_driveAmplitude(0.0f)
    , m_scanTargetPower(0.0f)
    , m_qualityFactor(0.0f)

    // VM runtime state
    , m_activityState(ActivityState::Idle)
    , m_program(nullptr)
    , m_programLength(0U)
    , m_protocolRequiresMotionControl(true)
    , m_motionControlEnabled(false)
    , m_inPrologue(false)
    , m_pc(0U)
    , m_instrStarted(false)
    , m_waitStartTick(0U)
    , m_armGatePc(0U)
    , m_armGateValid(false)

    // Hardware event and error flags
    , m_eventFlags(0U)

    , m_clampCommandTarget(DirectSolenoidChannel::State::DEENERGIZED)

    // Manual Z-leveling jog state
    , m_manualZTarget(0.0f)
    , m_manualLevelingLastTick(0U)
    , m_raiseButtonHeld(false)
    , m_lowerButtonHeld(false)

    // External notifications
    , m_stateChangedCallback(nullptr)
    , m_errorCallback(nullptr)
    , m_ultrasonicReportCallback(nullptr)
    , m_ultrasonicReportCallbackContext(nullptr)
    , m_telemetryCallback(nullptr)
    , m_telemetryCallbackContext(nullptr)

    // Injected hardware dependencies
    , m_zMotorControllerModule(zMotorController)
    , m_forceCoilControllerModule(forceCoilDriver)
    , m_yAxisRouter(yAxisRouter)
    , m_tAxisRouter(tAxisRouter)
    , m_pllModule(pll)
    , m_impedanceScannerModule(impedanceScanner)
    , m_clampSolenoid(clampSolenoid)
    , m_contactSensorMonitor(contactSensorMonitor)
    , m_timer(timer)

    // Impedance scan data
    , m_voltagePhasors{}
    , m_currentPhasors{}
    , m_impedances{}

    // Z-controller setpoint supplied by the VM
    , m_zMotorPositionSetpoint(0.0f)
    , m_zMotorSetpointActive(false)
{
    s_instance = this;
}

void BonderModule::configure(const Config& config)
{
    m_config = config;

    // Tail and tear are configured as a pair. Center the pair around the
    // router origin so their separation is preserved while the T axis uses
    // equal travel on opposite sides of zero instead of always moving in the
    // positive direction. This transformation is idempotent, so configuring
    // from an already-centered effective configuration is also safe.
    const float tAxisMean =
        0.5f * (m_config.tailPosition + m_config.tearPosition);
    m_config.tailPosition -= tAxisMean;
    m_config.tearPosition -= tAxisMean;

    if (m_config.numOfScannedFrequencies == 0U) {
        m_config.numOfScannedFrequencies = 1U;
    }
    if (m_config.numOfScannedFrequencies > BONDER_MODULE_SCAN_MAX_FREQUENCIES) {
        m_config.numOfScannedFrequencies = BONDER_MODULE_SCAN_MAX_FREQUENCIES;
    }

    const float frequencyStep =
        (m_config.scanStopFrequency - m_config.scanStartFrequency) /
        static_cast<float>(m_config.numOfScannedFrequencies);

    m_impedanceScannerModule->setScanParameters(
        m_config.numOfScannedFrequencies,
        m_config.scanStartFrequency,
        frequencyStep);
}

const BonderModule::Config& BonderModule::getConfig() const
{
    return m_config;
}

void BonderModule::addEventListenerCallbacks(BonderStateChangedCallback stateCb,
                                             BonderErrorCallback errorCb)
{
    m_stateChangedCallback = stateCb;
    m_errorCallback = errorCb;
}

void BonderModule::setUltrasonicReportListenerCallback(
    void *context, UltrasonicReportCallback callback)
{
    m_ultrasonicReportCallbackContext = context;
    m_ultrasonicReportCallback = callback;
}

void BonderModule::setTelemetryListenerCallback(void *context,
                                                TelemetryCallback callback)
{
    m_telemetryCallbackContext = context;
    m_telemetryCallback = callback;
}

BonderModule::VmStatus BonderModule::getVmStatus() const
{
    VmStatus status{};
    status.running = (m_activityState == ActivityState::Running);
    // During the reset prologue the VM has not entered the protocol yet;
    // report pc 0 with the prologue instruction it is actually executing.
    status.pc = m_inPrologue ? 0U : m_pc;
    status.eventFlags = collectEventFlags();

    if (status.running) {
        const Instruction *instr = nullptr;
        if (m_inPrologue) {
            instr = &kResetPrologue[m_pc];
        } else if (m_program != nullptr && m_pc < m_programLength) {
            instr = &m_program[m_pc];
        }
        if (instr != nullptr) {
            status.opcode = static_cast<uint8_t>(instr->opcode);
            status.mask = instr->mask;
        }
    }

    return status;
}

void BonderModule::notifyRightButton(bool pressed)
{
    m_raiseButtonHeld.store(pressed);
    setEventFlags(pressed ? EVENT_RIGHT_BUTTON_PRESSED
                          : EVENT_RIGHT_BUTTON_RELEASED);
}

void BonderModule::notifyLeftButton(bool pressed)
{
    m_lowerButtonHeld.store(pressed);
    setEventFlags(pressed ? EVENT_LEFT_BUTTON_PRESSED
                          : EVENT_LEFT_BUTTON_RELEASED);
}

void BonderModule::onStart()
{
    if (m_zMotorControllerModule == nullptr ||
        m_forceCoilControllerModule == nullptr || m_yAxisRouter == nullptr ||
        m_tAxisRouter == nullptr || m_pllModule == nullptr ||
        m_impedanceScannerModule == nullptr || m_clampSolenoid == nullptr ||
        m_contactSensorMonitor == nullptr || m_timer == nullptr) {
        setProcessError();
        return;
    }

    m_contactSensorMonitor->addStateListenerCallback(
        this, &BonderModule::onContactSensorStateChanged);
    m_timer->setExpirationListenerCallback(this, &BonderModule::onTimerDone);
    m_yAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onYAxisRouterDone);
    m_tAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onTAxisRouterDone);
    const bool callbacksRegistered =
        m_pllModule->addEventListenerCallback(
            this, &BonderModule::onPllEvent) &&
        m_impedanceScannerModule->addScanCompleteListenerCallback(
            this, &BonderModule::onImpedanceScanned) &&
        m_zMotorControllerModule->addPositionSetpointControllerCallback(
            this, &BonderModule::onZMotorPositionSetpoint);
    m_zMotorControllerModule->addEventListenerCallback(
        this, &BonderModule::onZMotorEvent);
    m_clampSolenoid->addStateListenerCallback(
        this, &BonderModule::onClampStateChanged);
    m_forceCoilControllerModule->addEventListenerCallback(&BonderModule::onForceCoilEvent);
    if (!callbacksRegistered) setProcessError();
}

bool BonderModule::setProtocol(const BonderProtocol& protocol)
{
    const bool engaged = (m_activityState == ActivityState::Running);
    if (engaged && !isAwaitingStartTrigger()) {
        return false;
    }

    m_program = protocol.getProtocolPtr();
    m_programLength = protocol.getProtocolSize();
    m_protocolRequiresMotionControl = protocol.requiresMotionControl();

    // Locate the first instruction that blocks on operator input; parked
    // there, the machine counts as armed-but-not-engaged (see
    // isAwaitingStartTrigger()).
    m_armGateValid = false;
    m_armGatePc = 0U;
    for (uint8_t i = 0U; i < m_programLength; ++i) {
        const Instruction& instr = m_program[i];
        const bool operatorWait =
            (instr.opcode == Opcode::WAIT) &&
            ((instr.mask & (EVENT_RIGHT_BUTTON_PRESSED |
                            EVENT_LEFT_BUTTON_PRESSED)) != 0U);
        if (operatorWait || instr.opcode == Opcode::MZMOVE ||
            instr.opcode == Opcode::MZDOWN) {
            m_armGatePc = i;
            m_armGateValid = true;
            break;
        }
    }

    if (engaged) {
        // Retasked while armed: the machine stays engaged and re-enters the
        // new program through the reset prologue, so its pc 0 also starts
        // from the idle posture with no stale event latches.
        clearAllFlags();
        m_pc = 0U;
        m_instrStarted = false;
        m_zMotorSetpointActive = false;
        m_inPrologue = m_motionControlEnabled;
    }
    return true;
}

bool BonderModule::engage()
{
    if (!isOperating() || m_activityState != ActivityState::Idle ||
        m_program == nullptr || m_programLength == 0U) {
        return false;
    }

    if (m_protocolRequiresMotionControl) {
        if (!m_forceCoilControllerModule->enableControl()) {
            return false;
        }
        if (!m_zMotorControllerModule->enableControl()) {
            m_forceCoilControllerModule->disableControl();
            return false;
        }
    }
    m_motionControlEnabled = m_protocolRequiresMotionControl;

    clearAllFlags();
    m_pc = 0U;
    m_instrStarted = false;
    m_zMotorSetpointActive = false;
    // The reset prologue drives the machine to the idle posture before pc 0;
    // protocols without motion control have no posture to establish.
    m_inPrologue = m_motionControlEnabled;
    m_activityState = ActivityState::Running;

    if (m_stateChangedCallback != nullptr) {
        m_stateChangedCallback(false);
    }
    return true;
}

void BonderModule::disengage()
{
    if (m_activityState != ActivityState::Running) return;

    emergencyStop();
    m_zMotorControllerModule->disableControl();
    m_forceCoilControllerModule->disableControl();
    m_motionControlEnabled = false;
    m_activityState = ActivityState::Idle;
    m_inPrologue = false;
    m_pc = 0U;
    m_instrStarted = false;
    m_zMotorSetpointActive = false;

    if (m_stateChangedCallback != nullptr) {
        m_stateChangedCallback(true);
    }
}

void BonderModule::onStop()
{
    disengage();
    if (m_contactSensorMonitor != nullptr) {
        m_contactSensorMonitor->addStateListenerCallback(nullptr, nullptr);
    }
    if (m_timer != nullptr) {
        m_timer->setExpirationListenerCallback(nullptr, nullptr);
    }
    if (m_yAxisRouter != nullptr) {
        m_yAxisRouter->addMoveCompleteListenerCallback(nullptr, nullptr);
    }
    if (m_tAxisRouter != nullptr) {
        m_tAxisRouter->addMoveCompleteListenerCallback(nullptr, nullptr);
    }
    if (m_zMotorControllerModule != nullptr) {
        m_zMotorControllerModule->addEventListenerCallback(nullptr, nullptr);
    }
    if (m_clampSolenoid != nullptr) {
        m_clampSolenoid->addStateListenerCallback(nullptr, nullptr);
    }
    if (m_forceCoilControllerModule != nullptr) {
        m_forceCoilControllerModule->addEventListenerCallback(nullptr);
    }
}

void BonderModule::onExecute()
{
    if (m_activityState != ActivityState::Running) return;
    executeVM();
}

// =============================================================================
// VM executor
// =============================================================================

void BonderModule::executeVM()
{
    if (m_zMotorSetpointActive &&
        !m_zMotorControllerModule->isControlEnabled()) {
        setEventFlags(EVENT_POSITION_ERROR);
    }

    // Chain instructions until one blocks; a full pass over the prologue and
    // the program is the hard bound so a malformed table cannot spin forever.
    const uint16_t instructionBound =
        static_cast<uint16_t>(m_programLength) + kResetPrologueLength;
    for (uint16_t executed = 0U; executed <= instructionBound; ++executed) {
        const Instruction& instr =
            m_inPrologue ? kResetPrologue[m_pc] : m_program[m_pc];

        const uint32_t errorFlags = collectEventFlags() & kErrorFlagsMask;
        if (errorFlags != 0U) {
            if (!m_inPrologue) {
                fireTelemetry(instr, false);
            }

            Error error = Error::UnableToSetPosition;
            if (errorFlags & EVENT_FORCE_COIL_ERROR) {
                error = Error::UnableToSetForceCoilCurrent;
            } else if (errorFlags & EVENT_US_POWER_ERROR) {
                error = Error::InsufficientBondingPower;
            } else if (errorFlags & EVENT_WAIT_TIMEOUT) {
                error = Error::ProtocolTimeout;
            }

            emergencyStop();
            m_activityState = ActivityState::Idle;
            m_zMotorControllerModule->disableControl();
            m_forceCoilControllerModule->disableControl();
            m_motionControlEnabled = false;
            m_inPrologue = false;
            m_pc = 0U;
            m_instrStarted = false;
            m_zMotorSetpointActive = false;

            if (m_errorCallback != nullptr) {
                m_errorCallback(error);
            }
            if (m_stateChangedCallback != nullptr) {
                m_stateChangedCallback(true);
            }
            return;
        }

        if (executeInstruction(instr) != InstrStatus::Done) {
            return;
        }

        if (!m_inPrologue) {
            fireTelemetry(instr, true);
        }

        // Blocking instructions consume the flags they waited on.
        if (instr.opcode == Opcode::WAIT || instr.opcode == Opcode::MZMOVE) {
            clearFlagsMask(instr.mask);
        }

        m_instrStarted = false;
        m_pc = static_cast<uint8_t>(m_pc + 1U);

        if (m_inPrologue) {
            if (m_pc >= kResetPrologueLength) {
                // Idle posture established: enter the protocol at pc 0 with
                // no stale event latches.
                m_inPrologue = false;
                m_pc = 0U;
                clearAllFlags();
            }
            continue;
        }

        if (m_pc >= m_programLength) {
            m_activityState = ActivityState::Idle;
            m_pc = 0U;
            m_zMotorSetpointActive = false;
            if (m_motionControlEnabled) {
                m_zMotorControllerModule->disableControl();
                m_forceCoilControllerModule->disableControl();
                m_motionControlEnabled = false;
            }

            if (m_stateChangedCallback != nullptr) {
                m_stateChangedCallback(true);
            }
            return;
        }
    }
}

BonderModule::InstrStatus BonderModule::executeInstruction(const Instruction& instr)
{
    switch (instr.opcode) {
    case Opcode::ZMOVE:
        clearFlagsMask(EVENT_Z_POSITION_REACHED);
        setZMotorPosition(resolveArg(instr));
        return InstrStatus::Done;

    case Opcode::MZMOVE:
        return executeMzMove(instr);

    case Opcode::MZDOWN:
        return executeMzDown(instr);

    case Opcode::YMOVE:
        clearFlagsMask(EVENT_Y_MOVE_COMPLETED);
        m_yAxisRouter->moveTo(resolveArg(instr));
        return InstrStatus::Done;

    case Opcode::TMOVE:
        clearFlagsMask(EVENT_T_MOVE_COMPLETED);
        m_tAxisRouter->moveTo(resolveArg(instr));
        return InstrStatus::Done;

    case Opcode::TIMER:
        clearFlagsMask(EVENT_TIMER_EXPIRED);
        m_timer->start(true, resolveArg(instr));
        return InstrStatus::Done;

    case Opcode::WAIT:
        if (flagsSatisfied(instr.mask)) {
            return InstrStatus::Done;
        }
        if (instr.timeoutMs == 0U) {
            return InstrStatus::Running;
        }
        if (!m_instrStarted) {
            m_waitStartTick = HAL_GetTick();
            m_instrStarted = true;
        } else if ((HAL_GetTick() - m_waitStartTick) >= instr.timeoutMs) {
            setEventFlags(EVENT_WAIT_TIMEOUT);
        }
        return InstrStatus::Running;

    case Opcode::CLRFLAGS:
        clearFlagsMask(instr.mask);
        return InstrStatus::Done;

    case Opcode::CLAMPOPEN:
        return executeClampCommand(DirectSolenoidChannel::State::ENERGIZED);

    case Opcode::CLAMPCLOSE:
        return executeClampCommand(DirectSolenoidChannel::State::DEENERGIZED);

    case Opcode::SCAN:
        clearFlagsMask(EVENT_SCAN_COMPLETED | EVENT_US_POWER_ERROR);
        m_scanTargetPower = resolveArg(instr);
        if (!m_impedanceScannerModule->beginScan(
                m_voltagePhasors, m_currentPhasors, m_impedances)) {
            setEventFlags(EVENT_US_POWER_ERROR);
        }
        return InstrStatus::Done;

    case Opcode::PLL:
        clearFlagsMask(EVENT_US_POWER_TRANSFERRED | EVENT_US_POWER_ERROR);
        if (!m_pllModule->beginTransfer(
                m_centerFrequency,
                m_driveAmplitude,
                resolveArg(instr),
                m_config.maxBondingDuration)) {
            setEventFlags(EVENT_US_POWER_ERROR);
        }
        return InstrStatus::Done;

    case Opcode::SETFORCE:
        clearFlagsMask(EVENT_FORCE_COIL_SETTLED);
        m_forceCoilControllerModule->setCurrentSetpoint(resolveArg(instr));
        return InstrStatus::Done;

    case Opcode::USREPORT:
        if (m_ultrasonicReportCallback != nullptr) {
            const UltrasonicReport report{
                m_centerFrequency,
                m_qualityFactor,
                m_pllModule->getAveragePower(),
                m_pllModule->getBondingDuration()
            };
            m_ultrasonicReportCallback(
                m_ultrasonicReportCallbackContext, report);
        }
        return InstrStatus::Done;
    }

    return InstrStatus::Done;
}

BonderModule::InstrStatus BonderModule::executeMzMove(const Instruction& instr)
{
    if (!m_instrStarted) {
        // Take over from the current height so the tool does not jump when
        // the operator gains control.
        clearFlagsMask(EVENT_Z_POSITION_REACHED);
        m_manualZTarget = (BONDER_MODULE_ZAXIS_WORKSPACE_SIZE / 2.0f) -
                          m_zMotorControllerModule->getPosition();
        setZMotorPosition(m_manualZTarget);
        m_manualLevelingLastTick = HAL_GetTick();
        m_instrStarted = true;
    }

    const uint32_t tick = HAL_GetTick();
    const float dt =
        static_cast<float>(tick - m_manualLevelingLastTick) * 1.0e-3f;
    m_manualLevelingLastTick = tick;

    const bool lowerHeld = m_lowerButtonHeld.load();
    const bool raiseHeld = m_raiseButtonHeld.load();

    if (lowerHeld != raiseHeld) {
        const float rate = resolveArg(instr);
        float target = m_manualZTarget + (raiseHeld ? rate : -rate) * dt;

        if (target < m_config.lowestOvertravel) {
            target = m_config.lowestOvertravel;
        }
        if (target > m_config.resetHeight) {
            target = m_config.resetHeight;
        }

        if (target != m_manualZTarget) {
            m_manualZTarget = target;
            setZMotorPosition(m_manualZTarget);
        }
    }

    const float measuredHeight = (BONDER_MODULE_ZAXIS_WORKSPACE_SIZE / 2.0f) -
                                 m_zMotorControllerModule->getPosition();
    const bool atLowestOvertravel =
        fabsf(measuredHeight - m_config.lowestOvertravel) <
        DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR;

    if (atLowestOvertravel && flagsSatisfied(instr.mask)) {
        return InstrStatus::Done;
    }

    return InstrStatus::Running;
}

BonderModule::InstrStatus BonderModule::executeMzDown(
    const Instruction& instr)
{
    if (!m_instrStarted) {
        // Continue from the measured height so entering setup cannot command
        // a discontinuous Z jump.
        clearFlagsMask(EVENT_Z_POSITION_REACHED);
        m_manualZTarget = (BONDER_MODULE_ZAXIS_WORKSPACE_SIZE / 2.0f) -
                          m_zMotorControllerModule->getPosition();
        setZMotorPosition(m_manualZTarget);
        m_manualLevelingLastTick = HAL_GetTick();
        m_instrStarted = true;
    }

    const uint32_t tick = HAL_GetTick();
    const float dt =
        static_cast<float>(tick - m_manualLevelingLastTick) * 1.0e-3f;
    m_manualLevelingLastTick = tick;

    if (!m_lowerButtonHeld.load()) {
        return InstrStatus::Done;
    }

    float target = m_manualZTarget - resolveArg(instr) * dt;
    if (target < m_config.lowestOvertravel) {
        target = m_config.lowestOvertravel;
    }
    if (target != m_manualZTarget) {
        m_manualZTarget = target;
        setZMotorPosition(m_manualZTarget);
    }

    return InstrStatus::Running;
}

BonderModule::InstrStatus BonderModule::executeClampCommand(
    DirectSolenoidChannel::State target)
{
    clearFlagsMask(EVENT_CLAMP_SETTLED);
    m_clampCommandTarget = target;

    if (m_clampSolenoid->getState() == target &&
        !m_clampSolenoid->isTransitioning()) {
        setEventFlags(EVENT_CLAMP_SETTLED);
        return InstrStatus::Done;
    }

    if (target == DirectSolenoidChannel::State::ENERGIZED) {
        m_clampSolenoid->energize();
    } else {
        m_clampSolenoid->deenergize();
    }

    return InstrStatus::Done;
}

// =============================================================================
// VM control
// =============================================================================

void BonderModule::emergencyStop()
{
    m_timer->stop();
    m_impedanceScannerModule->abortScan();
    m_pllModule->abortTransfer();
    m_yAxisRouter->stop();
    m_tAxisRouter->stop();
    m_forceCoilControllerModule->setCurrentSetpoint(0.0f);
    m_clampSolenoid->deenergize();
}

// =============================================================================
// Utilities
// =============================================================================

float BonderModule::resolveArg(const Instruction& instr) const
{
    return instr.arg != nullptr ? m_config.*(instr.arg) : 0.0f;
}

uint32_t BonderModule::collectEventFlags() const
{
    return m_eventFlags.load();
}

bool BonderModule::flagsSatisfied(uint32_t mask) const
{
    return (collectEventFlags() & mask) == mask;
}

void BonderModule::setEventFlags(uint32_t mask)
{
    m_eventFlags.fetch_or(mask);
}

void BonderModule::clearFlagsMask(uint32_t mask)
{
    m_eventFlags.fetch_and(~mask);
}

void BonderModule::clearAllFlags()
{
    m_eventFlags.store(0U);
}

void BonderModule::fireTelemetry(const Instruction& instr, bool succeeded)
{
    if (m_telemetryCallback == nullptr) {
        return;
    }

    Telemetry telemetry{};
    telemetry.pc = m_pc;
    telemetry.opcode = static_cast<uint8_t>(instr.opcode);
    telemetry.succeeded = succeeded;
    telemetry.mask = instr.mask;
    telemetry.eventFlags = collectEventFlags();
    telemetry.argValue = resolveArg(instr);
    telemetry.zPosition = m_zMotorControllerModule->getPosition();
    telemetry.zSetpoint = m_zMotorPositionSetpoint;
    telemetry.yPosition = m_yAxisRouter->getPosition();
    telemetry.tPosition = m_tAxisRouter->getPosition();
    telemetry.clampState = static_cast<uint8_t>(m_clampSolenoid->getState());
    telemetry.transferredEnergy = m_pllModule->getBondingEnergy();

    if (instr.mask & EVENT_SCAN_COMPLETED) {
        telemetry.scanCount = m_config.numOfScannedFrequencies;
        telemetry.centerFrequency = m_centerFrequency;
        telemetry.driveAmplitude = m_driveAmplitude;
        telemetry.impedances = m_impedances;
    }

    m_telemetryCallback(m_telemetryCallbackContext, telemetry);
}

void BonderModule::setZMotorPosition(float position)
{
    m_zMotorPositionSetpoint = (BONDER_MODULE_ZAXIS_WORKSPACE_SIZE / 2.0f) - position;
    m_zMotorSetpointActive = true;
    m_zMotorControllerModule->restartControlLoop();
}

void BonderModule::computeOperatingPoint(float targetPower)
{
    const float frequencyStep =
        (m_config.scanStopFrequency - m_config.scanStartFrequency) /
        static_cast<float>(m_config.numOfScannedFrequencies);

    TransducerAnalyzer::Parameters parameters;
    const bool fitted = TransducerAnalyzer::fit(
        m_impedances,
        m_config.numOfScannedFrequencies,
        m_config.scanStartFrequency,
        frequencyStep,
        parameters);

    m_qualityFactor = fitted ? parameters.qFactor : 0.0f;

    const uint8_t resonanceIndex = findResonanceIndex();
    const float dipFrequency = calculateCenterFrequency(resonanceIndex);

    // Trust the fit only when its resonance also agrees with the raw |Z|
    // minimum: a noise-pulled fit can center the PLL outside its tracking
    // clamp even though the regression converged.
    if (fitted &&
        parameters.seriesResonance >= m_config.scanStartFrequency &&
        parameters.seriesResonance <= m_config.scanStopFrequency &&
        fabsf(parameters.seriesResonance - dipFrequency) <=
            BONDER_MODULE_FIT_DIP_MAX_DEVIATION_BINS * frequencyStep) {
        m_centerFrequency = parameters.seriesResonance;
        const complexf admittance =
            TransducerAnalyzer::admittance(parameters, m_centerFrequency);
        m_driveAmplitude = amplitudeForTargetPower(admittance.re, targetPower);
        return;
    }

    m_centerFrequency = dipFrequency;
    m_driveAmplitude = calculateDriveAmplitude(resonanceIndex, targetPower);
}

float BonderModule::amplitudeForTargetPower(float realAdmittance, float targetPower)
{
    if (realAdmittance <= 0.0f || targetPower <= 0.0f) {
        return 0.0f;
    }

    float amplitude = sqrtf(targetPower / realAdmittance);

    return amplitude;
}

uint8_t BonderModule::findResonanceIndex()
{
    uint8_t bestIndex = 0U;
    float bestMagnitude = complexf_abs(m_impedances[0]);

    for (uint8_t i = 1U; i < m_config.numOfScannedFrequencies; ++i) {
        const float magnitude = complexf_abs(m_impedances[i]);
        if (magnitude < bestMagnitude) {
            bestMagnitude = magnitude;
            bestIndex = i;
        }
    }

    return bestIndex;
}

float BonderModule::calculateCenterFrequency(uint8_t resonanceIndex)
{
    const float frequencyStep =
        (m_config.scanStopFrequency - m_config.scanStartFrequency) /
        static_cast<float>(m_config.numOfScannedFrequencies);
    return m_config.scanStartFrequency +
           static_cast<float>(resonanceIndex) * frequencyStep;
}

float BonderModule::calculateDriveAmplitude(uint8_t resonanceIndex, float targetPower)
{
    const complexf impedance = m_impedances[resonanceIndex];
    const float magnitudeSquared = complexf_abs2(impedance);
    if (magnitudeSquared <= 0.0f) {
        return 0.0f;
    }

    return amplitudeForTargetPower(impedance.re / magnitudeSquared, targetPower);
}

// =============================================================================
// Callbacks
// =============================================================================

void BonderModule::onContactSensorStateChanged(
    void *context, PinMonitorChannel::PinState state)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    self->setEventFlags(state == PinMonitorChannel::PinState::ACTIVE
                            ? EVENT_CONTACT_CONNECTED
                            : EVENT_CONTACT_DISCONNECTED);
}

void BonderModule::onTimerDone(void *context, Timer *timer)
{
    (void)timer;
    static_cast<BonderModule *>(context)->setEventFlags(EVENT_TIMER_EXPIRED);
}

void BonderModule::onYAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule *>(context)->setEventFlags(EVENT_Y_MOVE_COMPLETED);
}

void BonderModule::onTAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule *>(context)->setEventFlags(EVENT_T_MOVE_COMPLETED);
}

void BonderModule::onPllEvent(void *context, PllModule::Event event)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (event == PllModule::Event::BondingCompleted) {
        self->setEventFlags(EVENT_US_POWER_TRANSFERRED);
    } else if (event == PllModule::Event::InsufficientBondingPower) {
        self->setEventFlags(EVENT_US_POWER_ERROR);
    }
}

void BonderModule::onForceCoilEvent(ForceCoilDriverModule::Event event)
{
    if (s_instance == nullptr) {
        return;
    }

    if (event == ForceCoilDriverModule::Event::SetpointAchieved) {
        s_instance->setEventFlags(EVENT_FORCE_COIL_SETTLED);
    } else if (event == ForceCoilDriverModule::Event::UnableToSetCurrent) {
        s_instance->setEventFlags(EVENT_FORCE_COIL_ERROR);
    }
}

bool BonderModule::onZMotorPositionSetpoint(void *context, float *positionSetpoint)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (positionSetpoint == nullptr ||
        self->m_activityState != ActivityState::Running ||
        !self->m_zMotorSetpointActive) {
        return false;
    }

    *positionSetpoint = self->m_zMotorPositionSetpoint;
    return true;
}

void BonderModule::onZMotorEvent(void *context,
                                 DcMotorPositionControllerModule::Event event)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (event == DcMotorPositionControllerModule::Event::SetpointReached) {
        self->setEventFlags(EVENT_Z_POSITION_REACHED);
    }
}

void BonderModule::onClampStateChanged(void *context,
                                       DirectSolenoidChannel::State state)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (state == self->m_clampCommandTarget) {
        self->setEventFlags(EVENT_CLAMP_SETTLED);
    }
}

void BonderModule::onImpedanceScanned(
    void *context, complexf *voltage, complexf *current, complexf *impedances)
{
    (void)voltage;
    (void)current;
    (void)impedances;

    BonderModule *self = static_cast<BonderModule *>(context);
    self->computeOperatingPoint(self->m_scanTargetPower);
    self->setEventFlags(EVENT_SCAN_COMPLETED);
}
