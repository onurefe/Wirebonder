#include "bonder_module.hpp"
#include "transducer_analyzer.hpp"
#include <cmath>

// =============================================================================
// Static VM definition
// =============================================================================

BonderModule *BonderModule::s_instance = nullptr;

const BonderModule::Step BonderModule::kBondSequence[] = {
    {&BonderModule::stepWaitForSemiAutoButton, BondingPhase::Phase1},
    {&BonderModule::stepMovingToSearchHeight, BondingPhase::Phase1},
    {&BonderModule::stepMovingToLowestOvertravel, BondingPhase::Phase1},
    {&BonderModule::stepWaitForZMotorPositionSettlement, BondingPhase::Phase1},
    {&BonderModule::stepWaitForBondingForceSettlement, BondingPhase::Phase1},
    {&BonderModule::stepScanImpedance, BondingPhase::Phase1},
    {&BonderModule::stepBond, BondingPhase::Phase1},
    {&BonderModule::stepCool, BondingPhase::Phase1},
    {&BonderModule::stepMoveToKinkHeight, BondingPhase::Phase1},
    {&BonderModule::stepYReverse, BondingPhase::Phase1},
    {&BonderModule::stepMoveToLoopHeight, BondingPhase::Phase1},
    {&BonderModule::stepWaitForSemiAutoButton, BondingPhase::Phase2},
    {&BonderModule::stepMovingToSearchHeight, BondingPhase::Phase2},
    {&BonderModule::stepMovingToLowestOvertravel, BondingPhase::Phase2},
    {&BonderModule::stepWaitForZMotorPositionSettlement, BondingPhase::Phase2},
    {&BonderModule::stepWaitForBondingForceSettlement, BondingPhase::Phase2},
    {&BonderModule::stepScanImpedance, BondingPhase::Phase2},
    {&BonderModule::stepBond, BondingPhase::Phase2},
    {&BonderModule::stepCool, BondingPhase::Phase2},
    {&BonderModule::stepTearTMove, BondingPhase::Phase2},
    {&BonderModule::stepMoveToResetHeight, BondingPhase::Phase2},
    {&BonderModule::stepRestoreYMove, BondingPhase::Phase2},
};

const uint8_t BonderModule::kSequenceLen =
    static_cast<uint8_t>(sizeof(kBondSequence) / sizeof(kBondSequence[0]));

// =============================================================================
// Public interface
// =============================================================================

BonderModule::BonderModule(DcMotorPositionControllerModule *zMotorController,
                           ForceCoilDriverModule *forceCoilDriver,
                           RouterChannel *yAxisRouter,
                           RouterChannel *tAxisRouter,
                           PllModule *pll,
                           UsImpedanceScannerModule *impedanceScanner,
                           SolenoidChannel *clampSolenoid,
                           PinMonitorChannel *contactSensorMonitor,
                           PinMonitorChannel *mouseRightButtonMonitor,
                           Timer *timer)
    : m_config{}
    , m_centerFrequency(0.0f)
    , m_driveAmplitude(0.0f)

    // VM runtime state
    , m_systemState(State::Uninit)
    , m_stepIndex(0U)
    , m_stepStarted(false)

    // Hardware event and error flags
    , m_tMoveCompleted(false)
    , m_yMoveCompleted(false)
    , m_forceCoilCurrentSettled(false)
    , m_contactPinConnected(false)
    , m_contactPinDisconnected(false)
    , m_timerExpired(false)
    , m_impedanceScanningCompleted(false)
    , m_usPowerTransferred(false)
    , m_clampOpened(false)
    , m_clampClosed(false)
    , m_rightButtonPressed(false)
    , m_rightButtonReleased(false)
    , m_positionError(false)
    , m_forceCoilError(false)
    , m_usPowerError(false)
    , m_tailMoveStarted(false)
    , m_resetRestoreStarted(false)

    // External notifications
    , m_stateChangedCallback(nullptr)
    , m_errorCallback(nullptr)

    // Injected hardware dependencies
    , m_zMotorControllerModule(zMotorController)
    , m_forceCoilControllerModule(forceCoilDriver)
    , m_yAxisRouter(yAxisRouter)
    , m_tAxisRouter(tAxisRouter)
    , m_pllModule(pll)
    , m_impedanceScannerModule(impedanceScanner)
    , m_clampSolenoid(clampSolenoid)
    , m_contactSensorMonitor(contactSensorMonitor)
    , m_mouseRightButtonMonitor(mouseRightButtonMonitor)
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

void BonderModule::init()
{
    if (m_systemState != State::Uninit) {
        return;
    }

    m_clampSolenoid->addStateListenerCallback(this, &BonderModule::onSolenoidChanged);
    m_contactSensorMonitor->addTransitionListenerCallback(
        this, &BonderModule::onContactSensorTransition);
    m_mouseRightButtonMonitor->addTransitionListenerCallback(
        this, &BonderModule::onMouseRightButtonTransition);
    m_timer->setExpirationListenerCallback(this, &BonderModule::onTimerDone);
    m_yAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onYAxisRouterDone);
    m_tAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onTAxisRouterDone);
    m_pllModule->addEventListenerCallback(this, &BonderModule::onPllEvent);
    m_impedanceScannerModule->addScanCompleteListenerCallback(
        this, &BonderModule::onImpedanceScanned);
    m_zMotorControllerModule->addPositionSetpointControllerCallback(
        this, &BonderModule::onZMotorPositionSetpoint);
    m_forceCoilControllerModule->addEventListenerCallback(&BonderModule::onForceCoilEvent);

    m_systemState = State::Ready;
}

void BonderModule::start()
{
    if (m_systemState != State::Ready) {
        return;
    }

    clearFlags();
    m_stepIndex = 0U;
    m_stepStarted = false;
    m_zMotorSetpointActive = false;
    m_systemState = State::Operating;

    m_forceCoilControllerModule->setCurrentSetpoint(0.0f);
    m_clampSolenoid->close();
    setZMotorPosition(m_config.resetHeight);

    if (m_stateChangedCallback != nullptr) {
        m_stateChangedCallback(false);
    }
}

void BonderModule::stop()
{
    if (m_systemState != State::Operating) {
        return;
    }

    emergencyStop();
    m_systemState = State::Ready;
    m_stepIndex = 0U;
    m_stepStarted = false;
    m_zMotorSetpointActive = false;

    if (m_stateChangedCallback != nullptr) {
        m_stateChangedCallback(true);
    }
}

void BonderModule::execute()
{
    if (m_systemState != State::Operating) {
        return;
    }

    m_impedanceScannerModule->execute();

    const Step& step = kBondSequence[m_stepIndex];
    const StepStatus status = (this->*step.fn)(step.phase);

    if (status == StepStatus::Error || m_positionError ||
        m_forceCoilError || m_usPowerError) {
        Error error = Error::UnableToSetPosition;
        if (m_forceCoilError) {
            error = Error::UnableToSetForceCoilCurrent;
        } else if (m_usPowerError) {
            error = Error::InsufficientBondingPower;
        }

        emergencyStop();
        m_systemState = State::Ready;
        m_stepIndex = 0U;
        m_stepStarted = false;
        m_zMotorSetpointActive = false;

        if (m_errorCallback != nullptr) {
            m_errorCallback(error);
        }
        if (m_stateChangedCallback != nullptr) {
            m_stateChangedCallback(true);
        }
        return;
    }

    if (status != StepStatus::Done) {
        return;
    }

    clearFlags();
    m_stepStarted = false;
    m_stepIndex = static_cast<uint8_t>(m_stepIndex + 1U);

    if (m_stepIndex < kSequenceLen) {
        return;
    }

    m_stepIndex = 0U;
    m_zMotorSetpointActive = false;

    if (m_stateChangedCallback != nullptr) {
        m_stateChangedCallback(true);
    }
}

// =============================================================================
// VM control
// =============================================================================

void BonderModule::emergencyStop()
{
    m_timer->stop();
    m_impedanceScannerModule->stop();
    m_pllModule->stop();
    m_yAxisRouter->stop();
    m_tAxisRouter->stop();
    m_forceCoilControllerModule->setCurrentSetpoint(0.0f);
    m_clampSolenoid->close();
}

// =============================================================================
// VM steps
// =============================================================================

BonderModule::StepStatus BonderModule::stepWaitForSemiAutoButton(BondingPhase phase)
{
    (void)phase;
    return m_rightButtonPressed ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMovingToSearchHeight(BondingPhase phase)
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilTrackingCurrent);
        setZMotorPosition(phase == BondingPhase::Phase1
                              ? m_config.firstSearchHeight
                              : m_config.secondSearchHeight);

        if (phase == BondingPhase::Phase2) {
            m_yAxisRouter->moveTo(m_config.yStepbackPosition);
            m_clampSolenoid->close();
        }

        m_stepStarted = true;
    }

    if (m_forceCoilError) {
        return StepStatus::Error;
    }

    if (phase == BondingPhase::Phase2 && m_yMoveCompleted) {
        m_clampSolenoid->open();
    }

    const bool yPositionReached =
        phase == BondingPhase::Phase1 || m_yMoveCompleted;
    if (yPositionReached && zMotorPositionReached() &&
        m_rightButtonReleased && m_forceCoilCurrentSettled) {
        return StepStatus::Done;
    }

    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMovingToLowestOvertravel(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilConstantCurrent);
        setZMotorPosition(m_config.lowestOvertravel);
        m_stepStarted = true;
    }

    if (m_forceCoilError) {
        return StepStatus::Error;
    }

    if (zMotorPositionReached() && m_contactPinDisconnected &&
        m_forceCoilCurrentSettled) {
        return StepStatus::Done;
    }

    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepWaitForZMotorPositionSettlement(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_timer->start(true, m_config.contactSettlingTime);
        m_stepStarted = true;
    }

    return m_timerExpired ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepWaitForBondingForceSettlement(BondingPhase phase)
{
    if (!m_stepStarted) {
        const float bondCurrent = phase == BondingPhase::Phase1
                                      ? m_config.forceCoilFirstBondCurrent
                                      : m_config.forceCoilSecondBondCurrent;
        m_forceCoilControllerModule->setCurrentSetpoint(bondCurrent);
        m_timer->start(true, m_config.contactSettlingTime);
        m_stepStarted = true;
    }

    if (m_forceCoilError) {
        return StepStatus::Error;
    }

    return m_timerExpired && m_forceCoilCurrentSettled
               ? StepStatus::Done
               : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepScanImpedance(BondingPhase phase)
{
    if (!m_stepStarted) {
        m_impedanceScannerModule->start(
            m_voltagePhasors, m_currentPhasors, m_impedances);
        m_stepStarted = true;
    }

    if (!m_impedanceScanningCompleted) {
        return StepStatus::Running;
    }

    const float targetPower = phase == BondingPhase::Phase1
                                  ? m_config.firstBondingPower
                                  : m_config.secondBondingPower;
    computeOperatingPoint(targetPower);
    return StepStatus::Done;
}

BonderModule::StepStatus BonderModule::stepBond(BondingPhase phase)
{
    if (!m_stepStarted) {
        const float bondingEnergy = phase == BondingPhase::Phase1
                                        ? m_config.firstBondingEnergy
                                        : m_config.secondBondingEnergy;
        m_pllModule->start(
            m_centerFrequency,
            m_driveAmplitude,
            bondingEnergy,
            m_config.maxBondingDuration);
        m_stepStarted = true;
    }

    if (m_usPowerError) {
        return StepStatus::Error;
    }

    return m_usPowerTransferred ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepCool(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilConstantCurrent);
        m_timer->start(true, m_config.coolingTime);
        m_stepStarted = true;
    }

    if (m_forceCoilError) {
        return StepStatus::Error;
    }

    return m_timerExpired && m_forceCoilCurrentSettled
               ? StepStatus::Done
               : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMoveToKinkHeight(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_clampSolenoid->open();
        setZMotorPosition(m_config.kinkHeight);
        m_tailMoveStarted = false;
        m_stepStarted = true;
    }

    if (m_contactPinConnected && !m_tailMoveStarted) {
        m_tAxisRouter->moveTo(m_config.tailPosition);
        m_tailMoveStarted = true;
    }

    if (zMotorPositionReached() && m_tMoveCompleted &&
        m_contactPinConnected) {
        return StepStatus::Done;
    }

    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepYReverse(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_yAxisRouter->moveTo(m_config.yReversePosition);
        m_stepStarted = true;
    }

    return m_yMoveCompleted ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMoveToLoopHeight(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        setZMotorPosition(m_config.loopHeight);
        m_stepStarted = true;
    }

    return zMotorPositionReached() ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepTearTMove(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_tAxisRouter->moveTo(m_config.tearPosition);
        m_clampSolenoid->close();
        m_stepStarted = true;
    }

    return m_tMoveCompleted ? StepStatus::Done : StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMoveToResetHeight(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        setZMotorPosition(m_config.resetHeight);
        m_timer->start(true, m_config.tailRestoreDelay);
        m_resetRestoreStarted = false;
        m_stepStarted = true;
    }

    if (m_timerExpired && !m_resetRestoreStarted) {
        m_tAxisRouter->moveTo(0.0f);
        m_pllModule->start(
            m_centerFrequency,
            m_driveAmplitude,
            m_config.secondBondingEnergy,
            m_config.maxBondingDuration);
        m_resetRestoreStarted = true;
    }

    if (m_usPowerError) {
        return StepStatus::Error;
    }

    if (m_resetRestoreStarted && zMotorPositionReached() &&
        m_tMoveCompleted && m_usPowerTransferred &&
        m_contactPinConnected) {
        return StepStatus::Done;
    }

    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepRestoreYMove(BondingPhase phase)
{
    (void)phase;

    if (!m_stepStarted) {
        m_yAxisRouter->moveTo(0.0f);
        m_stepStarted = true;
    }

    return m_yMoveCompleted ? StepStatus::Done : StepStatus::Running;
}

// =============================================================================
// Utilities
// =============================================================================

void BonderModule::clearFlags()
{
    m_tMoveCompleted = false;
    m_yMoveCompleted = false;
    m_forceCoilCurrentSettled = false;
    m_contactPinConnected = false;
    m_contactPinDisconnected = false;
    m_timerExpired = false;
    m_impedanceScanningCompleted = false;
    m_usPowerTransferred = false;
    m_clampOpened = false;
    m_clampClosed = false;
    m_rightButtonPressed = false;
    m_rightButtonReleased = false;
    m_positionError = false;
    m_forceCoilError = false;
    m_usPowerError = false;
    m_tailMoveStarted = false;
    m_resetRestoreStarted = false;
}

void BonderModule::setZMotorPosition(float position)
{
    m_zMotorPositionSetpoint = position;
    m_zMotorSetpointActive = true;
    m_zMotorControllerModule->restartControlLoop();
}

bool BonderModule::zMotorPositionReached()
{
    if (!m_zMotorSetpointActive) {
        return false;
    }

    if (!m_zMotorControllerModule->isOperating()) {
        m_positionError = true;
        return false;
    }

    const float positionError = fabsf(
        m_zMotorPositionSetpoint - m_zMotorControllerModule->getPosition());
    const float velocity = fabsf(m_zMotorControllerModule->getVelocity());

    return positionError < DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR &&
           velocity < DCMOTOR_POSITION_MODULE_MAX_VELOCITY_ERROR;
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

    if (fitted &&
        parameters.seriesResonance >= m_config.scanStartFrequency &&
        parameters.seriesResonance <= m_config.scanStopFrequency) {
        m_centerFrequency = parameters.seriesResonance;
        const complexf admittance =
            TransducerAnalyzer::admittance(parameters, m_centerFrequency);
        m_driveAmplitude = amplitudeForTargetPower(admittance.re, targetPower);
        return;
    }

    const uint8_t resonanceIndex = findResonanceIndex();
    m_centerFrequency = calculateCenterFrequency(resonanceIndex);
    m_driveAmplitude = calculateDriveAmplitude(resonanceIndex, targetPower);
}

float BonderModule::amplitudeForTargetPower(float realAdmittance, float targetPower)
{
    if (realAdmittance <= 0.0f || targetPower <= 0.0f) {
        return 0.0f;
    }

    float amplitude = sqrtf(targetPower / realAdmittance);
    if (amplitude > BONDER_MODULE_MAX_DRIVE_AMPLITUDE) {
        amplitude = BONDER_MODULE_MAX_DRIVE_AMPLITUDE;
    }
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

void BonderModule::onSolenoidChanged(void *context, SolenoidChannel::State state)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (state == SolenoidChannel::State::OPENED) {
        self->m_clampOpened = true;
    } else if (state == SolenoidChannel::State::CLOSED) {
        self->m_clampClosed = true;
    }
}

void BonderModule::onContactSensorTransition(
    void *context, PinMonitorChannel::Transition transition)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (transition == PinMonitorChannel::Transition::LOW_TO_HIGH) {
        self->m_contactPinDisconnected = true;
    } else if (transition == PinMonitorChannel::Transition::HIGH_TO_LOW) {
        self->m_contactPinConnected = true;
    }
}

void BonderModule::onMouseRightButtonTransition(
    void *context, PinMonitorChannel::Transition transition)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (transition == PinMonitorChannel::Transition::LOW_TO_HIGH) {
        self->m_rightButtonPressed = true;
    } else if (transition == PinMonitorChannel::Transition::HIGH_TO_LOW) {
        self->m_rightButtonReleased = true;
    }
}

void BonderModule::onTimerDone(void *context, Timer *timer)
{
    (void)timer;
    static_cast<BonderModule *>(context)->m_timerExpired = true;
}

void BonderModule::onYAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule *>(context)->m_yMoveCompleted = true;
}

void BonderModule::onTAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule *>(context)->m_tMoveCompleted = true;
}

void BonderModule::onPllEvent(void *context, PllModule::Event event)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (event == PllModule::Event::BondingCompleted) {
        self->m_usPowerTransferred = true;
    } else if (event == PllModule::Event::InsufficientBondingPower) {
        self->m_usPowerError = true;
    }
}

void BonderModule::onForceCoilEvent(ForceCoilDriverModule::Event event)
{
    if (s_instance == nullptr) {
        return;
    }

    if (event == ForceCoilDriverModule::Event::SetpointAchieved) {
        s_instance->m_forceCoilCurrentSettled = true;
    } else if (event == ForceCoilDriverModule::Event::UnableToSetCurrent) {
        s_instance->m_forceCoilError = true;
    }
}

bool BonderModule::onZMotorPositionSetpoint(void *context, float *positionSetpoint)
{
    BonderModule *self = static_cast<BonderModule *>(context);
    if (positionSetpoint == nullptr ||
        self->m_systemState != State::Operating ||
        !self->m_zMotorSetpointActive) {
        return false;
    }

    *positionSetpoint = self->m_zMotorPositionSetpoint;
    return true;
}

void BonderModule::onImpedanceScanned(
    void *context, complexf *voltage, complexf *current, complexf *impedances)
{
    (void)voltage;
    (void)current;
    (void)impedances;
    static_cast<BonderModule *>(context)->m_impedanceScanningCompleted = true;
}
