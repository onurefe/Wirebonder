#include "bonder_module.hpp"
#include "transducer_analyzer.hpp"
#include "main.h"
#include <cmath>

BonderModule *BonderModule::s_instance = nullptr;

// =============================================================================
// Step sequence table
// =============================================================================

const BonderModule::StepFn BonderModule::kBondSequence[] = {
    // 1. Idle — wait for operator trigger
    &BonderModule::stepWaitForTrigger,

    // 2. First bond
    &BonderModule::stepMoveToSearchHeight,
    &BonderModule::stepSearch,
    &BonderModule::stepSettle,
    &BonderModule::stepScanImpedance,
    &BonderModule::stepWeld,
    &BonderModule::stepFirstBondCool,

    // 3. Wire loop formation
    &BonderModule::stepFormLoopTAndZ,
    &BonderModule::stepFormLoopYReverse,
    &BonderModule::stepFormLoopZ,

    // 4. Second bond preparation
    &BonderModule::stepWaitSecondTrigger,
    &BonderModule::stepPrepCloseClamp,
    &BonderModule::stepPrepStepBack,
    &BonderModule::stepPrepOpenClamp,

    // 5. Second bond (search/settle/scan/weld steps reused from above)
    &BonderModule::stepMoveToSearchHeight,
    &BonderModule::stepSearch,
    &BonderModule::stepSettle,
    &BonderModule::stepScanImpedance,
    &BonderModule::stepWeld,
    &BonderModule::stepSecondBondCool,

    // 6. Tail restore
    &BonderModule::stepTearTMove,
    &BonderModule::stepRestoreZWaitContact,
    &BonderModule::stepRestoreScanImpedance,
    &BonderModule::stepRestoreTMoveWithUs,
    &BonderModule::stepRestoreYMove,
};

const uint8_t BonderModule::kSequenceLen =
    static_cast<uint8_t>(sizeof(BonderModule::kBondSequence) /
                         sizeof(BonderModule::kBondSequence[0]));

// =============================================================================
// Constructor
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
    , m_systemState(State::Uninit)
    , m_stepIndex(0U)
    , m_stepStarted(false)
    , m_stateChangedCallback(nullptr)
    , m_errorCallback(nullptr)
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
    , m_zMotorPositionSetpoint(0.0f)
    , m_zMotorSetpointActive(false)
{
    s_instance = this;
    clearFlags();
}

// =============================================================================
// Configuration
// =============================================================================

void BonderModule::configure(const Config& config)
{
    m_config = config;

    float frequencyStep = (config.scanStopFrequency - config.scanStartFrequency) /
                          static_cast<float>(config.numOfScannedFrequencies);

    m_impedanceScannerModule->setScanParameters(
        config.numOfScannedFrequencies,
        config.scanStartFrequency,
        frequencyStep);
}

const BonderModule::Config& BonderModule::getConfig() const
{
    return m_config;
}

// =============================================================================
// Lifecycle
// =============================================================================

void BonderModule::addEventListenerCallbacks(BonderStateChangedCallback stateCb,
                                             BonderErrorCallback errorCb)
{
    m_stateChangedCallback = stateCb;
    m_errorCallback        = errorCb;
}

void BonderModule::init()
{
    if (m_systemState != State::Uninit) return;

    m_clampSolenoid->addStateListenerCallback(this, &BonderModule::onSolenoidChanged);
    m_contactSensorMonitor->addTransitionListenerCallback(this, &BonderModule::onContactSensorTransition);
    m_mouseRightButtonMonitor->addTransitionListenerCallback(this, &BonderModule::onMouseRightButtonTransition);
    m_timer->setExpirationListenerCallback(this, &BonderModule::onTimerDone);
    m_yAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onYAxisRouterDone);
    m_tAxisRouter->addMoveCompleteListenerCallback(this, &BonderModule::onTAxisRouterDone);
    m_pllModule->addEventListenerCallback(this, &BonderModule::onPllEvent);
    m_impedanceScannerModule->addScanCompleteListenerCallback(this, &BonderModule::onImpedanceScanned);
    m_zMotorControllerModule->addPositionSetpointControllerCallback(
        this,
        &BonderModule::onZMotorPositionSetpoint);
    m_forceCoilControllerModule->addEventListenerCallback(&BonderModule::onForceCoilEvent);

    m_systemState = State::Ready;
}

void BonderModule::start()
{
    if (m_systemState != State::Ready) return;
    clearFlags();
    m_zMotorSetpointActive = false;
    m_stepIndex            = 0U;
    m_stepStarted          = false;
    m_systemState          = State::Operating;
}

void BonderModule::stop()
{
    if (m_systemState != State::Operating) return;
    emergencyStop();
    m_systemState = State::Ready;
}

void BonderModule::execute()
{
    if (m_systemState != State::Operating) return;

    m_impedanceScannerModule->execute();

    StepStatus s = (this->*kBondSequence[m_stepIndex])();

    if (s == StepStatus::Done) {
        m_stepStarted = false;
        clearFlags();
        m_stepIndex = static_cast<uint8_t>(m_stepIndex + 1U);

        if (m_stepIndex >= kSequenceLen) {
            m_stepIndex = 0U;
            if (m_stateChangedCallback) m_stateChangedCallback(true);
        }
    } else if (s == StepStatus::Error) {
        emergencyStop();
        m_stepIndex   = 0U;
        m_stepStarted = false;
        if (m_stateChangedCallback) m_stateChangedCallback(true);
    }
}

void BonderModule::emergencyStop()
{
    setZMotorPosition(m_config.resetHeight);
    m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilIdleCurrent);
    m_clampSolenoid->close();
    m_impedanceScannerModule->stop();
    m_pllModule->stop();
    m_timer->stop();
}

// =============================================================================
// Helpers
// =============================================================================

void BonderModule::clearFlags()
{
    m_tMoveCompleted             = false;
    m_yMoveCompleted             = false;
    m_forceCoilCurrentSettled    = false;
    m_contactPinConnected        = false;
    m_contactPinDisconnected     = false;
    m_timerExpired               = false;
    m_impedanceScanningCompleted = false;
    m_usPowerTransferred         = false;
    m_clampOpened                = false;
    m_clampClosed                = false;
    m_rightButtonPressed         = false;
    m_rightButtonReleased        = false;
    m_positionError              = false;
    m_forceCoilError             = false;
    m_usPowerError               = false;
}

void BonderModule::setZMotorPosition(float position)
{
    m_zMotorPositionSetpoint = position;
    m_zMotorSetpointActive = true;
    m_zMotorControllerModule->restartControlLoop();
}

bool BonderModule::zMotorPositionReached() const
{
    if (!m_zMotorSetpointActive || !m_zMotorControllerModule->isOperating()) {
        return false;
    }

    const float positionError = fabsf(m_zMotorPositionSetpoint -
                                      m_zMotorControllerModule->getPosition());
    const float velocity = fabsf(m_zMotorControllerModule->getVelocity());

    return (positionError < DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR) &&
           (velocity < DCMOTOR_POSITION_MODULE_MAX_VELOCITY_ERROR);
}

void BonderModule::computeOperatingPoint()
{
    float fstep = (m_config.scanStopFrequency - m_config.scanStartFrequency) /
                  static_cast<float>(m_config.numOfScannedFrequencies);

    TransducerAnalyzer::Parameters params;
    bool fitted = TransducerAnalyzer::fit(m_impedances,
                                    m_config.numOfScannedFrequencies,
                                    m_config.scanStartFrequency,
                                    fstep,
                                    params);

    // Reject fits whose resonance falls outside the scanned window; the
    // extrapolation is not trustworthy there.
    if (fitted &&
        params.seriesResonance >= m_config.scanStartFrequency &&
        params.seriesResonance <= m_config.scanStopFrequency) {
        m_centerFrequency = params.seriesResonance;

        complexf y = TransducerAnalyzer::admittance(params, m_centerFrequency);
        m_driveAmplitude = amplitudeForTargetPower(y.re);
        return;
    }

    // Fallback: nearest grid point to the impedance minimum.
    uint8_t resonance_index = findResonanceIndex();
    m_centerFrequency = calculateCenterFrequency(resonance_index);
    m_driveAmplitude = calculateDriveAmplitude(resonance_index);
}

float BonderModule::amplitudeForTargetPower(float realAdmittance) const
{
    if (realAdmittance <= 0.0f) {
        return 0.0f;
    }

    // P = A^2 * Re(Y)  =>  A = sqrt(P / Re(Y))
    float amplitude = sqrtf(m_config.targetPower / realAdmittance);

    if (amplitude > BONDER_MODULE_MAX_DRIVE_AMPLITUDE) {
        amplitude = BONDER_MODULE_MAX_DRIVE_AMPLITUDE;
    }

    return amplitude;
}

uint8_t BonderModule::findResonanceIndex() const
{
    uint8_t bestIndex     = 0U;
    float   bestMagnitude = complexf_abs(m_impedances[0]);

    for (uint8_t i = 1U; i < m_config.numOfScannedFrequencies; ++i) {
        float mag = complexf_abs(m_impedances[i]);
        if (mag < bestMagnitude) {
            bestMagnitude = mag;
            bestIndex     = i;
        }
    }

    return bestIndex;
}

float BonderModule::calculateCenterFrequency(uint8_t resonanceIndex) const
{
    float fstep = (m_config.scanStopFrequency - m_config.scanStartFrequency) /
                  static_cast<float>(m_config.numOfScannedFrequencies);
    return m_config.scanStartFrequency + resonanceIndex * fstep;
}

float BonderModule::calculateDriveAmplitude(uint8_t resonanceIndex) const
{
    const complexf z = m_impedances[resonanceIndex];
    const float z_abs2 = complexf_abs2(z);

    if (z_abs2 <= 0.0f) {
        return 0.0f;
    }

    return amplitudeForTargetPower(z.re / z_abs2);
}

// =============================================================================
// Step functions
// =============================================================================

BonderModule::StepStatus BonderModule::stepWaitForTrigger()
{
    if (m_rightButtonPressed) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepMoveToSearchHeight()
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilIdleCurrent);
        setZMotorPosition(m_config.searchHeight);
        m_stepStarted = true;
    }
    if (m_positionError)  return StepStatus::Error;
    if (m_forceCoilError) return StepStatus::Error;
    if (zMotorPositionReached() && m_forceCoilCurrentSettled && m_rightButtonReleased)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepSearch()
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilSearchingCurrent);
        setZMotorPosition(m_config.lowestOvertravel);
        m_stepStarted = true;
    }
    if (m_positionError)  return StepStatus::Error;
    if (m_forceCoilError) return StepStatus::Error;
    if (m_contactPinDisconnected && m_forceCoilCurrentSettled)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepSettle()
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilSettlingCurrent);
        m_timer->start(true, m_config.settlingTime);
        m_stepStarted = true;
    }
    if (m_forceCoilError) return StepStatus::Error;
    if (m_timerExpired && m_forceCoilCurrentSettled)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepScanImpedance()
{
    if (!m_stepStarted) {
        m_impedanceScannerModule->start(m_voltagePhasors, m_currentPhasors, m_impedances);
        m_stepStarted = true;
    }
    if (m_impedanceScanningCompleted) {
        computeOperatingPoint();
        return StepStatus::Done;
    }
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepWeld()
{
    if (!m_stepStarted) {
        m_pllModule->start(m_centerFrequency,
                     m_driveAmplitude,
                     m_config.bondingEnergy,
                     m_config.maxBondingDuration);
        m_stepStarted = true;
    }
    if (m_usPowerError)       return StepStatus::Error;
    if (m_usPowerTransferred) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepFirstBondCool()
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilIdleCurrent);
        m_clampSolenoid->close();
        m_timer->start(true, m_config.coolingTime);
        m_stepStarted = true;
    }
    if (m_forceCoilError) return StepStatus::Error;
    if (m_timerExpired && m_forceCoilCurrentSettled && m_clampOpened)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepFormLoopTAndZ()
{
    if (!m_stepStarted) {
        setZMotorPosition(m_config.kinkHeight);
        m_tAxisRouter->append(m_config.tailDisplacement);
        m_stepStarted = true;
    }
    if (m_positionError) return StepStatus::Error;
    if (zMotorPositionReached() && m_tMoveCompleted) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepFormLoopYReverse()
{
    if (!m_stepStarted) {
        m_yAxisRouter->append(m_config.yReverseDisplacement);
        m_stepStarted = true;
    }
    if (m_yMoveCompleted) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepFormLoopZ()
{
    if (!m_stepStarted) {
        setZMotorPosition(m_config.loopHeight);
        m_stepStarted = true;
    }
    if (m_positionError)  return StepStatus::Error;
    if (zMotorPositionReached()) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepWaitSecondTrigger()
{
    if (m_rightButtonPressed) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepPrepCloseClamp()
{
    if (!m_stepStarted) {
        m_clampSolenoid->open();    // open solenoid → closes clamp mechanically
        m_stepStarted = true;
    }
    if (m_clampClosed) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepPrepStepBack()
{
    if (!m_stepStarted) {
        m_yAxisRouter->append(m_config.yStepbackDisplacement);
        m_stepStarted = true;
    }
    if (m_yMoveCompleted) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepPrepOpenClamp()
{
    if (!m_stepStarted) {
        m_clampSolenoid->close();   // close solenoid → opens clamp mechanically
        m_stepStarted = true;
    }
    if (m_clampOpened) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepSecondBondCool()
{
    if (!m_stepStarted) {
        m_forceCoilControllerModule->setCurrentSetpoint(m_config.forceCoilIdleCurrent);
        m_clampSolenoid->open();    // open solenoid → closes clamp mechanically
        m_timer->start(true, m_config.coolingTime);
        m_stepStarted = true;
    }
    if (m_forceCoilError) return StepStatus::Error;
    if (m_timerExpired && m_forceCoilCurrentSettled && m_clampClosed)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepTearTMove()
{
    if (!m_stepStarted) {
        m_tAxisRouter->append(m_config.tearDisplacement);
        m_stepStarted = true;
    }
    if (m_tMoveCompleted) return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepRestoreZWaitContact()
{
    if (!m_stepStarted) {
        setZMotorPosition(m_config.resetHeight);
        m_timer->start(true, m_config.tailRestoreDelay);
        m_stepStarted = true;
    }
    if (m_positionError) return StepStatus::Error;
    if (m_contactPinConnected && m_timerExpired)
        return StepStatus::Done;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepRestoreScanImpedance()
{
    if (!m_stepStarted) {
        m_impedanceScannerModule->start(m_voltagePhasors, m_currentPhasors, m_impedances);
        m_stepStarted = true;
    }
    if (m_impedanceScanningCompleted) {
        computeOperatingPoint();
        return StepStatus::Done;
    }
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepRestoreTMoveWithUs()
{
    if (!m_stepStarted) {
        m_pllModule->start(m_centerFrequency,
                     m_driveAmplitude,
                     m_config.bondingEnergy,
                     m_config.maxBondingDuration);
        m_tAxisRouter->append(m_config.tailDisplacement);
        m_timer->start(true, m_config.yRestoreDelay);
        m_stepStarted = true;
    }
    // Once the Y-restore delay fires, kick off the Y move (only once).
    if (m_timerExpired) {
        m_timerExpired = false;
        m_yAxisRouter->append(m_config.yStepbackDisplacement);
    }
    if (m_usPowerError) return StepStatus::Error;
    return StepStatus::Running;
}

BonderModule::StepStatus BonderModule::stepRestoreYMove()
{
    if (m_tMoveCompleted && m_yMoveCompleted && zMotorPositionReached() && m_usPowerTransferred) {
        m_clampSolenoid->close();
        return StepStatus::Done;
    }
    return StepStatus::Running;
}

// =============================================================================
// Static ISR callbacks
// =============================================================================

void BonderModule::onSolenoidChanged(void *context, SolenoidChannel::State state)
{
    auto *self = static_cast<BonderModule*>(context);
    // Solenoid CLOSED (energised) → spring opens clamp
    // Solenoid OPENED (de-energised) → spring closes clamp
    if (state == SolenoidChannel::State::CLOSED) self->m_clampOpened = true;
    if (state == SolenoidChannel::State::OPENED) self->m_clampClosed = true;
}

void BonderModule::onContactSensorTransition(void *context,
                                              PinMonitorChannel::Transition transition)
{
    auto *self = static_cast<BonderModule*>(context);
    if (transition == PinMonitorChannel::Transition::LOW_TO_HIGH)
        self->m_contactPinDisconnected = true;
    if (transition == PinMonitorChannel::Transition::HIGH_TO_LOW)
        self->m_contactPinConnected = true;
}

void BonderModule::onMouseRightButtonTransition(void *context,
                                                 PinMonitorChannel::Transition transition)
{
    auto *self = static_cast<BonderModule*>(context);
    if (transition == PinMonitorChannel::Transition::LOW_TO_HIGH)
        self->m_rightButtonPressed = true;
    else if (transition == PinMonitorChannel::Transition::HIGH_TO_LOW)
        self->m_rightButtonReleased = true;
}

void BonderModule::onTimerDone(void *context, Timer *t)
{
    (void)t;
    static_cast<BonderModule*>(context)->m_timerExpired = true;
}

void BonderModule::onPllEvent(void *context, PllModule::Event event)
{
    auto *self = static_cast<BonderModule*>(context);
    if (event == PllModule::Event::BondingCompleted)
        self->m_usPowerTransferred = true;
    else if (event == PllModule::Event::InsufficientBondingPower)
        self->m_usPowerError = true;
}

void BonderModule::onForceCoilEvent(ForceCoilDriverModule::Event eventId)
{
    if (!s_instance) return;
    if (eventId == ForceCoilDriverModule::Event::SetpointAchieved)
        s_instance->m_forceCoilCurrentSettled = true;
    else if (eventId == ForceCoilDriverModule::Event::UnableToSetCurrent)
        s_instance->m_forceCoilError = true;
}

bool BonderModule::onZMotorPositionSetpoint(void *context, float *positionSetpoint)
{
    auto *self = static_cast<BonderModule*>(context);

    if (positionSetpoint == nullptr ||
        self->m_systemState != State::Operating ||
        !self->m_zMotorSetpointActive) {
        return false;
    }

    *positionSetpoint = self->m_zMotorPositionSetpoint;
    return true;
}

void BonderModule::onImpedanceScanned(void *context, complexf *v, complexf *c, complexf *i)
{
    (void)v; (void)c; (void)i;
    static_cast<BonderModule*>(context)->m_impedanceScanningCompleted = true;
}

void BonderModule::onYAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule*>(context)->m_yMoveCompleted = true;
}

void BonderModule::onTAxisRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;
    static_cast<BonderModule*>(context)->m_tMoveCompleted = true;
}
