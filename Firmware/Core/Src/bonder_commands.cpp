#include "bonder_commands.hpp"
#include "transducer_analyzer.hpp"
#include <cmath>

namespace {

// No Z command may ask for a position outside the travel window. Applied to
// the target rather than to the setpoint on its way out, so a command that is
// clamped still knows where it is going and completes there.
inline float clampToTravel(float position)
{
    return fminf(fmaxf(position, BONDER_MODULE_ZAXIS_MIN_POSITION),
                 BONDER_MODULE_ZAXIS_MAX_POSITION);
}

// The position loop calls its setpoint provider once per control tick, so this
// is the integration step of anything that shapes a setpoint from there.
constexpr float kControlPeriod = 1.0f / DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY;

// -----------------------------------------------------------------------------
// Trapezoid pieces, shared by the commands that walk a Z setpoint themselves.
// A ramp is specified by the distance it occupies rather than its duration, so
// it is the same length of travel whatever speed it runs at: a = v^2 / 2d.
// -----------------------------------------------------------------------------

inline float rampAcceleration(float speed, float accelerationDistance)
{
    return (accelerationDistance > 0.0f)
        ? ((speed * speed) / (2.0f * accelerationDistance))
        : 0.0f;
}

// The fastest approach that can still be brought to a stop in the travel that
// is left, so the ramp ends on the destination rather than at it.
inline float stopInTimeSpeed(float acceleration, float remaining)
{
    return sqrtf(2.0f * acceleration * fmaxf(remaining, 0.0f));
}

// One tick of velocity slew towards a target. A zero acceleration means no
// ramp at all: the velocity, and with it the setpoint, steps.
inline float slewVelocity(float velocity, float targetVelocity, float acceleration)
{
    if (acceleration <= 0.0f) {
        return targetVelocity;
    }

    const float step = acceleration * kControlPeriod;

    return (velocity < targetVelocity) ? fminf(velocity + step, targetVelocity)
                                       : fmaxf(velocity - step, targetVelocity);
}

// Never let a walked setpoint run away from the carriage: if the axis cannot
// keep up -- a commanded speed above the loop's velocity clamp, or something
// in the way -- an unbounded lead would be worked off as a rush the moment the
// obstruction cleared.
inline float boundedSetpoint(float setpoint, float measured)
{
    const float lead = BONDER_COMMAND_MZDRIVE_MAX_FOLLOWING_ERROR;

    return fminf(fmaxf(setpoint, measured - lead), measured + lead);
}

} // namespace

// =============================================================================
// BonderCommandZMove
// =============================================================================

bool BonderCommandZMove::start(void *args)
{
    BonderCommandZMove::Args *casted_args = reinterpret_cast<BonderCommandZMove::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_destination = clampToTravel(casted_args->zPosition);
    armTimeout(BONDER_COMMAND_ZMOVE_TIMEOUT_MS);

    /* Start from where the carriage is and walk to the destination, so the
       loop is never handed a step. */
    m_setpoint = m_resources->m_zMotorController->getPosition();
    m_velocity = 0.0f;
    m_speed = casted_args->maxSpeed;
    m_acceleration = casted_args->maxAcceleration;

    m_resources->m_zMotorController->addPositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);

    m_resources->m_zMotorController->restartControlLoop();

    return true;
}

BonderCommand::InstrStatus BonderCommandZMove::execute()
{
    if (!m_resources->m_zMotorController->isControlEnabled()) {
        notify(BonderCommandZMove::EventId::ControllerInitializationError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    if (isMotionCompleted()) {
        notify(BonderCommandZMove::EventId::SetpointReached);

        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandZMove::EventId::PositionError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

// Against this command's own destination, so a second Z command holding the
// arbiter cannot report an arrival on its behalf.
bool BonderCommandZMove::isMotionCompleted() const
{
    return fabsf(m_resources->m_zMotorController->getPosition() - m_destination) <
           DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR;
}

bool BonderCommandZMove::stop()
{
    m_resources->m_zMotorController->removePositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);

    return true;
}

bool BonderCommandZMove::onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                                  float *velocityFeedforward)
{
    BonderCommandZMove *self = static_cast<BonderCommandZMove *>(context);

    if ((positionSetpoint == nullptr) || (velocityFeedforward == nullptr)) {
        return false;
    }

    self->advanceProfile();
    *positionSetpoint = self->m_setpoint;
    // The profile's own velocity: the loop is left with the residual only.
    *velocityFeedforward = self->m_velocity;

    return true;
}

// One tick of the trapezoid: full speed towards the destination, given up
// early enough to stop on it, slewed into the current velocity and integrated.
void BonderCommandZMove::advanceProfile()
{
    const float remaining = m_destination - m_setpoint;
    const bool descending = (remaining < 0.0f);

    float targetVelocity = descending ? -m_speed : m_speed;
    const float limit = stopInTimeSpeed(m_acceleration, fabsf(remaining));

    if (m_acceleration > 0.0f) {
        targetVelocity = descending ? fmaxf(targetVelocity, -limit)
                                    : fminf(targetVelocity, limit);
    }

    m_velocity = slewVelocity(m_velocity, targetVelocity, m_acceleration);
    m_setpoint += m_velocity * kControlPeriod;

    // Stop on the destination rather than walking past it.
    const bool overshot = descending ? (m_setpoint < m_destination)
                                     : (m_setpoint > m_destination);
    if (overshot) {
        m_setpoint = m_destination;
        m_velocity = 0.0f;
    }

    m_setpoint = boundedSetpoint(m_setpoint,
                                 m_resources->m_zMotorController->getPosition());
}

// =============================================================================
// BonderCommandTachMove
// =============================================================================

bool BonderCommandTachMove::start(void *args)
{
    (void)args;

    m_setpoint = ZMOTOR_TACH_CAL_POSITION_MM;
    armTimeout(BONDER_COMMAND_ZMOVE_TIMEOUT_MS);

    m_resources->m_zMotorController->addPositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);

    m_resources->m_zMotorController->restartControlLoop();

    return true;
}

BonderCommand::InstrStatus BonderCommandTachMove::execute()
{
    if (!m_resources->m_zMotorController->isControlEnabled()) {
        notify(BonderCommandTachMove::EventId::ControllerInitializationError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    if (isMotionCompleted()) {
        notify(BonderCommandTachMove::EventId::SetpointReached);

        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandTachMove::EventId::PositionError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

// Against this command's own setpoint, so a second Z command holding the
// arbiter cannot report an arrival on its behalf.
bool BonderCommandTachMove::isMotionCompleted() const
{
    return fabsf(m_resources->m_zMotorController->getPosition() - m_setpoint) <
           DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR;
}

bool BonderCommandTachMove::stop()
{
    m_resources->m_zMotorController->removePositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);

    return true;
}

bool BonderCommandTachMove::onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                                     float *velocityFeedforward)
{
    BonderCommandTachMove *self = static_cast<BonderCommandTachMove *>(context);

    if ((positionSetpoint == nullptr) || (velocityFeedforward == nullptr)) {
        return false;
    }

    *positionSetpoint = self->m_setpoint;
    // A step setpoint carries no profile, so the loop does all of the work.
    *velocityFeedforward = 0.0f;

    return true;
}

// =============================================================================
// BonderCommandOpenZMove
// =============================================================================

bool BonderCommandOpenZMove::start(void *args)
{
    BonderCommandOpenZMove::Args *casted_args =
        reinterpret_cast<BonderCommandOpenZMove::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_drive = casted_args->drive;
    armTimeout(static_cast<uint32_t>(casted_args->duration * 1000.0f));

    /* Both loops out of the way: the position loop passes its input straight
       through as a target velocity, and the velocity PID passes that straight
       through as a drive. What the setpoint callback returns is therefore the
       voltage the motor sees. */
    m_resources->m_zMotorController->enableBypass();
    m_resources->m_zMotorController->enableDriveBypass();
    m_resources->m_zMotorController->addPositionSetpointControllerCallback(
        this, &this->onZMotorPositionSetpoint);

    return true;
}

// The duration is the completion condition, not a failure: this is a timed
// push, so running out of time is how it ends.
BonderCommand::InstrStatus BonderCommandOpenZMove::execute()
{
    if (!hasTimedOut()) {
        return BonderCommand::InstrStatus::Running;
    }

    notify(BonderCommandOpenZMove::EventId::MoveCompleted);
    stop();

    return BonderCommand::InstrStatus::Done;
}

bool BonderCommandOpenZMove::stop()
{
    m_resources->m_zMotorController->removePositionSetpointControllerCallback(
        this, &this->onZMotorPositionSetpoint);
    m_resources->m_zMotorController->disableDriveBypass();
    m_resources->m_zMotorController->disableBypass();

    return true;
}

bool BonderCommandOpenZMove::onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                                      float *velocityFeedforward)
{
    BonderCommandOpenZMove *self = static_cast<BonderCommandOpenZMove *>(context);

    if ((positionSetpoint == nullptr) || (velocityFeedforward == nullptr)) {
        return false;
    }

    // Bypassed, so this is a drive voltage rather than a position.
    *positionSetpoint = self->m_drive;
    *velocityFeedforward = 0.0f;

    return true;
}

// =============================================================================
// BonderCommandZReference
// =============================================================================

bool BonderCommandZReference::start(void *args)
{
    (void)args;

    m_measuredPosition = m_resources->m_zMotorController->getPosition();

    return true;
}

BonderCommand::InstrStatus BonderCommandZReference::execute()
{
    notify(BonderCommandZReference::EventId::OriginMeasured, &m_measuredPosition);

    return BonderCommand::InstrStatus::Done;
}

// =============================================================================
// BonderCommandMzDrive
// =============================================================================

bool BonderCommandMzDrive::start(void *args)
{
    BonderCommandMzDrive::Args *casted_args = reinterpret_cast<BonderCommandMzDrive::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_lowerHeight = clampToTravel(casted_args->lowerHeight);
    m_upperHeight = clampToTravel(casted_args->upperHeight);
    m_driveSpeed = casted_args->maxSpeed;
    m_acceleration = rampAcceleration(casted_args->maxSpeed,
                                      casted_args->maxStopDistance);

    m_ditherAmplitude = casted_args->ditherAmplitude;
    m_ditherDivider = casted_args->ditherDivider;
    m_ditherTicks = 0U;
    m_ditherPositive = false;

    // Stay put until a button is pressed: the axis is already parked wherever
    // the previous command left it.
    m_direction.store(Direction::None);
    m_destinationCommanded.store(false);
    m_setpoint = m_resources->m_zMotorController->getPosition();
    m_velocity = 0.0f;

    m_resources->m_zMotorController->addPositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);
    m_resources->m_zMotorController->restartControlLoop();

    return true;
}

BonderCommand::InstrStatus BonderCommandMzDrive::execute()
{
    if (!m_resources->m_zMotorController->isControlEnabled()) {
        notify(BonderCommandMzDrive::EventId::ControllerInitializationError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    // Read the buttons where they live; nothing relays them in.
    const bool lowerHeld =
        (m_resources->m_rightMouseButtonMonitor->getPinState() ==
         PinMonitorChannel::PinState::ACTIVE);
    const bool raiseHeld =
        (m_resources->m_leftMouseButtonMonitor->getPinState() ==
         PinMonitorChannel::PinState::ACTIVE);

    Direction direction = Direction::None;
    if (lowerHeld && !raiseHeld) {
        direction = Direction::Lower;
    } else if (raiseHeld && !lowerHeld) {
        direction = Direction::Raise;
    }

    // The control loop walks the setpoint from here; releasing both buttons
    // simply stops walking it, which parks the carriage where it stands.
    m_direction.store(direction);

    if (isMotionCompleted()) {
        notify(BonderCommandMzDrive::EventId::LowerBoundReached);

        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandMzDrive::stop()
{
    m_resources->m_zMotorController->removePositionSetpointControllerCallback(this, &this->onZMotorPositionSetpoint);

    return true;
}

bool BonderCommandMzDrive::onZMotorPositionSetpoint(void *context, float *positionSetpoint,
                                                    float *velocityFeedforward)
{
    BonderCommandMzDrive *self = static_cast<BonderCommandMzDrive *>(context);

    if ((positionSetpoint == nullptr) || (velocityFeedforward == nullptr)) {
        return false;
    }

    self->advanceProfile();
    *positionSetpoint = self->m_setpoint;
    /* The profile's own velocity, so the loop is left with the residual only,
       plus the dither -- which reaches the velocity loop's command exactly
       where its derivative term can turn each edge into a torque kick. */
    *velocityFeedforward = self->m_velocity + self->ditherVelocity();

    return true;
}

// Square wave at the control rate divided down. Amplitude is small enough to
// average to nothing mechanically; what breaks the stiction is the edge, not
// the level -- the velocity loop's derivative term sees a step and answers
// with an impulse, which is the same trick as injecting a digital square into
// an analogue velocity loop's summing junction.
float BonderCommandMzDrive::ditherVelocity()
{
    if ((m_ditherAmplitude <= 0.0f) || (m_ditherDivider == 0U)) {
        return 0.0f;
    }

    m_ditherTicks++;
    if (m_ditherTicks >= m_ditherDivider) {
        m_ditherTicks = 0U;
        m_ditherPositive = !m_ditherPositive;
    }

    return m_ditherPositive ? m_ditherAmplitude : -m_ditherAmplitude;
}

// One tick of the trapezoid. Up is positive throughout, so a descent is the
// negative direction and the descent target is the smaller coordinate.
void BonderCommandMzDrive::advanceProfile()
{
    advanceVelocity(approachLimited(requestedVelocity()));
    advanceSetpoint();
    limitSetpointLead();
}

// Both halves of it: the profile has walked the setpoint onto the descent
// target, and the carriage has caught up. The position loop's own
// SetpointReached answers neither -- it tracks a walked setpoint within
// tolerance for the whole descent.
bool BonderCommandMzDrive::isMotionCompleted() const
{
    if (!m_destinationCommanded.load()) {
        return false;
    }

    const float error =
        fabsf(m_resources->m_zMotorController->getPosition() - m_lowerHeight);

    return error < DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR;
}

// Full speed towards whichever bound the operator is holding, nothing when
// they are holding neither or both.
float BonderCommandMzDrive::requestedVelocity() const
{
    switch (m_direction.load()) {
    case Direction::Lower:
        return -m_driveSpeed;
    case Direction::Raise:
        return m_driveSpeed;
    case Direction::None:
    default:
        return 0.0f;
    }
}

// The same request, given up early enough to stop on the bound rather than at
// it: sqrt(2 a s) is the fastest the ramp can still be run out in the travel
// that is left.
float BonderCommandMzDrive::approachLimited(float velocity) const
{
    if ((m_acceleration <= 0.0f) || (velocity == 0.0f)) {
        return velocity;
    }

    const float remaining = (velocity > 0.0f)
        ? (m_upperHeight - m_setpoint)
        : (m_setpoint - m_lowerHeight);
    const float limit = stopInTimeSpeed(m_acceleration, remaining);

    return (velocity > 0.0f) ? fminf(velocity, limit) : fmaxf(velocity, -limit);
}

// Slew towards the target at the ramp's acceleration, which is what makes a
// press, a release and a reversal all take the same travel.
void BonderCommandMzDrive::advanceVelocity(float targetVelocity)
{
    m_velocity = slewVelocity(m_velocity, targetVelocity, m_acceleration);
}

void BonderCommandMzDrive::advanceSetpoint()
{
    const bool descending = (m_velocity < 0.0f);
    const bool ascending = (m_velocity > 0.0f);

    m_setpoint += m_velocity * kControlPeriod;

    const float ascentTarget = m_upperHeight;
    const float descentTarget = m_lowerHeight;

    if ((m_setpoint >= ascentTarget) || (m_setpoint <= descentTarget)) {
        // Reaching the descent target under power is what completes the
        // command; sitting there at the outset does not.
        if (descending && (m_setpoint <= descentTarget)) {
            m_destinationCommanded.store(true);
        }

        m_setpoint = fminf(fmaxf(m_setpoint, descentTarget), ascentTarget);
        m_velocity = 0.0f;
    } else if (ascending) {
        // Walked back off the target, so a later descent has to earn it again.
        m_destinationCommanded.store(false);
    }
}

void BonderCommandMzDrive::limitSetpointLead()
{
    m_setpoint = boundedSetpoint(m_setpoint,
                                 m_resources->m_zMotorController->getPosition());
}

// =============================================================================
// BonderCommandWaitZPosition
// =============================================================================

bool BonderCommandWaitZPosition::start(void *args)
{
    BonderCommandWaitZPosition::Args *casted_args =
        reinterpret_cast<BonderCommandWaitZPosition::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_targetHeight = clampToTravel(casted_args->zPosition);
    armTimeout(casted_args->timeoutMs);

    const float measuredHeight = m_resources->m_zMotorController->getPosition();
    m_awaitingAscent = (measuredHeight < m_targetHeight);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitZPosition::execute()
{
    const float measuredHeight = m_resources->m_zMotorController->getPosition();

    const bool reached = m_awaitingAscent ? (measuredHeight >= m_targetHeight)
                                          : (measuredHeight <= m_targetHeight);

    if (reached) {
        notify(BonderCommandWaitZPosition::EventId::PositionReached);
        return BonderCommand::InstrStatus::Done;
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitZPosition::EventId::TimedOut);
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitZPosition::stop()
{
    return true;
}

// =============================================================================
// BonderCommandYMove
// =============================================================================

bool BonderCommandYMove::start(void *args)
{
    BonderCommandYMove::Args *casted_args = reinterpret_cast<BonderCommandYMove::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_moveCompleted = false;
    armTimeout(BONDER_COMMAND_YMOVE_TIMEOUT_MS);

    m_resources->m_yAxisRouter->addMoveCompleteListenerCallback(this, &this->onRouterDone);
    m_resources->m_yAxisRouter->moveTo(casted_args->yPosition);

    return true;
}

BonderCommand::InstrStatus BonderCommandYMove::execute()
{
    if (m_moveCompleted) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandYMove::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandYMove::stop()
{
    m_resources->m_yAxisRouter->removeMoveCompleteListenerCallback(this, &this->onRouterDone);

    return true;
}

void BonderCommandYMove::onRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;

    BonderCommandYMove *self = static_cast<BonderCommandYMove *>(context);

    self->m_moveCompleted = true;
    self->notify(BonderCommandYMove::EventId::MoveCompleted);
}

// =============================================================================
// BonderCommandYReverse
// =============================================================================

bool BonderCommandYReverse::start(void *args)
{
    BonderCommandYReverse::Args *casted_args = reinterpret_cast<BonderCommandYReverse::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_moveCompleted = false;
    armTimeout(BONDER_COMMAND_YMOVE_TIMEOUT_MS);

    m_resources->m_yAxisRouter->addMoveCompleteListenerCallback(this, &this->onRouterDone);
    m_resources->m_yAxisRouter->moveTo(
        m_resources->m_yAxisRouter->getPosition() - casted_args->displacement);

    return true;
}

BonderCommand::InstrStatus BonderCommandYReverse::execute()
{
    if (m_moveCompleted) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandYReverse::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandYReverse::stop()
{
    m_resources->m_yAxisRouter->removeMoveCompleteListenerCallback(this, &this->onRouterDone);

    return true;
}

void BonderCommandYReverse::onRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;

    BonderCommandYReverse *self = static_cast<BonderCommandYReverse *>(context);

    self->m_moveCompleted = true;
    self->notify(BonderCommandYReverse::EventId::MoveCompleted);
}

// =============================================================================
// BonderCommandTMove
// =============================================================================

bool BonderCommandTMove::start(void *args)
{
    BonderCommandTMove::Args *casted_args = reinterpret_cast<BonderCommandTMove::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_moveCompleted = false;
    armTimeout(BONDER_COMMAND_TMOVE_TIMEOUT_MS);

    m_resources->m_tAxisRouter->addMoveCompleteListenerCallback(this, &this->onRouterDone);
    m_resources->m_tAxisRouter->moveTo(casted_args->relative
        ? m_resources->m_tAxisRouter->getPosition() + casted_args->position
        : casted_args->position);

    return true;
}

BonderCommand::InstrStatus BonderCommandTMove::execute()
{
    if (m_moveCompleted) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandTMove::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandTMove::stop()
{
    m_resources->m_tAxisRouter->removeMoveCompleteListenerCallback(this, &this->onRouterDone);

    return true;
}

void BonderCommandTMove::onRouterDone(void *context, RouterChannel *channel)
{
    (void)channel;

    BonderCommandTMove *self = static_cast<BonderCommandTMove *>(context);

    self->m_moveCompleted = true;
    self->notify(BonderCommandTMove::EventId::MoveCompleted);
}

// =============================================================================
// BonderCommandTimer
// =============================================================================

bool BonderCommandTimer::start(void *args)
{
    BonderCommandTimer::Args *casted_args = reinterpret_cast<BonderCommandTimer::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_expired = false;

    m_resources->m_timer->addExpirationListenerCallback(this, &this->onTimerDone);
    m_resources->m_timer->start(true, casted_args->durationSeconds);

    return true;
}

BonderCommand::InstrStatus BonderCommandTimer::execute()
{
    if (m_expired) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandTimer::stop()
{
    m_resources->m_timer->stop();
    m_resources->m_timer->removeExpirationListenerCallback(this, &this->onTimerDone);

    return true;
}

void BonderCommandTimer::onTimerDone(void *context, Timer *timer)
{
    (void)timer;

    BonderCommandTimer *self = static_cast<BonderCommandTimer *>(context);

    self->m_expired = true;
    self->notify(BonderCommandTimer::EventId::Expired);
}

// =============================================================================
// BonderCommandWaitContactEvents
// =============================================================================

bool BonderCommandWaitContactEvents::start(void *args)
{
    BonderCommandWaitContactEvents::Args *casted_args = reinterpret_cast<BonderCommandWaitContactEvents::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_isRisingEdge = casted_args->isRisingEdge;
    armTimeout(casted_args->timeoutMs);
    m_occurred = false;

    m_resources->m_contactSensorMonitor->addStateListenerCallback(
        this, &this->onContactSensorStateChanged);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitContactEvents::execute()
{
    if (m_occurred) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitContactEvents::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitContactEvents::stop()
{
    m_resources->m_contactSensorMonitor->removeStateListenerCallback(
        this, &this->onContactSensorStateChanged);

    return true;
}

void BonderCommandWaitContactEvents::onContactSensorStateChanged(
    void *context, PinMonitorChannel::PinState state)
{
    BonderCommandWaitContactEvents *self = static_cast<BonderCommandWaitContactEvents *>(context);

    // The monitor only calls on a change, so an active state is the
    // connecting edge and an inactive one the disconnecting edge.
    const bool risingEdge = (state == PinMonitorChannel::PinState::ACTIVE);
    if (risingEdge != self->m_isRisingEdge) {
        return;
    }

    self->m_occurred = true;
    self->notify(BonderCommandWaitContactEvents::EventId::EventOccurred,
                 &self->m_isRisingEdge);
}

// =============================================================================
// BonderCommandWaitContactState
// =============================================================================

bool BonderCommandWaitContactState::start(void *args)
{
    BonderCommandWaitContactState::Args *casted_args = reinterpret_cast<BonderCommandWaitContactState::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_awaitedState = casted_args->state;
    armTimeout(casted_args->timeoutMs);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitContactState::execute()
{
    const bool contactConnected =
        (m_resources->m_contactSensorMonitor->getPinState() ==
         PinMonitorChannel::PinState::ACTIVE);

    if (contactConnected == m_awaitedState) {
        notify(BonderCommandWaitContactState::EventId::StateReached,
               &m_awaitedState);
        return BonderCommand::InstrStatus::Done;
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitContactState::EventId::TimedOut);
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitContactState::stop()
{
    return true;
}

// =============================================================================
// BonderCommandWaitMouseLeftButtonEvents
// =============================================================================

bool BonderCommandWaitMouseLeftButtonEvents::start(void *args)
{
    BonderCommandWaitMouseLeftButtonEvents::Args *casted_args =
        reinterpret_cast<BonderCommandWaitMouseLeftButtonEvents::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_isRisingEdge = casted_args->isRisingEdge;
    armTimeout(casted_args->timeoutMs);
    m_occurred = false;

    m_resources->m_leftMouseButtonMonitor->addStateListenerCallback(
        this, &this->onButtonStateChanged);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitMouseLeftButtonEvents::execute()
{
    if (m_occurred) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitMouseLeftButtonEvents::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitMouseLeftButtonEvents::stop()
{
    m_resources->m_leftMouseButtonMonitor->removeStateListenerCallback(
        this, &this->onButtonStateChanged);

    return true;
}

void BonderCommandWaitMouseLeftButtonEvents::onButtonStateChanged(
    void *context, PinMonitorChannel::PinState state)
{
    BonderCommandWaitMouseLeftButtonEvents *self =
        static_cast<BonderCommandWaitMouseLeftButtonEvents *>(context);

    // The monitor only calls on a change, so an active state is the press and
    // an inactive one the release.
    const bool risingEdge = (state == PinMonitorChannel::PinState::ACTIVE);
    if (risingEdge != self->m_isRisingEdge) {
        return;
    }

    self->m_occurred = true;
    self->notify(BonderCommandWaitMouseLeftButtonEvents::EventId::EventOccurred,
                 &self->m_isRisingEdge);
}

// =============================================================================
// BonderCommandWaitMouseLeftButtonState
// =============================================================================

bool BonderCommandWaitMouseLeftButtonState::start(void *args)
{
    BonderCommandWaitMouseLeftButtonState::Args *casted_args =
        reinterpret_cast<BonderCommandWaitMouseLeftButtonState::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_awaitedState = casted_args->state;
    armTimeout(casted_args->timeoutMs);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitMouseLeftButtonState::execute()
{
    const bool buttonPressed =
        (m_resources->m_leftMouseButtonMonitor->getPinState() ==
         PinMonitorChannel::PinState::ACTIVE);

    if (buttonPressed == m_awaitedState) {
        notify(BonderCommandWaitMouseLeftButtonState::EventId::StateReached,
               &m_awaitedState);
        return BonderCommand::InstrStatus::Done;
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitMouseLeftButtonState::EventId::TimedOut);
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitMouseLeftButtonState::stop()
{
    return true;
}

// =============================================================================
// BonderCommandWaitMouseRightButtonEvents
// =============================================================================

bool BonderCommandWaitMouseRightButtonEvents::start(void *args)
{
    BonderCommandWaitMouseRightButtonEvents::Args *casted_args =
        reinterpret_cast<BonderCommandWaitMouseRightButtonEvents::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_isRisingEdge = casted_args->isRisingEdge;
    armTimeout(casted_args->timeoutMs);
    m_occurred = false;

    m_resources->m_rightMouseButtonMonitor->addStateListenerCallback(
        this, &this->onButtonStateChanged);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitMouseRightButtonEvents::execute()
{
    if (m_occurred) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitMouseRightButtonEvents::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitMouseRightButtonEvents::stop()
{
    m_resources->m_rightMouseButtonMonitor->removeStateListenerCallback(
        this, &this->onButtonStateChanged);

    return true;
}

void BonderCommandWaitMouseRightButtonEvents::onButtonStateChanged(
    void *context, PinMonitorChannel::PinState state)
{
    BonderCommandWaitMouseRightButtonEvents *self =
        static_cast<BonderCommandWaitMouseRightButtonEvents *>(context);

    const bool risingEdge = (state == PinMonitorChannel::PinState::ACTIVE);
    if (risingEdge != self->m_isRisingEdge) {
        return;
    }

    self->m_occurred = true;
    self->notify(BonderCommandWaitMouseRightButtonEvents::EventId::EventOccurred,
                 &self->m_isRisingEdge);
}

// =============================================================================
// BonderCommandWaitMouseRightButtonState
// =============================================================================

bool BonderCommandWaitMouseRightButtonState::start(void *args)
{
    BonderCommandWaitMouseRightButtonState::Args *casted_args =
        reinterpret_cast<BonderCommandWaitMouseRightButtonState::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_awaitedState = casted_args->state;
    armTimeout(casted_args->timeoutMs);

    return true;
}

BonderCommand::InstrStatus BonderCommandWaitMouseRightButtonState::execute()
{
    const bool buttonPressed =
        (m_resources->m_rightMouseButtonMonitor->getPinState() ==
         PinMonitorChannel::PinState::ACTIVE);

    if (buttonPressed == m_awaitedState) {
        notify(BonderCommandWaitMouseRightButtonState::EventId::StateReached,
               &m_awaitedState);
        return BonderCommand::InstrStatus::Done;
    }

    if (hasTimedOut()) {
        notify(BonderCommandWaitMouseRightButtonState::EventId::TimedOut);
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandWaitMouseRightButtonState::stop()
{
    return true;
}

// =============================================================================
// BonderCommandClampOpen
// =============================================================================

bool BonderCommandClampOpen::start(void *args)
{
    (void)args;

    m_settled = false;
    armTimeout(BONDER_COMMAND_CLAMP_TIMEOUT_MS);

    if (m_resources->m_clampSolenoid->getState() == SolenoidChannel::State::ENERGIZED &&
        !m_resources->m_clampSolenoid->isTransitioning()) {
        // Already there, so no state change is coming: raise the event here,
        // or a wait on it would never be satisfied.
        m_settled = true;
        notify(BonderCommandClampOpen::EventId::ClampSettled);
        return true;
    }

    m_resources->m_clampSolenoid->addStateListenerCallback(this, &this->onClampStateChanged);
    m_resources->m_clampSolenoid->energize();

    return true;
}

BonderCommand::InstrStatus BonderCommandClampOpen::execute()
{
    if (m_settled) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandClampOpen::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandClampOpen::stop()
{
    m_resources->m_clampSolenoid->removeStateListenerCallback(this, &this->onClampStateChanged);

    return true;
}

void BonderCommandClampOpen::onClampStateChanged(void *context, SolenoidChannel::State state)
{
    BonderCommandClampOpen *self = static_cast<BonderCommandClampOpen *>(context);

    if (state != SolenoidChannel::State::ENERGIZED) {
        return;
    }

    self->m_settled = true;
    self->notify(BonderCommandClampOpen::EventId::ClampSettled);
}

// =============================================================================
// BonderCommandClampClose
// =============================================================================

bool BonderCommandClampClose::start(void *args)
{
    (void)args;

    m_settled = false;
    armTimeout(BONDER_COMMAND_CLAMP_TIMEOUT_MS);

    if (m_resources->m_clampSolenoid->getState() == SolenoidChannel::State::DEENERGIZED &&
        !m_resources->m_clampSolenoid->isTransitioning()) {
        // Already there, so no state change is coming: raise the event here,
        // or a wait on it would never be satisfied.
        m_settled = true;
        notify(BonderCommandClampClose::EventId::ClampSettled);
        return true;
    }

    m_resources->m_clampSolenoid->addStateListenerCallback(this, &this->onClampStateChanged);
    m_resources->m_clampSolenoid->deenergize();

    return true;
}

BonderCommand::InstrStatus BonderCommandClampClose::execute()
{
    if (m_settled) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandClampClose::EventId::TimedOut);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandClampClose::stop()
{
    m_resources->m_clampSolenoid->removeStateListenerCallback(this, &this->onClampStateChanged);

    return true;
}

void BonderCommandClampClose::onClampStateChanged(void *context, SolenoidChannel::State state)
{
    BonderCommandClampClose *self = static_cast<BonderCommandClampClose *>(context);

    if (state != SolenoidChannel::State::DEENERGIZED) {
        return;
    }

    self->m_settled = true;
    self->notify(BonderCommandClampClose::EventId::ClampSettled);
}

// =============================================================================
// BonderCommandScan
// =============================================================================

bool BonderCommandScan::start(void *args)
{
    BonderCommandScan::Args *casted_args = reinterpret_cast<BonderCommandScan::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_numFrequencies = casted_args->numFrequencies;
    if (m_numFrequencies == 0U) {
        m_numFrequencies = 1U;
    }
    if (m_numFrequencies > BONDER_MODULE_SCAN_MAX_FREQUENCIES) {
        m_numFrequencies = BONDER_MODULE_SCAN_MAX_FREQUENCIES;
    }

    m_startFrequency = casted_args->startFrequency;
    m_frequencyStep = (casted_args->stopFrequency - casted_args->startFrequency) /
                      static_cast<float>(m_numFrequencies);
    m_targetPower = casted_args->targetPower;

    m_scanCompleted = false;
    m_scanFailed = false;
    m_result = Result{};
    armTimeout(BONDER_COMMAND_SCAN_TIMEOUT_MS);

    m_resources->m_usImpedanceScanner->setScanParameters(
        m_numFrequencies, m_startFrequency, m_frequencyStep);
    m_resources->m_usImpedanceScanner->addScanCompleteListenerCallback(
        this, &this->onImpedanceScanned);

    if (!m_resources->m_usImpedanceScanner->beginScan(
            m_voltagePhasors, m_currentPhasors, m_impedances)) {
        m_scanFailed = true;
    }

    return true;
}

BonderCommand::InstrStatus BonderCommandScan::execute()
{
    if (m_scanFailed) {
        notify(BonderCommandScan::EventId::ScanFailed);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    if (m_scanCompleted) {
        computeOperatingPoint();
        notify(BonderCommandScan::EventId::ScanCompleted, &m_result);

        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandScan::EventId::ScanFailed);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandScan::stop()
{
    m_resources->m_usImpedanceScanner->abortScan();
    m_resources->m_usImpedanceScanner->removeScanCompleteListenerCallback(
        this, &this->onImpedanceScanned);

    return true;
}

void BonderCommandScan::onImpedanceScanned(
    void *context, complexf *voltage, complexf *current, complexf *impedances)
{
    (void)voltage;
    (void)current;
    (void)impedances;

    BonderCommandScan *self = static_cast<BonderCommandScan *>(context);

    // The fit runs in execute(), on the caller's context, not here.
    self->m_scanCompleted = true;
}

void BonderCommandScan::computeOperatingPoint()
{
    TransducerAnalyzer::Parameters parameters;
    const bool fitted = TransducerAnalyzer::fit(
        m_impedances,
        static_cast<uint8_t>(m_numFrequencies),
        m_startFrequency,
        m_frequencyStep,
        parameters);

    m_result.qualityFactor = fitted ? parameters.qFactor : 0.0f;
    m_result.impedances = m_impedances;
    m_result.numFrequencies = m_numFrequencies;

    const uint8_t resonanceIndex = findResonanceIndex();
    const float dipFrequency = calculateCenterFrequency(resonanceIndex);
    const float stopFrequency =
        m_startFrequency + static_cast<float>(m_numFrequencies) * m_frequencyStep;

    // Trust the fit only when its resonance also agrees with the raw |Z|
    // minimum: a noise-pulled fit can center the PLL outside its tracking
    // clamp even though the regression converged.
    if (fitted &&
        parameters.seriesResonance >= m_startFrequency &&
        parameters.seriesResonance <= stopFrequency &&
        fabsf(parameters.seriesResonance - dipFrequency) <=
            BONDER_MODULE_FIT_DIP_MAX_DEVIATION_BINS * m_frequencyStep) {
        m_result.centerFrequency = parameters.seriesResonance;
        const complexf admittance =
            TransducerAnalyzer::admittance(parameters, m_result.centerFrequency);
        m_result.driveAmplitude = amplitudeForTargetPower(admittance.re, m_targetPower);

        // Frequency-PID tuning for this specific transducer. A trusted fit is
        // required (same gate as above); an infeasible margin at these
        // loop-shaping targets leaves the PLL on its current gains rather than
        // push a bad tuning in.
        TransducerAnalyzer::FrequencyPidTuning tuning;
        if (TransducerAnalyzer::frequencyPidTuning(
                parameters,
                PLL_MODULE_FREQ_PID_DEAD_TIME,
                PLL_MODULE_FREQ_PID_TARGET_PHASE_MARGIN_RAD,
                PLL_MODULE_FREQ_PID_TARGET_CROSSOVER_FRACTION,
                tuning)) {
            // Back the loop-shaped gain off by the safety margin, and clamp
            // the integral up to its floor: the formula's optimum is faster
            // than the capture transient tolerates (see
            // PLL_MODULE_FREQ_PID_MIN_INTEGRAL_TC).
            m_result.pidTuningValid = true;
            m_result.pidGain = tuning.gain / PLL_MODULE_FREQ_PID_GAIN_SAFETY_MARGIN;
            m_result.pidIntegralTc =
                fmaxf(tuning.integralTc, PLL_MODULE_FREQ_PID_MIN_INTEGRAL_TC);
            m_result.pidDerivativeTc = tuning.derivativeTc;
        }
        return;
    }

    m_result.centerFrequency = dipFrequency;
    m_result.driveAmplitude = calculateDriveAmplitude(resonanceIndex);
}

uint8_t BonderCommandScan::findResonanceIndex() const
{
    uint8_t bestIndex = 0U;
    float bestMagnitude = complexf_abs(m_impedances[0]);

    for (uint16_t i = 1U; i < m_numFrequencies; ++i) {
        const float magnitude = complexf_abs(m_impedances[i]);
        if (magnitude < bestMagnitude) {
            bestMagnitude = magnitude;
            bestIndex = static_cast<uint8_t>(i);
        }
    }

    return bestIndex;
}

float BonderCommandScan::calculateCenterFrequency(uint8_t resonanceIndex) const
{
    return m_startFrequency + static_cast<float>(resonanceIndex) * m_frequencyStep;
}

float BonderCommandScan::calculateDriveAmplitude(uint8_t resonanceIndex) const
{
    const complexf impedance = m_impedances[resonanceIndex];
    const float magnitudeSquared = complexf_abs2(impedance);
    if (magnitudeSquared <= 0.0f) {
        return 0.0f;
    }

    return amplitudeForTargetPower(impedance.re / magnitudeSquared, m_targetPower);
}

float BonderCommandScan::amplitudeForTargetPower(float realAdmittance, float targetPower)
{
    if (realAdmittance <= 0.0f || targetPower <= 0.0f) {
        return 0.0f;
    }

    return sqrtf(targetPower / realAdmittance);
}

// =============================================================================
// BonderCommandPll
// =============================================================================

bool BonderCommandPll::start(void *args)
{
    BonderCommandPll::Args *casted_args = reinterpret_cast<BonderCommandPll::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_transferred = false;
    m_failed = false;
    armTimeout(static_cast<uint32_t>(casted_args->maxDurationSeconds * 1000.0f) +
               BONDER_COMMAND_PLL_TIMEOUT_MARGIN_MS);

    if (casted_args->applyPidTuning) {
        m_resources->m_pll->setFrequencyPidTuning(casted_args->pidGain,
                                                  casted_args->pidIntegralTc,
                                                  casted_args->pidDerivativeTc);
    }

    m_resources->m_pll->addEventListenerCallback(this, &this->onPllEvent);

    if (!m_resources->m_pll->beginTransfer(casted_args->centerFrequency,
                                           casted_args->driveAmplitude,
                                           casted_args->energyJoules,
                                           casted_args->maxDurationSeconds)) {
        m_failed = true;
    }

    return true;
}

BonderCommand::InstrStatus BonderCommandPll::execute()
{
    if (m_failed) {
        notify(BonderCommandPll::EventId::PowerError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    if (m_transferred) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandPll::EventId::PowerError);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandPll::stop()
{
    m_resources->m_pll->abortTransfer();
    m_resources->m_pll->removeEventListenerCallback(this, &this->onPllEvent);

    return true;
}

void BonderCommandPll::onPllEvent(void *context, PllModule::Event event)
{
    BonderCommandPll *self = static_cast<BonderCommandPll *>(context);

    if (event == PllModule::Event::BondingCompleted) {
        self->m_transferred = true;
        self->notify(BonderCommandPll::EventId::PowerTransferred);
    } else if (event == PllModule::Event::InsufficientBondingPower) {
        self->m_failed = true;
    }
}

// =============================================================================
// BonderCommandUsReport
// =============================================================================

bool BonderCommandUsReport::start(void *args)
{
    BonderCommandUsReport::Args *casted_args = reinterpret_cast<BonderCommandUsReport::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_report.resonanceFrequency = casted_args->resonanceFrequency;
    m_report.qualityFactor = casted_args->qualityFactor;
    m_report.transferredPower = m_resources->m_pll->getAveragePower();
    m_report.bondingDuration = m_resources->m_pll->getBondingDuration();

    return true;
}

BonderCommand::InstrStatus BonderCommandUsReport::execute()
{
    notify(BonderCommandUsReport::EventId::ReportReady, &m_report);

    return BonderCommand::InstrStatus::Done;
}

// =============================================================================
// BonderCommandSetForce
// =============================================================================

bool BonderCommandSetForce::start(void *args)
{
    BonderCommandSetForce::Args *casted_args = reinterpret_cast<BonderCommandSetForce::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_settled = false;
    m_failed = false;
    armTimeout(BONDER_COMMAND_SETFORCE_TIMEOUT_MS);

    m_resources->m_forceCoilDriver->addEventListenerCallback(this, &this->onForceCoilEvent);
    m_resources->m_forceCoilDriver->setCurrentSetpoint(
        forceGramsToAmps(correctedForceGrams(casted_args->forceGrams,
                                             casted_args->forceOffsetGrams)));

    return true;
}

BonderCommand::InstrStatus BonderCommandSetForce::execute()
{
    if (m_failed) {
        notify(BonderCommandSetForce::EventId::Error);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    if (m_settled) {
        if (stop()) {
            return BonderCommand::InstrStatus::Done;
        } else {
            return BonderCommand::InstrStatus::Error;
        }
    }

    if (hasTimedOut()) {
        notify(BonderCommandSetForce::EventId::Error);
        stop();
        return BonderCommand::InstrStatus::Error;
    }

    return BonderCommand::InstrStatus::Running;
}

bool BonderCommandSetForce::stop()
{
    // The setpoint stays where it was commanded; only the listener is
    // released, so the force this command established survives it.
    m_resources->m_forceCoilDriver->removeEventListenerCallback(this, &this->onForceCoilEvent);

    return true;
}

// Applies the Setup-measured calibration offset (what the operator's gauge
// read minus what was commanded) to a requested force.
float BonderCommandSetForce::correctedForceGrams(float grams, float offset)
{
    // Zero means "coil off" and has to stay exactly zero -- the offset trims
    // real setpoints, it must never turn a release into a push.
    if (grams <= 0.0f) return 0.0f;

    const float corrected = grams - offset;
    return corrected > 0.0f ? corrected : 0.0f;
}

// Force is configured in grams; the force coil's PID operates on amps.
float BonderCommandSetForce::forceGramsToAmps(float grams)
{
    return (grams - FORCE_COIL_CURRENT_TO_GRAMS_OFFSET) /
        FORCE_COIL_CURRENT_TO_GRAMS_SCALE;
}

void BonderCommandSetForce::onForceCoilEvent(void *context, ForceCoilDriverModule::Event event)
{
    BonderCommandSetForce *self = static_cast<BonderCommandSetForce *>(context);

    if (event == ForceCoilDriverModule::Event::SetpointAchieved) {
        self->m_settled = true;
        self->notify(BonderCommandSetForce::EventId::Settled);
    } else if (event == ForceCoilDriverModule::Event::UnableToSetCurrent) {
        self->m_failed = true;
    }
}

// =============================================================================
// BonderCommandTachSample
// =============================================================================

bool BonderCommandTachSample::start(void *args)
{
    (void)args;

    m_expired = false;
    m_velocitySum = 0.0f;
    m_sampleCount = 0U;
    m_offsetResidual = 0.0f;
    m_positionAtStart = m_resources->m_zMotorController->getPosition();
    m_sampling = true;
    armTimeout(static_cast<uint32_t>(ZMOTOR_TACH_CAL_SAMPLE_DURATION_S * 1000.0f) +
               BONDER_COMMAND_TACHSAMPLE_TIMEOUT_MARGIN_MS);

    m_resources->m_zMotorController->addVelocityListenerCallback(this, &this->onZVelocityMeasured);
    m_resources->m_timer->addExpirationListenerCallback(this, &this->onTimerDone);
    m_resources->m_timer->start(true, ZMOTOR_TACH_CAL_SAMPLE_DURATION_S);

    return true;
}

BonderCommand::InstrStatus BonderCommandTachSample::execute()
{
    if (!m_expired) {
        if (hasTimedOut()) {
            notify(BonderCommandTachSample::EventId::TimedOut);
            stop();
            return BonderCommand::InstrStatus::Error;
        }

        return BonderCommand::InstrStatus::Running;
    }

    const float tachAverage = m_sampleCount > 0U
        ? m_velocitySum / static_cast<float>(m_sampleCount)
        : 0.0f;
    // Ground-truth average velocity from the independent LVDT position
    // measurement, so real motion during an imperfect hold isn't
    // misattributed to tachometer offset. Reduces to tachAverage exactly when
    // position held perfectly still.
    const float positionDelta =
        m_resources->m_zMotorController->getPosition() - m_positionAtStart;
    const float trueAverageVelocity = positionDelta / ZMOTOR_TACH_CAL_SAMPLE_DURATION_S;

    m_offsetResidual = tachAverage - trueAverageVelocity;
    notify(BonderCommandTachSample::EventId::SampleCompleted);

    if (stop()) {
        return BonderCommand::InstrStatus::Done;
    }

    return BonderCommand::InstrStatus::Error;
}

bool BonderCommandTachSample::stop()
{
    m_sampling = false;
    m_resources->m_timer->stop();
    m_resources->m_timer->removeExpirationListenerCallback(this, &this->onTimerDone);
    m_resources->m_zMotorController->removeVelocityListenerCallback(this, &this->onZVelocityMeasured);

    return true;
}

void BonderCommandTachSample::onTimerDone(void *context, Timer *timer)
{
    (void)timer;

    BonderCommandTachSample *self = static_cast<BonderCommandTachSample *>(context);

    self->m_sampling = false;
    self->m_expired = true;
}

void BonderCommandTachSample::onZVelocityMeasured(void *context, float velocity)
{
    BonderCommandTachSample *self = static_cast<BonderCommandTachSample *>(context);

    if (!self->m_sampling) {
        return;
    }

    self->m_velocitySum += velocity;
    self->m_sampleCount++;
}

// =============================================================================
// BonderCommandTachReport
// =============================================================================

bool BonderCommandTachReport::start(void *args)
{
    BonderCommandTachReport::Args *casted_args = reinterpret_cast<BonderCommandTachReport::Args *>(args);

    if (casted_args == nullptr) {
        return false;
    }

    m_offsetResidual = casted_args->offsetResidual;

    return true;
}

BonderCommand::InstrStatus BonderCommandTachReport::execute()
{
    notify(BonderCommandTachReport::EventId::ReportReady, &m_offsetResidual);

    return BonderCommand::InstrStatus::Done;
}
