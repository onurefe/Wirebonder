#ifndef ROUTER_MODULE_HPP
#define ROUTER_MODULE_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "generic.h"
#include "configuration.h"
#include "stepper_service.hpp"
#include "process.hpp"

// -----------------------------------------------------------------------
// Class: RouterChannel
// -----------------------------------------------------------------------
class RouterChannel {
public:
    using Callback = void (*)(void *context, RouterChannel *channel);

    RouterChannel(StepperChannel *stepper,
                  float maxVel,
                  float maxAcc,
                  float stepPerMm);

    void addMoveCompleteListenerCallback(void *context, Callback cb);

    // Move relative to the last completed logical position.
    void append(float displacement);
    // Move to an absolute logical position. Use setPosition() to establish a
    // physical reference; process restarts do not silently rebase the axis.
    void moveTo(float position);
    // Move with a velocity cap below the channel's configured maximum.
    void moveTo(float position, float velocityLimit);
    // Rebase the logical coordinate after a physical reference is found.
    bool setPosition(float position);
    void setTravelLimits(float minimumPosition, float maximumPosition);
    void clearTravelLimits();
    float getPosition() const;
    bool isBusy() const;

    float stepsToMillimeters(int32_t positionInSteps) const;

    void start();    // called by StepperRouterService::onStart
    void stop();     // called by StepperRouterService::onStop
    void execute();  // called by StepperRouterService::onExecute

private:
    struct RouteParams {
        float d;
        float vMax;
        float aMax;
        float tAcc;
        float dAcc;
        float tConst;
        float dConst;
    };

    void      formTrapezoidRoute(float displacement, float velocityLimit);
    float     getRoutePosition(bool &endOfRoute) const;
    float     getPositionAccelerating(float t) const;
    float     getPositionConstantVelocity(float t) const;
    float     getPositionDecelerating(float t) const;
    qint7_8_t  getStepperServiceSegment(float destinationPosition);
    qint55_8_t convertToStepperServicePosition(float position) const;

    StepperChannel *m_stepper;
    float           m_maxVelocity;
    float           m_maxAcceleration;
    float           m_stepPerMm;
    bool            m_travelLimitsEnabled;
    float           m_minimumPosition;
    float           m_maximumPosition;
    Callback        m_callback;
    void           *m_callbackContext;
    RouteParams     m_routeParams;
    bool            m_isBusy;
    float           m_position;       // last completed logical position (mm)
    float           m_targetPosition; // active move target (mm)
    uint32_t        m_numOfRenderedSegments;
    qint55_8_t      m_lastRenderedStepperServicePosition;
    int32_t         m_moveStartStepperPosition;
};

// -----------------------------------------------------------------------
// Class: StepperRouterService
// -----------------------------------------------------------------------
class StepperRouterService : public Process {
public:
    StepperRouterService();

    bool addChannel(RouterChannel *channel);

private:
    void onStart() override;
    void onStop() override;
    void onExecute() override;

    RouterChannel *m_channels[ROUTER_MODULE_MAX_NUM_OF_ROUTERS];
    uint8_t        m_numChannels;
};

#endif /* ROUTER_MODULE_HPP */
