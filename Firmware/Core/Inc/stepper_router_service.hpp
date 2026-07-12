#ifndef ROUTER_MODULE_HPP
#define ROUTER_MODULE_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "generic.h"
#include "configuration.h"
#include "stepper_service.hpp"

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
    // Move to an absolute logical position. The origin is zero at start().
    void moveTo(float position);
    float getPosition() const;
    bool isBusy() const;

    float stepsToMillimeters(int32_t positionInSteps) const;

    void start();    // called by StepperRouterService on startService
    void stop();     // called by StepperRouterService on stopService
    void execute();  // called by StepperRouterService on executeService

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

    void      formTrapezoidRoute(float displacement);
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
    Callback        m_callback;
    void           *m_callbackContext;
    RouteParams     m_routeParams;
    bool            m_isBusy;
    float           m_position;       // last completed logical position (mm)
    float           m_targetPosition; // active move target (mm)
    uint32_t        m_numOfRenderedSegments;
    qint55_8_t      m_lastRenderedStepperServicePosition;
};

// -----------------------------------------------------------------------
// Class: StepperRouterService
// -----------------------------------------------------------------------
class StepperRouterService {
public:
    StepperRouterService();

    bool addChannel(RouterChannel *channel);

    void initService()  {}
    void startService();
    void stopService();
    void executeService();

private:
    RouterChannel *m_channels[ROUTER_MODULE_MAX_NUM_OF_ROUTERS];
    uint8_t        m_numChannels;
    ServiceState   m_state;
};

#endif /* ROUTER_MODULE_HPP */
