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

    RouterChannel(StepperChannel *stepper, float maxVel, float maxAcc);

    void addMoveCompleteListenerCallback(void *context, Callback cb);

    void append(float displacement);
    bool isBusy() const;

    static float stepsToMeters(int32_t positionInSteps);

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
    float     getPosition(bool &endOfRoute) const;
    float     getPositionAccelerating(float t) const;
    float     getPositionConstantVelocity(float t) const;
    float     getPositionDecelerating(float t) const;
    qint7_8_t  getStepperServiceSegment(float destinationPosition);
    qint55_8_t convertToStepperServicePosition(float position) const;

    StepperChannel *m_stepper;
    float           m_maxVelocity;
    float           m_maxAcceleration;
    Callback        m_callback;
    void           *m_callbackContext;
    RouteParams     m_routeParams;
    bool            m_isBusy;
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
