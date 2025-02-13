#include "stepper.h"
#include "router.h"
#include "math.h"


/* Private typedefs --------------------------------------------------------*/
#define SEGMENT_RENDER_PERIOD (1. / BLOCK_EXECUTER_SEGMENT_RENDER_FREQUENCY)

/* Private variables -------------------------------------------------------*/
static g_state = STATE_UNINIT;

static Router_Handle_t *g_registeredRouters[ROUTER_MAX_NUM_OF_ROUTERS];
static uint8_t g_numRouters;

/* Private functions -------------------------------------------------------*/
qint23_8_t clamp(qint23_8_t value, qint23_8_t limit_min, qint23_8_t limit_max)
{
    value = (value > limit_max) ? limit_max : value;
    value = (value < limit_min) ? limit_min : value;

    return value;
}

Bool_t stepperBufferAvailable(Stepper_Handle_t *handle)
{
    return Stepper_SegmentQueueIsAvailable(handle);
}

Bool_t stepperBufferEmpty(Stepper_Handle_t *handle)
{
    return (Stepper_GetPendingSegmentCount(handle) == 0);
}

qint55_8_t convertToStepperPosition(float position)
{
    qint55_8_t position_in_steps;
    position_in_steps = (qint55_8_t)(position * 256 * 1000.0 * ROUTER_STEP_PER_MM);
    return position_in_steps;
}

void formTrapezoidRoute(Router_RouteParams_t *routeParams,
                        float displacement, 
                        float maxVelocity,
                        float maxAcceleration)
{
    float max_achievable_velocity = sqrtf(displacement * maxAcceleration);
    
    if (maxVelocity > max_achievable_velocity)
    {
        maxVelocity = max_achievable_velocity;
    }

    /* Compute the parameters for trajectory calculation. */
    float t_acc = maxVelocity / maxAcceleration;
    float d_acc = 0.5 * maxAcceleration * t_acc * t_acc;
    float d_dec = maxVelocity * t_acc - 0.5 * maxAcceleration * t_acc * t_acc;
    float d_const = (displacement - (d_acc + d_dec));
    float t_const = d_const / maxVelocity;

    /* Set the parameters for later use.*/
    routeParams->d = displacement;
    routeParams->vMax = maxVelocity;
    routeParams->aMax = maxAcceleration;
    routeParams->tAcc = t_acc;
    routeParams->dAcc = d_acc;
    routeParams->tConst = t_const;
    routeParams->dConst = d_const;
}

float getPositionAccelerating(Router_RouteParams_t *routeParams, float t)
{
    return 0.5 * routeParams->aMax * t * t;
}

float getPositionConstantVelocity(Router_RouteParams_t *routeParams, float t)
{
    float x0 = routeParams->dAcc;
    float t0 = routeParams->tAcc;
        
    float t_delta = (t - t0);
    float x_delta = routeParams->vMax * (t - t0);

    return (x0 + x_delta);
}

float getPositionDecelerating(Router_RouteParams_t *routeParams, float t)
{
    float x0 = routeParams->dAcc + routeParams->dConst;
    float t0 = routeParams->tAcc + routeParams->tConst;

    float t_delta = (t - t0);
    float x_delta = routeParams->vMax * t_delta - 0.5 * routeParams->aMax * t_delta * t_delta;
        
    return (x0 + x_delta);
}

float getPosition(Router_Handle_t *handle, Bool_t *endOfRoute)
{
    float x;
    float t;
    
    t = (float)handle->numOfRenderedSegments * SEGMENT_RENDER_PERIOD;
    Router_RouteParams_t *route_params = &handle->routeParams;

    if (route_params->tAcc > t) 
    {
        x = getPositionAccelerating(route_params, t);
    } 
    else if ((route_params->tConst + route_params->tAcc) > t) 
    {
        x= getPositionConstantVelocity(route_params, t);
    } 
    else if ((route_params->tConst + 2 * route_params->tAcc) > t) 
    {
        x = getPositionDecelerating(route_params, t);
    } 
    else
    {
        *endOfRoute = TRUE;
        x = route_params->d;
    }

    return x;
}

qint7_8_t getStepperSegment(Router_Handle_t *handle, float destinationPosition)
{
    qint55_8_t segment_displacement;
    qint55_8_t destination_position_in_steps;

    destination_position_in_steps = convertToStepperPosition(destinationPosition);
    segment_displacement = destination_position_in_steps - handle->lastRenderedStepperPosition;
    segment_displacement = clamp(segment_displacement, MIN_INT16, MAX_INT16);
    handle->lastRenderedStepperPosition += segment_displacement;

    return ((qint7_8_t)segment_displacement);
}

void executeRouter(Router_Handle_t *handle)
{
    if (!handle->isBusy) 
    {   
        return;
    }

    Bool_t end_of_route;
    
    if (stepperBufferAvailable(&handle->stepper))
    {
        float position = getPosition(handle, &end_of_route);
        qint7_8_t segment = getStepperSegment(handle, position);

        Bool_t no_new_segment = end_of_route && (segment == 0);

        /* End of route execution. */
        if (stepperBufferEmpty(&handle->stepper) && no_new_segment)
        {
            handle->callback();
            handle->isBusy = FALSE;
        }
        else
        {
            Stepper_EnqueueSegment(&handle->stepper, segment);
            handle->numOfRenderedSegments++;
        }
    }

}
/* Exported functions ------------------------------------------------------*/
void Router_Init(GPIO_TypeDef *enPort, 
                 uint16_t enPin, 
                 GPIO_TypeDef *rstPort, 
                 uint16_t rstPin)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    g_numRouters = 0;

    Stepper_Init(enPort, enPin, rstPort, rstPin);
    
    g_state = STATE_READY;
}

void Router_Register(Router_Handle_t *handle, 
                     GPIO_TypeDef *stepPort, 
                     uint16_t stepPin, 
                     GPIO_TypeDef *dirPort, 
                     uint16_t dirPin,
                     float maxVelocity, 
                     float maxAcceleration, 
                     Router_MotionCompletedCallback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    Stepper_Register(&handle->stepper, 
                     stepPort, 
                     stepPin, 
                     dirPort, 
                     dirPin);

    handle->maxVelocity = maxVelocity;
    handle->maxAcceleration = maxAcceleration;
    handle->callback = callback;

    g_registeredRouters[g_numRouters++] = handle;
}

void Router_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    for (uint16_t i = 0; i < g_numRouters; i++)
    {
        Router_Handle_t *handle = g_registeredRouters[i];

        handle->lastRenderedStepperPosition = 0;
        handle->numOfRenderedSegments = 0;
        handle->isBusy = FALSE;
    }

    Stepper_Start();
    g_state = STATE_OPERATING;
}

void Router_Execute(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    for (uint16_t i = 0; i < g_numRouters; i++)
    {
        executeRouter(g_registeredRouters[i]);
    }    
}

void Router_TimerTick(void)
{
    Stepper_TimerTick();
}

float Router_StepsToMeters(int32_t positionInSteps)
{
    return (((float)positionInSteps) / (1000.0 * ROUTER_STEP_PER_MM));
}

