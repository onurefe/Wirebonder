#include "zmotor_control.h"
#include "tachometer.h"         
#include "lvdt.h"               
#include "pid_controller.h"      
#include "stm32f4xx_hal.h"
#include "generic.h"
#include "configuration.h"
#include "math.h"

static State_t g_state = STATE_UNINIT;

static uint8_t g_numCallbacks;
static ZMotorControl_Callback_t g_callbacks[ZMOTOR_CONTROL_MAX_NUM_OF_CALLBACKS];

static PID_Controller pidPosition;
static PID_Controller pidVelocity;

static float g_positionSetpoint = 0.0f;

static TIM_HandleTypeDef *g_motorDrivePwmHtim;  // PWM timer handle
static uint32_t g_motorDriverPwmChannel;

static Bool_t g_velocityMeasurementReceived;
static Bool_t g_positionMeasurementReceived;
static Bool_t g_newSetpoint;

static float g_velocityMeasurement;
static float g_positionMeasurement;

/* Private functions ------------------------------------*/
static float clip(float value, float minValue, float maxValue)
{
    if (value > maxValue) 
    {
        value = maxValue;
    }
    else if (value < minValue)
    {
        value = minValue;
    }

    return value;
}

static void setDuty(float duty)
{
    duty = clip(duty, ZMOTOR_CONTROL_MIN_DUTY , ZMOTOR_CONTROL_MAX_DUTY);

    uint32_t period = g_motorDrivePwmHtim->Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    
    __HAL_TIM_SET_COMPARE(g_motorDrivePwmHtim, g_motorDriverPwmChannel, pulse);
}

static void notifySetpointAchieved(void)
{
    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        if (g_callbacks[i] != NULL)
        {
            g_callbacks[i](ZMOTOR_CONTROL_SETPOINT_ACHIEVED_EVENT_ID);
        }
    }
}

static void controlUpdate(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }
    
    float desired_velocity = PID_Execute(&pidPosition, 
                                         g_positionSetpoint, 
                                         g_positionMeasurement);

    float velocity_control_output = PID_Execute(&pidVelocity, 
                                                desired_velocity, 
                                                g_velocityMeasurement);
    
    if (g_newSetpoint) 
    {
        float position_error = fabsf(g_positionSetpoint - g_positionMeasurement);
        float velocity_error = fabsf(desired_velocity - g_velocityMeasurement);

        if ((position_error < ZMOTOR_SETPOINT_ACHIEVED_EVENT_MAX_POSITION_ERROR) && \
            (velocity_error < ZMOTOR_SETPOINT_ACHIEVED_EVENT_MAX_VELOCITY_ERROR))
        {
            notifySetpointAchieved();
            g_newSetpoint = FALSE;
        }
    }
    float duty = 0.5f + velocity_control_output;
    
    setDuty(duty);
}

static void tachometerMeasurementCallback(float velocity)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    g_velocityMeasurement = velocity;
    g_velocityMeasurementReceived = TRUE;

    if (g_positionMeasurementReceived) 
    {
        g_velocityMeasurementReceived = FALSE;
        g_positionMeasurementReceived = FALSE;
        
        controlUpdate();
    }
}

static void lvdtMeasurementCallback(float position)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    g_positionMeasurement = position;
    g_positionMeasurementReceived = TRUE;

    if (g_velocityMeasurementReceived) 
    {
        g_velocityMeasurementReceived = FALSE;
        g_positionMeasurementReceived = FALSE;

        controlUpdate();
    }
}
/* ------------------ Public Functions ------------------ */
void ZMotorControl_Init(DAC_HandleTypeDef *lvdtDac, 
                        TIM_HandleTypeDef *lvdtHtim, 
                        uint32_t lvdtDacChannel)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }
    
    /* Initialize external measurement modules */
    Tachometer_Init();
    LVDT_Init(lvdtDac, lvdtHtim, lvdtDacChannel);
    
    Tachometer_RegisterCallback(tachometerMeasurementCallback);
    LVDT_RegisterCallback(lvdtMeasurementCallback);

    /* Initialize PID controllers. */
    PID_Init(&pidPosition, 
             ZMOTOR_POSITION_CONTROL_PID_GAIN, 
             ZMOTOR_POSITION_CONTROL_PID_INTEGRAL_TC, 
             ZMOTOR_POSITION_CONTROL_PID_DERIVATIVE_TC, 
             ZMOTOR_POSITION_CONTROL_PID_DT, 
             ZMOTOR_POSITION_CONTROL_PID_FILTER_TC, 
             ZMOTOR_POSITION_CONTROL_PID_OUTPUT_MIN,
             ZMOTOR_POSITION_CONTROL_PID_OUTPUT_MAX);
    
    PID_Init(&pidVelocity, 
             ZMOTOR_VELOCITY_CONTROL_PID_GAIN, 
             ZMOTOR_VELOCITY_CONTROL_PID_INTEGRAL_TC, 
             ZMOTOR_VELOCITY_CONTROL_PID_DERIVATIVE_TC, 
             ZMOTOR_VELOCITY_CONTROL_PID_DT, 
             ZMOTOR_VELOCITY_CONTROL_PID_FILTER_TC, 
             ZMOTOR_VELOCITY_CONTROL_PID_OUTPUT_MIN,
             ZMOTOR_VELOCITY_CONTROL_PID_OUTPUT_MAX);
             
    g_state = STATE_READY;
}


void ZMotorControl_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }
    
    Tachometer_Start();
    LVDT_Start();
    
    PID_Start(&pidPosition, 0.);
    PID_Start(&pidVelocity, 0.);

    if (HAL_TIM_PWM_Start(g_motorDrivePwmHtim, g_motorDriverPwmChannel) != HAL_OK) 
    {
        return;
    }
    
    g_newSetpoint = FALSE;
    g_state = STATE_OPERATING;
}

void ZMotorControl_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }
    
    Tachometer_Stop();
    LVDT_PhaseDetection_Stop();
    
    HAL_TIM_PWM_Stop(g_motorDrivePwmHtim, g_motorDriverPwmChannel);
    
    g_state = STATE_READY;
}

void ZMotorControl_SetPositionSetpoint(float position)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    g_positionSetpoint = position;
    g_newSetpoint = TRUE;
}

void ZMotorControl_RegisterCallback(ZMotorControl_Callback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    if (g_numCallbacks > ZMOTOR_CONTROL_MAX_NUM_OF_CALLBACKS) {
        return;
    }

    g_callbacks[g_numCallbacks++] = callback;
}