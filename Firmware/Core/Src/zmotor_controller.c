#include "zmotor_controller.h"
#include "tachometer.h"
#include "timer_expire.h"       
#include "lvdt.h"               
#include "pid_controller.h"      
#include "stm32f4xx_hal.h"
#include "generic.h"
#include "configuration.h"
#include "math.h"

static State_t g_state = STATE_UNINIT;

static uint8_t g_numCallbacks;
static ZMotorController_Callback_t g_callbacks[ZMOTOR_CONTROLLER_MAX_NUM_OF_CALLBACKS];

static Pid_Controller pidPosition;
static Pid_Controller pidVelocity;

static float g_positionSetpoint = 0.0f;

static TIM_HandleTypeDef *g_motorDrivePwmHtim;
static uint32_t g_motorDriverPwmChannel;

static Bool_t g_velocityMeasurementReceived;
static Bool_t g_positionMeasurementReceived;
static Bool_t g_newSetpoint;

static TimerExpire_Handle_t g_settlingTimer;

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
    duty = clip(duty, ZMOTOR_CONTROLLER_MIN_DUTY , ZMOTOR_CONTROLLER_MAX_DUTY);

    uint32_t period = g_motorDrivePwmHtim->Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    
    __HAL_TIM_SET_COMPARE(g_motorDrivePwmHtim, g_motorDriverPwmChannel, pulse);
}

static void notifySetpointAchieved(void)
{
    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        g_callbacks[i](ZMOTOR_CONTROLLER_SETPOINT_ACHIEVED_EVENT_ID);
    }
}

static void notifySettlingError(void)
{
    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        g_callbacks[i](ZMOTOR_CONTROLLER_UNABLE_SET_POSITION_EVENT_ID);
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
    
    float duty = ZMOTOR_ZERO_VELOCITY_DUTY + velocity_control_output;
    
    setDuty(duty);
}

static void settlingTimerCallback(void *handle)
{
    (void)handle;

    if (g_newSetpoint) 
    {
        notifySettlingError();
        setDuty(ZMOTOR_ZERO_VELOCITY_DUTY);
        g_newSetpoint = FALSE;
    }
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
void ZMotorController_Init(DAC_HandleTypeDef *lvdtDac, 
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

    TimerExpire_Register(&g_settlingTimer, TRUE, ZMOTOR_SETTLING_TIME_WINDOW, settlingTimerCallback);

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


void ZMotorController_Start(float positionSetpoint)
{
    if (g_state != STATE_READY)
    {
        return;
    }
    
    Tachometer_Start();
    LVDT_Start();
    
    g_positionSetpoint = positionSetpoint;
    
    PID_Start(&pidPosition, positionSetpoint);
    PID_Start(&pidVelocity, 0.);

    if (HAL_TIM_PWM_Start(g_motorDrivePwmHtim, g_motorDriverPwmChannel) != HAL_OK) 
    {
        return;
    }
    
    setDuty(ZMOTOR_ZERO_VELOCITY_DUTY);

    g_newSetpoint = FALSE;
    g_state = STATE_OPERATING;
}

void ZMotorController_Stop(void)
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

void ZMotorController_SetPositionSetpoint(float position)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    TimerExpire_Activate(&g_settlingTimer);
    g_positionSetpoint = position;
    g_newSetpoint = TRUE;
}

void ZMotorController_RegisterCallback(ZMotorController_Callback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    if (g_numCallbacks > ZMOTOR_CONTROLLER_MAX_NUM_OF_CALLBACKS) {
        return;
    }

    g_callbacks[g_numCallbacks++] = callback;
}