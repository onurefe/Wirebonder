#include "force_coil_driver.h"
#include "adc2_conversion_orders.h"
#include "pid_controller.h"
#include "adc_dispatcher.h"
#include "generic.h"
#include "configuration.h"
#include "math.h"

/* Private definitions --------------------------------------------*/
#define OVERSAMPLING_NUMBER (ADC_DISPATCHER_NOTIFICATION_FREQUENCY / FORCE_COIL_DRIVER_CONTROL_UPDATE_FREQUENCY)

/* ----------------- Variables ------------------------------------*/
static State_t g_state = STATE_UNINIT;

static ForceCoil_Callback_t g_callbacks[ZMOTOR_CONTROL_MAX_NUM_OF_CALLBACKS];
static uint8_t g_numCallbacks;

static TIM_HandleTypeDef *g_htim;
static uint32_t g_channel;

static PID_Controller g_pid;

static Bool_t g_newSetpoint;

static uint16_t g_accumulator;
static uint16_t g_accumulatedSamples;
static float g_currentSetpoint = 0.0f;

/* Private functions ----------------------------------------------*/
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
    duty = clip(duty, FORCE_COIL_MIN_DUTY , FORCE_COIL_MAX_DUTY);

    uint32_t period = g_htim->Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    __HAL_TIM_SET_COMPARE(g_htim, g_channel, pulse);
}

float computeCurrentFromRawAverage(float raw_average)
{
    float voltage = ADC_VOLTAGE_RANGE * raw_average / (1 << ADC_BITS);
    float current = voltage * FORCE_COIL_DRIVER_V2I_CONVERSION_FACTOR;
    return current;
}

static void notifySetpointAchieved(void)
{
    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        if (g_callbacks[i] != NULL)
        {
            g_callbacks[i](FORCE_COIL_SETPOINT_ACHIEVED_EVENT_ID);
        }
    }
}

void controlUpdate(float measuredCurrent)
{
    float controlOutput = PID_Execute(&g_pid, g_currentSetpoint, measuredCurrent);
    
    if (g_newSetpoint) {
        float error = fabsf(measuredCurrent - g_currentSetpoint);
        if (error < FORCE_COIL_DRIVER_SETPOINT_ACHIEVED_EVENT_MAX_CURRENT_ERROR) 
        {
            notifySetpointAchieved();
            g_newSetpoint = FALSE;
        }
    }

    setDuty(controlOutput);
}

static void adcCallback(uint16_t value)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    if (g_accumulatedSamples >= OVERSAMPLING_NUMBER)
    {        
        float raw_average = (float)g_accumulator / g_accumulatedSamples;
        
        g_accumulatedSamples = 0;
        g_accumulator = 0;

        controlUpdate(computeCurrentFromRawAverage(raw_average));
    }
}

/* ----------------- Public Functions ----------------- */
void ForceCoilDriver_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }
    
    PID_Init(&g_pid, 
             FORCE_COIL_DRIVER_PID_GAIN, 
             FORCE_COIL_DRIVER_PID_INTEGRAL_TC,
             FORCE_COIL_DRIVER_PID_DERIVATIVE_TC,
             (1.0 / FORCE_COIL_DRIVER_CONTROL_UPDATE_FREQUENCY),
             FORCE_COIL_DRIVER_PID_INPUT_FILTER_TC,
             FORCE_COIL_DRIVER_PID_OUTPUT_MIN,
             FORCE_COIL_DRIVER_PID_OUTPUT_MAX);

    AdcDispatcher_RegisterChannel(FORCE_COIL_ISENS_ADC2_CONVERSION_ORDER, 
                                  adcCallback);

    g_numCallbacks = 0;

    g_htim = htim;
    g_channel = channel;
    g_state = STATE_READY;
}

void ForceCoilDriver_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }
        
    if (HAL_TIM_PWM_Start(g_htim, g_channel) != HAL_OK)
    {
        while (TRUE);
    }
    
    g_currentSetpoint = FORCE_COIL_DRIVER_PID_OUTPUT_MIN;
    g_accumulator = 0;
    g_accumulatedSamples = 0;
    g_newSetpoint = FALSE;

    PID_Start(&g_pid, 0.0);

    setDuty(FORCE_COIL_DRIVER_PID_OUTPUT_MIN);

    g_state = STATE_OPERATING;
}

void ForceCoilDriver_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }
    
    HAL_TIM_PWM_Stop(&g_htim, g_channel);

    g_state = STATE_READY;
}


void ForceCoilDriver_SetCurrentSetpoint(float currentSetpoint)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_newSetpoint = TRUE;
    g_currentSetpoint = currentSetpoint;
}

void ForceCoilDriver_RegisterCallback(ForceCoil_Callback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    if (g_numCallbacks > FORCE_COIL_MAX_NUM_OF_CALLBACKS) {
        return;
    }

    g_callbacks[g_numCallbacks++] = callback;
}