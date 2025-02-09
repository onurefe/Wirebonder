#include "force_coil_driver.h"
#include "adc2_conversion_orders.h"
#include "pid_controller.h"
#include "adc_dispatcher.h"
#include "generic.h"
#include "configuration.h"

/* ----------------- Variables ------------------------------------*/
static State_t g_state = STATE_UNINIT;
static TIM_HandleTypeDef *g_htim;
static uint32_t g_channel;
static PID_Controller g_pid;
static float g_setpoint = 0.0f;

/* Private functions ----------------------------------------------*/
static void setDuty(float duty)
{
    uint32_t period = g_htim->Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    __HAL_TIM_SET_COMPARE(g_htim, g_channel, pulse);
}

void controlLoopExecute(uint16_t value)
{
    float measured = (float)value * FORCE_COIL_DRIVER_ADC2I_CONVERSION_FACTOR;
    float controlOutput = PID_Execute(&g_pid, g_setpoint, measured);
    
    // Since control is unidirectional, clamp negative outputs to zero.
    if (controlOutput < 0.0f)
    {
        controlOutput = 0.0f;
    }
    
    setDuty(controlOutput);
}

static void adcCallback(uint16_t value)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    controlLoopExecute(value);
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
             (1.0 / FORCE_COIL_DRIVER_CONTROL_ISR_FREQUENCY),
             FORCE_COIL_DRIVER_PID_INPUT_FILTER_TC,
             FORCE_COIL_DRIVER_PID_OUTPUT_MIN,
             FORCE_COIL_DRIVER_PID_OUTPUT_MAX);

    PID_Start(&g_pid, FORCE_COIL_DRIVER_PID_OUTPUT_MIN);
    
    AdcDispatcher_RegisterChannel(FORCE_COIL_ISENS_ADC2_CONVERSION_ORDER, 
                                  (ADC_DISPATCHER_MEASUREMENT_FREQUENCY / FORCE_COIL_DRIVER_CONTROL_ISR_FREQUENCY), 
                                  adcCallback);

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
    
    g_setpoint = 0.0f;
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


void ForceCoilDriver_SetCurrentSetpoint(float setpoint)
{
    g_setpoint = setpoint;
}