#include "pid_controller.h"
#include "generic.h"

void PID_Init(PID_Controller *pid, float gain, float integral_tc, float derivative_tc, float dt,
              float filter_time_constant, float output_min, float output_max)
{
    if (pid == NULL) 
    {
        return;
    }
    
    if (pid->state != STATE_UNINIT) 
    {
        return;
    }

    /* Initialize new PID parameters */
    pid->gain = gain;
    pid->integral_tc = integral_tc;
    pid->derivative_tc = derivative_tc;
    pid->dt = dt;
    
    /* Set output limiting parameters */
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->filter_time_constant = filter_time_constant;
    
    /* Initialize EMA filter parameters for error */
    if (filter_time_constant > 0.0f) {
        pid->filter_alpha = dt / (filter_time_constant + dt);
    } else {
        /* If time constant is zero or negative, bypass filtering (α = 1) */
        pid->filter_alpha = 1.0f;
    }

    /* Set controller state to READY */
    pid->state = STATE_READY;
}

void PID_Start(PID_Controller *pid, float setpoint)
{
    if (pid == NULL) 
    {
        return;
    }
    
    if (pid->state != STATE_READY) 
    {
        return;
    }

    pid->setpoint = setpoint;
    
    /* Reset internal PID variables */
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    
    /* Reset the filtered error state */
    pid->filtered_error = 0.0f;
    
    /* Set state to OPERATING */
    pid->state = STATE_OPERATING;
}

float PID_Execute(PID_Controller *pid, float setpoint, float measured_value)
{
    if (pid == NULL) 
    {
        return 0.0;
    }
    
    if (pid->state != STATE_OPERATING) 
    {
        return 0.0;
    }

    /* Update the setpoint (it can change on the fly) */
    pid->setpoint = setpoint;
    
    /* Compute the raw error */
    float raw_error = setpoint - measured_value;
    
    /* Apply the EMA filter to the error */
    pid->filtered_error = pid->filter_alpha * raw_error +
                          (1.0f - pid->filter_alpha) * pid->filtered_error;
    
    /* Compute derivative of the filtered error */
    float derivative = (pid->filtered_error - pid->prev_error) / pid->dt;
    
    /* Update the integral term */
    pid->integral += pid->filtered_error * pid->dt;
    
    /* Compute the PID output using the new parameterization:
       output = gain * [ filtered_error + (1/integral_tc)*integral + derivative_tc * derivative ]
       Note: If integral_tc is 0, the integral term is disabled.
    */
    float integral_term = (pid->integral_tc > 0.0f) ? (pid->integral / pid->integral_tc) : 0.0f;
    float derivative_term = pid->derivative_tc * derivative;
    
    pid->output = pid->gain * (pid->filtered_error + integral_term + derivative_term);
    
    /* Saturate the output */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    /* Save the current filtered error for the next derivative calculation */
    pid->prev_error = pid->filtered_error;
    
    return pid->output;
}

void PID_Stop(PID_Controller *pid)
{
    if (pid == NULL) 
    {
        return;
    }
    
    if (pid->state != STATE_OPERATING) 
    {
        return;
    }
    
    /* Set state to READY to indicate the controller is stopped */
    pid->state = STATE_READY;
}
