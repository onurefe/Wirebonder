#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "generic.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define NULL if it isn't already defined */
#ifndef NULL
#define NULL ((void *)0)
#endif

typedef struct {
    /* PID parameters (new parameterization) */
    float gain;            /**< Overall gain (proportional action) */
    float integral_tc;     /**< Integral time constant (T_i). If zero, no integral action is used. */
    float derivative_tc;   /**< Derivative time constant (T_d) */
    float dt;              /**< Time step (in seconds) */
    float setpoint;        /**< Desired target value */

    /* Internal PID states */
    float integral;        /**< Integral accumulator */
    float prev_error;      /**< Previous filtered error (for derivative calculation) */
    float output;          /**< Last computed output */

    /* EMA filter parameters applied to the error */
    float filter_time_constant; /**< Time constant (τ) for the EMA filter */
    float filter_alpha;         /**< Precomputed filter coefficient: dt / (τ + dt) */
    float filtered_error;       /**< Filtered error value */

    /* Output limiting parameters */
    float output_min;      /**< Minimum allowable output */
    float output_max;      /**< Maximum allowable output */

    /* Controller state */
    State_t state;     /**< Current state of the controller */
} Pid_Controller;


void Pid_Init(Pid_Controller *pid, float gain, float integral_tc, float derivative_tc, float dt,
              float filter_time_constant, float output_min, float output_max);
void Pid_Start(Pid_Controller *pid, float setpoint);
float Pid_Execute(Pid_Controller *pid, float setpoint, float measured_value);
void Pid_Stop(Pid_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
