#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Define NULL if it isn't already defined */
#ifndef NULL
#define NULL ((void *)0)
#endif

/**
 * @brief Enumeration of the PID controller states.
 */
typedef enum {
    PID_STATE_UNINIT = 0,  /**< Controller is uninitialized */
    PID_STATE_READY,       /**< Controller is initialized and ready */
    PID_STATE_OPERATING    /**< Controller is currently operating */
} PID_State_t;

/**
 * @brief PID controller structure.
 *
 * This structure holds the parameters and internal state of the PID controller.
 * Instead of working with Kp, Ki, and Kd, the controller is parameterized in terms of:
 * - gain: overall proportional gain,
 * - integral_tc: the integral time constant (T_i),
 * - derivative_tc: the derivative time constant (T_d).
 *
 * The PID output is computed as:
 *
 *   output = gain * [ filtered_error + (1/integral_tc)*integral + (derivative_tc)*(derivative) ]
 *
 * where the error is filtered via a first-order exponential moving average (EMA) filter.
 */
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
    PID_State_t state;     /**< Current state of the controller */
} PID_Controller;

/**
 * @brief Initializes the PID controller.
 *
 * This function sets up the PID controller with the specified parameters, including the new
 * parametrization (gain, integral time constant, derivative time constant), time step, setpoint,
 * filter time constant (τ for the error filter), and output limits. The filter coefficient α is
 * computed once during initialization.
 *
 * @param pid                  Pointer to a PID_Controller instance.
 * @param gain                 Overall gain (proportional gain).
 * @param integral_tc          Integral time constant (T_i). Use 0 to disable integral action.
 * @param derivative_tc        Derivative time constant (T_d).
 * @param dt                   Time step (in seconds).
 * @param setpoint             The desired target value.
 * @param filter_time_constant The time constant (τ) for the EMA filter on the error.
 * @param output_min           Minimum allowable output.
 * @param output_max           Maximum allowable output.
 */
void PID_Init(PID_Controller *pid, float gain, float integral_tc, float derivative_tc, float dt, float setpoint,
              float filter_time_constant, float output_min, float output_max);

/**
 * @brief Starts the PID controller.
 *
 * Resets the integral, derivative, and error filter states and sets the controller state to OPERATING.
 *
 * @param pid       Pointer to a PID_Controller instance.
 */
void PID_Start(PID_Controller *pid);

/**
 * @brief Executes one PID control timestep.
 *
 * This function accepts a new setpoint and the current measured value. It computes the raw error,
 * filters it using an EMA filter, and then calculates the PID output using the new parameterization:
 *
 *   output = gain * [ filtered_error + (1/integral_tc)*integral + (derivative_tc)*(derivative) ]
 *
 * The computed output is then saturated within the specified limits.
 *
 * @param pid            Pointer to a PID_Controller instance.
 * @param setpoint       The desired target value for this timestep.
 * @param measured_value The current measured value.
 *
 * @return The saturated control output.
 */
float PID_Execute(PID_Controller *pid, float setpoint, float measured_value);

/**
 * @brief Stops the PID controller.
 *
 * Stops the controller by setting its state to READY.
 *
 * @param pid       Pointer to a PID_Controller instance.
 */
void PID_Stop(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
