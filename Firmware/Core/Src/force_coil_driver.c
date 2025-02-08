#include "force_coil_driver.h"
#include "pid_controller.h"      // Your PID controller library
#include "force_coil_isens.h"   // Integrated current measurement module
#include "stm32f4xx_hal.h"       // Adjust include for your target
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ----------------- Module State Definitions ----------------- */
typedef enum {
    FORCE_COIL_STATE_UNINIT = 0,
    FORCE_COIL_STATE_READY,
    FORCE_COIL_STATE_OPERATING
} ForceCoilState_t;

static ForceCoilState_t sForceCoilState = FORCE_COIL_STATE_UNINIT;

/* ----------------- Private Variables ----------------- */

/* PID controller for current control.
   Tune these parameters for slow dynamics. For example:
     Kp = 0.5, Ki = 0.1, Kd = 0.0, dt = 0.001 s,
     setpoint initially 0, and output limits = [0, 1].
*/
static PID_Controller pidCurrent;

/* PWM timer handle for driving the force coil.
   In this example, we use TIM3 Channel 1.
*/
static TIM_HandleTypeDef htim3;

/* Desired current setpoint (user-defined) */
static float sCurrentSetpoint = 0.0f;

/* Conversion factor to convert raw ADC reading (0..4095) to current units.
   For demonstration, we assume a full-scale ADC reading corresponds to 1.0 unit
   of current. Adjust this factor to your system.
*/
#define CURRENT_CONVERSION_FACTOR  (1.0f / 4095.0f)

/* ----------------- Private Function Prototypes ----------------- */
static void ForceCoil_PWMInit(void);
static void ForceCoil_PWMSetDuty(float duty);

/* ----------------- Public Functions ----------------- */

/**
 * @brief Initialize the force coil current driver module.
 *
 * This function initializes the PID controller for current control, configures
 * the PWM output, and leaves the module in READY state.
 */
void ForceCoilDriver_Init(void)
{
    if (sForceCoilState != FORCE_COIL_STATE_UNINIT)
    {
        // Already initialized – ignore repeated calls.
        return;
    }
    
    /* Initialize the PID controller for current control.
       Parameters here are example values; adjust as needed.
       PID_Init parameters (using a PID library that accepts Kp, Ki, Kd, dt,
       setpoint, filter_time_constant, output_min, output_max):
         - Kp = 0.5
         - Ki = 0.1
         - Kd = 0.0
         - dt = 0.001 s
         - initial setpoint = 0.0
         - filter_time_constant = 0.0 (or a small value, if applicable)
         - output limits: [0.0, 1.0]
    */
    PID_Init(&pidCurrent, 0.5f, 0.1f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 1.0f);
    PID_Start(&pidCurrent);
    
    /* Initialize the PWM output for driving the coil */
    ForceCoil_PWMInit();
    ForceCoilIsens_Init();

    sCurrentSetpoint = 0.0f;
    sForceCoilState = FORCE_COIL_STATE_READY;
}

/**
 * @brief Start the force coil driver.
 *
 * This function starts the ADC current measurement (via the integrated module)
 * and starts the PWM output.
 */
void ForceCoilDriver_Start(void)
{
    if (sForceCoilState != FORCE_COIL_STATE_READY)
    {
        return;
    }
    
    /* Start current measurement (assumes CurrentMeasurement module is used) */
    ForceCoilIsens_Start();
    
    /* Start the PWM output */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        // Error handling (e.g., log or set an error flag)
        return;
    }
    
    sForceCoilState = FORCE_COIL_STATE_OPERATING;
}

/**
 * @brief Stop the force coil driver.
 *
 * This function stops both the current measurement and PWM output.
 */
void ForceCoilDriver_Stop(void)
{
    if (sForceCoilState != FORCE_COIL_STATE_OPERATING)
    {
        return;
    }
    
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    ForceCoilIsens_Stop();
    
    sForceCoilState = FORCE_COIL_STATE_READY;
}

/**
 * @brief Set the desired coil current setpoint.
 *
 * @param setpoint The desired coil current (in the same units as produced by CURRENT_CONVERSION_FACTOR).
 */
void ForceCoilDriver_SetCurrentSetpoint(float setpoint)
{
    sCurrentSetpoint = setpoint;
}

/**
 * @brief Update the force coil driver control loop.
 *
 * This function reads the coil current (from the CurrentMeasurement module),
 * runs the PID controller, and updates the PWM duty cycle accordingly.
 */
void ForceCoilDriver_Update(void)
{
    if (sForceCoilState != FORCE_COIL_STATE_OPERATING)
    {
        return;
    }
    
    /* Get the latest raw ADC value from the current measurement module */
    uint16_t adcVal = ForceCoilIsens_GetValue();
    
    /* Convert the raw ADC value to a current measurement.
       Here, CURRENT_CONVERSION_FACTOR converts the ADC value into engineering units.
    */
    float measured = (float)adcVal * CURRENT_CONVERSION_FACTOR;
    
    /* Execute the PID controller.
       The PID controller's setpoint is the desired current, and the measured value is the coil current.
       The output is expected to be in the range [0, 1].
    */
    float controlOutput = PID_Execute(&pidCurrent, sCurrentSetpoint, measured);
    
    // Since control is unidirectional, clamp negative outputs to zero.
    if (controlOutput < 0.0f)
    {
        controlOutput = 0.0f;
    }
    
    /* Update the PWM duty cycle.
       The PWM duty cycle is directly set to the PID output.
       A duty of 0.0 corresponds to zero current; higher values produce higher current.
    */
    ForceCoil_PWMSetDuty(controlOutput);
}

/* ----------------- Private Functions ----------------- */

/**
 * @brief Initialize the PWM timer for force coil drive.
 *
 * In this example, TIM3 Channel 1 is used for PWM output.
 * The PWM frequency is set to 20 kHz.
 */
static void ForceCoil_PWMInit(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    /* Example configuration:
       Assume TIM3 clock is 84 MHz.
       To obtain a PWM frequency of 20 kHz:
       Choose PSC = 83 and ARR = 49, so that:
       84e6 / ((83+1)*(49+1)) = 84e6/(84*50) = 20,000 Hz.
    */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 49;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        // Error handling.
    }
    
    /* Configure PWM channel 1 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    // Start with 0% duty (no current).
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        // Error handling.
    }
}

/**
 * @brief Update the PWM duty cycle.
 *
 * @param duty Duty cycle in the range [0, 1].
 */
static void ForceCoil_PWMSetDuty(float duty)
{
    uint32_t period = htim3.Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}
