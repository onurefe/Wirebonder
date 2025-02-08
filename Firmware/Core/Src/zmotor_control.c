#include "zmotor_control.h"
#include "tachometer.h"         // Provides Tachometer_GetLatestSpeed(), Tachometer_Start(), etc.
#include "lvdt_phase_detection.h" // Provides LVDT_GetPhaseDifference(), LVDT_PhaseDetection_Start(), etc.
#include "pid_controller.h"      // Provides PID_Init(), PID_Start(), PID_Execute(), etc.
#include "stm32f4xx_hal.h"       // HAL drivers for PWM and timer (adjust as needed)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ------------------ Module State Definitions ------------------ */
typedef enum {
    MOTOR_STATE_UNINIT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_OPERATING
} MotorState_t;

static MotorState_t sMotorState = MOTOR_STATE_UNINIT;

/* ------------------ Global Variables ------------------ */
static float sPositionSetpoint = 0.0f;   // Desired position

/* PID controllers for position and speed */
static PID_Controller pidPosition;
static PID_Controller pidSpeed;

/* PWM output: using TIM2 Channel 1 for motor drive.
   The PWM duty cycle will be computed such that:
     - 0.5 (50%) means zero speed (no motion),
     - >0.5 means positive (forward) motion,
     - <0.5 means negative (reverse) motion.
 */
static TIM_HandleTypeDef htim2;  // PWM timer handle

/* ------------------ Forward Declarations ------------------ */
static void MotorControl_PWMInit(void);
static void MotorControl_PWMSetDuty(float duty);

/* ------------------ Public Functions ------------------ */

/**
 * @brief Initialize the motor control module.
 *
 * This function initializes the tachometer and LVDT modules,
 * configures the PID controllers for position and speed,
 * and initializes the PWM output.
 */
void MotorControl_Init(void)
{
    if (sMotorState != MOTOR_STATE_UNINIT)
    {
        // Already initialized.
        return;
    }
    
    /* Initialize external measurement modules */
    Tachometer_Init();
    LVDT_PhaseDetection_Init();
    
    /* Initialize the PID controllers.
       The following parameters are examples and must be tuned for your system.
       For the position PID controller, assume:
         - gain = 1.0,
         - integral time constant = 0.5 s,
         - derivative time constant = 0.1 s,
         - dt = 1 ms (0.001 s),
         - initial setpoint = 0,
         - output limits = [-1.0, 1.0].
       For the speed PID controller, assume:
         - gain = 1.0,
         - integral time constant = 0.3 s,
         - derivative time constant = 0.05 s,
         - dt = 1 ms,
         - initial setpoint = 0,
         - output limits = [-1.0, 1.0].
    */
    PID_Init(&pidPosition, 1.0f, 0.5f, 0.1f, 0.001f, 0.0f, 0.05f, -1.0f, 1.0f);
    PID_Init(&pidSpeed,    1.0f, 0.3f, 0.05f, 0.001f, 0.0f, 0.05f, -1.0f, 1.0f);
    PID_Start(&pidPosition);
    PID_Start(&pidSpeed);
    
    /* Initialize PWM output */
    MotorControl_PWMInit();
    
    sMotorState = MOTOR_STATE_READY;
}

/**
 * @brief Start the motor control loop.
 *
 * This function starts the tachometer and LVDT modules and begins PWM output.
 */
void MotorControl_Start(void)
{
    if (sMotorState != MOTOR_STATE_READY)
    {
        return;
    }
    
    Tachometer_Start();
    LVDT_PhaseDetection_Start();
    
    /* Start the PWM output */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        // Error handling: if PWM fails to start, consider error recovery.
        return;
    }
    
    sMotorState = MOTOR_STATE_OPERATING;
}

/**
 * @brief Stop the motor control loop.
 *
 * This function stops the measurement modules and PWM output.
 */
void MotorControl_Stop(void)
{
    if (sMotorState != MOTOR_STATE_OPERATING)
    {
        return;
    }
    
    Tachometer_Stop();
    LVDT_PhaseDetection_Stop();
    
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    
    sMotorState = MOTOR_STATE_READY;
}

/**
 * @brief Set the desired position setpoint.
 *
 * @param pos Desired position.
 */
void MotorControl_SetPositionSetpoint(float pos)
{
    sPositionSetpoint = pos;
}

/**
 * @brief Update the motor control loop.
 *
 * This function reads the measured position and speed, computes control commands
 * via the PID controllers, and updates the PWM duty cycle accordingly.
 * It should be called periodically (e.g., every 1 ms).
 */
void MotorControl_Update(void)
{
    if (sMotorState != MOTOR_STATE_OPERATING)
    {
        return;
    }
    
    /* Read measured position from the LVDT module.
       In this example, LVDT_GetPhaseDifference() returns a value proportional to position.
    */
    float measuredPosition = LVDT_GetPhaseDifference();
    
    /* Execute the position PID.
       The output is a desired speed setpoint.
    */
    float desiredSpeed = PID_Execute(&pidPosition, sPositionSetpoint, measuredPosition);
    
    /* Read the measured speed from the tachometer module */
    float measuredSpeed = Tachometer_GetLatestSpeed();
    
    /* Execute the speed PID.
       The output is a control signal that we map to PWM duty cycle.
    */
    float speedControlOutput = PID_Execute(&pidSpeed, desiredSpeed, measuredSpeed);
    
    /* Map the speed control output to a PWM duty cycle.
       We define:
         duty = 0.5 + (speedControlOutput),
       so that when speedControlOutput is 0 the duty is 50% (no motion).
       Limit duty to [0, 1].
    */
    float duty = 0.5f + speedControlOutput;
    if (duty > 1.0f)
        duty = 1.0f;
    else if (duty < 0.0f)
        duty = 0.0f;
    
    MotorControl_PWMSetDuty(duty);
}

/* ------------------ Private Functions for PWM Output ------------------ */

/**
 * @brief Initialize the PWM timer for motor control.
 *
 * In this example, we configure TIM2 Channel 1 for PWM output.
 * The PWM frequency is set to 20 kHz.
 */
static void MotorControl_PWMInit(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    /* Example configuration:
       Assume the timer clock is 84 MHz. To obtain a PWM frequency of 20 kHz:
       Choose PSC and ARR such that: 84e6 / ((PSC+1)*(ARR+1)) = 20e3.
       For instance, PSC = 83 and ARR = 49 yield: 84e6/(84*50)=20,000 Hz.
    */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 49;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        // Error handling.
    }
    
    /* Configure PWM channel 1 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    /* Start with a 50% duty cycle (neutral command) */
    sConfigOC.Pulse = (htim2.Init.Period + 1) / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        // Error handling.
    }
}

/**
 * @brief Update the PWM duty cycle.
 *
 * @param duty Duty cycle in the range [0, 1]. A duty of 0.5 corresponds to zero motor movement.
 */
static void MotorControl_PWMSetDuty(float duty)
{
    uint32_t period = htim2.Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * period);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
