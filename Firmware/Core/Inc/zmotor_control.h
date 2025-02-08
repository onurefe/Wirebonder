#ifndef ZMOTOR_CONTROL_H
#define ZMOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initialize the motor control module.
 *
 * This module integrates:
 *   - Tachometer module for speed measurement.
 *   - LVDT module for position measurement.
 *   - Two PID controllers (for position and speed).
 *   - PWM output for motor drive.
 *
 * Call this function once before using the module.
 */
void MotorControl_Init(void);

/**
 * @brief Start the motor control loop.
 *
 * This function starts the integrated measurement modules and PWM output.
 */
void MotorControl_Start(void);

/**
 * @brief Stop the motor control loop.
 *
 * This function stops the measurement modules and PWM output.
 */
void MotorControl_Stop(void);

/**
 * @brief Set the desired position setpoint.
 *
 * @param pos Desired position (in units consistent with the LVDT module).
 */
void MotorControl_SetPositionSetpoint(float pos);

/**
 * @brief Run one cycle of the motor control update loop.
 *
 * This function should be called periodically (e.g., from a timer interrupt or main loop)
 * to update the control loop.
 */
void MotorControl_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* ZMOTOR_CONTROL_H */
