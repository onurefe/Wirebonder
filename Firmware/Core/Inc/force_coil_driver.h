#ifndef FORCE_COIL_DRIVER_H
#define FORCE_COIL_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initialize the force coil current driver module.
 *
 * This module uses a PID controller to regulate the current in a force coil.
 * The coil is driven using a PWM output. A duty cycle of 0.0 produces zero current,
 * while higher duty cycles produce higher currents.
 *
 * This function must be called once before any other function.
 */
void ForceCoilDriver_Init(void);

/**
 * @brief Start the force coil driver.
 *
 * This function starts the PWM output (and any ADC measurements if implemented).
 */
void ForceCoilDriver_Start(void);

/**
 * @brief Stop the force coil driver.
 *
 * This function stops the PWM output.
 */
void ForceCoilDriver_Stop(void);

/**
 * @brief Set the desired current setpoint.
 *
 * @param setpoint The desired coil current (in units defined by your system).
 */
void ForceCoilDriver_SetCurrentSetpoint(float setpoint);

/**
 * @brief Update the force coil driver control loop.
 *
 * This function should be called periodically (e.g., every 1 ms) to:
 *   - Measure the coil current,
 *   - Run the PID controller,
 *   - Update the PWM duty cycle.
 */
void ForceCoilDriver_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_COIL_DRIVER_H */
