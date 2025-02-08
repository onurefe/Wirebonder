#ifndef FORCE_COIL_ISENS_H
#define FORCE_COIL_ISENS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initialize the current measurement module.
 *
 * Configures the ADC channel for reading the coil current.
 */
void ForceCoilIsens_Init(void);

/**
 * @brief Start current measurement.
 *
 * Starts the ADC dispatcher so that current measurements are updated.
 */
void ForceCoilIsens_Start(void);

/**
 * @brief Stop current measurement.
 *
 * Stops the ADC dispatcher.
 */
void ForceCoilIsens_Stop(void);

/**
 * @brief Get the latest coil current reading.
 *
 * @return The latest measured ADC value for the coil current.
 */
uint16_t ForceCoilIsens_GetValue(void);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_COIL_ISENS_H */
