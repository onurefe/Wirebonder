#ifndef PLL_H
#define PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initialize the PLL module.
 *
 * This function initializes the PLL PID controller and configures TIMER2.
 * It also registers the PLL callback with the impedance measurement module.
 */
void PLL_Init(void);

/**
 * @brief Start the PLL module.
 *
 * This function activates the PLL processing.
 */
void PLL_Start(void);

/**
 * @brief Stop the PLL module.
 *
 * This function stops the PLL processing.
 */
void PLL_Stop(void);

/**
 * @brief Activate the PLL with a specified center frequency.
 *
 * This function sets the desired (center) frequency. The PLL controller will
 * then add its computed frequency correction to this center frequency and update TIMER2.
 *
 * @param centerFrequency The desired center frequency (Hz) for TIMER2.
 */
void PLL_Activate(float centerFrequency);

#ifdef __cplusplus
}
#endif

#endif /* PLL_H */
