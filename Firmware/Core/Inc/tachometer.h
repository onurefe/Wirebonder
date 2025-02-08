#ifndef TACHOMETER_H
#define TACHOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Speed callback type.
 *
 * This function pointer type defines a callback that is invoked when a new
 * downsampled speed measurement is available. The callback receives the speed,
 * which is proportional to the tachometer voltage.
 */
typedef void (*TACHO_SpeedCallback)(float speed);

/**
 * @brief Initializes the tachometer module.
 *
 * This function registers the ADC channel used for the tachometer measurement.
 */
void Tachometer_Init(void);

/**
 * @brief Starts the tachometer measurement.
 *
 * This function resets the downsampling accumulator and starts the ADC dispatcher.
 */
void Tachometer_Start(void);

/**
 * @brief Stops the tachometer measurement.
 *
 * This function stops the ADC dispatcher.
 */
void Tachometer_Stop(void);

/**
 * @brief Registers a speed callback.
 *
 * Multiple callbacks can be registered. When a new downsampled speed measurement
 * is ready, all registered callbacks are notified.
 *
 * @param callback A pointer to a function conforming to TACHO_SpeedCallback.
 */
void Tachometer_RegisterSpeedCallback(TACHO_SpeedCallback callback);

/**
 * @brief Gets the latest computed speed.
 *
 * @return The latest computed speed.
 */
float Tachometer_GetLatestSpeed(void);

#ifdef __cplusplus
}
#endif

#endif /* TACHOMETER_H */
