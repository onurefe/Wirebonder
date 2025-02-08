#ifndef IMPEDANCE_MEASUREMENT_H
#define IMPEDANCE_MEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Structure representing a complex impedance.
 *
 * - magnitude: The ratio of voltage amplitude to current amplitude.
 * - phase: The phase difference (in radians) between voltage and current, in the range [-π, π].
 */
typedef struct {
    float magnitude;
    float phase;
} ComplexImpedance;

/**
 * @brief Callback type for complex impedance measurement.
 *
 * When a downsampled measurement is ready, the registered callback(s) are invoked
 * with a ComplexImpedance structure.
 *
 * @param impedance The measured complex impedance.
 */
typedef void (*ImpedanceMeasurementCallback)(ComplexImpedance impedance);

/**
 * @brief Initialize the complex impedance measurement module.
 *
 * This function initializes internal buffers and state.
 */
void ImpedanceMeasurement_Init(void);

/**
 * @brief Start the impedance measurement.
 *
 * After calling this, the module will process DMA callback data.
 * The module must be in READY state before starting.
 */
void ImpedanceMeasurement_Start(void);

/**
 * @brief Stop the impedance measurement.
 *
 * The module will ignore any further DMA callback data until started again.
 */
void ImpedanceMeasurement_Stop(void);

/**
 * @brief Register a callback to receive impedance measurements.
 *
 * Multiple callbacks can be registered.
 *
 * @param callback Function pointer to be called when a new impedance measurement is ready.
 */
void ImpedanceMeasurement_RegisterCallback(ImpedanceMeasurementCallback callback);

/**
 * @brief DMA half-transfer callback.
 *
 * This function should be called by the DMA half-transfer interrupt.
 * @param buffer Pointer to the first half of the DMA buffer.
 * @param length Number of uint16_t elements in this half-buffer.
 */
void ImpedanceMeasurement_DMAHalfCallback(uint16_t *buffer, uint16_t length);

/**
 * @brief DMA transfer complete callback.
 *
 * This function should be called by the DMA transfer complete interrupt.
 * @param buffer Pointer to the second half (or full DMA buffer as configured) of the DMA buffer.
 * @param length Number of uint16_t elements in this buffer.
 */
void ImpedanceMeasurement_DMAFullCallback(uint16_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* IMPEDANCE_MEASUREMENT_H */
