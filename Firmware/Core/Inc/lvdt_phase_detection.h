#ifndef LVDT_PHASE_DETECTION_H
#define LVDT_PHASE_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Phase callback type.
 *
 * This function pointer type defines the callback that is invoked when a new
 * phase difference is available. The callback receives the computed phase
 * difference (in radians, wrapped to the range –π to +π).
 */
typedef void (*LVDT_PhaseCallback)(float phaseDifference);

/**
 * @brief Initializes the LVDT phase detection module.
 *
 * This function registers the ADC channels for channels A and B.
 */
void LVDT_PhaseDetection_Init(void);

/**
 * @brief Starts the phase detection.
 *
 * This function starts the ADC dispatcher.
 */
void LVDT_PhaseDetection_Start(void);

/**
 * @brief Stops the phase detection.
 *
 * This function stops the ADC dispatcher.
 */
void LVDT_PhaseDetection_Stop(void);

/**
 * @brief Returns the latest computed phase difference.
 *
 * The phase difference is computed as (phaseA – phaseB) and is wrapped to the
 * range –π to +π.
 *
 * @return The phase difference in radians.
 */
float LVDT_GetPhaseDifference(void);

/**
 * @brief Registers a phase callback.
 *
 * Multiple callbacks can be registered. When a new phase difference is computed,
 * all registered callbacks will be called.
 *
 * @param callback A pointer to a function conforming to LVDT_PhaseCallback.
 */
void LVDT_RegisterPhaseCallback(LVDT_PhaseCallback callback);

#ifdef __cplusplus
}
#endif

#endif /* LVDT_PHASE_DETECTION_H */
