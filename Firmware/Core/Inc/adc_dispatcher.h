#ifndef ADC_DISPATCHER_H
#define ADC_DISPATCHER_H

#include "stm32f4xx_hal.h"  // Change if using a different series
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of ADC channels that can be registered.
 *        Increase as needed.
 */
#define ADC_DISPATCHER_MAX_CHANNELS   8

/**
 * @brief Callback type for when a new ADC sample is available for a specific channel.
 *
 * @param channelIndex   Index of the channel in the dispatcher’s internal list
 * @param value          The latest ADC sample (raw 12-bit for example)
 */
typedef void (*AdcCallback_t)(uint8_t channelIndex, uint16_t value);

/**
 * @brief Register a channel in the ADC dispatcher.
 *        Must be called before ADC_Dispatcher_Init() so all channels are known.
 *
 * @param adcChannel   e.g. ADC_CHANNEL_6, ADC_CHANNEL_7, ...
 * @param callback     Optional pointer to a user callback. Pass NULL if not needed.
 *
 * @return The index of the channel in the dispatcher, or -1 if it fails (no space).
 */
int8_t ADC_Dispatcher_RegisterChannel(uint32_t adcChannel, AdcCallback_t callback);

/**
 * @brief Initialize the ADC dispatcher (timer, ADC, DMA) once all channels are registered.
 *
 * @param hadc   Pointer to an ADC handle (the dispatcher will configure it).
 * @param htim   Pointer to the timer handle used for ADC triggers.
 * @param sampleFreqHz The desired sampling frequency (approx).
 */
void ADC_Dispatcher_Init(ADC_HandleTypeDef *hadc, 
                         TIM_HandleTypeDef *htim, 
                         uint32_t sampleFreqHz);

/**
 * @brief Start the ADC conversions with DMA in circular mode.
 */
void ADC_Dispatcher_Start(void);

/**
 * @brief Stop the ADC conversions.
 */
void ADC_Dispatcher_Stop(void);

/**
 * @brief Get the most recent sample for a particular registered channel index.
 *        (Useful if the user doesn’t want to deal with callbacks.)
 *
 * @param channelIndex Index returned by ADC_Dispatcher_RegisterChannel().
 * @return Latest 16-bit ADC value (raw).
 */
uint16_t ADC_Dispatcher_GetValue(uint8_t channelIndex);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DISPATCHER_H */
