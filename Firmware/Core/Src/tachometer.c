#include "tachometer.h"
#include "adc_dispatcher.h"  // Our modular ADC dispatcher
#include <stdint.h>
#include <stdbool.h>

/* ---------------- Definitions & Variables -------------- */

/**
 * Downsampling factor. The tachometer ADC measurements come at 4000 Hz.
 * By averaging DOWNSAMPLE_FACTOR samples, the effective rate is reduced to
 * 4000 / DOWNSAMPLE_FACTOR.
 */
#define DOWNSAMPLE_FACTOR  16

/**
 * Scaling factor to convert the averaged ADC value into speed.
 * For example, if the ADC value is proportional to voltage and the voltage
 * is proportional to speed, this factor converts the raw average to a speed unit.
 * Adjust this value as needed.
 */
#define TACHO_SPEED_SCALE  (0.1f)

// ADC channel index for the tachometer measurement.
// For this example, we assume the tachometer signal is connected to ADC_CHANNEL_8.
static int8_t sTachoChannelIndex = -1;

// Accumulator and counter used for downsampling (averaging).
static uint32_t sAccumulatedValue = 0;
static uint16_t sSampleCount = 0;

// Latest computed speed (after conversion).
static float sLatestSpeed = 0.0f;

// Maximum number of registered speed callback functions.
#define MAX_SPEED_CALLBACKS 8
static TACHO_SpeedCallback sSpeedCallbacks[MAX_SPEED_CALLBACKS];
static uint8_t sNumSpeedCallbacks = 0;

/* ----------------- State Definitions ----------------- */
typedef enum {
    TACHO_STATE_UNINIT = 0,
    TACHO_STATE_READY,
    TACHO_STATE_OPERATING
} TACHO_State_t;

static TACHO_State_t sTachoState = TACHO_STATE_UNINIT;

/* ---------------- Forward Declaration ----------------- */
static void adcCallbackTachometer(uint8_t channelIndex, uint16_t value);

/*-----------------------------------------------------------
  Public Functions
-----------------------------------------------------------*/

/**
 * @brief Initialize the tachometer module.
 *
 * Registers the tachometer ADC channel with its callback.
 * This function is allowed only when the module is uninitialized.
 */
void Tachometer_Init(void)
{
    if (sTachoState != TACHO_STATE_UNINIT)
    {
        // Already initialized; ignore repeated calls.
        return;
    }

    /* Register the tachometer ADC channel.
       Here we assume the tachometer signal is on ADC_CHANNEL_8.
       (Replace ADC_CHANNEL_8 with the correct channel for your hardware.) */
    sTachoChannelIndex = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_8, adcCallbackTachometer);
    sAccumulatedValue = 0;
    sSampleCount = 0;
    sLatestSpeed = 0.0f;
    sNumSpeedCallbacks = 0;  // Clear any previous registrations.

    sTachoState = TACHO_STATE_READY;
}

/**
 * @brief Start the tachometer measurement.
 *
 * Resets the accumulator and sample counter and starts the ADC dispatcher.
 * This function is allowed only when the module is in READY state.
 */
void Tachometer_Start(void)
{
    if (sTachoState != TACHO_STATE_READY)
    {
        // Only allowed to start when the module is ready.
        return;
    }

    sAccumulatedValue = 0;
    sSampleCount = 0;
    ADC_Dispatcher_Start();
    sTachoState = TACHO_STATE_OPERATING;
}

/**
 * @brief Stop the tachometer measurement.
 *
 * Stops the ADC dispatcher.
 * This function is allowed only when the module is in OPERATING state.
 */
void Tachometer_Stop(void)
{
    if (sTachoState != TACHO_STATE_OPERATING)
    {
        // Only allowed to stop when the module is operating.
        return;
    }

    ADC_Dispatcher_Stop();
    sTachoState = TACHO_STATE_READY;
}

/**
 * @brief Register a speed callback.
 *
 * Multiple callbacks can be registered. When a new downsampled speed measurement
 * is ready, all registered callbacks are notified.
 *
 * @param callback A pointer to a function conforming to TACHO_SpeedCallback.
 */
void Tachometer_RegisterSpeedCallback(TACHO_SpeedCallback callback)
{
    if (callback != 0 && sNumSpeedCallbacks < MAX_SPEED_CALLBACKS)
    {
        sSpeedCallbacks[sNumSpeedCallbacks++] = callback;
    }
}

/**
 * @brief Get the latest computed speed.
 *
 * @return The latest computed speed.
 */
float Tachometer_GetLatestSpeed(void)
{
    return sLatestSpeed;
}

/*-----------------------------------------------------------
  ADC Callback Implementation
-----------------------------------------------------------*/

/**
 * @brief ADC callback for the tachometer channel.
 *
 * This callback is invoked at 4000 Hz by the ADC dispatcher. It accumulates
 * ADC samples until DOWNSAMPLE_FACTOR samples have been received, computes
 * their average, converts the average to a speed value, and then notifies all
 * registered callbacks.
 */
static void adcCallbackTachometer(uint8_t channelIndex, uint16_t value)
{
    (void)channelIndex;  // Unused parameter

    // Process samples only if the module is operating.
    if (sTachoState != TACHO_STATE_OPERATING)
    {
        return;
    }

    sAccumulatedValue += value;
    sSampleCount++;

    if (sSampleCount >= DOWNSAMPLE_FACTOR)
    {
        /* Compute the average ADC value over the downsample factor. */
        float avgValue = (float)sAccumulatedValue / sSampleCount;
        
        /* Convert the average ADC value into a speed value using the scaling factor. */
        float speed = avgValue * TACHO_SPEED_SCALE;
        
        /* Update the latest computed speed. */
        sLatestSpeed = speed;
        
        /* Notify all registered callbacks with the new speed value. */
        for (uint8_t i = 0; i < sNumSpeedCallbacks; i++)
        {
            if (sSpeedCallbacks[i] != 0)
            {
                sSpeedCallbacks[i](speed);
            }
        }
        
        /* Reset the accumulator and counter for the next downsample period. */
        sAccumulatedValue = 0;
        sSampleCount = 0;
    }
}
