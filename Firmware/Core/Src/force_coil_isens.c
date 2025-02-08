#include "force_coil_isens.h"
#include "adc_dispatcher.h"  // Our modular ADC dispatcher must provide ADC_CHANNEL_9, etc.
#include <stdint.h>
#include <stdbool.h>

/* ----------------- State Definitions ----------------- */
typedef enum {
    STATE_UNINIT = 0,
    STATE_READY,
    STATE_OPERATING
} CurrentState_t;

static CurrentState_t sCurrentState = STATE_UNINIT;

/* ----------------- Private Variables ----------------- */
// ADC channel index for the coil current measurement.
// For this example, we assume the sensor is connected to ADC_CHANNEL_9.
static int8_t sCurrentChannelIndex = -1;

// Latest ADC reading for the coil current.
static volatile uint16_t sLatestCurrentADC = 0;

/* ----------------- Forward Declarations ----------------- */
/**
 * @brief ADC callback for current measurement.
 *
 * This function is called by the ADC dispatcher whenever a new sample is available.
 */
static void adcCallbackCurrent(uint8_t channelIndex, uint16_t value)
{
    (void)channelIndex;  // Unused parameter
    sLatestCurrentADC = value;
}

/* ----------------- Public Functions ----------------- */

void ForceCoilIsens_Init(void)
{
    if (sCurrentState != STATE_UNINIT)
    {
        // Already initialized; ignore repeated calls.
        return;
    }
    
    /* Register the ADC channel for current measurement.
       In this example, we assume the current sensor is connected to ADC_CHANNEL_9.
       Replace ADC_CHANNEL_9 with the correct channel for your hardware.
    */
    sCurrentChannelIndex = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_9, adcCallbackCurrent);
    
    sLatestCurrentADC = 0;
    sCurrentState = STATE_READY;
}

void ForceCoilIsens_Start(void)
{
    if (sCurrentState != STATE_READY)
    {
        // Only start if the module is ready.
        return;
    }
    
    ADC_Dispatcher_Start();
    sCurrentState = STATE_OPERATING;
}

void ForceCoilIsens_Stop(void)
{
    if (sCurrentState != STATE_OPERATING)
    {
        // Only stop if the module is operating.
        return;
    }
    
    ADC_Dispatcher_Stop();
    sCurrentState = STATE_READY;
}

uint16_t ForceCoilIsens_GetValue(void)
{
    return sLatestCurrentADC;
}
