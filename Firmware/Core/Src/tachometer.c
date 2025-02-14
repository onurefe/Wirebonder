#include "tachometer.h"
#include "adc_dispatcher.h"  // Our modular ADC dispatcher
#include "generic.h"
#include "configuration.h"
#include "adc_conversion_orders.h"

/* ------------------Private definitions ---------------- */
#define DOWNSAMPLING_FACTOR (ADC_DISPATCHER_NOTIFICATION_FREQUENCY / TACHOMETER_NOTIFICATION_FREQUENCY)

/* ----------------- Private Variables ----------------- */
static State_t g_state = STATE_UNINIT;

static uint8_t g_numCallbacks;
static Tachometer_MeasurementCallback_t g_measurementCallbacks[TACHOMETER_MAX_NUM_OF_CALLBACKS];

/* Working registers ---------------------------------------------*/
static uint16_t g_accumulatedSamples;
static uint32_t g_accumulator;

/* Private functions --------------------------------------------*/
static void notify(float velocity)
{
    /* Notify all registered callbacks with the new speed value. */
    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        if (g_measurementCallbacks[i] != 0)
        {
            g_measurementCallbacks[i](velocity);
        }
    }
}

static float computeVelocity(float rawMeasurementAverage)
{
    float measured_voltage = ADC_VOLTAGE_RANGE * rawMeasurementAverage / \
    (1 << ADC_BITS) ;
    
    float velocity = (measured_voltage - TACHOMETER_ZERO_VELOCITY_VOLTAGE) * \
    TACHOMETER_VOLTAGE_TO_RPM_CONVERSION_FACTOR;

    return velocity;
}

static void adcCallbackTachometer(uint16_t value)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_accumulator += value;
    g_accumulatedSamples++;

    if (g_accumulatedSamples >= DOWNSAMPLING_FACTOR)
    {
        notify(computeVelocity((float)g_accumulator / g_accumulatedSamples));
        
        g_accumulator = 0;
        g_accumulatedSamples = 0;
    }
}

/* Exported functions ------------------------------------------*/
void Tachometer_Init(void)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    AdcDispatcher_RegisterChannel(ZMOTOR_TACHOMETER_ADC2_CONVERSION_ORDER, 
                                  adcCallbackTachometer);

    g_numCallbacks = 0;

    g_state = STATE_READY;
}

void Tachometer_Start(void)
{
    if (g_state != STATE_READY)
    {
        // Only allowed to start when the module is ready.
        return;
    }

    g_accumulator = 0;
    g_accumulatedSamples = 0;

    g_state = STATE_OPERATING;
}


void Tachometer_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_state = STATE_READY;
}


void Tachometer_RegisterCallback(Tachometer_MeasurementCallback_t callback)
{
    if (g_numCallbacks < TACHOMETER_MAX_NUM_OF_CALLBACKS)
    {
        g_measurementCallbacks[g_numCallbacks++] = callback;
    }
}
