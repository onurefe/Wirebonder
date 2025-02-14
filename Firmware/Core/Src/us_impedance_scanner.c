#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "stm32f4xx_hal.h"
#include "configuration.h"
#include "math.h"
#include "generic.h"
#include "adc_conversion_orders.h"
#include "complex.h"

/* Private definitions -----------------------------------*/
#define DAC_RANGE (1 << DAC_BITS)
#define US_IMPEDANCE_SCANNER_ADC_BUFFER_SIZE (2 * US_IMPEDANCE_SCANNER_ADC_SAMPLES)
#define US_IMPEDANCE_SCANNER_DAC_BUFFER_SIZE (US_IMPEDANCE_SCANNER_DAC_SAMPLES)

/* Private variables -------------------------------------*/
static uint16_t g_adcBuffer[US_IMPEDANCE_SCANNER_ADC_BUFFER_SIZE];
static uint16_t g_dacBuffer[US_IMPEDANCE_SCANNER_DAC_BUFFER_SIZE];

static State_t g_state = STATE_UNINIT;

static complexf *g_impedances;
static complexf *g_vPhasors;
static complexf *g_iPhasors;

static UsImpedanceScanner_Callback_t g_callback;

/* Private functions -------------------------------------*/
float fillDDSBuffer(uint16_t *dacBuffer)
{
    for (uint16_t n=0; n < US_IMPEDANCE_SCANNER_DAC_SAMPLES; n++)
    {
        float v = 0.;

        for (uint16_t i=0; i < US_IMPEDANCE_SCANNER_NUM_FREQUENCIES; i++)
        {
            float f = US_IMPEDANCE_SCANNER_MIN_FREQUENCY + i * US_IMPEDANCE_SCANNER_FREQUENCY_STEP;
            float t = (float)n / DAC1_SAMPLING_FREQUENCY;
            float phase = 2.0 * M_PI * f * t;

            v += sinf(phase);
        }

        v = 0.5 * (1. + v / ((float)US_IMPEDANCE_SCANNER_NUM_FREQUENCIES));
        dacBuffer[n] = (uint16_t)(v * DAC_RANGE);
    }
}

void computePhasors(uint16_t *adcBuffer, complexf *vPhasors, complexf *iPhasors)
{
    for (uint16_t i=0; i < US_IMPEDANCE_SCANNER_NUM_FREQUENCIES; i++)
    {
        vPhasors[i].re = 0.;
        vPhasors[i].im = 0.;
        iPhasors[i].re = 0.;
        iPhasors[i].im = 0.;

        for (uint16_t n=0; n < US_IMPEDANCE_SCANNER_ADC_SAMPLES; n++)
        {
            float f = US_IMPEDANCE_SCANNER_MIN_FREQUENCY + i * US_IMPEDANCE_SCANNER_FREQUENCY_STEP;
            float t = (float)n / ADC1_SAMPLING_FREQUENCY;

            float sin_phase = sinf(2.0 * M_PI * f * t);
            float cos_phase = cosf(2.0 * M_PI * f * t);

            vPhasors[i].re += (float)adcBuffer[2*n + VSENS_CONVERSION_ORDER] * sin_phase;
            vPhasors[i].im += (float)adcBuffer[2*n + VSENS_CONVERSION_ORDER] * cos_phase;
            
            iPhasors[i].re += (float)adcBuffer[2*n + ISENS_CONVERSION_ORDER] * sin_phase;
            iPhasors[i].im += (float)adcBuffer[2*n + ISENS_CONVERSION_ORDER] * cos_phase;
        }
    }
}

void computeImpedances(complexf *vPhasors, complexf *iPhasors, complexf *impedances)
{
    for (uint16_t i = 0; i < US_IMPEDANCE_SCANNER_NUM_FREQUENCIES; i++)
    {
        impedances[i] = complexf_div(vPhasors[i], iPhasors[i]);
    }   
}

void ultrasonicDriverNotificationCallback(uint16_t eventId, uint16_t *buffer)
{
    if (eventId == US_DRIVER_OPERATION_COMPLETED_EVENT_ID) 
    {
        computePhasors(g_adcBuffer, g_vPhasors, g_iPhasors);
        computeImpedances(g_vPhasors, g_iPhasors, g_impedances);
        
        if (g_callback) 
        {
            g_callback(g_vPhasors, g_iPhasors, g_impedances);
        }
        
        g_state = STATE_READY;
    }
}

/* Public functions --------------------------------------*/
void UsImpedanceScanner_Init(void)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    g_state = STATE_READY;
}

void UsImpedanceScanner_Start(complexf *voltagePhasors, 
                              complexf *currentPhasors, 
                              complexf *impedances, 
                              UsImpedanceScanner_Callback_t callback)
{
    if (g_state != STATE_READY) 
    {
        return;
    }

    /* Start working registers. */
    g_vPhasors = voltagePhasors;
    g_iPhasors = currentPhasors;
    g_impedances = impedances;
    g_callback = callback;

    /* Fill DDS buffer. */
    fillDDSBuffer(g_dacBuffer);    

    /* Start ultrasonic driver. */
    UltrasonicDriver_Start(g_dacBuffer, 
                           g_adcBuffer, 
                           US_IMPEDANCE_SCANNER_DAC_BUFFER_SIZE, 
                           US_IMPEDANCE_SCANNER_ADC_BUFFER_SIZE, 
                           FALSE, 
                           ultrasonicDriverNotificationCallback);

    g_state = STATE_OPERATING;
}

void UsImpedanceScanner_Stop(void)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    UltrasonicDriver_Stop();
    g_state = STATE_READY;
}
