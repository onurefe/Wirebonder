#include "pll.h"
#include "pid_controller.h"
#include "ultrasonic_driver.h"
#include "stm32f4xx_hal.h"
#include "configuration.h"
#include "math.h"
#include "generic.h"
#include "adc_conversion_orders.h"

/* Private variables -------------------------------------*/
static const uint16_t g_sineLookup[PLL_SINE_LOOKUP_SIZE];

static uint16_t g_dacBuffer[PLL_ADC_DOUBLE_BUFFER_SIZE];
static uint16_t g_adcBuffer[PLL_DAC_DOUBLE_BUFFER_SIZE];

static State_t g_state = STATE_UNINIT;
static PID_Controller g_pid;

static float g_centerFrequency;
static float g_correctionFrequency;
static float g_ddsPhase;

/* Private functions -------------------------------------*/
float mean(uint16_t *rawSignal, uint16_t numSamples)
{
    uint32_t accumulator = 0;

    for (uint16_t i = 0; i < numSamples; i++)
    {
        accumulator += rawSignal[i];
    }

    float mean = (float)accumulator / numSamples;
    return mean;
}

uint16_t computeLag(void)
{
    float lag_in_samples;
    lag_in_samples = roundf((float)ADC1_SAMPLING_FREQUENCY / (4 * g_centerFrequency));
    
    return (uint16_t)lag_in_samples;
}

float computePhaseLag(uint16_t lagInSamples, float outputFrequency)
{
    float delay = (float)lagInSamples / ADC1_SAMPLING_FREQUENCY;
    
    return 2 * M_PI * outputFrequency * delay;
}

float fillDDSBuffer(uint16_t *buffer, uint16_t numSamples, float frequency, float startingPhase)
{
    float phase;

    for (uint16_t i=0; i < numSamples; i++)
    {
        float t = (float)i / DAC1_SAMPLING_FREQUENCY;
        phase = startingPhase + 2.0 * M_PI * frequency * t;

        uint32_t idx = (uint32_t)fabsf((float)PLL_SINE_LOOKUP_SIZE * phase / (2 * M_PI));
        buffer[i] = g_sineLookup[idx % PLL_SINE_LOOKUP_SIZE]; 
    }

    float next_starting_phase = phase + 2.0 * M_PI * frequency / DAC1_SAMPLING_FREQUENCY;
    return next_starting_phase;
}

float computePhaseDifference(uint16_t *sampleBuffer, uint16_t numSamples)
{
    float raw_active_power = 0.;
    float raw_pseudo_reactive_power = 0.;
    float raw_mean_voltage = 0.;
    float raw_mean_current = 0.;

    uint16_t lag = computeLag();
    
    for (uint16_t i = 0; i < numSamples; i++)
    {
        raw_mean_voltage += sampleBuffer[2*i + VSENS_CONVERSION_ORDER]; 
        raw_mean_current += sampleBuffer[2*i + ISENS_CONVERSION_ORDER];
    }

    raw_mean_current = raw_mean_current / numSamples;
    raw_mean_voltage = raw_mean_voltage / numSamples;

    for (uint16_t i = lag; i < numSamples; i++)
    {
        float raw_voltage = (float)sampleBuffer[2*i + VSENS_CONVERSION_ORDER] - raw_mean_voltage;
        float raw_current = (float)sampleBuffer[2*i + ISENS_CONVERSION_ORDER] - raw_mean_current;
        float raw_current_lagged = (float)sampleBuffer[2*(i - lag) + ISENS_CONVERSION_ORDER] - raw_mean_current;

        raw_active_power += raw_voltage * raw_current;
        raw_pseudo_reactive_power += raw_voltage * raw_current_lagged;
    }

    float phase_shift = computePhaseLag(lag, g_centerFrequency + g_correctionFrequency);
    float raw_reactive_power = (raw_pseudo_reactive_power - cosf(phase_shift) * raw_active_power) \
     / sinf(phase_shift);

    return atan2f(raw_active_power, raw_reactive_power);
}

void ultrasonicDriverNotificationCallback(uint16_t eventId, uint16_t *buffer)
{
    if (eventId == ULTRASONIC_DRIVER_ADC_BUFFER_FULL_EVENT_ID) 
    {
        float phase_difference = computePhaseDifference(buffer, PLL_ADC_DOUBLE_BUFFER_SIZE/2);
        g_correctionFrequency = PID_Execute(&g_pid, 0., phase_difference);
    }

    if (eventId == ULTRASONIC_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID)
    {
        float output_frequency = g_centerFrequency + g_correctionFrequency;
        g_ddsPhase = fillDDSBuffer(buffer, (PLL_DAC_DOUBLE_BUFFER_SIZE/2), output_frequency, g_ddsPhase);     
    }
}

/* Public functions --------------------------------------*/
void PLL_Init(void)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    PID_Init(&g_pid, 
             PLL_PID_CONTROLLER_GAIN, 
             PLL_PID_CONTROLLER_INTEGRAL_TC, 
             PLL_PID_CONTROLLER_DERIVATIVE_TC, 
             (1.0 / PLL_PID_CONTROLLER_UPDATE_FREQUENCY), 
             PLL_PID_CONTROLLER_FILTER_TC, 
             PLL_PID_CONTROLLER_MIN_FREQUENCY_DEVIATION, 
             PLL_PID_CONTROLLER_MAX_FREQUENCY_DEVIATION);

    g_state = STATE_READY;
}

void PLL_Start(float centerFrequency)
{
    if (g_state != STATE_READY) 
    {
        return;
    }

    /* Start PID controller. */
    PID_Start(&g_pid, 0.);

    /* Start working registers. */
    g_correctionFrequency = 0.;
    g_ddsPhase = 0.;

    /* Fill DDS buffer initially. */
    g_ddsPhase = fillDDSBuffer(g_dacBuffer, 
                               PLL_DAC_DOUBLE_BUFFER_SIZE, 
                               g_centerFrequency, 
                               g_ddsPhase);    

    /* Start ultrasonic driver. */
    UltrasonicDriver_Start(g_dacBuffer, 
                           g_adcBuffer, 
                           PLL_DAC_DOUBLE_BUFFER_SIZE, 
                           PLL_ADC_DOUBLE_BUFFER_SIZE, 
                           TRUE, 
                           ultrasonicDriverNotificationCallback);

    g_state = STATE_OPERATING;
}

void PLL_Stop(void)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    UltrasonicDriver_Stop();

    g_state = STATE_READY;
}
