#include "pll.h"
#include "pid_controller.h"
#include "us_driver.h"
#include "stm32f4xx_hal.h"
#include "configuration.h"
#include "complex.h"
#include "math.h"
#include "generic.h"
#include "adc_conversion_orders.h"

/* Private constants -------------------------------------*/
// Factor of 4 from using double buffer and measuring both current and voltage.
#define ADC_BUFFER_SIZE                 (4 * PLL_ADC_SAMPLES)

// Factor of 2 from using double buffer.
#define DAC_BUFFER_SIZE                 (2 * PLL_DAC_SAMPLES)


#define CONTROL_UPDATE_FREQUENCY \
((float)ADC1_SAMPLING_FREQUENCY / PLL_ADC_SAMPLES)

#define CONTROL_UPDATE_PERIOD           (1.0 / CONTROL_UPDATE_FREQUENCY)

/* Private variables -------------------------------------*/
static uint16_t g_sineLookup[PLL_SINE_LOOKUP_SIZE];

static uint16_t g_adcBuffer[ADC_BUFFER_SIZE];
static uint16_t g_dacBuffer[DAC_BUFFER_SIZE];

static Pll_Callback_t g_callback;

static State_t g_state = STATE_UNINIT;
static Pid_Controller g_pid;

static float g_centerFrequency;
static float g_targetBondingEnergy;
static float g_maxBondingDuration;

static float g_bondingDuration;
static float g_bondingEnergy;

static float g_ddsPhase;
static float g_correctionFrequency;

/* Private functions -------------------------------------*/
inline float rawValueToVoltage(uint16_t adcValue)
{
    float adc_in = ADC_VOLTAGE_RANGE * (float)adcValue / (1<< ADC_BITS);
    
    return ((adc_in - 0.5 * ADC_VOLTAGE_RANGE) / US_VOLTAGE_SENSING_GAIN);
}

inline float rawValueToCurrent(uint16_t adcValue)
{
    float adc_in = ADC_VOLTAGE_RANGE * (float)adcValue / (1<< ADC_BITS);
    
    return ((adc_in - 0.5 * ADC_VOLTAGE_RANGE) / US_CURRENT_SENSING_GAIN);
}

void fillSineLookup(void)
{
    float phase;

    for (uint16_t i = 0; i < PLL_SINE_LOOKUP_SIZE; i++) 
    {
        phase = 2 * M_PI * (float)i / PLL_SINE_LOOKUP_SIZE;
        g_sineLookup[i] = (uint16_t)(0.5 * (sinf(phase) + 1.0) * (1 << DAC_BITS)); 
    }
}

uint16_t computeSampleLag(void)
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

float fillDdsBuffer(uint16_t *buffer, float frequency, float startingPhase)
{
    float phase;

    for (uint16_t i=0; i < PLL_DAC_SAMPLES; i++)
    {
        float t = (float)i / DAC1_SAMPLING_FREQUENCY;
        phase = startingPhase + 2.0 * M_PI * frequency * t;

        uint32_t idx = (uint32_t)fabsf((float)PLL_SINE_LOOKUP_SIZE * phase / (2 * M_PI));
        buffer[i] = g_sineLookup[idx % PLL_SINE_LOOKUP_SIZE]; 
    }

    float next_starting_phase = phase + 2.0 * M_PI * frequency / DAC1_SAMPLING_FREQUENCY;
    return next_starting_phase;
}

void computeComplexPower(uint16_t *sampleBuffer, complexf *power)
{
    uint32_t raw_mean_voltage = 0;
    uint32_t raw_mean_current = 0;

    uint16_t lag_in_samples = computeSampleLag();
    
    /* Compute mean values. Mean values will be subtracted when computing power. */
    for (uint16_t i = 0; i < PLL_ADC_SAMPLES; i++)
    {
        raw_mean_voltage += sampleBuffer[2*i + VSENS_CONVERSION_ORDER]; 
        raw_mean_current += sampleBuffer[2*i + ISENS_CONVERSION_ORDER];
    }

    float mean_current = rawValueToCurrent(raw_mean_current / PLL_ADC_SAMPLES);
    float mean_voltage = rawValueToVoltage(raw_mean_voltage / PLL_ADC_SAMPLES);

    float active_power = 0.;
    float pseudo_reactive_power = 0.;

    /* Compute active and pseudo-reactive power. We can't compute reactive power 
    exactly in this phase since the lag is not exactly 90 degrees. */
    for (uint16_t i = lag_in_samples; i < PLL_ADC_SAMPLES; i++)
    {
        float voltage = rawValueToVoltage(sampleBuffer[2*i + VSENS_CONVERSION_ORDER]);
        float current = rawValueToCurrent(sampleBuffer[2*i + ISENS_CONVERSION_ORDER]);
        float current_lagged = rawValueToCurrent(sampleBuffer[2*(i - lag_in_samples) + ISENS_CONVERSION_ORDER]);

        active_power += (voltage - mean_voltage) * (current - mean_current);
        pseudo_reactive_power += (voltage - mean_voltage) * (current_lagged - mean_current);
    }

    active_power = active_power / (float)(PLL_ADC_SAMPLES - lag_in_samples);
    pseudo_reactive_power = pseudo_reactive_power / (float)(PLL_ADC_SAMPLES - lag_in_samples);

    /* Since the phase shift is not exactly 90 degrees, fix the pseudo reactive power. */
    float phase_shift = computePhaseLag(lag_in_samples, g_centerFrequency + g_correctionFrequency);
    power->re = active_power;
    power->im = (pseudo_reactive_power - cosf(phase_shift) * active_power) / sinf(phase_shift);
}

void ultrasonicDriverNotificationCallback(uint16_t eventId, uint16_t *buffer)
{
    /* Process measurements. */
    if (eventId == US_DRIVER_ADC_BUFFER_FULL_EVENT_ID) 
    {
        float phase;
        complexf power;

        /* Compute complex power. */
        computeComplexPower(buffer, &power);
        
        /* Update controller output. */
        phase = atan2f(power.re, power.im);
        g_correctionFrequency = PID_Execute(&g_pid, 0., phase);

        /* Accumulate transferred bonding energy and duration. */
        g_bondingEnergy += power.re * CONTROL_UPDATE_PERIOD;
        g_bondingDuration += CONTROL_UPDATE_PERIOD;

        /* If sufficient energy is transferred, the operation
        is completed. */
        if (g_bondingEnergy >= g_targetBondingEnergy)
        {
            g_callback(PLL_BONDING_COMPLETED_EVENT_ID);
            
            UltrasonicDriver_Stop();
            g_state = STATE_READY;
        }

        /* If sufficient energy is transferred withing the given
        duration, take this as an error. */
        if (g_bondingDuration >= g_maxBondingDuration)
        {
            g_callback(PLL_INSUFFICIENT_BONDING_POWER_EVENT_ID);
            
            UltrasonicDriver_Stop();
            g_state = STATE_READY;
        }
    }

    /* Fill DDS buffer using the updated output frequency. */
    if (eventId == US_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID)
    {
        float output_frequency = g_centerFrequency + g_correctionFrequency;
        g_ddsPhase = fillDdsBuffer(buffer, output_frequency, g_ddsPhase);     
    }
}

/* Public functions --------------------------------------*/
void Pll_Init(void)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }

    Pid_Init(&g_pid, 
             PLL_PID_CONTROLLER_GAIN, 
             PLL_PID_CONTROLLER_INTEGRAL_TC, 
             PLL_PID_CONTROLLER_DERIVATIVE_TC, 
             CONTROL_UPDATE_PERIOD, 
             PLL_PID_CONTROLLER_FILTER_TC, 
             PLL_PID_CONTROLLER_MIN_FREQUENCY_DEVIATION, 
             PLL_PID_CONTROLLER_MAX_FREQUENCY_DEVIATION);

    fillSineLookup();

    g_state = STATE_READY;
}

void Pll_Start(float centerFrequency, 
               float bondingEnergy, 
               float maxBondingDuration, 
               Pll_Callback_t callback)
{
    if (g_state != STATE_READY) 
    {
        return;
    }

    g_centerFrequency = centerFrequency;
    g_targetBondingEnergy = bondingEnergy;
    g_maxBondingDuration = maxBondingDuration;
    g_callback = callback;

    /* Start PID controller. */
    Pid_Start(&g_pid, 0.);

    /* Clear working registers. */
    g_correctionFrequency = 0.;
    g_ddsPhase = 0.;
    g_bondingEnergy = 0.;
    g_bondingDuration = 0.;

    /* Fill DDS buffer initially. */
    g_ddsPhase = fillDdsBuffer(g_dacBuffer, 
                               g_centerFrequency, 
                               g_ddsPhase);    

    /* Start ultrasonic driver. */
    UsDriver_Start(g_dacBuffer, 
                   g_adcBuffer, 
                   DAC_BUFFER_SIZE, 
                   ADC_BUFFER_SIZE, 
                   TRUE, 
                   ultrasonicDriverNotificationCallback);

    g_state = STATE_OPERATING;
}

void Pll_Stop(void)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    UltrasonicDriver_Stop();

    g_state = STATE_READY;
}
