#include "adc_dispatcher.h"
#include "adc_conversion_orders.h"
#include "lvdt.h"
#include "stm32f4xx_hal.h"
#include "generic.h"
#include "configuration.h"

/* ----------------- Private Definitions ----------------- */
#define LOOKUP_TABLE_SIZE   (LVDT_WAVE_GENERATION_SAMPLING_FREQUENCY / LVDT_DRIVING_FREQUENCY)
#define DOWNSAMPLING_FACTOR (ADC_DISPATCHER_NOTIFICATION_FREQUENCY / LVDT_NOTIFICATION_FREQUENCY)

/* ----------------- Private Variables ----------------- */
static DAC_HandleTypeDef    *g_hdac;
static TIM_HandleTypeDef    *g_htim;
static uint32_t             g_dac_channel;

static State_t g_state = STATE_UNINIT;

static uint8_t g_numCallbacks;
static LVDT_MeasurementCallback_t g_measurementCallbacks[LVDT_MAX_NUM_OF_CALLBACKS];

static uint16_t g_sineLUT[LOOKUP_TABLE_SIZE];

/* Working registers ---------------------------------------------*/
static uint16_t g_aChannelAccumulatedSamples;
static uint16_t g_bChannelAccumulatedSamples;

static uint32_t g_aChannelAccumulator[4];
static uint32_t g_bChannelAccumulator[4];

static Bool_t g_aChannelMeasurementReady;
static Bool_t g_bChannelMeasurementReady;

static float g_aChannelMeasurementValue;
static float g_bChannelMeasurementValue;

/* ----------------- Private Functions ------------------------- */
static void generateSineLookupTable(void)
{
    for (uint32_t i = 0; i < LOOKUP_TABLE_SIZE; i++)
    {
        float theta = 2.0f * 3.14159265359f * (float)i / (float)LOOKUP_TABLE_SIZE;
        float sin_val = (sinf(theta) + 1.0f) / 2.0f;
        g_sineLUT[i] = (uint16_t)(sin_val * 4095.0f);
    }
}

static void compute4PointVoltage(uint32_t *accumulatedSamples, 
                                 float *amplitude)
{
    float S1 = (float)accumulatedSamples[0];
    float S2 = (float)accumulatedSamples[1];
    float S3 = (float)accumulatedSamples[2];
    float S4 = (float)accumulatedSamples[3];

    /* 3. Compute I and Q */
    float I = S1 - S3;  // ~ 2A * sin(phase)
    float Q = S2 - S4;  // ~ 2A * cos(phase)

    *amplitude = 0.5 * sqrtf(I * I + Q * Q);
}

static void notifyMeasurement(void)
{
    float amplitude_diff = g_aChannelMeasurementValue - g_bChannelMeasurementValue;
    float position = amplitude_diff * LVDT_AMPLITUDE_DIFFERENCE_TO_POSITION_CONVERSION_FACTOR;

    for (uint8_t i = 0; i < g_numCallbacks; i++)
    {
        if (g_measurementCallbacks[i] != NULL)
        {
            g_measurementCallbacks[i](position);
        }
    }
}

/* Channel A callback */
static void adcDispatcherCallbackChannelA(uint16_t value)
{
    /* Process samples only when operating */
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* Accumulate ADC values */
    g_aChannelAccumulator[g_aChannelAccumulatedSamples % 4] += value;
    g_aChannelAccumulatedSamples++;

    /* If enough samples are accumulated, compute amplitude */
    if (g_aChannelAccumulatedSamples >= DOWNSAMPLING_FACTOR)
    {
        float amplitude;
        compute4PointVoltage(g_aChannelAccumulatedSamples, &amplitude);
        
        g_aChannelAccumulatedSamples = 0;
        g_aChannelAccumulator[0] = 0;
        g_aChannelAccumulator[1] = 0;
        g_aChannelAccumulator[2] = 0;
        g_aChannelAccumulator[3] = 0;
        g_aChannelMeasurementValue = amplitude;

        /* If channel B has also provided a new phase, notify callbacks */
        if (g_bChannelMeasurementReady)
        {
            notifyMeasurement();
            g_aChannelMeasurementReady = FALSE;
            g_bChannelMeasurementReady = FALSE;
        }
    }
}

/* Channel B callback */
static void adcDispatcherCallbackChannelB(uint16_t value)
{
    /* Process samples only when operating */
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* Accumulate ADC values */
    g_bChannelAccumulator[g_bChannelAccumulatedSamples % 4] += value;
    g_bChannelAccumulatedSamples++;

    /* If enough samples are accumulated, compute amplitude */
    if (g_bChannelAccumulatedSamples >= DOWNSAMPLING_FACTOR)
    {
        float amplitude;
        compute4PointVoltage(g_bChannelAccumulatedSamples, &amplitude);
        
        g_bChannelAccumulatedSamples = 0;
        g_bChannelAccumulator[0] = 0;
        g_bChannelAccumulator[1] = 0;
        g_bChannelAccumulator[2] = 0;
        g_bChannelAccumulator[3] = 0;
        g_bChannelMeasurementValue = amplitude;

        /* If channel A has also provided a new phase, notify callbacks */
        if (g_aChannelMeasurementReady)
        {
            notifyMeasurement();
            g_aChannelMeasurementReady = FALSE;
            g_bChannelMeasurementReady = FALSE;
        }
    }
}


/* ----------------------------------------------------------------
   Public Functions 
----------------------------------------------------------------- */
void LVDT_Init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t dac_channel)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    AdcDispatcher_RegisterChannel(LVDT_A_ADC2_CONVERSION_ORDER,
                                  adcDispatcherCallbackChannelA);

    AdcDispatcher_RegisterChannel(LVDT_B_ADC2_CONVERSION_ORDER, 
                                  adcDispatcherCallbackChannelB);

    generateSineLookupTable();
    
    g_htim = htim;
    g_hdac = hdac;
    g_dac_channel = dac_channel;

    g_numCallbacks = 0;

    g_state = STATE_READY;
}

void LVDT_RegisterCallback(LVDT_MeasurementCallback_t callback)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    if (g_numCallbacks > LVDT_MAX_NUM_OF_CALLBACKS) {
        return;
    }

    g_measurementCallbacks[g_numCallbacks++] = callback;
}

void LVDT_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    // Clear working registers.
    g_aChannelAccumulatedSamples = 0;
    g_bChannelAccumulatedSamples = 0;

    g_aChannelMeasurementReady = FALSE;
    g_bChannelMeasurementReady = FALSE;

    for (uint8_t i = 0; i < 4; i++) {
        g_aChannelAccumulator[i] = 0;
        g_bChannelAccumulator[i] = 0;
    }

    // Start signal generation.
    if (HAL_DAC_Start_DMA(g_hdac,
                          g_dac_channel,
                          (uint32_t*)g_sineLUT,
                          LOOKUP_TABLE_SIZE,
                          DAC_ALIGN_12B_R) != HAL_OK)
    {
        return;
    }

    if (HAL_TIM_Base_Start(g_htim) != HAL_OK)
    {
        // Handle error as appropriate.
        HAL_DAC_Stop_DMA(g_hdac, g_dac_channel);
        return;
    }

    g_state = STATE_OPERATING;
}

void LVDT_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    HAL_DAC_Stop_DMA(g_hdac, g_dac_channel);
    HAL_TIM_Base_Stop(g_htim);

    g_state = STATE_READY;
}
