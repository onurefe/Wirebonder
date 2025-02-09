#include "adc_dispatcher.h"
#include "generic.h"

/* ------------------------------------------------------------------------
   Private Definitions / Declarations
------------------------------------------------------------------------- */
typedef struct
{
    uint8_t                     conversionOrder;    // e.g. ADC_CHANNEL_6
    AdcDispatcher_Callback_t    callback;       // user callback or NULL
    uint16_t                    downsamplingFactor;
    uint32_t                    accumulator;
    uint16_t                    accumulatedSamples;
    
} AdcDispatcher_Channel_t;

/* -- Static variables -- */
static State_t                  g_state = STATE_UNINIT;
static ADC_HandleTypeDef        *g_hadc = NULL;
static TIM_HandleTypeDef        *g_htim = NULL;
static uint8_t                  g_numChannels = 0;
static AdcDispatcher_Channel_t  g_channels[ADC_DISPATCHER_MAX_CHANNELS];

// This array will hold the ADC converted results in the order the channels are configured.
static uint16_t g_AdcBuffer[2*ADC_DISPATCHER_MAX_CHANNELS] = {0};

/* ------------------------------------------------------------------------
   Helper Functions                                                         
------------------------------------------------------------------------- */
void measurementHandler(uint16_t *buffer) 
{
    if (g_state != STATE_OPERATING) {
        return;
    }

    // Update channel data and call callbacks if assigned
    for (uint8_t c = 0; c < g_numChannels; c++)
    {
        uint16_t value = buffer[g_channels[c].conversionOrder];
        g_channels[c].accumulator += value;
        g_channels[c].accumulatedSamples++;

        if (g_channels[c].accumulatedSamples >= g_channels[c].downsamplingFactor) {
            uint16_t downsampled_value = g_channels[c].accumulator / g_channels[c].accumulatedSamples;
            g_channels[c].callback(downsampled_value);
            g_channels[c].accumulator = 0;
            g_channels[c].accumulatedSamples = 0;
        }
    }
}

/* ------------------------------------------------------------------------
   Public Functions
------------------------------------------------------------------------- */
void AdcDispatcher_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    if (g_state != STATE_UNINIT) {
        return;
    }

    // Store global references
    g_hadc = hadc;
    g_htim = htim;
}

int8_t AdcDispatcher_RegisterChannel(uint8_t conversionOrder, uint16_t downsamplingFactor, AdcDispatcher_Callback_t callback)
{
    if (g_state != STATE_READY) {
        return;
    }

    if (g_numChannels >= ADC_DISPATCHER_MAX_CHANNELS)
    {
        return -1;
    }

    g_channels[g_numChannels].conversionOrder = conversionOrder;
    g_channels[g_numChannels].callback = callback;
    g_channels[g_numChannels].downsamplingFactor = downsamplingFactor;
    g_numChannels++;

    return (g_numChannels - 1);
}

void AdcDispatcher_Start(void)
{
    if (g_state != STATE_READY) {
        return;
    }

    // Reset all channel accumulators.
    for (uint8_t c = 0; c < g_numChannels; c++) {
        g_channels[c].accumulatedSamples = 0;
        g_channels[c].accumulator = 0;
    }

    // Start DMA transfer in circular mode
    HAL_ADC_Start_DMA(g_hadc, (uint32_t*)g_AdcBuffer, 2*g_numChannels);

    // Start timer that triggers the ADC
    HAL_TIM_Base_Start(g_htim);
}

void AdcDispatcher_Stop(void)
{
    // Stop ADC + DMA
    HAL_ADC_Stop_DMA(g_hadc);

    // Stop timer
    HAL_TIM_Base_Stop(g_htim);
}

/* ------------------------------------------------------------------------
   HAL Callbacks for DMA completion
------------------------------------------------------------------------- */
/**
 * @brief If desired, you can also handle half-transfer callback for double-buffering:
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == g_hadc)
    {
        measurementHandler(g_AdcBuffer);
    }
}

/**
 * @brief Conversion Complete callback in non-blocking mode (DMA).
 *        Called when the entire sequence of channels has been converted once.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == g_hadc)
    {
        measurementHandler(&g_AdcBuffer[g_numChannels]);
    }
}
