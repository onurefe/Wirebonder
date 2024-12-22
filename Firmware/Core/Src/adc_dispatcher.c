#include "adc_dispatcher.h"
#include <string.h>  // for memset if needed

/* ------------------------------------------------------------------------
   Private Definitions / Declarations
------------------------------------------------------------------------- */

/**
 * @brief Structure to hold per-channel information: which ADC channel,
 *        optional user callback, and the latest sample value.
 */
typedef struct
{
    uint32_t       AdcChannel;    // e.g. ADC_CHANNEL_6
    AdcCallback_t  Callback;       // user callback or NULL
    uint16_t       LatestValue;    // last captured ADC value
} AdcDispatcher_Channel_t;

/* -- Static variables -- */
static ADC_HandleTypeDef    *g_hadc   = NULL;
static TIM_HandleTypeDef    *g_htim   = NULL;
static uint8_t               g_numChannels = 0;
static AdcDispatcher_Channel_t g_channels[ADC_DISPATCHER_MAX_CHANNELS];

// This array will hold the ADC converted results in the order the channels are configured.
static uint16_t g_AdcValues[ADC_DISPATCHER_MAX_CHANNELS] = {0};

/* ------------------------------------------------------------------------
   Private Function Prototypes
------------------------------------------------------------------------- */
static void ADC_Dispatcher_ConfigADC(uint32_t sampleFreqHz);
static void ADC_Dispatcher_ConfigTimer(uint32_t sampleFreqHz);

/* ------------------------------------------------------------------------
   Public Functions
------------------------------------------------------------------------- */

/**
 * @brief Register a channel prior to initialization. 
 */
int8_t ADC_Dispatcher_RegisterChannel(uint32_t adcChannel, AdcCallback_t callback)
{
    if (g_numChannels >= ADC_DISPATCHER_MAX_CHANNELS)
    {
        return -1; // no space
    }
    g_channels[g_numChannels].AdcChannel = adcChannel;
    g_channels[g_numChannels].Callback   = callback;
    g_channels[g_numChannels].LatestValue = 0;
    g_numChannels++;
    return (g_numChannels - 1);
}

void ADC_Dispatcher_Init(ADC_HandleTypeDef *hadc, 
                         TIM_HandleTypeDef *htim, 
                         uint32_t sampleFreqHz)
{
    // Store global references
    g_hadc = hadc;
    g_htim = htim;

    // Basic check
    if (g_numChannels == 0)
    {
        // No channels registered; either return or handle error
        return;
    }

    // 1) Configure Timer for the desired sample frequency
    ADC_Dispatcher_ConfigTimer(sampleFreqHz);

    // 2) Configure ADC in scan mode with 'g_numChannels' channels
    ADC_Dispatcher_ConfigADC(sampleFreqHz);
}

void ADC_Dispatcher_Start(void)
{
    // Start DMA transfer in circular mode
    HAL_ADC_Start_DMA(g_hadc, 
                      (uint32_t*)g_AdcValues, 
                      g_numChannels);

    // Start timer that triggers the ADC
    HAL_TIM_Base_Start(g_htim);
}

void ADC_Dispatcher_Stop(void)
{
    // Stop ADC + DMA
    HAL_ADC_Stop_DMA(g_hadc);

    // Stop timer
    HAL_TIM_Base_Stop(g_htim);
}

uint16_t ADC_Dispatcher_GetValue(uint8_t channelIndex)
{
    if (channelIndex >= g_numChannels)
    {
        return 0; // error
    }
    return g_channels[channelIndex].LatestValue;
}

/* ------------------------------------------------------------------------
   Private Functions
------------------------------------------------------------------------- */

static void ADC_Dispatcher_ConfigTimer(uint32_t sampleFreqHz)
{
    /* Example for TIM3 on an STM32F4 at 84MHz on APB1:
       freq = TIMclk / ((PSC+1)*(ARR+1))

       Suppose we want sampleFreqHz = 10kHz
       PSC=83 => Timer freq => 84MHz/(83+1) = 1MHz
       ARR = (1MHz / sampleFreqHz) - 1
       For 10kHz => ARR = 100 -1 = 99

       In practice, you should compute PSC and ARR based on sampleFreqHz. 
       For clarity, we do a simple example calculation. 
    */

    // Enable clock if not done
    // __HAL_RCC_TIM3_CLK_ENABLE(); // if needed

    g_htim->Instance = TIM3;  // e.g. using TIM3
    g_htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Very simple approximate approach (actual might need more robust logic)
    uint32_t timerClock = 84000000; // e.g. 84MHz on APB1
    uint32_t prescaler = 83;       // => timer freq = 1MHz
    uint32_t arr = (timerClock/(prescaler+1))/sampleFreqHz - 1;

    g_htim->Init.Prescaler = prescaler;
    g_htim->Init.Period    = arr;

    HAL_TIM_Base_Init(g_htim);

    // Configure master mode to trigger ADC on update
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(g_htim, &sMasterConfig);
}

static void ADC_Dispatcher_ConfigADC(uint32_t sampleFreqHz)
{
    // Enable ADC clock if not done
    // __HAL_RCC_ADC1_CLK_ENABLE(); or __HAL_RCC_ADC2_CLK_ENABLE(); etc.
    // Adjust for your chosen ADC instance

    // For simplicity, assume g_hadc->Instance = ADC1 or ADC2, etc.
    g_hadc->Init.Resolution            = ADC_RESOLUTION_12B;
    g_hadc->Init.ScanConvMode          = ENABLE;            // multiple channels
    g_hadc->Init.ContinuousConvMode    = DISABLE;           // hardware trigger
    g_hadc->Init.DiscontinuousConvMode = DISABLE;
    g_hadc->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    g_hadc->Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO; // link to TIM3
    g_hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    g_hadc->Init.NbrOfConversion       = g_numChannels;     // how many channels in total
    g_hadc->Init.DMAContinuousRequests = ENABLE;            // for circular mode
    g_hadc->Init.EOCSelection          = ADC_EOC_SEQ_CONV;

    HAL_ADC_Init(g_hadc);

    // Configure each channel
    ADC_ChannelConfTypeDef sConfig = {0};
    for (uint8_t i = 0; i < g_numChannels; i++)
    {
        sConfig.Channel      = g_channels[i].AdcChannel;
        sConfig.Rank         = i + 1; // rank from 1..g_numChannels
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // pick appropriate sampling time
        sConfig.Offset       = 0;
        HAL_ADC_ConfigChannel(g_hadc, &sConfig);
    }
}

/* ------------------------------------------------------------------------
   HAL Callbacks for DMA completion
------------------------------------------------------------------------- */

/**
 * @brief Conversion Complete callback in non-blocking mode (DMA).
 *        Called when the entire sequence of channels has been converted once.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == g_hadc)
    {
        // Update channel data and call callbacks if assigned
        for (uint8_t i = 0; i < g_numChannels; i++)
        {
            g_channels[i].LatestValue = g_AdcValues[i];
            if (g_channels[i].Callback)
            {
                g_channels[i].Callback(i, g_AdcValues[i]);
            }
        }
    }
}

/**
 * @brief If desired, you can also handle half-transfer callback for double-buffering:
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Typically used if the DMA buffer is bigger (e.g., double the channel count).
    // In this single-sequence scenario, we may not need it.
}
