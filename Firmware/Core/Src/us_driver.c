#include "us_driver.h"

/* Private variables --------------------------------------------------*/
/* Peripherals etc. */
static TIM_HandleTypeDef *g_htim;
static ADC_HandleTypeDef *g_hadc;
static DAC_HandleTypeDef *g_hdac;
static uint32_t g_dacChannel;

/* State variable. */
static State_t g_state = STATE_UNINIT;

/* Working registers. */
static uint16_t *g_adcBuffer;
static uint16_t *g_dacBuffer;

static uint16_t g_adcBufferSize;
static uint16_t g_dacBufferSize;

static Bool_t g_continuousOperation;
static UsDriver_Callback_t g_callback;

/* Public functions ----------------------------------------------------*/
void UsDriver_Init(TIM_HandleTypeDef *htim, 
                   ADC_HandleTypeDef *hadc, 
                   DAC_HandleTypeDef *hdac, 
                   uint32_t dacChannel)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }
    
    g_htim = htim;
    g_hadc = hadc;
    g_hdac = hdac;
    
    g_dacChannel = dacChannel;

    g_state = STATE_READY;
}

void UsDriver_Start(uint16_t *dacBuffer, 
                    uint16_t *adcBuffer, 
                    uint16_t dacBufferSize, 
                    uint16_t adcBufferSize, 
                    Bool_t continuousOperation, 
                    UsDriver_Callback_t callback)
{
    if (g_state != STATE_READY) 
    {
        return;
    }
    
    /* Start signal generation(not started since triggering timer is activated). */
    if (HAL_DAC_Start_DMA(g_hadc, g_dacChannel, dacBuffer, dacBufferSize, DAC_ALIGN_12B_R) != HAL_OK) 
    {
        while (TRUE);
    }

    /* Start ADC. */
    if (HAL_ADC_Start_DMA(g_hadc, adcBuffer, adcBufferSize) != HAL_OK) 
    {
        while (TRUE);
    }

    /* Start the timer driving both ADC and DAC. */
    if (HAL_TIM_Base_Start(g_htim) != HAL_OK) 
    {
        while (TRUE);
    }

    g_adcBuffer = adcBuffer;
    g_dacBuffer = dacBuffer;

    g_adcBufferSize = adcBufferSize;
    g_dacBufferSize = dacBufferSize;

    g_callback = callback;
    g_continuousOperation = continuousOperation;

    g_state = STATE_OPERATING;
}

void UsDriver_Stop(void)
{
    if (g_state != STATE_OPERATING) 
    {
        return;
    }

    /* Start the timer driving both ADC and DAC. */
    if (HAL_TIM_Base_Stop(g_htim) != HAL_OK) 
    {
        while (TRUE);
    }

    /* Stop ADC. */
    if (HAL_ADC_Stop_DMA(g_hadc) != HAL_OK) 
    {
        while (TRUE);
    }

    /* Stop DAC. */
    if (HAL_DAC_Stop_DMA(g_hadc, g_dacChannel) != HAL_OK) 
    {
        while (TRUE);
    }

    g_state = STATE_READY;
}

void UsDriver_ADCDMAHalfCallback(ADC_HandleTypeDef *hadc)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* For continuous operation, circular buffer is assumed. */
    if (g_continuousOperation) 
    {
        g_callback(US_DRIVER_ADC_BUFFER_FULL_EVENT_ID, g_adcBuffer);
    }
}

void UsDriver_ADCDMAFullCallback(ADC_HandleTypeDef *hadc)
{
    if (g_state != STATE_OPERATING)
        return;
    
    /* For continuous operation, circular buffer is assumed. */
    if (g_continuousOperation) 
    {
        g_callback(US_DRIVER_ADC_BUFFER_FULL_EVENT_ID, &g_adcBuffer[g_adcBufferSize / 2]);
    }
    else
    {
        g_callback(US_DRIVER_OPERATION_COMPLETED_EVENT_ID, &g_adcBuffer[g_adcBufferSize / 2]);
        UltrasonicDriver_Stop();
    }
}

void UsDriver_DACDMAHalfCallback(DAC_HandleTypeDef *hdac)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    /* For continuous operation, circular buffer is assumed. */
    if (g_continuousOperation) 
    {
        g_callback(US_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID, g_dacBuffer);
    }
}

void UsDriver_DACDMAFullCallback(DAC_HandleTypeDef *hdac)
{
    if (g_state != STATE_OPERATING)
        return;
    
    /* For continuous operation, circular buffer is assumed. */
    if (g_continuousOperation) 
    {
        g_callback(US_DRIVER_DAC_BUFFER_EMPTY_EVENT_ID, &g_dacBuffer[g_dacBufferSize / 2]);
    }
}