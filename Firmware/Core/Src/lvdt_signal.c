#include "lvdt_signal.h"
#include <math.h>  // for sinf()
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"  // Adjust include as needed for your HAL definitions
#include "generic.h"
#include "configuration.h"

/* ----------------- Private Definitions ----------------- */
#define LVDT_DAC_SAMPLES (LVDT_GENERATION_SAMPLING_RATE / LVDT_SIGNAL_FREQ)

/* ----------------- Private Variables ----------------- */
static DAC_HandleTypeDef    *g_hdac;
static TIM_HandleTypeDef    *g_htim;
static uint32_t             g_dac_channel;

static State_t g_state = STATE_UNINIT;

static uint16_t g_sineLUT[LVDT_DAC_SAMPLES];

/* ----------------- Private Function Prototypes ----------------- */
static void generateSineLUT(void)
{
    for (uint32_t i = 0; i < LVDT_DAC_SAMPLES; i++)
    {
        float theta = 2.0f * 3.14159265359f * (float)i / (float)LVDT_DAC_SAMPLES;
        float sin_val = (sinf(theta) + 1.0f) / 2.0f; // map [-1..1] to [0..1]
        g_sineLUT[i] = (uint16_t)(sin_val * 4095.0f);
    }
}


/* ----------------------------------------------------------------
   Public Functions 
----------------------------------------------------------------- */
void LVDT_Signal_Init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t dac_channel)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    generateSineLUT();
    
    g_htim = htim;
    g_hdac = hdac;
    g_dac_channel = dac_channel;

    g_state = STATE_READY;
}

void LVDT_Signal_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    if (HAL_DAC_Start_DMA(g_hdac,
                          g_dac_channel,
                          (uint32_t*)g_sineLUT,
                          LVDT_DAC_SAMPLES,
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

void LVDT_Signal_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    HAL_DAC_Stop_DMA(g_hdac, g_dac_channel);
    HAL_TIM_Base_Stop(g_htim);

    g_state = STATE_READY;
}
