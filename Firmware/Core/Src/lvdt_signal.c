#include "lvdt_signal.h"
#include <math.h>  // for sinf() if you want to generate the LUT at runtime

/* ----------------- Private Definitions ----------------- */

// We want a 1 kHz sine wave
// Let's choose 100 samples per cycle => sample rate = 1 kHz * 100 = 100 kHz
#define LVDT_SAMPLES        100
#define LVDT_WAVE_FREQ_HZ   1000    // 1 kHz output
#define LVDT_SAMPLE_RATE    (LVDT_WAVE_FREQ_HZ * LVDT_SAMPLES) // 100 kHz

// 12-bit DAC => 0..4095
static uint16_t LVDT_SineLUT[LVDT_SAMPLES];

/* ----------------- Private Variables ----------------- */

// Handles for DAC, Timer, DMA
static DAC_HandleTypeDef    hdac;
static DMA_HandleTypeDef    hdma_dac2;
static TIM_HandleTypeDef    htim6;

/* ----------------- Private Function Prototypes ----------------- */
static void LVDT_GenerateSineLUT(void);
static void LVDT_TIM6_Init(void);
static void LVDT_DAC2_Init(void);
static void LVDT_DMA_Init(void);

/* ----------------------------------------------------------------
   Public Functions 
----------------------------------------------------------------- */

/**
 * @brief Initialize the sine wave generator:
 *        1) Prepares the LUT for a 1 kHz sine wave
 *        2) Configures DAC2 (PA5), TIM6 for the trigger, and DMA in circular mode
 *        3) Leaves the system ready to start, but does not actually start the wave
 */
void LVDT_Signal_Init(void)
{
    // 1) Generate the LUT
    LVDT_GenerateSineLUT();

    // 2) Initialize the peripherals
    //    (Assume the system clock is already configured in your main application)
    LVDT_DMA_Init();
    LVDT_DAC2_Init();
    LVDT_TIM6_Init();
}

/**
 * @brief Start outputting the 1 kHz sine wave on DAC Channel 2.
 */
void LVDT_Signal_Start(void)
{
    // Start DAC Channel 2 in DMA circular mode
    HAL_DAC_Start_DMA(&hdac,
                      DAC_CHANNEL_2,
                      (uint32_t*)LVDT_SineLUT,
                      LVDT_SAMPLES,
                      DAC_ALIGN_12B_R);

    // Start the timer that triggers the DAC
    HAL_TIM_Base_Start(&htim6);
}

/**
 * @brief Stop the DAC output (and timer if desired).
 */
void LVDT_Signal_Stop(void)
{
    // Stop the DAC channel
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);

    // Optionally stop the timer
    HAL_TIM_Base_Stop(&htim6);

    // The peripherals remain initialized and can be restarted
}

/* ----------------------------------------------------------------
   Private Functions
----------------------------------------------------------------- */

/**
 * @brief Generate a single cycle of sine wave data in the range [0..4095]
 *        with LVDT_SAMPLES points.
 */
static void LVDT_GenerateSineLUT(void)
{
    for (uint32_t i = 0; i < LVDT_SAMPLES; i++)
    {
        float theta = 2.0f * 3.14159265359f * (float)i / (float)LVDT_SAMPLES;
        float sin_val = (sinf(theta) + 1.0f) / 2.0f; // map [-1..1] to [0..1]
        LVDT_SineLUT[i] = (uint16_t)(sin_val * 4095.0f);
    }
}

/**
 * @brief Timer 6 Initialization to output an Update event at LVDT_SAMPLE_RATE
 *        which triggers the DAC updates.
 *
 *        Timer clock assumed to be APB1 clock (e.g., 84 MHz). 
 *        freq = TIMCLK / ((PSC+1)*(ARR+1))
 */
static void LVDT_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    // Example:
    // PSC = 83 => timer clock = 84MHz / (83+1) = 1 MHz
    // ARR = 9  => update event = 1MHz / (9+1) = 100 kHz
    htim6.Instance = TIM6;
    htim6.Init.Prescaler         = 83;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 9;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);

    // Configure master mode to trigger DAC on update event
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

/**
 * @brief Initialize DAC channel 2 (PA5 on STM32F446),
 *        triggered by TIM6, 12-bit right alignment, output buffer enabled.
 */
static void LVDT_DAC2_Init(void)
{
    __HAL_RCC_DAC_CLK_ENABLE();

    hdac.Instance = DAC;
    HAL_DAC_Init(&hdac);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger      = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
}

/**
 * @brief Initialize the DMA for DAC Channel 2 in circular mode.
 */
static void LVDT_DMA_Init(void)
{
    // On STM32F446, DAC2 typically uses DMA1_Stream6/Channel7, but confirm with your datasheet.
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_dac2.Instance                 = DMA1_Stream6;
    hdma_dac2.Init.Channel             = DMA_CHANNEL_7;
    hdma_dac2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_dac2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_dac2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_dac2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_dac2.Init.Mode                = DMA_CIRCULAR;
    hdma_dac2.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_dac2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_dac2);

    // Link DMA handle to DAC handle
    __HAL_LINKDMA(&hdac, DMA_Handle2, hdma_dac2);

    // If you want DMA IRQs for half/full transfer callbacks, enable them here:
    // HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}
