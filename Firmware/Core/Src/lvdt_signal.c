#include "lvdt_signal.h"
#include <math.h>  // for sinf()
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"  // Adjust include as needed for your HAL definitions

/* ----------------- Private Definitions ----------------- */

// We want a 1 kHz sine wave.
// Let's choose 100 samples per cycle => sample rate = 1 kHz * 100 = 100 kHz.
#define LVDT_SAMPLES        100
#define LVDT_WAVE_FREQ_HZ   1000    // 1 kHz output
#define LVDT_SAMPLE_RATE    (LVDT_WAVE_FREQ_HZ * LVDT_SAMPLES) // 100 kHz

// 12-bit DAC: output range 0..4095
static uint16_t LVDT_SineLUT[LVDT_SAMPLES];

/* ----------------- Private Variables ----------------- */

// Peripheral handles for DAC, Timer, DMA
static DAC_HandleTypeDef    hdac;
static DMA_HandleTypeDef    hdma_dac2;
static TIM_HandleTypeDef    htim6;

/* ----------------- State Definitions ----------------- */
typedef enum {
    LVDT_SIGNAL_STATE_UNINIT = 0,
    LVDT_SIGNAL_STATE_READY,
    LVDT_SIGNAL_STATE_OPERATING
} LVDT_SignalState_t;

static LVDT_SignalState_t sSignalState = LVDT_SIGNAL_STATE_UNINIT;

/* ----------------- Private Function Prototypes ----------------- */
static void LVDT_GenerateSineLUT(void);
static void LVDT_TIM6_Init(void);
static void LVDT_DAC2_Init(void);
static void LVDT_DMA_Init(void);

/* ----------------------------------------------------------------
   Public Functions 
----------------------------------------------------------------- */

/**
 * @brief Initialize the sine wave generator.
 *        1) Generates the LUT for a 1 kHz sine wave.
 *        2) Configures DAC2 (PA5), TIM6 for triggering, and DMA in circular mode.
 *        3) Leaves the system ready to start, but does not start the wave.
 *
 * This function is allowed only when the module is uninitialized.
 */
void LVDT_Signal_Init(void)
{
    if (sSignalState != LVDT_SIGNAL_STATE_UNINIT)
    {
        // Already initialized – ignore or report an error.
        return;
    }

    // 1) Generate the sine lookup table.
    LVDT_GenerateSineLUT();

    // 2) Initialize the peripherals.
    //    (Assume the system clock is already configured in your main application)
    LVDT_DMA_Init();
    LVDT_DAC2_Init();
    LVDT_TIM6_Init();

    sSignalState = LVDT_SIGNAL_STATE_READY;
}

/**
 * @brief Start outputting the 1 kHz sine wave on DAC Channel 2.
 *
 * This function starts the DAC in DMA circular mode and starts TIM6.
 * It is allowed only when the module is in READY state.
 */
void LVDT_Signal_Start(void)
{
    if (sSignalState != LVDT_SIGNAL_STATE_READY)
    {
        // Only allow start when the system is ready.
        return;
    }

    // Start DAC Channel 2 in DMA circular mode.
    if (HAL_DAC_Start_DMA(&hdac,
                          DAC_CHANNEL_2,
                          (uint32_t*)LVDT_SineLUT,
                          LVDT_SAMPLES,
                          DAC_ALIGN_12B_R) != HAL_OK)
    {
        // Handle error as appropriate.
        return;
    }

    // Start the timer that triggers the DAC.
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        // Handle error as appropriate.
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
        return;
    }

    sSignalState = LVDT_SIGNAL_STATE_OPERATING;
}

/**
 * @brief Stop the DAC output (and the trigger timer).
 *
 * This function stops DAC Channel 2 and TIM6.
 * It is allowed only when the module is in OPERATING state.
 */
void LVDT_Signal_Stop(void)
{
    if (sSignalState != LVDT_SIGNAL_STATE_OPERATING)
    {
        // Only allow stop if the system is operating.
        return;
    }

    // Stop the DAC channel and the timer.
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
    HAL_TIM_Base_Stop(&htim6);

    sSignalState = LVDT_SIGNAL_STATE_READY;
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
 * @brief Timer 6 Initialization to output an update event at LVDT_SAMPLE_RATE,
 *        which triggers DAC updates.
 *
 *        Timer clock assumed to be APB1 clock (e.g., 84 MHz).
 *        Frequency = TIMCLK / ((PSC+1)*(ARR+1))
 */
static void LVDT_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    // Example configuration:
    // PSC = 83 => timer clock = 84MHz / (83+1) = 1 MHz
    // ARR = 9  => update event = 1MHz / (9+1) = 100 kHz
    htim6.Instance = TIM6;
    htim6.Init.Prescaler         = 83;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 9;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        // Error handling: you may log or signal an error here.
    }

    // Configure master mode to trigger DAC on update event.
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        // Error handling.
    }
}

/**
 * @brief Initialize DAC channel 2 (PA5 on STM32F446),
 *        triggered by TIM6, 12-bit right alignment, output buffer enabled.
 */
static void LVDT_DAC2_Init(void)
{
    __HAL_RCC_DAC_CLK_ENABLE();

    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        // Error handling.
    }

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger      = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        // Error handling.
    }
}

/**
 * @brief Initialize the DMA for DAC Channel 2 in circular mode.
 */
static void LVDT_DMA_Init(void)
{
    // On STM32F446, DAC2 typically uses DMA1_Stream6/Channel7; confirm with your datasheet.
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

    if (HAL_DMA_Init(&hdma_dac2) != HAL_OK)
    {
        // Error handling.
    }

    // Link DMA handle to DAC handle.
    __HAL_LINKDMA(&hdac, DMA_Handle2, hdma_dac2);

    // Optionally, enable DMA IRQs for half/full transfer callbacks:
    // HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}
