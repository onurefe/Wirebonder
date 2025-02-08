#include "pll.h"
#include "pid_controller.h"      // PID controller library
#include "impedance_measurement.h" // Provides ComplexImpedance and callback registration
#include "stm32f4xx_hal.h"       // HAL for timer configuration (adjust as needed)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ------------------ Internal State Definitions ------------------ */
typedef enum {
    PLL_STATE_UNINIT = 0,
    PLL_STATE_READY,
    PLL_STATE_OPERATING
} PLL_State_t;

static PLL_State_t sPLLState = PLL_STATE_UNINIT;

/* ------------------ Internal Variables ------------------ */
static float sCenterFrequency = 0.0f;   // Center frequency (Hz)
static PID_Controller pidPLL;           // PLL PID controller instance

// TIMER2 handle – used to drive the piezoelectric transducer.
static TIM_HandleTypeDef htim2;

/* ------------------ Private Function Prototypes ------------------ */
static void PLL_ImpedanceCallback(ComplexImpedance impedance);
static void PLL_UpdateTimer2Frequency(float newFrequency);

/* ------------------ Private Functions ------------------ */

/**
 * @brief Update TIMER2 frequency.
 *
 * Given a desired new frequency (Hz), this function recalculates TIMER2's period
 * and restarts the timer. For this example, we assume that:
 *   - TIMER2 clock is derived from a fixed clock (e.g. 84 MHz),
 *   - A fixed prescaler (e.g. 83) is used, so that the timer clock becomes 84e6/84 = 1 MHz.
 * Then the period (ARR) is calculated as: ARR = (1e6 / newFrequency) - 1.
 *
 * @param newFrequency The new frequency (Hz) for TIMER2.
 */
static void PLL_UpdateTimer2Frequency(float newFrequency)
{
    // Fixed prescaler value.
    uint32_t prescaler = 83; // Timer clock = 84e6/(83+1)=1e6 Hz.
    uint32_t arr = (uint32_t)(1000000.0f / newFrequency) - 1;
    
    // Stop TIMER2 before reconfiguration.
    HAL_TIM_Base_Stop(&htim2);
    
    htim2.Init.Prescaler = prescaler;
    htim2.Init.Period = arr;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        // Error handling: you might log or assert here.
    }
    
    // Restart TIMER2.
    HAL_TIM_Base_Start(&htim2);
}

/**
 * @brief PLL callback registered with the impedance measurement module.
 *
 * This function is called whenever a new impedance measurement is ready.
 * The measured phase (in radians) is used as the error (desired phase = 0).
 * The PID controller computes a frequency correction (in Hz) which is added to
 * the center frequency to determine the new TIMER2 frequency.
 *
 * @param impedance The measured complex impedance.
 */
static void PLL_ImpedanceCallback(ComplexImpedance impedance)
{
    // Error is the measured phase (we desire zero phase difference).
    float error = impedance.phase; // desired phase is 0.
    
    // Compute PID output (frequency correction in Hz).
    float freqCorrection = PID_Execute(&pidPLL, 0.0f, error);
    
    // Compute new frequency = center frequency + frequency correction.
    float newFrequency = sCenterFrequency + freqCorrection;
    
    // Optionally enforce minimum/maximum frequency limits.
    if (newFrequency < 1.0f)
        newFrequency = 1.0f;
    
    // Update TIMER2 frequency.
    PLL_UpdateTimer2Frequency(newFrequency);
}

/* ------------------ Public Functions ------------------ */

void PLL_Init(void)
{
    if (sPLLState != PLL_STATE_UNINIT)
        return;
    
    // Initialize the PLL PID controller.
    // Example parameters: Kp = 10.0, Ki = 50.0, Kd = 0.0, dt = 0.001 s, output limits = [-100, 100] Hz.
    PID_Init(&pidPLL, 10.0f, 50.0f, 0.0f, 0.001f, 0.0f, 0.05f, -100.0f, 100.0f);
    PID_Start(&pidPLL);
    
    // Initialize TIMER2.
    // Enable TIM2 clock.
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    
    // Use a default center frequency; this will be updated when PLL_Activate() is called.
    sCenterFrequency = 100000.0f; // e.g., 100 kHz default.
    uint32_t prescaler = 83; // Fixed prescaler => timer clock = 1 MHz.
    uint32_t arr = (uint32_t)(1000000.0f / sCenterFrequency) - 1;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        // Error handling.
    }
    HAL_TIM_Base_Start(&htim2);
    
    // Register the PLL callback with the impedance measurement module.
    ImpedanceMeasurement_RegisterCallback(PLL_ImpedanceCallback);
    
    sPLLState = PLL_STATE_READY;
}

void PLL_Start(void)
{
    if (sPLLState != PLL_STATE_READY)
        return;
    
    sPLLState = PLL_STATE_OPERATING;
}

void PLL_Stop(void)
{
    if (sPLLState != PLL_STATE_OPERATING)
        return;
    
    sPLLState = PLL_STATE_READY;
}

/**
 * @brief Activate the PLL with the given center frequency.
 *
 * This function sets the center frequency, updates TIMER2 accordingly,
 * and starts the PLL processing. The final frequency applied to TIMER2 is:
 *   finalFrequency = centerFrequency + (PID output).
 *
 * @param centerFrequency The desired center frequency in Hz.
 */
void PLL_Activate(float centerFrequency)
{
    // Set the center frequency.
    sCenterFrequency = centerFrequency;
    
    // Update TIMER2 to run at the center frequency initially.
    PLL_UpdateTimer2Frequency(sCenterFrequency);
    
    // Start PLL processing.
    PLL_Start();
}
