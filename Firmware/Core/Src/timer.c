#include "stm32f4xx_hal.h"

/* Private definitions -----------------------------------------------------*/
#define TIMER_MAX_NUM_DELEGATES  5
#define STATE_UNINIT 0
#define STATE_READY  1
#define STATE_OPERATING 2

typedef void (*Timer2_TickDelegate_t)(void);

/* Private variables -------------------------------------------------------*/
static uint8_t State = STATE_UNINIT;
static Timer2_TickDelegate_t RegisteredDelegates[TIMER_MAX_NUM_DELEGATES] = {NULL};
static uint8_t NumOfDelegates = 0;
static uint32_t Ticks = 0;

/* Handle for TIM2 */
static TIM_HandleTypeDef htim2;

/* Exported functions ------------------------------------------------------*/
void Timer2_Init()
{
    NumOfDelegates = 0;
    Ticks = 0;
    State = STATE_READY;
    
    // Initialize TIM2 for a specified interrupt frequency (example: 1kHz)
    // Adjust as needed based on your system clock and desired frequency.
    
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    /* Compute prescaler and period for desired frequency.
       Assume TIMER2_ISR_FREQUENCY and a known timer clock.
       For example:
       Timer clock = 90 MHz (depends on your configuration)
       Desired frequency = TIMER2_ISR_FREQUENCY
       
       Let's say TIMER2_ISR_FREQUENCY = 1000 Hz
       
       Prescaler = 89 (to get 1 MHz tick)
       Period = 999 (1 MHz / 1000 = 1kHz)
    */
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 89;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        // Initialization Error
        // Handle error
    }

    // Configure the NVIC for TIM2 interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void Timer2_Register(Timer2_TickDelegate_t timerDelegate)
{
    if (State != STATE_READY) {
        return;
    }

    if (NumOfDelegates < TIMER_MAX_NUM_DELEGATES) {
        RegisteredDelegates[NumOfDelegates++] = timerDelegate;
    }
}

void Timer2_Start()
{
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        // Start error
        // Handle error
    }
    State = STATE_OPERATING;
}

void Timer2_Stop()
{
    if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) {
        // Stop error
        // Handle error
    }
    State = STATE_READY;
}

uint32_t Timer2_GetTicks()
{
    return Ticks;
}

/* The HAL timer interrupt callback ----------------------------------------*/
/* This function is called inside HAL_TIM_IRQHandler when TIM2 update event occurs */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (State == STATE_OPERATING) {
            Ticks++;
            for (uint8_t i = 0; i < NumOfDelegates; i++) {
                if (RegisteredDelegates[i] != NULL) {
                    RegisteredDelegates[i]();
                }
            }
        }
    }
}

/* STM32F4xx HAL TIM2 IRQ Handler ------------------------------------------*/
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}
