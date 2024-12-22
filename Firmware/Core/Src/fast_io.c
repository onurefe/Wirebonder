#include "fast_io.h"

void FastIO_PinMode(FastIO_Pin_t *pin, uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Specify which pin we’re configuring
    GPIO_InitStruct.Pin = pin->GPIO_Pin;

    // Interpret `mode`: 
    //   For example, let “0” = Input, “1” = Output
    if (mode == FASTIO_INPUT)
    {
        // Input floating (or pull-up/down depending on your need)
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; 
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else
    {
        // Output push-pull
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }

    // Initialize the pin using the HAL
    HAL_GPIO_Init(pin->GPIOx, &GPIO_InitStruct);
}

Bool_t FastIO_ReadPin(FastIO_Pin_t *pin)
{
    // Read the pin state using the HAL
    GPIO_PinState pinState = HAL_GPIO_ReadPin(pin->GPIOx, pin->GPIO_Pin);
    return (pinState == GPIO_PIN_SET) ? TRUE : FALSE;
}

void FastIO_SetPin(FastIO_Pin_t *pin)
{
    // Set pin output to high
    HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_SET);
}

void FastIO_ClearPin(FastIO_Pin_t *pin)
{
    // Set pin output to low
    HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_RESET);
}

void FastIO_GetPinObject(GPIO_TypeDef *GPIOx, uint16_t gpioPin, FastIO_Pin_t *fastIOPin)
{
    // Simple “constructor” for the FastIO_Pin_t struct
    fastIOPin->GPIOx    = GPIOx;
    fastIOPin->GPIO_Pin = gpioPin;
}
