#include "fast_io.h"

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
