#include "fast_io.h"

Bool_t FastIO_Read(FastIO_Pin_t *pin)
{
    // Read the pin state using the HAL
    GPIO_PinState pinState = HAL_GPIO_ReadPin(pin->GPIOx, pin->GPIO_Pin);
    Bool_t logic_state = (pinState == GPIO_PIN_SET) ? TRUE : FALSE;
    if (pin->inverted)
    {
        return !logic_state;
    }
    else
    {
        return logic_state;
    }
}

void FastIO_Set(FastIO_Pin_t *pin)
{
    if (pin->inverted)
    {
        HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_SET);
    }
}

void FastIO_Clear(FastIO_Pin_t *pin)
{
    if (pin->inverted)
    {
        HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(pin->GPIOx, pin->GPIO_Pin, GPIO_PIN_RESET);
    }
}

void FastIO_GetPinObject(GPIO_TypeDef *GPIOx, uint16_t gpioPin, Bool_t inverted, FastIO_Pin_t *fastIOPin)
{
    // Simple “constructor” for the FastIO_Pin_t struct
    fastIOPin->GPIOx    = GPIOx;
    fastIOPin->GPIO_Pin = gpioPin;
    fastIOPin->inverted = inverted;
}
