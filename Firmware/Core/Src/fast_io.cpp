#include "fast_io.hpp"

FastIO::FastIO(GPIO_TypeDef *GPIOx, uint16_t gpioPin, Bool_t inverted)
{
    this->GPIOx = GPIOx;
    this->gpioPin = gpioPin;
    this->inverted = inverted;
}

Bool_t FastIO::read()
{
    // Read the pin state using the HAL
    GPIO_PinState pinState = HAL_GPIO_ReadPin(this->GPIOx, this->gpioPin);
    Bool_t logic_state = (pinState == GPIO_PIN_SET) ? TRUE : FALSE;
    
    if (this->inverted) {
        return !logic_state;
    } else {
        return logic_state;
    }
}

void FastIO::set()
{
    if (this->inverted) {
        HAL_GPIO_WritePin(this->GPIOx, this->gpioPin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(this->GPIOx, this->gpioPin, GPIO_PIN_SET);
    }
}

void FastIO::clear()
{
    if (this->inverted) {
        HAL_GPIO_WritePin(this->GPIOx, this->gpioPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(this->GPIOx, this->gpioPin, GPIO_PIN_RESET);
    }
}
