#ifndef FAST_IO_HPP
#define FAST_IO_HPP

#include "configuration.h"
#include "generic.h"         // Assuming this defines Bool_t, TRUE/FALSE, etc.

class FastIO
{
    private:
        GPIO_TypeDef *GPIOx;
        uint16_t gpioPin;
        Bool_t inverted;

    public:
        FastIO(GPIO_TypeDef *GPIOx, uint16_t gpioPin, Bool_t inverted);

        Bool_t read();
        void set();
        void clear();
};

#endif // FAST_IO_HPP