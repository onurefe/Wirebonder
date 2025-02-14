#ifndef FAST_IO_H
#define FAST_IO_H

#include "configuration.h"
#include "generic.h"         // Assuming this defines Bool_t, TRUE/FALSE, etc.

/* Exported types ----------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef *GPIOx;   // e.g. GPIOA, GPIOB, etc.
    uint16_t GPIO_Pin;     // e.g. GPIO_PIN_0, GPIO_PIN_1, etc.
    Bool_t inverted;
} FastIO_Pin_t;

/* Exported functions ------------------------------------------------------*/
extern Bool_t FastIO_Read  (FastIO_Pin_t *pin);
extern void  FastIO_Set    (FastIO_Pin_t *pin);
extern void  FastIO_Clear  (FastIO_Pin_t *pin);
extern void  FastIO_GetPinObject(GPIO_TypeDef *GPIOx, 
                                 uint16_t gpioPin, 
                                 Bool_t inverted, 
                                 FastIO_Pin_t *fastIOPin);

#endif // FAST_IO_H