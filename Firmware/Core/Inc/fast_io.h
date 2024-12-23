#ifndef FAST_IO_H
#define FAST_IO_H

#include "stm32f4xx_hal.h"
#include "generic.h"         // Assuming this defines Bool_t, TRUE/FALSE, etc.

#define FASTIO_INPUT ((uint8_t)0x00)
#define FASTIO_OUTPUT ((uint8_t)0x01)

/* Exported types ----------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef *GPIOx;   // e.g. GPIOA, GPIOB, etc.
    uint16_t GPIO_Pin;     // e.g. GPIO_PIN_0, GPIO_PIN_1, etc.
} FastIO_Pin_t;

/* Exported functions ------------------------------------------------------*/
extern void  FastIO_PinMode   (FastIO_Pin_t *pin, uint8_t mode);
extern Bool_t FastIO_ReadPin  (FastIO_Pin_t *pin);
extern void  FastIO_SetPin    (FastIO_Pin_t *pin);
extern void  FastIO_ClearPin  (FastIO_Pin_t *pin);
extern void  FastIO_GetPinObject(GPIO_TypeDef *GPIOx, uint16_t gpioPin, FastIO_Pin_t *fastIOPin);

#endif // FAST_IO_H