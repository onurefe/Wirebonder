#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

// Initialize the DWT cycle counter
extern void DWT_Delay_Init(void);

// Blocking delay in microseconds
extern void DWT_Delay_us(uint32_t us);

#endif /* DELAY_H */
