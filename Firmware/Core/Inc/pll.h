#ifndef PLL_H
#define PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported functions ----------------------------------------*/
void PLL_Init(void);
void PLL_Start(float centerFrequency);
void PLL_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* PLL_H */
