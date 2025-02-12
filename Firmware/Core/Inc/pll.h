#ifndef PLL_H
#define PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Events ----------------------------------------------------*/
#define PLL_BONDING_COMPLETED_EVENT_ID 0 
#define PLL_INSUFFICIENT_BONDING_POWER_EVENT_ID 1

/* Exported types --------------------------------------------*/
typedef void (*Pll_Callback_t)(uint16_t eventId);

/* Exported functions ----------------------------------------*/
void Pll_Init(void);
void Pll_Start(float centerFrequency, 
               float bondingEnergyInJoules, 
               float maxBondingDuration, 
               Pll_Callback_t callback);

void Pll_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* PLL_H */
