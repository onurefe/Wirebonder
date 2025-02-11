#ifndef US_IMPEDANCE_SCANNER_H
#define US_IMPEDANCE_SCANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "complex.h"
#include <stdint.h>

/* Exported types --------------------------------------------*/
typedef void (*UsImpedanceScanner_Callback_t)(complexf *voltagePhasors, 
                                              complexf *currentPhasors, 
                                              complexf *impedances);

/* Exported functions ----------------------------------------*/
void UsImpedanceScanner_Init(void);

void UsImpedanceScanner_Start(complexf *voltagePhasors, 
                              complexf *currentPhasors, 
                              complexf *impedances, 
                              UsImpedanceScanner_Callback_t callback);

void UsImpedanceScanner_Stop(void);

#ifdef __cplusplus
}
#endif

#endif