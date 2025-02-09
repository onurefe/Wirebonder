#ifndef TACHOMETER_H
#define TACHOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*Tachometer_MeasurementCallback_t)(float velocity);

void Tachometer_Init(void);
void Tachometer_Start(void);
void Tachometer_Stop(void);
void Tachometer_RegisterCallback(Tachometer_MeasurementCallback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* TACHOMETER_H */
