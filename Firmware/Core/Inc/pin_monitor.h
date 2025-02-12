#ifndef PIN_MONITOR
#define PIN_MONITOR

#include "fast_io.h"
#include "configuration.h"
#include "generic.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    PIN_MONITOR_TRANSITION_NONE = 0,
    PIN_MONITOR_TRANSITION_LOW_TO_HIGH,
    PIN_MONITOR_TRANSITION_HIGH_TO_LOW
};
typedef uint8_t PinMonitor_Transition_t;

enum
{
    PIN_MONITOR_LEVEL_LOW = 0,
    PIN_MONITOR_LEVEL_HIGH = 1,
    PIN_MONITOR_LEVEL_UNDETERMINED = 2,
};
typedef uint8_t PinMonitor_Level_t;

typedef void (*PinMonitor_TransitionEventDelegate_t)(PinMonitor_Transition_t transition);

/* Exported functions ------------------------------------------------------*/
void PinMonitor_Init(void);
void PinMonitor_Register(GPIO_TypeDef *gpio, 
                         uint16_t gpioPin, 
                         PinMonitor_TransitionEventDelegate_t transitionEventDelegate);
void PinMonitor_Start(void);
void PinMonitor_Stop(void);
PinMonitor_Level_t PinMonitor_GetPinState(FastIO_Pin_t pin);

#endif