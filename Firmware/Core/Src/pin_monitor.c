#include "configuration.h"
#include "timer.h"
#include "fast_io.h"
#include "pin_monitor.h"

/* Private variables -------------------------------------------------------*/
static PinMonitor_TransitionEventDelegate_t g_transitionEventDelegates[PIN_MONITOR_MAX_NUMBER_OF_MONITORED_PINS];
static FastIO_Pin_t g_monitoredPins[PIN_MONITOR_MAX_NUMBER_OF_MONITORED_PINS];
static uint8_t g_numOfMonitoredPins;

static State_t g_state = STATE_UNINIT;

static Bool_t g_pinStates[PIN_MONITOR_MAX_NUMBER_OF_MONITORED_PINS];
static uint16_t g_tickCounter;

/* Private functions -------------------------------------------------------*/
void timerTickDelegate(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_tickCounter++;
    if (g_tickCounter < (TIMER_TICK_FREQUENCY / PIN_MONITOR_SAMPLING_FREQUENCY))
    {
        return;
    }

    g_tickCounter = 0;

    PinMonitor_Level_t pin_state;
    PinMonitor_Transition_t transition;

    for (uint8_t i = 0; i < g_numOfMonitoredPins; i++)
    {
        pin_state = FastIO_ReadPin(&g_monitoredPins[i]) ? PIN_MONITOR_LEVEL_HIGH : PIN_MONITOR_LEVEL_LOW;
        transition = detectTransition(g_pinStates[i], pin_state);
        g_pinStates[i] = pin_state;

        if (transition != PIN_MONITOR_TRANSITION_NONE)
        {
            g_transitionEventDelegates[i](transition);
        }
    }
}

PinMonitor_Transition_t detectTransition(PinMonitor_Level_t lastPinState, PinMonitor_Level_t newPinState)
{
    if ((lastPinState == PIN_MONITOR_LEVEL_LOW) && (newPinState == PIN_MONITOR_LEVEL_HIGH))
    {
        return PIN_MONITOR_TRANSITION_LOW_TO_HIGH;
    }
    else if ((lastPinState == PIN_MONITOR_LEVEL_HIGH) && (newPinState == PIN_MONITOR_LEVEL_LOW))
    {
        return PIN_MONITOR_TRANSITION_HIGH_TO_LOW;
    }
    else
    {
        return PIN_MONITOR_TRANSITION_NONE;
    }
}

/* Exported functions ------------------------------------------------------*/
void PinMonitor_Init(void)
{
    if (g_state != STATE_UNINIT) 
    {
        return;
    }
    
    g_numOfMonitoredPins = 0;
    g_state = STATE_READY;
}

void PinMonitor_Register(GPIO_TypeDef *gpio, 
                         uint16_t gpioPin, 
                         PinMonitor_TransitionEventDelegate_t transitionEventDelegate)
{
    if (g_state != STATE_READY) 
    {
        return;
    } 

    g_transitionEventDelegates[g_numOfMonitoredPins] = transitionEventDelegate;
    
    g_monitoredPins[g_numOfMonitoredPins].GPIOx = gpio;
    g_monitoredPins[g_numOfMonitoredPins].GPIO_Pin = gpioPin;

    g_numOfMonitoredPins++;
}

void PinMonitor_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    for (uint16_t i = 0; i < g_numOfMonitoredPins; i++)
    {
        g_pinStates[i] = PIN_MONITOR_LEVEL_UNDETERMINED;
    }

    g_tickCounter = 0;
    
    Timer_Register(timerTickDelegate, NULL);

    g_state = STATE_OPERATING;
}

void PinMonitor_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    g_state = STATE_READY;
}

PinMonitor_Level_t PinMonitor_GetPinState(FastIO_Pin_t pin)
{
    for (uint8_t i = 0; i < PIN_MONITOR_MAX_NUMBER_OF_MONITORED_PINS; i++)
    {
        if ((g_monitoredPins[i].GPIOx == pin.GPIOx) && \
            (g_monitoredPins[i].GPIO_Pin == pin.GPIO_Pin)) 
        {
            return g_pinStates[i];
        }
    }

    return PIN_MONITOR_LEVEL_UNDETERMINED;
}