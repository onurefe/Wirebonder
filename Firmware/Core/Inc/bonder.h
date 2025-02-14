#ifndef BONDER

#include "fast_io.h"
#include "configuration.h"
#include "generic.h"

/* Exported types --------------------------------------------*/
enum
{
    BONDER_STATE_IDLE = 0,
    BONDER_STATE_MOVING_TO_SEARCH_HEIGHT = 1,
    BONDER_STATE_SEARCHING = 2,
    BONDER_STATE_SETTLING = 3,
    BONDER_STATE_SCANNING_IMPEDANCE = 4,
    BONDER_STATE_WELDING = 5,
    BONDER_STATE_COOLING = 6,
    BONDER_STATE_LEAVING = 7
};
typedef uint8_t Bonder_State_t;

typedef void (*Bonder_StateChangedCallback_t)(Bonder_State_t previousState, 
                                              Bonder_State_t newState);

/* Exported functions ----------------------------------------*/
void Bonder_Init(void);
void Bonder_Start(void);
void Bonder_Execute(void);
void Bonder_Stop(void);

#endif
