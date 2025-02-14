#include "bonder.h"
#include "pin_monitor.h"
#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "timer_expire.h"
#include "pll.h"
#include "zmotor_control.h"
#include "force_coil_driver.h"
#include "complex.h"

static Bonder_State_t g_bonderState = BONDER_STATE_IDLE;

static float g_bonderForceCoilTrackingCurrent;
static float g_bonderForceCoilSearchingCurrent;
static float g_bonderForceCoilSettlingCurrent;
static float g_bonderForceCoilWeldingCurrent;

static float g_searchHeight; 
static float g_lowestOvertravel;
static float g_leavingHeight;

static float g_bondingEnergy;
static float g_centerFrequency;
static float g_maxBondingDuration;

static Bool_t g_moveToSearchHeightCmdReceived;
static Bool_t g_weldCmdReceived;
static Bool_t g_movedToHeight;
static Bool_t g_contactPinDisconnected;
static Bool_t g_settlingTimerExpired;
static Bool_t g_coolingTimerExpired;
static Bool_t g_impedanceScanningCompleted;
static Bool_t g_weldingCompleted;

static TimerExpire_Handle_t g_settlingTimer;
static TimerExpire_Handle_t g_coolingTimer;

static complexf g_voltagePhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
static complexf g_currentPhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
static complexf g_impedances[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];

static Bonder_StateChangedCallback_t g_callback;

/* Callback functions -------------------------------------------------*/
void contactSensorTransitionCallback(PinMonitor_Transition_t transition)
{
    if (transition == PIN_MONITOR_TRANSITION_LOW_TO_HIGH)
    {
        g_contactPinDisconnected = TRUE;
    }
}

void pllCallback(uint16_t eventId)
{
    if (eventId == PLL_BONDING_COMPLETED_EVENT_ID)
    {
        g_weldingCompleted = TRUE;
    }
    else if (eventId == PLL_INSUFFICIENT_BONDING_POWER_EVENT_ID)
    {
        // Handle this error!
    }
}

void timerExpiredCallback(void *handle)
{
    TimerExpire_Handle_t *casted_handle = (TimerExpire_Handle_t *)handle;
    if (casted_handle == &g_settlingTimer)
    {
        g_settlingTimerExpired = TRUE;
    }
    else if (casted_handle == &g_coolingTimer)
    {
        g_coolingTimerExpired = TRUE;
    }}

void impedanceScannerCallback(complexf *voltagePhasors, 
                              complexf *currentPhasors, 
                              complexf *impedances)
{
    g_impedanceScanningCompleted = TRUE;
}

static void zmotorControlCallback(uint16_t eventId)
{
    if (eventId == ZMOTOR_CONTROL_SETPOINT_ACHIEVED_EVENT_ID)
    {
        g_movedToHeight = TRUE;
    }
}

/* Private functions --------------------------------------------------*/
void bonderStateIdle(void)
{
    if (g_moveToSearchHeightCmdReceived)
    {
        g_moveToSearchHeightCmdReceived = FALSE;

        ForceCoilDriver_SetCurrentSetpoint(g_bonderForceCoilTrackingCurrent);
        ZMotorControl_SetPositionSetpoint(g_searchHeight);

        g_callback(BONDER_STATE_IDLE, BONDER_STATE_MOVING_TO_SEARCH_HEIGHT);
        g_bonderState = BONDER_STATE_MOVING_TO_SEARCH_HEIGHT;
    }
}

void bonderStateLevelingDown(void)
{
    if (g_movedToHeight && g_weldCmdReceived)
    {
        g_movedToHeight = FALSE;
        g_weldCmdReceived = FALSE;

        ForceCoilDriver_SetCurrentSetpoint(g_bonderForceCoilSearchingCurrent);
        ZMotorControl_SetPositionSetpoint(g_lowestOvertravel);

        g_callback(BONDER_STATE_MOVING_TO_SEARCH_HEIGHT, BONDER_STATE_SEARCHING);
        g_bonderState = BONDER_STATE_SEARCHING;
    }
}

void bonderStateSearching(void)
{
    if (g_contactPinDisconnected)
    {
        g_contactPinDisconnected = FALSE;

        TimerExpire_Activate(&g_settlingTimer);
        ForceCoilDriver_SetCurrentSetpoint(g_bonderForceCoilSettlingCurrent);
        
        g_callback(BONDER_STATE_SEARCHING, BONDER_STATE_SETTLING);
        g_bonderState = BONDER_STATE_SETTLING;
    }
}

void bonderStateSettling(void)
{
    if (g_settlingTimerExpired)
    {
        g_settlingTimerExpired = FALSE;

        UsImpedanceScanner_Start(g_voltagePhasors, 
                                 g_currentPhasors, 
                                 g_impedances, 
                                 impedanceScannerCallback);

        g_callback(BONDER_STATE_SETTLING, BONDER_STATE_SCANNING_IMPEDANCE);
        g_bonderState = BONDER_STATE_SCANNING_IMPEDANCE;
    }
}

void bonderStateScanningImpedance(void)
{
    if (g_impedanceScanningCompleted)
    {
        g_impedanceScanningCompleted = FALSE;
        
        g_centerFrequency = calculateCenterFrequency();
        Pll_Start(g_centerFrequency, 
                  g_bondingEnergy, 
                  g_maxBondingDuration, 
                  pllCallback);

        g_callback(BONDER_STATE_SCANNING_IMPEDANCE, BONDER_STATE_WELDING);
        g_bonderState = BONDER_STATE_WELDING;
    }
}

void bonderStateWelding(void)
{
    if (g_weldingCompleted)
    {
        g_weldingCompleted = FALSE;

        TimerExpire_Activate(&g_coolingTimer);

        g_callback(BONDER_STATE_WELDING, BONDER_STATE_COOLING);
        g_bonderState = BONDER_STATE_COOLING;
    }
}

void bonderStateCooling(void)
{
    if (g_coolingTimerExpired)
    {
        g_coolingTimerExpired = FALSE;

        ZMotorControl_SetPositionSetpoint(g_leavingHeight);

        g_callback(BONDER_STATE_COOLING, BONDER_STATE_LEAVING);
        g_bonderState = BONDER_STATE_LEAVING;
    }
}

void bonderStateLevelingUp(void)
{
    if (g_movedToHeight)
    {
        g_movedToHeight = FALSE;

        g_callback(BONDER_STATE_LEAVING, BONDER_STATE_IDLE);
        g_bonderState = BONDER_STATE_IDLE;
    }
}

/* Exported functions -------------------------------------------------*/
void Bonder_Init(void)
{
    PinMonitor_Register(CONTACT_SENSORS_TIP_GPIO_Port, 
                        CONTACT_SENSORS_TIP_Pin, 
                        contactSensorTransitionCallback);
}

void Bonder_Start(void)
{
}

void Bonder_MoveToSearchHeight(void)
{
}

void Bonder_Weld(void)
{
    g_weldCmdReceived = TRUE;
}

void Bonder_Execute(void)
{
    if (g_bonderState == BONDER_STATE_IDLE)
    {
        bonderStateIdle();
    }
    else if (g_bonderState == BONDER_STATE_MOVING_TO_SEARCH_HEIGHT)
    {
        bonderStateLevelingDown();
    }
    else if (g_bonderState == BONDER_STATE_SEARCHING)
    {
        bonderStateSearching();
    }
    else if (g_bonderState == BONDER_STATE_SETTLING)
    {
        bonderStateSettling();
    }
    else if (g_bonderState == BONDER_STATE_WELDING)
    {
        bonderStateWelding();
    }
    else if (g_bonderState == BONDER_STATE_COOLING)
    {
        bonderStateCooling();
    }
    else if (g_bonderState == BONDER_STATE_LEAVING)
    {
        bonderStateLevelingUp();
    }
}

void Bonder_Stop(void)
{
}