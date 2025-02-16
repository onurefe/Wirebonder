#include "bonder.h"
#include "pin_monitor.h"
#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "timer_expire.h"
#include "pll.h"
#include "zmotor_controller.h"
#include "force_coil_driver.h"
#include "complex.h"
#include "solenoid_driver.h"
#include "router.h"

/* Private variables -----------------------------------------------------*/
/* Welding configuration variables. */
static float g_forceCoilIdleCurrent;
static float g_forceCoilSearchingCurrent;
static float g_forceCoilSettlingCurrent;
static float g_forceCoilWeldingCurrent;

static float g_loopHeight;
static float g_resetHeight;
static float g_searchHeight; 
static float g_kinkHeight;
static float g_lowestOvertravel;

static float g_tailDisplacement;
static float g_tearDisplacement;
static float g_yReverseDisplacement;
static float g_yStepbackDisplacement;

static float g_bondingEnergy;
static float g_centerFrequency;
static float g_maxBondingDuration;

static float g_settlingTime;
static float g_coolingTime;
static float g_tailRestoreDelay;
static float g_yRestoreDelay; 

/* State variables. */
static State_t g_state = STATE_UNINIT;
static Bool_t g_firstLoop;
static Bonder_OperationState_t g_operationState = BONDER_OPERATION_STATE_IDLE;

/* Flags. */
static Bool_t g_zMoveCompleted;
static Bool_t g_tMoveCompleted;
static Bool_t g_yMoveCompleted;
static Bool_t g_forceCoilCurrentSettled;
static Bool_t g_contactPinConnected;
static Bool_t g_contactPinDisconnected;
static Bool_t g_settlingTimerExpired;
static Bool_t g_coolingTimerExpired;
static Bool_t g_impedanceScanningCompleted;
static Bool_t g_usPowerTransferred;
static Bool_t g_clampOpened;
static Bool_t g_clampClosed;
static Bool_t g_rightButtonPressed;
static Bool_t g_rightButtonReleased;
static Bool_t g_restoreTailTimerExpired;
static Bool_t g_restoreYPositionTimerExpired;

/* Callbacks. */
static Bonder_ErrorOccurredCallback_t g_errorOccurredCallback;
static Bonder_StateChangedCallback_t g_stateChangedCallback;

/* Modules. */
static SolenoidDriver_Handle_t g_clampSolenoid;
static Router_Handle_t g_tAxisRouter;
static Router_Handle_t g_yAxisRouter;

/* Timers. */
static TimerExpire_Handle_t g_settlingTimer;
static TimerExpire_Handle_t g_coolingTimer;
static TimerExpire_Handle_t g_restoreYPositionTimer;
static TimerExpire_Handle_t g_restoreTailTimer;

/* Measurement variables. */
static complexf g_voltagePhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
static complexf g_currentPhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
static complexf g_impedances[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];

/* Callback functions -------------------------------------------------*/
void solenoidDriverPositionChangedCallback(void *solenoidHandle, 
                                                  SolenoidDriver_Position_t previousPosition,
                                                  SolenoidDriver_Position_t newPosition)
{
    (void)solenoidHandle;
    (void)previousPosition;

    if (newPosition == SOLENOID_DRIVER_POSITION_LOW)
    {
        g_clampOpened = TRUE;
    }

    if (newPosition == SOLENOID_DRIVER_POSITION_HIGH)
    {
        g_clampClosed = TRUE;
    }
}

static void contactSensorTransitionCallback(PinMonitor_Transition_t transition)
{
    if (transition == PIN_MONITOR_TRANSITION_LOW_TO_HIGH)
    {
        g_contactPinDisconnected = TRUE;
    }
}

static void mouseRightButtonTransitionCallback(PinMonitor_Transition_t transition)
{
    if (transition == PIN_MONITOR_TRANSITION_LOW_TO_HIGH)
    {
        g_rightButtonPressed = TRUE;
    }
    else if (transition == PIN_MONITOR_TRANSITION_HIGH_TO_LOW)
    {
        g_rightButtonReleased = TRUE;
    }
}

static void pllCallback(uint16_t eventId)
{
    if (eventId == PLL_BONDING_COMPLETED_EVENT_ID)
    {
        g_usPowerTransferred = TRUE;
    }
    else if (eventId == PLL_INSUFFICIENT_BONDING_POWER_EVENT_ID)
    {
        /* Add more sophisticated error handling algorithm. */
        g_errorOccurredCallback(BONDER_ERROR_INSUFFICIENT_BONDING_POWER);
        Bonder_Stop();
    }
}

static void forceCoilCallback(uint16_t eventId)
{
    if (eventId == FORCE_COIL_SETPOINT_ACHIEVED_EVENT_ID)
    {
        g_forceCoilCurrentSettled = TRUE;
    }
    else if (eventId == FORCE_COIL_UNABLE_TO_SET_CURRENT_EVENT_ID)
    {
        g_errorOccurredCallback(BONDER_ERROR_UNABLE_TO_SET_FORCE_COIL_CURRENT);
        // This reports error, but there aren't any additional control. 
    }
}

static void settingTimerCallback(void *handle)
{
    (void)handle;
    g_settlingTimerExpired = TRUE;
}

static void coolingTimerCallback(void *handle)
{
    (void)handle;
    g_coolingTimerExpired = TRUE;
}

static void restoreTailTimerCallback(void *handle)
{
    (void)handle;
    g_restoreTailTimerExpired = TRUE;
}

static void restoreYPositionTimerCallback(void *handle)
{
    (void)handle;
    g_restoreYPositionTimerExpired = TRUE;
}

static void impedanceScannerCallback(complexf *voltagePhasors, 
                                     complexf *currentPhasors, 
                                     complexf *impedances)
{
    g_impedanceScanningCompleted = TRUE;
}

static void ZMotorControllerCallback(uint16_t eventId)
{
    if (eventId == ZMOTOR_CONTROLLER_SETPOINT_ACHIEVED_EVENT_ID)
    {
        g_zMoveCompleted = TRUE;
    }
    else if (eventId == ZMOTOR_CONTROLLER_UNABLE_SET_POSITION_EVENT_ID)
    {
        g_errorOccurredCallback(BONDER_ERROR_UNABLE_TO_SET_POSITION);
        // This reports error, but there aren't any additional control. 
    }
}

static void yAxisRouterCallback(void)
{
    g_yMoveCompleted = TRUE;
}

static void tAxisRouterCallback(void)
{
    g_tMoveCompleted = TRUE;
}

/* Private functions --------------------------------------------------*/
void clearFlags(void)
{
    g_tMoveCompleted = FALSE;
    g_yMoveCompleted = FALSE;
    g_zMoveCompleted = FALSE;
    g_forceCoilCurrentSettled = FALSE;

    
    
    g_contactPinConnected = FALSE;  
    g_contactPinDisconnected = FALSE;
    g_settlingTimerExpired = FALSE;
    g_coolingTimerExpired = FALSE;
    g_impedanceScanningCompleted = FALSE;
    g_usPowerTransferred = FALSE;

    g_clampOpened = FALSE;
    g_clampClosed = FALSE;
}

void stateIdle(void)
{
    if (g_rightButtonPressed)
    {
        clearFlags();

        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilIdleCurrent);
        ZMotorController_SetPositionSetpoint(g_searchHeight);

        g_stateChangedCallback(BONDER_OPERATION_STATE_IDLE, 
                               BONDER_OPERATION_STATE_MOVING_TO_SEARCH_HEIGHT);

        g_operationState = BONDER_OPERATION_STATE_MOVING_TO_SEARCH_HEIGHT;
    }
}

void stateLevelingDown(void)
{
    if (g_zMoveCompleted && g_forceCoilCurrentSettled && g_rightButtonReleased)
    {
        clearFlags();

        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilSearchingCurrent);
        ZMotorController_SetPositionSetpoint(g_lowestOvertravel);

        g_stateChangedCallback(BONDER_OPERATION_STATE_MOVING_TO_SEARCH_HEIGHT, 
                               BONDER_OPERATION_STATE_SEARCHING);

        g_operationState = BONDER_OPERATION_STATE_SEARCHING;
    }
}

void stateSearching(void)
{
    if (g_contactPinDisconnected && g_forceCoilCurrentSettled)
    {
        clearFlags();

        TimerExpire_Activate(&g_settlingTimer);
        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilSettlingCurrent);
        
        g_stateChangedCallback(BONDER_OPERATION_STATE_SEARCHING, 
                               BONDER_OPERATION_STATE_SETTLING);

        g_operationState = BONDER_OPERATION_STATE_SETTLING;
    }
}

void stateSettling(void)
{
    if (g_settlingTimerExpired && g_forceCoilCurrentSettled)
    {
        clearFlags();

        UsImpedanceScanner_Start(g_voltagePhasors, 
                                 g_currentPhasors, 
                                 g_impedances, 
                                 impedanceScannerCallback);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SETTLING, 
                               BONDER_OPERATION_STATE_SCANNING_IMPEDANCE);

        g_operationState = BONDER_OPERATION_STATE_SCANNING_IMPEDANCE;
    }
}

void stateScanningImpedance(void)
{
    if (g_impedanceScanningCompleted)
    {
        clearFlags();
        
        g_centerFrequency = calculateCenterFrequency();
        Pll_Start(g_centerFrequency, 
                  g_bondingEnergy, 
                  g_maxBondingDuration, 
                  pllCallback);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SCANNING_IMPEDANCE, 
                               BONDER_OPERATION_STATE_WELDING);

        g_operationState = BONDER_OPERATION_STATE_WELDING;
    }
}

void stateWelding(void)
{
    if (g_usPowerTransferred)
    {
        clearFlags();

        TimerExpire_Activate(&g_coolingTimer);
        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilIdleCurrent);
        SolenoidDriver_SetLowPosition(&g_clampSolenoid);

        g_stateChangedCallback(BONDER_OPERATION_STATE_WELDING, 
                               BONDER_OPERATION_STATE_COOLING);

        g_operationState = BONDER_OPERATION_STATE_COOLING;
    }
}

void stateCooling(void)
{
    if (g_coolingTimerExpired && g_forceCoilCurrentSettled && g_clampOpened)
    {
        clearFlags();

        ZMotorController_SetPositionSetpoint(g_kinkHeight);
        Router_Append(&g_tAxisRouter, g_tailDisplacement);
            
        g_stateChangedCallback(BONDER_OPERATION_STATE_COOLING, 
                              BONDER_OPERATION_STATE_FORMING_LOOP_T_MOVE);

        g_operationState = BONDER_OPERATION_STATE_FORMING_LOOP_T_MOVE;
    }
}

void stateFormingLoopTMove(void)
{
    if (g_zMoveCompleted && g_tMoveCompleted)
    {
        clearFlags();

        Router_Append(&g_yAxisRouter, g_yReverseDisplacement);

        g_stateChangedCallback(BONDER_OPERATION_STATE_FORMING_LOOP_T_MOVE, 
                               BONDER_OPERATION_STATE_FORMING_LOOP_Y_REVERSE_MOVE);

        g_operationState = BONDER_OPERATION_STATE_FORMING_LOOP_Y_REVERSE_MOVE;
    }
}

void stateFormingLoopYReverseMove(void)
{
    if (g_yMoveCompleted)
    {
        clearFlags();

        ZMotorController_SetPositionSetpoint(g_loopHeight);

        g_stateChangedCallback(BONDER_OPERATION_STATE_FORMING_LOOP_Y_REVERSE_MOVE, 
                               BONDER_OPERATION_STATE_FORMING_LOOP_Z_MOVE);

        g_operationState = BONDER_OPERATION_STATE_FORMING_LOOP_Z_MOVE;
    }
}

void stateFormingLoopZMove(void)
{
    if (g_zMoveCompleted)
    {
        clearFlags();

        g_stateChangedCallback(BONDER_OPERATION_STATE_FORMING_LOOP_Z_MOVE, 
                               BONDER_OPERATION_STATE_WAITING_SECOND_WELD_TRIGGER);

        g_operationState = BONDER_OPERATION_STATE_WAITING_SECOND_WELD_TRIGGER;
    }
}

void stateWaitingSecondWeldTrigger(void)
{
    if (g_rightButtonPressed)
    {
        clearFlags();

        SolenoidDriver_SetLowPosition(&g_clampSolenoid);

        g_stateChangedCallback(BONDER_OPERATION_STATE_WAITING_SECOND_WELD_TRIGGER, 
                               BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_CLOSING_CLAMP);

        g_operationState = BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_CLOSING_CLAMP;
    }    
}

void statePreparingSecondWeldClosingClamp(void)
{
    if (g_clampClosed)
    {
        clearFlags();
        Router_Append(&g_yAxisRouter, g_yStepbackDisplacement);

        g_stateChangedCallback(BONDER_OPERATION_STATE_WAITING_SECOND_WELD_TRIGGER, 
                               BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_STEPPING_BACK);

        g_operationState = BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_STEPPING_BACK;
    }
}

void statePreparingSecondWeldSteppingBack(void)
{
    if (g_yMoveCompleted)
    {
        clearFlags();
        SolenoidDriver_SetHighPosition(&g_clampSolenoid);

        g_stateChangedCallback(BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_STEPPING_BACK, 
                               BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_OPENING_CLAMP);

        g_operationState = BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_OPENING_CLAMP;
    }
}

void statePreparingSecondWeldOpeningClamp(void)
{
    if (g_clampOpened)
    {
        clearFlags();

        g_stateChangedCallback(BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_STEPPING_BACK, 
                   BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_OPENING_CLAMP);

        g_operationState = BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_OPENING_CLAMP;
    }
}

void stateSecondWeldMovingToSearchHeight(void)
{
    if (g_zMoveCompleted && g_forceCoilCurrentSettled && g_rightButtonReleased)
    {
        clearFlags();

        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilSearchingCurrent);
        ZMotorController_SetPositionSetpoint(g_lowestOvertravel);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_MOVING_TO_SEARCH_HEIGHT, 
                               BONDER_OPERATION_STATE_SECOND_WELD_SEARCHING);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_SEARCHING;
    }
}

void stateSecondWeldSearching(void)
{
    if (g_contactPinDisconnected && g_forceCoilCurrentSettled)
    {
        clearFlags();

        TimerExpire_Activate(&g_settlingTimer);
        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilSettlingCurrent);
        
        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_SEARCHING, 
                               BONDER_OPERATION_STATE_SECOND_WELD_SETTLING);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_SETTLING;
    }
}

void stateSecondWeldSettling(void)
{
    if (g_settlingTimerExpired && g_forceCoilCurrentSettled)
    {
        clearFlags();

        UsImpedanceScanner_Start(g_voltagePhasors, 
                                 g_currentPhasors, 
                                 g_impedances, 
                                 impedanceScannerCallback);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_SETTLING, 
                               BONDER_OPERATION_STATE_SCANNING_IMPEDANCE);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_SCANNING_IMPEDANCE;
    }
}

void stateSecondWeldScanningImpedance(void)
{
    if (g_impedanceScanningCompleted)
    {
        clearFlags();
        
        g_centerFrequency = calculateCenterFrequency();
        Pll_Start(g_centerFrequency, 
                  g_bondingEnergy, 
                  g_maxBondingDuration, 
                  pllCallback);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_SCANNING_IMPEDANCE, 
                               BONDER_OPERATION_STATE_SECOND_WELD_WELDING);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_WELDING;
    }
}

void stateSecondWeldWelding(void)
{
    if (g_usPowerTransferred)
    {
        clearFlags();

        TimerExpire_Activate(&g_coolingTimer);
        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilIdleCurrent);
        SolenoidDriver_SetLowPosition(&g_clampSolenoid);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_WELDING, 
                               BONDER_OPERATION_STATE_SECOND_WELD_COOLING);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_COOLING;
    }
}

void stateSecondWeldCooling(void)
{
    if (g_coolingTimerExpired && g_forceCoilCurrentSettled && g_clampClosed)
    {
        clearFlags();

        Router_Append(&g_tAxisRouter, g_tearDisplacement);
            
        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_COOLING, 
                               BONDER_OPERATION_STATE_SECOND_WELD_TEARING_TMOVE);

        g_operationState = BONDER_OPERATION_STATE_SECOND_WELD_TEARING_TMOVE;
    }
}

void stateSecondWeldTearingTmove(void)
{
    if (g_tMoveCompleted)
    {
        clearFlags();

        ZMotorController_SetPositionSetpoint(g_resetHeight);
        TimerExpire_Activate(&g_restoreTailTimer);

        g_stateChangedCallback(BONDER_OPERATION_STATE_SECOND_WELD_TEARING_TMOVE, 
                               BONDER_OPERATION_STATE_RESTORING_ZMOVE);

        g_operationState = BONDER_OPERATION_STATE_RESTORING_ZMOVE;
    }
}

void stateRestoringZMove(void)
{
    if (g_contactPinConnected && g_restoreTailTimerExpired)
    {
        g_contactPinConnected = FALSE;
        g_restoreTailTimerExpired = FALSE;

        UsImpedanceScanner_Start(g_voltagePhasors, 
                                 g_currentPhasors, 
                                 g_impedances, 
                                 impedanceScannerCallback);

        g_stateChangedCallback(BONDER_OPERATION_STATE_RESTORING_ZMOVE, 
                               BONDER_OPERATION_STATE_RESTORING_ZMOVE_WITH_SCANNING_IMPEDANCE);

        g_operationState = BONDER_OPERATION_STATE_RESTORING_ZMOVE_WITH_SCANNING_IMPEDANCE;
    }
}

void stateRestoringZMoveWithScanningImpedance(void)
{
    if (g_impedanceScanningCompleted)
    {
        g_impedanceScanningCompleted = FALSE;

        g_centerFrequency = calculateCenterFrequency();
        
        Pll_Start(g_centerFrequency, 
                  g_bondingEnergy, 
                  g_maxBondingDuration, 
                  pllCallback);
        
        Router_Append(&g_tAxisRouter, g_tailDisplacement);
        TimerExpire_Activate(&g_restoreYPositionTimer);

        g_stateChangedCallback(BONDER_OPERATION_STATE_RESTORING_ZMOVE_WITH_SCANNING_IMPEDANCE, 
                               BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_TMOVE_WITH_US_DRIVE);

        g_operationState = BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_TMOVE_WITH_US_DRIVE;
    }
}

void stateRestoringZMoveAndTMoveWithUsDrive(void)
{
    if (g_restoreYPositionTimerExpired)
    {
        g_restoreYPositionTimerExpired = FALSE;
        
        Router_Append(&g_yAxisRouter, g_yStepbackDisplacement);
        g_stateChangedCallback(BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_TMOVE_WITH_US_DRIVE, 
                               BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_YMOVE);

        g_operationState = BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_YMOVE;
    }   
}

void stateRestoringZMoveAndYMove(void)
{
    if (g_tMoveCompleted && g_yMoveCompleted && g_zMoveCompleted && g_usPowerTransferred)
    {
        clearFlags();
        
        g_stateChangedCallback(BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_YMOVE, 
                               BONDER_OPERATION_STATE_IDLE);

        g_operationState = BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_YMOVE;
    }   
}

/* Exported functions -------------------------------------------------*/
void Bonder_Init(Bonder_StateChangedCallback_t stateChangedCb, 
                 Bonder_ErrorOccurredCallback_t errorOccurredCb)
{
    if (g_state != STATE_UNINIT)
    {
        return;
    }

    /* Register timers. */
    TimerExpire_Register(&g_settlingTimer, 
                         TRUE, 
                         g_settlingTime, 
                         settingTimerCallback);

    TimerExpire_Register(&g_coolingTimer, 
                         TRUE, 
                         g_coolingTime, 
                         coolingTimerCallback);
    
    TimerExpire_Register(&g_restoreTailTimer, 
                         TRUE, 
                         g_tailRestoreDelay, 
                         restoreYPositionTimerCallback);

    TimerExpire_Register(&g_restoreYPositionTimer, 
                         TRUE, 
                         g_yRestoreDelay, 
                         restoreYPositionTimerCallback);

    /* Register pins & logic inputs. */
    PinMonitor_Register(CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port, 
                        CONTACT_SENSORS_MOUSE_RIGHT_Pin, 
                        mouseRightButtonTransitionCallback);

    PinMonitor_Register(CONTACT_SENSORS_TIP_GPIO_Port, 
                        CONTACT_SENSORS_TIP_Pin, 
                        contactSensorTransitionCallback);

    /* Register solenoid driver. */
    SolenoidDriver_Register(&g_clampSolenoid, 
                            DRIVES_SOL1H_GPIO_Port, 
                            DRIVES_SOL1L_GPIO_Port,
                            DRIVES_SOL1H_Pin, 
                            DRIVES_SOL1L_Pin,
                            SOLENOID_DRIVER_POSITION_LOW,
                            solenoidDriverPositionChangedCallback);

    /* Register routers. */
    Router_Register(&g_yAxisRouter, 
                    STEPPER_Y_STEP_GPIO_Port, 
                    STEPPER_Y_STEP_Pin,
                    STEPPER_Y_DIR_GPIO_Port, 
                    STEPPER_Y_DIR_Pin, 
                    Y_AXIS_MAX_VELOCITY, 
                    Y_AXIS_MAX_ACCELERATION, 
                    yAxisRouterCallback);

    Router_Register(&g_tAxisRouter, 
                    STEPPER_TEAR_STEP_GPIO_Port, 
                    STEPPER_TEAR_STEP_Pin,
                    STEPPER_TEAR_DIR_GPIO_Port, 
                    STEPPER_TEAR_DIR_Pin, 
                    TEAR_AXIS_MAX_VELOCITY, 
                    TEAR_AXIS_MAX_ACCELERATION, 
                    tAxisRouterCallback);

    /* Register Z motor controller. */
    ZMotorController_RegisterCallback(ZMotorControllerCallback);

    /* Register force coil driver. */
    ForceCoilDriver_RegisterCallback(forceCoilCallback);

    g_state = STATE_READY;
}

void Bonder_Start(void)
{
    if (g_state != STATE_READY)
    {
        return;
    }

    /* Start modules. */
    clearFlags();
    g_state = STATE_OPERATING;
    g_operationState = BONDER_OPERATION_STATE_IDLE;
}

void Bonder_Execute(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }

    if (g_operationState == BONDER_OPERATION_STATE_IDLE) {
        stateIdle();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_MOVING_TO_SEARCH_HEIGHT) {
        stateLevelingDown();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SEARCHING) {
        stateSearching();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SETTLING) {
        stateSettling();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SCANNING_IMPEDANCE) {
        stateScanningImpedance();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_WELDING) {
        stateWelding();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_COOLING) {
        stateCooling();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_FORMING_LOOP_T_MOVE) {
        stateFormingLoopTMove();
    }
    else if (g_operationState ==  BONDER_OPERATION_STATE_FORMING_LOOP_Y_REVERSE_MOVE) {
        stateFormingLoopYReverseMove();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_FORMING_LOOP_Z_MOVE) {
        stateFormingLoopZMove();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_WAITING_SECOND_WELD_TRIGGER) {
        stateWaitingSecondWeldTrigger();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_CLOSING_CLAMP) {
        statePreparingSecondWeldClosingClamp();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_STEPPING_BACK) {
        statePreparingSecondWeldSteppingBack();
    }
    else if(g_operationState == BONDER_OPERATION_STATE_PREPARING_SECOND_WELD_OPENING_CLAMP) {
        statePreparingSecondWeldOpeningClamp();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_MOVING_TO_SEARCH_HEIGHT) {
        stateSecondWeldMovingToSearchHeight();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_SEARCHING) {
        stateSecondWeldSearching();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_SETTLING) {
        stateSecondWeldSettling();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_SCANNING_IMPEDANCE) {
        stateSecondWeldScanningImpedance();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_WELDING) {
        stateSecondWeldWelding();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_COOLING) {
        stateSecondWeldCooling();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_SECOND_WELD_TEARING_TMOVE) {
        stateSecondWeldTearingTmove();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_RESTORING_ZMOVE) {
        stateRestoringZMove();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_RESTORING_ZMOVE_WITH_SCANNING_IMPEDANCE) {
        stateRestoringZMoveWithScanningImpedance();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_TMOVE_WITH_US_DRIVE) {
        stateRestoringZMoveAndTMoveWithUsDrive();
    }
    else if (g_operationState == BONDER_OPERATION_STATE_RESTORING_ZMOVE_AND_YMOVE) {
        stateRestoringZMoveAndYMove();
    }
}

void Bonder_Stop(void)
{
    if (g_state != STATE_OPERATING)
    {
        return;
    }
    
    if (g_operationState != BONDER_OPERATION_STATE_IDLE)
    {
        ZMotorController_SetPositionSetpoint(g_resetHeight);
        ForceCoilDriver_SetCurrentSetpoint(g_forceCoilIdleCurrent);
        SolenoidDriver_SetLowPosition(&g_clampSolenoid);
        UsImpedanceScanner_Stop();
        Pll_Stop();
    }

    g_state = STATE_READY;
}