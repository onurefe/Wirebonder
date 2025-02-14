#include "robot.h"
#include "timer_expire.h"
#include "pin_monitor.h"
#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "solenoid_driver.h"
#include "pll.h"
#include "zmotor_control.h"
#include "router.h"
#include "force_coil_driver.h"
#include "complex.h"
#include "main.h"

/* Exported variables ------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac1;
extern DMA_HandleTypeDef hdma_dac2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

/* Private definitions -----------------------------------------------------*/
enum
{
    ROBOT_STATE_UNINIT = 0,
    ROBOT_STATE_READY = 1,
    ROBOT_STATE_IDLE = 2,
    ROBOT_STATE_EXECUTING_TASK = 3,
    ROBOT_STATE_ERROR = 4,
};
typedef uint8_t RobotState_t;

/* Private variables -------------------------------------------------------*/
static RobotState_t g_robotState;
static Router_Handle_t g_yRouter;
static Router_Handle_t g_tearRouter;

complexf VoltagePhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
complexf CurrentPhasors[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];
complexf Impedances[US_IMPEDANCE_SCANNER_NUM_FREQUENCIES];

/* Private functions -------------------------------------------------------*/
static void mouseLeftButtonTransitionCb(PinMonitor_Transition_t transition)
{
}

static void mouseRightButtonTransitionCb(PinMonitor_Transition_t transition)
{
}

static void yLimitSwitchTransitionCb(PinMonitor_Transition_t transition)
{
}

static void tipContactSensorTransitionCb(PinMonitor_Transition_t transition)
{
}

static void usImpedanceScannedCb(complexf *voltagePhasors, 
                                 complexf *currentPhasors, 
                                 complexf *impedances)
{
}

static void pllBondingCompletedCb(uint16_t eventId)
{
}

static void forceCoilCb(uint16_t eventId)
{
}

static void zmotorControlCb(uint16_t eventId)
{
}

void yRouterCb(void)
{
}

void tearRouterCb(void)
{
}

/* Exported functions ------------------------------------------------------*/
void Robot_Init(void)
{
    if (g_robotState != STATE_UNINIT)
    {
        return;
    }

    Router_Init(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin,
                 STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin);

    /* We need to share peripherals here! */
    /* Hopefully, LVDT and Ultrasonic driver wouldn't conflict. */
    Timer_Init(&htim8);
    PinMonitor_Init();
    Pll_Init();
    UsDriver_Init(&htim1, &hadc1, &hdac, DAC_CHANNEL_1);
    ZMotorControl_Init(&hdac, &htim3, DAC1_CHANNEL_2);
    ForceCoilDriver_Init(&htim3, FORCE_COIL_CHANNEL);

    g_robotState = STATE_READY;
}

void Robot_Start(void)
{
    if (g_robotState != STATE_READY)
    {
        return;
    }

    /* Register contact sensors. */
    PinMonitor_Register(CONTACT_SENSORS_MOUSE_LEFT_GPIO_Port, 
                        CONTACT_SENSORS_MOUSE_LEFT_Pin, 
                        mouseLeftButtonTransitionCb);

    PinMonitor_Register(CONTACT_SENSORS_MOUSE_RIGHT_GPIO_Port, 
                        CONTACT_SENSORS_MOUSE_RIGHT_Pin, 
                        mouseRightButtonTransitionCb);

    PinMonitor_Register(CONTACT_SENSORS_TIP_GPIO_Port, 
                        CONTACT_SENSORS_TIP_Pin, 
                        tipContactSensorTransitionCb);

    PinMonitor_Register(CONTACT_SENSORS_YLIM_GPIO_Port, 
                        CONTACT_SENSORS_YLIM_Pin, 
                        yLimitSwitchTransitionCb);

    /* Register routers. */
    Router_Register(&g_yRouter, STEPPER_Y_STEP_GPIO_Port, STEPPER_Y_STEP_Pin,
    STEPPER_Y_DIR_GPIO_Port, STEPPER_Y_DIR_Pin, Y_AXIS_MAX_VELOCITY, Y_AXIS_MAX_ACCELERATION, yRouterCb);
    
    Router_Register(&g_tearRouter, STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin,
    STEPPER_TEAR_DIR_GPIO_Port, STEPPER_TEAR_DIR_Pin, TEAR_AXIS_MAX_VELOCITY, TEAR_AXIS_MAX_ACCELERATION, tearRouterCb);

    /* Start modules. */
    PinMonitor_Start();
    Timer_Start();
    ZMotorControl_Start(0.);

    
    ForceCoilDriver_Start();

    Router_Start();
}

void Robot_Execute()
{
    if (g_robotState != STATE_OPERATING)
    {
        return;
    }

    Router_Execute();
}

void Robot_Stop()
{
    if (g_robotState != STATE_OPERATING)
    {
        return;
    }
    g_robotState = STATE_READY;
}

typedef uint8_t SemiAutoBondTask_State_t;

/* Now what to do? Robot may conduct tasks. Each task should contain bunch of variables. And callback functions 
will set flags showing that o subtask is completed. So there should be a task executer. Okay */
typedef struct 
{
    SemiAutoBondTask_State_t state;
    Bool_t zPositionAchieved;
    Bool_t contactPinTriggered;
    Bool_t semiAutoButtonReleased;
    Bool_t stabilitizationTimerExpired;
    Bool_t bondingForceTimerExpired;
    Bool_t restingTimerExpired;
    Bool_t impedanceScanned;
    Bool_t bondingCompleted;
    Bool_t clampEnergized;
    Bool_t tearMoveCompleted;
    float centerFrequency;
} SemiAutoBondTask_Variables_t;

SemiAutoBondTask_Variables_t g_autoBondVariables;

void clearFlags(void)
{
    g_autoBondVariables.zPositionAchieved = FALSE;
    g_autoBondVariables.contactPinTriggered = FALSE;
    g_autoBondVariables.semiAutoButtonReleased = FALSE;
    g_autoBondVariables.stabilitizationTimerExpired = FALSE;
    g_autoBondVariables.bondingForceTimerExpired = FALSE;
    g_autoBondVariables.restingTimerExpired = FALSE;
    g_autoBondVariables.impedanceScanned = FALSE;
    g_autoBondVariables.bondingCompleted = FALSE;
    g_autoBondVariables.clampEnergized = FALSE;
    g_autoBondVariables.tearMoveCompleted = FALSE;
}

enum
{
    SEMI_AUTO_BOND_TASK_STATE_REVERSE_Z_MOVE_TO_SEARCH_LEVEL_1 = 0,
    SEMI_AUTO_BOND_TASK_STATE_REVERSE_Z_MOVE_TO_BOND_LEVEL_1,
    SEMI_AUTO_BOND_TASK_STATE_STABILIZE_1,
    SEMI_AUTO_BOND_TASK_STATE_BONDING_FORCE_1,
    SEMI_AUTO_BOND_TASK_STATE_IMPEDANCE_SCANNING_1,
    SEMI_AUTO_BOND_TASK_STATE_APPLYING_US_POWER_1,
    SEMI_AUTO_BOND_TASK_STATE_RESTING_1,
    SEMI_AUTO_BOND_TASK_STATE_ENERGISING_CLAMP,
    SEMI_AUTO_BOND_TASK_STATE_Z_MOVE_TO_KINK_HEIGHT_AND_T_MOVE,
    SEMI_AUTO_BOND_TASK_STATE_Y_REVERSE_MOVE,
    SEMI_AUTO_BOND_TASK_STATE_Z_MOVE_TO_LOOP_HEIGHT,
    SEMI_AUTO_BOND_TASK_STATE_Y_FORWARD_MOVE,
    SEMI_AUTO_BOND_TASK_STATE_TOGGLING_CLAMP,
    SEMI_AUTO_BOND_TASK_STATE_LEVELING_DOWN_TO_SEARCH_LEVEL_2,
    SEMI_AUTO_BOND_TASK_STATE_LEVELING_DOWN_TO_BOND_LEVEL_2,
    SEMI_AUTO_BOND_TASK_STATE_STABILIZE_2,
    SEMI_AUTO_BOND_TASK_STATE_BONDING_FORCE_2,
    SEMI_AUTO_BOND_TASK_STATE_IMPEDANCE_SCANNING_2,
    SEMI_AUTO_BOND_TASK_STATE_APPLYING_US_POWER_2,
    SEMI_AUTO_BOND_TASK_STATE_RESTING_2,
    SEMI_AUTO_BOND_TASK_STATE_RELEASING_CLAMP,
    SEMI_AUTO_BOND_TASK_STATE_T_MOVE,
    SEMI_AUTO_BOND_TASK_STATE_Z_MOVE_TO_RESET_LEVEL,
    SEMI_AUTO_BOND_TASK_STATE_REVERSE_T_MOVE_WITH_US
};

#define LOWEST_OVERTRAVEL 1.0
void reverseZMoveToSearchLevel1(void)
{
    if (g_autoBondVariables.zPositionAchieved && g_autoBondVariables.semiAutoButtonReleased)
    {
        clearFlags();
        ForceCoilDriver_SetCurrentSetpoint(FORCE_COIL_CONSTANT_FORCE_CURRENT);
        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_REVERSE_Z_MOVE_TO_BOND_LEVEL_1;
    }
}

TimerExpire_Handle_t g_firstStabilizationTimer;
TimerExpire_Handle_t g_bondingForceTimer;
TimerExpire_Handle_t g_restingTimer;

void reverseZMoveToBondLevel1(void)
{
    if (g_autoBondVariables.contactPinTriggered)
    {
        clearFlags();
        ZMotorControl_SetPositionSetpoint(LOWEST_OVERTRAVEL);
        TimerExpire_Activate(&g_firstStabilizationTimer);
        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_STABILIZE_1;
    }
}

void stabilize1(void)
{
    if (g_autoBondVariables.stabilitizationTimerExpired)
    {
        clearFlags();
        TimerExpire_Activate(&g_bondingForceTimer);
        ForceCoilDriver_SetCurrentSetpoint(FORCE_COIL_BONDING_FORCE1_CURRENT);
        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_BONDING_FORCE_1;
    }
}

void bondingForce1(void)
{
    if (g_autoBondVariables.bondingForceTimerExpired)
    {
        clearFlags();
        TimerExpire_Activate(&g_bondingForceTimer);
        UsImpedanceScanner_Start(VoltagePhasors, 
                                 CurrentPhasors, 
                                 Impedances, 
                                 usImpedanceScannedCb);
        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_IMPEDANCE_SCANNING_1;
    }
}

#define BONDING_ENERGY_IN_JOULES 2.0
#define MAX_BONDING_DURATION 1.0

void impedanceScanning1(void)
{
    if (g_autoBondVariables.impedanceScanned)
    {
        clearFlags();
        Pll_Start(g_autoBondVariables.centerFrequency, 
                  BONDING_ENERGY_IN_JOULES,
                  MAX_BONDING_DURATION, 
                  pllBondingCompletedCb);

        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_APPLYING_US_POWER_1;
    }
}

void applyingUsPower1(void)
{
    if (g_autoBondVariables.bondingCompleted)
    {
        clearFlags();
        TimerExpire_Activate(&g_restingTimer);
        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_RESTING_1;
    }
}

SolenoidDriver_Handle_t g_clampSolenoid;

#define TEAR_DISPLACEMENT_1 1.0

void resting1(void)
{
    if (g_autoBondVariables.restingTimerExpired)
    {
        clearFlags();
        SolenoidDriver_SetHighPosition(&g_clampSolenoid);

        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_ENERGISING_CLAMP;
    }
}

#define KINK_HEIGHT_1 2.0

void energyzingClamp1(void)
{
    if (g_autoBondVariables.clampEnergized)
    {
        clearFlags();

        ZMotorControl_SetPositionSetpoint(KINK_HEIGHT_1);
        Router_Append(&g_tearRouter, TEAR_DISPLACEMENT_1);

        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_Z_MOVE_TO_KINK_HEIGHT_AND_T_MOVE;
    }
}

void zMoveToKinkHeightAndTMove(void)
{
    if (g_autoBondVariables.tearMoveCompleted)
    {
        clearFlags();

        ZMotorControl_SetPositionSetpoint(KINK_HEIGHT_1);
        Router_Append(&g_tearRouter, TEAR_DISPLACEMENT_1);

        g_autoBondVariables.state = SEMI_AUTO_BOND_TASK_STATE_Y_REVERSE_MOVE;
    }
}
