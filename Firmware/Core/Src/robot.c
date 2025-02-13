#include "robot.h"
#include "timer.h"
#include "pin_monitor.h"
#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "pll.h"
#include "zmotor_control.h"
#include "router.h"
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
