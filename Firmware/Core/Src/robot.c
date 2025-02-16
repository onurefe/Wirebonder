#include "robot.h"
#include "timer_expire.h"
#include "pin_monitor.h"
#include "solenoid_driver.h"
#include "zmotor_controller.h"
#include "router.h"
#include "force_coil_driver.h"
#include "complex.h"
#include "main.h"
#include "bonder.h"

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
static Router_Handle_t g_yAxisRouter;
static Router_Handle_t g_tearAxisRouter;

/* Private functions -------------------------------------------------------*/
static void yAxisLimitSwitchTransitionCallback(PinMonitor_Transition_t transition)
{
}

static void bonderStateChangedCallback(Bonder_OperationState_t previousState, 
                                       Bonder_OperationState_t newState)
{
}

static void bonderErrorOccurredCallback(Bonder_Error_t error)
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
    ZMotorController_Init(&hdac, &htim3, DAC_CHANNEL_2);
    ForceCoilDriver_Init(&htim3, );

    Bonder_Init(bonderStateChangedCallback, 
                bonderErrorOccurredCallback);

    g_robotState = STATE_READY;
}

void Robot_Start(void)
{
    if (g_robotState != STATE_READY)
    {
        return;
    }

    PinMonitor_Register(CONTACT_SENSORS_YLIM_GPIO_Port, 
                        CONTACT_SENSORS_YLIM_Pin, 
                        yAxisLimitSwitchTransitionCallback);


    /* Start modules. */
    PinMonitor_Start();
    Timer_Start();
    ZMotorController_Start(0.);
    ForceCoilDriver_Start();
    Router_Start();
    Bonder_Start();

    g_robotState = STATE_OPERATING;
}

void Robot_Execute(void)
{
    if (g_robotState != STATE_OPERATING)
    {
        return;
    }

    Router_Execute();
    Bonder_Execute();
}

void Robot_Stop(void)
{
    if (g_robotState != STATE_OPERATING)
    {
        return;
    }
    g_robotState = STATE_READY;
}