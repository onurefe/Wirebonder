#include "robot.h"
#include "timer.h"
#include "pin_monitor.h"
#include "us_impedance_scanner.h"
#include "us_driver.h"
#include "pll.h"
#include "zmotor_control.h"
#include "stepper.h"
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
Stepper_Handle_t g_tearMotorStepperHandle;
Stepper_Handle_t g_yMotorStepperHandle;

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

static void tearStepperControlCb(uint16_t eventId)
{
}

static void yStepperControlCb(uint16_t eventId)
{
}

/* Private variables -------------------------------------------------------*/
static RobotState_t g_robotState;

/* Exported functions ------------------------------------------------------*/
void Robot_Init(void)
{
    if (g_robotState != STATE_UNINIT)
    {
        return;
    }

    /* Find a way to handle EN pins. They are common now which 
    lead to conflicts. */
    Stepper_Init(&g_yMotorStepperHandle,
                 STEPPER_EN_GPIO_Port, STEPPER_EN_Pin,
                 STEPPER_Y_STEP_GPIO_Port, STEPPER_Y_STEP_Pin,
                 STEPPER_Y_DIR_GPIO_Port, STEPPER_Y_DIR_Pin);

    Stepper_Init(&g_tearMotorStepperHandle,
                 STEPPER_EN_GPIO_Port, STEPPER_EN_Pin,
                 STEPPER_TEAR_STEP_GPIO_Port, STEPPER_TEAR_STEP_Pin,
                 STEPPER_TEAR_DIR_GPIO_Port, STEPPER_TEAR_DIR_Pin);

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

    PinMonitor_Start();
    Timer_Start();
    ZMotorControl_Start(0.);

    Stepper_Start(&g_yMotorStepperHandle);
    Stepper_Start(&g_tearMotorStepperHandle);

}

void Robot_Execute()
{
    if (g_robotState != STATE_OPERATING)
    {
        return;
    }
}

void Robot_Stop()
{
    Loadcell_Stop();
    PinMonitor_Stop();
    BlockExecuter_Stop();
    Cli_Stop();

    g_robotState = STATE_READY;
}

/* Private functions -------------------------------------------------------*/
/* Cli callbacks */
void cliGetDeviceInfoCb()
{
    if (isRobotInitializing())
    {
        setRobotState(ROBOT_STATE_IDLE);
    }
}

void cliReadMemoryOverviewCb()
{
    uint8_t num_of_pulses = PulseScheduler_GetNumOfPulses();
    uint8_t num_of_blocks = BlockExecuter_GetNumOfBlocks();

    Cli_SendReadMemoryOverviewOperationCompletedResponse(num_of_pulses, num_of_blocks);
}

void cliReadPulseCb(int pulseIdx)
{
    PulseScheduler_Pulse_t pulse;

    if (PulseScheduler_LoadPulseFromEeprom((uint8_t)pulseIdx, &pulse))
    {
        Cli_SendReadPulseOperationCompletedResponse(pulse.offsetTimeInSec,
                                                    pulse.onIntervalInSec,
                                                    pulse.offIntervalInSec,
                                                    pulse.chamberIdx,
                                                    pulse.repetition);
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_ITEM_DOESNT_EXIST);
    }
}

void cliReadBlockCb(int blockIdx)
{
    BlockExecuter_Block_t block;

    if (!BlockExecuter_LoadBlockFromEeprom((uint8_t)blockIdx, &block))
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_ITEM_DOESNT_EXIST);
        Cli_SendOperationCompletedResponse(false);
        return;
    }

    switch (block.blockType)
    {
    case BLOCK_EXECUTER_BLOCK_TYPE_HOME:
        sendReadResponse((BlockExecuter_HomeBlock_t *)block.derivedBlock);
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_MOVE:
        sendReadResponse((BlockExecuter_MoveBlock_t *)block.derivedBlock);
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_SINE:
        sendReadResponse((BlockExecuter_SineBlock_t *)block.derivedBlock);
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_RAMP:
        sendReadResponse((BlockExecuter_RampBlock_t *)block.derivedBlock);
        break;
    }
}

void cliResetCb()
{
    ResetRequested = true;
    ResetRequestTick = Timer1_GetTicks();
    Cli_SendOperationCompletedResponse(true);
}

void cliAppendPulseCb(float offsetTimeInSec,
                      float onIntervalInSec,
                      float offIntervalInSec,
                      int chamberIdx,
                      int repetition)
{
    if (isRobotAvailableForEepromAccess())
    {
        uint8_t pulse_idx;
        PulseScheduler_Pulse_t pulse;

        pulse.offsetTimeInSec = offsetTimeInSec;
        pulse.onIntervalInSec = onIntervalInSec;
        pulse.offIntervalInSec = offIntervalInSec;
        pulse.chamberIdx = (uint8_t)chamberIdx;
        pulse.repetition = (uint32_t)repetition;

        if (PulseScheduler_AppendPulseToEeprom(&pulse, &pulse_idx))
        {
            Cli_SendAppendItemOperationCompletedResponse(pulse_idx);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliAppendHomeBlockCb(float velocityInMmPerSec)
{
    if (isRobotAvailableForEepromAccess())
    {
        uint8_t block_idx;
        BlockExecuter_HomeBlock_t home_block;
        home_block.velocityInMmSec = velocityInMmPerSec;

        if (BlockExecuter_AppendHomeBlockToEeprom(&home_block, &block_idx))
        {
            Cli_SendAppendItemOperationCompletedResponse(block_idx);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliAppendMoveBlockCb(float displacementInMm, float velocityInMmPerSec, int forceLimitingEnabled,
                          float forceLimitInNewtons, int mergingType, List<int> *chamberIdxs)
{
    uint8_t block_idx;

    if (isRobotAvailableForEepromAccess())
    {
        BlockExecuter_MoveBlock_t move_block;
        move_block.displacementInMm = displacementInMm;
        move_block.velocityInMmSec = velocityInMmPerSec;
        move_block.forceLimitingEnabled = forceLimitingEnabled > 0 ? 1 : 0;
        move_block.forceLimitInNewtons = forceLimitInNewtons;
        move_block.mergingType = mergingType;

        Utils_ListToArr(chamberIdxs, move_block.chamberIdxs, &move_block.numOfUsedChambers);

        if (BlockExecuter_AppendMoveBlockToEeprom(&move_block, &block_idx))
        {
            Cli_SendAppendItemOperationCompletedResponse(block_idx);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliAppendSineBlockCb(float amplitudeInMm,
                          float periodInSec,
                          List<int> *pulseIdxs,
                          int cycles)
{
    if (isRobotAvailableForEepromAccess())
    {
        uint8_t block_idx;
        BlockExecuter_SineBlock_t sine_block;

        sine_block.amplitudeInMm = amplitudeInMm;
        sine_block.periodInSec = periodInSec;
        sine_block.cycles = (uint32_t)cycles;
        Utils_ListToArr(pulseIdxs, sine_block.pulseIdxs, &sine_block.numPulses);

        if (BlockExecuter_AppendSineBlockToEeprom(&sine_block, &block_idx))
        {
            Cli_SendAppendItemOperationCompletedResponse(block_idx);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliAppendRampBlockCb(float amplitudeInMm,
                          float stretchDurationInSec,
                          float holdDurationInSec,
                          float recoverDurationInSec,
                          float restDurationInSec,
                          List<int> *pulseIdxs,
                          int cycles)
{
    if (isRobotAvailableForEepromAccess())
    {
        uint8_t block_idx;
        BlockExecuter_RampBlock_t ramp_block;

        ramp_block.amplitudeInMm = amplitudeInMm;
        ramp_block.stretchDurationInSec = stretchDurationInSec;
        ramp_block.holdDurationInSec = holdDurationInSec;
        ramp_block.recoverDurationInSec = recoverDurationInSec;
        ramp_block.restDurationInSec = restDurationInSec;
        ramp_block.cycles = (uint32_t)cycles;

        Utils_ListToArr(pulseIdxs, ramp_block.pulseIdxs, &ramp_block.numPulses);

        if (BlockExecuter_AppendRampBlockToEeprom(&ramp_block, &block_idx))
        {
            Cli_SendAppendItemOperationCompletedResponse(block_idx);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliExecuteHomeBlockCb(float velocityInMmPerSec)
{
    if (isRobotAvailableForBlockExecution())
    {
        if (BlockExecuter_EnqueueHomeBlock(velocityInMmPerSec))
        {
            Cli_SendOperationStartedResponse();
            setRobotState(ROBOT_STATE_EXECUTING_BLOCK);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_BLOCK_EXECUTION);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliExecuteMoveBlockCb(float displacementInMm,
                           float velocityInMmPerSec,
                           int forceLimitingEnabled,
                           float forceLimitInNewtons,
                           int mergingType,
                           List<int> *chamberIdxs)
{
    if (isRobotAvailableForBlockExecution())
    {
        uint8_t chamber_idxs[NUMBER_OF_CHAMBERS];
        uint8_t num_of_elements;
        Utils_ListToArr(chamberIdxs, chamber_idxs, &num_of_elements);

        if (BlockExecuter_EnqueueMoveBlock(displacementInMm,
                                           velocityInMmPerSec,
                                           forceLimitingEnabled,
                                           forceLimitInNewtons,
                                           mergingType,
                                           chamber_idxs,
                                           num_of_elements))
        {
            Cli_SendOperationStartedResponse();
            setRobotState(ROBOT_STATE_EXECUTING_BLOCK);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_BLOCK_EXECUTION);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliExecuteBlockFromEepromCb(int blockIdx)
{
    if (isRobotAvailableForBlockExecution())
    {
        if (BlockExecuter_EnqueueBlockWithIdx((uint8_t)blockIdx))
        {
            Cli_SendOperationStartedResponse();
            setRobotState(ROBOT_STATE_EXECUTING_BLOCK);
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_BLOCK_EXECUTION);
    }
}

void cliRunTapeCb()
{
    bool success;
    if (isRobotAvailableForRunningTape())
    {
        TapeReadIdx = 0;
        setRobotState(ROBOT_STATE_RUNNING_TAPE);
        success = true;
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_RUNNING_TAPE);
        success = false;
    }

    Cli_SendOperationCompletedResponse(success);
}

void cliClearTapeCb()
{
    bool success;
    if (isRobotAvailableForEepromAccess())
    {
        PulseScheduler_ClearEepromMemory();
        BlockExecuter_ClearEepromMemory();
        success = true;
    }
    else
    {
        success = false;
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_IS_NOT_AVAILABLE_FOR_EEPROM_ACCESS);
    }

    Cli_SendOperationCompletedResponse(success);
}

void cliPauseCb()
{
    bool success;
    if (isRobotPausable())
    {
        pauseByCommand();
        success = true;
    }
    else
    {
        success = false;
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_NO_PAUSABLE_OPERATION_IS_RUNNING);
    }

    Cli_SendOperationCompletedResponse(success);
}

void cliResumeCb()
{
    bool success;
    if (isRobotPausedByCommand())
    {
        resume();
        success = true;
    }
    else
    {
        success = false;
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_HASNT_BEEN_PAUSED_BY_COMMAND);
    }

    Cli_SendOperationCompletedResponse(success);
}

void cliStopCb()
{
    if (isRobotActive())
    {
        BlockExecuter_Restart();
        setRobotState(ROBOT_STATE_IDLE);
        Cli_SendOperationCompletedResponse(true);
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_NO_STOPPABLE_OPERATION_IS_RUNNING);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliCalibrateLoadcellsNoLoadReadingsCb(float calibrationPeriodInSec)
{
    if (isRobotAvailableForCalibration())
    {
        if (Loadcell_CalibrateNoLoadReadings(&loadcellCalibrationCompletedCb, calibrationPeriodInSec))
        {
            setRobotState(ROBOT_STATE_CALIBRATING);
            Cli_SendOperationStartedResponse();
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_SHOULD_BE_IN_IDLE_STATE_FOR_CALIBRATION);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliCalibrateLoadcellsLoadedReadingsCb(float calibrationWeightInKg, float calibrationPeriodInSec)
{
    if (isRobotAvailableForCalibration())
    {
        if (Loadcell_CalibrateLoadedReadings(&loadcellCalibrationCompletedCb, calibrationPeriodInSec,
                                             calibrationWeightInKg))
        {
            setRobotState(ROBOT_STATE_CALIBRATING);
            Cli_SendOperationStartedResponse();
        }
        else
        {
            Cli_SendOperationCompletedResponse(false);
        }
    }
    else
    {
        Exceptions_ThrowException(EXCEPTIONS_EXCEPTION_CODE_DEVICE_SHOULD_BE_IN_IDLE_STATE_FOR_CALIBRATION);
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliActivateReportingCb(float reportPeriodInSec)
{
    if (Loadcell_IsForwardingSensorReadings())
    {
        DeviceMonitor_ActivateReporting(reportPeriodInSec);
        Cli_SendOperationCompletedResponse(true);
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliDeactivateReportingCb()
{
    DeviceMonitor_DeactivateReporting();
    Cli_SendOperationCompletedResponse(true);
}

void cliSetCurrentTimeCb(float timeInSec)
{
    if (DeviceMonitor_IsReportingActive())
    {
        DeviceMonitor_SetCurrentTime(timeInSec);
        Cli_SendOperationCompletedResponse(true);
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliTareCb()
{
    if (DeviceMonitor_IsReportingActive())
    {
        DeviceMonitor_Tare();
        Cli_SendOperationCompletedResponse(true);
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
    }
}

void cliSetCurrentPositionCb(float positionInMm)
{
    if (DeviceMonitor_IsReportingActive())
    {
        DeviceMonitor_SetCurrentPosition(positionInMm);
        Cli_SendOperationCompletedResponse(true);
    }
    else
    {
        Cli_SendOperationCompletedResponse(false);
    }
}

/* Pin monitor callbacks */
void pinMonitorEmergencyButtonTransitionCb(PinMonitor_Transition_t transition)
{
    return;
    if (transition == ROBOT_EMERGENCY_BUTTON_TRIGGER_TRANSITION)
    {
        Cli_NotifyEmergencyButtonPressed();

        if (isRobotActive())
        {
            pauseByEmergencyStopButton();
        }
    }
    else
    {
        Cli_NotifyEmergencyButtonReleased();

        if (isRobotPausedByEmergencyStopButton())
        {
            resume();
        }
    }
}

/* Loadcell callbacks */
void reportReadyCb(float timestamp, float position, float *loadcellMeasurementsInNewtons,
                   bool *pulserActivityStates)
{
    List<int> pulser_activity_states;
    List<float> loadcell_measurements;

    Utils_ArrToList(pulserActivityStates, NUMBER_OF_CHAMBERS, &pulser_activity_states);
    Utils_ArrToList(loadcellMeasurementsInNewtons, NUMBER_OF_CHAMBERS, &loadcell_measurements);

    Cli_NotifyReportReady(timestamp,
                          position,
                          &pulser_activity_states,
                          &loadcell_measurements);
}

void loadcellCalibrationCompletedCb(float *results)
{
    List<float> calibration_results;
    Utils_ArrToList(results, NUMBER_OF_CHAMBERS, &calibration_results);

    Cli_SendCalibrateLoadcellsOperationCompletedResponse(&calibration_results);
    setRobotState(ROBOT_STATE_IDLE);
}

/* Block executer callbacks */
void blockExecuterBlockLoadedCb(BlockExecuter_BlockType_t blockType)
{
    switch (blockType)
    {
    case BLOCK_EXECUTER_BLOCK_TYPE_HOME:
        Cli_NotifyBlockLoaded("Home");
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_MOVE:
        Cli_NotifyBlockLoaded("Move");
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_SINE:
        Cli_NotifyBlockLoaded("Sine");
        break;

    case BLOCK_EXECUTER_BLOCK_TYPE_RAMP:
        Cli_NotifyBlockLoaded("Ramp");
        break;

    default:
        Cli_NotifyBlockLoaded("ERROR:Unknown block type.");
        setRobotState(ROBOT_STATE_ERROR);
        break;
    }
}

void blockExecuterBlockConsumedCb()
{
    if (RobotState == ROBOT_STATE_EXECUTING_BLOCK)
    {
        Cli_SendOperationCompletedResponse(true);
        if (BlockExecuter_BlockQueueIsEmpty())
        {
            setRobotState(ROBOT_STATE_IDLE);
        }
    }
    else if (RobotState == ROBOT_STATE_RUNNING_TAPE)
    {
        Cli_NotifyBlockConsumed();
    }
}

void blockExecuterLimitSwitchTriggeredCb()
{
    Cli_NotifyLimitSwitchTriggered();
}

void blockExecuterErrorOccurredCb()
{
    Cli_NotifyBlockExecutionErrorOccurred();
    setRobotState(ROBOT_STATE_ERROR);
}

/* Private functions ---------------------------------------------------------*/
void sendReadResponse(BlockExecuter_HomeBlock_t *homeBlock)
{
    Cli_SendReadBlockOperationCompletedResponseForHomeBlock(homeBlock->velocityInMmSec);
}

void sendReadResponse(BlockExecuter_MoveBlock_t *moveBlock)
{
    List<int> chamber_idxs;
    Utils_ArrToList(moveBlock->chamberIdxs, moveBlock->numOfUsedChambers, &chamber_idxs);
    Cli_SendReadBlockOperationCompletedResponseForMoveBlock(moveBlock->displacementInMm,
                                                            moveBlock->velocityInMmSec,
                                                            moveBlock->forceLimitingEnabled,
                                                            moveBlock->forceLimitInNewtons,
                                                            moveBlock->mergingType,
                                                            &chamber_idxs);
}

void sendReadResponse(BlockExecuter_SineBlock_t *sineBlock)
{
    List<int> pulse_idxs;
    Utils_ArrToList(sineBlock->pulseIdxs, sineBlock->numPulses, &pulse_idxs);

    Cli_SendReadBlockOperationCompletedResponseForSineBlock(sineBlock->amplitudeInMm,
                                                            sineBlock->periodInSec,
                                                            &pulse_idxs,
                                                            sineBlock->cycles);
}

void sendReadResponse(BlockExecuter_RampBlock_t *rampBlock)
{
    List<int> pulse_idxs;
    Utils_ArrToList(rampBlock->pulseIdxs, rampBlock->numPulses, &pulse_idxs);

    Cli_SendReadBlockOperationCompletedResponseForRampBlock(rampBlock->amplitudeInMm,
                                                            rampBlock->stretchDurationInSec,
                                                            rampBlock->holdDurationInSec,
                                                            rampBlock->recoverDurationInSec,
                                                            rampBlock->restDurationInSec,
                                                            &pulse_idxs,
                                                            rampBlock->cycles);
}

void pauseByEmergencyStopButton()
{
    BlockExecuter_Pause();
    RobotStateBeforePausing = RobotState;
    setRobotState(ROBOT_STATE_PAUSED_BY_EMERGENCY_STOP_BUTTON);
}

void pauseByCommand()
{
    BlockExecuter_Pause();
    RobotStateBeforePausing = RobotState;
    setRobotState(ROBOT_STATE_PAUSED_BY_COMMAND);
}

void resume()
{
    BlockExecuter_Resume();
    setRobotState(RobotStateBeforePausing);
}

bool isEmergencyButtonHold()
{
    PinMonitor_Level_t emergency_button_pin_level;
    emergency_button_pin_level = PinMonitor_GetPinState(ROBOT_EMERGENCY_BUTTON_PIN);

    if (ROBOT_EMERGENCY_BUTTON_TRIGGER_TRANSITION == PIN_MONITOR_TRANSITION_HIGH_TO_LOW)
    {
        return (emergency_button_pin_level == PIN_MONITOR_LEVEL_LOW);
    }

    return (emergency_button_pin_level == PIN_MONITOR_LEVEL_HIGH);
}

void setRobotState(RobotState_t newState)
{
    if (newState != RobotState)
    {
        RobotState = newState;
        Cli_NotifyStateChanged(newState);
    }
}

bool isResetPeriodPassed()
{
    return (Timer1_GetTicks() - ResetRequestTick) >
           (((uint32_t)ROBOT_RESET_REQUEST_APPLICATION_TIME_IN_MS * TIMER1_ISR_FREQUENCY) / 1000);
}

bool isRobotInitializing()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_INITIALIZING));
}

bool isRobotAvailableForRunningTape()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_IDLE));
}

bool isRobotAvailableForCalibration()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_IDLE));
}

bool isRobotAvailableForEepromAccess()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_IDLE));
}

bool isRobotAvailableForBlockExecution()
{
    return ((State == STATE_OPERATING) && ((RobotState == ROBOT_STATE_IDLE) ||
                                           (RobotState == ROBOT_STATE_EXECUTING_BLOCK)));
}

bool isRobotPausable()
{
    return ((State == STATE_OPERATING) && ((RobotState == ROBOT_STATE_EXECUTING_BLOCK) ||
                                           (RobotState == ROBOT_STATE_RUNNING_TAPE)));
}

bool isRobotPausedByEmergencyStopButton()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_PAUSED_BY_EMERGENCY_STOP_BUTTON));
}

bool isRobotPausedByCommand()
{
    return ((State == STATE_OPERATING) && (RobotState == ROBOT_STATE_PAUSED_BY_COMMAND));
}

bool isRobotActive()
{
    return ((State == STATE_OPERATING) && ((RobotState == ROBOT_STATE_RUNNING_TAPE) ||
                                           (RobotState == ROBOT_STATE_PAUSED_BY_COMMAND) ||
                                           (RobotState == ROBOT_STATE_PAUSED_BY_EMERGENCY_STOP_BUTTON) ||
                                           (RobotState == ROBOT_STATE_EXECUTING_BLOCK)));
}

bool initializationTimerElapsed()
{
    return (Timer1_GetTicks() >= (((uint32_t)TIMER1_ISR_FREQUENCY) * ROBOT_INITIALIZATION_PERIOD_IN_SEC));
}

void getCliDelegates(Cli_Delegates_t *delegates)
{
    delegates->getDeviceInfo = cliGetDeviceInfoCb;
    delegates->setCurrentPosition = cliSetCurrentPositionCb;
    delegates->readMemoryOverview = cliReadMemoryOverviewCb;
    delegates->readPulse = cliReadPulseCb;
    delegates->readBlock = cliReadBlockCb;
    delegates->reset = cliResetCb;
    delegates->appendPulse = cliAppendPulseCb;
    delegates->appendHomeBlock = cliAppendHomeBlockCb;
    delegates->appendMoveBlock = cliAppendMoveBlockCb;
    delegates->appendSineBlock = cliAppendSineBlockCb;
    delegates->appendRampBlock = cliAppendRampBlockCb;
    delegates->executeHomeBlock = cliExecuteHomeBlockCb;
    delegates->executeMoveBlock = cliExecuteMoveBlockCb;
    delegates->executeBlockFromEeprom = cliExecuteBlockFromEepromCb;
    delegates->runTape = cliRunTapeCb;
    delegates->clearTape = cliClearTapeCb;
    delegates->pause = cliPauseCb;
    delegates->resume = cliResumeCb;
    delegates->stop = cliStopCb;
    delegates->calibrateLoadcellsNoLoadReadings = cliCalibrateLoadcellsNoLoadReadingsCb;
    delegates->calibrateLoadcellsLoadedReadings = cliCalibrateLoadcellsLoadedReadingsCb;
    delegates->activateReporting = cliActivateReportingCb;
    delegates->deactivateReporting = cliDeactivateReportingCb;
    delegates->tare = cliTareCb;
    delegates->setCurrentPosition = cliSetCurrentPositionCb;
    delegates->setCurrentTime = cliSetCurrentTimeCb;
}

void getBlockExecuterDelegates(BlockExecuter_Delegates_t *delegates)
{
    delegates->blockLoaded = blockExecuterBlockLoadedCb;
    delegates->blockConsumed = blockExecuterBlockConsumedCb;
    delegates->errorOccurred = blockExecuterErrorOccurredCb;
    delegates->limitSwitchTriggered = blockExecuterLimitSwitchTriggeredCb;
}