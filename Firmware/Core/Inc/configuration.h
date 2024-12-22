#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* General configuration ---------------------------------------------------*/
#define NUMBER_OF_CHAMBERS 3

/* Timer module configuration ----------------------------------------------*/
#define TIMER_UC_CLOCK_FREQUENCY 16000000UL
#define TIMER_MAX_NUM_DELEGATES 16U

#define TIMER1_ISR_FREQUENCY 1000UL
#define TIMER2_ISR_FREQUENCY 8000UL

/* Serial module. ----------------------------------------------------------*/
#define SERIAL_BAUDRATE 9600
#define SERIAL_LINE_TERMINATOR "\n"

/* EEPROM Manager module configuration -------------------------------------*/
#define EEPROM_MANAGER_MAX_USABLE_MEMORY_SIZE 4096
#define EEPROM_MANAGER_MAX_NUM_OF_ALLOCATIONS 20

/* Pin monitor module configuration ----------------------------------------*/
#define PIN_MONITOR_MAX_NUMBER_OF_MONITORED_PINS 8
#define PIN_MONITOR_SAMPLING_FREQUENCY 50

/* Stepper module configuration --------------------------------------------*/
#define STEPPER_SEGMENT_QUEUE_CAPACITY_IN_NUM_ELEMENTS 32

#define STEPPER_DIR_PIN_INVERT FALSE
#define STEPPER_CFG_EN_PIN 38
#define STEPPER_STEP_PIN 54
#define STEPPER_DIR_PIN 55

/* Pulser module configuration ---------------------------------------------*/
#define PULSER_TIME_RESOLUTION_IN_MS 1
#define PULSER_UPDATE_FREQUENCY (1000 / PULSER_TIME_RESOLUTION_IN_MS)

#define PULSER_TRACE_QUEUE_CAPACITY_IN_NUM_OF_ELEMENTS 32
#define PULSER_NUMBER_OF_TIME_SLOTS (PULSER_UPDATE_FREQUENCY / BLOCK_EXECUTER_SEGMENT_RENDER_FREQUENCY)

#define PULSER_CHAMBER0_PIN 50
#define PULSER_CHAMBER1_PIN 31
#define PULSER_CHAMBER2_PIN 33

/* HX711 module configuration ----------------------------------------------*/
#define HX711_SCK_PIN A9
#define HX711_SDA_PINS \
    {                  \
        A10,           \
            A11,       \
            A12,       \
    }

#define HX711_POWERUP_DELAY_IN_MS 400
#define HX711_POWERDOWN_DELAY_IN_MS 400
#define HX711_CONFIGURATION HX711_CONFIGURATION_CHANNEL_A_GAIN_128

#define HX711_SAMPLING_PERIOD_IN_MS 200

/* Loadcell module configuration -------------------------------------------*/
#define LOADCELL_DEFAULT_TARE_READINGS \
    {                                  \
        0.                             \
    }

#define LOADCELL_DEFAULT_NO_LOAD_READINGS \
    {                                     \
        0.                                \
    }

#define LOADCELL_DEFAULT_WEIGHT_LOADED_READINGS \
    {                                           \
        1e4                                     \
    }

#define LOADCELL_DEFAULT_CALIBRATION_WEIGHT_IN_KG 1.
#define LOADCELL_GRAVITATIONAL_ACCELERATION 9.81

/* Pulse scheduluer module configuration -----------------------------------*/
#define PULSE_SCHEDULER_MAX_NUM_OF_RECORDED_PULSES 125

/* Router module configuration ---------------------------------------------*/
#define ROUTER_STEP_PER_MM 400

/* Block executer module configuration -------------------------------------*/
#define BLOCK_EXECUTER_BLOCK_QUEUE_CAPACITY_IN_NUM_OF_ELEMENTS 4

#define BLOCK_EXECUTER_SEGMENT_RENDER_FREQUENCY 100

#define BLOCK_EXECUTER_HOMING_MAX_DISPLACEMENT_IN_MM 105.

#define BLOCK_EXECUTER_MIN_LIM_SW_PIN 3
#define BLOCK_EXECUTER_MAX_LIM_SW_PIN 2
#define BLOCK_EXECUTER_MIN_LIM_SW_PINMODE INPUT_PULLUP
#define BLOCK_EXECUTER_MAX_LIM_SW_PINMODE INPUT_PULLUP
#define BLOCK_EXECUTER_MIN_LIM_SW_TRIGGER_TRANSITION PIN_MONITOR_TRANSITION_LOW_TO_HIGH
#define BLOCK_EXECUTER_MAX_LIM_SW_TRIGGER_TRANSITION PIN_MONITOR_TRANSITION_LOW_TO_HIGH

#define BLOCK_EXECUTER_MAX_NUM_OF_PULSES_PER_BLOCK 10

#define BLOCK_EXECUTER_MAX_NUM_OF_RECORDED_BLOCKS 25

/* Device monitor module configuration -------------------------------------*/
#define DEVICE_MONITOR_QUEUE_CAPACITY_IN_SAMPLES 4

/* Cli module configuration ------------------------------------------------*/
#define CLI_DEVICE_MODEL_STR "AxoMechano3"
#define CLI_HARDWARE_REVISION_NUMBER 1
#define CLI_FIRMWARE_REVISION_NUMBER 1
#define CLI_DEVICE_SERIAL_NUMBER 0x00000000

#define CLI_ARRAY_ELEMENT_SEPERATOR ','
#define CLI_FLOAT_PARAM_NUM_DECIMAL_PLACES 3

#define CLI_COMMAND_QUEUE_SIZE 256
#define CLI_ERROR_QUEUE_SIZE 128
#define CLI_SERIAL_CHANNEL 0

/* Robot module configuration ----------------------------------------------*/
#define ROBOT_INITIALIZATION_PERIOD_IN_SEC 2.

#define ROBOT_EMERGENCY_BUTTON_PIN 18
#define ROBOT_EMERGENCY_BUTTON_PINMODE INPUT_PULLUP
#define ROBOT_EMERGENCY_BUTTON_TRIGGER_TRANSITION PIN_MONITOR_TRANSITION_HIGH_TO_LOW

#define ROBOT_RESET_REQUEST_APPLICATION_TIME_IN_MS 250

#endif