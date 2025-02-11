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

/* DAC ---------------------------------------------------------------------*/
#define DAC_VOLTAGE_RANGE 3.3
#define DAC_BITS 12
#define DAC1_SAMPLING_FREQUENCY 348000

/* ADC VOLTAGE RANGE -------------------------------------------------------*/
#define ADC_VOLTAGE_RANGE 3.3
#define ADC_BITS 12
#define ADC1_SAMPLING_FREQUENCY 348000

/* Ultrasonic Transducer ---------------------------------------------------*/
#define US_IMPEDANCE_SCANNER_NUM_FREQUENCIES 16
#define US_IMPEDANCE_SCANNER_MIN_FREQUENCY 56000
#define US_IMPEDANCE_SCANNER_MAX_FREQUENCY 64000
#define US_IMPEDANCE_SCANNER_FREQUENCY_STEP \
((US_IMPEDANCE_SCANNER_MAX_FREQUENCY - US_IMPEDANCE_SCANNER_MIN_FREQUENCY) / \
 US_IMPEDANCE_SCANNER_NUM_FREQUENCIES)

#define US_IMPEDANCE_SCANNER_ADC_SAMPLES \
(ADC1_SAMPLING_FREQUENCY / US_IMPEDANCE_SCANNER_FREQUENCY_STEP)

#define US_IMPEDANCE_SCANNER_DAC_SAMPLES \
(DAC1_SAMPLING_FREQUENCY / US_IMPEDANCE_SCANNER_FREQUENCY_STEP)

/* PLL ---------------------------------------------------------------------*/
#define PLL_ADC_DOUBLE_BUFFER_SIZE 1024
#define PLL_DAC_DOUBLE_BUFFER_SIZE 1024
#define PLL_SINE_LOOKUP_SIZE 4096

#define PLL_PID_CONTROLLER_GAIN 1.
#define PLL_PID_CONTROLLER_INTEGRAL_TC 1.
#define PLL_PID_CONTROLLER_DERIVATIVE_TC 0.
#define PLL_PID_CONTROLLER_UPDATE_FREQUENCY 50
#define PLL_PID_CONTROLLER_FILTER_TC 0.
#define PLL_PID_CONTROLLER_MIN_FREQUENCY_DEVIATION -1000.
#define PLL_PID_CONTROLLER_MAX_FREQUENCY_DEVIATION 1000.

/* ADC Dispatcher ----------------------------------------------------------*/
#define ADC_DISPATCHER_NOTIFICATION_FREQUENCY 4000

/* LVDT --------------------------------------------------------------------*/
#define LVDT_MAX_NUM_OF_CALLBACKS                                  8
#define LVDT_DRIVING_FREQUENCY                                      1000
#define LVDT_WAVE_GENERATION_SAMPLING_FREQUENCY                     (128 * LVDT_DRIVING_FREQUENCY)
#define LVDT_NOTIFICATION_FREQUENCY                                 100
#define LVDT_AMPLITUDE_DIFFERENCE_TO_POSITION_CONVERSION_FACTOR     1.0

/* Tachometer --------------------------------------------------------------*/
#define TACHOMETER_MAX_NUM_OF_CALLBACKS             8
#define TACHOMETER_NOTIFICATION_FREQUENCY           100
#define TACHOMETER_VOLTAGE_TO_RPM_CONVERSION_FACTOR 1.0
#define TACHOMETER_ZERO_VELOCITY_VOLTAGE            1.65

/* ADC2 Conversion orders --------------------------------------------------*/
#define FORCE_COIL_ISENS_ORDER 0
#define ZMOTOR_TACHOMETER_ORDER 1
#define LVDT_A_ORDER 2
#define LVDT_B_ORDER 3

/* Z Motor Control ---------------------------------------------------------*/
#define ZMOTOR_CONTROL_MAX_NUM_OF_CALLBACKS 8

#define ZMOTOR_ZERO_VELOCITY_DUTY 0.5

#define ZMOTOR_SETPOINT_ACHIEVED_EVENT_MAX_POSITION_ERROR 1e-2 
#define ZMOTOR_SETPOINT_ACHIEVED_EVENT_MAX_VELOCITY_ERROR 1e-2

#define ZMOTOR_POSITION_CONTROL_PID_GAIN 1.0
#define ZMOTOR_POSITION_CONTROL_PID_INTEGRAL_TC 0.1
#define ZMOTOR_POSITION_CONTROL_PID_DERIVATIVE_TC 0.
#define ZMOTOR_POSITION_CONTROL_PID_DT 0.01 
#define ZMOTOR_POSITION_CONTROL_PID_FILTER_TC 0.05
#define ZMOTOR_POSITION_CONTROL_PID_OUTPUT_MIN -1.0
#define ZMOTOR_POSITION_CONTROL_PID_OUTPUT_MAX 1.0

#define ZMOTOR_VELOCITY_CONTROL_PID_GAIN 1.0
#define ZMOTOR_VELOCITY_CONTROL_PID_INTEGRAL_TC 0.1
#define ZMOTOR_VELOCITY_CONTROL_PID_DERIVATIVE_TC 0.
#define ZMOTOR_VELOCITY_CONTROL_PID_DT 0.01 
#define ZMOTOR_VELOCITY_CONTROL_PID_FILTER_TC 0.05
#define ZMOTOR_VELOCITY_CONTROL_PID_OUTPUT_MIN -0.5
#define ZMOTOR_VELOCITY_CONTROL_PID_OUTPUT_MAX 0.5

#define ZMOTOR_CONTROL_MAX_DUTY 1.0 
#define ZMOTOR_CONTROL_MIN_DUTY 0.0

/* Force coil driver module configuration ----------------------------------*/
#define FORCE_COIL_MAX_NUM_OF_CALLBACKS 8

#define FORCE_COIL_DRIVER_SETPOINT_ACHIEVED_EVENT_MAX_CURRENT_ERROR 1e-2

#define FORCE_COIL_DRIVER_CONTROL_UPDATE_FREQUENCY 1000
#define FORCE_COIL_DRIVER_V2I_CONVERSION_FACTOR   (1.0/0.781)

#define FORCE_COIL_DRIVER_PID_GAIN 1.0
#define FORCE_COIL_DRIVER_PID_INTEGRAL_TC 0.01
#define FORCE_COIL_DRIVER_PID_DERIVATIVE_TC 0.0
#define FORCE_COIL_DRIVER_PID_INPUT_FILTER_TC 0.001
#define FORCE_COIL_DRIVER_PID_OUTPUT_MIN 0.0
#define FORCE_COIL_DRIVER_PID_OUTPUT_MAX 1.0

#define FORCE_COIL_MAX_DUTY 1.0 
#define FORCE_COIL_MIN_DUTY 0.0

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