#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "main.h"

/* Eeprom emulator -----------------------------------------------------------*/
#define EEPROM_EMULATOR_FLASH_SECTOR                                 FLASH_SECTOR_7
#define EEPROM_EMULATOR_FLASH_SECTOR_ADDR                            0x08060000UL
#define EEPROM_EMULATOR_FLASH_SECTOR_SIZE                            (128 * 1024)
#define EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES                           1024
#define EEPROM_EMULATOR_MAX_NUM_OF_OBJECTS                           32

/* LvdtSensorModule ----------------------------------------------------------------*/
#define LVDT_MODULE_DRIVING_FREQUENCY                                1000
#define LVDT_MODULE_EXCITATION_AMPLITUDE                             (DAC2_VOLTAGE_RANGE / 2.0f)
#define LVDT_MODULE_EXCITATION_AVERAGE                               (DAC2_VOLTAGE_RANGE / 2.0f)

#define LVDT_MODULE_STROKE_MM                                        32.0
#define LVDT_MODULE_MIN_TOTAL_MAGNITUDE                              1E-3

/* PllModule -----------------------------------------------------------------*/
#define PLL_MODULE_CENTER_FREQUENCY                                  60000.0f
#define PLL_MODULE_CONTROL_FREQ                                      1000

#define PLL_MODULE_FREQ_CORRECTION_QUEUE_DEPTH                       4

/* Tuned from the bench-fitted transducer (fs = 59.7 kHz, Q = 333):
   plant gain 2Q/fs = 0.0112 rad/Hz; dead time ~5 ms (correction queue +
   demod window + transducer ring-in 2Q/ws ~ 1.8 ms) -> loop shaping gives
   Kp ~ 41 Hz/rad. Integral kept slower than the formula optimum: during
   capture the loop crosses the fs..fp phase plateau where plant gain is
   low, and a fast integral winds up there. Filter smooths per-window
   demodulation noise (~2 control periods).
   MAX_DEVIATION is asymmetric on purpose: above fp (~+1 kHz from center)
   the phase slope flips sign and the loop would be POSITIVE feedback,
   running away upward; the clamp keeps the loop on the series branch. */
/* Kp = 40 limit-cycled at the resonance (bench: period-8 oscillation,
   +-70 Hz): loop gain Kp*K = 0.45/tick exceeds the ~0.35 stability limit
   of the ~3-tick loop delay (queue + demod + transducer ring-in).
   Kp = 10 -> loop gain 0.11, ~3x margin. */
#define PLL_MODULE_FREQ_PID_GAIN                                     10.0
#define PLL_MODULE_FREQ_PID_INTEGRAL_TC                              0.05
#define PLL_MODULE_FREQ_PID_DERIVATIVE_TC                            0.0
#define PLL_MODULE_FREQ_PID_FILTER_TC                                0.002
#define PLL_MODULE_FREQ_PID_MIN_DEVIATION                            -1000.0
#define PLL_MODULE_FREQ_PID_MAX_DEVIATION                            500.0

/* DebugService --------------------------------------------------------------*/
/* When enabled, the bonder state machine is not started and the debugger
   owns the subsystems through the DebugService channels (see
   debug_service.hpp). */
#define DEBUG_ENABLED                                                1

/* Debug channel ids — assigned centrally so no two channels collide. The
   DebugService rejects id 0 (reserved) and duplicate ids. */
#define DEBUG_CHANNEL_ID_IMPEDANCE_SCANNER                           1
#define DEBUG_CHANNEL_ID_PLL                                         2
#define DEBUG_CHANNEL_ID_TONE_GENERATOR                              3
#define DEBUG_CHANNEL_ID_KEYPAD                                      4
#define DEBUG_CHANNEL_ID_MOTOR_VELOCITY_CONTROLLER                   5
#define DEBUG_CHANNEL_ID_FORCE_COIL                                  6
#define DEBUG_CHANNEL_ID_MOTOR_POSITION_CONTROLLER                   7

#define DEBUG_TELEMETRY_DEPTH                                        4000
#define DEBUG_TELEMETRY_BUFFER_SIZE_BYTES                            (DEBUG_TELEMETRY_DEPTH * 16)

#define DEBUG_MOTOR_VELOCITY_CONTROLLER_TELEMETRY_DEPTH              DEBUG_TELEMETRY_DEPTH
#define DEBUG_FORCE_COIL_TELEMETRY_DEPTH                             DEBUG_TELEMETRY_DEPTH
#define DEBUG_MOTOR_POSITION_CONTROLLER_TELEMETRY_DEPTH              DEBUG_TELEMETRY_DEPTH
#define DEBUG_MOTOR_POSITION_STALL_RELAX_DRIVE                       0.05f

#define DEBUG_PLL_TELEMETRY_DEPTH                                    DEBUG_TELEMETRY_DEPTH
#define DEBUG_PLL_DEFAULT_MAX_DURATION                               2.0f
#define DEBUG_PLL_DEFAULT_BONDING_ENERGY                             1e9f

/* UsImpedanceScannerModule --------------------------------------------------*/
#define SCANNER_SYNTHESIS_BUFFER_SIZE                                1440
#define SCANNER_ADC_CAPTURE_SIZE                                     1440
/* Discarded 4 ms captures before the measured one: the transducer rings
   up with tau = 2Q/ws (~1.8 ms at Q=333, longer unloaded); 8 iterations
   = 32 ms covers ~0.1% settling even at Q ~ 1000. */
#define SCANNER_WARMUP_ITERATIONS                                    8

/* ForceCoilDriverModule -----------------------------------------------------------*/
#define FORCE_COIL_MODULE_CURRENT_ERROR_TOLERANCE                    1e-2

#define FORCE_COIL_MODULE_CONTROL_FREQUENCY                          1000

#define FORCE_COIL_MODULE_V2I_CONVERSION_FACTOR                      1.2804097311139564
#define FORCE_COIL_MODULE_ZERO_CURRENT_VOLTAGE                       0.41917057903

#define FORCE_COIL_MODULE_PID_GAIN                                   0.25f
#define FORCE_COIL_MODULE_PID_INTEGRAL_TC                            0.005f
#define FORCE_COIL_MODULE_PID_DERIVATIVE_TC                          0.0f

#define FORCE_COIL_MODULE_PID_INPUT_FILTER_TC                        0.0

#define FORCE_COIL_MODULE_PID_OUTPUT_MIN                             0.0
#define FORCE_COIL_MODULE_PID_OUTPUT_MAX                             1.0

#define FORCE_COIL_MODULE_MIN_DUTY                                   0.0

/* DcMotorVelocityControllerModule (inner loop: tachometer -> PWM) -----------*/
#define DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY                    1000

/* Tachometer AnalogChannel wiring (Robot-level; feeds the velocity loop). */
#define ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC                     (3.2*48.925662f)
#define ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE               1.65

#define DCMOTOR_VELOCITY_MODULE_MIN_DUTY                             0.05f
#define DCMOTOR_VELOCITY_MODULE_MAX_DUTY                             0.95f
#define DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY                   0.5
#define DCMOTOR_VELOCITY_MODULE_VOLTAGE_TO_DUTY_SCALE                (1.0f / 30.0f)

#define DCMOTOR_VELOCITY_MODULE_CONTROLLER_PREAMPLIFIER_GAIN         2.5e-3

#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_RI                  1e3
#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_RF                  121e3
#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_CF                  1e-7

#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_CLAMP_MIN           -13.5
#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_CLAMP_MAX           13.5

/* DcMotorPositionControllerModule (LVDT -> velocity correction) ------------*/
#define DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY                    1000

#define DCMOTOR_POSITION_MODULE_PROPORTIONAL_GAIN                    100.0f
#define DCMOTOR_POSITION_MODULE_OUTPUT_MIN                           -40.0f
#define DCMOTOR_POSITION_MODULE_OUTPUT_MAX                           40.0f

#define DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR                   1e-2
#define DCMOTOR_POSITION_MODULE_MAX_VELOCITY_ERROR                   1e-2

/* RouterModule --------------------------------------------------------------*/
#define ROUTER_MODULE_SEGMENT_RENDER_FREQUENCY                       100.
#define ROUTER_MODULE_MAX_NUM_OF_ROUTERS                             4
#define ROUTER_MODULE_STEP_PER_MM                                    400

/* BonderModule --------------------------------------------------------------*/
/* Force coil currents (A) */
#define BONDER_MODULE_DEFAULT_FORCE_COIL_IDLE_CURRENT                0.1
#define BONDER_MODULE_DEFAULT_FORCE_COIL_SEARCHING_CURRENT           0.1
#define BONDER_MODULE_DEFAULT_FORCE_COIL_SETTLING_CURRENT            0.15
#define BONDER_MODULE_DEFAULT_FORCE_COIL_WELDING_CURRENT             0.5

/* Z-axis heights (mm) — Z increases upward, bond pad contact at z≈0 */
#define BONDER_MODULE_DEFAULT_RESET_HEIGHT                           8.0    /* retracted home position              */
#define BONDER_MODULE_DEFAULT_LOOP_HEIGHT                            3.0    /* apex of the wire loop                */
#define BONDER_MODULE_DEFAULT_SEARCH_HEIGHT                          1.5    /* start of controlled descent          */
#define BONDER_MODULE_DEFAULT_KINK_HEIGHT                            0.3    /* wire kink point, just above pad      */
#define BONDER_MODULE_DEFAULT_LOWEST_OVERTRAVEL                      (-0.3) /* maximum overtravel below pad surface */

/* XY-axis displacements (mm) */
#define BONDER_MODULE_DEFAULT_TAIL_DISPLACEMENT                      2.0  /* T-axis extension for tail formation  */
#define BONDER_MODULE_DEFAULT_TEAR_DISPLACEMENT                      1.5  /* T-axis pull for wire tear            */
#define BONDER_MODULE_DEFAULT_Y_REVERSE_DISPLACEMENT                 0.5  /* Y reverse move during loop forming   */
#define BONDER_MODULE_DEFAULT_Y_STEPBACK_DISPLACEMENT                0.3  /* Y stepback during 2nd-weld prep      */

/* Ultrasonic bonding */
#define BONDER_MODULE_DEFAULT_TARGET_POWER                           0.3    /* normalised power setpoint (0–1)      */
#define BONDER_MODULE_MAX_DRIVE_AMPLITUDE                            1.0    /* upper bound for computed US amplitude */
#define BONDER_MODULE_DEFAULT_BONDING_ENERGY                         0.01   /* bonding energy (J)                   */
#define BONDER_MODULE_DEFAULT_MAX_BONDING_DURATION                   0.5    /* safety timeout (s)                   */

/* Timing (s) */
#define BONDER_MODULE_DEFAULT_SETTLING_TIME                          0.05    /* wait after contact for force to settle */
#define BONDER_MODULE_DEFAULT_COOLING_TIME                           0.05    /* wait after weld for bond to solidify   */
#define BONDER_MODULE_DEFAULT_TAIL_RESTORE_DELAY                     0.1     /* delay before tail restore move         */
#define BONDER_MODULE_DEFAULT_Y_RESTORE_DELAY                        0.05    /* delay before Y restore move            */

/* Impedance scan sweep (passed to UsImpedanceScannerModule::Config) */
#define BONDER_MODULE_SCAN_MAX_FREQUENCIES                           64
#define BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES                   16
#define BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY                     56000.0
#define BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY                     64000.0
#define BONDER_MODULE_DEFAULT_SCAN_FREQUENCY_STEP \
    ((BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY - BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY) / \
      BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES)

/* Robot module --------------------------------------------------------------*/
#define ROBOT_Y_AXIS_MAX_VELOCITY                           0.05
#define ROBOT_Y_AXIS_MAX_ACCELERATION                       0.05
#define ROBOT_T_AXIS_MAX_VELOCITY                           0.05
#define ROBOT_T_AXIS_MAX_ACCELERATION                       0.05

#define ROBOT_BONDER_CONFIG_OBJECT_ID                       1

/* ADC1 configs -------------------------------------------------------------*/
#define ADC1_NUM_CONVERSIONS                                2
#define ADC1_BITS                                           12
#define ADC1_VOLTAGE_RANGE                                  3.3
#define ADC1_SAMPLES_PER_CHANNEL                            (ADC1_SAMPLING_FREQ / PLL_MODULE_CONTROL_FREQ)

/* ADC2 configs -------------------------------------------------------------*/
#define ADC2_NUM_CONVERSIONS                                4
#define ADC2_BITS                                           12
#define ADC2_VOLTAGE_RANGE                                  3.3
#define ADC2_SAMPLES_PER_CHANNEL                            (ADC2_SAMPLING_FREQ / FORCE_COIL_MODULE_CONTROL_FREQUENCY)

/* DAC1 configs -------------------------------------------------------------*/
#define DAC1_SAMPLES                                        (DAC1_SAMPLING_FREQ / PLL_MODULE_CONTROL_FREQ)
#define DAC1_BITS                                           12
#define DAC1_VOLTAGE_RANGE                                  3.3

/* DAC2 configs --------------------------------------------------------------*/
#define DAC2_SAMPLES                                        (DAC2_SAMPLING_FREQ / LVDT_MODULE_DRIVING_FREQUENCY)
#define DAC2_BITS                                           12
#define DAC2_VOLTAGE_RANGE                                  3.3

/* ADC Channels --------------------------------------------------------------*/
/* Divider: node * 0.1043740753040508 = ADC pin; gain converts pin volts
   back to the node voltage. */
#define ADC_CHANNEL_US_VSENS_GAIN                           (1.0 / 0.1043740753040508)
/* Shunt + amplifier transimpedance: 3.7272727272727266 V at the ADC pin
   per amp; gain converts pin volts back to amps. */
#define ADC_CHANNEL_US_ISENS_GAIN                           (1.0 / 3.7272727272727266)

/* V/I ADC sequencing skew: I-sense converts one rank after V-sense, so
   its phasor is advanced by 2*pi*f*skew. Bench-calibrated with a
   resistive load: -15.23 deg at 60 kHz -> 705 ns. Consumers rotate the
   current phasor by exp(-j*2*pi*f*skew) to restore true phase. */
#define ADC_CHANNEL_US_VI_SKEW_SECONDS                      7.05e-7f

#define ADC_CHANNEL_US_VSENS_CONVERSION_ORDER               0
#define ADC_CHANNEL_US_ISENS_CONVERSION_ORDER               1

#define ADC_CHANNEL_FORCE_COIL_ISENS_CONVERSION_ORDER       0
#define ADC_CHANNEL_ZMOTOR_TACHOMETER_CONVERSION_ORDER      1
#define ADC_CHANNEL_LVDT_A_CONVERSION_ORDER                 2
#define ADC_CHANNEL_LVDT_B_CONVERSION_ORDER                 3

#define ADC_CHANNEL_PLL_DEMODULATION_SAMPLES                (ADC1_SAMPLING_FREQ / PLL_MODULE_CONTROL_FREQ) 
#define ADC_CHANNEL_LVDT_DEMODULATION_SAMPLES               (ADC2_SAMPLING_FREQ / DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY)
#define ADC_CHANNEL_FORCE_COIL_ISENS_OVERSAMPLING_RATIO     (ADC2_SAMPLING_FREQ / FORCE_COIL_MODULE_CONTROL_FREQUENCY)
#define ADC_CHANNEL_ZMOTOR_TACHOMETER_OVERSAMPLING_RATIO    (ADC2_SAMPLING_FREQ / DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY)

/* TimerExpireService --------------------------------------------------------*/
#define TIMER_EXPIRE_SERVICE_MAX_HANDLES                    32
#define TIMER_EXPIRE_SERVICE_TICK_FREQUENCY                 1000

/* AdcService ----------------------------------------------------------------*/
#define ADC_SERVICE_MAX_HANDLES                             2
#define ADC_SERVICE_MAX_CHANNELS                            8

/* DacService ----------------------------------------------------------------*/
#define DAC_SERVICE_MAX_HANDLES                             1
#define DAC_SERVICE_MAX_CHANNELS                            4

/* PwmService ----------------------------------------------------------------*/
#define PWM_SERVICE_MAX_CHANNELS                            4

#define TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES       (TIM1_PWM_FREQ / FORCE_COIL_MODULE_CONTROL_FREQUENCY)
#define TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES       (TIM1_PWM_FREQ / DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY)

#define TIM1_PWM_CHANNEL1_SAMPLES                           TIM1_PWM_CHANNEL1_SEGMENT_LIFETIME_IN_SAMPLES
#define TIM1_PWM_CHANNEL2_SAMPLES                           TIM1_PWM_CHANNEL2_SEGMENT_LIFETIME_IN_SAMPLES

/* PinMonitorService ---------------------------------------------------------*/
#define PIN_MONITOR_SERVICE_MAX_PINS                        16
#define PIN_MONITOR_NORMAL_SAMPLING_FREQUENCY               50    /* 20 ms polling interval */
#define PIN_MONITOR_CRITICAL_SAMPLING_FREQUENCY             1000  /* 1 ms, every tick      */
#define PIN_MONITOR_BLIND_REGION_MS                         20U   /* post-callback lockout  */

/* ControlPanelService -------------------------------------------------------*/
#define CONTROL_PANEL_POLL_FREQUENCY                        40    /* 25 ms poll interval */

/* SolenoidService -----------------------------------------------------------*/
#define SOLENOID_SERVICE_MAX_INSTANCES                      8
#define SOLENOID_SERVICE_TRANSITION_TIME                    0.05
#define SOLENOID_SERVICE_EXECUTION_PERIOD                   0.01
#define SOLENOID_SERVICE_INIT_DELAY_MS                      100

/* StepperService ------------------------------------------------------------*/
#define STEPPER_SERVICE_DIR_PIN_INVERT                      false
#define STEPPER_SERVICE_MAX_MOTOR_COUNT                     4
#define STEPPER_SERVICE_SEGMENT_QUEUE_CAPACITY              32

/* ExpanderService -----------------------------------------------------------*/
#define IO_EXPANDER_SERVICE_MAX_HANDLES                     2
#define IO_EXPANDER_MAX_CHANNELS_PER_SERVICE                4

/* LCD expander (PCA9538, A0=A1=0) -------------------------------------------
 * PCA9538 wiring: P0=RS  P1=RW  P2=EN  P3=D4  P4=D5  P5=D6  P6=D7
 * Pin assignments are used internally by LcdModule; only address and direction
 * are needed here. All 8 pins are outputs → direction = 0x00.                */
#define LCD_EXPANDER_I2C_ADDRESS      0x73U
#define LCD_EXPANDER_DIRECTION        0x00U

/* Keypad expander (PCA9535) — all buttons and LEDs --------------------------
 * Single 16-bit expander serving the entire keypad (left + right panels).
 * Pin numbers are 1-based IDC positions: 1–8 = port 0 (IO0_0..IO0_7),
 *                                        9–16 = port 1 (IO1_0..IO1_7).
 * All button pins are inputs. LED pins (IO1_4..IO1_7) are outputs.          */
#define KEYPAD_EXPANDER_I2C_ADDRESS   0x27U
#if DEBUG_ENABLED
/* Bridge/test builds: ALL pins as inputs. */
#define KEYPAD_EXPANDER_PORT0_DIR     0xFFU
#define KEYPAD_EXPANDER_PORT1_DIR     0x0FU
#else
#define KEYPAD_EXPANDER_PORT0_DIR     0xFFU   /* all inputs */
#define KEYPAD_EXPANDER_PORT1_DIR     0x0FU   /* bits 0-3 inputs; bits 4-7 outputs (LEDs) */
#endif

/* Navigation */
#define KEYPAD_BTN_UP_A               1U    /* IO0_0 */
#define KEYPAD_BTN_UP_B               11U   /* IO1_2 */
#define KEYPAD_BTN_DOWN_A             6U    /* IO0_5 */
#define KEYPAD_BTN_DOWN_B             11U   /* IO1_2 */
#define KEYPAD_BTN_LEFT_A             3U    /* IO0_2 */
#define KEYPAD_BTN_LEFT_B             9U    /* IO1_0 */
#define KEYPAD_BTN_RIGHT_A            4U    /* IO0_3 */
#define KEYPAD_BTN_RIGHT_B            10U   /* IO1_1 */
#define KEYPAD_BTN_PLUS_A             4U    /* IO0_3 */
#define KEYPAD_BTN_PLUS_B             11U   /* IO1_2 */
#define KEYPAD_BTN_MINUS_A            3U    /* IO0_2 */
#define KEYPAD_BTN_MINUS_B            11U   /* IO1_2 */
#define KEYPAD_BTN_SAVE_A             2U    /* IO0_1 */
#define KEYPAD_BTN_SAVE_B             11U   /* IO1_2 */
#define KEYPAD_BTN_LOAD_A             1U    /* IO0_0 */
#define KEYPAD_BTN_LOAD_B             10U   /* IO1_1 */
#define KEYPAD_BTN_ENTER_A            1U    /* IO0_0 */
#define KEYPAD_BTN_ENTER_B            9U    /* IO1_0 */

/* Hotkeys */
#define KEYPAD_BTN_TAIL_PLUS_A        6U    /* IO0_5 */
#define KEYPAD_BTN_TAIL_PLUS_B        10U   /* IO1_1 */
#define KEYPAD_BTN_TAIL_MINUS_A       7U    /* IO0_6 */
#define KEYPAD_BTN_TAIL_MINUS_B       10U   /* IO1_1 */
#define KEYPAD_BTN_LOOP_PLUS_A        5U    /* IO0_4 */
#define KEYPAD_BTN_LOOP_PLUS_B        11U   /* IO1_2 */
#define KEYPAD_BTN_LOOP_MINUS_A       7U    /* IO0_6 */
#define KEYPAD_BTN_LOOP_MINUS_B       11U   /* IO1_2 */
#define KEYPAD_BTN_SEARCH_PLUS_A      5U    /* IO0_4 */
#define KEYPAD_BTN_SEARCH_PLUS_B      9U    /* IO1_0 */
#define KEYPAD_BTN_SEARCH_MINUS_A     7U    /* IO0_6 */
#define KEYPAD_BTN_SEARCH_MINUS_B     9U    /* IO1_0 */
#define KEYPAD_BTN_STEP_PLUS_A        3U    /* IO0_2 */
#define KEYPAD_BTN_STEP_PLUS_B        10U   /* IO1_1 */
#define KEYPAD_BTN_STEP_MINUS_A       5U    /* IO0_4 */
#define KEYPAD_BTN_STEP_MINUS_B       10U   /* IO1_1 */

/* Factory reset (right panel) */
/* Right panel (16-pin IDC, non-inverted; IDC 2 = GND). RESET was formerly
   named FACTORY_RESET — same pins, real panel label is RESET. */
#define KEYPAD_BTN_RESET_A            4U    /* IO0_3, IDC {14,7}  */
#define KEYPAD_BTN_RESET_B            12U   /* IO1_3 */
#define KEYPAD_BTN_TEST_A             3U    /* IO0_2, IDC {14,8}  */
#define KEYPAD_BTN_TEST_B             12U   /* IO1_3 */
#define KEYPAD_BTN_SETUP_A            6U    /* IO0_5, IDC {14,5}  */
#define KEYPAD_BTN_SETUP_B            12U   /* IO1_3 */
#define KEYPAD_BTN_LIGHT_A            5U    /* IO0_4, IDC {14,6}  */
#define KEYPAD_BTN_LIGHT_B            12U   /* IO1_3 */
#define KEYPAD_BTN_CLAMP_OPEN_A      7U    /* IO0_6, IDC {14,3}  */
#define KEYPAD_BTN_CLAMP_OPEN_B      12U   /* IO1_3 */
/* IDC 4 is marked NC in the intermediate-card map but the wiring pattern
   says IO0_7 — VERIFY on the bench (debug-keys) before trusting. */
#define KEYPAD_BTN_HIGH_RESET_A       8U    /* IO0_7?, IDC {14,4} */
#define KEYPAD_BTN_HIGH_RESET_B       12U   /* IO1_3 */

/* Right-panel LEDs: anode on expander pin, cathode hardwired to GND
   (IDC 2) — drive HIGH to light. */
#define KEYPAD_LED_TEST               13U   /* IO1_4, IDC 10 */
#define KEYPAD_LED_SETUP              14U   /* IO1_5, IDC 12 */
#define KEYPAD_LED_CLAMP_OPEN         15U   /* IO1_6, IDC 11 */

/* Left panel (14-pin IDC, inverted ribbon; derived from bench contact
   tests: MANUAL = IDC {3,11}, ESC/DEL = IDC {3,9}, ADD = IDC {3,7}). */
#define KEYPAD_BTN_MANUAL_A           6U    /* IO0_5 */
#define KEYPAD_BTN_MANUAL_B           9U    /* IO1_0 */
#define KEYPAD_BTN_ESC_DEL_A          4U    /* IO0_3 */
#define KEYPAD_BTN_ESC_DEL_B          9U    /* IO1_0 */
#define KEYPAD_BTN_ADD_A              2U    /* IO0_1 */
#define KEYPAD_BTN_ADD_B              9U    /* IO1_0 */

/* UiModule — parameter display intervals ------------------------------------*/
/* Force coil currents (A) */
#define UI_MODULE_FORCE_CURRENT_MIN                         0.0f
#define UI_MODULE_FORCE_CURRENT_MAX                         2.0f

/* Z-axis heights (mm) */
#define UI_MODULE_HEIGHT_MIN                                0.0f
#define UI_MODULE_HEIGHT_MAX                                20.0f
#define UI_MODULE_KINK_HEIGHT_MIN                          -2.0f
#define UI_MODULE_KINK_HEIGHT_MAX                           5.0f
#define UI_MODULE_OVERTRAVEL_MIN                           -2.0f
#define UI_MODULE_OVERTRAVEL_MAX                            0.0f

/* XY-axis displacements (mm) */
#define UI_MODULE_LARGE_DISPLACEMENT_MIN                    0.0f
#define UI_MODULE_LARGE_DISPLACEMENT_MAX                    10.0f
#define UI_MODULE_SMALL_DISPLACEMENT_MIN                    0.0f
#define UI_MODULE_SMALL_DISPLACEMENT_MAX                    5.0f

/* Ultrasonic bonding */
#define UI_MODULE_TARGET_POWER_MIN                          0.0f
#define UI_MODULE_TARGET_POWER_MAX                          1.0f
#define UI_MODULE_BONDING_ENERGY_MIN                        0.0f
#define UI_MODULE_BONDING_ENERGY_MAX                        1.0f
#define UI_MODULE_MAX_BONDING_DURATION_MIN                  0.0f
#define UI_MODULE_MAX_BONDING_DURATION_MAX                  60.0f

/* Timing (s) */
#define UI_MODULE_TIMING_MIN                                0.0f
#define UI_MODULE_TIMING_MAX                                5.0f

/* Impedance scan (frequency in kHz — display scale = 0.001 from Hz) */
#define UI_MODULE_SCAN_FREQ_MIN                             50.0f
#define UI_MODULE_SCAN_FREQ_MAX                             70.0f
#define UI_MODULE_SCAN_NUM_FREQS_MIN                        1.0f
#define UI_MODULE_SCAN_NUM_FREQS_MAX                        64.0f


#endif /* CONFIGURATION_H */
