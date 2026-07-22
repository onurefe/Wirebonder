#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "main.h"

/* Eeprom emulator -----------------------------------------------------------*/
#define EEPROM_EMULATOR_FLASH_AREA_A_SECTOR                          FLASH_SECTOR_6
#define EEPROM_EMULATOR_FLASH_AREA_A_ADDR                            0x08040000UL
#define EEPROM_EMULATOR_FLASH_AREA_B_SECTOR                          FLASH_SECTOR_7
#define EEPROM_EMULATOR_FLASH_AREA_B_ADDR                            0x08060000UL
#define EEPROM_EMULATOR_FLASH_AREA_SIZE                              (128 * 1024)
#define EEPROM_EMULATOR_MAX_NUM_OF_ENTRIES                           1024

/* When enabled, a newly compiled EepromEmulator image logically clears all
   stored objects once, then records its __DATE__/__TIME__ build signature.
   Reboots and reflashing the exact same binary preserve the stored objects. */
#define EEPROM_RESET_ON_NEW_BUILD                                    1

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

/* Firmware operating mode ---------------------------------------------------- */
#define FIRMWARE_MODE_NORMAL                                         0
#define FIRMWARE_MODE_DEBUG_PLL                                      1
#define FIRMWARE_MODE_DEBUG_IMPEDANCE_SCANNER                        2
#define FIRMWARE_MODE_DEBUG_TONE_GENERATOR                           3
#define FIRMWARE_MODE_DEBUG_KEYPAD                                   4
#define FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY                           5
#define FIRMWARE_MODE_DEBUG_FORCE_COIL                               6
#define FIRMWARE_MODE_DEBUG_MOTOR_POSITION                           7
#define FIRMWARE_MODE_DEBUG_STEPPER_ROUTER                           8
#define FIRMWARE_MODE_DEBUG_LEDS                                     9
#define FIRMWARE_MODE_DEBUG_LCD                                      10
#define FIRMWARE_MODE_DEBUG_SOLENOIDS                                11
#define FIRMWARE_MODE_DEBUG_BONDER                                   12
#define FIRMWARE_MODE_DEBUG_HOMING                                   13

#define FIRMWARE_MODE                                                FIRMWARE_MODE_DEBUG_MOTOR_POSITION

/* Debug environment ids — assigned centrally, embedded in the upper half of
   the debugger command word so a mismatched host script / firmware image
   pairing is rejected instead of silently misinterpreted. Numbering keeps the
   historical debug-channel ids so the debugBridge front-ends are unchanged. */
#define DEBUG_ENVIRONMENT_ID_IMPEDANCE_SCANNER                       1
#define DEBUG_ENVIRONMENT_ID_PLL                                     2
#define DEBUG_ENVIRONMENT_ID_TONE_GENERATOR                          3
#define DEBUG_ENVIRONMENT_ID_KEYPAD                                  4
#define DEBUG_ENVIRONMENT_ID_MOTOR_VELOCITY_CONTROLLER               5
#define DEBUG_ENVIRONMENT_ID_FORCE_COIL                              6
#define DEBUG_ENVIRONMENT_ID_MOTOR_POSITION_CONTROLLER               7
#define DEBUG_ENVIRONMENT_ID_STEPPER_ROUTER                          8
#define DEBUG_ENVIRONMENT_ID_LEDS                                    9
#define DEBUG_ENVIRONMENT_ID_LCD                                     10
#define DEBUG_ENVIRONMENT_ID_SOLENOIDS                               12
#define DEBUG_ENVIRONMENT_ID_BONDER                                  13
#define DEBUG_ENVIRONMENT_ID_HOMING                                  14

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
/* The Newman-phased multitone is normalized so its measured peak sits at
   this fraction of the DAC half-range; the rest is clipping headroom. */
#define SCANNER_SYNTHESIS_PEAK_HEADROOM                              0.9f
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

/* Area light ---------------------------------------------------------------*/
/* Logical illumination duty. The hardware output is DRIVES_AREALIGHT_nPWM,
   so Robot converts this to the complementary raw TIM8 duty. */
#define AREA_LIGHT_PWM_DUTY_RATIO                                    0.5f

/* DcMotorVelocityControllerModule (inner loop: tachometer -> PWM) -----------*/
#define DCMOTOR_VELOCITY_MODULE_CONTROL_FREQUENCY                    1000

/* Tachometer AnalogChannel wiring (Robot-level; feeds the velocity loop). */
#define ZMOTOR_MODULE_TACHOMETER_V_TO_MM_PER_SEC                     (3.2*48.925662f)
#define ZMOTOR_MODULE_TACHOMETER_ZERO_VELOCITY_VOLTAGE               1.65

#define DCMOTOR_VELOCITY_MODULE_MIN_DUTY                             0.05f
#define DCMOTOR_VELOCITY_MODULE_MAX_DUTY                             0.95f
#define DCMOTOR_VELOCITY_MODULE_ZERO_VELOCITY_DUTY                   0.5
#define DCMOTOR_VELOCITY_MODULE_VOLTAGE_TO_DUTY_SCALE                (1.0f / 27.0f)

/* TODO: Reduce preamplifier gain but increase position module proportionality 
gain by the same factor. Changing position module limits can also be a viable option. */
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
#define DCMOTOR_POSITION_MODULE_MAX_POSITION_ERROR                   2.0e-1f /* mm */

/* RouterModule --------------------------------------------------------------*/
#define ROUTER_MODULE_SEGMENT_RENDER_FREQUENCY                       100.
#define ROUTER_MODULE_MAX_NUM_OF_ROUTERS                             4

#define ROUTER_MODULE_Y_AXIS_STEPS_PER_MM                            2000.0f
#define ROUTER_MODULE_T_AXIS_STEPS_PER_MM                            400.0f

/* BonderModule --------------------------------------------------------------*/
/* Force coil currents (A) */
#define BONDER_MODULE_DEFAULT_FORCE_COIL_CONSTANT_CURRENT            0.10
#define BONDER_MODULE_DEFAULT_FORCE_COIL_TRACKING_CURRENT            0.20
#define BONDER_MODULE_DEFAULT_FORCE_COIL_FIRST_BOND_CURRENT          0.30
#define BONDER_MODULE_DEFAULT_FORCE_COIL_SECOND_BOND_CURRENT         0.30

#define BONDER_MODULE_ZAXIS_WORKSPACE_SIZE                           9.0

/* Z-axis heights (mm) — Z increases upward, bond pad contact at z≈0 */
#define BONDER_MODULE_DEFAULT_RESET_HEIGHT                           8.0    /* retracted home position              */
#define BONDER_MODULE_DEFAULT_LOOP_HEIGHT                            5.0    /* apex of the wire loop                */
#define BONDER_MODULE_DEFAULT_FIRST_SEARCH_HEIGHT                    3.5    /* first-bond controlled descent        */
#define BONDER_MODULE_DEFAULT_SECOND_SEARCH_HEIGHT                   3.5    /* second-bond controlled descent       */
#define BONDER_MODULE_DEFAULT_KINK_HEIGHT                            2.5    /* wire kink point, just above pad      */
#define BONDER_MODULE_DEFAULT_LOWEST_OVERTRAVEL                      (-0.3) /* maximum overtravel below pad surface */
#define BONDER_MODULE_DEFAULT_MANUAL_LEVELING_RATE                   1.0    /* manual-mode Z jog rate (mm/s)        */
#define BONDER_MODULE_DEFAULT_SECOND_Z_HEIGHT                        2.5    /* table-tear height for the Y tail/tear */

/* Y/T logical positions (mm), relative to the router origin at startup.
   BonderModule subtracts the mean of the configured T endpoints, so the
   defaults below become tail=-0.75 mm and tear=+0.75 mm at runtime. */
#define BONDER_MODULE_DEFAULT_TAIL_POSITION                          2.0  /* uncentered T tail endpoint           */
#define BONDER_MODULE_DEFAULT_TEAR_POSITION                          3.5  /* uncentered T tear endpoint           */
#define BONDER_MODULE_DEFAULT_Y_REVERSE_POSITION                     0.5  /* Y position during loop formation     */
#define BONDER_MODULE_DEFAULT_Y_STEPBACK_POSITION                    0.8  /* Y position during second-bond prep   */
#define BONDER_MODULE_DEFAULT_Y_TAIL_POSITION                        2.0  /* table-tear Y position forming the tail */
#define BONDER_MODULE_DEFAULT_Y_TEAR_POSITION                        3.5  /* table-tear Y position for the wire tear */

/* Ultrasonic bonding */
#define BONDER_MODULE_DEFAULT_FIRST_BONDING_POWER                    0.4    /* first-bond electrical power (W)      */
#define BONDER_MODULE_DEFAULT_SECOND_BONDING_POWER                   0.4    /* second-bond electrical power (W)     */
#define BONDER_MODULE_DEFAULT_FIRST_BONDING_ENERGY                   0.020   /* first-bond energy (J)                */
#define BONDER_MODULE_DEFAULT_SECOND_BONDING_ENERGY                  0.020   /* second-bond energy (J)               */
#define BONDER_MODULE_DEFAULT_TAIL_ASSIST_POWER                      0.1    /* ultrasonic tail-assist power (W)     */
#define BONDER_MODULE_DEFAULT_TAIL_ASSIST_ENERGY                     0.1   /* ultrasonic tail-assist energy (J)    */
#define BONDER_MODULE_DEFAULT_FORCE_SETUP_DURATION                   5.0     /* setup force-measurement hold (s)     */
#define BONDER_MODULE_DEFAULT_MAX_BONDING_DURATION                   5.0    /* safety timeout (s)                   */

/* Timing (s) */
#define BONDER_MODULE_DEFAULT_SETTLING_TIME                          0.1    /* wait after contact for force to settle */
#define BONDER_MODULE_DEFAULT_COOLING_TIME                           0.1    /* wait after weld for bond to solidify   */
#define BONDER_MODULE_DEFAULT_TAIL_RESTORE_DELAY                     0.1     /* delay before tail restore move         */
#define BONDER_MODULE_DEFAULT_TEAR_STABILIZATION_TIME                0.05    /* wait after tear before restoring axes  */

/* Impedance scan sweep (passed to UsImpedanceScannerModule::Config).
   The tone step must stay on the capture window's coherence grid
   (ADC1_SAMPLING_FREQ / SCANNER_ADC_CAPTURE_SIZE = 250 Hz). */
#define BONDER_MODULE_SCAN_MAX_FREQUENCIES                           32
#define BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES                   16
#define BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY                     58000.0
#define BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY                     62000.0
#define BONDER_MODULE_DEFAULT_SCAN_FREQUENCY_STEP \
    ((BONDER_MODULE_DEFAULT_SCAN_MAX_FREQUENCY - BONDER_MODULE_DEFAULT_SCAN_MIN_FREQUENCY) / \
      BONDER_MODULE_DEFAULT_SCAN_NUM_FREQUENCIES)

/* Trust the transducer fit only when its series resonance lands within
   this many scan bins of the raw |Z| minimum; otherwise fall back to the
   minimum bin. A noise-pulled fit can put the PLL center outside its
   tracking clamp even when the regression converges. */
#define BONDER_MODULE_FIT_DIP_MAX_DEVIATION_BINS                     2.0f

/* Robot module --------------------------------------------------------------*/
#define ROBOT_Y_AXIS_MAX_VELOCITY                           50.0
#define ROBOT_Y_AXIS_MAX_ACCELERATION                       50.0
/* Homing direction: -1 seeks the workspace origin (0 mm), +1 seeks the
   far boundary (WORKSPACE_SIZE_MM). The post-home coordinate range is always
   0..WORKSPACE_SIZE_MM regardless of which end owns the limit switch. */
#define ROBOT_Y_AXIS_HOMING_DIRECTION                       (-1)
#define ROBOT_Y_AXIS_WORKSPACE_SIZE_MM                      18.0f
#define ROBOT_Y_AXIS_HOMING_VELOCITY_MM_PER_S               2.0f
#define ROBOT_Y_AXIS_HOMING_BACKOFF_MM                      1.0f
#define ROBOT_Y_AXIS_HOMING_SEARCH_MARGIN_MM                1.0f
#define ROBOT_T_AXIS_MAX_VELOCITY                           2.5
#define ROBOT_T_AXIS_MAX_ACCELERATION                       10.0

#define ROBOT_ACTIVE_CONFIG_OBJECT_ID                       1U
#define ROBOT_CONFIG_OBJECT_ID_BASE                         0x0100U
#define ROBOT_CONFIGURATION_NAME_SIZE                       20U

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

/* Ultrasonic drive chain gain: transducer volts per DAC volt (input LPF +
   power amplifier + ZOH). Bench-measured with debug-tone at 59 kHz:
   0.9 V DAC amplitude -> 9.925 V at the transducer. */
#define US_DRIVE_CHAIN_GAIN                                 11.0278

/* DAC2 configs --------------------------------------------------------------*/
#define DAC2_SAMPLES                                        (DAC2_SAMPLING_FREQ / LVDT_MODULE_DRIVING_FREQUENCY)
#define DAC2_BITS                                           12
#define DAC2_VOLTAGE_RANGE                                  3.3

/* ADC Channels --------------------------------------------------------------*/
/* LPF at the V-sense input attenuates the signal by this factor at 60 kHz
   (measured with an oscilloscope); folded into the V-sense gain below. */
#define ADC_CHANNEL_US_VSENS_LPF_ATTENUATION                0.90776255707
/* Divider: node * 0.1043740753040508 = ADC pin; gain converts pin volts
   back to the node voltage, correcting the input-LPF attenuation. */
#define ADC_CHANNEL_US_VSENS_GAIN                           (1.0 / (0.1043740753040508 * ADC_CHANNEL_US_VSENS_LPF_ATTENUATION))
/* LPF at the I-sense input attenuates the signal by this factor at 59 kHz;
   calibrated with a 187.2 ohm precision resistor (measured |Z| read
   201.8 ohm with the V-sense already corrected: 187.2 / 201.8). */
#define ADC_CHANNEL_US_ISENS_LPF_ATTENUATION                0.9276511397
/* Shunt + amplifier transimpedance: 3.7272727272727266 V at the ADC pin
   per amp; gain converts pin volts back to amps, correcting the input-LPF
   attenuation. */
/* Current-transformer amplitude calibration: with the CT installed a
   187.5 ohm resistor read |Z| = 164.68 ohm (mean of a 58-61.75 kHz scan,
   flat across the band; bench 2026-07-19), i.e. the CT path delivers
   187.5/164.68 more pin volts per amp than the transimpedance above
   assumes. */
#define ADC_CHANNEL_US_ISENS_CT_EXTRA_TRANSIMPEDANCE        (187.5 / 164.676)
#define ADC_CHANNEL_US_ISENS_GAIN                           (1.0 / (3.7272727272727266 * ADC_CHANNEL_US_ISENS_LPF_ATTENUATION * ADC_CHANNEL_US_ISENS_CT_EXTRA_TRANSIMPEDANCE))

/* V/I ADC sequencing skew: I-sense converts one rank after V-sense, so
   its phasor is advanced by 2*pi*f*skew. Bench-calibrated with a
   resistive load: -15.23 deg at 60 kHz -> 705 ns. Consumers rotate the
   current phasor by exp(-j*2*pi*f*skew) to restore true phase. */
#define ADC_CHANNEL_US_VI_SKEW_SECONDS                      7.05e-7f

/* Current-transformer phase lead: with the CT installed the same 187.5 ohm
   resistor scan read a constant -2.64 deg impedance phase across
   58-61.75 kHz (frequency-independent, so a CT phase lead rather than
   additional time skew). Consumers add this to the skew angle when
   rotating the current phasor by exp(-j*theta). */
#define ADC_CHANNEL_US_ISENS_PHASE_LEAD_RAD                 0.04607f

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
#define KEYPAD_BTN_DEBOUNCE_MS                              60U   /* min stable-released time
                                                                     before a press counts */

/* SolenoidService -----------------------------------------------------------*/
#define SOLENOID_SERVICE_MAX_INSTANCES                      8
#define SOLENOID_SERVICE_TRANSITION_TIME                    0.05
#define SOLENOID_SERVICE_EXECUTION_PERIOD                   0.01
#define SOLENOID_SERVICE_INIT_DELAY_MS                      100

/* Clamp solenoid (non-latching): mechanical transition times in seconds. */
#define CLAMP_SOLENOID_ENERGIZE_TIME                        0.4
#define CLAMP_SOLENOID_DEENERGIZE_TIME                      0.4

/* Auxiliary non-latching solenoids: mechanical transition times in seconds. */
#define SOL1_SOLENOID_ENERGIZE_TIME                         0.2
#define SOL1_SOLENOID_DEENERGIZE_TIME                       0.2
#define SOL2_SOLENOID_ENERGIZE_TIME                         0.2
#define SOL2_SOLENOID_DEENERGIZE_TIME                       0.2

/* StepperService ------------------------------------------------------------*/
#define STEPPER_SERVICE_DIR_PIN_INVERT                      false
#define STEPPER_SERVICE_MAX_MOTOR_COUNT                     4
#define STEPPER_SERVICE_SEGMENT_QUEUE_CAPACITY              32

/* ExpanderService -----------------------------------------------------------*/
#define IO_EXPANDER_SERVICE_MAX_HANDLES                     2
#define IO_EXPANDER_MAX_CHANNELS_PER_SERVICE                4

/* LCD expander (PCA9538, A0=A1=0) -------------------------------------------
 * PCA9538 wiring: P0=RS  P1=RW  P2=EN  P3=D4  P4=D5  P5=D6  P6=D7
 * Pin assignments are used internally by LcdControllerModule; only address and direction
 * are needed here. All 8 pins are outputs → direction = 0x00.                */
#define LCD_EXPANDER_I2C_ADDRESS      0x73U
#define LCD_EXPANDER_DIRECTION        0x00U

/* Keypad expander (PCA9535) — all buttons and LEDs --------------------------
 * Single 16-bit expander serving the entire keypad (left + right panels).
 * Pin numbers are 1-based IDC positions: 1–8 = port 0 (IO0_0..IO0_7),
 *                                        9–16 = port 1 (IO1_0..IO1_7).
 * All button pins are inputs. LED pins (IO1_4..IO1_7) are outputs.          */
#define KEYPAD_EXPANDER_I2C_ADDRESS   0x27U
#define KEYPAD_EXPANDER_PORT0_DIR     0xFFU   /* all inputs */
#define KEYPAD_EXPANDER_PORT1_DIR     0x0FU   /* bits 0-3 inputs; bits 4-7 outputs (LEDs) */

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
#define KEYPAD_LED_MANUAL             16U   /* IO1_7 */

/* UserInterfaceModule -------------------------------------------------------*/
#define USER_INTERFACE_MODULE_REPORT_DURATION_MS                     5000U

/* Left panel (14-pin IDC, inverted ribbon; derived from bench contact
   tests: MANUAL = IDC {3,11}, ESC/DEL = IDC {3,9}, ADD = IDC {3,7}). */
#define KEYPAD_BTN_MANUAL_A           6U    /* IO0_5 */
#define KEYPAD_BTN_MANUAL_B           9U    /* IO1_0 */
#define KEYPAD_BTN_ESC_DEL_A          4U    /* IO0_3 */
#define KEYPAD_BTN_ESC_DEL_B          9U    /* IO1_0 */
#define KEYPAD_BTN_ADD_A              2U    /* IO0_1 */
#define KEYPAD_BTN_ADD_B              9U    /* IO1_0 */

/* ConfigurationEditor — parameter display intervals ------------------*/
/* Force coil currents (A) */
#define CONFIGURATION_EDITOR_FORCE_CURRENT_MIN                         0.0f
#define CONFIGURATION_EDITOR_FORCE_CURRENT_MAX                         2.0f

/* Z-axis heights (mm) */
#define CONFIGURATION_EDITOR_HEIGHT_MIN                                0.0f
#define CONFIGURATION_EDITOR_HEIGHT_MAX                                20.0f
#define CONFIGURATION_EDITOR_KINK_HEIGHT_MIN                          -2.0f
#define CONFIGURATION_EDITOR_KINK_HEIGHT_MAX                           5.0f
#define CONFIGURATION_EDITOR_OVERTRAVEL_MIN                           -2.0f
#define CONFIGURATION_EDITOR_OVERTRAVEL_MAX                            0.0f

/* XY-axis displacements (mm) */
#define CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MIN                    0.0f
#define CONFIGURATION_EDITOR_LARGE_DISPLACEMENT_MAX                    10.0f
#define CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MIN                    0.0f
#define CONFIGURATION_EDITOR_SMALL_DISPLACEMENT_MAX                    5.0f

/* Ultrasonic bonding */
#define CONFIGURATION_EDITOR_TARGET_POWER_MIN                          0.0f
#define CONFIGURATION_EDITOR_TARGET_POWER_MAX                          1.0f
#define CONFIGURATION_EDITOR_BONDING_ENERGY_MIN                        0.0f
#define CONFIGURATION_EDITOR_BONDING_ENERGY_MAX                        1.0f
#define CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MIN                  0.0f
#define CONFIGURATION_EDITOR_MAX_BONDING_DURATION_MAX                  60.0f

/* Timing (s) */
#define CONFIGURATION_EDITOR_TIMING_MIN                                0.0f
#define CONFIGURATION_EDITOR_TIMING_MAX                                5.0f

/* Impedance scan (frequency in kHz — display scale = 0.001 from Hz) */
#define CONFIGURATION_EDITOR_SCAN_FREQ_MIN                             50.0f
#define CONFIGURATION_EDITOR_SCAN_FREQ_MAX                             70.0f
#define CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MIN                        1.0f
#define CONFIGURATION_EDITOR_SCAN_NUM_FREQS_MAX                        64.0f


#endif /* CONFIGURATION_H */
