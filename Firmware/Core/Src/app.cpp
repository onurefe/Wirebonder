#include "app.h"
#include "configuration.h"

#if FIRMWARE_MODE == FIRMWARE_MODE_NORMAL
#include "robot.hpp"
using ActiveApplication = Robot;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_PLL
#include "DebugEnvironment/debug_environment_pll.hpp"
using ActiveApplication = PllDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_IMPEDANCE_SCANNER
#include "DebugEnvironment/debug_environment_impedance_scanner.hpp"
using ActiveApplication = ImpedanceScannerDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_TONE_GENERATOR
#include "DebugEnvironment/debug_environment_tone_generator.hpp"
using ActiveApplication = ToneGeneratorDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_KEYPAD
#include "DebugEnvironment/debug_environment_keypad.hpp"
using ActiveApplication = KeypadDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY
#include "DebugEnvironment/debug_environment_motor_velocity.hpp"
using ActiveApplication = MotorVelocityDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_FORCE_COIL
#include "DebugEnvironment/debug_environment_force_coil.hpp"
using ActiveApplication = ForceCoilDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_MOTOR_POSITION
#include "DebugEnvironment/debug_environment_motor_position.hpp"
using ActiveApplication = MotorPositionDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_STEPPER_ROUTER
#include "DebugEnvironment/debug_environment_stepper_router.hpp"
using ActiveApplication = StepperRouterDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LEDS
#include "DebugEnvironment/debug_environment_leds.hpp"
using ActiveApplication = LedsDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_LCD
#include "DebugEnvironment/debug_environment_lcd.hpp"
using ActiveApplication = LcdDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_SOLENOIDS
#include "DebugEnvironment/debug_environment_solenoids.hpp"
using ActiveApplication = SolenoidsDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_BONDER
#include "DebugEnvironment/debug_environment_bonder.hpp"
using ActiveApplication = BonderDebugEnvironment;
#elif FIRMWARE_MODE == FIRMWARE_MODE_DEBUG_HOMING
#include "DebugEnvironment/debug_environment_homing.hpp"
using ActiveApplication = HomingDebugEnvironment;
#else
#error "FIRMWARE_MODE selects no application"
#endif

static ActiveApplication *g_application = nullptr;

extern "C" {

void App_Init(void)
{
    if (g_application) {
        return;
    }

    static ActiveApplication instance;
    g_application = &instance;
}

void App_Start(void)
{
    if (g_application) {
        g_application->start();
    }
}

void App_Execute(void)
{
    if (g_application) {
        g_application->execute();
    }
}

} // extern "C"
