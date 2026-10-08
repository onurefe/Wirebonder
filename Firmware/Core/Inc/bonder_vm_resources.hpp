#pragma once

#include <atomic>
#include <cstdint>
#include "generic.h"
#include "configuration.h"
#include "complex.h"
#include "timer_expire_service.hpp"
#include "solenoid_service.hpp"
#include "fast_io.hpp"
#include "pin_monitor_service.hpp"
#include "stepper_router_service.hpp"
#include "dc_motor_position_controller_module.hpp"
#include "force_coil_module.hpp"
#include "pll_module.hpp"
#include "us_impedance_scanner_module.hpp"

class BonderVMResources {
    public:
    BonderVMResources(DcMotorPositionControllerModule *zMotorController,
        ForceCoilDriverModule *forceCoilDriver,
        RouterChannel *yAxisRouter,
        RouterChannel *tAxisRouter,
        PllModule *pll,
        UsImpedanceScannerModule *impedanceScanner,
        SolenoidChannel *clampSolenoid,
        PinMonitorChannel *contactSensorMonitor,
        PinMonitorChannel *leftMouseButtonMonitor,
        PinMonitorChannel *rightMouseButtonMonitor,
        Timer *timer):
        m_zMotorController(zMotorController)
        , m_forceCoilDriver(forceCoilDriver)
        , m_yAxisRouter(yAxisRouter)
        , m_tAxisRouter(tAxisRouter)
        , m_pll(pll)
        , m_usImpedanceScanner(impedanceScanner)
        , m_clampSolenoid(clampSolenoid)
        , m_contactSensorMonitor(contactSensorMonitor)
        , m_leftMouseButtonMonitor(leftMouseButtonMonitor)
        , m_rightMouseButtonMonitor(rightMouseButtonMonitor)
        , m_timer(timer)
    {
    }

    DcMotorPositionControllerModule *m_zMotorController;
    ForceCoilDriverModule *m_forceCoilDriver;
    RouterChannel *m_yAxisRouter;
    RouterChannel *m_tAxisRouter;
    PllModule *m_pll;
    UsImpedanceScannerModule *m_usImpedanceScanner;
    SolenoidChannel *m_clampSolenoid;
    PinMonitorChannel *m_contactSensorMonitor;
    PinMonitorChannel *m_leftMouseButtonMonitor;
    PinMonitorChannel *m_rightMouseButtonMonitor;
    Timer *m_timer;
};
