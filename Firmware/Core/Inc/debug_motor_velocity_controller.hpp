#ifndef DEBUG_MOTOR_VELOCITY_CONTROLLER_HPP
#define DEBUG_MOTOR_VELOCITY_CONTROLLER_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "dc_motor_velocity_controller_module.hpp"
#include "timer_expire_service.hpp"

class DebugMotorVelocityController : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_MOTOR_VELOCITY_CONTROLLER;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2
    };

    enum ResultCode : uint32_t {
        RESULT_SETPOINT_ACHIEVED = 0
    };

    DebugMotorVelocityController();

    void init(DcMotorVelocityControllerModule *velocityController);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    static void onVelocityMeasured(void *context, float measuredVelocity);
    static bool onVelocityControlUpdate(void *context, float *targetVelocity);

    void start();
    void stop();

    DcMotorVelocityControllerModule *m_velocityController = nullptr;

    float *m_telemetry = nullptr;
    uint32_t m_sampleIdx;
    uint32_t m_sampleLimit;
    bool m_busy = false;

    float m_stepValue;
};

#endif
