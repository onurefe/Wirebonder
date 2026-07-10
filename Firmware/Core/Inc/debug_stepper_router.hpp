#ifndef DEBUG_STEPPER_ROUTER_HPP
#define DEBUG_STEPPER_ROUTER_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "stepper_router_service.hpp"

class DebugStepperRouter : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_STEPPER_ROUTER;

    enum Command : uint16_t {
        CMD_MOVE = 1,
        CMD_STOP = 2
    };

    enum Axis : uint32_t {
        AXIS_Y = 0,
        AXIS_T = 1
    };

    enum ResultCode : uint32_t {
        RESULT_MOVE_COMPLETE = 0,
        RESULT_STOPPED = 1
    };

    DebugStepperRouter();

    void init(RouterChannel *yAxisRouter, RouterChannel *tAxisRouter);

    uint16_t channelId() const override
    {
        return ChannelId;
    }

    bool canRunWhileBusy(uint16_t localCommand) const override
    {
        return localCommand == CMD_STOP;
    }

    void handleCommand(uint16_t localCommand) override;
    void poll() override;

    bool isBusy() const override
    {
        return m_busy;
    }

    void abort() override;

private:
    RouterChannel *routerForAxis(uint32_t axis) const;

    void move();
    void stop(uint32_t resultCode);

    RouterChannel *m_yAxisRouter = nullptr;
    RouterChannel *m_tAxisRouter = nullptr;
    RouterChannel *m_activeRouter = nullptr;
    uint32_t m_activeAxis = AXIS_Y;
    bool m_busy = false;
};

#endif /* DEBUG_STEPPER_ROUTER_HPP */
