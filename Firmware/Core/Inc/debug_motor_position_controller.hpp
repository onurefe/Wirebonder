#ifndef DEBUG_MOTOR_POSITION_CONTROLLER_HPP
#define DEBUG_MOTOR_POSITION_CONTROLLER_HPP

#include <cstdint>

#include "configuration.h"
#include "debug_service.hpp"
#include "dc_motor_position_controller_module.hpp"

struct DebugMotorPositionTelemetrySample {
    float position;
    float magA;
    float magB;
};

struct DebugMotorPositionStallTelemetrySample {
    float drive;
    float position;
    float magA;
    float magB;
};

class DebugMotorPositionController : public DebugChannel {
public:
    static constexpr uint16_t ChannelId = DEBUG_CHANNEL_ID_MOTOR_POSITION_CONTROLLER;

    enum Command : uint16_t {
        CMD_START = 1,
        CMD_STOP = 2,
        CMD_STALL_SCAN = 3
    };

    enum ResultCode : uint32_t {
        RESULT_CAPTURE_COMPLETE = 0
    };

    DebugMotorPositionController();

    void init(DcMotorPositionControllerModule *positionController);

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
    enum class Mode {
        STEP,
        STALL_SCAN
    };

    enum class StallScanPhase {
        SETTLING,
        RELAXING
    };

    static bool onProvidePositionSetpoint(void *context, float *positionSetpoint);

    void start();
    void startStallScan();
    void stop();
    bool provideStepSetpoint(float *positionSetpoint);
    bool provideStallScanSetpoint(float *positionSetpoint);
    void finish(uint32_t sampleCount);

    DcMotorPositionControllerModule *m_positionController = nullptr;

    DebugMotorPositionTelemetrySample *m_telemetry = nullptr;
    DebugMotorPositionStallTelemetrySample *m_stallTelemetry = nullptr;
    uint32_t m_sampleIdx;
    uint32_t m_sampleLimit;
    uint32_t m_settleIdx;
    uint32_t m_settleLimit;
    uint32_t m_relaxIdx;
    uint32_t m_relaxLimit;
    bool m_busy = false;

    Mode m_mode = Mode::STEP;
    StallScanPhase m_stallScanPhase = StallScanPhase::SETTLING;
    float m_stepValue;
    float m_currentDrive;
    float m_endDrive;
    float m_driveStep;
    float m_relaxDrive;
};

#endif /* DEBUG_MOTOR_POSITION_CONTROLLER_HPP */
