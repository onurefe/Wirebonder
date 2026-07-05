#ifndef PWM_SERVICE_HPP
#define PWM_SERVICE_HPP

#include "stm32f4xx_hal.h"
#include "configuration.h"
#include "generic.h"

#include <cstdint>

// -----------------------------------------------------------------------
// Interface: IPwmChannel
// -----------------------------------------------------------------------
class IPwmChannel {
public:
    virtual ~IPwmChannel() = default;

    virtual void initializeBuffer() = 0;
    virtual void refill(bool isSecondHalf) = 0;
    virtual void stop() = 0;

    virtual TIM_HandleTypeDef *getTimHandle() const = 0;
    virtual uint32_t getTimChannel() const = 0;
    virtual uint32_t* getDmaBuffer() const = 0;
    virtual uint32_t getDmaBufferLength() const = 0;

    virtual bool isValid() const = 0;
};

// -----------------------------------------------------------------------
// Class: PwmRampChannel
// -----------------------------------------------------------------------
class PwmRampChannel final : public IPwmChannel {
public:
    PwmRampChannel(
        TIM_HandleTypeDef *htim,
        uint32_t timChannel,
        uint32_t* dmaBuffer,
        uint32_t dmaBufferLength,
        uint32_t segmentLifetimeInSamples,
        // Advanced-timer (TIM1/TIM8) channels wired as a complementary pair
        // (CHx + CHxN) must also enable the CHxN output. Leave false for a
        // plain single-ended channel.
        bool complementaryOutput = false);
    
    // Returns true when the controller is active; the target duty is written
    // through the pointer. The first active controller wins (see beginNextRamp).
    using TargetUpdateCallback = bool (*)(void* context, float *targetDuty);
    bool addTargetDutyControllerCallback(void* context, TargetUpdateCallback targetUpdateCallback);

    bool start(float initialDuty);
    void stop() override;

    void initializeBuffer() override;
    void refill(bool isSecondHalf) override;
    
    TIM_HandleTypeDef *getTimHandle() const override;

    uint32_t getTimChannel() const override;
    uint32_t* getDmaBuffer() const override;
    uint32_t getDmaBufferLength() const override;

    bool isValid() const override;
    bool isRunning() const { return m_running; }

private:
    void beginNextRamp();

    uint32_t dutyToCompareValue(float duty);
    float clampDuty(float duty);
private:
    TIM_HandleTypeDef* m_htim;
    uint32_t m_timChannel;

    uint32_t* m_dmaBuffer;
    uint32_t m_dmaBufferLength;

    uint32_t m_segmentLifetimeInSamples;
    uint32_t m_samplesIntoRamp;

    float m_currentDuty;
    float m_dutyStepPerSample;
    bool m_running;
    bool m_complementaryOutput;

    struct TargetControllerRegistration {
        TargetUpdateCallback callback;
        void* context;
    };

    static constexpr uint8_t kMaxTargetControllerCallbacks = 4U;

    TargetControllerRegistration m_targetControllerCallbacks[kMaxTargetControllerCallbacks];
    uint8_t m_targetControllerCallbackCount;
};

// -----------------------------------------------------------------------
// Class: DirectPwmChannel
//
// Simple PWM output with no DMA. The compare register is written
// immediately each time setDuty() is called. Suitable for low-rate
// outputs (solenoids, heaters, fans) that do not need sample-accurate
// waveform shaping.
//
// The timer must already be configured for PWM output mode by CubeMX.
// -----------------------------------------------------------------------
class DirectPwmChannel {
public:
    DirectPwmChannel(TIM_HandleTypeDef *htim, uint32_t timChannel);

    void start(float initialDuty);
    void stop();
    void setDuty(float duty);

private:
    uint32_t dutyToCompare(float duty) const;

    TIM_HandleTypeDef *m_htim;
    uint32_t           m_timChannel;
};

// -----------------------------------------------------------------------
// Class: PwmService
// -----------------------------------------------------------------------
class PwmService {
public:
    PwmService();

    bool addChannel(IPwmChannel* channel);

    void initService() {}
    void startService();
    void stopService();
    void executeService() {}
    bool isOperating() const { return m_state == ServiceState::OPERATING; }

    static IPwmChannel *getChannel(TIM_HandleTypeDef* htim);
    static uint32_t activeChannelToTimChannel(uint32_t activeChannel);

private:
    IPwmChannel *m_channels[PWM_SERVICE_MAX_CHANNELS];
    uint8_t      m_channelCount;
    ServiceState m_state;
};

#endif // PWM_SERVICE_HPP
