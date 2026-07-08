#include "pwm_service.hpp"

static PwmService *g_instance = nullptr;

// =======================================================================
// DirectPwmChannel
// =======================================================================
DirectPwmChannel::DirectPwmChannel(TIM_HandleTypeDef *htim, uint32_t timChannel)
    : m_htim(htim)
    , m_timChannel(timChannel)
{
}

void DirectPwmChannel::start(float initialDuty)
{
    __HAL_TIM_SET_COMPARE(m_htim, m_timChannel, dutyToCompare(initialDuty));
    HAL_TIM_PWM_Start(m_htim, m_timChannel);
}

void DirectPwmChannel::stop()
{
    HAL_TIM_PWM_Stop(m_htim, m_timChannel);
}

void DirectPwmChannel::setDuty(float duty)
{
    __HAL_TIM_SET_COMPARE(m_htim, m_timChannel, dutyToCompare(duty));
}

uint32_t DirectPwmChannel::dutyToCompare(float duty) const
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return static_cast<uint32_t>(duty * static_cast<float>(m_htim->Init.Period));
}

// =======================================================================
// PwmRampChannel
// =======================================================================
PwmRampChannel::PwmRampChannel(
    TIM_HandleTypeDef *htim,
    uint32_t timChannel,
    uint16_t* dmaBuffer,
    uint32_t dmaBufferLength,
    uint32_t segmentLifetimeInSamples,
    bool complementaryOutput)
    : m_htim(htim)
    , m_timChannel(timChannel)
    , m_dmaBuffer(dmaBuffer)
    , m_dmaBufferLength(dmaBufferLength)
    , m_segmentLifetimeInSamples(segmentLifetimeInSamples == 0U ? 1U : segmentLifetimeInSamples)
    , m_samplesIntoRamp(0U)
    , m_currentDuty(0.0f)
    , m_dutyStepPerSample(0.0f)
    , m_running(false)
    , m_complementaryOutput(complementaryOutput)
    , m_targetControllerCallbacks{}
    , m_targetControllerCallbackCount(0)
{
}

bool PwmRampChannel::addTargetDutyControllerCallback(void* context, TargetUpdateCallback targetUpdateCallback) {
    if (targetUpdateCallback == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < m_targetControllerCallbackCount; i++) {
        if (m_targetControllerCallbacks[i].context == context &&
            m_targetControllerCallbacks[i].callback == targetUpdateCallback) {
            return true;
        }
    }

    if (m_targetControllerCallbackCount >= kMaxTargetControllerCallbacks) {
        return false;
    }

    m_targetControllerCallbacks[m_targetControllerCallbackCount++] =
        TargetControllerRegistration{targetUpdateCallback, context};
    return true;
}

TIM_HandleTypeDef *PwmRampChannel::getTimHandle() const
{
    return m_htim;
}

uint32_t PwmRampChannel::getTimChannel() const {
    return m_timChannel;
}

uint16_t* PwmRampChannel::getDmaBuffer() const {
    return m_dmaBuffer;
}

uint32_t PwmRampChannel::getDmaBufferLength() const {
    return m_dmaBufferLength;
}

bool PwmRampChannel::isValid() const {
    return m_dmaBuffer != nullptr
        && m_dmaBufferLength >= 2U
        && (m_dmaBufferLength % 2U) == 0U;
}

bool PwmRampChannel::start(float initialDuty)
{
    m_currentDuty = clampDuty(initialDuty);
    initializeBuffer();

    // HAL takes pData as uint32_t*, but the DMA is configured for HALFWORD
    // transfers to the 16-bit CCR, so the buffer is uint16_t (see .msp DMA
    // init). Pass the address through; the DMA moves one 16-bit word per item.
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(m_htim,
        m_timChannel,
        reinterpret_cast<uint32_t*>(m_dmaBuffer),
        m_dmaBufferLength);

    // The DMA start above only enables the main output (CCxE). For a
    // complementary pair the CHxN output (e.g. the motor's nPWM pin) is driven
    // from the same compare register but needs its own enable (CCxNE).
    if (status == HAL_OK && m_complementaryOutput) {
        status = HAL_TIMEx_PWMN_Start(m_htim, m_timChannel);
    }

    m_running = (status == HAL_OK);
    return m_running;
}

void PwmRampChannel::stop(void)
{
    if (m_complementaryOutput) {
        HAL_TIMEx_PWMN_Stop(m_htim, m_timChannel);
    }

    HAL_TIM_PWM_Stop_DMA(m_htim, m_timChannel);
    m_running = false;
}

void PwmRampChannel::initializeBuffer() {
    // Called from start() before m_running is set, so it must NOT gate on
    // m_running (that would skip the pre-fill and leave the DMA playing a
    // zero/stale buffer until the first refill IRQ).
    if (!isValid()) {
        return;
    }

    const uint16_t compareValue = dutyToCompareValue(m_currentDuty);

    for (uint32_t i = 0U; i < m_dmaBufferLength; ++i) {
        m_dmaBuffer[i] = compareValue;
    }

    m_samplesIntoRamp = 0U;
    m_dutyStepPerSample = 0.0f;

    beginNextRamp();
}

void PwmRampChannel::refill(bool isSecondHalf) {
    if (!isValid()) {
        return;
    }

    const uint32_t halfLength = m_dmaBufferLength / 2U;

    uint16_t* dst = isSecondHalf ? &m_dmaBuffer[halfLength] : m_dmaBuffer;

    for (uint32_t i = 0U; i < halfLength; ++i) {
        if (m_samplesIntoRamp >= m_segmentLifetimeInSamples) {
            beginNextRamp();
        }

        dst[i] = dutyToCompareValue(m_currentDuty);

        m_currentDuty += m_dutyStepPerSample;
        ++m_samplesIntoRamp;
    }
}

void PwmRampChannel::beginNextRamp() {
    m_currentDuty = clampDuty(m_currentDuty);

    // First active controller supplies the target duty; if none is active the
    // ramp holds the current duty.
    float targetDuty = m_currentDuty;

    for (uint8_t i = 0; i < m_targetControllerCallbackCount; i++) {
        if (m_targetControllerCallbacks[i].callback(
                m_targetControllerCallbacks[i].context, &targetDuty)) {
            break;
        }
    }

    targetDuty = clampDuty(targetDuty);

    m_dutyStepPerSample =
        (targetDuty - m_currentDuty) / static_cast<float>(m_segmentLifetimeInSamples);

    m_samplesIntoRamp = 0U;
}

uint16_t PwmRampChannel::dutyToCompareValue(float duty) {
    uint32_t period = m_htim->Init.Period;

    return static_cast<uint16_t>(
        clampDuty(duty) * static_cast<float>(period)
    );
}

float PwmRampChannel::clampDuty(float duty) {
    if (duty < 0.0f) {
        return 0.0f;
    }

    if (duty > 1.0f) {
        return 1.0f;
    }

    return duty;
}

// =======================================================================
// PwmService
// =======================================================================
PwmService::PwmService()
    : m_channelCount(0U)
    , m_state(ServiceState::READY)
{
    for (uint8_t i = 0; i < PWM_SERVICE_MAX_CHANNELS; i++) {
        m_channels[i] = nullptr;
    }

    g_instance = this;
}

void PwmService::startService()
{
    if (m_state != ServiceState::READY) {
        return;
    }

    m_state = ServiceState::OPERATING;
}

void PwmService::stopService()
{
    if (m_state != ServiceState::OPERATING) {
        return;
    }

    for (uint8_t i = 0U; i < m_channelCount; ++i) {
        if (m_channels[i] != nullptr) {
            m_channels[i]->stop();
        }
    }

    m_state = ServiceState::READY;
}

bool PwmService::addChannel(IPwmChannel* channel) {
    if (channel == nullptr) {
        return false;
    }

    if (!channel->isValid()) {
        return false;
    }

    if (m_channelCount >= PWM_SERVICE_MAX_CHANNELS) {
        return false;
    }

    m_channels[m_channelCount++] = channel;
    return true;
}

IPwmChannel *PwmService::getChannel(TIM_HandleTypeDef* htim)
{
    if (g_instance == nullptr || g_instance->m_state != ServiceState::OPERATING) {
        return nullptr;
    }

    uint32_t activeTimChannel = activeChannelToTimChannel(htim->Channel);

    if (activeTimChannel == 0xFFFFFFFFU) {
        return nullptr;
    }

    for (uint8_t i = 0U; i < g_instance->m_channelCount; ++i) {
        IPwmChannel* channel = g_instance->m_channels[i];

        bool same_timer   = htim->Instance == channel->getTimHandle()->Instance;
        bool same_channel = channel->getTimChannel() == activeTimChannel;

        if (same_timer && same_channel) {
            return channel;
        }
    }

    return nullptr;
}

uint32_t PwmService::activeChannelToTimChannel(uint32_t activeChannel) {
    switch (activeChannel) {
        case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
        case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
        case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
        case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
        default: return 0xFFFFFFFFU;
    }
}

// =======================================================================
// C-Linkage HAL Callbacks
// =======================================================================
extern "C" {
    void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef* htim) {
        IPwmChannel* channel = PwmService::getChannel(htim);
        if (channel) {
            channel->refill(false);
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
        IPwmChannel* channel = PwmService::getChannel(htim);
        if (channel) {
            channel->refill(true);
        }
    }
}
