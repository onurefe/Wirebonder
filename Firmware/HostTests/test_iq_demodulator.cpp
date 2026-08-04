#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "adc_service.hpp"

static std::vector<uint16_t> makeSine(float freq, uint32_t N, float amp, float offset) {
    std::vector<uint16_t> s(N);
    for (uint32_t n = 0; n < N; n++) {
        float v = offset + amp * sinf(2.f * static_cast<float>(M_PI) * freq * static_cast<float>(n));
        s[n] = static_cast<uint16_t>(std::max(0.f, std::min(4095.f, v + 0.5f)));
    }
    return s;
}

class IQDemodTest : public ::testing::Test {
protected:
    static constexpr uint32_t N    = 360;
    static constexpr float    FREQ = 6.f / 360.f;   // 6 exact cycles — no spectral leakage
    static constexpr uint8_t  BITS = 12;
    static constexpr float    VR   = 3.3f;
    static constexpr float    AMP  = 512.f;          // ADC counts peak amplitude
    static constexpr float    OFF  = 2048.f;         // ADC counts DC offset

    float re = 0.f, im = 0.f;

    static void cb(void *ctx, float r, float i) {
        auto *self = static_cast<IQDemodTest *>(ctx);
        self->re = r;
        self->im = i;
    }

    void run(std::vector<uint16_t> &samples, float freq = FREQ) {
        IQDemodulatorChannel ch(0, N, freq, 1.f);
        ch.addMeasurementListenerCallback(this, cb);
        ch.enable();
        InterleavedBuffer buf(reinterpret_cast<volatile uint16_t *>(samples.data()), 0, 1);
        ch.process(buf, N, BITS, VR);
    }
};

TEST_F(IQDemodTest, OnFrequencyMagnitude) {
    auto s = makeSine(FREQ, N, AMP, OFF);
    run(s);
    float expected = AMP * VR / 4096.f;
    float mag = std::sqrt(re * re + im * im);
    EXPECT_NEAR(mag, expected, expected * 0.02f);
}

TEST_F(IQDemodTest, OffFrequencyRejection) {
    auto s = makeSine(FREQ * 2.f, N, AMP, OFF);
    run(s, FREQ);
    float expected = AMP * VR / 4096.f;
    EXPECT_LT(std::sqrt(re * re + im * im), expected * 0.1f);
}

TEST_F(IQDemodTest, DCRejection) {
    std::vector<uint16_t> s(N, static_cast<uint16_t>(OFF));
    run(s);
    float expected = AMP * VR / 4096.f;
    EXPECT_LT(std::sqrt(re * re + im * im), expected * 0.05f);
}

TEST_F(IQDemodTest, InterleavedTwoChannel) {
    auto tone = makeSine(FREQ, N, AMP, OFF);
    std::vector<uint16_t> interleaved(N * 2);
    for (uint32_t i = 0; i < N; i++) {
        interleaved[i * 2]     = 2048;      // channel 0 — filler
        interleaved[i * 2 + 1] = tone[i];   // channel 1 — signal
    }
    IQDemodulatorChannel ch(1, N, FREQ, 1.f);
    ch.addMeasurementListenerCallback(this, cb);
    ch.enable();
    InterleavedBuffer buf(reinterpret_cast<volatile uint16_t *>(interleaved.data()), 1, 2);
    ch.process(buf, N, BITS, VR);
    float expected = AMP * VR / 4096.f;
    EXPECT_NEAR(std::sqrt(re * re + im * im), expected, expected * 0.02f);
}
