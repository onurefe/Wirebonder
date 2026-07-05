#include <gtest/gtest.h>
#include <cmath>

#include "transducer_analyzer.hpp"

// -----------------------------------------------------------------------
// Synthetic transducer (same parameter class as test_pll.cpp):
//   fs = 60 kHz, Q = 300, C0 = 3 nF, C1 = 60 pF
//   L1 = 1/(ws^2*C1), R1 = ws*L1/Q
// -----------------------------------------------------------------------
namespace {

constexpr float FS = 60000.f;
constexpr float Q  = 300.f;
constexpr float C0 = 3e-9f;
constexpr float C1 = 60e-12f;

const float WS = 2.f * static_cast<float>(M_PI) * FS;
const float L1 = 1.f / (WS * WS * C1);
const float R1 = WS * L1 / Q;

const float FP = FS * std::sqrt(1.f + C1 / C0);

complexf trueImpedance(float freq) {
    float w  = 2.f * static_cast<float>(M_PI) * freq;
    complexf zm = complexf_create(R1, w * L1 - 1.f / (w * C1));
    complexf ym = complexf_div(complexf_create(1.f, 0.f), zm);
    complexf y  = complexf_add(ym, complexf_create(0.f, w * C0));
    return complexf_div(complexf_create(1.f, 0.f), y);
}

void sampleGrid(complexf *out, uint8_t count, float startFreq, float step) {
    for (uint8_t i = 0; i < count; i++) {
        out[i] = trueImpedance(startFreq + i * step);
    }
}

} // namespace

TEST(TransducerAnalyzer, RecoversParametersFromFineGrid) {
    constexpr uint8_t N = 32;
    const float start = 59000.f;
    const float step  = (61500.f - start) / N;

    complexf z[N];
    sampleGrid(z, N, start, step);

    TransducerAnalyzer::Parameters p{};
    ASSERT_TRUE(TransducerAnalyzer::fit(z, N, start, step, p));

    EXPECT_NEAR(p.seriesResonance, FS, FS * 0.001f);      // 0.1 %
    EXPECT_NEAR(p.parallelResonance, FP, FP * 0.001f);
    EXPECT_NEAR(p.r1, R1, R1 * 0.02f);                    // 2 %
    EXPECT_NEAR(p.c0, C0, C0 * 0.05f);                    // 5 %
    EXPECT_NEAR(p.qFactor, Q, Q * 0.05f);
}

TEST(TransducerAnalyzer, BeatsGridResolutionOnCoarseScan) {
    // 8 points over 2.5 kHz -> ~312 Hz grid spacing, so only ~1 point falls
    // inside the fs/Q = 200 Hz resonance bandwidth. The nearest-grid-point
    // method can be off by half a step (~156 Hz); require the fit to do at
    // least 4x better than the grid (a quarter step).
    constexpr uint8_t N = 8;
    const float start = 59100.f;
    const float step  = 2500.f / N;

    complexf z[N];
    sampleGrid(z, N, start, step);

    TransducerAnalyzer::Parameters p{};
    ASSERT_TRUE(TransducerAnalyzer::fit(z, N, start, step, p));

    EXPECT_NEAR(p.seriesResonance, FS, step / 4.f);
}

TEST(TransducerAnalyzer, AdmittanceMatchesPlantModel) {
    constexpr uint8_t N = 32;
    const float start = 59000.f;
    const float step  = (61500.f - start) / N;

    complexf z[N];
    sampleGrid(z, N, start, step);

    TransducerAnalyzer::Parameters p{};
    ASSERT_TRUE(TransducerAnalyzer::fit(z, N, start, step, p));

    // Compare fitted vs true admittance at series resonance, where the
    // drive amplitude computation evaluates it.
    complexf yFit  = TransducerAnalyzer::admittance(p, FS);
    complexf zTrue = trueImpedance(FS);
    complexf yTrue = complexf_div(complexf_create(1.f, 0.f), zTrue);

    EXPECT_NEAR(yFit.re, yTrue.re, yTrue.re * 0.02f);
}

TEST(TransducerAnalyzer, RejectsTooFewPoints) {
    complexf z[3] = {trueImpedance(59500.f), trueImpedance(60000.f),
                     trueImpedance(60500.f)};
    TransducerAnalyzer::Parameters p{};
    EXPECT_FALSE(TransducerAnalyzer::fit(z, 3, 59500.f, 500.f, p));
}

TEST(TransducerAnalyzer, RejectsDegenerateData) {
    // Flat purely-resistive data has no resonance; the regression slope
    // and intercept cannot both be physical.
    constexpr uint8_t N = 16;
    complexf z[N];
    for (uint8_t i = 0; i < N; i++) {
        z[i] = complexf_create(100.f, 0.f);
    }

    TransducerAnalyzer::Parameters p{};
    EXPECT_FALSE(TransducerAnalyzer::fit(z, N, 59000.f, 100.f, p));
}

// -----------------------------------------------------------------------
// Frequency PID tuning
// -----------------------------------------------------------------------

TEST(TransducerAnalyzer, PidTuningMatchesLoopShapingConditions) {
    constexpr uint8_t N = 32;
    const float start = 59000.f;
    const float step  = (61500.f - start) / N;

    complexf z[N];
    sampleGrid(z, N, start, step);

    TransducerAnalyzer::Parameters p{};
    ASSERT_TRUE(TransducerAnalyzer::fit(z, N, start, step, p));

    // Dead time: 2-sample correction queue at 1 kHz control rate plus half
    // a demodulation window (~0.5 ms) -> 2.5 ms.
    const float theta = 2.5e-3f;
    const float margin = static_cast<float>(M_PI) / 3.f;  // 60 deg
    const float cross = 1.0f;                              // wc = 1/theta

    TransducerAnalyzer::FrequencyPidTuning t{};
    ASSERT_TRUE(TransducerAnalyzer::frequencyPidTuning(p, theta, margin, cross, t));

    // Verify the two design conditions directly on the open loop
    // L(jw) = Kp*(1 + 1/(jw*Ti)) * K * e^(-jw*theta) at w = wc.
    const float K  = 2.f * p.qFactor / p.seriesResonance;
    const float wc = cross / theta;

    float mag = t.gain * K * std::sqrt(1.f + 1.f / (wc * t.integralTc * wc * t.integralTc));
    float phase = -wc * theta - std::atan(1.f / (wc * t.integralTc));

    EXPECT_NEAR(mag, 1.f, 0.01f);                                      // |L(wc)| = 1
    EXPECT_NEAR(phase, -static_cast<float>(M_PI) + margin, 0.01f);     // margin met
    EXPECT_FLOAT_EQ(t.derivativeTc, 0.f);
}

TEST(TransducerAnalyzer, PidTuningRejectsInfeasibleMargin) {
    TransducerAnalyzer::Parameters p{};
    p.qFactor = 300.f;
    p.seriesResonance = 60000.f;

    TransducerAnalyzer::FrequencyPidTuning t{};

    // Margin + crossover phase already exceed 180 deg -> no PI can help.
    EXPECT_FALSE(TransducerAnalyzer::frequencyPidTuning(
        p, 2.5e-3f, 2.8f, 1.0f, t));

    // Tiny requested lag (margin + crossover ~ pi) is also infeasible.
    EXPECT_FALSE(TransducerAnalyzer::frequencyPidTuning(
        p, 2.5e-3f, static_cast<float>(M_PI) / 3.f, 0.05f, t));
}
