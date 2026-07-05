#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "zaxis_analyzer.hpp"
#include "pid_controller.hpp"

// -----------------------------------------------------------------------
// Synthetic Z drive: first-order duty -> velocity plant sampled at the
// 250 Hz control rate of the Z motor module. The capture consists of
// calibrated LVDT positions (exact integral of the velocity) and raw
// tachometer voltages with a known gain and standstill offset.
// -----------------------------------------------------------------------
namespace {

constexpr float KM     = 8.0f;    // (mm/s) per unit duty
constexpr float TAU    = 0.05f;   // s
constexpr float FS     = 250.0f;  // control rate (Hz)
constexpr float STEP   = 0.2f;    // duty step away from the null duty

constexpr float TACHO_GAIN = 2.5f;   // (mm/s) per volt (unknown to the fit)
constexpr float TACHO_ZERO = 1.65f;  // standstill output (V)

struct Capture {
    std::vector<float> positions;
    std::vector<float> voltages;
};

Capture stepCapture(uint16_t count, float v0) {
    Capture c;
    c.positions.resize(count);
    c.voltages.resize(count);

    float v_ss = v0 + KM * STEP;
    for (uint16_t i = 0; i < count; i++) {
        float t = (float)i / FS;
        float v = v_ss + (v0 - v_ss) * std::exp(-t / TAU);

        // p(t) = Integral(v dt), closed form for the first-order step.
        c.positions[i] = v_ss * t + (v0 - v_ss) * TAU * (1.f - std::exp(-t / TAU));
        c.voltages[i] = v / TACHO_GAIN + TACHO_ZERO;
    }
    return c;
}

} // namespace

// -----------------------------------------------------------------------
// Identification
// -----------------------------------------------------------------------

TEST(ZAxisAnalyzer, RecoversModelAndTachoCalibration) {
    auto c = stepCapture(128, 0.f);   // 0.512 s ≈ 10*tau

    ZAxisAnalyzer::MotorModel m{};
    ZAxisAnalyzer::TachometerCalibration tacho{};
    ASSERT_TRUE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), c.voltages.data(), 128, FS, STEP, m, tacho));

    EXPECT_NEAR(m.gain, KM, KM * 0.02f);
    EXPECT_NEAR(m.timeConstant, TAU, TAU * 0.05f);
    EXPECT_NEAR(tacho.gain, TACHO_GAIN, TACHO_GAIN * 0.02f);
    EXPECT_NEAR(tacho.zeroVelocityVoltage, TACHO_ZERO, 0.02f);
}

TEST(ZAxisAnalyzer, HandlesNonzeroInitialVelocity) {
    // Same dynamics riding on a 3 mm/s pre-step operating point.
    auto c = stepCapture(128, 3.f);

    ZAxisAnalyzer::MotorModel m{};
    ZAxisAnalyzer::TachometerCalibration tacho{};
    ASSERT_TRUE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), c.voltages.data(), 128, FS, STEP, m, tacho));

    EXPECT_NEAR(m.gain, KM, KM * 0.02f);
    EXPECT_NEAR(m.timeConstant, TAU, TAU * 0.05f);
    EXPECT_NEAR(tacho.gain, TACHO_GAIN, TACHO_GAIN * 0.02f);
    EXPECT_NEAR(tacho.zeroVelocityVoltage, TACHO_ZERO, 0.02f);
}

TEST(ZAxisAnalyzer, RecoversInvertedTachoPolarity) {
    // Tacho wired backwards: voltage falls as the head moves up. The LVDT
    // is ground truth, so the fit should report a negative tacho gain and
    // still recover the motor model.
    auto c = stepCapture(128, 0.f);
    for (auto &u : c.voltages) {
        u = 2.f * TACHO_ZERO - u;   // mirror around the standstill voltage
    }

    ZAxisAnalyzer::MotorModel m{};
    ZAxisAnalyzer::TachometerCalibration tacho{};
    ASSERT_TRUE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), c.voltages.data(), 128, FS, STEP, m, tacho));

    EXPECT_NEAR(m.gain, KM, KM * 0.02f);
    EXPECT_NEAR(tacho.gain, -TACHO_GAIN, TACHO_GAIN * 0.02f);
    EXPECT_NEAR(tacho.zeroVelocityVoltage, TACHO_ZERO, 0.02f);
}

TEST(ZAxisAnalyzer, RejectsUnsettledCapture) {
    // 0.1 s record of a 50 ms time constant — settling not contained.
    auto c = stepCapture(25, 0.f);

    ZAxisAnalyzer::MotorModel m{};
    ZAxisAnalyzer::TachometerCalibration tacho{};
    EXPECT_FALSE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), c.voltages.data(), 25, FS, STEP, m, tacho));
}

TEST(ZAxisAnalyzer, RejectsZeroStepAndBadData) {
    auto c = stepCapture(128, 0.f);
    ZAxisAnalyzer::MotorModel m{};
    ZAxisAnalyzer::TachometerCalibration tacho{};

    EXPECT_FALSE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), c.voltages.data(), 128, FS, 0.f, m, tacho));
    EXPECT_FALSE(ZAxisAnalyzer::fitFirstOrder(
        nullptr, c.voltages.data(), 128, FS, STEP, m, tacho));
    EXPECT_FALSE(ZAxisAnalyzer::fitFirstOrder(
        c.positions.data(), nullptr, 128, FS, STEP, m, tacho));

    // Motionless record: flat position, flat voltage — the regressors are
    // collinear and there is no response to fit.
    std::vector<float> flat_p(128, 0.f);
    std::vector<float> flat_u(128, TACHO_ZERO);
    EXPECT_FALSE(ZAxisAnalyzer::fitFirstOrder(
        flat_p.data(), flat_u.data(), 128, FS, STEP, m, tacho));
}

// -----------------------------------------------------------------------
// Tuning formulas
// -----------------------------------------------------------------------

TEST(ZAxisAnalyzer, VelocityTuningCancelsMotorPole) {
    ZAxisAnalyzer::MotorModel m{KM, TAU};
    const float lambda = 0.02f;

    ZAxisAnalyzer::PidTuning t{};
    ASSERT_TRUE(ZAxisAnalyzer::velocityPidTuning(m, lambda, t));

    EXPECT_FLOAT_EQ(t.integralTc, TAU);                    // zero cancels pole
    EXPECT_FLOAT_EQ(t.gain, TAU / (KM * lambda));
    EXPECT_FLOAT_EQ(t.derivativeTc, 0.f);
}

TEST(ZAxisAnalyzer, PositionTuningIsProportionalOnly) {
    const float lambda = 0.02f;
    const float n = 4.f;

    ZAxisAnalyzer::PidTuning t{};
    ASSERT_TRUE(ZAxisAnalyzer::positionPidTuning(lambda, n, t));

    EXPECT_FLOAT_EQ(t.gain, 1.f / (n * lambda));
    EXPECT_FLOAT_EQ(t.integralTc, 0.f);
    EXPECT_FLOAT_EQ(t.derivativeTc, 0.f);

    // Separation below 1 would put both loops at the same bandwidth.
    EXPECT_FALSE(ZAxisAnalyzer::positionPidTuning(lambda, 0.5f, t));
}

// -----------------------------------------------------------------------
// Closed loop: the real PidController with the computed gains against the
// identified plant must track a velocity step without overshoot.
// -----------------------------------------------------------------------

TEST(ZAxisAnalyzer, ClosedVelocityLoopTracksWithoutOvershoot) {
    ZAxisAnalyzer::MotorModel m{KM, TAU};
    const float lambda = 0.02f;

    ZAxisAnalyzer::PidTuning t{};
    ASSERT_TRUE(ZAxisAnalyzer::velocityPidTuning(m, lambda, t));

    const float dt = 1.f / FS;
    PidController pid(PidController::Config{
        t.gain, t.integralTc, t.derivativeTc, dt, 0.f, -0.5f, 0.5f});
    pid.start();

    float v = 0.f;
    float v_max = 0.f;
    const float setpoint = 1.0f;   // mm/s

    for (int i = 0; i < 250; i++) {   // 1 s = 50*lambda
        float duty = pid.execute(setpoint, v);
        v += dt * (KM * duty - v) / TAU;   // Euler plant update
        if (v > v_max) v_max = v;
    }

    EXPECT_NEAR(v, setpoint, 0.02f);       // integral removes offset
    EXPECT_LT(v_max, setpoint * 1.05f);    // pole cancellation: no overshoot
}
