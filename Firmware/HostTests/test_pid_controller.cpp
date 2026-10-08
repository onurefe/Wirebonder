#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "pid_controller.hpp"

// leakTc defaults to infinity, i.e. no integral decay: expf(-dt/inf) is exactly
// 1.0, so every test below that predicts an integral term arithmetically stays
// exact. LeakyIntegralDecays overrides it. Fields are named rather than
// positional so a future Config member cannot silently shift the rest along.
static PidController::Config cfg(float kp, float ti, float td, float dt,
                                 float ftc, float mn, float mx,
                                 float leakTc = std::numeric_limits<float>::infinity()) {
    PidController::Config config{};
    config.gain         = kp;
    config.integralTc   = ti;
    config.derivativeTc = td;
    config.dt           = dt;
    config.filterTc     = ftc;
    config.leakTc       = leakTc;
    config.outputMin    = mn;
    config.outputMax    = mx;
    return config;
}

TEST(PidController, ZeroErrorProducesZeroOutput) {
    PidController pid(cfg(2.f, 0.f, 0.f, 0.01f, 0.f, -10.f, 10.f));
    pid.start();
    EXPECT_FLOAT_EQ(pid.execute(0.f, 0.f), 0.f);
}

TEST(PidController, ProportionalResponse) {
    // output = Kp * error = 3 * (10-5) = 15
    PidController pid(cfg(3.f, 0.f, 0.f, 0.01f, 0.f, -100.f, 100.f));
    pid.start();
    EXPECT_NEAR(pid.execute(10.f, 5.f), 15.f, 1e-4f);
}

TEST(PidController, IntegralAccumulation) {
    // After 10 steps at error=1, dt=0.1: integral=1.0
    // output = Kp*(e + integral/Ti) = 1*(1 + 1.0/1) = 2
    PidController pid(cfg(1.f, 1.f, 0.f, 0.1f, 0.f, -1000.f, 1000.f));
    pid.start();
    float out = 0.f;
    for (int i = 0; i < 10; i++) out = pid.execute(1.f, 0.f);
    EXPECT_NEAR(out, 2.f, 1e-4f);
}

TEST(PidController, LeakyIntegralDecays) {
    // With leakage the integral is a geometric series rather than an unbounded
    // ramp: each step accumulates error*dt then decays by exp(-dt/leakTc), so
    // it converges to dt*m/(1-m) instead of growing without bound.
    const float dt     = 0.1f;
    const float leakTc = 0.1f;
    const float m      = std::exp(-dt / leakTc);
    const float steady = dt * m / (1.0f - m);

    PidController pid(cfg(1.f, 1.f, 0.f, dt, 0.f, -1000.f, 1000.f, leakTc));
    pid.start();

    float out = 0.f;
    for (int i = 0; i < 200; i++) out = pid.execute(1.f, 0.f);

    // output = Kp*(e + integral/Ti) = 1*(1 + steady/1)
    EXPECT_NEAR(out, 1.f + steady, 1e-4f);

    // The undecayed integral after 200 steps would be 20.0; leakage must hold
    // it far below that.
    EXPECT_LT(steady, 0.1f);
}

TEST(PidController, DerivativeOnStepThenZero) {
    // Step: error 0→5, derivative = (5-0)/0.1 = 50
    // output = 1*(5 + 0 + 1.0*50) = 55
    PidController pid(cfg(1.f, 0.f, 1.f, 0.1f, 0.f, -1000.f, 1000.f));
    pid.start();
    EXPECT_NEAR(pid.execute(5.f, 0.f), 55.f, 1e-3f);
    // Same error next step: derivative = 0, output = 1*5 = 5
    EXPECT_NEAR(pid.execute(5.f, 0.f), 5.f, 1e-3f);
}

TEST(PidController, OutputClampedAtMax) {
    PidController pid(cfg(100.f, 0.f, 0.f, 0.01f, 0.f, -1.f, 1.f));
    pid.start();
    EXPECT_FLOAT_EQ(pid.execute(10.f, 0.f), 1.f);
}

TEST(PidController, OutputClampedAtMin) {
    PidController pid(cfg(100.f, 0.f, 0.f, 0.01f, 0.f, -1.f, 1.f));
    pid.start();
    EXPECT_FLOAT_EQ(pid.execute(-10.f, 0.f), -1.f);
}

TEST(PidController, DoesNothingBeforeStart) {
    PidController pid(cfg(5.f, 0.f, 0.f, 0.01f, 0.f, -100.f, 100.f));
    EXPECT_FLOAT_EQ(pid.execute(10.f, 0.f), 0.f);
}

TEST(PidController, StopAndRestartResetsIntegral) {
    PidController pid(cfg(1.f, 1.f, 0.f, 0.1f, 0.f, -1000.f, 1000.f));
    pid.start();
    for (int i = 0; i < 10; i++) pid.execute(1.f, 0.f);
    pid.stop();
    pid.start();
    // Fresh integral: after 1 step integral=0.1, output = 1*(1 + 0.1/1) = 1.1
    EXPECT_NEAR(pid.execute(1.f, 0.f), 1.1f, 1e-4f);
}

TEST(PidController, BypassReturnsSetpointClampedToOutputLimits) {
    PidController pid(cfg(100.f, 1.f, 1.f, 0.1f, 0.f, -0.5f, 0.5f));
    pid.start();

    pid.enableBypass();

    EXPECT_FLOAT_EQ(pid.execute(0.25f, 100.f), 0.25f);
    EXPECT_FLOAT_EQ(pid.execute(2.0f, 100.f), 0.5f);
    EXPECT_FLOAT_EQ(pid.execute(-2.0f, 100.f), -0.5f);
}

TEST(PidController, BypassIsDisabledByDefault) {
    PidController pid(cfg(2.f, 0.f, 0.f, 0.01f, 0.f, -100.f, 100.f));
    pid.start();

    EXPECT_FALSE(pid.isBypassEnabled());
    EXPECT_FLOAT_EQ(pid.execute(10.f, 5.f), 10.f);
}

TEST(PidController, DisableBypassRestoresClosedLoopCalculation) {
    PidController pid(cfg(3.f, 0.f, 0.f, 0.01f, 0.f, -100.f, 100.f));
    pid.start();

    pid.enableBypass();
    EXPECT_FLOAT_EQ(pid.execute(10.f, 5.f), 10.f);

    pid.disableBypass();
    EXPECT_FALSE(pid.isBypassEnabled());
    EXPECT_FLOAT_EQ(pid.execute(10.f, 5.f), 15.f);
}
