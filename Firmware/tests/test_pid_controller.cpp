#include <gtest/gtest.h>
#include "pid_controller.hpp"

static PidController::Config cfg(float kp, float ti, float td, float dt,
                                  float ftc, float mn, float mx) {
    return {kp, ti, td, dt, ftc, mn, mx};
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
