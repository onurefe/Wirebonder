#include <gtest/gtest.h>
#include "leaky_integrator_controller.hpp"

static LeakyIntegratorController::Config cfg(float ri, float rf, float cf,
                                             float dt, float preamp,
                                             float mn, float mx)
{
    return {ri, rf, cf, mn, mx, dt, preamp};
}

TEST(LeakyIntegratorController, DoesNothingBeforeStart)
{
    LeakyIntegratorController controller(cfg(1000.0f, 1000.0f, 0.1f,
                                            0.1f, 1.0f, -10.0f, 10.0f));

    EXPECT_FLOAT_EQ(controller.execute(1.0f, 0.0f), 0.0f);
}

TEST(LeakyIntegratorController, IntegratesInputCurrentAndLeaksStoredOutput)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 1.0f, -10.0f, 10.0f));
    controller.start();

    const float first = controller.execute(1.0f, 0.0f);
    const float second = controller.execute(1.0f, 0.0f);

    EXPECT_NEAR(first, 0.05f, 1e-5f);
    EXPECT_NEAR(second, 0.0995f, 1e-5f);
}

TEST(LeakyIntegratorController, AppliesPreamplifierGain)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 0.5f, -10.0f, 10.0f));
    controller.start();

    EXPECT_NEAR(controller.execute(1.0f, 0.0f), 0.025f, 1e-5f);
}

TEST(LeakyIntegratorController, SettlesAtPreamplifiedAnalogDcGain)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 0.5f, -10.0f, 10.0f));
    controller.start();

    float output = 0.0f;
    for (int i = 0; i < 2000; i++) {
        output = controller.execute(1.0f, 0.0f);
    }

    EXPECT_NEAR(output, 2.5f, 1e-3f);
}

TEST(LeakyIntegratorController, InvalidComponentsDisableIntegration)
{
    LeakyIntegratorController zeroRi(cfg(0.0f, 10.0f, 1.0f,
                                        0.1f, 1.0f, -10.0f, 10.0f));
    zeroRi.start();
    EXPECT_FLOAT_EQ(zeroRi.execute(1.0f, 0.0f), 0.0f);

    LeakyIntegratorController zeroRf(cfg(2.0f, 0.0f, 1.0f,
                                        0.1f, 1.0f, -10.0f, 10.0f));
    zeroRf.start();
    EXPECT_FLOAT_EQ(zeroRf.execute(1.0f, 0.0f), 0.0f);

    LeakyIntegratorController zeroCf(cfg(2.0f, 10.0f, 0.0f,
                                        0.1f, 1.0f, -10.0f, 10.0f));
    zeroCf.start();
    EXPECT_FLOAT_EQ(zeroCf.execute(1.0f, 0.0f), 0.0f);

    LeakyIntegratorController zeroDt(cfg(2.0f, 10.0f, 1.0f,
                                        0.0f, 1.0f, -10.0f, 10.0f));
    zeroDt.start();
    EXPECT_FLOAT_EQ(zeroDt.execute(1.0f, 0.0f), 0.0f);
}

TEST(LeakyIntegratorController, OutputIsClamped)
{
    LeakyIntegratorController controller(cfg(1.0f, 10.0f, 1.0f,
                                            1.0f, 1.0f, -0.5f, 0.5f));
    controller.start();

    EXPECT_FLOAT_EQ(controller.execute(1.0f, 0.0f), 0.5f);
    EXPECT_FLOAT_EQ(controller.execute(-1.0f, 0.0f), -0.5f);
}

TEST(LeakyIntegratorController, StopAndRestartResetsOutput)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 1.0f, -10.0f, 10.0f));
    controller.start();
    controller.execute(1.0f, 0.0f);
    controller.execute(1.0f, 0.0f);

    controller.stop();
    controller.start();

    EXPECT_NEAR(controller.execute(1.0f, 0.0f), 0.05f, 1e-5f);
}

TEST(LeakyIntegratorController, BypassReturnsSetpointClampedToOutputLimits)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 1.0f, -0.5f, 0.5f));
    controller.start();

    controller.enableBypass();

    EXPECT_FLOAT_EQ(controller.execute(0.25f, 100.0f), 0.25f);
    EXPECT_FLOAT_EQ(controller.execute(2.0f, 100.0f), 0.5f);
    EXPECT_FLOAT_EQ(controller.execute(-2.0f, 100.0f), -0.5f);
}

TEST(LeakyIntegratorController, DisableBypassRestoresClosedLoopCalculation)
{
    LeakyIntegratorController controller(cfg(2.0f, 10.0f, 1.0f,
                                            0.1f, 1.0f, -10.0f, 10.0f));
    controller.start();

    controller.enableBypass();
    EXPECT_FLOAT_EQ(controller.execute(1.0f, 0.0f), 1.0f);

    controller.disableBypass();
    EXPECT_FALSE(controller.isBypassEnabled());
    EXPECT_NEAR(controller.execute(1.0f, 0.0f), 0.05f, 1e-5f);
}
